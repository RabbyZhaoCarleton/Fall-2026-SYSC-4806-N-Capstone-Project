"""fake_world: a lightweight stand-in for Gazebo + SLAM + explore_lite + Nav2.

The full Gazebo simulation (sim_delivery.launch.py) takes minutes: the robot
first has to explore the maze. To test and demonstrate the global planner
quickly, this node fakes everything around it, with the *same topics, frames and
action* the real system uses, so the global planner and the task manager cannot
tell the difference:

    /map, /global_costmap/costmap   nav_msgs/OccupancyGrid   the fully explored maze, from a layout file
                                    (publish_costmap:=false leaves the costmap to a real Nav2)
    /tf                             map -> base_link         a simple kinematic robot
    /explore/status, /explore/exits explore_lite_msgs        "exploration done" and where the exits are
    navigate_to_pose                nav2_msgs/action/NavigateToPose
                                    drives the fake robot along a grid route at
                                    cruise_speed, turning in place at sharp corners
    /fake_world/robot               visualization_msgs/Marker   the robot, for RViz

The robot "teleports" smoothly (no physics, no collisions): this simulator is
for checking the planning logic and the ROS interfaces, not robot dynamics.
Set speed_scale below 1 to make the robot slower than the planner's time model
expects, to see how the estimates compare with "measured" times.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from ..core.grid import LETHAL, UNKNOWN
from ..core.maze_layout import layout_costmap, load_layout
from ..core.search import dijkstra, extract_path, smooth_path
from ..core.time_model import angle_diff
from .conversions import quaternion_from_yaw, yaw_from_quaternion

try:
    from explore_lite_msgs.msg import ExploreStatus, Frontier, FrontierArray
except ImportError:  # pragma: no cover
    ExploreStatus = Frontier = FrontierArray = None

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
DT = 0.05  # s, motion simulation step


class FakeWorld(Node):
    def __init__(self):
        super().__init__('fake_world')
        p = self.declare_parameter
        layout_file = p('layout_file', '').value
        self.x = p('start_x', 0.25).value
        self.y = p('start_y', -0.25).value
        self.yaw = p('start_yaw', 0.0).value
        self.frame = p('global_frame', 'map').value
        self.robot_frame = p('robot_frame', 'base_link').value
        self.cruise_speed = p('cruise_speed', 0.12).value
        self.rotate_speed = p('rotate_speed', 0.8).value
        self.inplace_turn_threshold = p('inplace_turn_threshold', 0.785).value
        self.speed_scale = p('speed_scale', 1.0).value
        self.exploration_time = p('exploration_time', 3.0).value
        # false: leave /global_costmap/costmap to a real Nav2 costmap (built from /map).
        self.publish_costmap = p('publish_costmap', True).value
        if not layout_file:
            raise RuntimeError('layout_file parameter is required')

        self.layout = load_layout(layout_file)
        # Inflation matching maze_navigation/config/nav2_maze.yaml.
        self.grid, values, info = layout_costmap(
            self.layout, resolution=p('resolution', 0.05).value,
            robot_radius=p('robot_radius', 0.11).value,
            inflation_radius=p('inflation_radius', 0.2).value,
            cost_scaling_factor=p('cost_scaling_factor', 5.0).value)
        self.costmap_msg = self._grid_msg(values, info)
        # The SLAM map: walls and free space only, no inflation.
        self.map_msg = self._grid_msg(
            [v if v in (UNKNOWN, LETHAL) else 0 for v in values], info)

        self.lock = threading.Lock()   # the pose is read by the TF timer, written by the action
        group = ReentrantCallbackGroup()
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        if self.publish_costmap:
            self.costmap_pub = self.create_publisher(OccupancyGrid, '/global_costmap/costmap',
                                                     LATCHED)
        self.robot_pub = self.create_publisher(Marker, '/fake_world/robot', 10)
        self.tf = TransformBroadcaster(self)
        self.publish_maps()
        self.create_timer(2.0, self.publish_maps)
        self.create_timer(DT, self.publish_pose)

        self.nav_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self.execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)

        if ExploreStatus is not None:
            self.status_pub = self.create_publisher(ExploreStatus, '/explore/status', LATCHED)
            self.exits_pub = self.create_publisher(FrontierArray, '/explore/exits', LATCHED)
            self.status_pub.publish(ExploreStatus(status=ExploreStatus.EXPLORATION_STARTED))
            self.explore_timer = self.create_timer(self.exploration_time, self.finish_exploring)
        self.get_logger().info(
            f'Fake world: {self.layout.rows} x {self.layout.cols} cell maze with '
            f'{len(self.layout.exits)} exits; robot at ({self.x:.2f}, {self.y:.2f}).')

    # ---------------------------------------------------- world
    def _grid_msg(self, values, info):
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame
        msg.info.resolution = info['resolution']
        msg.info.width = info['width']
        msg.info.height = info['height']
        msg.info.origin.position.x = info['origin_x']
        msg.info.origin.position.y = info['origin_y']
        msg.info.origin.orientation.w = 1.0
        msg.data = [int(v) for v in values]
        return msg

    def publish_maps(self):
        stamp = self.get_clock().now().to_msg()
        self.map_msg.header.stamp = stamp
        self.map_pub.publish(self.map_msg)
        if self.publish_costmap:
            self.costmap_msg.header.stamp = stamp
            self.costmap_pub.publish(self.costmap_msg)

    def finish_exploring(self):
        """Pretend explore_lite has mapped the maze, found the exits and come back."""
        self.explore_timer.cancel()
        msg = FrontierArray()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.layout.exits:
            f = Frontier()
            f.centroid.x, f.centroid.y = x, y
            f.state = Frontier.EXIT
            f.no_return_ratio = -1.0
            msg.frontiers.append(f)
        self.exits_pub.publish(msg)
        self.status_pub.publish(ExploreStatus(status=ExploreStatus.EXPLORATION_COMPLETE))
        self.status_pub.publish(ExploreStatus(status=ExploreStatus.RETURNED_TO_ORIGIN))
        self.get_logger().info('Exploration "finished": exits published.')

    def publish_pose(self):
        with self.lock:
            x, y, yaw = self.x, self.y, self.yaw
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame
        t.child_frame_id = self.robot_frame
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation = quaternion_from_yaw(yaw)
        self.tf.sendTransform(t)

        m = Marker()
        m.header = t.header
        m.ns, m.id, m.type = 'robot', 0, Marker.ARROW
        m.pose.position.x, m.pose.position.y = x, y
        m.pose.orientation = t.transform.rotation
        m.scale.x, m.scale.y, m.scale.z = 0.22, 0.12, 0.08
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.4, 1.0, 1.0
        self.robot_pub.publish(m)

    # ---------------------------------------------------- driving
    def execute(self, goal_handle):
        """NavigateToPose: route to the goal on the grid, then drive along it."""
        target = goal_handle.request.pose.pose
        gx, gy = target.position.x, target.position.y
        gyaw = yaw_from_quaternion(target.orientation)
        with self.lock:
            sx, sy = self.x, self.y
        start = self.grid.nearest_free(sx, sy, 0.3)
        goal = self.grid.nearest_free(gx, gy, 0.3)
        if start is None or goal is None:
            self.get_logger().warn('Goal or robot is off the free space; aborting.')
            goal_handle.abort()
            return NavigateToPose.Result(error_code=1, error_msg='no free cell')
        _, parent = dijkstra(self.grid, start, 2.0, targets=[goal])
        cells = extract_path(parent, start, goal)
        if cells is None:
            goal_handle.abort()
            return NavigateToPose.Result(error_code=2, error_msg='no route')
        points = smooth_path(self.grid, cells, 0.9)
        points[0], points[-1] = (sx, sy), (gx, gy)

        speed = self.cruise_speed * self.speed_scale
        turn_speed = self.rotate_speed * self.speed_scale
        feedback = NavigateToPose.Feedback()
        remaining = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))

        def turn_to(heading):
            # Sharp turns: rotate in place. Gentle ones: just take the new heading.
            with self.lock:
                d = angle_diff(self.yaw, heading)
            if abs(d) < self.inplace_turn_threshold:
                with self.lock:
                    self.yaw = heading
                return True
            for _ in range(int(abs(d) / (turn_speed * DT))):
                if goal_handle.is_cancel_requested:
                    return False
                with self.lock:
                    self.yaw += math.copysign(turn_speed * DT, d)
                time.sleep(DT)
            with self.lock:
                self.yaw = heading
            return True

        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-6:
                continue
            if not turn_to(math.atan2(y1 - y0, x1 - x0)):
                goal_handle.canceled()
                return NavigateToPose.Result()
            steps = max(1, int(seg / (speed * DT)))
            for k in range(1, steps + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return NavigateToPose.Result()
                with self.lock:
                    self.x = x0 + (x1 - x0) * k / steps
                    self.y = y0 + (y1 - y0) * k / steps
                time.sleep(DT)
                remaining -= seg / steps
                feedback.distance_remaining = max(0.0, remaining)
                goal_handle.publish_feedback(feedback)
        # Like Nav2's goal checker (yaw_goal_tolerance 0.25): finish at the goal heading.
        with self.lock:
            need_turn = abs(angle_diff(self.yaw, gyaw)) > 0.25
        if need_turn and not turn_to(gyaw):
            goal_handle.canceled()
            return NavigateToPose.Result()
        goal_handle.succeed()
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    node = FakeWorld()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
