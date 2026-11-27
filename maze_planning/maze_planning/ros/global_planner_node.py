"""global_planner: the ROS 2 node that plans the order of deliveries.

Subscribes
    /global_costmap/costmap   nav_msgs/OccupancyGrid           Nav2's global costmap (walls, inflation, keepout zones)
    /delivery/objects         maze_interfaces/DeliveryObjectArray   objects, their exits and states (from the task manager)
    /delivery/exits           maze_interfaces/ExitBinArray     exit bins found so far (from the task manager)
    /tf                       map -> base_link                 where the robot is

Publishes (all latched: transient local, so late subscribers get the latest)
    /delivery/plan            maze_interfaces/DeliveryPlan     the plan: ordered steps with goal poses and times
    /delivery/plan_path       nav_msgs/Path                    the whole planned route (RViz / GUI)
    /delivery/plan_markers    visualization_msgs/MarkerArray   routes per trip, numbered goals, objects, exits

Service
    /global_planner/get_plan  maze_interfaces/srv/GetDeliveryPlan
        replan: true  -> plan now from the robot's current pose and return it
        replan: false -> return the latest plan

The node plans by itself whenever the objects or exits change (auto_replan), and
whenever the task manager asks through the service, which it does before
starting each trip. Planning itself is done by core/planner.py.
"""

import rclpy
from maze_interfaces.msg import DeliveryObjectArray, DeliveryPlan, ExitBinArray
from maze_interfaces.srv import GetDeliveryPlan
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from ..core.planner import NO_PLAN, GlobalPlanner, PlannerConfig, PlanResult, Pose2D
from ..core.time_model import TimeModel
from .conversions import (costgrid_from_occupancy, exits_from_msg, plan_markers,
                          plan_to_msg, plan_to_path, tasks_from_objects, yaw_from_quaternion)

# Latched topics: a node that subscribes later still receives the last message.
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner')

        # ---------------------------------------------------- parameters
        # Every value can be changed in config/global_planner.yaml.
        p = self.declare_parameter
        self.global_frame = p('global_frame', 'map').value
        self.robot_frame = p('robot_frame', 'base_link').value
        costmap_topic = p('costmap_topic', '/global_costmap/costmap').value
        objects_topic = p('objects_topic', '/delivery/objects').value
        exits_topic = p('exits_topic', '/delivery/exits').value
        plan_topic = p('plan_topic', '/delivery/plan').value
        self.auto_replan = p('auto_replan', True).value
        self.allow_unknown = p('allow_unknown', False).value
        self.blocked_threshold = p('blocked_threshold', 99).value

        tm = TimeModel(
            cruise_speed=p('cruise_speed', 0.12).value,
            rotate_speed=p('rotate_speed', 0.8).value,
            inplace_turn_threshold=p('inplace_turn_threshold', 0.785).value,
            turn_overhead=p('turn_overhead', 0.5).value,
            holonomic=p('holonomic', False).value,
            pick_time=p('pick_time', 20.0).value,
            place_time=p('place_time', 8.0).value)
        self.config = PlannerConfig(
            capacity=p('capacity', 1).value,
            max_exact_objects=p('max_exact_objects', 12).value,
            cost_travel_multiplier=p('cost_travel_multiplier', 2.0).value,
            robot_snap_distance=p('robot_snap_distance', 0.3).value,
            pickup_standoff=p('pickup_standoff', 0.20).value,
            exit_standoff=p('exit_standoff', 0.25).value,
            approach_search_radius=p('approach_search_radius', 0.6).value,
            max_goal_kappa=p('max_goal_kappa', 0.8).value,
            smoothing_max_kappa=p('smoothing_max_kappa', 0.9).value,
            time_model=tm)
        self.planner = GlobalPlanner(self.config)

        # ---------------------------------------------------- inputs
        self.costmap_msg = None
        self.grid = None              # CostGrid made from costmap_msg, built when needed
        self.objects_msg = None
        self.exits_msg = None
        self.dirty = False            # objects or exits changed since the last plan

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 publishes the costmap latched and reliable.
        self.create_subscription(OccupancyGrid, costmap_topic, self.on_costmap, LATCHED)
        self.create_subscription(DeliveryObjectArray, objects_topic, self.on_objects, LATCHED)
        self.create_subscription(ExitBinArray, exits_topic, self.on_exits, LATCHED)

        # ---------------------------------------------------- outputs
        self.plan_pub = self.create_publisher(DeliveryPlan, plan_topic, LATCHED)
        self.path_pub = self.create_publisher(Path, plan_topic + '_path', LATCHED)
        self.marker_pub = self.create_publisher(MarkerArray, plan_topic + '_markers', LATCHED)
        self.create_service(GetDeliveryPlan, '~/get_plan', self.on_get_plan)

        self.plan_id = 0
        self.last_plan = None
        # Auto-replanning is checked once a second, so a burst of updates (all the
        # objects arriving, several exits) gives one plan, not many.
        self.create_timer(1.0, self.on_timer)
        self.get_logger().info(
            f'Global planner ready: capacity {self.config.capacity}, '
            f'objects on {objects_topic}, exits on {exits_topic}, costmap {costmap_topic}.')

    # ---------------------------------------------------- callbacks
    def on_costmap(self, msg):
        self.costmap_msg = msg
        self.grid = None   # rebuilt lazily: converting takes time, plans are rarer than updates

    def on_objects(self, msg):
        self.objects_msg = msg
        self.dirty = True

    def on_exits(self, msg):
        self.exits_msg = msg
        self.dirty = True

    def on_timer(self):
        if self.auto_replan and self.dirty and self.objects_msg is not None:
            self.replan()

    def on_get_plan(self, request, response):
        if request.replan or self.last_plan is None:
            self.replan()
        response.plan = self.last_plan
        return response

    # ---------------------------------------------------- planning
    def robot_pose(self):
        """Robot pose in the global frame from TF, or None."""
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time(),
                                                timeout=Duration(seconds=0.2))
        except TransformException as e:
            self.get_logger().warn(f'No robot pose ({self.global_frame} -> '
                                   f'{self.robot_frame}): {e}', throttle_duration_sec=5.0)
            return None
        tr = t.transform.translation
        return Pose2D(tr.x, tr.y, yaw_from_quaternion(t.transform.rotation))

    def replan(self):
        """Plan from the current inputs and publish the result."""
        self.dirty = False
        result = self._plan()
        self.plan_id += 1
        stamp = self.get_clock().now().to_msg()
        self.last_plan = plan_to_msg(result, self.plan_id, self.global_frame, stamp)
        self.plan_pub.publish(self.last_plan)
        self.path_pub.publish(plan_to_path(result, self.global_frame, stamp))
        objects = tasks_from_objects(self.objects_msg) if self.objects_msg else []
        exits = exits_from_msg(self.exits_msg) if self.exits_msg else []
        self.marker_pub.publish(plan_markers(result, objects, exits, self.global_frame, stamp))

        log = self.get_logger().warn if result.status == NO_PLAN else self.get_logger().info
        log(f'Plan {self.plan_id}: {result.message} (planned in {result.planning_time:.2f} s)')
        return self.last_plan

    def _plan(self):
        if self.objects_msg is None:
            return PlanResult(NO_PLAN, 'No objects received yet.')
        if self.costmap_msg is None:
            return PlanResult(NO_PLAN, 'No costmap received yet.')
        robot = self.robot_pose()
        if robot is None:
            return PlanResult(NO_PLAN, 'The robot pose is not available from TF.')
        if self.grid is None:
            self.grid = costgrid_from_occupancy(self.costmap_msg, self.blocked_threshold,
                                                self.allow_unknown)
        objects = tasks_from_objects(self.objects_msg)
        exits = exits_from_msg(self.exits_msg) if self.exits_msg else []
        try:
            return self.planner.plan(self.grid, robot, objects, exits)
        except Exception as e:  # never let a planning bug kill the node mid-mission
            self.get_logger().error(f'Planning failed: {e!r}')
            return PlanResult(NO_PLAN, f'Planning failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
