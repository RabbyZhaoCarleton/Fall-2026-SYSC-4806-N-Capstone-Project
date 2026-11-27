"""mock_task_manager: a stand-in for the team's task_manager node, for simulation.

The proposal (2.2, 2.3.2) gives the task manager the job of coordinating the
sorting: it gets each object's class and exit from the GCS, sends Nav2 its goals,
and commands each pickup and drop-off. That node is not written yet, so this one
does the same job *with fake perception and a fake arm*, to exercise the global
planner end to end in simulation:

  1. Wait for exploration to finish: explore_lite publishes "returned_to_origin"
     on /explore/status (or start right away, or on the ~/start service).
  2. "Scan" the objects at the entrance: after scan_time, publish them with the
     class and exit from the scenario file (on the robot, the GCS does this).
  3. Name the exits: explore_lite publishes where exits are (/explore/exits) but
     not which bin is which (the GCS reads the bin signs on the robot). Each exit
     is labelled with the nearest bin from the scenario's ground truth.
  4. Loop: ask the global planner for a fresh plan (get_plan, replan=true), do the
     plan's first trip (NavigateToPose for NAVIGATE steps, a pause for PICK /
     PLACE), mark the objects CARRIED / DELIVERED, repeat until nothing is left.
     Planning again before each trip means the plan always starts from where the
     robot really is, and takes in any map changes.
  5. Log every step's estimated vs measured time and a mission summary
     (proposal 2.7: first delivery vs later deliveries).

Publishes (latched)
    /delivery/objects          maze_interfaces/DeliveryObjectArray
    /delivery/exits            maze_interfaces/ExitBinArray
    /delivery/mission_status   std_msgs/String
Subscribes
    /explore/status            explore_lite_msgs/ExploreStatus  (if explore_lite_msgs is installed)
    /explore/exits             explore_lite_msgs/FrontierArray
Uses
    /global_planner/get_plan   maze_interfaces/srv/GetDeliveryPlan (client)
    navigate_to_pose           nav2_msgs/action/NavigateToPose (client)
Offers
    /mock_task_manager/start   std_srvs/Trigger
"""

import math
import threading
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose
from maze_interfaces.msg import (DeliveryObject, DeliveryObjectArray, DeliveryPlan, ExitBin,
                                 ExitBinArray, PlanStep)
from maze_interfaces.srv import GetDeliveryPlan
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:  # only needed when exploring with explore_lite (Gazebo or the lightweight sim)
    from explore_lite_msgs.msg import ExploreStatus, Frontier, FrontierArray
except ImportError:  # pragma: no cover
    ExploreStatus = Frontier = FrontierArray = None

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
ACTION = {PlanStep.NAVIGATE_TO_PICKUP: 'NAVIGATE_TO_PICKUP', PlanStep.PICK: 'PICK',
          PlanStep.NAVIGATE_TO_EXIT: 'NAVIGATE_TO_EXIT', PlanStep.PLACE: 'PLACE'}


def wait_future(future, timeout):
    """Block this (mission) thread until an rclpy future is done. True if it finished."""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    return done.wait(timeout)


class MockTaskManager(Node):
    def __init__(self):
        super().__init__('mock_task_manager')
        p = self.declare_parameter
        self.frame = p('global_frame', 'map').value
        scenario_file = p('scenario_file', '').value
        self.start_on = p('start_on', 'explore_done').value   # explore_done | immediately | service
        self.exits_source = p('exits_source', 'explore').value  # explore | scenario
        self.exit_match_radius = p('exit_match_radius', 0.6).value
        self.scan_time = p('scan_time', 2.0).value
        self.pick_time = p('pick_time', 5.0).value       # simulated arm time (s)
        self.place_time = p('place_time', 3.0).value
        self.nav_retries = p('nav_retries', 1).value
        self.nav_timeout = p('nav_timeout', 180.0).value
        self.exit_wait_timeout = p('exit_wait_timeout', 30.0).value

        if not scenario_file:
            raise RuntimeError('scenario_file parameter is required')
        with open(scenario_file) as f:
            self.scenario = yaml.safe_load(f)

        group = ReentrantCallbackGroup()
        self.objects_pub = self.create_publisher(DeliveryObjectArray, '/delivery/objects', LATCHED)
        self.exits_pub = self.create_publisher(ExitBinArray, '/delivery/exits', LATCHED)
        self.status_pub = self.create_publisher(String, '/delivery/mission_status', LATCHED)
        self.plan_client = self.create_client(GetDeliveryPlan, '/global_planner/get_plan',
                                              callback_group=group)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                       callback_group=group)
        self.create_service(Trigger, '~/start', self.on_start, callback_group=group)

        self.start_event = threading.Event()
        self.exits = {}           # exit id -> (x, y), as labelled so far
        self.objects = []         # DeliveryObject list
        if ExploreStatus is not None:
            self.create_subscription(ExploreStatus, '/explore/status', self.on_explore_status,
                                     LATCHED, callback_group=group)
            if self.exits_source == 'explore':
                self.create_subscription(FrontierArray, '/explore/exits', self.on_explore_exits,
                                         LATCHED, callback_group=group)
        elif self.exits_source == 'explore' or self.start_on == 'explore_done':
            self.get_logger().warn('explore_lite_msgs is not installed: using the scenario '
                                   'exits and starting right away.')
            self.exits_source, self.start_on = 'scenario', 'immediately'

        if self.exits_source == 'scenario':
            for eid, e in self.scenario['exits'].items():
                self.exits[eid] = (float(e['x']), float(e['y']))
            self.publish_exits()
        if self.start_on == 'immediately':
            self.start_event.set()

        self.say(f'Waiting to start ({self.start_on}).')
        self.thread = threading.Thread(target=self.run_mission, daemon=True)
        self.thread.start()

    # ---------------------------------------------------- inputs
    def on_start(self, request, response):
        self.start_event.set()
        response.success, response.message = True, 'Mission started.'
        return response

    def on_explore_status(self, msg):
        if self.start_on == 'explore_done' and msg.status in (
                ExploreStatus.RETURNED_TO_ORIGIN, ExploreStatus.RETURN_FAILED):
            self.get_logger().info(f'Exploration ended ({msg.status}).')
            self.start_event.set()

    def on_explore_exits(self, msg):
        """Label explore_lite's exits with the nearest bin from the scenario.

        On the robot the GCS will read the bin's sign / QR code instead. The
        position always comes from explore_lite (what the robot actually found).
        """
        truth = self.scenario['exits']
        changed = False
        for f in msg.frontiers:
            if f.state != Frontier.EXIT:
                continue
            x, y = f.centroid.x, f.centroid.y
            best, best_d = None, self.exit_match_radius
            for eid, e in truth.items():
                d = math.hypot(x - e['x'], y - e['y'])
                if d < best_d:
                    best, best_d = eid, d
            if best is None:
                self.get_logger().warn(f'Exit at ({x:.2f}, {y:.2f}) matches no bin in the '
                                       'scenario', throttle_duration_sec=10.0)
            elif self.exits.get(best) != (x, y):
                self.exits[best] = (x, y)
                changed = True
                self.get_logger().info(f'Exit {best} at ({x:.2f}, {y:.2f})')
        if changed:
            self.publish_exits()

    # ---------------------------------------------------- outputs
    def say(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def publish_exits(self):
        msg = ExitBinArray()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for eid, (x, y) in sorted(self.exits.items()):
            pose = Pose(position=Point(x=x, y=y))
            pose.orientation.w = 1.0
            msg.exits.append(ExitBin(id=eid, pose=pose))
        self.exits_pub.publish(msg)

    def publish_objects(self):
        msg = DeliveryObjectArray()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.objects = self.objects
        self.objects_pub.publish(msg)

    def set_state(self, object_id, state):
        for o in self.objects:
            if o.id == object_id:
                o.state = state
        self.publish_objects()

    # ---------------------------------------------------- mission
    def request_plan(self):
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('global_planner/get_plan is not available')
            return None
        future = self.plan_client.call_async(GetDeliveryPlan.Request(replan=True))
        if not wait_future(future, 30.0):
            return None
        return future.result().plan

    def navigate(self, pose):
        """Drive to pose with Nav2. Returns True on success."""
        if not self.nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 navigate_to_pose is not available')
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = pose
        send = self.nav_client.send_goal_async(goal)
        if not wait_future(send, 10.0) or not send.result().accepted:
            return False
        result = send.result().get_result_async()
        if not wait_future(result, self.nav_timeout):
            send.result().cancel_goal_async()
            return False
        return result.result().status == GoalStatus.STATUS_SUCCEEDED

    def handle_nav_failure(self, step, failures):
        """Count a failed drive; after nav_retries extra tries, give up on what it was for.

        Giving up marks objects FAILED, so the planner leaves them out and the
        rest of the mission goes on (proposal: signal for human assistance).
        """
        key = step.object_id if step.action == PlanStep.NAVIGATE_TO_PICKUP else step.exit_id
        failures[key] = failures.get(key, 0) + 1
        if failures[key] <= self.nav_retries:
            self.say(f'Navigation to {key} failed ({failures[key]} time(s)); planning again.')
            return
        if step.action == PlanStep.NAVIGATE_TO_PICKUP:
            lost = [o for o in self.objects if o.id == key]
        else:  # the exit cannot be reached: every object still going there fails
            lost = [o for o in self.objects if o.exit_id == key and
                    o.state in (DeliveryObject.WAITING, DeliveryObject.CARRIED)]
        for o in lost:
            o.state = DeliveryObject.FAILED
        self.publish_objects()
        self.say(f'Gave up on {key} after {failures[key]} failed drives: '
                 f'{", ".join(o.id for o in lost)} need human assistance.')

    def run_mission(self):
        self.start_event.wait()
        t0 = time.monotonic()
        self.say('Mission started: scanning the objects at the entrance.')
        time.sleep(self.scan_time)

        classes = self.scenario['classes']
        for o in self.scenario['objects']:
            pose = Pose(position=Point(x=float(o['x']), y=float(o['y'])))
            pose.orientation.w = 1.0
            self.objects.append(DeliveryObject(id=o['id'], object_class=o['class'],
                                               exit_id=classes[o['class']], pose=pose,
                                               state=DeliveryObject.WAITING))
        self.publish_objects()
        self.say(f'Classified {len(self.objects)} objects: ' + ', '.join(
            f'{o.id}={o.object_class}->{o.exit_id}' for o in self.objects))

        # Wait until every exit the objects need has been found and labelled.
        needed = {o.exit_id for o in self.objects}
        deadline = time.monotonic() + self.exit_wait_timeout
        while not needed <= set(self.exits) and time.monotonic() < deadline:
            time.sleep(0.5)
        missing = needed - set(self.exits)
        if missing:
            # Proposal 2.3.1: fewer exits than classes -> signal for human help.
            self.say(f'Exits not found: {", ".join(sorted(missing))}. Objects for them '
                     'are left out of the plan; human assistance needed for those.')

        trip_times, estimates = [], []
        nav_failures = {}
        while rclpy.ok():
            plan = self.request_plan()
            if plan is None:
                self.say('No answer from the global planner; stopping.')
                break
            if plan.status in (DeliveryPlan.NOTHING_TO_DO, DeliveryPlan.NO_PLAN) \
                    or not plan.steps:
                self.say(f'Planner: {plan.message}')
                break
            trip = plan.steps[0].trip
            steps = [s for s in plan.steps if s.trip == trip]
            self.say(f'Plan {plan.plan_id}: {plan.message} Doing its first trip '
                     f'({len(steps)} steps).')
            estimates.append(sum(s.duration for s in steps))

            t_trip = time.monotonic()
            ok = True
            for s in steps:
                t_step = time.monotonic()
                if s.action in (PlanStep.NAVIGATE_TO_PICKUP, PlanStep.NAVIGATE_TO_EXIT):
                    if not self.navigate(s.target):
                        self.handle_nav_failure(s, nav_failures)
                        ok = False
                        break   # plan again from wherever the robot ended up
                elif s.action == PlanStep.PICK:
                    time.sleep(self.pick_time)
                    self.set_state(s.object_id, DeliveryObject.CARRIED)
                elif s.action == PlanStep.PLACE:
                    time.sleep(self.place_time)
                    self.set_state(s.object_id, DeliveryObject.DELIVERED)
                took = time.monotonic() - t_step
                self.get_logger().info(
                    f'  {ACTION[s.action]:<19} {s.object_id or s.exit_id:<8} '
                    f'estimated {s.duration:6.1f} s, took {took:6.1f} s')
            if ok:
                trip_times.append(time.monotonic() - t_trip)
                self.say(f'Trip {len(trip_times)} done in {trip_times[-1]:.1f} s '
                         f'(estimated {estimates[-1]:.1f} s).')

        delivered = sum(o.state == DeliveryObject.DELIVERED for o in self.objects)
        summary = (f'Mission over: {delivered}/{len(self.objects)} delivered in '
                   f'{time.monotonic() - t0:.1f} s.')
        if len(trip_times) > 1:
            later = sum(trip_times[1:]) / (len(trip_times) - 1)
            summary += (f' Trip times: {", ".join(f"{t:.1f}" for t in trip_times)} s '
                        f'(first {trip_times[0]:.1f} s, mean of later ones {later:.1f} s).')
        self.say(summary)


def main(args=None):
    rclpy.init(args=args)
    node = MockTaskManager()
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
