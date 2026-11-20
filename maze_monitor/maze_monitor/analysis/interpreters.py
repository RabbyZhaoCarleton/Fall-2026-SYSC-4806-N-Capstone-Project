"""Interpreters: one per kind of data. See docs/design.md for the reasoning."""

import math
import re
from collections import deque

import numpy as np

from .analyzer import Interpreter, fmt_xy
from .geometry import Pose2D, direction_name, wrap, yaw_from_quaternion

ODOM = 'nav_msgs/msg/Odometry'
TF = 'tf2_msgs/msg/TFMessage'
TWIST = 'geometry_msgs/msg/Twist'
TWIST_STAMPED = 'geometry_msgs/msg/TwistStamped'
SCAN = 'sensor_msgs/msg/LaserScan'
IMU = 'sensor_msgs/msg/Imu'
GRID = 'nav_msgs/msg/OccupancyGrid'
PATH = 'nav_msgs/msg/Path'
STATUS = 'action_msgs/msg/GoalStatusArray'
NAV_FEEDBACK = 'nav2_msgs/action/NavigateToPose_FeedbackMessage'
BT_LOG = 'nav2_msgs/msg/BehaviorTreeLog'
MARKERS = 'visualization_msgs/msg/MarkerArray'
EXPLORE_STATUS = 'explore_lite_msgs/msg/ExploreStatus'
FRONTIER_STATES = 'explore_lite_msgs/msg/FrontierArray'
LOG = 'rcl_interfaces/msg/Log'
CLOCK = 'rosgraph_msgs/msg/Clock'

DEG = math.degrees


def _frame(name):
    return name.lstrip('/')


# ======================================================================== pose
class PoseInterpreter(Interpreter):
    """Robot position and heading in the map frame; localization jumps."""

    types = (ODOM, TF)

    def __init__(self):
        self.odom_pose = None
        self.prev_odom_pose = None
        self.map_odom = None
        self.distance = 0.0
        self.last_stamp = None

    def accepts(self, topic, type_name):
        if type_name == ODOM:
            return topic.rstrip('/').endswith('odom')
        return type_name == TF and topic == '/tf'

    def on_message(self, ctx, t, topic, msg):
        if isinstance_type(msg, 'Odometry'):
            self._on_odom(ctx, t, msg)
        else:
            self._on_tf(ctx, t, msg)

    def _on_odom(self, ctx, t, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None and self.last_stamp - 5.0 < stamp < self.last_stamp - 1e-3:
            return  # older than what we have: a second publisher (see StampInterpreter)
        self.last_stamp = stamp
        p = msg.pose.pose
        self.odom_pose = Pose2D(p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        if self.prev_odom_pose is not None:
            step = self.odom_pose.distance(self.prev_odom_pose)
            if step < 0.5:  # ignore teleports (simulation reset)
                self.distance += step
        self.prev_odom_pose = self.odom_pose
        tw = msg.twist.twist
        ctx.state['odom.v'] = tw.linear.x
        ctx.state['odom.w'] = tw.angular.z
        ctx.state['odom.t'] = t
        ctx.emit(t, 'odom.v', tw.linear.x, 'Actual linear speed (odometry)', 'm/s', min_interval=0.05)
        ctx.emit(t, 'odom.w', tw.angular.z, 'Actual turn rate (odometry)', 'rad/s', min_interval=0.05)
        ctx.emit(t, 'pose.distance', self.distance, 'Distance driven (odometry)', 'm', min_interval=0.2)
        self._publish_pose(ctx, t)

    def _publish_pose(self, ctx, t):
        if self.odom_pose is None:
            return
        frame = 'map' if self.map_odom is not None else 'odom'
        pose = self.map_odom.compose(self.odom_pose) if self.map_odom else self.odom_pose
        ctx.state['pose'] = pose
        ctx.emit(t, 'pose.x', pose.x, 'Position x', 'm', min_interval=0.05)
        ctx.emit(t, 'pose.y', pose.y, 'Position y', 'm', min_interval=0.05)
        ctx.emit(t, 'pose.yaw', pose.yaw, 'Heading', 'rad', min_interval=0.05)
        ctx.emit(t, 'pose.frame', frame, 'Pose frame', numeric=False)

    def _on_tf(self, ctx, t, msg):
        for tr in msg.transforms:
            if _frame(tr.header.frame_id) != 'map' or _frame(tr.child_frame_id) != 'odom':
                continue
            q = tr.transform.rotation
            new = Pose2D(tr.transform.translation.x, tr.transform.translation.y,
                         yaw_from_quaternion(q))
            old = self.map_odom
            self.map_odom = new
            if old is not None and self.odom_pose is not None:
                before = old.compose(self.odom_pose)
                after = new.compose(self.odom_pose)
                jump = before.distance(after)
                turn = abs(DEG(wrap(after.yaw - before.yaw)))
                ctx.emit(t, 'loc.correction', jump, 'SLAM correction step', 'm', min_interval=0.1)
                if jump > ctx.th('localization_jump_m') or turn > ctx.th('localization_jump_deg'):
                    ctx.event(
                        t, 'warning', 'localization',
                        f'Localization jump: {jump * 100:.0f} cm, {turn:.0f}°',
                        'SLAM corrected the map position of the robot at once. The robot was '
                        'not where it believed it was before this moment (or after it). Plans '
                        'made before the jump can lead into walls.',
                        x=after.x, y=after.y)
            self._publish_pose(ctx, t)


def grid_array(msg):
    """OccupancyGrid data as an int8 NumPy array (-1 unknown, 0..100 occupancy)."""
    try:
        return np.frombuffer(msg.data, dtype=np.int8)
    except (TypeError, ValueError):
        return np.asarray(msg.data, dtype=np.int8)


def isinstance_type(msg, name):
    return type(msg).__name__ == name


# ======================================================================== motion
class MotionInterpreter(Interpreter):
    """Commanded versus actual motion: blocked, stuck, spinning, oscillating."""

    types = (TWIST, TWIST_STAMPED, ODOM)

    def __init__(self):
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_t = None
        self.blocked_since = None
        self.turn_blocked_since = None
        self.history = deque()  # (t, x, y, yaw)
        self.turned = deque()  # (t, |dyaw|)
        self.last_yaw = None
        self.flips = deque()
        self.last_sign = 0
        self.nav_since = None

    def accepts(self, topic, type_name):
        if type_name in (TWIST, TWIST_STAMPED):
            return topic == '/cmd_vel'
        return type_name == ODOM and topic.rstrip('/').endswith('odom')

    def on_message(self, ctx, t, topic, msg):
        if type(msg).__name__ in ('Twist', 'TwistStamped'):
            tw = msg.twist if hasattr(msg, 'twist') else msg
            self.cmd_v, self.cmd_w = tw.linear.x, tw.angular.z
            self.last_cmd_t = t
            ctx.state['cmd.v'], ctx.state['cmd.w'], ctx.state['cmd.t'] = self.cmd_v, self.cmd_w, t
            ctx.emit(t, 'cmd.v', self.cmd_v, 'Commanded linear speed', 'm/s', min_interval=0.05)
            ctx.emit(t, 'cmd.w', self.cmd_w, 'Commanded turn rate', 'rad/s', min_interval=0.05)
            self._oscillation(ctx, t)
            return
        self._blocked(ctx, t)

    # -------------------------------------------------------- detectors
    def _blocked(self, ctx, t):
        v, w = ctx.state.get('odom.v', 0.0), ctx.state.get('odom.w', 0.0)
        min_cmd, ratio, hold = ctx.th('blocked_min_cmd'), ctx.th('blocked_ratio'), ctx.th('blocked_time')
        lin_blocked = abs(self.cmd_v) >= min_cmd and (v * math.copysign(1, self.cmd_v)) < ratio * abs(self.cmd_v)
        if lin_blocked:
            self.blocked_since = self.blocked_since if self.blocked_since is not None else t
        else:
            self.blocked_since = None
        active = self.blocked_since is not None and t - self.blocked_since >= hold
        if active:
            ahead = 'clear.front' if self.cmd_v > 0 else 'clear.back'
            clearance = ctx.state.get(ahead)
            direction = 'forward' if self.cmd_v > 0 else 'backward'
            close = clearance is not None and clearance < 2 * ctx.th('near_wall_clearance')
            detail = (f'Commanded {abs(self.cmd_v):.2f} m/s {direction}, measured {v:.2f} m/s '
                      f'for {t - self.blocked_since:.1f} s.')
            if clearance is not None:
                detail += f' Clearance {direction}: {clearance:.2f} m.'
            if close:
                ctx.condition(t, 'blocked', True, 'error', 'collision',
                              'Likely collision: pushing into an obstacle',
                              detail + ' The robot is driving into something it is touching.')
            else:
                ctx.condition(t, 'blocked', True, 'warning', 'motion',
                              'Not following the speed command',
                              detail + ' Possible causes: an obstacle the lidar cannot see, '
                              'wheel slip, or a slow simulation.')
        else:
            ctx.condition(t, 'blocked', False)

        turn_blocked = (abs(self.cmd_w) >= 0.2 and abs(self.cmd_v) < min_cmd
                        and (w * math.copysign(1, self.cmd_w)) < ratio * abs(self.cmd_w))
        if turn_blocked:
            self.turn_blocked_since = self.turn_blocked_since if self.turn_blocked_since is not None else t
        else:
            self.turn_blocked_since = None
        if self.turn_blocked_since is not None and t - self.turn_blocked_since >= hold:
            ctx.condition(t, 'turn_blocked', True, 'warning', 'motion',
                          'Not turning as commanded',
                          f'Commanded a turn of {DEG(self.cmd_w):.0f}°/s in place, measured '
                          f'{DEG(w):.0f}°/s. The robot may be wedged against a wall.')
        else:
            ctx.condition(t, 'turn_blocked', False)

    def _oscillation(self, ctx, t):
        sign = 0 if abs(self.cmd_w) < 0.1 else (1 if self.cmd_w > 0 else -1)
        if sign and self.last_sign and sign != self.last_sign:
            self.flips.append(t)
        if sign:
            self.last_sign = sign
        window = ctx.th('oscillation_window')
        while self.flips and t - self.flips[0] > window:
            self.flips.popleft()
        n = len(self.flips)
        ctx.condition(t, 'oscillation', n >= ctx.th('oscillation_flips'), 'warning', 'motion',
                      'Oscillating: steering keeps flipping',
                      f'The turn command changed direction {n} times in {window:.0f} s. The '
                      'controller cannot settle on a direction, often because the path runs '
                      'close to a wall or the goal is unreachable.', hold=1.0)

    def tick(self, ctx, t):
        pose = ctx.pose
        if pose is None:
            return
        self.history.append((t, pose.x, pose.y))
        window = ctx.th('stuck_window')
        while self.history and t - self.history[0][0] > window:
            self.history.popleft()

        if self.last_yaw is not None:
            self.turned.append((t, abs(wrap(pose.yaw - self.last_yaw)), pose.x, pose.y))
        self.last_yaw = pose.yaw
        spin_window = ctx.th('spin_window')
        while self.turned and t - self.turned[0][0] > spin_window:
            self.turned.popleft()

        navigating = ctx.state.get('nav.active') or (
            self.last_cmd_t is not None and t - self.last_cmd_t < 2.0
            and (abs(self.cmd_v) > 0.01 or abs(self.cmd_w) > 0.05))

        if navigating:
            self.nav_since = self.nav_since if self.nav_since is not None else t
        else:
            self.nav_since = None
        full = self.history and t - self.history[0][0] >= window - 0.3
        if full and navigating and t - self.nav_since >= window:
            x0, y0 = self.history[0][1], self.history[0][2]
            moved = max(math.hypot(x - x0, y - y0) for _, x, y in self.history)
            stuck = moved < ctx.th('stuck_distance')
            ctx.condition(t, 'stuck', stuck, 'warning', 'stuck',
                          f'Stuck: moved {moved * 100:.1f} cm in {window:.0f} s',
                          f'The robot moved only {moved:.2f} m in the last {window:.0f} s although '
                          'it was navigating. Look at the Surroundings and Navigation cards for '
                          'the reason (wall contact, recoveries, no valid path).', hold=1.0)
        else:
            ctx.condition(t, 'stuck', False, hold=1.0)

        if len(self.turned) > 4:
            total = DEG(sum(d for _, d, _, _ in self.turned))
            xs = [x for _, _, x, _ in self.turned]
            ys = [y for _, _, _, y in self.turned]
            spread = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            spinning = total >= ctx.th('spin_angle_deg') and spread < ctx.th('spin_distance')
            ctx.condition(t, 'spinning', spinning, 'warning', 'motion',
                          'Spinning in place',
                          f'Turned {total:.0f}° in {spin_window:.0f} s while moving only '
                          f'{spread:.2f} m. Nav2 runs a spin recovery when it is stuck, and '
                          'explore_lite turns to look around; long spinning means neither helps.',
                          hold=1.0)


# ======================================================================== clearance
class ClearanceInterpreter(Interpreter):
    """Distances to walls around the robot, from the lidar."""

    types = (SCAN, ODOM)

    # Sector centers and half-widths in degrees, robot frame (0 = ahead, + = left).
    SECTORS = {'front': (0, 45), 'left': (90, 45), 'back': (180, 45), 'right': (-90, 45)}

    def __init__(self):
        self.prev_ranges = None
        self.ahead = deque()  # (t, straight-ahead range, odom distance)
        self.slip_since = None

    def accepts(self, topic, type_name):
        if type_name == SCAN:
            return True
        return type_name == ODOM and topic.rstrip('/').endswith('odom')

    def on_message(self, ctx, t, topic, msg):
        if type(msg).__name__ == 'Odometry':
            return  # slip check runs on scans; odometry already updates ctx.state
        r = np.asarray(msg.ranges, dtype=np.float64)
        n = len(r)
        if n == 0:
            return
        ang = msg.angle_min + np.arange(n) * msg.angle_increment
        ang = (ang + math.pi) % (2 * math.pi) - math.pi
        rmin, rmax = msg.range_min, msg.range_max
        valid = np.isfinite(r) & (r >= rmin) & (r <= rmax)
        too_close = (np.isneginf(r)) | (np.isfinite(r) & (r < rmin))
        # Beams that saw a wall at the edge of the measurable range and now see
        # nothing: the wall came closer than the lidar can measure.
        vanished = np.zeros(n, dtype=bool)
        if self.prev_ranges is not None and len(self.prev_ranges) == n:
            prev = self.prev_ranges
            vanished = (np.isposinf(r) | np.isnan(r)) & np.isfinite(prev) & (prev < rmin + 0.04)
        self.prev_ranges = r
        blind = too_close | vanished
        radius = ctx.config.robot_radius

        def sector_min(center_deg, half_deg):
            d = np.abs((np.degrees(ang) - center_deg + 180) % 360 - 180)
            m = d <= half_deg
            if np.count_nonzero(blind & m) >= 3:
                return 0.0
            sel = r[m & valid]
            return float(sel.min()) - radius if sel.size else None

        clear = {}
        for name, (c, h) in self.SECTORS.items():
            clear[name] = sector_min(c, h)
            ctx.state[f'clear.{name}'] = clear[name]
            if clear[name] is not None:
                ctx.emit(t, f'clear.{name}', clear[name], f'Clearance {name}', 'm')
        if valid.any():
            i = int(np.argmin(np.where(valid, r, np.inf)))
            nearest, bearing = float(r[i]) - radius, float(ang[i])
        else:
            nearest, bearing = None, None
        if np.count_nonzero(blind) >= 3:
            nearest, bearing = 0.0, float(np.mean(ang[blind]))
        ctx.state['clear.min'], ctx.state['clear.bearing'] = nearest, bearing
        ctx.emit(t, 'clear.min', nearest, 'Nearest wall clearance', 'm')
        ctx.emit(t, 'clear.bearing', bearing, 'Bearing of nearest wall', 'rad')
        ctx.emit(t, 'scan.valid_pct', 100.0 * np.count_nonzero(valid) / n, 'Valid lidar readings', '%')
        ctx.emit(t, 'scan.blind', float(np.count_nonzero(blind)),
                 'Readings closer than the lidar minimum', '')

        near_th, contact_th = ctx.th('near_wall_clearance'), ctx.th('contact_clearance')
        if nearest is not None:
            where = f'{direction_name(bearing)} ({DEG(bearing):.0f}°)'
            if nearest <= contact_th:
                ctx.condition(t, 'contact', True, 'error', 'collision', 'Touching a wall', hold=1.5,
                              detail=
                              f'Clearance {max(nearest, 0) * 100:.1f} cm, {where}. '
                              + ('The lidar cannot measure this close; readings below its '
                                 f'{rmin:.2f} m minimum count as contact.'
                                 if np.count_nonzero(blind) >= 3 else ''))
            else:
                ctx.condition(t, 'contact', False, hold=1.5)
            near = nearest < near_th
            ev = ctx.condition(t, 'near_wall', near, 'warning', 'proximity',
                               f'Close to a wall: {max(nearest, 0) * 100:.1f} cm, {where}',
                               f'The nearest wall is {max(nearest, 0):.3f} m from the robot edge, '
                               f'{where}. Nav2 keeps a safety margin of the inflation radius; '
                               'driving this close means the plan or the map is off.', hold=1.5)
            if ev is not None:
                worst = min(ev.extra.get('worst', 1e9), nearest)
                ev.extra['worst'] = worst

        self._slip(ctx, t, r, ang, valid)

    def _slip(self, ctx, t, r, ang, valid):
        cone = valid & (np.abs(ang) <= math.radians(10))
        v = ctx.state.get('odom.v', 0.0)
        w = ctx.state.get('odom.w', 0.0)
        dist = ctx.timeline.value('pose.distance', t, 0.0)
        if not cone.any() or v < 0.08 or abs(w) > 0.3:
            self.ahead.clear()
            self.slip_since = None
            ctx.condition(t, 'slip', False)
            return
        front = float(r[cone].min())
        if front > 1.0:
            self.ahead.clear()
            ctx.condition(t, 'slip', False)
            return
        self.ahead.append((t, front, dist))
        window = ctx.th('slip_time')
        while self.ahead and t - self.ahead[0][0] > window:
            self.ahead.popleft()
        if t - self.ahead[0][0] < window * 0.8:
            return
        expected = dist - self.ahead[0][2]
        actual = self.ahead[0][1] - front
        slipping = expected > 0.08 and actual < 0.25 * expected
        ctx.condition(t, 'slip', slipping, 'warning', 'collision',
                      'Wheels turn but the robot does not move',
                      f'Odometry reports {expected:.2f} m forward in {window:.1f} s, but the '
                      f'wall ahead came only {max(actual, 0):.2f} m closer. The wheels are '
                      'probably slipping against an obstacle, so odometry is wrong.', hold=1.0)


# ======================================================================== impact
class ImpactInterpreter(Interpreter):
    types = (IMU,)

    def __init__(self):
        self.first_t = None
        self.bucket_t = None
        self.bucket_max = 0.0
        self.last_event_t = -1e9

    def on_message(self, ctx, t, topic, msg):
        if self.first_t is None:
            self.first_t = t
        a = math.hypot(msg.linear_acceleration.x, msg.linear_acceleration.y)
        if self.bucket_t is None or t - self.bucket_t >= 0.05:
            if self.bucket_t is not None:
                ctx.emit(self.bucket_t, 'imu.accel', self.bucket_max,
                         'Horizontal acceleration (IMU)', 'm/s²')
            self.bucket_t, self.bucket_max = t, a
        else:
            self.bucket_max = max(self.bucket_max, a)
        if a > ctx.th('impact_accel') and t - self.first_t > 3.0 and t - self.last_event_t > 1.0:
            self.last_event_t = t
            # A hit needs a wall right where the robot was heading, or one it touches.
            v = ctx.state.get('odom.v', 0.0)
            ahead = ctx.state.get('clear.front' if v >= 0 else 'clear.back')
            nearest = ctx.state.get('clear.min')
            hit_ahead = ahead is not None and abs(v) > 0.02 and ahead < ctx.th('near_wall_clearance')
            touching = nearest is not None and nearest <= 2 * ctx.th('contact_clearance')
            if hit_ahead or touching:
                clearance = ahead if hit_ahead else nearest
                ctx.event(t, 'warning', 'collision', f'Bump against a wall: {a:.1f} m/s²',
                          f'The IMU measured a sudden horizontal acceleration of {a:.1f} m/s² '
                          f'while a wall was {max(clearance, 0) * 100:.1f} cm away'
                          + (' in the direction of travel' if hit_ahead else '') +
                          '. The robot probably hit it.', merge_key='bump', merge_window=2.0)
            else:
                ctx.event(t, 'info', 'motion', f'Jolt: {a:.1f} m/s²',
                          f'The IMU measured a sudden horizontal acceleration of {a:.1f} m/s², '
                          'but no wall was near, so the robot stopped or started abruptly rather '
                          'than hitting something. Abrupt stops come from emergency stops (for '
                          "example, Nav2's collision monitor) or from speed commands that skip "
                          'the acceleration limits.', merge_key='jolt', merge_window=3.0)


# ======================================================================== navigation
GOAL_STATES = {0: 'unknown', 1: 'accepted', 2: 'active', 3: 'canceling',
               4: 'succeeded', 5: 'canceled', 6: 'aborted'}
MAIN_ACTIONS = ('/navigate_to_pose', '/navigate_through_poses')
RECOVERY_NODES = {'Spin': 'spin in place', 'BackUp': 'back up', 'Wait': 'wait',
                  'DriveOnHeading': 'drive straight', 'AssistedTeleop': 'assisted teleop'}


class NavigationInterpreter(Interpreter):
    types = (STATUS, NAV_FEEDBACK, PATH, BT_LOG)

    def __init__(self):
        self.goals = {}  # goal id -> (status, start time)
        self.succeeded = 0
        self.failed = 0
        self.recoveries = 0
        self.route_changes = 0
        self.plan_failures = 0
        self.last_path_len = None
        self.last_goal = None
        self.quick = deque()  # times of goals that succeeded within a second

    def accepts(self, topic, type_name):
        if type_name == STATUS:
            return any(topic == a + '/_action/status' for a in MAIN_ACTIONS)
        if type_name == PATH:
            return topic == '/plan'
        return type_name in (NAV_FEEDBACK, BT_LOG)

    def on_message(self, ctx, t, topic, msg):
        name = type(msg).__name__
        if name == 'GoalStatusArray':
            self._status(ctx, t, msg)
        elif name == 'NavigateToPose_FeedbackMessage':
            fb = msg.feedback
            ctx.emit(t, 'nav.distance_remaining', fb.distance_remaining, 'Distance to goal', 'm',
                     min_interval=0.1)
            ctx.emit(t, 'nav.goal_recoveries', float(fb.number_of_recoveries),
                     'Recoveries for this goal', '')
        elif name == 'Path':
            self._path(ctx, t, msg)
        elif name == 'BehaviorTreeLog':
            self._bt(ctx, t, msg)

    def _status(self, ctx, t, msg):
        for st in msg.status_list:
            gid = bytes(st.goal_info.goal_id.uuid).hex()
            state = GOAL_STATES.get(st.status, 'unknown')
            old = self.goals.get(gid)
            if old is not None and old[0] == state:
                continue
            start = old[1] if old else t
            self.goals[gid] = (state, start)
            if old is None and state in ('succeeded', 'aborted', 'canceled'):
                continue  # finished before the monitor saw it start
            took = t - start
            if state in ('accepted', 'active') and (old is None):
                ctx.state['nav.active'] = True
                ctx.emit(t, 'nav.goal_start', t, 'Goal start time', 's', force=True)
                ctx.event(t, 'info', 'navigation', 'New goal',
                          'Nav2 accepted a new navigation goal.', merge_key='new_goal',
                          merge_window=2.0)
            elif state == 'succeeded':
                self.succeeded += 1
                ctx.event(t, 'info', 'navigation', f'Goal reached in {took:.1f} s',
                          merge_key='reached_quick' if took < 1.0 else None, merge_window=5.0)
                if took < 1.0:
                    self.quick.append(t)
                while self.quick and t - self.quick[0] > 30.0:
                    self.quick.popleft()
                if len(self.quick) >= 5:
                    ctx.event(t, 'warning', 'navigation', 'Goals succeed without moving',
                              f'{len(self.quick)} goals in the last 30 s succeeded within a '
                              'second of being sent. The goals are probably already within '
                              'the goal tolerance of the robot (xy_goal_tolerance in the Nav2 '
                              'parameters), so Nav2 reports success at once, and the sender '
                              '(for example, explore_lite) keeps sending them.',
                              merge_key='goal_churn', merge_window=30.0)
            elif state == 'aborted':
                self.failed += 1
                ctx.event(t, 'error', 'navigation', f'Goal failed after {took:.1f} s',
                          'Nav2 aborted the goal: it could not plan a path or could not follow '
                          'it, even after its recoveries. Look at the events just before this '
                          'one for the cause.')
            elif state == 'canceled':
                ctx.event(t, 'info', 'navigation', f'Goal canceled after {took:.1f} s',
                          'The goal was canceled, usually because a newer goal replaced it.')
            ctx.emit(t, 'nav.state', state, 'Goal state', numeric=False)
        active = any(s in ('accepted', 'active', 'canceling') for s, _ in self.goals.values())
        ctx.state['nav.active'] = active
        if not active and self.goals:
            last = max(self.goals.values(), key=lambda v: v[1])[0]
            ctx.emit(t, 'nav.state', last, 'Goal state', numeric=False)
        elif active:
            ctx.emit(t, 'nav.state', 'active', 'Goal state', numeric=False)
        ctx.emit(t, 'nav.active', 1.0 if active else 0.0, 'Goal active', '')
        ctx.emit(t, 'nav.goals_succeeded', float(self.succeeded), 'Goals reached', '')
        ctx.emit(t, 'nav.goals_failed', float(self.failed), 'Goals failed', '')

    def _path(self, ctx, t, msg):
        poses = msg.poses
        if not poses:
            ctx.event(t, 'warning', 'planning', 'Planner returned an empty path',
                      'The global planner found no path to the goal. The goal may be inside '
                      'a wall, in unknown space, or cut off by inflated obstacles.',
                      merge_key='empty_path', merge_window=5.0)
            ctx.emit(t, 'nav.path_length', 0.0, 'Planned path length', 'm')
            return
        pts = [(p.pose.position.x, p.pose.position.y) for p in poses]
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        goal = pts[-1]
        ctx.emit(t, 'nav.path_length', length, 'Planned path length', 'm')
        ctx.emit(t, 'nav.goal_x', goal[0], 'Goal x', 'm')
        ctx.emit(t, 'nav.goal_y', goal[1], 'Goal y', 'm')
        same_goal = self.last_goal is not None and math.hypot(
            goal[0] - self.last_goal[0], goal[1] - self.last_goal[1]) < 0.1
        if same_goal and self.last_path_len is not None:
            if length > self.last_path_len + 0.5 and length > 1.3 * self.last_path_len:
                self.route_changes += 1
                ctx.event(t, 'info', 'planning',
                          f'Route got longer: {self.last_path_len:.1f} m → {length:.1f} m',
                          'The planner switched to a longer route to the same goal. The map '
                          'probably revealed that the shorter way is blocked.',
                          merge_key='route_longer', merge_window=3.0)
        self.last_goal, self.last_path_len = goal, length
        ctx.emit(t, 'nav.route_changes', float(self.route_changes), 'Route changes', '')

    def _bt(self, ctx, t, msg):
        for ch in msg.event_log:
            node, cur, prev = ch.node_name, ch.current_status, ch.previous_status
            base = node.split('-')[0]
            if base in RECOVERY_NODES and cur == 'RUNNING' and prev != 'RUNNING':
                self.recoveries += 1
                ctx.event(t, 'warning', 'recovery', f'Recovery: {RECOVERY_NODES[base]}',
                          f'Nav2 started the "{node}" recovery behavior because navigation '
                          'failed. Recoveries mean the robot was stuck or had no valid path.')
            elif base.startswith('Clear') and 'Costmap' in base and cur == 'SUCCESS':
                which = 'local' if 'Local' in base else 'global' if 'Global' in base else 'all'
                ctx.event(t, 'info', 'recovery', f'Recovery: cleared {which} costmap',
                          'Nav2 cleared obstacles from a costmap to retry planning.',
                          merge_key=f'clear_{which}', merge_window=2.0)
            elif node == 'ComputePathToPose' and cur == 'FAILURE':
                self.plan_failures += 1
                ctx.event(t, 'warning', 'planning', 'Planner failed to find a path',
                          'The global planner could not compute a path to the goal.',
                          merge_key='plan_fail', merge_window=3.0)
            elif node == 'FollowPath' and cur == 'FAILURE':
                ctx.event(t, 'warning', 'planning', 'Controller failed to follow the path',
                          'The local controller could not find safe velocities along the path, '
                          'usually because an obstacle blocks it or the robot is too close to '
                          'a wall.', merge_key='follow_fail', merge_window=3.0)
        ctx.emit(t, 'nav.recoveries', float(self.recoveries), 'Recoveries', '')
        ctx.emit(t, 'nav.plan_failures', float(self.plan_failures), 'Planner failures', '')


# ======================================================================== exploration
# explore_lite (capstone patches 0002-0004) gives each frontier a state. It
# publishes them on /explore/frontier_states (patch 0004) and as marker colours on
# /explore/frontiers: blue open, yellow exit candidate, orange postponed, purple
# maze exit, red abandoned. Runs recorded before patch 0004 only have the colours.
FRONTIER_KINDS = {
    'abandoned': ('explore.blacklisted', 'Abandoned frontiers', 'warning',
                  'Exploration gave up on a frontier',
                  'explore_lite failed to reach this frontier max_frontier_attempts times '
                  '(Nav2 failures, or no progress within progress_timeout / commit_timeout), '
                  'so it will not try it again.'),
    'postponed': ('explore.postponed', 'Postponed frontiers', 'info',
                  'Exploration postponed a frontier',
                  'explore_lite could not reach this frontier for now (Nav2 failure, a timeout, '
                  'or a wall cutting it off). It will retry it once no other frontier is left.'),
    'candidate': ('explore.exit_candidates', 'Exit candidates', 'info',
                  'Exploration spotted a probable maze exit',
                  'The lidar looked through most of this frontier and got no return: nothing '
                  'lies behind it within lidar range. explore_lite explores the open frontiers '
                  'first and confirms it from another position.'),
    'exit': ('explore.exits', 'Maze exits', 'info',
             'Exploration found a maze exit',
             'This frontier opens to the outside: the lidar saw no return through it from two '
             'positions, or the robot reached it and it stayed unexplored. explore_lite will '
             'not drive to it again.'),
}
# explore_lite_msgs/msg/Frontier state values
STATE_KINDS = {0: 'open', 1: 'candidate', 2: 'exit', 3: 'postponed', 4: 'abandoned'}


def frontier_kind(color):
    """Classify a frontier marker colour from explore_lite, or None for open frontiers."""
    if color.b > 0.8 and color.r > 0.4:
        return 'exit'
    if color.r > 0.8 and color.b < 0.3:
        if color.g > 0.75:
            return 'candidate'
        return 'postponed' if color.g > 0.3 else 'abandoned'
    return None


def exit_detail(frontier):
    """How explore_lite decided that a frontier (explore_lite_msgs/Frontier) is an exit."""
    if frontier.exit_views >= 2:
        return (f'The lidar saw no return through this frontier from {frontier.exit_views} '
                'positions: nothing lies behind it within lidar range, so it opens to the '
                'outside. explore_lite will not drive to it again.')
    return ('The robot reached this frontier but it stayed unexplored, so it opens to the '
            'outside. explore_lite will not drive to it again.')


class ExplorationInterpreter(Interpreter):
    types = (GRID, MARKERS, EXPLORE_STATUS, FRONTIER_STATES)

    def __init__(self):
        self.last_known = 0.0
        self.last_growth_t = None
        # frontier positions already reported, per state
        self.reported = {kind: [] for kind in FRONTIER_KINDS}
        self.status = None
        self.typed = False  # /explore/frontier_states seen: ignore the marker colours

    def accepts(self, topic, type_name):
        if type_name == GRID:
            return topic == '/map'
        if type_name == MARKERS:
            return topic.endswith('frontiers')
        if type_name == FRONTIER_STATES:
            return topic.endswith('frontier_states') or topic.endswith('exits')
        return type_name == EXPLORE_STATUS

    def _report(self, ctx, t, found):
        """Emit counts per state and an event for each newly seen frontier in that state.

        found: kind -> list of (x, y, detail or None)
        """
        for kind, (key, label, severity, title, detail) in FRONTIER_KINDS.items():
            ctx.emit(t, key, float(len(found[kind])), label, '')
            for x, y, own_detail in found[kind]:
                # a frontier drifts as the map grows: one event per place
                # (explore_lite's frontier_match_radius)
                seen = self.reported[kind]
                if x is None:
                    if None in seen:
                        continue
                    seen.append(None)
                else:
                    if any(p is not None and math.hypot(x - p[0], y - p[1]) < 0.3 for p in seen):
                        continue
                    seen.append((x, y))
                ctx.event(t, severity, 'exploration', f'{title} at {fmt_xy(x, y)}',
                          own_detail or detail, x=x, y=y)

    def on_message(self, ctx, t, topic, msg):
        name = type(msg).__name__
        if name == 'OccupancyGrid':
            data = grid_array(msg)
            cell = msg.info.resolution ** 2
            known = float(np.count_nonzero(data != -1)) * cell
            occupied = float(np.count_nonzero(data >= 65)) * cell
            ctx.emit(t, 'map.known_area', known, 'Known map area', 'm²')
            ctx.emit(t, 'map.occupied_area', occupied, 'Occupied map area', 'm²')
            ctx.emit(t, 'map.width', msg.info.width * msg.info.resolution, 'Map width', 'm')
            ctx.emit(t, 'map.height', msg.info.height * msg.info.resolution, 'Map height', 'm')
            if known > self.last_known + 0.02 or self.last_growth_t is None:
                self.last_growth_t = t
                self.last_known = known
        elif name == 'FrontierArray':
            if topic.endswith('exits'):
                text = '; '.join(f'({f.centroid.x:.2f}, {f.centroid.y:.2f})'
                                 for f in msg.frontiers)
                ctx.emit(t, 'explore.exit_list', text or '—', 'Exit positions', numeric=False)
                return
            self.typed = True
            ctx.emit(t, 'explore.frontiers', float(len(msg.frontiers)), 'Frontiers', '')
            found = {kind: [] for kind in FRONTIER_KINDS}
            for f in msg.frontiers:
                kind = STATE_KINDS.get(f.state)
                if kind in found:
                    detail = exit_detail(f) if kind == 'exit' else None
                    found[kind].append((f.centroid.x, f.centroid.y, detail))
            self._report(ctx, t, found)
        elif name == 'MarkerArray':
            if self.typed:
                return
            spheres = [m for m in msg.markers if m.action == 0 and m.type == 2]
            centroids = {m.id: (m.pose.position.x, m.pose.position.y) for m in spheres}
            ctx.emit(t, 'explore.frontiers', float(len(spheres)), 'Frontiers', '')
            # explore_lite colours each frontier's points by its state; the matching
            # centroid sphere has the next id.
            found = {kind: [] for kind in FRONTIER_KINDS}
            for m in msg.markers:
                if m.action == 0 and m.type == 8:
                    kind = frontier_kind(m.color)
                    if kind:
                        found[kind].append(centroids.get(m.id + 1, (None, None)) + (None,))
            self._report(ctx, t, found)
        elif name == 'ExploreStatus':
            if msg.status == self.status:
                return
            self.status = msg.status
            ctx.emit(t, 'explore.status', msg.status, 'Exploration status', numeric=False)
            text = msg.status.replace('_', ' ')
            sev = 'info'
            detail = ''
            if msg.status == 'exploration_complete':
                detail = 'No reachable frontiers are left.'
            ctx.event(t, sev, 'exploration', f'Exploration: {text}', detail)

    def tick(self, ctx, t):
        exploring = self.status in ('exploration_started', 'exploration_in_progress') or (
            self.status is None and ctx.timeline.value('explore.frontiers', t, 0) > 0)
        stall = ctx.th('exploration_stall')
        stalled = (exploring and self.last_growth_t is not None
                   and t - self.last_growth_t > stall)
        ctx.condition(t, 'explore_stall', stalled, 'warning', 'exploration',
                      'Exploration stalled',
                      f'The known map area has not grown for {stall:.0f} s while exploring.',
                      hold=1.0)


# ======================================================================== logs
class LogInterpreter(Interpreter):
    types = (LOG,)

    def __init__(self):
        self.warnings = 0
        self.errors = 0

    def on_message(self, ctx, t, topic, msg):
        if msg.level < 30 or msg.name == 'maze_monitor':
            return
        sev = 'warning' if msg.level < 40 else 'error'
        if sev == 'warning':
            self.warnings += 1
        else:
            self.errors += 1
        text = msg.msg.strip().splitlines()[0] if msg.msg.strip() else '(empty message)'
        short = re.sub(r'-?\d+(\.\d+)?', '#', text)  # merge messages that differ only in numbers
        ctx.event(t, sev, 'log', f'[{msg.name}] {text[:120]}', msg.msg,
                  merge_key=f'log:{msg.name}:{short}', merge_window=10.0)
        ctx.emit(t, 'log.warnings', float(self.warnings), 'Log warnings', '')
        ctx.emit(t, 'log.errors', float(self.errors), 'Log errors', '')


# ======================================================================== health
class HealthInterpreter(Interpreter):
    """Topic rates, silent topics, and the simulation real-time factor."""

    types = (CLOCK,)
    STEADY = ('/scan', '/odom', '/imu', '/clock', '/tf')

    def __init__(self):
        self.clock = deque()  # (wall t, sim t)
        self.slow_since = None
        self.tracks = {}

    def on_any(self, ctx, t, track):
        self.tracks[track.name] = track

    def on_message(self, ctx, t, topic, msg):
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock.append((t, sim))
        while self.clock and t - self.clock[0][0] > 5.0:
            self.clock.popleft()
        ctx.emit(t, 'sim.time', sim, 'Simulation time', 's', min_interval=0.2)
        if len(self.clock) >= 2 and t - self.clock[0][0] >= 2.0:
            rtf = (sim - self.clock[0][1]) / (t - self.clock[0][0])
            ctx.emit(t, 'sim.rtf', rtf, 'Real-time factor', '×', min_interval=0.2)
            slow = 0 <= rtf < ctx.th('slow_rtf')
            self.slow_since = (self.slow_since if self.slow_since is not None else t) if slow else None
            ctx.condition(t, 'slow_sim', slow and t - self.slow_since >= 5.0, 'warning', 'system',
                          f'Simulation runs at {rtf:.2f}× real time',
                          'Gazebo cannot keep up with real time. Nav2 and SLAM then miss '
                          'their deadlines, which shows as jerky motion, late transforms, and '
                          '"control loop missed its desired rate" warnings.', hold=2.0)

    def tick(self, ctx, t):
        k = ctx.th('silent_periods')
        for name in self.STEADY:
            tr = self.tracks.get(name)
            if tr is None or len(tr) < 20:
                continue
            times = tr.times
            period = (times[-1] - times[-20]) / 19.0
            if period <= 0 or period > 1.0:
                continue
            gap = t - times[-1]
            silent = gap > max(1.0, k * period)
            ctx.condition(t, f'silent:{name}', silent, 'warning', 'topic',
                          f'{name} went silent',
                          f'No message on {name} for {gap:.1f} s (usually {1 / period:.1f} Hz). '
                          'Its publisher stopped, crashed, or is overloaded.', hold=0.0)


# ======================================================================== stamps
class StampInterpreter(Interpreter):
    """Header stamps that jump backward: two publishers on one topic, or a clock reset."""

    types = (ODOM, SCAN, IMU)

    def __init__(self):
        self.last = {}

    def on_message(self, ctx, t, topic, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        prev = self.last.get(topic)
        if prev is not None and stamp < prev - 0.05:
            back = prev - stamp
            if back > 5.0:
                ctx.event(t, 'warning', 'system', f'Clock restarted on {topic}',
                          f'Message time went back {back:.1f} s. The simulation was probably '
                          'reset or restarted.', merge_key=f'reset:{topic}', merge_window=5.0)
                self.last[topic] = stamp
                return
            ctx.event(t, 'error', 'topic', f'Timestamps go backward on {topic}',
                      f'A message stamped {stamp:.2f} s arrived after one stamped {prev:.2f} s. '
                      'Usually two nodes (or two simulations) publish this topic, so other '
                      'nodes see the robot in two states at once, which breaks transforms and '
                      'odometry. Check with: ros2 topic info '
                      f'{topic} --verbose. (Runs recorded before the monitor used a steady '
                      'clock can also show this when the WSL system clock stepped back.)',
                      merge_key=f'stamp:{topic}', merge_window=5.0)
            return  # keep the newest stamp so each stale message is counted
        self.last[topic] = stamp


def default_interpreters():
    return [PoseInterpreter(), StampInterpreter(), MotionInterpreter(), ClearanceInterpreter(), ImpactInterpreter(),
            NavigationInterpreter(), ExplorationInterpreter(), LogInterpreter(),
            HealthInterpreter()]
