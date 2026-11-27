"""Converting between ROS messages and the planner's plain Python classes.

Kept separate from the nodes so the conversions can be tested on their own and
so the core planner never imports ROS.
"""

import math

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from maze_interfaces.msg import DeliveryObject, DeliveryPlan, PlanStep
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ..core.grid import CostGrid
from ..core.planner import (NAVIGATE_TO_EXIT, NAVIGATE_TO_PICKUP, PICK, PLACE,
                            ExitInfo, ObjectTask, Pose2D)


# ------------------------------------------------------------ poses
def yaw_from_quaternion(q):
    """Heading (rotation about z) of a geometry_msgs/Quaternion, in radians."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def pose_from_2d(p):
    return Pose(position=Point(x=float(p.x), y=float(p.y), z=0.0),
                orientation=quaternion_from_yaw(p.yaw))


def pose2d_from_pose(pose):
    return Pose2D(pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))


# ------------------------------------------------------------ inputs
def costgrid_from_occupancy(msg, blocked_threshold=99, allow_unknown=False):
    """nav_msgs/OccupancyGrid (the global costmap) -> CostGrid.

    The costmap's origin has no rotation in Nav2, so only its position is used.
    """
    info = msg.info
    return CostGrid(info.width, info.height, info.resolution,
                    info.origin.position.x, info.origin.position.y, msg.data,
                    blocked_threshold=blocked_threshold, allow_unknown=allow_unknown)


def tasks_from_objects(msg):
    """DeliveryObjectArray -> ObjectTask list of the objects still to deliver."""
    tasks = []
    for o in msg.objects:
        if o.state not in (DeliveryObject.WAITING, DeliveryObject.CARRIED):
            continue
        tasks.append(ObjectTask(id=o.id, exit_id=o.exit_id,
                                x=o.pose.position.x, y=o.pose.position.y,
                                carried=o.state == DeliveryObject.CARRIED,
                                object_class=o.object_class))
    return tasks


def exits_from_msg(msg):
    """ExitBinArray -> ExitInfo list."""
    return [ExitInfo(e.id, e.pose.position.x, e.pose.position.y) for e in msg.exits]


# ------------------------------------------------------------ outputs
def plan_to_msg(result, plan_id, frame_id, stamp):
    """PlanResult -> maze_interfaces/DeliveryPlan."""
    msg = DeliveryPlan()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.plan_id = plan_id
    msg.status = result.status
    msg.message = result.message
    msg.solver = result.solver
    msg.capacity = result.capacity
    msg.total_time = float(result.total_time)
    msg.total_distance = float(result.total_distance)
    msg.planning_time = float(result.planning_time)
    for s in result.steps:
        msg.steps.append(PlanStep(
            action=s.action, trip=s.trip, object_id=s.object_id, exit_id=s.exit_id,
            target=pose_from_2d(s.target), distance=float(s.distance),
            duration=float(s.duration), start_time=float(s.start_time)))
    msg.unplanned_object_ids = [oid for oid, _ in result.unplanned]
    msg.unplanned_reasons = [why for _, why in result.unplanned]
    return msg


def plan_to_path(result, frame_id, stamp):
    """The whole planned route as one nav_msgs/Path (for RViz and the GUI)."""
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for s in result.steps:
        for i, (x, y) in enumerate(s.path):
            # Point each pose along the route to the next point (just for display).
            if i + 1 < len(s.path):
                nx, ny = s.path[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = s.target.yaw
            ps = PoseStamped()
            ps.header = path.header
            ps.pose = pose_from_2d(Pose2D(x, y, yaw))
            path.poses.append(ps)
    return path


# Distinct colours for trips, so each trip's route shows up separately in RViz.
_TRIP_COLOURS = [(0.12, 0.47, 0.71), (1.0, 0.5, 0.05), (0.17, 0.63, 0.17),
                 (0.84, 0.15, 0.16), (0.58, 0.40, 0.74), (0.55, 0.34, 0.29)]


def _colour(i, a=1.0):
    r, g, b = _TRIP_COLOURS[i % len(_TRIP_COLOURS)]
    return ColorRGBA(r=r, g=g, b=b, a=a)


def plan_markers(result, objects, exits, frame_id, stamp):
    """RViz markers: each trip's route in its own colour, numbered goals, objects and exits."""
    markers = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)
    mid = 0

    def new(ns, mtype):
        nonlocal mid
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        mid += 1
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    # Routes, one line strip per driving step, coloured by trip.
    for s in result.steps:
        if s.action in (NAVIGATE_TO_PICKUP, NAVIGATE_TO_EXIT) and len(s.path) > 1:
            m = new('route', Marker.LINE_STRIP)
            m.scale.x = 0.025
            m.color = _colour(s.trip, 0.9)
            m.points = [Point(x=x, y=y, z=0.03) for x, y in s.path]
            markers.markers.append(m)

    # Goal poses as arrows with their step number and action.
    number = 0
    for s in result.steps:
        if s.action not in (PICK, PLACE):
            continue
        number += 1
        arrow = new('goals', Marker.ARROW)
        arrow.pose = pose_from_2d(s.target)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.15, 0.03, 0.03
        arrow.color = _colour(s.trip)
        markers.markers.append(arrow)
        text = new('labels', Marker.TEXT_VIEW_FACING)
        text.pose.position.x = s.target.x
        text.pose.position.y = s.target.y
        text.pose.position.z = 0.15 + 0.07 * (number % 3)
        text.scale.z = 0.06
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        what = 'pick ' + s.object_id if s.action == PICK else 'drop ' + s.object_id
        text.text = f'{number}. {what}'
        markers.markers.append(text)

    # Objects (small cubes) and exits (tall green bars).
    for o in objects:
        m = new('objects', Marker.CUBE)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = o.x, o.y, 0.03
        m.scale.x = m.scale.y = m.scale.z = 0.05
        m.color = ColorRGBA(r=0.9, g=0.8, b=0.1, a=1.0)
        markers.markers.append(m)
    for e in exits:
        m = new('exits', Marker.CYLINDER)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = e.x, e.y, 0.1
        m.scale.x = m.scale.y = 0.08
        m.scale.z = 0.2
        m.color = ColorRGBA(r=0.1, g=0.8, b=0.2, a=0.9)
        markers.markers.append(m)
        t = new('exit_names', Marker.TEXT_VIEW_FACING)
        t.pose.position.x, t.pose.position.y, t.pose.position.z = e.x, e.y, 0.3
        t.scale.z = 0.08
        t.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0)
        t.text = e.id
        markers.markers.append(t)
    return markers
