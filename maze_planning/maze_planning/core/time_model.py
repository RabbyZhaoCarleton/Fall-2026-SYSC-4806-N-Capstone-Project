"""How long a drive takes.

The goal is the most *time*-efficient plan, not just the shortest. Driving time
is more than distance / speed:

* The robot slows down and turns in place at sharp corners. With Nav2's rotation
  shim controller (nav2_maze.yaml: angular_dist_threshold 0.785 rad), any heading
  change over ~45 degrees is done as an in-place rotation.
* After a pickup the robot faces the object; after a drop-off it faces the exit.
  Leaving usually means turning around first.

So a leg's time is

    time = distance / cruise_speed
         + sum over sharp turns (|turn| / rotate_speed + turn_overhead)

Small heading changes are taken while driving and cost nothing extra.

With a holonomic (mecanum) drive the robot can strafe, so it does not need to
face the direction it drives in; then only the change between the start and
goal headings is counted.

All of these are parameters (config/global_planner.yaml). Tune them by comparing
the planner's estimates with the measured step times that the task manager logs.
"""

import math
from dataclasses import dataclass

from .search import polyline_length


def angle_diff(a, b):
    """Smallest signed angle from a to b, in (-pi, pi]."""
    d = (b - a) % (2.0 * math.pi)
    if d > math.pi:
        d -= 2.0 * math.pi
    return d


@dataclass
class LegEstimate:
    distance: float      # m driven
    time: float          # s
    turning: float       # rad turned in place in total (for reports)


@dataclass
class TimeModel:
    cruise_speed: float = 0.12          # m/s average forward speed (Nav2 max_vel_x is 0.15)
    rotate_speed: float = 0.8           # rad/s average in-place rotation speed
    inplace_turn_threshold: float = 0.785  # rad; sharper turns are done in place
    turn_overhead: float = 0.5          # s lost per in-place turn (stopping, settling)
    holonomic: bool = False             # True for a mecanum drive that strafes
    pick_time: float = 20.0             # s to pick up one object (align, grasp, lift, check)
    place_time: float = 8.0             # s to drop one object into a bin

    def _turn_cost(self, angle):
        angle = abs(angle)
        if angle < self.inplace_turn_threshold:
            return 0.0, 0.0
        return angle / self.rotate_speed + self.turn_overhead, angle

    def leg(self, points, start_yaw, goal_yaw):
        """Estimate driving along points (list of (x, y)) from start_yaw, ending at goal_yaw."""
        distance = polyline_length(points)
        time = distance / self.cruise_speed
        turning = 0.0

        if self.holonomic:
            t, a = self._turn_cost(angle_diff(start_yaw, goal_yaw))
            return LegEstimate(distance, time + t, a)

        # Heading of every segment that is long enough to have a direction.
        headings = []
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            if math.hypot(x1 - x0, y1 - y0) > 1e-6:
                headings.append(math.atan2(y1 - y0, x1 - x0))

        # The robot turns from where it faces now, through every corner of the
        # route, to the heading wanted at the goal.
        sequence = [start_yaw] + headings + [goal_yaw]
        for a, b in zip(sequence, sequence[1:]):
            t, angle = self._turn_cost(angle_diff(a, b))
            time += t
            turning += angle
        return LegEstimate(distance, time, turning)
