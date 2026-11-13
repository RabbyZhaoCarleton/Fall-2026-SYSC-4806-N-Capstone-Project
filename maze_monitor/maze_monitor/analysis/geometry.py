"""Small 2D geometry helpers. Angles are in radians unless a name says otherwise."""

import math


def wrap(angle):
    """Wrap an angle to the range [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q):
    """Yaw (rotation about z) of a geometry_msgs Quaternion-like object."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Pose2D:
    __slots__ = ('x', 'y', 'yaw')

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw

    def compose(self, other):
        """Return self * other (apply other in the frame of self)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(self.x + c * other.x - s * other.y,
                      self.y + s * other.x + c * other.y,
                      wrap(self.yaw + other.yaw))

    def distance(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)

    def __repr__(self):
        return f'Pose2D({self.x:.3f}, {self.y:.3f}, {math.degrees(self.yaw):.1f} deg)'


def direction_name(bearing):
    """Name a robot-relative bearing (0 = ahead, positive = left)."""
    deg = math.degrees(wrap(bearing))
    names = [(22.5, 'ahead'), (67.5, 'front left'), (112.5, 'left'), (157.5, 'rear left')]
    a = abs(deg)
    if a >= 157.5:
        return 'behind'
    for limit, name in names:
        if a < limit:
            return name.replace('left', 'right') if deg < 0 else name
    return 'behind'
