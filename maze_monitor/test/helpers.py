"""Build synthetic runs for the detector tests."""

import math

from rclpy.serialization import serialize_message

from maze_monitor.config import Config
from maze_monitor.session import Session


class Feeder:
    def __init__(self, **thresholds):
        self.config = Config.load()
        self.config.thresholds.update(thresholds)
        self.session = Session(self.config, 'live', start_ns=0)

    def feed(self, t, topic, msg, type_str=None):
        ts = type_str or f'{type(msg).__module__.split(".")[0]}/msg/{type(msg).__name__}'
        self.session.ingest(topic, ts, serialize_message(msg), int(t * 1e9))

    def finish(self, t):
        self.session.analyzer.finish(t)
        return self.session.timeline

    def events(self, key=None, category=None):
        return [e for e in self.session.timeline.events
                if (key is None or e.key == key) and (category is None or e.category == category)]


def odom(x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0):
    from nav_msgs.msg import Odometry
    o = Odometry()
    o.pose.pose.position.x, o.pose.pose.position.y = x, y
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
    return o


def cmd(v=0.0, w=0.0):
    from geometry_msgs.msg import TwistStamped
    c = TwistStamped()
    c.twist.linear.x, c.twist.angular.z = v, w
    return c


def scan(fn):
    """fn(bearing in degrees, -180..180) -> range."""
    from sensor_msgs.msg import LaserScan
    s = LaserScan()
    s.angle_min, s.angle_increment = 0.0, 2 * math.pi / 360
    s.range_min, s.range_max = 0.12, 3.5
    s.ranges = [float(fn(((a + 180) % 360) - 180)) for a in range(360)]
    return s
