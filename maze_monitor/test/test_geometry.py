import math

from maze_monitor.analysis.geometry import Pose2D, direction_name, wrap


def test_wrap():
    assert abs(wrap(3 * math.pi) - (-math.pi)) < 1e-9
    assert abs(wrap(0.5)) - 0.5 < 1e-9


def test_compose():
    a = Pose2D(1.0, 0.0, math.pi / 2)
    b = Pose2D(1.0, 0.0, 0.0)
    c = a.compose(b)
    assert abs(c.x - 1.0) < 1e-9 and abs(c.y - 1.0) < 1e-9


def test_direction_names():
    assert direction_name(0.0) == 'ahead'
    assert direction_name(math.radians(45)) == 'front left'
    assert direction_name(math.radians(-90)) == 'right'
    assert direction_name(math.radians(180)) == 'behind'
