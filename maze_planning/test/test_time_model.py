"""Driving time estimates."""

import math

from maze_planning.core.time_model import TimeModel, angle_diff


def test_angle_diff_wraps():
    assert math.isclose(angle_diff(math.radians(170), math.radians(-170)), math.radians(20))
    assert math.isclose(angle_diff(0.0, math.pi), math.pi)


def test_straight_leg_is_distance_over_speed():
    tm = TimeModel(cruise_speed=0.1)
    est = tm.leg([(0, 0), (1, 0)], 0.0, 0.0)
    assert math.isclose(est.distance, 1.0) and math.isclose(est.time, 10.0)


def test_u_turn_costs_rotation_time():
    tm = TimeModel(cruise_speed=0.1, rotate_speed=1.0, turn_overhead=0.5)
    ahead = tm.leg([(0, 0), (1, 0)], 0.0, 0.0)
    behind = tm.leg([(0, 0), (-1, 0)], 0.0, math.pi)  # must turn around first
    assert math.isclose(behind.time - ahead.time, math.pi / 1.0 + 0.5)


def test_gentle_turns_are_free():
    tm = TimeModel(cruise_speed=0.1, inplace_turn_threshold=0.785)
    est = tm.leg([(0, 0), (1, 0), (2, 0.3)], 0.0, math.atan2(0.3, 1.0))
    assert math.isclose(est.time, est.distance / 0.1)


def test_holonomic_ignores_route_corners():
    tm = TimeModel(cruise_speed=0.1, holonomic=True)
    est = tm.leg([(0, 0), (1, 0), (1, 1)], 0.0, 0.0)
    assert math.isclose(est.time, 20.0)
