"""The ordering solver: exact == brute force, and the plans are valid."""

import itertools
import math
import random

import pytest

from maze_planning.core.solver import DeliverySolver, SolverInput, brute_force_time


def random_problem(rng, n_objects, n_exits, capacity, same_pickup=False):
    """Places: 0 robot, 1..n_exits exits, then one pickup per object. Random positions."""
    n_places = 1 + n_exits + n_objects
    pts = [(rng.uniform(0, 5), rng.uniform(0, 5)) for _ in range(n_places)]
    if same_pickup:  # all objects at the entrance, next to the robot
        for k in range(n_objects):
            pts[1 + n_exits + k] = (pts[0][0] + rng.uniform(-0.1, 0.1), pts[0][1])
    leg = [[math.dist(a, b) / 0.12 + (0 if a == b else 2.0) for b in pts] for a in pts]
    pickup = [1 + n_exits + k for k in range(n_objects)]
    drop = [1 + rng.randrange(n_exits) for _ in range(n_objects)]
    return SolverInput(0, pickup, drop, leg, pick_time=20.0, place_time=8.0, capacity=capacity)


def check_valid(inp, sol):
    """Every object picked once (unless carried) and dropped once at its own exit."""
    picked = [i for t in sol.trips for i in t.picks]
    dropped = []
    for t in sol.trips:
        assert len(t.picks) <= inp.capacity
        for place, objs in t.drops:
            for i in objs:
                assert inp.drop[i] == place
                dropped.append(i)
    waiting = [i for i, p in enumerate(inp.pickup) if p is not None]
    assert sorted(picked) == sorted(waiting)
    assert sorted(dropped) == list(range(len(inp.pickup)))
    assert math.isclose(sum(t.time for t in sol.trips), sol.total_time)


@pytest.mark.parametrize('capacity', [1, 2, 3])
@pytest.mark.parametrize('seed', range(15))
def test_exact_matches_brute_force(capacity, seed):
    rng = random.Random(seed * 10 + capacity)
    inp = random_problem(rng, n_objects=rng.randint(1, 5), n_exits=rng.randint(1, 3),
                         capacity=capacity, same_pickup=seed % 2 == 0)
    sol = DeliverySolver(inp).solve()
    assert sol.solver == 'exact'
    check_valid(inp, sol)
    assert math.isclose(sol.total_time, brute_force_time(inp), rel_tol=1e-9)


def test_capacity_one_with_objects_at_the_entrance_ends_at_the_farthest_exit():
    # Objects at the entrance, capacity 1: every trip is entrance -> exit -> entrance,
    # except the last one, which does not come back. So the best plan does the
    # farthest exit last (that saves the longest return drive).
    leg = [[0, 10, 30, 0], [10, 0, 25, 10], [30, 25, 0, 30], [0, 10, 30, 0]]
    # places: 0 robot, 1 near exit, 2 far exit, 3 entrance pickup (same spot as the robot)
    inp = SolverInput(0, pickup=[3, 3, 3], drop=[2, 1, 1], leg_time=leg,
                      pick_time=0, place_time=0, capacity=1)
    sol = DeliverySolver(inp).solve()
    assert sol.trips[-1].drops[0][0] == 2
    assert math.isclose(sol.total_time, 10 + 10 + 10 + 10 + 30)


def test_carried_object_is_delivered_first():
    rng = random.Random(5)
    inp = random_problem(rng, n_objects=4, n_exits=2, capacity=1)
    inp.pickup[2] = None   # object 2 is already in the gripper
    sol = DeliverySolver(inp).solve()
    check_valid(inp, sol)
    assert sol.trips[0].picks == [] and sol.trips[0].drops[0][1] == [2]
    # Optimal among plans that deliver object 2 first: compare with brute force on
    # the rest, started from each possible first drop.
    first_drop = inp.drop[2]
    t_first = inp.leg_time[0][first_drop] + inp.place_time
    rest = SolverInput(first_drop, [p for k, p in enumerate(inp.pickup) if k != 2],
                       [d for k, d in enumerate(inp.drop) if k != 2], inp.leg_time,
                       inp.pick_time, inp.place_time, 1)
    assert math.isclose(sol.total_time, t_first + brute_force_time(rest))


def test_capacity_two_groups_same_exit():
    # Two objects for the same far exit: with capacity 2 one trip takes both.
    leg = [[0, 50, 0], [50, 0, 50], [0, 50, 0]]
    one = DeliverySolver(SolverInput(0, [2, 2], [1, 1], leg, 0, 0, 1)).solve()
    two = DeliverySolver(SolverInput(0, [2, 2], [1, 1], leg, 0, 0, 2)).solve()
    assert len(one.trips) == 2 and math.isclose(one.total_time, 150)
    assert len(two.trips) == 1 and math.isclose(two.total_time, 50)


def test_heuristic_is_valid_and_close():
    rng = random.Random(42)
    for _ in range(5):
        inp = random_problem(rng, n_objects=7, n_exits=3, capacity=1, same_pickup=True)
        exact = DeliverySolver(inp, max_exact_objects=12).solve()
        heur = DeliverySolver(inp, max_exact_objects=3).solve()
        assert heur.solver == 'heuristic'
        check_valid(inp, heur)
        assert heur.total_time >= exact.total_time - 1e-9
        assert heur.total_time <= exact.total_time * 1.10   # within 10 % here


def test_many_objects_exact_is_fast():
    import time
    rng = random.Random(3)
    inp = random_problem(rng, n_objects=10, n_exits=3, capacity=1, same_pickup=True)
    t = time.monotonic()
    sol = DeliverySolver(inp).solve()
    assert sol.solver == 'exact'
    check_valid(inp, sol)
    assert time.monotonic() - t < 5.0
