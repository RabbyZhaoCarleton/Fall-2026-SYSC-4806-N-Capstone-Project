"""The whole planner on maze drawings (no ROS)."""

import math
import os
import random

import pytest

from maze_planning.core.maze_layout import layout_costmap, load_layout, parse_layout, random_layout
from maze_planning.core.planner import (NAVIGATE_TO_EXIT, NAVIGATE_TO_PICKUP, NO_PLAN,
                                        NOTHING_TO_DO, OK, PARTIAL, PICK, PLACE, ExitInfo,
                                        GlobalPlanner, ObjectTask, PlannerConfig, Pose2D)
from maze_planning.tools.plan_offline import check_against_brute_force

HERE = os.path.dirname(os.path.abspath(__file__))
LAYOUTS = os.path.join(HERE, '..', 'layouts')


@pytest.fixture(scope='module')
def default_maze():
    layout = load_layout(os.path.join(LAYOUTS, 'default_maze.txt'))
    grid, _, _ = layout_costmap(layout)
    return layout, grid


def test_layout_matches_the_gazebo_maze(default_maze):
    layout, grid = default_maze
    assert (layout.rows, layout.cols) == (3, 4)
    assert sorted(layout.exits) == sorted([(2.0, -0.25), (1.25, -1.5)])
    assert not grid.blocked[grid.index(*grid.world_to_cell(0.25, -0.25))]   # start cell free
    assert grid.blocked[grid.index(*grid.world_to_cell(1.0, -0.25))]        # wall at x = 1.0


def test_plan_delivers_every_object_once(default_maze):
    layout, grid = default_maze
    exits = [ExitInfo('exit_A', 2.0, -0.25), ExitInfo('exit_B', 1.25, -1.5)]
    objects = [ObjectTask('o1', 'exit_A', 0.4, -0.15), ObjectTask('o2', 'exit_B', 0.4, -0.25),
               ObjectTask('o3', 'exit_A', 0.4, -0.35)]
    result = GlobalPlanner().plan(grid, Pose2D(0.25, -0.25, 0.0), objects, exits)
    assert result.status == OK and result.solver == 'exact'
    actions = [s.action for s in result.steps]
    assert actions == [NAVIGATE_TO_PICKUP, PICK, NAVIGATE_TO_EXIT, PLACE] * 3
    assert sorted(s.object_id for s in result.steps if s.action == PLACE) == ['o1', 'o2', 'o3']
    for s in result.steps:
        if s.action == PLACE:
            assert s.exit_id == {'o1': 'exit_A', 'o2': 'exit_B', 'o3': 'exit_A'}[s.object_id]
    # Times add up, and every route stays on free cells.
    clock = 0.0
    for s in result.steps:
        assert math.isclose(s.start_time, clock, abs_tol=1e-6)
        clock += s.duration
        for x, y in s.path:
            assert not grid.blocked[grid.index(*grid.world_to_cell(x, y))]
    assert math.isclose(clock, result.total_time)


def test_drop_poses_are_inside_the_maze_facing_the_exit(default_maze):
    _, grid = default_maze
    exits = [ExitInfo('exit_A', 2.0, -0.25)]
    result = GlobalPlanner().plan(grid, Pose2D(0.25, -0.25, 0.0),
                                  [ObjectTask('o1', 'exit_A', 0.4, -0.25)], exits)
    drop = next(s for s in result.steps if s.action == NAVIGATE_TO_EXIT).target
    assert drop.x < 2.0                                  # inside, not in the opening
    assert abs(math.hypot(2.0 - drop.x, -0.25 - drop.y) - 0.25) < 0.08   # ~exit_standoff
    assert abs(drop.yaw) < math.radians(30)              # facing +x, towards the exit


def test_farthest_exit_is_last(default_maze):
    _, grid = default_maze
    exits = [ExitInfo('exit_A', 2.0, -0.25), ExitInfo('exit_B', 1.25, -1.5)]
    objects = [ObjectTask(f'o{i}', e, 0.4, -0.2 - 0.05 * i)
               for i, e in enumerate(['exit_A', 'exit_B', 'exit_A', 'exit_B'])]
    planner = GlobalPlanner()
    robot = Pose2D(0.25, -0.25, 0.0)
    result = planner.plan(grid, robot, objects, exits)
    # With the objects at the entrance and capacity 1, every trip is entrance ->
    # exit -> back, except the last, which does not come back. The fastest plan
    # therefore ends at the exit with the longest drive back to the entrance.
    inp = planner.prepare(grid, robot, objects, exits).solver_input
    pick = inp.pickup[0]
    back = {'exit_A': inp.leg_time[inp.drop[0]][pick], 'exit_B': inp.leg_time[inp.drop[1]][pick]}
    last_exit = [s.exit_id for s in result.steps if s.action == PLACE][-1]
    assert last_exit == max(back, key=back.get)


def test_missing_exit_gives_partial_plan(default_maze):
    _, grid = default_maze
    objects = [ObjectTask('o1', 'exit_A', 0.4, -0.25), ObjectTask('o2', 'exit_Z', 0.4, -0.3)]
    result = GlobalPlanner().plan(grid, Pose2D(0.25, -0.25, 0.0), objects,
                                  [ExitInfo('exit_A', 2.0, -0.25)])
    assert result.status == PARTIAL
    assert [oid for oid, _ in result.unplanned] == ['o2']
    assert {s.object_id for s in result.steps if s.object_id} == {'o1'}


def test_nothing_to_do_and_no_plan(default_maze):
    _, grid = default_maze
    assert GlobalPlanner().plan(grid, Pose2D(0.25, -0.25), [], []).status == NOTHING_TO_DO
    far_away = GlobalPlanner().plan(grid, Pose2D(10.0, 10.0), [ObjectTask('o', 'e', 0.4, -0.2)],
                                    [ExitInfo('e', 2.0, -0.25)])
    assert far_away.status == NO_PLAN


def test_carried_object_is_dropped_first(default_maze):
    _, grid = default_maze
    exits = [ExitInfo('exit_A', 2.0, -0.25), ExitInfo('exit_B', 1.25, -1.5)]
    objects = [ObjectTask('o1', 'exit_A', 0.4, -0.25),
               ObjectTask('o2', 'exit_B', 0.4, -0.3, carried=True)]
    result = GlobalPlanner().plan(grid, Pose2D(0.25, -0.25, 0.0), objects, exits)
    assert [s.action for s in result.steps[:2]] == [NAVIGATE_TO_EXIT, PLACE]
    assert result.steps[1].object_id == 'o2'


def test_capacity_two_takes_fewer_trips(default_maze):
    _, grid = default_maze
    exits = [ExitInfo('exit_A', 2.0, -0.25), ExitInfo('exit_B', 1.25, -1.5)]
    objects = [ObjectTask(f'o{i}', e, 0.4, -0.2 - 0.05 * i)
               for i, e in enumerate(['exit_A', 'exit_A', 'exit_B', 'exit_B'])]
    one = GlobalPlanner(PlannerConfig(capacity=1)).plan(grid, Pose2D(0.25, -0.25), objects, exits)
    two = GlobalPlanner(PlannerConfig(capacity=2)).plan(grid, Pose2D(0.25, -0.25), objects, exits)
    assert len({s.trip for s in one.steps}) == 4
    assert len({s.trip for s in two.steps}) <= 3
    assert two.total_time < one.total_time


@pytest.mark.parametrize('seed', range(20))
def test_random_mazes_are_planned_optimally(seed):
    """Proposal test 4.2.5: the time-optimal route in 100 % of (20+) random layouts."""
    rng = random.Random(seed)
    layout = parse_layout(random_layout(5, 5, 3, rng, extra_openings=rng.randint(0, 3)))
    grid, _, _ = layout_costmap(layout)
    exits = [ExitInfo(f'e{i}', x, y) for i, (x, y) in enumerate(layout.exits)]
    objects = [ObjectTask(f'o{i}', rng.choice(exits).id, 0.25, -0.40) for i in range(4)]
    planner = GlobalPlanner()
    robot = Pose2D(0.25, -0.25, 0.0)
    result = planner.plan(grid, robot, objects, exits)
    assert result.status == OK
    assert math.isclose(result.total_time,
                        check_against_brute_force(planner, grid, robot, objects, exits),
                        rel_tol=1e-9)


def test_object_against_a_wall_can_still_be_picked(default_maze):
    # The object is inside the wall's inflation zone (closer than the robot's
    # radius). The robot cannot stand there, but it can stop nearby and reach it.
    _, grid = default_maze
    x, y = 0.04, -0.75          # 4 cm from the maze's left outer wall
    cell = grid.world_to_cell(x, y)
    assert grid.blocked[grid.index(*cell)]
    result = GlobalPlanner().plan(grid, Pose2D(0.25, -0.25), [ObjectTask('o', 'e', x, y)],
                                  [ExitInfo('e', 2.0, -0.25)])
    assert result.status == OK
    pick = next(s for s in result.steps if s.action == PICK).target
    # About pickup_standoff (0.2 m) away, facing the object.
    assert abs(math.hypot(pick.x - x, pick.y - y) - 0.2) < 0.05
    facing = math.atan2(y - pick.y, x - pick.x)
    assert abs(math.atan2(math.sin(pick.yaw - facing), math.cos(pick.yaw - facing))) < 0.05


def test_object_behind_a_wall_is_not_approached_through_it(default_maze):
    # An object just below the wall at y = -0.5, x 0.5-1.0 (row 0/1 boundary of
    # column 1 is open, column 3 is a wall). Its approach pose must be on the same
    # side of the wall as the object.
    _, grid = default_maze
    x, y = 1.75, -0.6           # just below the wall between rows 0 and 1, column 3
    result = GlobalPlanner().plan(grid, Pose2D(0.25, -0.25), [ObjectTask('o', 'e', x, y)],
                                  [ExitInfo('e', 1.25, -1.5)])
    pick = next(s for s in result.steps if s.action == PICK).target
    assert pick.y < -0.5
