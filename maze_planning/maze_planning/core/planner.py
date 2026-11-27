"""The global delivery planner, without ROS.

    GlobalPlanner.plan(grid, robot, objects, exits) -> PlanResult

Pipeline (each step is explained where it happens below):

  1. Search from the robot: which cells can it reach at all?
  2. Places: turn every object and exit into a pose the robot can drive to
     (in front of the object / in front of the exit opening).
  3. Legs: the driving time between every pair of places (one Dijkstra per place,
     then the time model).
  4. Order: the solver picks the fastest sequence of trips.
  5. Steps: write the order out as NAVIGATE / PICK / NAVIGATE / PLACE steps with
     goal poses and estimated times, which the ROS node publishes.

The ROS node (ros/global_planner_node.py) only converts messages to these
classes and back, so everything here can be tested without ROS.
"""

import math
import time as _time
from dataclasses import dataclass, field

from .search import dijkstra, extract_path, smooth_path
from .solver import DeliverySolver, SolverInput
from .time_model import TimeModel

# Step actions; the same numbers as maze_interfaces/msg/PlanStep.
NAVIGATE_TO_PICKUP = 0
PICK = 1
NAVIGATE_TO_EXIT = 2
PLACE = 3
ACTION_NAMES = {NAVIGATE_TO_PICKUP: 'NAVIGATE_TO_PICKUP', PICK: 'PICK',
                NAVIGATE_TO_EXIT: 'NAVIGATE_TO_EXIT', PLACE: 'PLACE'}

# Plan status; the same numbers as maze_interfaces/msg/DeliveryPlan.
OK = 0
PARTIAL = 1
NOTHING_TO_DO = 2
NO_PLAN = 3


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass
class ObjectTask:
    """An object the plan must deliver (WAITING or CARRIED objects only)."""
    id: str
    exit_id: str
    x: float
    y: float
    carried: bool = False
    object_class: str = ''


@dataclass
class ExitInfo:
    id: str
    x: float      # centre of the exit opening
    y: float


@dataclass
class Step:
    action: int
    trip: int
    object_id: str = ''
    exit_id: str = ''
    target: Pose2D = None
    distance: float = 0.0
    duration: float = 0.0
    start_time: float = 0.0
    path: list = field(default_factory=list)   # (x, y) points driven in this step


@dataclass
class PlanResult:
    status: int
    message: str
    solver: str = ''
    capacity: int = 1
    steps: list = field(default_factory=list)
    total_time: float = 0.0
    total_distance: float = 0.0
    planning_time: float = 0.0
    unplanned: list = field(default_factory=list)   # [(object id, reason)]


@dataclass
class Problem:
    """Everything the solver needs, worked out from the costmap (planner steps 1-3)."""
    objects: list          # ObjectTask list that can be planned (solver object i = objects[i])
    unplanned: list        # [(object id, reason)]
    place_poses: list      # Pose2D per place; place 0 is the robot
    legs: list             # legs[a][b] = (LegEstimate, [(x, y), ...]) or None
    solver_input: SolverInput


@dataclass
class PlannerConfig:
    capacity: int = 1                    # objects carried at once
    max_exact_objects: int = 12          # above this, use the heuristic solver
    cost_travel_multiplier: float = 2.0  # mu in c(u,v) = d(u,v) * (1 + mu * kappa(v))
    robot_snap_distance: float = 0.3     # m; how far to look for a free cell near the robot
    pickup_standoff: float = 0.20        # m from the object's centre to the robot's centre when picking
    exit_standoff: float = 0.25          # m from the exit opening to the robot's centre when dropping
    approach_search_radius: float = 0.6  # m around an object/exit to look for its approach pose
    max_goal_kappa: float = 0.8          # approach poses must be at least this far from walls (0..1)
    smoothing_max_kappa: float = 0.9     # straight shortcuts may not pass cells costlier than this
    time_model: TimeModel = field(default_factory=TimeModel)


class GlobalPlanner:
    def __init__(self, config=None):
        self.config = config or PlannerConfig()

    # ------------------------------------------------------------ places
    def _approach_pose(self, grid, reach_cost, tx, ty, standoff):
        """Pose from which the robot works on something at (tx, ty).

        The robot cannot drive *onto* an object or *into* an exit opening, so the
        goal for Nav2 is a free cell about `standoff` metres away that

          * the robot can reach (finite cost from the robot's search),
          * is not too close to a wall (kappa <= max_goal_kappa),
          * can see (tx, ty) in a straight line with no wall in between (so it
            is in the same corridor, not on the other side of a wall),

        facing (tx, ty). Of those, the one closest to `standoff` wins, then the
        cheapest to reach. Returns (cell index, Pose2D) or (None, None).
        """
        cfg = self.config
        target = grid.world_to_cell(tx, ty)
        if target is None:
            return None, None
        tidx = grid.index(*target)
        r_cells = int(math.ceil(cfg.approach_search_radius / grid.resolution))
        best_key, best = None, None
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                cx, cy = target[0] + dx, target[1] + dy
                if not grid.in_bounds(cx, cy):
                    continue
                i = grid.index(cx, cy)
                if grid.blocked[i] or grid.kappa[i] > cfg.max_goal_kappa:
                    continue
                if math.isinf(reach_cost[i]):
                    continue
                wx, wy = grid.cell_to_world(cx, cy)
                d = math.hypot(wx - tx, wy - ty)
                if d > cfg.approach_search_radius:
                    continue
                if not grid.line_of_sight(i, tidx, walls_only=True):
                    continue
                key = (round(abs(d - standoff) / grid.resolution), reach_cost[i])
                if best_key is None or key < best_key:
                    best_key, best = key, (i, wx, wy)
        if best is None:
            return None, None
        i, wx, wy = best
        yaw = math.atan2(ty - wy, tx - wx) if (wx, wy) != (tx, ty) else 0.0
        return i, Pose2D(wx, wy, yaw)

    # ------------------------------------------------------------ main
    def plan(self, grid, robot, objects, exits):
        """Plan the delivery of `objects` from the robot pose `robot`.

        Args:
            grid: CostGrid (the global costmap).
            robot: Pose2D of the robot in the map frame.
            objects: ObjectTask list (only objects still to deliver).
            exits: ExitInfo list (every exit found so far).
        Returns:
            PlanResult.
        """
        t_start = _time.monotonic()
        problem = self.prepare(grid, robot, objects, exits)
        if isinstance(problem, PlanResult):   # nothing to plan, or it cannot be planned
            problem.planning_time = _time.monotonic() - t_start
            return problem
        result = self.solve(problem)
        result.planning_time = _time.monotonic() - t_start
        return result

    def prepare(self, grid, robot, objects, exits):
        """Steps 1-3: places and leg times. Returns a Problem, or a PlanResult if
        there is nothing that can be planned."""
        cfg = self.config
        tm = cfg.time_model

        if not objects:
            return PlanResult(NOTHING_TO_DO, 'No object is waiting or carried.',
                              capacity=cfg.capacity)

        # 1. Where can the robot go? Snap its position to a free cell (it may be
        #    slightly inside the inflation near a wall) and search the whole grid.
        start_idx = grid.nearest_free(robot.x, robot.y, cfg.robot_snap_distance)
        if start_idx is None:
            return PlanResult(NO_PLAN, 'The robot is not on a free costmap cell '
                              f'(none within {cfg.robot_snap_distance} m).',
                              capacity=cfg.capacity)
        reach_cost, _ = dijkstra(grid, start_idx, cfg.cost_travel_multiplier)

        # 2. Places. Place 0 is the robot. Every object waiting to be picked up
        #    gets a pickup place; every exit used gets a drop-off place.
        place_cells = [start_idx]
        place_poses = [robot]
        exit_by_id = {e.id: e for e in exits}
        exit_place = {}         # exit id -> place index
        planned, unplanned = [], []
        pickup_of, drop_of = [], []

        for obj in objects:
            ex = exit_by_id.get(obj.exit_id)
            if ex is None:
                unplanned.append((obj.id, f'exit "{obj.exit_id}" has not been found'))
                continue
            if obj.exit_id not in exit_place:
                cell, pose = self._approach_pose(grid, reach_cost, ex.x, ex.y,
                                                 cfg.exit_standoff)
                if cell is None:
                    exit_place[obj.exit_id] = None
                else:
                    exit_place[obj.exit_id] = len(place_cells)
                    place_cells.append(cell)
                    place_poses.append(pose)
            if exit_place[obj.exit_id] is None:
                unplanned.append((obj.id, f'exit "{obj.exit_id}" cannot be reached'))
                continue
            if obj.carried:
                pickup = None
            else:
                cell, pose = self._approach_pose(grid, reach_cost, obj.x, obj.y,
                                                 cfg.pickup_standoff)
                if cell is None:
                    unplanned.append((obj.id, 'the object cannot be reached'))
                    continue
                pickup = len(place_cells)
                place_cells.append(cell)
                place_poses.append(pose)
            planned.append(obj)
            pickup_of.append(pickup)
            drop_of.append(exit_place[obj.exit_id])

        if not planned:
            reasons = '; '.join(f'{oid}: {why}' for oid, why in unplanned)
            return PlanResult(NO_PLAN, f'No object can be delivered ({reasons}).',
                              capacity=cfg.capacity, unplanned=unplanned)

        # 3. Legs: driving time between every pair of places. One Dijkstra per
        #    place gives the routes to all the others; each route is smoothed and
        #    timed. legs[a][b] = (LegEstimate, [(x, y), ...]).
        n = len(place_cells)
        legs = [[None] * n for _ in range(n)]
        for a in range(n):
            cost, parent = dijkstra(grid, place_cells[a], cfg.cost_travel_multiplier,
                                    targets=place_cells)
            for b in range(n):
                cells = extract_path(parent, place_cells[a], place_cells[b])
                if cells is None:
                    continue  # not reachable (cannot happen if both are reachable from the robot)
                points = smooth_path(grid, cells, cfg.smoothing_max_kappa)
                # The route starts and ends at the exact poses, not cell centres.
                points[0] = (place_poses[a].x, place_poses[a].y)
                points[-1] = (place_poses[b].x, place_poses[b].y)
                legs[a][b] = (tm.leg(points, place_poses[a].yaw, place_poses[b].yaw), points)

        leg_time = [[legs[a][b][0].time if legs[a][b] else math.inf for b in range(n)]
                    for a in range(n)]
        solver_input = SolverInput(
            start=0, pickup=pickup_of, drop=drop_of, leg_time=leg_time,
            pick_time=tm.pick_time, place_time=tm.place_time, capacity=cfg.capacity)
        return Problem(planned, unplanned, place_poses, legs, solver_input)

    def solve(self, problem):
        """Steps 4-5: order the deliveries and write out the steps."""
        cfg = self.config
        tm = cfg.time_model
        planned, unplanned, legs = problem.objects, problem.unplanned, problem.legs
        place_poses = problem.place_poses
        pickup_of, drop_of = problem.solver_input.pickup, problem.solver_input.drop

        # 4. Order the deliveries.
        solution = DeliverySolver(problem.solver_input,
                                  max_exact_objects=cfg.max_exact_objects).solve()
        if math.isinf(solution.total_time):
            return PlanResult(NO_PLAN, 'Some places cannot be reached from each other.',
                              capacity=cfg.capacity, unplanned=unplanned)

        # 5. Steps with goal poses and estimated times.
        steps, clock, distance, here = [], 0.0, 0.0, 0

        def drive(action, trip, to, object_id='', exit_id=''):
            nonlocal clock, distance, here
            est, points = legs[here][to]
            steps.append(Step(action, trip, object_id, exit_id, place_poses[to],
                              est.distance, est.time, clock, points))
            clock += est.time
            distance += est.distance
            here = to

        def work(action, trip, object_id, exit_id, duration):
            nonlocal clock
            steps.append(Step(action, trip, object_id, exit_id, place_poses[here],
                              0.0, duration, clock, []))
            clock += duration

        for t, trip in enumerate(solution.trips):
            for k in trip.picks:
                obj = planned[k]
                drive(NAVIGATE_TO_PICKUP, t, pickup_of[k], object_id=obj.id)
                work(PICK, t, obj.id, '', tm.pick_time)
            for place, objs in trip.drops:
                exit_id = planned[objs[0]].exit_id
                drive(NAVIGATE_TO_EXIT, t, place, exit_id=exit_id)
                for k in objs:
                    work(PLACE, t, planned[k].id, exit_id, tm.place_time)

        status = PARTIAL if unplanned else OK
        msg = (f'{len(planned)} object(s) in {len(solution.trips)} trip(s), '
               f'about {clock:.0f} s and {distance:.1f} m ({solution.solver} solver).')
        if unplanned:
            msg += f' Left out: {", ".join(oid for oid, _ in unplanned)}.'
        return PlanResult(status, msg, solution.solver, cfg.capacity, steps, clock,
                          distance, 0.0, unplanned)
