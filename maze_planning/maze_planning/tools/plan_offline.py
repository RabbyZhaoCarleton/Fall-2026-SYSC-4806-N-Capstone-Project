"""plan_offline: run the global planner on a maze drawing, with no ROS and no simulator.

    # Plan a scenario and draw it (open the .svg in a browser):
    ros2 run maze_planning plan_offline --layout layouts/three_exits_3m.txt \
        --scenario config/scenarios/three_exits_3m.yaml --svg plan.svg

    # Proposal test 4.2.5 (route optimisation on 20+ random mazes): plan on random
    # mazes and check every plan against brute force (every possible order).
    ros2 run maze_planning plan_offline --benchmark 20

Works without sourcing ROS too:  python3 -m maze_planning.tools.plan_offline ...
"""

import argparse
import math
import random
import sys
import time

import yaml

from ..core.maze_layout import layout_costmap, load_layout, parse_layout, random_layout
from ..core.planner import (ACTION_NAMES, NAVIGATE_TO_EXIT, NAVIGATE_TO_PICKUP, PICK, PLACE,
                            ExitInfo, GlobalPlanner, ObjectTask, PlannerConfig, Pose2D)
from ..core.solver import brute_force_time

COLOURS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']


def scenario_inputs(scenario):
    """Objects, exits and start pose from a scenario YAML (see config/scenarios/)."""
    classes = scenario['classes']
    objects = [ObjectTask(o['id'], classes[o['class']], float(o['x']), float(o['y']),
                          object_class=o['class']) for o in scenario['objects']]
    exits = [ExitInfo(eid, float(e['x']), float(e['y'])) for eid, e in scenario['exits'].items()]
    s = scenario.get('start', {'x': 0.25, 'y': -0.25, 'yaw': 0.0})
    return objects, exits, Pose2D(float(s['x']), float(s['y']), float(s.get('yaw', 0.0)))


def print_plan(result):
    print(result.message)
    print(f'{"#":>3}  {"trip":>4}  {"action":<19} {"what":<10} {"goal (x, y, yaw)":<24}'
          f'{"dist":>6} {"time":>7} {"starts":>7}')
    for k, s in enumerate(result.steps, 1):
        what = s.object_id if s.action in (NAVIGATE_TO_PICKUP, PICK) else \
            (s.exit_id if s.action == NAVIGATE_TO_EXIT else f'{s.object_id}>{s.exit_id}')
        goal = f'({s.target.x:5.2f}, {s.target.y:5.2f}, {math.degrees(s.target.yaw):4.0f})'
        print(f'{k:>3}  {s.trip:>4}  {ACTION_NAMES[s.action]:<19} {what:<10} {goal:<24}'
              f'{s.distance:5.2f}m {s.duration:6.1f}s {s.start_time:6.1f}s')
    for oid, why in result.unplanned:
        print(f'  not planned: {oid}: {why}')


def write_svg(path, layout, result, objects, exits, robot, scale=160):
    """Draw the maze, exits, objects and each trip's route (one colour per trip)."""
    w, h = layout.cols * layout.cell, layout.rows * layout.cell
    pad = 0.4

    def px(x, y):
        return ((x + pad) * scale, (-y + pad) * scale)

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{(w + 2 * pad) * scale:.0f}" '
           f'height="{(h + 2 * pad) * scale + 30:.0f}" font-family="sans-serif">',
           '<rect width="100%" height="100%" fill="white"/>']
    for (x0, y0), (x1, y1) in layout.walls:
        a, b = px(x0, y0), px(x1, y1)
        out.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" '
                   'stroke="#334" stroke-width="4" stroke-linecap="round"/>')
    for e in exits:
        x, y = px(e.x, e.y)
        out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="9" fill="#2a2"/>'
                   f'<text x="{x + 12:.1f}" y="{y - 10:.1f}" font-size="14" fill="#2a2">'
                   f'{e.id}</text>')
    for o in objects:
        x, y = px(o.x, o.y)
        out.append(f'<rect x="{x - 4:.1f}" y="{y - 4:.1f}" width="8" height="8" fill="#db2"/>')
    for s in result.steps:
        if s.action in (NAVIGATE_TO_PICKUP, NAVIGATE_TO_EXIT) and len(s.path) > 1:
            pts = ' '.join('%.1f,%.1f' % px(x, y) for x, y in s.path)
            c = COLOURS[s.trip % len(COLOURS)]
            # Trips are offset a little so overlapping routes stay visible.
            off = 3 * (s.trip % 4)
            out.append(f'<polyline points="{pts}" fill="none" stroke="{c}" stroke-width="3" '
                       f'stroke-opacity="0.8" transform="translate({off},{off})"/>')
    n = 0
    for s in result.steps:
        if s.action == PLACE:
            n += 1
            x, y = px(s.target.x, s.target.y)
            c = COLOURS[s.trip % len(COLOURS)]
            out.append(f'<text x="{x + 6 + 12 * (n % 3):.1f}" y="{y + 16:.1f}" font-size="13" '
                       f'fill="{c}">{n}:{s.object_id}</text>')
    x, y = px(robot.x, robot.y)
    out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="7" fill="none" stroke="#00f" '
               'stroke-width="3"/>')
    out.append(f'<text x="8" y="{(h + 2 * pad) * scale + 20:.0f}" font-size="14">'
               f'{result.message}</text></svg>')
    with open(path, 'w') as f:
        f.write('\n'.join(out))


def benchmark(args):
    """Random mazes: plan each one and compare with brute force."""
    rng = random.Random(args.seed)
    cfg = PlannerConfig(capacity=args.capacity)
    worst_ratio, times, failures = 1.0, [], 0
    for k in range(args.benchmark):
        text = random_layout(args.rows, args.cols, args.exits, rng,
                             extra_openings=rng.randint(0, 4))
        layout = parse_layout(text)
        grid, _, _ = layout_costmap(layout)
        exits = [ExitInfo(f'exit_{i}', x, y) for i, (x, y) in enumerate(layout.exits)]
        # Objects at the entrance (start cell), each assigned a random exit.
        objects = [ObjectTask(f'obj_{i}', rng.choice(exits).id,
                              0.25 + rng.uniform(-0.05, 0.05), -0.40 + rng.uniform(-0.04, 0.04))
                   for i in range(args.objects)]
        robot = Pose2D(0.25, -0.25, 0.0)
        planner = GlobalPlanner(cfg)
        result = planner.plan(grid, robot, objects, exits)
        times.append(result.planning_time)

        # Rebuild the same leg times the planner used and brute-force them.
        exact = check_against_brute_force(planner, grid, robot, objects, exits)
        ratio = result.total_time / exact if exact > 0 else 1.0
        worst_ratio = max(worst_ratio, ratio)
        ok = abs(ratio - 1.0) < 1e-6
        failures += not ok
        trips = len({s.trip for s in result.steps})
        print(f'maze {k + 1:2d}: {trips:2d} trips, '
              f'plan {result.total_time:7.1f} s, brute force {exact:7.1f} s, '
              f'{"optimal" if ok else "NOT OPTIMAL"}, planned in {result.planning_time:.3f} s')
    print(f'\n{args.benchmark} mazes, {args.objects} objects, {args.exits} exits, '
          f'capacity {args.capacity}: {args.benchmark - failures} optimal, '
          f'worst plan/optimum {worst_ratio:.4f}, planning time mean '
          f'{sum(times) / len(times):.3f} s, max {max(times):.3f} s')
    return 0 if failures == 0 else 1


def check_against_brute_force(planner, grid, robot, objects, exits):
    """Optimal total time of the same problem, by trying every possible plan.

    Uses the planner's own places and leg times (prepare), so it checks only the
    ordering: does the solver really find the fastest sequence?
    """
    problem = planner.prepare(grid, robot, objects, exits)
    return brute_force_time(problem.solver_input)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--layout', help='maze drawing (.txt, make_maze.py format)')
    ap.add_argument('--scenario', help='objects / classes / exits (.yaml)')
    ap.add_argument('--capacity', type=int, default=1, help='objects carried at once')
    ap.add_argument('--svg', help='write a drawing of the plan to this file')
    ap.add_argument('--benchmark', type=int, metavar='N',
                    help='plan N random mazes and check each plan against brute force')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--rows', type=int, default=6)
    ap.add_argument('--cols', type=int, default=6)
    ap.add_argument('--exits', type=int, default=3)
    ap.add_argument('--objects', type=int, default=5)
    args = ap.parse_args(argv)

    if args.benchmark:
        return benchmark(args)
    if not (args.layout and args.scenario):
        ap.error('give --layout and --scenario, or --benchmark N')

    layout = load_layout(args.layout)
    with open(args.scenario) as f:
        objects, exits, robot = scenario_inputs(yaml.safe_load(f))
    grid, _, _ = layout_costmap(layout)
    t = time.monotonic()
    result = GlobalPlanner(PlannerConfig(capacity=args.capacity)).plan(grid, robot, objects, exits)
    print(f'(planned in {time.monotonic() - t:.3f} s)')
    print_plan(result)
    if args.svg:
        write_svg(args.svg, layout, result, objects, exits, robot)
        print(f'Wrote {args.svg}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
