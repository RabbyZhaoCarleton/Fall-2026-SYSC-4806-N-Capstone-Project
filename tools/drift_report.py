#!/usr/bin/env python3
"""Measure how far each localization estimate drifted from the simulator's true pose.

Needs a run recorded with /ground_truth/poses (maze.launch.py bridges it from
Gazebo; runs recorded before that topic existed can't be measured). Compares,
against the truth:
    wheels  /odom                      wheel odometry alone
    ekf     /odometry/filtered         EKF (wheel speed + IMU gyro), if it ran
    slam    map->odom + odom->base     the pose Nav2 and explore_lite use

Usage (source the workspace first):
    python3 tools/drift_report.py ~/maze_runs/<run>
    python3 tools/drift_report.py ~/maze_runs/<run> --every 2

All poses are expressed in the frame the robot started in, which is where the
odom and map frames begin.
"""
import argparse
import bisect
import math
import os
import sys

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    sys.exit(f'{e}\nSource ROS first: source /opt/ros/jazzy/setup.bash')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    """Pose b expressed in a's parent frame, for 2D poses (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class Series:
    """Time-sorted 2D poses, sampled by linear interpolation."""

    def __init__(self):
        self.t, self.p = [], []

    def add(self, t, pose):
        self.t.append(t)
        self.p.append(pose)

    def finish(self):
        order = sorted(range(len(self.t)), key=self.t.__getitem__)
        self.t = [self.t[i] for i in order]
        self.p = [self.p[i] for i in order]
        return self

    def at(self, t):
        """Pose at time t, or None outside the recorded span."""
        if not self.t or t < self.t[0] or t > self.t[-1]:
            return None
        i = bisect.bisect_left(self.t, t)
        if self.t[i] == t or i == 0:
            return self.p[i]
        t0, t1, a, b = self.t[i - 1], self.t[i], self.p[i - 1], self.p[i]
        w = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return (a[0] + w * (b[0] - a[0]), a[1] + w * (b[1] - a[1]), wrap(a[2] + w * wrap(b[2] - a[2])))


def read_run(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    s = {k: Series() for k in ('truth', 'wheels', 'ekf', 'map_odom', 'odom_base')}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic not in ('/ground_truth/poses', '/odom', '/odometry/filtered', '/tf'):
            continue
        msg = deserialize_message(raw, get_message(types[topic]))
        if topic == '/ground_truth/poses':
            # poses[0] is the robot model (base_footprint in world); the rest are its links.
            if msg.poses:
                p = msg.poses[0]
                s['truth'].add(stamp(msg.header), (p.position.x, p.position.y, yaw_of(p.orientation)))
            continue
        if topic in ('/odom', '/odometry/filtered'):
            p = msg.pose.pose
            s['wheels' if topic == '/odom' else 'ekf'].add(
                stamp(msg.header), (p.position.x, p.position.y, yaw_of(p.orientation)))
            continue
        for tr in msg.transforms:
            pose = (tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation))
            parent, child = tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/')
            if (parent, child) == ('map', 'odom'):
                s['map_odom'].add(stamp(tr.header), pose)
            elif (parent, child) == ('odom', 'base_footprint'):
                s['odom_base'].add(stamp(tr.header), pose)
    return {k: v.finish() for k, v in s.items()}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run', help='run folder (contains bag/) or the bag folder itself')
    ap.add_argument('--every', type=float, default=5.0, help='seconds between table rows (default 5)')
    args = ap.parse_args()

    run = os.path.expanduser(args.run)
    bag_dir = os.path.join(run, 'bag') if os.path.isdir(os.path.join(run, 'bag')) else run
    s = read_run(bag_dir)
    truth = s['truth']
    if len(truth.t) < 2:
        sys.exit('No /ground_truth/poses in this run: it was recorded before maze.launch.py '
                 'bridged the simulator\'s true pose.')

    # Truth in the robot's starting frame, which is where odom and map begin.
    origin_inv = inverse(truth.p[0])
    estimates = {
        'wheels': lambda t: s['wheels'].at(t),
        'ekf': lambda t: s['ekf'].at(t),
    }
    if s['map_odom'].t:
        def slam(t):
            mo, ob = s['map_odom'].at(t), s['odom_base'].at(t)
            if ob is None:
                return None
            # Before slam_toolbox's first correction, map and odom coincide.
            return compose(mo if mo is not None else (0.0, 0.0, 0.0), ob)
        estimates['slam'] = slam
    names = [n for n in estimates if any(estimates[n](t) is not None for t in truth.t[::50])]

    stats = {n: {'max_pos': 0.0, 'max_yaw': 0.0, 'sum_pos': 0.0, 'count': 0, 'last': None} for n in names}
    rows, travelled, prev, next_row = [], 0.0, None, truth.t[0]
    for t, p in zip(truth.t, truth.p):
        true = compose(origin_inv, p)
        if prev is not None:
            travelled += math.hypot(true[0] - prev[0], true[1] - prev[1])
        prev = true
        errs = {}
        for n in names:
            e = estimates[n](t)
            if e is None:
                continue
            pos, yaw = math.hypot(e[0] - true[0], e[1] - true[1]), abs(wrap(e[2] - true[2]))
            errs[n] = (pos, yaw)
            st = stats[n]
            st['max_pos'], st['max_yaw'] = max(st['max_pos'], pos), max(st['max_yaw'], yaw)
            st['last'] = (pos, yaw)
            st['sum_pos'] += pos
            st['count'] += 1
        if t >= next_row:
            rows.append((t, travelled, true, errs))
            next_row = t + args.every

    print(f'Run: {run}')
    print(f'Sim time {truth.t[0]:.1f}-{truth.t[-1]:.1f} s, robot travelled {travelled:.2f} m (true path)\n')
    head = ' '.join(f'{n + " pos cm":>12} {n + " yaw°":>10}' for n in names)
    print(f'{"t (s)":>6} {"dist m":>7} {"true x":>7} {"true y":>7} {"yaw°":>6}  {head}')
    for t, dist, true, errs in rows:
        cols = ' '.join(f'{errs[n][0] * 100:12.1f} {math.degrees(errs[n][1]):10.1f}' if n in errs
                        else f'{"-":>12} {"-":>10}' for n in names)
        print(f'{t:6.1f} {dist:7.2f} {true[0]:7.3f} {true[1]:7.3f} {math.degrees(true[2]):6.1f}  {cols}')
    print('\nError against the true pose:')
    for n in names:
        st = stats[n]
        per_m = f', {st["last"][0] / travelled * 100:.1f}% of distance travelled' if travelled > 0.05 else ''
        print(f'  {n:6s}  mean {st["sum_pos"] / st["count"] * 100:5.1f} cm   '
              f'final {st["last"][0] * 100:5.1f} cm, {math.degrees(st["last"][1]):5.1f}°   '
              f'max {st["max_pos"] * 100:5.1f} cm, {math.degrees(st["max_yaw"]):5.1f}°{per_m}')


if __name__ == '__main__':
    main()
