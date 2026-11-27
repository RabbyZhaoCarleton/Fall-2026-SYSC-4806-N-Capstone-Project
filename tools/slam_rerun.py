#!/usr/bin/env python3
"""Re-run slam_toolbox on a recorded run and score it against the simulator's true pose.

A recorded run already holds everything slam_toolbox read live: /scan, the
odom->base_footprint transform, /tf_static and /clock. This tool plays those
back into a fresh slam_toolbox (with the parameters you choose), drops the
run's own map->odom so the new SLAM publishes its own, and compares the
resulting pose with /ground_truth/poses, the same way tools/drift_report.py
does.

Odometry input (--odom):
    ekf     odom->base_footprint as recorded (the EKF, if the run used it)
    wheels  odom->base_footprint rebuilt from /odom (gz wheel odometry alone).
            A config that only looks good with the EKF (because SLAM stopped
            correcting and just passes odometry through) fails here.

Each re-run uses its own ROS_DOMAIN_ID, so it never sees a live simulation and
several can run at the same time. Playback is paced by the recorded /clock;
--rate 2 replays at twice the recorded speed.

Usage (source the workspace first):
    python3 tools/slam_rerun.py ~/maze_runs/<run>
    python3 tools/slam_rerun.py ~/maze_runs/<run> --odom wheels \\
        --set resolution=0.025 --set correlation_search_space_smear_deviation=0.05
    python3 tools/slam_rerun.py ~/maze_runs/<run> --json result.json
    python3 tools/slam_rerun.py ~/maze_runs/<run> --record ~/maze_runs

--record writes the re-run as a new run folder (<run>_rerun_<time>) that the
maze monitor can open. Its bag holds the original run's topics with SLAM's
outputs (map->odom, /map, /pose) replaced by the re-run's (the original
/map_metadata and SLAM graph visualization are left out); everything else (Nav2, explore_lite) is what happened live, so it was based
on the original SLAM, not this one.
"""
import argparse
import bisect
import datetime
import json
import math
import os
import random
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drift_report import Series, compose, inverse, read_run, wrap, yaw_of  # noqa: E402

DEFAULT_PARAMS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                              'maze_navigation', 'config', 'slam_maze.yaml')
PLAYED = ('/clock', '/scan', '/tf', '/tf_static', '/odom')
# Topics slam_toolbox publishes; --record replaces the original run's copies.
SLAM_OUTPUTS = ('/map', '/map_metadata', '/pose', '/slam_toolbox/graph_visualization',
                '/slam_toolbox/scan_visualization')


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def make_params(base_file, overrides, out_file):
    """Write base_file with overrides applied; refuse keys the base file doesn't have."""
    with open(base_file) as f:
        doc = yaml.safe_load(f)
    params = doc['slam_toolbox']['ros__parameters']
    for item in overrides:
        key, sep, value = item.partition('=')
        if not sep:
            sys.exit(f'--set needs key=value, got {item!r}')
        if key not in params:
            sys.exit(f'--set {key}: not a parameter in {base_file} (typo?)')
        new = yaml.safe_load(value)
        # Keep float parameters float: ROS refuses an int for a double parameter.
        if isinstance(params[key], float) and isinstance(new, int) and not isinstance(new, bool):
            new = float(new)
        params[key] = new
    with open(out_file, 'w') as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    return params


def load_inputs(bag_dir, odom_source):
    """Read the messages to play, in recorded order, as (recv_ns, topic, msg)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import TransformStamped

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in ('/clock', '/scan', '/tf', '/tf_static') if t not in types]
    if missing:
        sys.exit(f'Bag lacks {", ".join(missing)}; cannot re-run SLAM on it.')
    if odom_source == 'wheels' and '/odom' not in types:
        sys.exit('Bag has no /odom; --odom wheels is not possible.')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in PLAYED if t in types]))

    out, static = [], {}
    ob = Series()  # odom->base_footprint as fed to SLAM, for scoring
    clock = []     # (sim time, recv_ns), to map sim time back to the recording's timeline
    while reader.has_next():
        topic, raw, ns = reader.read_next()
        msg = deserialize_message(raw, get_message(types[topic]))
        if topic == '/clock':
            clock.append((msg.clock.sec + msg.clock.nanosec * 1e-9, ns))
            out.append((ns, topic, msg))
        elif topic == '/scan':
            out.append((ns, topic, msg))
        elif topic == '/tf_static':
            for tr in msg.transforms:
                static[(tr.header.frame_id, tr.child_frame_id)] = tr
        elif topic == '/tf':
            keep = []
            for tr in msg.transforms:
                pair = (tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))
                if pair == ('map', 'odom'):
                    continue
                if pair == ('odom', 'base_footprint'):
                    if odom_source == 'wheels':
                        continue
                    ob.add(stamp(tr.header), (tr.transform.translation.x, tr.transform.translation.y,
                                              yaw_of(tr.transform.rotation)))
                keep.append(tr)
            if keep:
                msg.transforms = keep
                out.append((ns, topic, msg))
        elif topic == '/odom' and odom_source == 'wheels':
            p = msg.pose.pose
            tr = TransformStamped()
            tr.header = msg.header
            tr.child_frame_id = msg.child_frame_id
            tr.transform.translation.x, tr.transform.translation.y = p.position.x, p.position.y
            tr.transform.translation.z = p.position.z
            tr.transform.rotation = p.orientation
            ob.add(stamp(msg.header), (p.position.x, p.position.y, yaw_of(p.orientation)))
            out.append((ns, '/tf', TFMessage(transforms=[tr])))
    if not static:
        sys.exit('Bag has no /tf_static transforms.')
    return out, TFMessage(transforms=list(static.values())), ob.finish(), clock


def sim_to_recv_ns(clock):
    sims, nss = [c[0] for c in clock], [c[1] for c in clock]

    def convert(t):
        i = bisect.bisect_left(sims, t)
        if i <= 0:
            return nss[0]
        if i >= len(sims):
            return nss[-1] + int((t - sims[-1]) * 1e9)
        s0, s1, n0, n1 = sims[i - 1], sims[i], nss[i - 1], nss[i]
        return n0 + int((n1 - n0) * ((t - s0) / (s1 - s0) if s1 > s0 else 0.0))
    return convert


def score(truth, ob, mo):
    """Pose error of map->odom∘odom->base against the truth, in the start frame."""
    origin_inv = inverse(truth.p[0])
    err_pos, err_yaw, odom_pos, travelled, prev = [], [], [], 0.0, None
    for t, p in zip(truth.t, truth.p):
        o = ob.at(t)
        if o is None:
            continue
        true = compose(origin_inv, p)
        if prev is not None:
            travelled += math.hypot(true[0] - prev[0], true[1] - prev[1])
        prev = true
        m = mo.at(t) if mo.t and t >= mo.t[0] else None
        est = compose(m if m is not None else (0.0, 0.0, 0.0), o)
        err_pos.append(math.hypot(est[0] - true[0], est[1] - true[1]))
        err_yaw.append(abs(wrap(est[2] - true[2])))
        odom_pos.append(math.hypot(o[0] - true[0], o[1] - true[1]))
    if not err_pos:
        return None
    mean = lambda v: sum(v) / len(v)  # noqa: E731
    return {
        'travelled_m': round(travelled, 3),
        'mean_cm': round(100 * mean(err_pos), 2),
        'max_cm': round(100 * max(err_pos), 2),
        'final_cm': round(100 * err_pos[-1], 2),
        'mean_deg': round(math.degrees(mean(err_yaw)), 2),
        'max_deg': round(math.degrees(max(err_yaw)), 2),
        'odom_mean_cm': round(100 * mean(odom_pos), 2),
        'odom_max_cm': round(100 * max(odom_pos), 2),
        'corrections': len(mo.t),
        'max_correction_cm': round(100 * max((math.hypot(p[0], p[1]) for p in mo.p), default=0.0), 2),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run', help='run folder (contains bag/) or the bag folder itself')
    ap.add_argument('--params', default=DEFAULT_PARAMS,
                    help='slam_toolbox params file (default: maze_navigation/config/slam_maze.yaml)')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                    help='override one parameter of --params (repeatable)')
    ap.add_argument('--odom', choices=('ekf', 'wheels'), default='ekf',
                    help='odometry fed to SLAM (default: ekf, as recorded)')
    ap.add_argument('--rate', type=float, default=2.0, help='playback speed (default 2)')
    ap.add_argument('--domain', type=int, default=None,
                    help='ROS_DOMAIN_ID for this re-run (default: a free one in 30-99)')
    ap.add_argument('--json', help='also write the result to this file')
    ap.add_argument('--record', metavar='RUNS_DIR',
                    help='save the re-run as a run folder in RUNS_DIR for the maze monitor')
    ap.add_argument('--log', help='write slam_toolbox output to this file (default: discard)')
    ap.add_argument('--label', default='', help='name for this config in the output')
    ap.add_argument('--skip', type=float, default=0.0, metavar='SECONDS',
                    help='hold back scans for the first SECONDS of sim time. SLAM adds one scan '
                         'every minimum_time_interval, so this changes which scans it uses; '
                         're-running with a few small offsets shows how much a score is luck')
    args = ap.parse_args()

    run = os.path.abspath(os.path.expanduser(args.run))
    bag_dir = os.path.join(run, 'bag') if os.path.isdir(os.path.join(run, 'bag')) else run
    run_dir = run if bag_dir != run else os.path.dirname(run)

    # Pick the domain before rclpy starts: it reads ROS_DOMAIN_ID once.
    domain = args.domain if args.domain is not None else reserve_domain()
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from lifecycle_msgs.srv import ChangeState
    from lifecycle_msgs.msg import Transition
    from ament_index_python.packages import get_package_prefix

    tmp = tempfile.mkdtemp(prefix='slam_rerun_')
    params_file = os.path.join(tmp, 'slam_params.yaml')
    params = make_params(os.path.abspath(os.path.expanduser(args.params)), args.set, params_file)

    t0 = time.monotonic()
    inputs, static_msg, ob, clock = load_inputs(bag_dir, args.odom)
    truth = read_run(bag_dir)['truth']
    if len(truth.t) < 2:
        sys.exit('No /ground_truth/poses in this run; nothing to score against.')
    load_s = time.monotonic() - t0

    exe = os.path.join(get_package_prefix('slam_toolbox'), 'lib', 'slam_toolbox', 'async_slam_toolbox_node')
    log = open(args.log, 'w') if args.log else subprocess.DEVNULL
    slam = subprocess.Popen(
        [exe, '--ros-args', '-r', '__node:=slam_toolbox', '--params-file', params_file,
         '-p', 'use_sim_time:=true', '-p', 'use_lifecycle_manager:=false'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    rclpy.init()
    node = rclpy.create_node('slam_rerun_player', parameter_overrides=[])
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    pubs = {
        '/clock': node.create_publisher(Clock, '/clock', 10),
        '/scan': node.create_publisher(LaserScan, '/scan', 10),
        '/tf': node.create_publisher(TFMessage, '/tf', 100),
    }
    static_pub = node.create_publisher(TFMessage, '/tf_static', latched)

    mo = Series()
    outputs = []  # (sim time, topic, msg) published by the new SLAM
    lock = threading.Lock()

    def on_tf(msg):
        for tr in msg.transforms:
            if tr.header.frame_id.lstrip('/') == 'map' and tr.child_frame_id.lstrip('/') == 'odom':
                with lock:
                    mo.add(stamp(tr.header), (tr.transform.translation.x, tr.transform.translation.y,
                                              yaw_of(tr.transform.rotation)))
                    if args.record:
                        outputs.append((stamp(tr.header), '/tf', TFMessage(transforms=[tr])))

    node.create_subscription(TFMessage, '/tf', on_tf, 1000)
    scan_stamps = []  # one /pose per scan slam_toolbox added to its graph

    def on_pose(msg):
        with lock:
            scan_stamps.append(stamp(msg.header))
            if args.record:
                outputs.append((stamp(msg.header), '/pose', msg))
    node.create_subscription(PoseWithCovarianceStamped, '/pose', on_pose, 100)
    if args.record:
        def keep(topic):
            def cb(msg):
                with lock:
                    outputs.append((stamp(msg.header), topic, msg))
            return cb
        node.create_subscription(OccupancyGrid, '/map', keep('/map'), latched)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    def fail(text):
        node.get_logger().error(text)
        stop_slam(slam)
        sys.exit(text)

    def transition(client, tid):
        future = client.call_async(ChangeState.Request(transition=Transition(id=tid)))
        deadline = time.monotonic() + 15
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.done() and future.result().success

    try:
        # The recorded first /clock, so SLAM has a time before it configures.
        first_clock = next(m for _, t, m in inputs if t == '/clock')
        static_pub.publish(static_msg)
        pubs['/clock'].publish(first_clock)

        client = node.create_client(ChangeState, '/slam_toolbox/change_state')
        if not client.wait_for_service(timeout_sec=20):
            fail('slam_toolbox did not start (no /slam_toolbox/change_state service)')
        if not transition(client, Transition.TRANSITION_CONFIGURE):
            fail('slam_toolbox configure failed')
        if not transition(client, Transition.TRANSITION_ACTIVATE):
            fail('slam_toolbox activate failed')
        deadline = time.monotonic() + 10
        while pubs['/scan'].get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        if pubs['/scan'].get_subscription_count() == 0:
            fail('slam_toolbox never subscribed to /scan')

        # Play back, paced by the recorded /clock.
        sim0, wall0 = None, time.monotonic()
        for _, topic, msg in inputs:
            if topic == '/clock':
                sim = msg.clock.sec + msg.clock.nanosec * 1e-9
                if sim0 is None:
                    sim0 = sim
                delay = wall0 + (sim - sim0) / args.rate - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            elif topic == '/scan' and stamp(msg.header) < sim0 + args.skip:
                continue
            pubs[topic].publish(msg)
        play_s = time.monotonic() - wall0
        time.sleep(2.0)  # let SLAM finish the scans it has queued
    finally:
        stop_slam(slam)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

    with lock:
        mo.finish()
    result = score(truth, ob, mo)
    if result is None:
        sys.exit('No overlap between odometry and ground truth; nothing to score.')
    if not mo.t:
        print('WARNING: slam_toolbox published no map->odom; the score is odometry alone.',
              file=sys.stderr)
    result.update({
        'label': args.label,
        'run': os.path.basename(run_dir.rstrip('/')),
        'odom': args.odom,
        'rate': args.rate,
        'skip': args.skip,
        'overrides': dict(s.partition('=')[::2] for s in args.set),
        'scans_added': len(scan_stamps),
        'scan_stamps': [round(t, 3) for t in scan_stamps],
        'play_s': round(play_s, 1),
        'load_s': round(load_s, 1),
    })

    print(f"{result['run']}  odom={args.odom}  {' '.join(args.set) or '(base params)'}")
    print(f"  slam   mean {result['mean_cm']:5.1f} cm  max {result['max_cm']:5.1f} cm  "
          f"final {result['final_cm']:5.1f} cm   yaw mean {result['mean_deg']:4.2f}°  max {result['max_deg']:4.2f}°")
    print(f"  odom   mean {result['odom_mean_cm']:5.1f} cm  max {result['odom_max_cm']:5.1f} cm   "
          f"({result['corrections']} map->odom msgs, largest correction "
          f"{result['max_correction_cm']:.1f} cm, {len(scan_stamps)} scans added, true path {result['travelled_m']:.2f} m)")
    if args.json:
        with open(os.path.expanduser(args.json), 'w') as f:
            json.dump(result, f, indent=2)
    if args.record:
        path = record(run_dir, bag_dir, os.path.expanduser(args.record), params_file, args,
                      outputs, sim_to_recv_ns(clock), result)
        print(f'  recorded {path}')
    shutil.rmtree(tmp, ignore_errors=True)


_domain_lock = None


def reserve_domain():
    """A ROS_DOMAIN_ID (30-99) that no other slam_rerun on this machine holds.

    The lock file stays locked until this process exits, so parallel re-runs
    never share a domain (they would see each other's /tf and /scan).
    """
    global _domain_lock
    import fcntl
    domains = list(range(30, 100))
    random.shuffle(domains)
    for d in domains:
        f = open(os.path.join(tempfile.gettempdir(), f'slam_rerun_domain_{d}.lock'), 'w')
        try:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            f.close()
            continue
        _domain_lock = f
        return d
    sys.exit('No free ROS_DOMAIN_ID in 30-99 (70 re-runs already running?)')


def stop_slam(proc):
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=10)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()


def record(run_dir, bag_dir, runs_dir, params_file, args, outputs, to_ns, result):
    """Copy the run with SLAM's outputs swapped for the re-run's, as a new run folder."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message

    name = f"{os.path.basename(run_dir.rstrip('/'))}_rerun_{datetime.datetime.now():%H-%M-%S}"
    out_dir = os.path.join(runs_dir, name)
    os.makedirs(out_dir)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = reader.get_all_topics_and_types()
    types = {t.name: t.type for t in topics}
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=os.path.join(out_dir, 'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for i, t in enumerate(topics):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=i, name=t.name, type=t.type, serialization_format='cdr',
            offered_qos_profiles=t.offered_qos_profiles))

    # Merge the original messages (minus SLAM's) with the re-run's, in time order.
    new = sorted(((to_ns(t), topic, msg) for t, topic, msg in outputs), key=lambda x: x[0])
    k = 0
    while reader.has_next():
        topic, raw, ns = reader.read_next()
        while k < len(new) and new[k][0] <= ns:
            if new[k][1] in types:
                writer.write(new[k][1], serialize_message(new[k][2]), new[k][0])
            k += 1
        if topic in SLAM_OUTPUTS:
            continue
        if topic == '/tf':
            msg = deserialize_message(raw, get_message(types[topic]))
            keep = [tr for tr in msg.transforms
                    if (tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/')) != ('map', 'odom')]
            if not keep:
                continue
            if len(keep) != len(msg.transforms):
                msg.transforms = keep
                raw = serialize_message(msg)
        writer.write(topic, raw, ns)
    for ns, topic, msg in new[k:]:
        if topic in types:
            writer.write(topic, serialize_message(msg), ns)
    del writer

    with open(os.path.join(run_dir, 'run.json')) as f:
        meta = json.load(f)
    shutil.copy(params_file, os.path.join(out_dir, 'slam_params.yaml'))
    meta.update({
        'name': name,
        'label': 'rerun',
        'summary': {},
        'starred': False,
        'notes': (f"SLAM re-run of {os.path.basename(run_dir.rstrip('/'))} with odom={args.odom}, "
                  f"overrides: {' '.join(args.set) or 'none'}. Only map->odom, /map and /pose come "
                  f"from the re-run; Nav2 and explore_lite messages are from the original run. "
                  f"Score vs truth: mean {result['mean_cm']} cm, max {result['max_cm']} cm, "
                  f"max yaw {result['max_deg']} deg."),
    })
    meta.setdefault('launch_args', {})['slam_params'] = os.path.join(out_dir, 'slam_params.yaml')
    with open(os.path.join(out_dir, 'run.json'), 'w') as f:
        json.dump(meta, f, indent=2, sort_keys=True)
    return out_dir


if __name__ == '__main__':
    main()
