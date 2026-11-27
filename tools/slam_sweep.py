#!/usr/bin/env python3
"""Compare slam_toolbox configs by re-running SLAM on several recorded runs.

Runs tools/slam_rerun.py for every combination of config x run x odometry x
scan offset, in parallel, and prints each config's average error against the
true pose. One re-run is not enough to judge a config: holding back the first
scan or two (--skips) changes which scans SLAM adds, and that alone moved one
run's mean error between 3.4 and 5.5 cm. Averaging over offsets and runs gives
a score that reflects the parameters rather than luck.

Configs are given as NAME:KEY=VALUE,KEY=VALUE (overrides of --params);
"base" (no overrides) is always included.

Usage (source the workspace first):
    python3 tools/slam_sweep.py ~/maze_runs/<run1> ~/maze_runs/<run2> \\
        --config fine:resolution=0.025 \\
        --config smear:correlation_search_space_smear_deviation=0.05 \\
        --odom ekf wheels --skips 0 0.2 0.4
"""
import argparse
import concurrent.futures
import json
import os
import statistics
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))


def parse_config(text):
    name, sep, body = text.partition(':')
    if not sep or not name:
        sys.exit(f'--config needs NAME:KEY=VALUE[,KEY=VALUE...], got {text!r}')
    return name, [kv for kv in body.split(',') if kv]


def run_one(job, out_dir, params, rate):
    name, sets, run, odom, skip = job
    out = os.path.join(out_dir, f'{name}_{os.path.basename(run.rstrip("/"))}_{odom}_{skip}.json')
    cmd = [sys.executable, os.path.join(HERE, 'slam_rerun.py'), run, '--odom', odom,
           '--skip', str(skip), '--rate', str(rate), '--json', out, '--label', name]
    if params:
        cmd += ['--params', params]
    for kv in sets:
        cmd += ['--set', kv]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0 or not os.path.exists(out):
        return job, None, (proc.stderr or proc.stdout).strip().splitlines()[-1:] or ['failed']
    with open(out) as f:
        return job, json.load(f), None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='run folders to re-run SLAM on')
    ap.add_argument('--config', action='append', default=[], metavar='NAME:KEY=VALUE,...')
    ap.add_argument('--params', help='base params file (default: slam_rerun.py default)')
    ap.add_argument('--odom', nargs='+', choices=('ekf', 'wheels'), default=['ekf'])
    ap.add_argument('--skips', nargs='+', type=float, default=[0.0, 0.2, 0.4])
    ap.add_argument('--rate', type=float, default=4.0, help='playback speed (default 4)')
    ap.add_argument('--jobs', type=int, default=max(1, (os.cpu_count() or 2) // 2),
                    help='re-runs at a time (default: half the CPUs)')
    ap.add_argument('--out', help='folder for per-run JSON results (default: a temp folder)')
    ap.add_argument('--no-base', action='store_true', help='leave out the unmodified config')
    args = ap.parse_args()

    configs = ([] if args.no_base else [('base', [])]) + [parse_config(c) for c in args.config]
    runs = [os.path.abspath(os.path.expanduser(r)) for r in args.runs]
    out_dir = args.out or tempfile.mkdtemp(prefix='slam_sweep_')
    os.makedirs(out_dir, exist_ok=True)
    jobs = [(n, s, r, o, k) for n, s in configs for r in runs for o in args.odom for k in args.skips]
    print(f'{len(jobs)} re-runs, {args.jobs} at a time; results in {out_dir}', flush=True)

    results, failures = {}, []
    with concurrent.futures.ThreadPoolExecutor(args.jobs) as pool:
        futures = [pool.submit(run_one, j, out_dir, args.params, args.rate) for j in jobs]
        for i, fut in enumerate(concurrent.futures.as_completed(futures), 1):
            job, res, err = fut.result()
            if res is None:
                failures.append((job, err))
            else:
                results.setdefault((job[0], job[3]), []).append(res)
            print(f'\r  {i}/{len(jobs)} done', end='', flush=True)
    print()

    print(f'\n{"config":24s} {"odom":6s} {"n":>3s} {"mean cm":>8s} {"max cm":>7s} '
          f'{"worst":>6s} {"max yaw°":>8s} {"worst°":>7s}   per run (mean of max cm)')
    for name, sets in configs:
        for odom in args.odom:
            rs = results.get((name, odom), [])
            if not rs:
                print(f'{name:24s} {odom:6s}   0  (all failed)')
                continue
            per_run = {}
            for r in rs:
                per_run.setdefault(r['run'], []).append(r['max_cm'])
            runs_txt = '  '.join(f'{statistics.mean(v):.1f}' for _, v in sorted(per_run.items()))
            print(f'{name:24s} {odom:6s} {len(rs):3d} '
                  f'{statistics.mean(r["mean_cm"] for r in rs):8.2f} '
                  f'{statistics.mean(r["max_cm"] for r in rs):7.2f} '
                  f'{max(r["max_cm"] for r in rs):6.1f} '
                  f'{statistics.mean(r["max_deg"] for r in rs):8.2f} '
                  f'{max(r["max_deg"] for r in rs):7.2f}   {runs_txt}')
    odom_rows = {}
    for (name, odom), rs in results.items():
        for r in rs:
            odom_rows.setdefault((odom, r['run']), r['odom_max_cm'])
    print('\nodometry alone, max cm: ' + '  '.join(
        f'{odom}/{run[:19]}={v:.1f}' for (odom, run), v in sorted(odom_rows.items())))
    print('runs: ' + '  '.join(sorted({os.path.basename(r) for r in runs})))
    for job, err in failures:
        print(f'FAILED {job[0]} {os.path.basename(job[2])} {job[3]} skip {job[4]}: {" ".join(err)}')


if __name__ == '__main__':
    main()
