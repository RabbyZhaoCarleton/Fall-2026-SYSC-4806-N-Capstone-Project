#!/usr/bin/env python3
"""LiDAR range-accuracy test at several distances, with a countdown to move the target.

RUNNING THE SCRIPT
    cp "/mnt/c/Users/n_bod/OneDrive/Desktop/Fall-2026-SYSC-4806-N-Capstone-Project/lidar_bringup/launch/rplidar_c1.launch.py" ~/ros2_ws/src/capstone/lidar_bringup/launch/
    cd ~/ros2_ws && colcon build --symlink-install --packages-select lidar_bringup && source install/setup.bash
    # (relaunch the LiDAR, then in a second terminal:)
    python3 "/mnt/c/Users/n_bod/OneDrive/Desktop/Fall-2026-SYSC-4806-N-Capstone-Project/lidar_bringup/scripts/accuracy_steps.py"

WHAT IT DOES
  For each position 1..N:
    1. Counts down (default 10 s) on the terminal while you move the flat board to the next
       distance. Scans that arrive during the countdown are ignored.
    2. Records a number of scans (default 50 = 5 s at 10 Hz). In EVERY scan it fits a straight
       line to the board (the beams within +-window/2 of the test direction) and takes the
       PERPENDICULAR distance from the LiDAR to that line.
    3. Prints a summary for that position and keeps the per-scan values.
  At the end it prints:
    * a summary table (mean, standard deviation, 95 % confidence interval, min/max, board angle)
    * a RAW block with every per-scan value (paste this into the chat for analysis)
    * if you gave the true distances (--truth), the accuracy analysis (see STATISTICS)
  and saves everything to ~/lidar_logs/accuracy_steps_<date>_<time>.csv

WHY THE PERPENDICULAR DISTANCE (not a single beam)
  A single beam reads d / cos(theta) if the board is turned by theta, so a board turned by
  3 deg would add 0.14 % error that is really a set-up error. The line fit uses the ~20-60
  beams on the board, gives the true perpendicular distance whatever the small tilt, and
  also reports the tilt (board angle) so you can see how square it was. The single beam in
  the test direction is recorded too (column `beam_m`) for comparison.

NOTHING IS HARD-CODED: the number of positions, the countdown, the number of scans and the
true distances are all arguments, so the same script calibrates 0.15 m or 10 m.

  python3 accuracy_steps.py                                   # 3 positions, analyse later
  python3 accuracy_steps.py --positions 5 --wait 15 --scans 100
  python3 accuracy_steps.py --truth 0.30 0.60 1.20            # tape distances -> full analysis
  python3 accuracy_steps.py --analyze ~/lidar_logs/accuracy_steps_X.csv --truth 0.3 0.6 1.2

TEST DIRECTION: --direction is the LiDAR angle the board is placed at, in the scan's own
frame. Default 180 = the ARROW side of the RPLIDAR C1 (its published 0 deg points away from
the arrow; measured with find_front.py). The launch file's lidar_yaw does not change this.

MEASURE THE TRUE DISTANCE from the centre of the spinning head (the LiDAR's origin) to the
board, square to the board. If you measure from the housing instead, the analysis shows that
as a constant offset (see "offset" below), not as a LiDAR error.

STATISTICS (what is computed and why)
  Per position (repeatability):
    mean, sample standard deviation (= PRECISION / noise),
    95 % confidence interval of the mean (Student t, n-1 dof).
    The per-scan values are correlated in time slightly, so the CI is a lower bound.
  Across positions (accuracy, needs --truth):
    error_i = mean_i - true_i            (BIAS at each distance)
    RMSE    = sqrt(mean(error_i^2))      (overall accuracy number for the report)
    Linear calibration by ordinary least squares:  measured = offset + scale * true
      offset : constant error (e.g. tape measured from the housing, or the sensor's zero)
      scale  : proportional error (1.000 = perfect); (scale-1)*1000 = mm of error per metre
      R^2 and residuals show whether a straight line explains the errors.
    This is the standard method for calibrating a range sensor against a reference. With
    only 3 distances the line has 1 degree of freedom left, so its confidence intervals are
    wide; use 5+ distances spread over the working range for a trustworthy calibration.
    Correction to apply afterwards:  true = (measured - offset) / scale.
"""
import argparse
import csv
import math
import os
import sys
import time
import warnings
from datetime import datetime

import numpy as np

# Two-sided 95 % Student-t critical values, dof 1..30 (dof > 30 -> 1.96).
_T95 = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179,
        2.160, 2.145, 2.131, 2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064,
        2.060, 2.056, 2.052, 2.048, 2.045, 2.042]


def t95(dof):
    return math.nan if dof < 1 else (_T95[dof - 1] if dof <= 30 else 1.96)


# =============================================================================== measurement
def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def mean_angle_deg(angles_deg):
    """Circular mean: averaging +179 and -179 must give 180, not 0."""
    if len(angles_deg) == 0:
        return math.nan
    a = np.radians(np.asarray(angles_deg, float))
    return math.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean()))


def board_from_scan(angle_min, angle_inc, ranges, range_min, range_max,
                    direction_deg=180.0, window_deg=30.0, gap_m=0.05, min_points=5):
    """Fit the board in one scan. Returns dict or None if no usable board.

    1. Take the beams within +-window/2 of `direction_deg`.
    2. Keep only the surface that contains the beam closest to the direction: walk outwards
       from it and stop where the range jumps by more than gap_m (that is another object).
    3. Total-least-squares line through those points -> perpendicular distance and the
       angle of the perpendicular (= where the board faces; equals direction when square).
    """
    r = np.asarray(ranges, dtype=float)
    n = r.size
    if n == 0:
        return None
    centre = int(round((math.radians(direction_deg) - angle_min) / angle_inc)) % n
    half = int(round(math.radians(window_deg) / 2.0 / angle_inc))
    ok = lambda i: math.isfinite(r[i]) and range_min <= r[i] <= range_max  # noqa: E731

    # Start from the valid beam nearest the direction (the centre beam may be a dropout).
    start = next((centre + s * k for k in range(0, 4) for s in (1, -1)
                  if ok((centre + s * k) % n)), None)
    if start is None:
        return None
    idx = [start % n]
    for step in (1, -1):                                   # grow left and right
        last, missed = start % n, 0
        for k in range(1, half + 1):
            i = (start + step * k) % n
            if not ok(i):
                missed += 1
                if missed > 2:
                    break
                continue
            if abs(r[i] - r[last]) > gap_m:
                break
            idx.append(i)
            last, missed = i, 0
    if len(idx) < min_points:
        return None
    idx = np.array(sorted(idx, key=lambda i: wrap_deg(math.degrees(angle_min + i * angle_inc)
                                                       - direction_deg)))
    a = angle_min + idx * angle_inc
    pts = np.column_stack([r[idx] * np.cos(a), r[idx] * np.sin(a)])
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c)
    normal = vt[1]
    d = float(normal @ c)
    if d < 0:
        normal, d = -normal, -d
    resid = (pts - c) @ normal
    beam = r[centre] if ok(centre) else math.nan
    return {
        "perp_m": d,
        "angle_deg": math.degrees(math.atan2(normal[1], normal[0])),
        "beam_m": float(beam),
        "points": int(len(idx)),
        "fit_rms_mm": float(np.sqrt(np.mean(resid ** 2)) * 1000.0),
    }


# =============================================================================== statistics
def describe(values_m):
    """Descriptive stats of one position (input metres, output mm except n)."""
    v = np.asarray(values_m, dtype=float) * 1000.0
    n = v.size
    out = {"n": n}
    if n == 0:
        return out
    out["mean_mm"] = float(v.mean())
    out["std_mm"] = float(v.std(ddof=1)) if n > 1 else math.nan
    out["ci95_mm"] = t95(n - 1) * out["std_mm"] / math.sqrt(n) if n > 1 else math.nan
    out["min_mm"], out["max_mm"] = float(v.min()), float(v.max())
    return out


def calibrate(true_m, mean_m):
    """OLS fit  measured = offset + scale * true  plus error summary.

    Returns dict with errors (mm), rmse_mm, offset_mm, scale, their 95 % CIs (if dof >= 1),
    r2, residuals_mm. Needs at least 2 points (3+ for CIs).
    """
    x = np.asarray(true_m, float)
    y = np.asarray(mean_m, float)
    n = x.size
    err_mm = (y - x) * 1000.0
    res = {"errors_mm": err_mm.tolist(), "rmse_mm": float(np.sqrt(np.mean(err_mm ** 2))),
           "mean_error_mm": float(err_mm.mean()), "n": n}
    if n < 2 or np.ptp(x) == 0:
        return res
    X = np.column_stack([np.ones(n), x])
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    offset, scale = float(coef[0]), float(coef[1])
    fit = X @ coef
    resid = y - fit
    ss_res = float(resid @ resid)
    ss_tot = float(((y - y.mean()) ** 2).sum())
    res.update(offset_mm=offset * 1000.0, scale=scale,
               scale_ppm=(scale - 1.0) * 1e6,
               r2=1.0 - ss_res / ss_tot if ss_tot > 0 else math.nan,
               residuals_mm=(resid * 1000.0).tolist())
    dof = n - 2
    if dof >= 1:
        s2 = ss_res / dof
        cov = s2 * np.linalg.inv(X.T @ X)
        t = t95(dof)
        res.update(offset_ci_mm=t * math.sqrt(cov[0, 0]) * 1000.0,
                   scale_ci=t * math.sqrt(cov[1, 1]), dof=dof)
    return res


def analysis_text(positions, truth=None):
    """positions: list of dicts {label, values (perp_m list), angles (deg list)}."""
    L = []
    L.append("SUMMARY (per position, perpendicular distance from line fit)")
    L.append(f"{'pos':>3} {'n':>4} {'mean (m)':>10} {'std (mm)':>9} {'95% CI':>10} "
             f"{'min (m)':>9} {'max (m)':>9} {'board angle':>12}")
    means = []
    for k, p in enumerate(positions, 1):
        s = describe(p["values"])
        if s["n"] == 0:
            L.append(f"{k:>3} {0:>4}  no board found")
            means.append(math.nan)
            continue
        ang = mean_angle_deg(p["angles"])
        means.append(s["mean_mm"] / 1000.0)
        L.append(f"{k:>3} {s['n']:>4} {s['mean_mm'] / 1000:>10.4f} {s['std_mm']:>9.2f} "
                 f"{'±' + format(s['ci95_mm'], '.2f'):>10} {s['min_mm'] / 1000:>9.4f} {s['max_mm'] / 1000:>9.4f} "
                 f"{ang:>+11.1f}°")
    if truth is None:
        L.append("\n(no true distances given: run with --analyze <csv> --truth d1 d2 ... "
                 "or paste the output + your tape distances)")
        return "\n".join(L)
    if len(truth) != len(positions):
        L.append(f"\n--truth has {len(truth)} values but there are {len(positions)} positions")
        return "\n".join(L)
    good = [(t, m) for t, m in zip(truth, means) if math.isfinite(m)]
    cal = calibrate([g[0] for g in good], [g[1] for g in good])
    L.append("\nACCURACY (measured mean - true)")
    L.append(f"{'pos':>3} {'true (m)':>9} {'mean (m)':>10} {'error (mm)':>11} {'error %':>8}")
    for k, (t, m) in enumerate(zip(truth, means), 1):
        if math.isfinite(m):
            e = (m - t) * 1000.0
            L.append(f"{k:>3} {t:>9.4f} {m:>10.4f} {e:>+11.2f} {e / (t * 10):>+8.2f}")
    L.append(f"\nRMSE over positions: {cal['rmse_mm']:.2f} mm   mean error: "
             f"{cal['mean_error_mm']:+.2f} mm")
    if "scale" in cal:
        L.append("Calibration line (OLS):  measured = offset + scale x true")
        oc = f" ± {cal['offset_ci_mm']:.1f}" if "offset_ci_mm" in cal else ""
        sc = f" ± {cal['scale_ci']:.4f}" if "scale_ci" in cal else ""
        L.append(f"  offset = {cal['offset_mm']:+.2f}{oc} mm     scale = {cal['scale']:.5f}{sc}"
                 f"   ({(cal['scale'] - 1) * 1000:+.2f} mm per metre)")
        L.append(f"  R^2 = {cal['r2']:.6f}    residuals (mm): "
                 + ", ".join(f"{r:+.2f}" for r in cal["residuals_mm"]))
        if cal.get("dof", 0) < 3:
            L.append("  Note: few distances -> the ± (95 %) ranges are wide; use 5+ for a "
                     "trustworthy calibration.")
        L.append(f"  Correction:  true = (measured - ({cal['offset_mm'] / 1000:+.4f})) / "
                 f"{cal['scale']:.5f}    [metres]")
    return "\n".join(L)


# =============================================================================== I/O
def save_csv(path, positions):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["position", "scan", "perp_m", "board_angle_deg", "beam_m", "points",
                    "fit_rms_mm"])
        for k, p in enumerate(positions, 1):
            for j, row in enumerate(p["rows"]):
                w.writerow([k, j, f"{row['perp_m']:.5f}", f"{row['angle_deg']:.2f}",
                            "" if not math.isfinite(row["beam_m"]) else f"{row['beam_m']:.4f}",
                            row["points"], f"{row['fit_rms_mm']:.2f}"])


def load_csv(path):
    pos = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            p = pos.setdefault(int(r["position"]), {"values": [], "angles": [], "rows": []})
            p["values"].append(float(r["perp_m"]))
            p["angles"].append(float(r["board_angle_deg"]))
    return [pos[k] for k in sorted(pos)]


def raw_block(positions):
    """Compact per-scan values in mm, one line per position (for pasting into a chat)."""
    lines = ["RAW per-scan perpendicular distances (mm):"]
    for k, p in enumerate(positions, 1):
        lines.append(f"P{k}: " + " ".join(f"{v * 1000:.1f}" for v in p["values"]))
    return "\n".join(lines)


def countdown(seconds, message, spin):
    """Show a countdown on one terminal line, keep ROS spinning so the queue stays fresh."""
    end = time.monotonic() + seconds
    while True:
        left = end - time.monotonic()
        if left <= 0:
            break
        bar = "#" * int(seconds - left) + "." * int(math.ceil(left))
        sys.stdout.write(f"\r{message}  {left:4.1f} s  [{bar}] ")
        sys.stdout.flush()
        spin(0.1)
    sys.stdout.write(f"\r{message}  GO - measuring, keep still{' ' * 30}\n")
    sys.stdout.flush()


# =============================================================================== main
def main():
    p = argparse.ArgumentParser(description="LiDAR accuracy test at several distances.")
    p.add_argument("--positions", type=int, default=3, help="number of distances (default 3)")
    p.add_argument("--wait", type=float, default=10.0, help="countdown before each, s (10)")
    p.add_argument("--scans", type=int, default=50, help="scans per position (50 = 5 s)")
    p.add_argument("--direction", type=float, default=180.0,
                   help="LiDAR angle of the board, deg (180 = arrow side of the C1)")
    p.add_argument("--window", type=float, default=30.0,
                   help="angular width searched for the board, deg (default 30)")
    p.add_argument("--truth", type=float, nargs="+",
                   help="true distances in metres (centre of LiDAR head to board), one per position")
    p.add_argument("--analyze", metavar="CSV", help="analyse a saved CSV instead of measuring")
    p.add_argument("--topic", default="/scan")
    p.add_argument("--out-dir", default="~/lidar_logs")
    a = p.parse_args()

    if a.analyze:
        print(analysis_text(load_csv(os.path.expanduser(a.analyze)), a.truth))
        return 0
    if a.truth and len(a.truth) != a.positions:
        a.positions = len(a.truth)          # --truth decides how many positions

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node("accuracy_steps")
    state = {"collect": False, "rows": [], "skipped": 0, "seen": 0}

    def on_scan(m):
        state["seen"] += 1
        if not state["collect"] or len(state["rows"]) >= a.scans:
            return
        b = board_from_scan(m.angle_min, m.angle_increment, m.ranges,
                            max(m.range_min, 0.05), m.range_max, a.direction, a.window)
        if b is None:
            state["skipped"] += 1
        else:
            state["rows"].append(b)

    node.create_subscription(LaserScan, a.topic, on_scan, qos_profile_sensor_data)
    spin = lambda t: rclpy.spin_once(node, timeout_sec=t)  # noqa: E731

    t0 = time.monotonic()
    while state["seen"] == 0:
        spin(0.2)
        if time.monotonic() - t0 > 10:
            print("No /scan after 10 s: is the LiDAR driver running?", file=sys.stderr)
            return 1

    print(f"LiDAR accuracy test: {a.positions} positions, board at {a.direction:+.0f} deg "
          f"(arrow side = 180), {a.scans} scans each.")
    positions = []
    for k in range(1, a.positions + 1):
        tgt = f" (target {a.truth[k - 1]:.3f} m)" if a.truth else ""
        countdown(a.wait, f"Position {k}/{a.positions}{tgt}: move the board now", spin)
        state.update(collect=True, rows=[], skipped=0)
        t_start = time.monotonic()
        while len(state["rows"]) < a.scans:
            spin(0.1)
            if time.monotonic() - t_start > a.scans / 10.0 * 3 + 5:
                print(f"  only {len(state['rows'])} usable scans (board not found in "
                      f"{state['skipped']}); continuing")
                break
        state["collect"] = False
        rows = state["rows"]
        pos = {"rows": rows, "values": [r["perp_m"] for r in rows],
               "angles": [r["angle_deg"] for r in rows]}
        positions.append(pos)
        s = describe(pos["values"])
        if s["n"]:
            print(f"  P{k}: {s['mean_mm'] / 1000:.4f} m  std {s['std_mm']:.2f} mm  "
                  f"board angle {mean_angle_deg(pos['angles']):+.1f} deg  "
                  f"({s['n']} scans, {np.mean([r['points'] for r in rows]):.0f} beams on board)")
        else:
            print(f"  P{k}: board not found - is it within +-{a.window / 2:.0f} deg of "
                  f"{a.direction:+.0f} deg and 0.05-{a.window} m away?")
    node.destroy_node()
    rclpy.shutdown()

    path = os.path.join(os.path.expanduser(a.out_dir),
                        datetime.now().strftime("accuracy_steps_%Y%m%d_%H%M%S.csv"))
    save_csv(path, positions)
    print("\n" + analysis_text(positions, a.truth))
    print("\n" + raw_block(positions))
    print(f"\nSaved: {path}")
    if not a.truth:
        print(f"Analyse later: python3 {os.path.basename(__file__)} --analyze {path} "
              f"--truth <d1> <d2> ... (metres)")
    return 0


if __name__ == "__main__":
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        sys.exit(main())
