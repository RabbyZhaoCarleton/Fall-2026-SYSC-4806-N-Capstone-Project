#!/usr/bin/env python3
"""Find the LiDAR's real 0-degree direction from a flat object placed in front of it.

HOW TO USE
  1. Keep the LiDAR driver running (ros2 launch lidar_bringup rplidar_c1.launch.py).
  2. Stand a flat, matte object (cardboard, a book) 0.3-1 m from the LiDAR, roughly square to
     the arrow on the housing, and keep everything else at least ~0.5 m farther away.
  3. In another terminal:   python3 find_front.py            (options: --help)
     Finer table:            python3 find_front.py --step 2
     Zoom on one direction:  python3 find_front.py --center 180 --span 30 --step 0.5

RUNNING THE SCRIPT
    SCRIPT="/mnt/c/Users/n_bod/OneDrive/Desktop/Fall-2026-SYSC-4806-N-Capstone-Project/lidar_bringup/scripts/find_front.py"
    python3 "$SCRIPT" --step 2                              # whole 360°, every 2°
    python3 "$SCRIPT" --center 180 --span 30 --step 0.5     # every beam from 165° to 195° (the arrow side)

WHAT IT DOES
  1. Collects ~2 s of scans (20 at 10 Hz) and takes the MEDIAN of each beam, which removes the
     random 1-2 mm noise and the occasional missing return.
  2. Prints the distance every 10 degrees all the way around (quick 360-degree overview).
  3. Splits the circle into separate surfaces: consecutive beams belong to the same surface
     unless the distance jumps by more than --gap metres or beams are missing.
  4. Fits a straight line to each surface (total least squares, i.e. PCA). A surface is FLAT
     when the points sit within a few mm of the line (RMS residual).
  5. For each flat surface, computes the angle at which a beam would hit it at exactly 90 deg
     (the foot of the perpendicular from the LiDAR to the line) and the perpendicular distance.
  6. The flat surface you placed is normally the CLOSEST flat one. If you set it square to
     where you believe 0 deg is, its perpendicular angle IS the error in that belief:
         perpendicular angle ~ 0 deg   -> the arrow is the 0-degree direction
         perpendicular angle ~ 180 deg -> the published 0 deg points the OTHER way
     If the object was not exactly square, this measures where it actually faces instead,
     so also check the "front_offset" printed for the object (see the note it prints).

ANGLES are ROS angles in the laser frame: 0 deg = +x of laser_frame, positive = counter-
clockwise (to the LEFT) seen from above. Same angles the other scripts print.
"""
import argparse
import math
import sys
import warnings

import numpy as np


# ------------------------------------------------------------------------- pure analysis
def median_scan(scans, range_min, range_max):
    """Per-beam median over several scans; NaN where fewer than half the scans returned."""
    r = np.array(scans, dtype=float)                       # shape: (n_scans, n_beams)
    r[~np.isfinite(r) | (r < range_min) | (r > range_max)] = np.nan
    ok = np.sum(np.isfinite(r), axis=0) >= max(1, r.shape[0] // 2)
    with np.errstate(all="ignore"), warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)    # all-NaN beams are expected
        med = np.nanmedian(r, axis=0)
    med[~ok] = np.nan
    return med


def split_segments(angles, ranges, gap_m, max_missing):
    """Group consecutive beams into surfaces. Returns lists of beam indices.

    A new surface starts when the distance jumps by more than gap_m between neighbouring
    valid beams, or when more than max_missing beams in a row have no return. The circle is
    closed: a surface crossing the +-180 deg seam is joined back together.
    """
    n = len(ranges)
    valid = np.isfinite(ranges)
    if not valid.any():
        return []
    # Start the walk just after an invalid beam or a jump, so no surface is cut by the seam.
    start = 0
    for i in range(n):
        j = (i - 1) % n
        if not valid[i] or not valid[j] or abs(ranges[i] - ranges[j]) > gap_m:
            start = i
            break
    segs, cur, missing, last = [], [], 0, None
    for k in range(n):
        i = (start + k) % n
        if not valid[i]:
            missing += 1
            if missing > max_missing and cur:
                segs.append(cur); cur, last = [], None
            continue
        if last is not None and (abs(ranges[i] - ranges[last]) > gap_m or missing > max_missing):
            segs.append(cur); cur = []
        cur.append(i); last, missing = i, 0
    if cur:
        segs.append(cur)
    return segs


def fit_line(angles, ranges, idx):
    """Total-least-squares line through the points of one surface.

    Returns dict: perp_angle_deg, perp_dist_m, rms_mm, length_m, span (deg from, deg to),
    foot_inside (True if the perpendicular foot lies on the visible part of the surface).
    """
    a, r = angles[idx], ranges[idx]
    pts = np.column_stack([r * np.cos(a), r * np.sin(a)])
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c)
    direction, normal = vt[0], vt[1]                       # line direction, line normal
    d = float(normal @ c)                                  # signed perpendicular distance
    if d < 0:                                              # make the normal point AWAY from
        normal, d = -normal, -d                            # the LiDAR
    resid = (pts - c) @ normal
    t = (pts - c) @ direction                              # position along the line
    foot_t = float((d * normal - c) @ direction)           # where the perpendicular lands
    return {
        "perp_angle_deg": math.degrees(math.atan2(normal[1], normal[0])),
        "perp_dist_m": d,
        "rms_mm": float(np.sqrt(np.mean(resid ** 2)) * 1000.0),
        "length_m": float(t.max() - t.min()),
        "span": (math.degrees(a[0]), math.degrees(a[-1])),
        "foot_inside": bool(t.min() <= foot_t <= t.max()),
        "n": len(idx),
        "mean_range_m": float(np.mean(r)),
    }


def find_flat_surfaces(angle_min, angle_inc, ranges, gap_m=0.05, max_missing=3,
                       min_points=6, min_length_m=0.05, max_rms_mm=6.0):
    """All flat surfaces in a (median) scan, closest first."""
    n = len(ranges)
    angles = (angle_min + angle_inc * np.arange(n) + math.pi) % (2 * math.pi) - math.pi
    out = []
    for idx in split_segments(angles, np.asarray(ranges, float), gap_m, max_missing):
        if len(idx) < min_points:
            continue
        f = fit_line(angles, np.asarray(ranges, float), np.array(idx))
        if f["length_m"] >= min_length_m and f["rms_mm"] <= max_rms_mm:
            out.append(f)
    return sorted(out, key=lambda f: f["perp_dist_m"])


def overview(angle_min, angle_inc, ranges, step_deg=10.0, center_deg=None, span_deg=None):
    """Distance table around the circle, one cell per step_deg.

    Without center_deg: the whole 360 degrees. With center_deg/span_deg: only the window
    center +- span/2 (a "zoom"), e.g. center 180, span 30, step 0.5 shows every beam
    between 165 and 195 deg. Steps finer than the beam spacing (0.5 deg on the C1) just
    repeat the nearest beam, so 0.5 deg is the finest useful step.
    """
    n = len(ranges)
    if center_deg is None:
        start, stop = -180.0, 180.0
    else:
        start, stop = center_deg - span_deg / 2.0, center_deg + span_deg / 2.0 + 1e-9
    cells, deg = [], start
    while deg < stop:
        i = round((math.radians(deg) - angle_min) / angle_inc) % n
        shown = (deg + 180.0) % 360.0 - 180.0             # print as -180..+180
        label = f"{shown:+7.1f}" if step_deg % 1 else f"{shown:+5.0f}"
        r = ranges[i]
        cells.append(f"{label}:{r:6.3f}" if math.isfinite(r) else f"{label}:   -  ")
        deg += step_deg
    per_row = 6 if center_deg is None or step_deg >= 1 else 5
    return "\n".join("  ".join(cells[k:k + per_row]) for k in range(0, len(cells), per_row))


def report(angle_min, angle_inc, ranges, args):
    if args.center is None:
        title = f"360-degree overview (median distance in m every {args.step:g} deg; '-' = no return):"
    else:
        title = (f"Zoom {args.center - args.span / 2:+g}..{args.center + args.span / 2:+g} deg "
                 f"(median distance in m every {args.step:g} deg; '-' = no return):")
    lines = [title, overview(angle_min, angle_inc, ranges, args.step, args.center, args.span), ""]
    flats = find_flat_surfaces(angle_min, angle_inc, ranges, args.gap, 3, 6,
                               args.min_length, args.max_rms)
    if not flats:
        lines.append("No flat surface found. Move the object closer (0.3-1 m), use a matte "
                     "light surface, or loosen --max-rms.")
        return "\n".join(lines)
    lines.append("Flat surfaces, closest first:")
    lines.append(f"{'#':>2} {'perp angle':>11} {'perp dist':>10} {'seen from':>17} "
                 f"{'width':>7} {'flatness':>9}  note")
    for k, f in enumerate(flats[:args.top]):
        note = "" if f["foot_inside"] else "perpendicular point is off the visible part"
        lines.append(f"{k:>2} {f['perp_angle_deg']:>+10.1f}° {f['perp_dist_m']:>9.3f}m "
                     f"{f['span'][0]:>+7.1f}..{f['span'][1]:>+6.1f}° {f['length_m']:>6.2f}m "
                     f"{f['rms_mm']:>6.1f}mm  {note}")
    best = flats[0]
    a = best["perp_angle_deg"]
    lines += ["",
              f"Closest flat surface: the LiDAR is perpendicular to it at {a:+.1f} deg, "
              f"{best['perp_dist_m']:.3f} m away.",
              "If that is the object you set square to the arrow:"]
    if abs(a) <= 5:
        lines.append("  -> the arrow IS the 0-degree direction (within the object's alignment).")
    elif abs(abs(a) - 180) <= 5:
        lines.append("  -> 0 deg points OPPOSITE the arrow. Use lidar_yaw:=3.14159 when mounting "
                     "the arrow forward, or turn the LiDAR around.")
    else:
        lines.append(f"  -> the object faces {a:+.1f} deg. Either it isn't square to the arrow, "
                     f"or 0 deg is {a:+.1f} deg away from the arrow. Re-square the object "
                     "(or rotate it 90 deg and re-run): if the answer stays the same, "
                     "the arrow is off by that much.")
    lines.append("Use the perpendicular distance as the 'expected' value for the tape test.")
    return "\n".join(lines)


# ------------------------------------------------------------------------- ROS part
def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--scans", type=int, default=20, help="scans to average (default 20 = 2 s)")
    p.add_argument("--gap", type=float, default=0.05,
                   help="distance jump (m) that starts a new surface (default 0.05)")
    p.add_argument("--max-rms", type=float, default=6.0,
                   help="max deviation from a straight line to count as flat, mm (default 6)")
    p.add_argument("--min-length", type=float, default=0.05,
                   help="minimum visible width of a surface, m (default 0.05)")
    p.add_argument("--top", type=int, default=8, help="how many flat surfaces to list")
    p.add_argument("--topic", default="/scan")
    p.add_argument("--step", type=float, default=10.0,
                   help="angle step of the distance table, deg (default 10; 0.5 = every beam)")
    p.add_argument("--center", type=float, default=None,
                   help="zoom the table on this angle, deg (e.g. 180 for the arrow side)")
    p.add_argument("--span", type=float, default=30.0,
                   help="width of the zoom window, deg (default 30)")
    args = p.parse_args()

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node("find_front")
    scans, meta = [], {}

    def on_scan(m):
        if len(scans) < args.scans:
            scans.append(list(m.ranges))
            meta.update(amin=m.angle_min, inc=m.angle_increment,
                        rmin=m.range_min, rmax=m.range_max, n=len(m.ranges))

    node.create_subscription(LaserScan, args.topic, on_scan, qos_profile_sensor_data)
    print(f"Collecting {args.scans} scans from {args.topic} ... keep everything still.")
    waited = 0.0
    while rclpy.ok() and len(scans) < args.scans:
        rclpy.spin_once(node, timeout_sec=0.2)
        waited += 0.2
        if waited > 10 and not scans:
            print("No scans after 10 s: is the LiDAR driver running?", file=sys.stderr)
            return 1
    node.destroy_node()
    rclpy.shutdown()

    scans = [s for s in scans if len(s) == meta["n"]]       # ignore any odd-sized scan
    med = median_scan(scans, max(meta["rmin"], 0.08), meta["rmax"])
    print(report(meta["amin"], meta["inc"], med, args))
    return 0


if __name__ == "__main__":
    sys.exit(main())
