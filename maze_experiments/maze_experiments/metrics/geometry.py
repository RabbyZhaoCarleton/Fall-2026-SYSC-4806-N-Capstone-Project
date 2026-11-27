"""Maze ground truth as geometry: wall boxes, free-space samples, map alignment.

Pure numpy, no ROS (ENR2). Wall boxes are (x, y, sx, sy) in world coordinates,
from maze.json (AR13). The map frame starts at the robot's true start pose, so
a map point p is at compose(start, p) in the world.
"""

import math

import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    """Pose b expressed in a's parent frame, for 2D poses (x, y, yaw) (drift_report.py)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def transform_points(pose, pts):
    """Points (N x 2) from a frame at `pose` into its parent frame."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    return np.column_stack((pose[0] + c * pts[:, 0] - s * pts[:, 1],
                            pose[1] + s * pts[:, 0] + c * pts[:, 1]))


def boxes_array(boxes):
    b = np.asarray(boxes, dtype=float).reshape(-1, 4)
    return b


def distance_to_boxes(pts, boxes, chunk=4096):
    """Distance from each point to the nearest wall box (0 inside a box)."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    b = boxes_array(boxes)
    if len(b) == 0:
        return np.full(len(pts), np.inf)
    hx, hy = b[:, 2] / 2, b[:, 3] / 2
    out = np.empty(len(pts))
    for i in range(0, len(pts), chunk):
        p = pts[i:i + chunk]
        dx = np.maximum(np.abs(p[:, 0:1] - b[None, :, 0]) - hx[None, :], 0.0)
        dy = np.maximum(np.abs(p[:, 1:2] - b[None, :, 1]) - hy[None, :], 0.0)
        out[i:i + chunk] = np.sqrt(dx * dx + dy * dy).min(axis=1)
    return out


def free_samples(meta, spacing, margin):
    """Points inside the maze, on a grid of `spacing`, at least `margin` from every wall."""
    g = meta['grid']
    width, length = g['columns'] * g['pitch'], g['rows'] * g['pitch']
    t = g['wall_thickness']
    xs = np.arange(t / 2 + spacing / 2, width - t / 2, spacing)
    ys = -np.arange(t / 2 + spacing / 2, length - t / 2, spacing)
    gx, gy = np.meshgrid(xs, ys)
    pts = np.column_stack((gx.ravel(), gy.ravel()))
    d = distance_to_boxes(pts, meta['walls']['boxes'])
    return pts[d >= margin]


def wall_samples(meta, spacing):
    """Points along every wall's centreline, `spacing` apart."""
    pts = []
    for x, y, sx, sy in meta['walls']['boxes']:
        if sx >= sy:
            n = max(int(math.ceil(sx / spacing)), 1)
            xs = np.linspace(x - sx / 2, x + sx / 2, n + 1)
            pts.append(np.column_stack((xs, np.full_like(xs, y))))
        else:
            n = max(int(math.ceil(sy / spacing)), 1)
            ys = np.linspace(y - sy / 2, y + sy / 2, n + 1)
            pts.append(np.column_stack((np.full_like(ys, x), ys)))
    return np.vstack(pts) if pts else np.zeros((0, 2))


class GridMap:
    """An OccupancyGrid's essentials, without ROS types."""

    def __init__(self, width, height, resolution, origin, data, stamp=None):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin = origin                  # (x, y, yaw) of cell (0, 0)'s corner, map frame
        self.data = np.asarray(data, dtype=np.int8).reshape(self.height, self.width)
        self.stamp = stamp

    @classmethod
    def from_msg(cls, msg, stamp=None):
        o = msg.info.origin
        yaw = math.atan2(2 * (o.orientation.w * o.orientation.z + o.orientation.x *
                              o.orientation.y),
                         1 - 2 * (o.orientation.y ** 2 + o.orientation.z ** 2))
        return cls(msg.info.width, msg.info.height, msg.info.resolution,
                   (o.position.x, o.position.y, yaw), msg.data, stamp)

    def indices(self, pts_map):
        """(row, col, valid) of points given in the map frame."""
        local = transform_points(inverse(self.origin), pts_map)
        col = np.floor(local[:, 0] / self.resolution).astype(int)
        row = np.floor(local[:, 1] / self.resolution).astype(int)
        valid = (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        return row, col, valid

    def cell_centres(self, mask):
        """Map-frame centres of the cells where mask is true."""
        rows, cols = np.nonzero(mask)
        local = np.column_stack(((cols + 0.5) * self.resolution, (rows + 0.5) * self.resolution))
        return transform_points(self.origin, local)

    def known_fraction(self, pts_map):
        if len(pts_map) == 0:
            return 0.0
        row, col, valid = self.indices(pts_map)
        known = np.zeros(len(pts_map), dtype=bool)
        known[valid] = self.data[row[valid], col[valid]] != -1
        return float(known.mean())


def wall_precision_recall(grid, start, meta, occupied_threshold=65):
    """Final map against the true walls, within one map cell (ER1, Mapping).

    Precision: share of occupied map cells within one cell of a true wall.
    Recall: share of points along the true walls' centrelines with an occupied
    cell within one cell (plus half the wall thickness) of them.
    """
    res = grid.resolution
    occ = grid.data >= occupied_threshold
    if not occ.any():
        return None, 0.0, 0
    centres_world = transform_points(start, grid.cell_centres(occ))
    boxes = meta['walls']['boxes']
    d = distance_to_boxes(centres_world, boxes)
    precision = float((d <= res).mean())
    t = meta['grid']['wall_thickness']
    samples = wall_samples(meta, res / 2)
    samples_map = transform_points(inverse(start), samples)
    # Dilate the occupied cells by the tolerance and look the samples up.
    reach = int(math.ceil((res + t / 2) / res))
    dil = occ.copy()
    for dr in range(-reach, reach + 1):
        for dc in range(-reach, reach + 1):
            if dr == 0 and dc == 0:
                continue
            shifted = np.zeros_like(occ)
            r0, r1 = max(dr, 0), occ.shape[0] + min(dr, 0)
            c0, c1 = max(dc, 0), occ.shape[1] + min(dc, 0)
            shifted[r0:r1, c0:c1] = occ[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
            dil |= shifted
    row, col, valid = grid.indices(samples_map)
    hit = np.zeros(len(samples_map), dtype=bool)
    hit[valid] = dil[row[valid], col[valid]]
    recall = float(hit.mean()) if len(hit) else None
    return precision, recall, int(occ.sum())


def match_exits(true_exits, detected, tolerance):
    """One-to-one nearest matching. Returns (pairs [(true i, detected j, distance)], missed, false)."""
    pairs = []
    cand = []
    for i, t in enumerate(true_exits):
        for j, d in enumerate(detected):
            dist = math.hypot(t[0] - d[0], t[1] - d[1])
            if dist <= tolerance:
                cand.append((dist, i, j))
    used_t, used_d = set(), set()
    for dist, i, j in sorted(cand):
        if i in used_t or j in used_d:
            continue
        used_t.add(i)
        used_d.add(j)
        pairs.append((i, j, dist))
    missed = [i for i in range(len(true_exits)) if i not in used_t]
    false = [j for j in range(len(detected)) if j not in used_d]
    return pairs, missed, false
