"""Maze parameters and their rules (AR3, AR5, AR6, AR7).

Pure Python, no ROS (ANR1). Dimensions are measured between the centrelines
of the outer walls, so width = columns x cell pitch (AR5). The cell pitch is
the corridor width plus the wall thickness (1.4).
"""

from dataclasses import asdict, dataclass, field, fields

TOLERANCE = 0.001        # m: the pitch must divide the dimensions within 1 mm (AR6)
MIN_WALL_THICKNESS = 0.01
MAX_CELLS = 100          # per side; far beyond any physical maze, keeps generation fast
START_MODES = ('fixed', 'random')


@dataclass
class MazeParams:
    width: float = 5.0             # m, x extent between outer wall centrelines
    length: float = 5.0            # m, y extent between outer wall centrelines
    corridor_width: float = 0.48   # m, clear width between wall faces
    wall_thickness: float = 0.02   # m
    wall_height: float = 0.30      # m (make_maze.py's HEIGHT)
    exits: int = 2
    min_exit_separation: int = 2   # cells, centre to centre along the boundary
    allow_exit_in_start_cell: bool = False
    start: str = 'fixed'           # 'fixed' or 'random'
    start_cell: list = field(default_factory=lambda: [0, 0])  # [column, row]; row 0 is the top
    loop_fraction: float = 0.0     # share of the remaining inner walls removed (AR11)

    @property
    def pitch(self):
        return self.corridor_width + self.wall_thickness

    @property
    def columns(self):
        return max(int(round(self.width / self.pitch)), 0) if self.pitch > 0 else 0

    @property
    def rows(self):
        return max(int(round(self.length / self.pitch)), 0) if self.pitch > 0 else 0

    def to_dict(self):
        d = asdict(self)
        d['start_cell'] = list(self.start_cell)
        return d

    @classmethod
    def from_dict(cls, data):
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(data) - known)
        if unknown:
            raise ValueError(f'unknown maze parameter(s): {", ".join(unknown)}')
        kwargs = dict(data)
        for name in ('width', 'length', 'corridor_width', 'wall_thickness', 'wall_height',
                     'loop_fraction'):
            if name in kwargs:
                kwargs[name] = float(kwargs[name])
        for name in ('exits', 'min_exit_separation'):
            if name in kwargs:
                kwargs[name] = int(kwargs[name])
        if 'start_cell' in kwargs:
            kwargs['start_cell'] = [int(v) for v in kwargs['start_cell']]
        return cls(**kwargs)


def divides(length, pitch):
    if pitch <= 0:
        return False
    n = round(length / pitch)
    return n >= 1 and abs(n * pitch - length) <= TOLERANCE


def valid_corridor_widths(width, length, wall_thickness, minimum=0.0, maximum=None):
    """Corridor widths whose pitch divides both dimensions (AR6), widest first."""
    out = []
    if width <= 0 or length <= 0:
        return out
    for n in range(1, MAX_CELLS + 1):
        pitch = width / n
        corridor = pitch - wall_thickness
        if corridor < minimum - 1e-9:
            break
        if maximum is not None and corridor > maximum + 1e-9:
            continue
        if divides(length, pitch) and round(length / pitch) <= MAX_CELLS:
            out.append(round(corridor, 4))
    return out


def nearest_valid(value, candidates):
    if not candidates:
        return []
    return sorted(candidates, key=lambda c: abs(c - value))[:2]


def boundary_segments(columns, rows):
    """Outer-wall segments in clockwise order from the top-left corner.

    Each is (index, side, column, row). The distance between two segments
    along the boundary, in cells, is min(|a - b|, P - |a - b|) with
    P = 2 (columns + rows).
    """
    segs = []
    for c in range(columns):
        segs.append(('top', c, 0))
    for r in range(rows):
        segs.append(('right', columns - 1, r))
    for c in range(columns - 1, -1, -1):
        segs.append(('bottom', c, rows - 1))
    for r in range(rows - 1, -1, -1):
        segs.append(('left', 0, r))
    return [(i, *s) for i, s in enumerate(segs)]


def boundary_distance(a, b, perimeter):
    d = abs(a - b)
    return min(d, perimeter - d)


def allowed_exit_segments(columns, rows, start_cell, allow_in_start):
    segs = boundary_segments(columns, rows)
    if allow_in_start or start_cell is None:
        return segs
    sc, sr = start_cell
    return [s for s in segs if (s[2], s[3]) != (sc, sr)]


def blank_row_conflict(a, b, columns=None):
    """True if exits a and b are the left and right side of the same row.

    make_maze.py drops blank layout lines, so a row with exits on both sides
    and no inner wall would lose its line. Never putting exits on both sides
    of one row rules that out for every maze, and keeps max_exits() exact.
    """
    return {a[1], b[1]} == {'left', 'right'} and a[3] == b[3]


def max_exits(columns, rows, separation, start_cell=None, allow_in_start=False):
    """The most exits that fit with the separation rule (AR3); exact, by trying every start."""
    if columns < 1 or rows < 1:
        return 0
    perimeter = 2 * (columns + rows)
    segs = {s[0]: s for s in allowed_exit_segments(columns, rows, start_cell, allow_in_start)}
    allowed = sorted(segs)
    if not allowed:
        return 0
    separation = max(int(separation), 1)
    best = 0
    # Greedy clockwise from a fixed first exit is optimal on a line; trying every
    # first exit makes it optimal on the closed boundary.
    for first in allowed:
        chosen = [first]
        for idx in sorted(allowed, key=lambda i: (i - first) % perimeter)[1:]:
            if ((idx - chosen[-1]) % perimeter >= separation
                    and (first - idx) % perimeter >= separation
                    and not any(blank_row_conflict(segs[idx], segs[c], columns)
                                for c in chosen)):
                chosen.append(idx)
        best = max(best, len(chosen))
    return best


def validate(params, profile=None):
    """Check maze parameters. Returns (errors, warnings), lists of strings; never raises."""
    errors, warnings = [], []
    p = params
    if p.wall_thickness < MIN_WALL_THICKNESS - 1e-9:
        errors.append(f'Wall thickness {p.wall_thickness:.3f} m is below the minimum '
                      f'{MIN_WALL_THICKNESS:.2f} m.')
    if p.corridor_width <= 0:
        errors.append('Corridor width must be positive.')
    if p.width <= 0 or p.length <= 0:
        errors.append('Width and length must be positive.')
    if errors:
        return errors, warnings
    pitch = p.pitch
    for name, value in (('width', p.width), ('length', p.length)):
        if not divides(value, pitch):
            valid = valid_corridor_widths(p.width, p.length, p.wall_thickness,
                                          minimum=profile.min_corridor_width if profile else 0)
            near = nearest_valid(p.corridor_width, valid)
            hint = (' Nearest valid corridor widths: ' + ', '.join(f'{c:.3f} m' for c in near)
                    if near else ' No corridor width fits these dimensions; change them.')
            errors.append(f'The cell pitch {pitch:.3f} m (corridor {p.corridor_width:.3f} + '
                          f'wall {p.wall_thickness:.3f}) does not divide the {name} '
                          f'{value:.3f} m.{hint}')
            break
    cols, rows = p.columns, p.rows
    if cols > MAX_CELLS or rows > MAX_CELLS:
        errors.append(f'The grid is {cols} x {rows} cells; at most {MAX_CELLS} per side.')
    if profile is not None:
        if p.corridor_width < profile.min_corridor_width - 1e-9:
            errors.append(
                f'Corridors of {p.corridor_width:.3f} m are too narrow for the '
                f'{profile.name} robot: the minimum is {profile.min_corridor_width:.2f} m '
                f'(2 x navigation radius {profile.navigation_radius:.2f} + 2 x costmap '
                f'resolution {profile.costmap_resolution:.2f}).')
        elif p.corridor_width < profile.tested_corridor_width - 1e-9:
            warnings.append(
                f'Corridors of {p.corridor_width:.3f} m are narrower than the '
                f'{profile.tested_corridor_width:.2f} m the {profile.name} stack was tuned on; '
                f'results may reflect a maze the stack was not tuned for.')
        if p.wall_height < profile.min_wall_height - 1e-9:
            errors.append(f'Walls of {p.wall_height:.2f} m are lower than the lidar '
                          f'({profile.lidar_height:.3f} m) + 0.05 m = '
                          f'{profile.min_wall_height:.3f} m.')
    if p.start not in START_MODES:
        errors.append(f'start must be one of {", ".join(START_MODES)}.')
    if p.min_exit_separation < 1:
        errors.append('min_exit_separation must be at least 1 cell.')
    if not 0.0 <= p.loop_fraction <= 1.0:
        errors.append('loop_fraction must be between 0 and 1.')
    if errors:
        return errors, warnings
    if p.start == 'fixed':
        sc, sr = p.start_cell
        if not (0 <= sc < cols and 0 <= sr < rows):
            errors.append(f'Start cell {p.start_cell} is outside the {cols} x {rows} grid.')
            return errors, warnings
        start = (sc, sr)
    else:
        start = None
    # With a random start, the start can be any cell: use the worst case (a corner cell).
    limit = max_exits(cols, rows, p.min_exit_separation,
                      start if start is not None else (0, 0), p.allow_exit_in_start_cell)
    if p.exits < 1:
        errors.append('A maze needs at least 1 exit.')
    elif p.exits > limit:
        errors.append(f'{p.exits} exits do not fit a {cols} x {rows} grid with exits at least '
                      f'{p.min_exit_separation} cells apart: at most {limit}.')
    return errors, warnings
