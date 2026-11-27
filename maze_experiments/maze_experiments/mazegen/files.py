"""A maze's files: ASCII layout (AR2), Gazebo world (AR1, AR9), preview (AR14).

The layout uses the format of maze_sim/layouts/*.txt and make_maze.py: each
cell is 3 characters wide, "+--" is a top wall and "+  " an opening, and "|"
on the odd lines is a wall on a cell's left side. Lines starting with "#" are
comments.

The world uses make_maze.py's plugins and its wall geometry: walls are boxes
centred on the grid lines, CELL + THICK long. The walls are computed from the
layout text with make_maze.py's own algorithm, so the two always agree. Only
the ground plane differs: it covers the maze plus the lidar range + 0.5 m on
every side (AR9), where make_maze.py uses a fixed 10 m x 10 m plane.
"""

import struct
import zlib

WORLD_NAME = 'maze_world'

HEADER = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="maze_world">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <pose>{px:.3f} {py:.3f} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{sx:.3f} {sy:.3f}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{sx:.3f} {sy:.3f}</size></plane></geometry>
        </visual>
      </link>
    </model>
"""

FOOTER = """  </world>
</sdf>
"""


# ------------------------------------------------------------------ layout
def layout_lines(maze):
    lines = []
    for r in range(maze.rows + 1):
        lines.append('+' + ''.join(('--' if maze.h[r][c] else '  ') + '+'
                                   for c in range(maze.columns)))
        if r < maze.rows:
            lines.append(''.join(('|' if maze.v[r][c] else ' ') + '  '
                                 for c in range(maze.columns))
                         + ('|' if maze.v[r][maze.columns] else ' '))
    return lines


def layout_is_parseable(maze):
    """make_maze.py drops blank lines, so a wall line with no wall at all would be lost."""
    return all(line.strip() for line in layout_lines(maze))


def layout_text(maze, comments=()):
    head = ''.join(f'# {c}\n' if c else '#\n' for c in comments)
    return head + '\n'.join(layout_lines(maze)) + '\n'


def strip_comments(text):
    return '\n'.join(l for l in text.splitlines() if not l.lstrip().startswith('#'))


# ------------------------------------------------------------------ walls
def walls_from_layout(text, cell, thick):
    """(x, y, sx, sy) of every wall box, exactly as make_maze.py's build() places them."""
    lines = [l for l in strip_comments(text).splitlines() if l.strip()]
    walls = []
    for i, line in enumerate(lines):
        if i % 2 == 0:  # horizontal wall line
            row = i // 2
            for c in range(len(line) // 3):
                if line[3 * c + 1: 3 * c + 3] == '--':
                    walls.append((c * cell + cell / 2, -row * cell, cell + thick, thick))
        else:           # vertical wall line
            row = i // 2
            for c in range(len(line) // 3 + 1):
                if 3 * c < len(line) and line[3 * c] == '|':
                    walls.append((c * cell, -(row * cell + cell / 2), thick, cell + thick))
    return walls


def wall_model(name, x, y, sx, sy, height):
    size = f'{sx:.3f} {sy:.3f} {height:.3f}'
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {height / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
          <material><ambient>0.3 0.3 0.6 1</ambient><diffuse>0.3 0.3 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def ground_plane(columns, rows, pitch, thick, margin):
    width, length = columns * pitch, rows * pitch
    return {'center': (width / 2, -length / 2),
            'size': (width + thick + 2 * margin, length + thick + 2 * margin)}


def world_sdf(text, pitch, thick, height, plane):
    walls = walls_from_layout(text, pitch, thick)
    body = ''.join(wall_model(f'wall_{n}', *w, height) for n, w in enumerate(walls))
    (px, py), (sx, sy) = plane['center'], plane['size']
    return HEADER.format(px=px, py=py, sx=sx, sy=sy) + body + FOOTER


# ------------------------------------------------------------------ preview
# Colors follow maze_monitor's style.py: black walls (reality), green exits,
# blue start (where the mission returns to), white free space.
WHITE = (255, 255, 255)
WALL = (0x21, 0x21, 0x21)
EXIT = (0x2e, 0x7d, 0x32)
START = (0x15, 0x65, 0xc0)
FRAME = (0xd8, 0xd8, 0xd8)


class Canvas:
    def __init__(self, w, h, bg=WHITE):
        self.w, self.h = w, h
        self.px = bytearray(bytes(bg) * (w * h))

    def rect(self, x0, y0, x1, y1, color):
        x0, x1 = sorted((int(round(x0)), int(round(x1))))
        y0, y1 = sorted((int(round(y0)), int(round(y1))))
        x0, y0 = max(x0, 0), max(y0, 0)
        x1, y1 = min(x1, self.w), min(y1, self.h)
        if x1 <= x0 or y1 <= y0:
            return
        row = bytes(color) * (x1 - x0)
        for y in range(y0, y1):
            i = (y * self.w + x0) * 3
            self.px[i:i + len(row)] = row

    def disc(self, cx, cy, radius, color):
        r2 = radius * radius
        for y in range(int(cy - radius), int(cy + radius) + 1):
            for x in range(int(cx - radius), int(cx + radius) + 1):
                if 0 <= x < self.w and 0 <= y < self.h and (x - cx) ** 2 + (y - cy) ** 2 <= r2:
                    i = (y * self.w + x) * 3
                    self.px[i:i + 3] = bytes(color)

    def png(self):
        raw = bytearray()
        stride = self.w * 3
        for y in range(self.h):
            raw.append(0)
            raw += self.px[y * stride:(y + 1) * stride]

        def chunk(tag, data):
            body = tag + data
            return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body))
        return (b'\x89PNG\r\n\x1a\n'
                + chunk(b'IHDR', struct.pack('>IIBBBBB', self.w, self.h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(bytes(raw), 9))
                + chunk(b'IEND', b''))


def preview_png(maze, walls, size=360):
    """A small picture of the maze: walls, exits and start (deterministic bytes)."""
    p = maze.params
    pitch, thick = p.pitch, p.wall_thickness
    width, length = maze.columns * pitch, maze.rows * pitch
    border = 12
    scale = (size - 2 * border) / max(width, length)
    w = int(round(width * scale)) + 2 * border
    h = int(round(length * scale)) + 2 * border
    cv = Canvas(w, h)
    def to_px(x, y):
        return border + x * scale, border + (-y) * scale
    t = max(thick * scale, 2.0)
    for x, y, sx, sy in walls:
        cx, cy = to_px(x, y)
        hw = max(sx * scale, t) / 2
        hh = max(sy * scale, t) / 2
        cv.rect(cx - hw, cy - hh, cx + hw, cy + hh, WALL)
    for side, c, r in maze.exits:
        gx, gy = exit_gap_centre(maze, side, c, r)
        cx, cy = to_px(gx, gy)
        half = pitch * scale * 0.4
        bar = max(t, 3.0)
        if side in ('top', 'bottom'):
            cv.rect(cx - half, cy - bar / 2, cx + half, cy + bar / 2, EXIT)
        else:
            cv.rect(cx - bar / 2, cy - half, cx + bar / 2, cy + half, EXIT)
    sx, sy = cell_centre(maze, *maze.start)
    cx, cy = to_px(sx, sy)
    cv.disc(cx, cy, max(pitch * scale * 0.22, 3.0), START)
    return cv.png()


# ------------------------------------------------------------------ geometry
def cell_centre(maze, c, r):
    pitch = maze.params.pitch
    return c * pitch + pitch / 2, -(r * pitch + pitch / 2)


def exit_gap_centre(maze, side, c, r):
    pitch = maze.params.pitch
    if side == 'top':
        return c * pitch + pitch / 2, 0.0
    if side == 'bottom':
        return c * pitch + pitch / 2, -maze.rows * pitch
    if side == 'left':
        return 0.0, -(r * pitch + pitch / 2)
    return maze.columns * pitch, -(r * pitch + pitch / 2)


def free_area(maze):
    """Floor area inside the outer walls not covered by inner walls (m²)."""
    p = maze.params
    pitch, t = p.pitch, p.wall_thickness
    interior = (maze.columns * pitch - t) * (maze.rows * pitch - t)
    segments = 0
    posts = set()
    for r in range(1, maze.rows):
        for c in range(maze.columns):
            if maze.h[r][c]:
                segments += 1
                for pc in (c, c + 1):
                    if 0 < pc < maze.columns:
                        posts.add((pc, r))
    for r in range(maze.rows):
        for c in range(1, maze.columns):
            if maze.v[r][c]:
                segments += 1
                for pr in (r, r + 1):
                    if 0 < pr < maze.rows:
                        posts.add((c, pr))
    return interior - segments * (pitch - t) * t - len(posts) * t * t
