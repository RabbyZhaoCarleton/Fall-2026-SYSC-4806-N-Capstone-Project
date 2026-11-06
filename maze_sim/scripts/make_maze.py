#!/usr/bin/env python3
"""Convert an ASCII grid maze into a Gazebo SDF world."""

import os

CELL = 0.5        # cell size in metres (match your physical maze)
THICK = 0.02      # wall thickness
HEIGHT = 0.3      # wall height

# Each cell is 3 characters wide: "+--" for a top wall, "+  " for an opening.
# "|" on the odd lines marks a wall on the cell's left side.
MAZE = """
+--+--+--+--+
|     |     
+  +  +  +--+
|  |     |  |
+  +--+--+  +
|           |
+--+--+  +--+
"""

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
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
        </visual>
      </link>
    </model>
"""

FOOTER = """  </world>
</sdf>
"""

def wall(name, x, y, sx, sy):
    size = f"{sx:.3f} {sy:.3f} {HEIGHT:.3f}"
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {HEIGHT / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
          <material><ambient>0.3 0.3 0.6 1</ambient><diffuse>0.3 0.3 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

def build(maze):
    lines = [l for l in maze.splitlines() if l.strip()]
    walls = []
    for i, line in enumerate(lines):
        if i % 2 == 0:  # horizontal wall line
            row = i // 2
            for c in range(len(line) // 3):
                if line[3 * c + 1 : 3 * c + 3] == "--":
                    x = c * CELL + CELL / 2
                    y = -row * CELL
                    walls.append((x, y, CELL + THICK, THICK))
        else:           # vertical wall line
            row = i // 2
            for c in range(len(line) // 3 + 1):
                if 3 * c < len(line) and line[3 * c] == "|":
                    x = c * CELL
                    y = -(row * CELL + CELL / 2)
                    walls.append((x, y, THICK, CELL + THICK))
    body = "".join(wall(f"wall_{n}", *w) for n, w in enumerate(walls))
    return HEADER + body + FOOTER

if __name__ == "__main__":
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "worlds", "maze.sdf")
    with open(out, "w") as f:
        f.write(build(MAZE))
    print(f"Wrote {os.path.normpath(out)}")