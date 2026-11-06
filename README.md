# Capstone maze robot

| Package | Purpose | Desktop (sim) | Raspberry Pi 5 |
|---|---|---|---|
| `maze_description` | Robot model (URDF/xacro) | yes | yes |
| `maze_navigation` | slam_toolbox, Nav2 and explore_lite configuration | yes | yes |
| `maze_bringup` | Real hardware drivers and robot launch files | no | yes |
| `maze_sim` | Gazebo worlds, maze generator, simulation launch files | yes | no |

## Setup

This repository goes inside the `src` folder of a ROS 2 Jazzy workspace.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <this repository's URL> capstone
vcs import < capstone/capstone.repos
```

### Desktop (simulation)

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch maze_sim maze.launch.py explore:=true
```

### Raspberry Pi 5 (real robot)

The Pi does not build the simulation or install Gazebo.

```bash
cd ~/ros2_ws
rosdep install --from-paths src/capstone/maze_description src/capstone/maze_navigation src/capstone/maze_bringup src/m-explore-ros2 --ignore-src -r -y
colcon build --symlink-install --packages-skip maze_sim turtlebot3_gazebo turtlebot3_simulations turtlebot3_fake_node
```

## Simulation

```bash
ros2 launch maze_sim maze.launch.py explore:=true gui:=false
```

Useful arguments (see all with `--show-args`): `world`, `x_pose`, `y_pose`, `gui`, `rviz`, `nav`, `explore`.

To change the maze, edit the ASCII drawing in `maze_sim/scripts/make_maze.py` and run it. It writes `maze_sim/worlds/maze.sdf`. Rebuild `maze_sim` after adding a new world file.
