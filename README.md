# Fall-2026-SYSC-4806-N-Capstone-Project

An autonomous robot that maps an unknown grid maze with a 2D lidar (SLAM), plans how to deliver objects to the right exits, and does it with a 6-DOF arm. ROS 2 Jazzy packages for the Gazebo simulation and the real robot (Raspberry Pi 5).

| Package | Purpose |
|---|---|
| `maze_description` | Robot model (URDF/xacro) |
| `maze_navigation` | slam_toolbox, Nav2 and explore_lite configuration |
| `maze_bringup` | Hardware drivers and launch files for the real robot |
| `maze_sim` | Gazebo worlds, maze generator, simulation launch files |

## Setup

Windows users work inside WSL with Ubuntu 24.04. In **PowerShell as administrator**:

```powershell
wsl --install -d Ubuntu-24.04
```

Then, in the **Ubuntu** terminal, sign in to GitHub (the repository is private):

```bash
sudo apt update && sudo apt install -y gh git
gh config set browser "/mnt/c/Windows/System32/rundll32.exe url.dll,FileProtocolHandler"
gh auth login        # GitHub.com, HTTPS, authenticate Git: yes, web browser
gh auth setup-git
```

Clone to exactly this location (inside Linux, not under `/mnt/c`) and run the setup script:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
gh repo clone RabbyZhaoCarleton/Fall-2026-SYSC-4806-N-Capstone-Project capstone
cd capstone && ./setup.sh
```

`setup.sh` installs ROS 2 Jazzy and all dependencies, fetches the pinned third-party packages, builds the workspace and updates `~/.bashrc`. Open a **new** terminal afterwards. On the Raspberry Pi use `./setup.sh --robot` (skips Gazebo and simulation packages).

Edit the code with VS Code's **WSL** extension: `code ~/ros2_ws/src/capstone`.

## Updating, switching branches and running

```bash
cd ~/ros2_ws/src/capstone
git pull                    # or: git switch <branch> && git pull
./setup.sh                  # safe to rerun; picks up new dependencies and rebuilds
```

If only code in existing packages changed, rebuilding is enough:

```bash
cd ~/ros2_ws && colcon build --symlink-install
```

Always build from `~/ros2_ws`, never from inside `src`, and open a new terminal after a package is added. If switching branches leaves errors like `package '...' not found`, do a clean build: `cd ~/ros2_ws && rm -rf build install log && colcon build --symlink-install`.

Then open the monitor, which starts, stops and restarts simulation runs for you:

```bash
ros2 run maze_monitor maze_monitor
```

Work on a branch and open a pull request rather than pushing to `main`; the **build** check must pass before merging.

## Using the simulation

The easiest way is the maze monitor, which works as a run manager:

```bash
ros2 run maze_monitor maze_monitor
```

Pick the settings in its **Run** panel (explore or drive yourself, Gazebo window, RViz, world, start pose, single Nav2/SLAM/explore_lite parameters) and press **Start** (F5), **Stop** (Shift+F5) or **Restart** (Ctrl+F5). Every run is recorded to `~/maze_runs` with its settings, parameter files and launch output. **Robot control** pauses and resumes exploration (F3) and drives the robot with W A S D or the arrow keys (F2). Save settings you use often as presets, then `ros2 run maze_monitor maze_monitor --preset NAME --start`. See `maze_monitor/docs/user-manual.md`.

Or start a run from a terminal, which also opens a monitor that records it:

```bash
ros2 launch maze_sim maze.launch.py [argument:=value ...]
```

| Argument | Default | Meaning |
|---|---|---|
| `explore` | `false` | Start Nav2 and explore_lite to map the maze autonomously |
| `nav` | `false` | Start Nav2 only; send goals with **2D Goal Pose** in RViz |
| `gui` | `true` | Open the Gazebo window (the simulation runs either way) |
| `rviz` | `true` | Open RViz |
| `slam` | `true` | Start slam_toolbox |
| `world` | `maze_sim/worlds/maze.sdf` | World file to load |
| `x_pose`, `y_pose` | `0.25`, `-0.25` | Robot start position (m) |
| `monitor` | `true` | Open a monitor that records this run (the run manager passes `monitor:=false`) |

`--show-args` lists every argument, including the parameter-file overrides.

Useful commands while it runs (from a second terminal):

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maze_map                       # save the map
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"     # pause exploring (true resumes)
```

**Change the maze:** edit the ASCII drawing in `maze_sim/scripts/make_maze.py` and run it; it writes `maze_sim/worlds/maze.sdf`.

**Common problems:** "A Gazebo server is already running" or the robot stutters → run `gzkill`. Gazebo/RViz slow → `python3 tools/check_gpu.py`; if it says `llvmpipe`, rerun `./setup.sh`. Gazebo window crashes → use `gui:=false`. RViz ignores clicks → minimize and restore it, or `wsl --shutdown` from PowerShell.
