# Fall-2026-SYSC-4806-N-Capstone-Project

An autonomous robot that maps an unknown grid maze with a 2D lidar (SLAM), plans how to deliver objects to the right exits, and does it with a 6-DOF arm. ROS 2 Jazzy packages for the Gazebo simulation and the real robot (Raspberry Pi 5).

| Package | Purpose |
|---|---|
| `maze_description` | Robot model (URDF/xacro) |
| `maze_navigation` | slam_toolbox, Nav2 and explore_lite configuration |
| `maze_bringup` | Hardware drivers and launch files for the real robot |
| `maze_sim` | Gazebo worlds, maze generator, simulation launch files |
| `maze_interfaces` | Messages and services shared between nodes (objects, exit bins, delivery plan) |
| `maze_planning` | Global path planning: the fastest order to deliver the objects to their exits |

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

Then run the simulation:

```bash
ros2 launch maze_sim maze.launch.py explore:=true gui:=false
```

Work on a branch and open a pull request rather than pushing to `main`; the **build** check must pass before merging.

## Using the simulation

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

`--show-args` lists every argument, including the parameter-file overrides.

Useful commands while it runs (from a second terminal):

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maze_map                       # save the map
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"     # pause exploring (true resumes)
```

**Delivery (global planning) simulations** (details: `maze_planning/docs/global-planner-guide.md`):

```bash
ros2 launch maze_planning sim_lightweight.launch.py      # seconds: fake maze/robot, no Gazebo
ros2 launch maze_planning sim_delivery.launch.py         # Gazebo: explore, then deliver
ros2 run maze_planning plan_offline --benchmark 20       # check plans against brute force
```

**Change the maze:** edit the ASCII drawing in `maze_sim/scripts/make_maze.py` and run it; it writes `maze_sim/worlds/maze.sdf`.

**Common problems:** "A Gazebo server is already running" or the robot stutters → run `gzkill`. Gazebo/RViz slow → `python3 tools/check_gpu.py`; if it says `llvmpipe`, rerun `./setup.sh`. Gazebo window crashes → use `gui:=false`. RViz ignores clicks → minimize and restore it, or `wsl --shutdown` from PowerShell.
