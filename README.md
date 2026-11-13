# Fall-2026-SYSC-4806-N-Capstone-Project

An autonomous robot that maps an unknown grid maze with a 2D lidar (SLAM), plans a time-optimal sequence for delivering objects to the right exits, and carries it out with a 6-DOF arm. This repository holds the ROS 2 Jazzy packages for both the Gazebo simulation and the real robot (Raspberry Pi 5).

| Package | Purpose | Simulation (PC) | Raspberry Pi 5 |
|---|---|---|---|
| `maze_description` | Robot model (URDF/xacro) | yes | yes |
| `maze_navigation` | slam_toolbox, Nav2 and explore_lite configuration | yes | yes |
| `maze_bringup` | Hardware drivers and robot launch files | no | yes |
| `maze_sim` | Gazebo worlds, maze generator, simulation launch files | yes | no |
| `maze_monitor` | Debugging monitor: live analysis, run recording, replay, run management | yes | no (run it on the PC) |

Third-party source packages (TurtleBot3 simulation, m-explore-ros2) are pinned in `capstone.repos` and fetched by `setup.sh`.

## Getting started on Windows

Everything runs inside WSL (Windows Subsystem for Linux) with Ubuntu 24.04. Expect about 30 to 60 minutes the first time, mostly downloads.

### 1. Install Ubuntu 24.04 in WSL

Open **PowerShell as administrator** and run:

```powershell
wsl --install -d Ubuntu-24.04
```

Restart if asked. When Ubuntu opens for the first time, choose a username and password. The password is what `sudo` asks for later.

If you already have WSL, check that you have Ubuntu 24.04 with `wsl -l -v`.

### 2. Sign in to GitHub inside Ubuntu

The repository is private, so Ubuntu needs your GitHub login. In the **Ubuntu** terminal, install the GitHub CLI and tell it how to open your Windows browser. Linux in WSL has no browser of its own, so without the second line the login can't open the GitHub page and fails.

```bash
sudo apt update && sudo apt install -y gh git
gh config set browser "/mnt/c/Windows/System32/rundll32.exe url.dll,FileProtocolHandler"
gh auth login
```

Choose GitHub.com, HTTPS, Yes (authenticate Git), and "Login with a web browser". Copy the one-time code it shows, then press Enter. GitHub opens in your Windows browser; paste the code there and approve. Back in the terminal, wait for "Authentication complete". Then let git use that login:

```bash
gh auth setup-git
git config --global user.name "Your Name"
git config --global user.email "your-email@cmail.carleton.ca"
```

### 3. Clone the repository into a workspace

The repository must be cloned to exactly this location, inside the Linux file system (not under `/mnt/c`):

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
gh repo clone RabbyZhaoCarleton/Fall-2026-SYSC-4806-N-Capstone-Project capstone
```

### 4. Run the setup script

```bash
cd ~/ros2_ws/src/capstone
./setup.sh
```

It installs ROS 2 Jazzy and all dependencies, fetches the third-party packages, builds the workspace, picks your dedicated GPU for Gazebo and RViz, and adds the needed lines to `~/.bashrc`. It asks for your Ubuntu password when it needs to install something.

### 5. Run the simulation

Open a **new** Ubuntu terminal, then:

```bash
ros2 launch maze_sim maze.launch.py explore:=true gui:=false
```

RViz opens and the robot starts exploring the maze on its own.

## Editing the code

Use VS Code with the **WSL** extension. From the Ubuntu terminal:

```bash
code ~/ros2_ws/src/capstone
```

This edits the files inside Linux directly. Don't edit them through `\\wsl$` from Windows programs or keep a second copy on the C: drive.

## Day-to-day workflow

```bash
cd ~/ros2_ws/src/capstone
git pull
./setup.sh        # safe to rerun; picks up new dependencies and rebuilds
```

If you only changed code in existing packages, rebuilding is enough:

```bash
cd ~/ros2_ws && colcon build --symlink-install
```

Run `colcon build` from `~/ros2_ws`, never from inside `src`. After a package is added, open a new terminal (or run `source ~/ros2_ws/install/setup.bash`) so ROS can find it.

Work on a branch and open a pull request instead of pushing to `main` directly:

```bash
git switch -c feature/short-description
# ...commit your changes...
git push -u origin feature/short-description
gh pr create
```

GitHub builds every push and pull request automatically (the **build** check). Please make sure it passes before merging.

## Using the simulation

```bash
ros2 launch maze_sim maze.launch.py [arguments]
```

| Argument | Default | Meaning |
|---|---|---|
| `explore` | `false` | Start Nav2 and explore_lite for autonomous mapping |
| `nav` | `false` | Start Nav2 only, to send goals yourself |
| `gui` | `true` | Open the Gazebo window (the simulation runs either way) |
| `rviz` | `true` | Open RViz |
| `world` | `maze_sim/worlds/maze.sdf` | World file to load |
| `x_pose`, `y_pose` | `0.25`, `-0.25` | Robot start position in metres |
| `monitor` | `true` | Open the maze monitor (see below) |
| `monitor_record` | `true` | Save each run to `~/maze_runs` |

See all arguments with `ros2 launch maze_sim maze.launch.py --show-args`.

**Send a goal:** launch with `nav:=true`, then use **2D Goal Pose** in RViz (click, and drag to set the heading).

**Save the map** while the simulation is still running, from a second terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maze_map
```

**Pause or resume exploration** without restarting:

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
```

**Change the maze:** edit the ASCII drawing in `maze_sim/scripts/make_maze.py` and run it. It writes `maze_sim/worlds/maze.sdf`. Rebuild after adding a new world file.

## Debugging with the maze monitor

The maze monitor opens with the simulation. It explains in plain language what the robot is doing, flags problems such as collisions, getting stuck, failed plans, and localization jumps, and records every run so that you can replay it with a timeline.

To look at saved runs later, without the simulation:

```bash
ros2 run maze_monitor maze_monitor --no-ros --replay latest
```

See the [user manual](maze_monitor/docs/user-manual.md) and the [design document](maze_monitor/docs/design.md).

## Troubleshooting

**`gh auth login` fails** with "Failed opening a web browser" and then "Too many requests ... (slow_down)": the GitHub CLI couldn't open a browser, and GitHub then blocked the login for a few minutes. Run the `gh config set browser ...` line from step 2, wait about 5 minutes, and run `gh auth login` again. If the terminal still shows an error after you approved in the browser, run `gh auth status`; the login often went through anyway.

**"Could not get lock /var/lib/dpkg/lock-frontend ... held by process ... (unattended-upgr)"**: Ubuntu is installing its automatic updates in the background, which it often does right after a new install. `setup.sh` now waits for this by itself; if you see the error from another command, wait until `pgrep -af /usr/bin/unattended-upgrade` prints nothing, then run the command again. Don't delete the lock files or kill the update.

**`package '...' not found`**: the terminal was opened before the package was built. Open a new terminal, or run `source ~/ros2_ws/install/setup.bash`.

**"A Gazebo server is already running"**, the wrong world appears, or the robot stutters: an earlier simulation is still running in the background. Stop it with `gzkill`, then launch again.

**Gazebo and RViz are slow**: check which GPU they use. `llvmpipe` means the CPU is drawing everything.

```bash
python3 ~/ros2_ws/src/capstone/tools/check_gpu.py
```

If it says `llvmpipe`, open a new terminal and check again. If it still does, rerun `./setup.sh`.

**RViz shows everything but ignores clicks**: minimize RViz and restore it. If that doesn't help, close everything, run `wsl --shutdown` in Windows PowerShell, and open a new Ubuntu terminal.

**The Gazebo window crashes**: the simulation keeps running without it. Use `gui:=false`; RViz shows everything needed.

## Raspberry Pi 5 (real robot)

On the Pi, running Ubuntu 24.04, clone the repository to the same place and run the script in robot mode. It skips Gazebo and the simulation packages:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
gh repo clone RabbyZhaoCarleton/Fall-2026-SYSC-4806-N-Capstone-Project capstone
cd capstone && ./setup.sh --robot
```
