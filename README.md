# Fall-2026-SYSC-4806-N-Capstone-Project

An autonomous robot that maps an unknown grid maze with a 2D lidar (SLAM), plans how to deliver objects to the right exits, and does it with a 6-DOF arm. ROS 2 Jazzy packages for the Gazebo simulation and the real robot (Raspberry Pi 5).

| Package | Purpose |
|---|---|
| `maze_description` | Robot model (URDF/xacro) |
| `maze_navigation` | slam_toolbox, Nav2 and explore_lite configuration |
| `maze_bringup` | Hardware drivers and launch files for the real robot |
| `maze_sim` | Gazebo worlds, maze generator, simulation launch files |
| `lidar_bringup` | RPLIDAR C1 driver launch, udev rule and scan monitor (PC and Pi) |

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

`setup.sh` installs ROS 2 Jazzy and all dependencies, fetches the pinned third-party packages (including the Slamtec `sllidar_ros2` driver), builds the workspace and updates `~/.bashrc`. Open a **new** terminal afterwards. On the Raspberry Pi use `./setup.sh --robot` (skips Gazebo and simulation packages).

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

**Change the maze:** edit the ASCII drawing in `maze_sim/scripts/make_maze.py` and run it; it writes `maze_sim/worlds/maze.sdf`.

**Common problems:** "A Gazebo server is already running" or the robot stutters → run `gzkill`. Gazebo/RViz slow → `python3 tools/check_gpu.py`; if it says `llvmpipe`, rerun `./setup.sh`. Gazebo window crashes → use `gui:=false`. RViz ignores clicks → minimize and restore it, or `wsl --shutdown` from PowerShell.

## Starting the lidar (RPLIDAR C1)

**WSL only:** Windows owns the USB port, so pass the lidar through to Ubuntu with [usbipd-win](https://github.com/dorssel/usbipd-win) (`winget install usbipd` once). In PowerShell:

```powershell
usbipd list                          # find the "Silicon Labs CP210x" row, e.g. BUSID 2-4
usbipd bind --busid 2-4              # once per device, in an ADMIN PowerShell
usbipd attach --wsl --busid 2-4      # every time the lidar is plugged in or WSL restarts
```

Once per machine (PC or Pi), install the udev rule so the lidar always appears as `/dev/rplidar`, then replug it (WSL: `usbipd detach --busid 2-4`, then attach again):

```bash
~/ros2_ws/src/capstone/lidar_bringup/scripts/install_udev.sh   # add --remove-brltty if ttyUSB0 vanishes
ls -l /dev/rplidar                                            # -> ttyUSB0
```

Start the driver, the `base_link → laser_frame` transform and the scan monitor:

```bash
ros2 launch lidar_bringup rplidar_c1.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `use_rviz` | `false` | Open RViz showing `/scan` (PC only, not on the Pi) |
| `use_monitor` | `true` | Report scan rate, latency and dropouts |
| `serial_port` | `/dev/rplidar` | Serial device, e.g. `/dev/ttyUSB0` before the udev rule is installed |
| `lidar_x`, `lidar_y`, `lidar_z`, `lidar_yaw` | `0`, `0`, `0.10`, `0` | Mounting pose on the robot (m, rad) |

Check it works (from a second terminal; `/scan` is Best Effort, so the tools need the same QoS):

```bash
ros2 topic hz /scan --qos-reliability best_effort      # about 10 Hz
```

One "No /scan messages received yet" warning at startup is normal while the motor spins up. Validation, load testing, the Pi and power notes are in the [lidar_bringup README](lidar_bringup/README.md).

**Common problems:** `Serial port problem: ...` → follow the fix it prints (usually `usbipd attach` on WSL, or the udev rule). `usbipd attach` fails with "WSL failed to parse distributions" → run `$env:WSL_UTF8 = $null` in that PowerShell and retry. RViz shows no points → set the scan topic's reliability to **Best Effort** and Fixed Frame to `laser_frame`. Port busy → close any other program or driver instance using it.
