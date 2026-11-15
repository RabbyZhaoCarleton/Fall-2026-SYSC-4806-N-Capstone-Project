#!/usr/bin/env bash
# One-command setup for the capstone ROS 2 workspace.
#
#   ./setup.sh           Desktop or laptop (Ubuntu 24.04 in WSL, or native): simulation included
#   ./setup.sh --robot   Raspberry Pi 5 (Ubuntu 24.04): no Gazebo, no simulation packages
#
# Safe to run again at any time, for example after pulling changes that add dependencies.
# It only uses sudo when something actually needs installing.

set -euo pipefail

MODE=sim
for arg in "$@"; do
  case "$arg" in
    --robot|--pi) MODE=robot ;;
    -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown option: $arg (use --help)"; exit 1 ;;
  esac
done

step() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
warn() { printf '\033[1;33mWARNING:\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31mERROR:\033[0m %s\n' "$*" >&2; exit 1; }

# Ubuntu installs security updates in the background (unattended-upgrades), usually in the
# first minutes after a new install or after WSL starts. Only one program can install packages
# at a time, so wait for it instead of failing with "Could not get lock /var/lib/dpkg/lock-frontend".
APT_OPTS=(-o DPkg::Lock::Timeout=600)
wait_for_apt() {
  local waited=0
  while pgrep -f '^/usr/bin/python3 /usr/bin/unattended-upgrade( |$)' >/dev/null \
        || pgrep -x 'apt|apt-get|dpkg|aptitude' >/dev/null; do
    if [ "$waited" -eq 0 ]; then
      info "Ubuntu is installing its automatic updates in the background. Waiting for it to finish"
      info "(this can take several minutes on a new install; don't close the terminal)..."
    fi
    sleep 10; waited=$((waited + 10))
    if [ $((waited % 60)) -eq 0 ]; then info "still waiting ($((waited / 60)) min)"; fi
    [ "$waited" -lt 1800 ] || die "Still waiting for other package installs after 30 minutes. Restart WSL (wsl --shutdown in PowerShell) and run ./setup.sh again."
  done
}

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_NAME="$(basename "$REPO_DIR")"
SRC_DIR="$(dirname "$REPO_DIR")"
WS="$(dirname "$SRC_DIR")"
IS_WSL=false; grep -qi microsoft /proc/version 2>/dev/null && IS_WSL=true

# ---------------------------------------------------------------- checks
step "Checking the system"
[ "$(id -u)" -ne 0 ] || die "Run this as your normal user, not with sudo. It asks for your password when needed."
# shellcheck disable=SC1091
. /etc/os-release
[ "${VERSION_CODENAME:-}" = "noble" ] || die "This needs Ubuntu 24.04 (found: ${PRETTY_NAME:-unknown})."
case "$REPO_DIR" in
  /mnt/*) die "The repository is on the Windows drive ($REPO_DIR). Clone it inside WSL instead, at ~/ros2_ws/src/capstone (see README.md)." ;;
esac
[ "$(basename "$SRC_DIR")" = "src" ] || die "The repository must be inside a workspace's src folder, e.g. ~/ros2_ws/src/capstone (it is at $REPO_DIR)."
info "Mode:      $MODE"
info "Workspace: $WS"
info "WSL:       $IS_WSL"

# ---------------------------------------------------------------- ROS 2 apt source
step "ROS 2 Jazzy apt repository"
if dpkg -s ros2-apt-source >/dev/null 2>&1 || [ -f /etc/apt/sources.list.d/ros2.sources ] || [ -f /etc/apt/sources.list.d/ros2.list ]; then
  info "already configured"
else
  wait_for_apt
  sudo apt-get "${APT_OPTS[@]}" update
  sudo apt-get "${APT_OPTS[@]}" install -y curl software-properties-common
  sudo add-apt-repository -y universe
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  [ -n "$ROS_APT_SOURCE_VERSION" ] || die "Could not look up the ros-apt-source release. Check the internet connection and try again."
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  wait_for_apt
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get "${APT_OPTS[@]}" update
fi

# ---------------------------------------------------------------- apt packages
step "Base packages"
if [ "$MODE" = sim ]; then
  PKGS=(ros-jazzy-desktop ros-dev-tools git)
else
  PKGS=(ros-jazzy-ros-base ros-dev-tools git)
fi
MISSING=()
for p in "${PKGS[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || MISSING+=("$p"); done
if [ ${#MISSING[@]} -eq 0 ]; then
  info "already installed: ${PKGS[*]}"
else
  info "installing: ${MISSING[*]} (this is the long part, often 10 to 30 minutes)"
  wait_for_apt
  sudo apt-get "${APT_OPTS[@]}" update
  sudo apt-get "${APT_OPTS[@]}" install -y "${MISSING[@]}"
fi

# Load ROS into this script (rosdep and colcon need ROS_DISTRO and the underlay)
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

# ---------------------------------------------------------------- rosdep
step "rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --rosdistro jazzy >/dev/null
info "rosdep database updated"

# ---------------------------------------------------------------- third-party sources
step "Third-party source repositories (capstone.repos)"
cd "$SRC_DIR"
vcs import --input "$REPO_NAME/capstone.repos" .

# ---------------------------------------------------------------- dependencies
step "Dependencies of the workspace packages"
cd "$WS"
if [ "$MODE" = sim ]; then
  DEP_PATHS=(src)
else
  DEP_PATHS=("src/$REPO_NAME/maze_description" "src/$REPO_NAME/maze_navigation" "src/$REPO_NAME/maze_bringup"
             "src/$REPO_NAME/lidar_bringup" src/sllidar_ros2
             src/m-explore-ros2/explore src/m-explore-ros2/explore_lite_msgs)
fi
if rosdep check --from-paths "${DEP_PATHS[@]}" --ignore-src --rosdistro jazzy >/dev/null 2>&1; then
  info "all dependencies already installed"
else
  wait_for_apt
  rosdep install --from-paths "${DEP_PATHS[@]}" --ignore-src -y --rosdistro jazzy
fi

# ---------------------------------------------------------------- build
step "Building the workspace"
# Load the ROS environment again: packages rosdep just installed (such as Gazebo's gz_*_vendor
# packages) add their own paths, which only appear after sourcing. Without this, a first-time
# build fails with: Could not find a package configuration file provided by "gz-sim8".
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
if [ "$MODE" = sim ]; then
  # turtlebot3_gazebo is built from the pinned source in capstone.repos, which may also be installed from apt.
  colcon build --symlink-install --allow-overriding turtlebot3_gazebo \
    --packages-up-to maze_sim maze_description maze_navigation maze_bringup lidar_bringup
else
  colcon build --symlink-install --packages-up-to maze_description maze_navigation maze_bringup lidar_bringup
fi

# ---------------------------------------------------------------- GPU (WSL only)
GPU_LINES=""
if $IS_WSL && [ "$MODE" = sim ]; then
  step "Graphics: choosing the GPU for Gazebo and RViz"
  renderer_for() {  # $1 = adapter name to request; prints the renderer Mesa actually uses
    GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME="$1" \
      python3 "$REPO_DIR/tools/check_gpu.py" 2>/dev/null | sed -n 's/^Renderer: //p' | head -n1
  }
  PS=""
  for c in powershell.exe /mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe; do
    if command -v "$c" >/dev/null 2>&1 || [ -x "$c" ]; then PS="$c"; break; fi
  done
  GPUS=""
  [ -n "$PS" ] && GPUS=$(timeout 30 "$PS" -NoProfile -NonInteractive -Command "(Get-CimInstance Win32_VideoController).Name" < /dev/null 2>/dev/null | tr -d '\r' | sed '/^\s*$/d' || true)
  CANDIDATES=()
  if [ -n "$GPUS" ]; then
    info "GPUs found:"; while IFS= read -r g; do info "  $g"; done <<< "$GPUS"
    # Dedicated GPUs first; integrated ones (Intel Iris/UHD, AMD Radeon(TM) Graphics) last.
    # (each grep may find nothing, hence the "|| true")
    mapfile -t CANDIDATES < <(
      { grep -i 'nvidia' <<< "$GPUS" || true
        grep -i 'arc' <<< "$GPUS" || true
        grep -iE 'radeon (rx|pro)' <<< "$GPUS" || true
        printf '%s\n' "$GPUS"; } | awk 'NF && !seen[$0]++')
  else
    info "Could not list the Windows GPUs; trying common dedicated GPU names instead."
    CANDIDATES=("NVIDIA" "Arc" "Radeon RX" "Radeon Pro" "")
  fi
  CHOSEN_NAME=""; CHOSEN_RENDERER=""
  for cand in "${CANDIDATES[@]}"; do
    r=$(renderer_for "$cand" || true)
    if [[ "$r" == *D3D12* ]] && { [ -z "$cand" ] || [[ "${r,,}" == *"${cand,,}"* ]]; }; then
      CHOSEN_NAME="$cand"; CHOSEN_RENDERER="$r"; break
    fi
  done
  if [ -n "$CHOSEN_RENDERER" ]; then
    info "Using: $CHOSEN_RENDERER"
    GPU_LINES="export GALLIUM_DRIVER=d3d12"
    [ -n "$CHOSEN_NAME" ] && GPU_LINES="$GPU_LINES
export MESA_D3D12_DEFAULT_ADAPTER_NAME=\"$CHOSEN_NAME\""
  else
    warn "No GPU could be used through D3D12. Gazebo and RViz will use software rendering (slower)."
    warn "Updating the Windows graphics driver usually fixes this. Then run ./setup.sh again."
  fi
fi

# ---------------------------------------------------------------- links open in Windows (WSL only)
if $IS_WSL; then
  step "Opening links in the Windows browser"
  OPENER=/mnt/c/Windows/System32/rundll32.exe
  if [ -x "$OPENER" ] && command -v gh >/dev/null 2>&1; then
    # Lets gh open web pages (login, "gh pr view --web", ...) in the Windows default browser.
    gh config set browser "$OPENER url.dll,FileProtocolHandler"
    info "GitHub CLI will open links in your Windows browser"
  else
    info "skipped (GitHub CLI or $OPENER not found)"
  fi
fi

# ---------------------------------------------------------------- ~/.bashrc
step "Shell setup (~/.bashrc)"
BEGIN='# >>> capstone setup (managed by setup.sh) >>>'
END='# <<< capstone setup <<<'
BLOCK="$BEGIN
source /opt/ros/jazzy/setup.bash
[ -f \"$WS/install/setup.bash\" ] && source \"$WS/install/setup.bash\"
export TURTLEBOT3_MODEL=burger"
[ -n "$GPU_LINES" ] && BLOCK="$BLOCK
$GPU_LINES"
[ "$MODE" = sim ] && BLOCK="$BLOCK
alias gzkill='pkill -f \"gz sim\"'"
BLOCK="$BLOCK
$END"
touch ~/.bashrc
if grep -qF "$BEGIN" ~/.bashrc; then
  # Replace the existing block
  awk -v b="$BEGIN" -v e="$END" '$0==b{skip=1} !skip{print} $0==e{skip=0}' ~/.bashrc > ~/.bashrc.capstone.tmp
  mv ~/.bashrc.capstone.tmp ~/.bashrc
fi
printf '\n%s\n' "$BLOCK" >> ~/.bashrc
info "updated"

# ---------------------------------------------------------------- done
step "Done"
if [ "$MODE" = sim ]; then
  cat <<MSG
    Open a NEW terminal (so it picks up ~/.bashrc), then start the simulation:

        ros2 launch maze_sim maze.launch.py explore:=true gui:=false

    See README.md for the other options and for troubleshooting.
MSG
else
  cat <<MSG
    Open a NEW terminal (so it picks up ~/.bashrc). The robot packages are built.
MSG
fi
