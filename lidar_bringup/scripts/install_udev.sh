#!/usr/bin/env bash
# Installs the RPLIDAR udev rule and fixes common Ubuntu 24.04 serial-port problems.
# Usage: ./scripts/install_udev.sh [--remove-brltty]
set -euo pipefail

RULE_SRC="$(dirname "$(readlink -f "$0")")/../udev/99-rplidar.rules"
RULE_DST="/etc/udev/rules.d/99-rplidar.rules"

echo "[1/4] Installing udev rule -> ${RULE_DST}"
sudo cp "${RULE_SRC}" "${RULE_DST}"

echo "[2/4] Adding ${USER} to 'dialout' (re-login required to take effect)"
sudo usermod -aG dialout "${USER}"

echo "[3/4] Reloading udev"
sudo udevadm control --reload-rules
sudo udevadm trigger

if [[ "${1:-}" == "--remove-brltty" ]]; then
  # brltty (braille display daemon) grabs CP210x/CH340 adapters on Ubuntu and makes
  # /dev/ttyUSB* vanish seconds after plug-in.
  echo "[4/4] Removing brltty"
  sudo apt-get remove -y brltty
else
  echo "[4/4] Skipped brltty removal (pass --remove-brltty if ttyUSB0 disappears)"
fi

echo
echo "Unplug/replug the LiDAR, then check:  ls -l /dev/rplidar"
