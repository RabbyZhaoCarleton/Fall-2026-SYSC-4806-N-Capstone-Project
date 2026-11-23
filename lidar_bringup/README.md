# lidar_bringup — RPLIDAR C1 on ROS 2 Jazzy (PC first, Raspberry Pi 5 later)

```
lidar_bringup/
├── udev/99-rplidar.rules          persistent /dev/rplidar name
├── scripts/install_udev.sh        installs rule, dialout group, optional brltty removal
├── config/rplidar_c1.yaml         sllidar_node parameters
├── config/rviz/lidar.rviz         RViz2 preset (Best Effort QoS, fixed frame laser_frame)
├── launch/rplidar_c1.launch.py    driver + static TF + monitor (+ optional RViz)
└── lidar_bringup/
    ├── perf_monitor.py            PerfTracker (perf_counter latency ms + loop Hz), FixedRateLoop
    ├── pinout.py                  central, type-tagged pinout dict + validator
    ├── mock_hardware.py           type-checked mock backend (no GPIO needed)
    ├── scan_monitor_node.py       /scan rate, jitter, latency, age, validity, watchdog
    └── load_generator.py          CPU load to emulate SLAM/Nav2
```

## 1. Build
`setup.sh` (see the top-level README) fetches the Slamtec driver `sllidar_ros2` (pinned in
`capstone.repos`) and builds this package, on the PC and on the Pi (`--robot`). After a `git pull`:
```bash
cd ~/ros2_ws/src/capstone && ./setup.sh
```
To build only the LiDAR packages:
```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-up-to lidar_bringup && source install/setup.bash
```

## 1b. WSL only: the USB device is attached automatically
Windows owns USB devices; WSL only sees the LiDAR after it is passed through with
[usbipd-win](https://github.com/dorssel/usbipd-win). **The launch file now does this for you**
(`auto_attach:=true`, the default). Install usbipd once from PowerShell: `winget install usbipd`.

On every launch (`lidar_bringup/usb_attach.py`):
1. runs `usbipd.exe list` through WSL and finds the LiDAR by **USB ID `10c4:ea60`** (the CP210x
   adapter), not by BUSID, so it works whichever port/hub/laptop it is plugged into;
2. if the device is "Not shared" it runs `usbipd bind` once. That needs admin rights, so a
   **Windows UAC prompt** appears the first time (disable with `auto_bind:=false`);
3. runs `usbipd attach --wsl <this distro> --busid <found BUSID>` unless it is already attached;
4. waits for `/dev/rplidar` (or the single `/dev/ttyUSB*`) and starts the driver on it.

On the Raspberry Pi 5 steps 1-3 are skipped automatically; it only waits for the port.

```bash
ros2 run lidar_bringup attach_lidar --list   # what usbipd sees (BUSIDs, states)
ros2 run lidar_bringup attach_lidar          # attach only; prints the port path
ros2 launch lidar_bringup rplidar_c1.launch.py lidar_busid:=1-1    # force a BUSID
```
**ESP32 on the same machine:** its CP210x chip has the same USB ID. If two devices match, the
code refuses to guess and lists them; choose with `lidar_busid:=<BUSID>` or
`lidar_description:=<text>` (a substring of the description column).

Troubleshooting:
- *"usbipd.exe not found"*: install usbipd-win, then open a new WSL terminal.
- *Attached but no serial port*: `dmesg | tail` should show `cp210x`; if not, run `wsl --update`
  (the WSL kernel lacks the CP210x driver).
- *"WSL failed to parse distributions"* (garbled text): the Windows variable `WSL_UTF8=1` is set.
  The code overrides it for its own calls; for manual use run `$env:WSL_UTF8 = $null` in PowerShell.
- *`/dev/rplidar` missing but `/dev/ttyUSB0` works*: udev isn't active in WSL. Enable systemd
  (`[boot]\nsystemd=true` in `/etc/wsl.conf`, then `wsl --shutdown`) and run `install_udev.sh`.
  Until then the code falls back to the lone ttyUSB device.
- The LiDAR is attached to WSL while the launch runs; `usbipd detach --busid <BUSID>` (PowerShell)
  gives it back to Windows.
- Manual fallback (what the code automates): `usbipd list`, then
  `usbipd bind --busid X` (admin, once) and `usbipd attach --wsl --busid X`.

Unit tests (no hardware): `cd lidar_bringup && python3 -m unittest discover -s test -v`

## 2. Persistent device name
```bash
~/ros2_ws/src/capstone/lidar_bringup/scripts/install_udev.sh   # add --remove-brltty if ttyUSB0 vanishes
# replug the LiDAR (WSL: usbipd detach + attach) so the rule applies
ls -l /dev/rplidar          # -> /dev/rplidar -> ttyUSB0
```
**ESP32 clash:** the C1's adapter (CP210x, 10c4:ea60) is the same chip many ESP32 boards use.
If both are plugged in, switch to Option B (serial) or C (USB port path, `KERNELS==`) in the rule.

## 3. Run
```bash
ros2 launch lidar_bringup rplidar_c1.launch.py                    # driver + TF + monitor
ros2 launch lidar_bringup rplidar_c1.launch.py use_rviz:=true     # PC only
ros2 launch lidar_bringup rplidar_c1.launch.py lidar_x:=0.08 lidar_z:=0.15   # real mount offsets
```

## 4. Validation checklist
| Check | Command | Pass |
|---|---|---|
| Topic exists | `ros2 topic list \| grep scan` | `/scan` listed |
| Rate | `ros2 topic hz /scan --qos-reliability best_effort` | ~10 Hz |
| Content | `ros2 topic echo /scan --once --qos-reliability best_effort` | `frame_id: laser_frame`, 720 ranges (angle_compensate), `range_min` 0.05, `range_max` 16 |
| QoS | `ros2 topic info /scan -v` | publisher is BEST_EFFORT |
| TF | `ros2 run tf2_ros tf2_echo base_link laser_frame` | transform printed |
| RViz | `use_rviz:=true` | wall points appear; Fixed Frame `laser_frame`; scan topic reliability **Best Effort** |
| Monitor | launch output / `ros2 run lidar_bringup scan_monitor` | `[scan_cb] ~10 Hz`, no "BELOW TARGET". One "No /scan messages received yet" warning at startup is normal (the motor is spinning up) |
| Accuracy | tape-measure a flat wall at 0.3/1/3 m | within your 5 cm spec (proposal §4.2) |

RViz showing nothing is almost always QoS (Reliable subscriber vs Best Effort publisher) or a wrong
Fixed Frame.

## 5. Performance under load
```bash
# Terminal A: driver + monitor already running
# Terminal B: emulate SLAM/Nav2 CPU load
ros2 run lidar_bringup load_generator --cores 3 --duration 60
#   (or: stress-ng --cpu 3 --timeout 60s)
# Terminal C: also inject per-scan work in the monitor to find your headroom
ros2 run lidar_bringup scan_monitor --ros-args -p simulated_load_ms:=40.0
```
Healthy: rate stays ≥ ~9.5 Hz, p95 callback latency well under 100 ms (one scan period), zero
deadline misses. If the rate sags only under load, you are CPU-starved (see pitfalls). Use
`PerfTracker` / `FixedRateLoop` from `perf_monitor.py` in your own SLAM, navigation and motor loops.

Pinout / mock checks (no ROS or hardware needed):
```bash
python3 -m lidar_bringup.pinout          # dumps table, flags illegal pin use
python3 -m lidar_bringup.mock_hardware   # 50 Hz mock loop + type-guard demo
```

## 6. Migrating to the Raspberry Pi 5
1. Flash Ubuntu Server 24.04 arm64 for Pi, install the same ROS 2 Jazzy packages (arm64 binaries exist),
   clone and `colcon build` natively on the Pi. No code changes.
2. Re-run `install_udev.sh`; confirm `/dev/rplidar`.
3. Run the driver + monitor on the Pi, RViz on the laptop (same `ROS_DOMAIN_ID`, see pitfalls).
4. Re-run section 4 and 5. Compare Hz/latency to your PC baseline.
5. Edit **only** `pinout.py` for the final wiring; keep control code using logical pin names.

## 7. Pitfalls
**Raspberry Pi 5 / arm64**
- **Missing `/dev/ttyUSB*`:** Ubuntu's Pi kernel ships USB-serial drivers in an extra package:
  `sudo apt install linux-modules-extra-raspi`, then reboot. Check `dmesg | tail` for `cp210x`.
- **GPIO libraries:** `RPi.GPIO` does not work on Pi 5 (RP1 chip). Use `gpiozero` with `lgpio`
  (`sudo apt install python3-gpiozero python3-lgpio`) or libgpiod. The header GPIO chip is `/dev/gpiochip4`
  on some kernels. Hardware PWM needs `dtoverlay=pwm-2chan` in `/boot/firmware/config.txt`.
- **Header UART:** use `/dev/ttyAMA0` (enable `dtparam=uart0=on`); the Pi 5 debug UART is separate.
- **Heat/throttling:** Pi 5 throttles at ~80 °C; use the active cooler. Throttling shows up as scan-rate sag
  under SLAM load (`vcgencmd get_throttled`, `vcgencmd measure_temp`).
- **Build memory/time:** if `colcon build` is slow or OOMs, use `--parallel-workers 2`.
- **RViz on the Pi:** do not. Visualise from the laptop.

**Power**
- The Pi 5 wants a 5 V / 5 A (27 W PD) supply. Without PD negotiation it caps USB output at ~600 mA total
  and prints a power warning. If you power it from a 3S → 5 V buck converter, size it ≥5 A and add
  `usb_max_current_enable=1` to `/boot/firmware/config.txt` so the USB ports can supply 1.6 A.
- The C1 draws a few hundred mA and spikes at motor spin-up (check the Slamtec datasheet). Symptoms of
  brown-out: LiDAR stops spinning, `/scan` goes silent, `dmesg` shows USB disconnect/reconnect.
  Keep servo/arm and drive-motor current on **separate regulators** from the Pi/LiDAR rail.
- Use a short, good-quality data cable. Long or thin USB cables are a classic dropout cause.

**Serial port management (Ubuntu 24.04)**
- The launch file checks the serial port before starting the driver. If it is missing or not
  readable/writable, it prints `Serial port problem: ...` with the fix (usbipd attach on WSL, udev
  rule, permissions) and stops. Without this check the driver only says
  `Error, unexpected error, code: 80008004` and the motor never spins (a bug in the Slamtec SDK hides
  the failed port open).
- **brltty** steals CP210x/CH340 adapters → `ttyUSB0` appears then vanishes. Remove it (`--remove-brltty`).
- Add your user to `dialout` and re-login, or `/dev/rplidar` will be "permission denied" (rule also sets mode 0666).
- Only one process may open the port. Close `minicom`/`screen`/a second driver instance first.
- Ports renumber with plug order (`ttyUSB0/1`); never hard-code them. Always use `/dev/rplidar`.
- Baud is **460800** for the C1 (A1/A2 tutorials use 115200 and will time out).

**ROS 2 networking (PC ↔ Pi)**
- Same `ROS_DOMAIN_ID` on both; DDS multicast discovery is flaky on some Wi-Fi/campus networks.
  Consider `rmw_cyclonedds_cpp` with a unicast peer list.
- `/scan` is Best Effort; `ros2 topic hz/echo` and RViz must use a compatible (Best Effort) QoS.
- Keep clocks synced (`chrony`) or the monitor's "age" value and TF timestamps will be wrong.
- For PC-only testing set `ROS_LOCALHOST_ONLY=1` to avoid chatter on the lab network.

**Sensor / design notes**
- 12 m is for white targets; dark or matte-black surfaces return much less. Test with your actual cardboard/paper walls.
  When you configure `slam_toolbox`, set `max_laser_range` to what you actually measure (a 5 m × 5 m maze
  rarely needs more than ~6–8 m); a shorter limit reduces noise and CPU.
- Blind zone is ~5 cm: mount the LiDAR so walls are not routinely closer than that
  (your 25 cm corridors with a ≥5 cm clearance spec leave little margin, so mount it centred).
- The C1 is a **2D** scanner. Section 2.4 of the proposal calls it "2.5D (detects elevation changes)";
  consider rewording that, since the proposal also lists "Not 3D" as a con.
- The scan rotation is fixed at ~10 Hz and not adjustable on the C1; plan `expected_hz`, Nav2 and SLAM update rates around that.
