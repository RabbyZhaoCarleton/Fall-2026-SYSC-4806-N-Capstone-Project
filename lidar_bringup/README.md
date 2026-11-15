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

## 1. Prerequisites (Ubuntu 24.04 + ROS 2 Jazzy, same steps on PC and Pi 5)
```bash
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions git   # Pi: ros-jazzy-ros-base
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git      # vendor driver (build from source)
cp -r /path/to/lidar_bringup .                              # this package
cd ~/ws && rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install && source install/setup.bash
```
If the Slamtec repo's default branch does not build on Jazzy, check its Issues/branches; the
node/param names used here (`sllidar_node`, `serial_port`, `serial_baudrate`, `frame_id`,
`inverted`, `angle_compensate`, `scan_mode`) match its C1 launch file.

## 2. Persistent device name
```bash
./src/lidar_bringup/scripts/install_udev.sh        # add --remove-brltty if ttyUSB0 vanishes
# log out/in (dialout group), replug the LiDAR
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
| Content | `ros2 topic echo /scan --once --qos-reliability best_effort` | `frame_id: laser_frame`, ~500 ranges, `range_max` about 12 |
| QoS | `ros2 topic info /scan -v` | publisher is BEST_EFFORT |
| TF | `ros2 run tf2_ros tf2_echo base_link laser_frame` | transform printed |
| RViz | `use_rviz:=true` | wall points appear; Fixed Frame `laser_frame`; scan topic reliability **Best Effort** |
| Monitor | launch output / `ros2 run lidar_bringup scan_monitor` | `[scan_cb] ~10 Hz`, no "BELOW TARGET" |
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
