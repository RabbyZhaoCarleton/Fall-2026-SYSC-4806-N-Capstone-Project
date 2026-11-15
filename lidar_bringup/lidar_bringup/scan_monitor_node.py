"""ROS 2 node that verifies the /scan topic is alive and measures its health.

Reports every `report_period_s`:
  * scan rate (Hz) vs expected (C1 ~ 10 Hz) and period jitter
  * callback processing latency (ms)  - how long OUR work takes
  * message age (ms)                  - now minus header.stamp (transport + driver delay)
  * valid-point percentage, min/max range, point count
  * watchdog warning if the topic goes silent

`simulated_load_ms` lets you inject fake per-scan work (like a SLAM step) to see how much
headroom the pipeline has before scans start being dropped.

NOTE on message age: it is only meaningful if the publisher and this node share a clock
(same machine, or chrony/NTP-synced machines). Otherwise ignore that column.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data   # BEST_EFFORT, matches sllidar_node
from sensor_msgs.msg import LaserScan

from .perf_monitor import PerfTracker


class ScanMonitor(Node):
    def __init__(self) -> None:
        super().__init__("scan_monitor")

        # ---- Parameters ----------------------------------------------------
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("expected_hz", 10.0)        # RPLIDAR C1 nominal
        self.declare_parameter("min_hz_warn", 8.0)         # warn below this rate
        self.declare_parameter("report_period_s", 2.0)
        self.declare_parameter("watchdog_s", 1.0)          # silence threshold
        self.declare_parameter("simulated_load_ms", 0.0)   # fake per-scan CPU work
        self.declare_parameter("expected_frame_id", "laser_frame")

        topic = self.get_parameter("scan_topic").value
        self.expected_hz = float(self.get_parameter("expected_hz").value)
        self.min_hz_warn = float(self.get_parameter("min_hz_warn").value)
        self.watchdog_s = float(self.get_parameter("watchdog_s").value)
        self.sim_load_s = float(self.get_parameter("simulated_load_ms").value) / 1e3
        self.expected_frame = self.get_parameter("expected_frame_id").value

        # ---- State ---------------------------------------------------------
        # Deadline = one scan period: if processing takes longer, scans will be dropped.
        self.tracker = PerfTracker("scan_cb", target_hz=self.expected_hz)
        self.age_ms = 0.0
        self.valid_pct = math.nan
        self.n_points = 0
        self.rmin = self.rmax = math.nan
        self.last_rx = None
        self.warned_frame = False

        self.create_subscription(LaserScan, topic, self._on_scan, qos_profile_sensor_data)
        self.create_timer(float(self.get_parameter("report_period_s").value), self._report)
        self.create_timer(0.25, self._watchdog)
        self.get_logger().info(f"Monitoring '{topic}' (expect ~{self.expected_hz:.0f} Hz)")

    # ------------------------------------------------------------------ callback
    def _on_scan(self, msg: LaserScan) -> None:
        self.tracker.start()
        self.last_rx = time.perf_counter()

        if msg.header.frame_id != self.expected_frame and not self.warned_frame:
            self.get_logger().warn(f"frame_id is '{msg.header.frame_id}', expected "
                                   f"'{self.expected_frame}' - TF/SLAM will break")
            self.warned_frame = True

        # Message age vs header stamp (see module docstring caveat)
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.age_ms = (self.get_clock().now().nanoseconds - stamp_ns) / 1e6

        # Light-weight scan quality metrics
        r = np.asarray(msg.ranges, dtype=np.float32)
        valid = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        self.n_points = r.size
        self.valid_pct = 100.0 * float(valid.sum()) / max(1, r.size)
        if valid.any():
            self.rmin, self.rmax = float(r[valid].min()), float(r[valid].max())

        if self.sim_load_s > 0.0:                    # emulate SLAM-style work
            t_end = time.perf_counter() + self.sim_load_s
            while time.perf_counter() < t_end:
                pass

        self.tracker.stop()

    # --------------------------------------------------------------------- timers
    def _report(self) -> None:
        if self.tracker.total_iterations == 0:
            self.get_logger().warn("No /scan messages received yet "
                                   "(driver not running? wrong QoS? ROS_DOMAIN_ID?)")
            return
        s = self.tracker.stats()
        line = (f"{self.tracker.report()} | age {self.age_ms:.1f} ms | pts {self.n_points} | "
                f"valid {self.valid_pct:.0f}% | range {self.rmin:.2f}-{self.rmax:.2f} m")
        if s["freq_hz"] < self.min_hz_warn:
            self.get_logger().warn(line)
        else:
            self.get_logger().info(line)

    def _watchdog(self) -> None:
        if self.last_rx is not None and time.perf_counter() - self.last_rx > self.watchdog_s:
            self.get_logger().error(f"/scan silent for >{self.watchdog_s:.1f}s "
                                    "(USB dropout / brown-out / driver crash?)")
            self.last_rx = time.perf_counter()    # rate-limit the message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Final: " + node.tracker.report())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
