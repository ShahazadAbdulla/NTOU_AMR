#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import CompressedImage, Imu, NavSatFix


class StreamTracker:
    def __init__(self, name: str, max_ok_ms: float, warn_ms: float, window_size: int = 500):
        self.name = name
        self.max_ok_ms = max_ok_ms
        self.warn_ms = warn_ms
        self.deltas_ms: deque[float] = deque(maxlen=window_size)
        self.future_count = 0
        self.total_count = 0

    def record(self, host_now_ns: int, msg_stamp_ns: int):
        delta_ms = (host_now_ns - msg_stamp_ns) / 1e6
        self.total_count += 1
        if delta_ms < 0.0:
            self.future_count += 1
        self.deltas_ms.append(delta_ms)

    def summary(self) -> str:
        if not self.deltas_ms:
            return f"{self.name:<8} | No messages received yet"

        vals = sorted(self.deltas_ms)
        n = len(vals)
        mn = vals[0]
        p50 = vals[n // 2]
        p95 = vals[min(n - 1, int(n * 0.95))]
        mx = vals[-1]
        mean = sum(vals) / n
        std = math.sqrt(sum((v - mean) ** 2 for v in vals) / n)

        if self.future_count > 0 and mn < -5.0:
            verdict = "\033[1;31mFAIL (Future Stamp > 5ms)\033[0m"
        elif p95 <= self.max_ok_ms:
            verdict = "\033[1;32mEXCELLENT (tf2 / EKF Ready)\033[0m"
        elif p95 <= self.warn_ms:
            verdict = "\033[1;33mUSABLE (Set transform_tolerance=0.1)\033[0m"
        else:
            verdict = "\033[1;31mHIGH LAG (Check Wi-Fi / AP)\033[0m"

        return (
            f"{self.name:<8} | N={self.total_count:<6d} | "
            f"Min:{mn:+6.2f}ms  P50:{p50:+6.2f}ms  P95:{p95:+6.2f}ms  Max:{mx:+6.2f}ms  "
            f"Jitter:{std:5.2f}ms | Future:{self.future_count} | {verdict}"
        )


class LatencyAuditor(Node):
    def __init__(self):
        super().__init__("project_zero_latency_auditor")

        self.imu_tracker = StreamTracker("IMU", max_ok_ms=20.0, warn_ms=50.0, window_size=800)
        self.cam_tracker = StreamTracker("CAMERA", max_ok_ms=50.0, warn_ms=100.0, window_size=90)
        self.gps_tracker = StreamTracker("GPS", max_ok_ms=250.0, warn_ms=500.0, window_size=20)
        self.sync_state = "UNKNOWN"
        self.skew_ppm = "0.000"
        self.rtt_ms = "0.00"

        imu_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        rel_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(Imu, "/project_zero/imu/data", self._on_imu, imu_qos)
        self.create_subscription(CompressedImage, "/project_zero/camera/image/compressed", self._on_cam, rel_qos)
        self.create_subscription(NavSatFix, "/project_zero/gps/fix", self._on_gps, rel_qos)
        self.create_subscription(DiagnosticStatus, "/project_zero/clock/status", self._on_diag, rel_qos)

        self.create_timer(1.0, self._print_report)

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _on_imu(self, msg: Imu):
        now_ns = self.get_clock().now().nanoseconds
        self.imu_tracker.record(now_ns, self._stamp_to_ns(msg.header.stamp))

    def _on_cam(self, msg: CompressedImage):
        now_ns = self.get_clock().now().nanoseconds
        self.cam_tracker.record(now_ns, self._stamp_to_ns(msg.header.stamp))

    def _on_gps(self, msg: NavSatFix):
        now_ns = self.get_clock().now().nanoseconds
        self.gps_tracker.record(now_ns, self._stamp_to_ns(msg.header.stamp))

    def _on_diag(self, msg: DiagnosticStatus):
        self.sync_state = msg.message
        for kv in msg.values:
            if kv.key == "skew_ppm":
                self.skew_ppm = kv.value
            elif kv.key == "path_delay_min_ns":
                self.rtt_ms = f"{int(kv.value) / 1e6:.2f}"

    def _print_report(self):
        print("\033[2J\033[H", end="")  # Clear screen cleanly
        print("=========================================================================================================")
        print(f" PROJECT ZERO — LIVE ROS 2 TIMESTAMP & TF2 LATENCY AUDIT   |   Sync: {self.sync_state} (RTT: {self.rtt_ms} ms, Skew: {self.skew_ppm} ppm)")
        print("=========================================================================================================")
        print(self.imu_tracker.summary())
        print(self.cam_tracker.summary())
        print(self.gps_tracker.summary())
        print("---------------------------------------------------------------------------------------------------------")
        print(" Delta = (Host ROS Now - Sensor Header Stamp). Positive = Causal (Good). Negative = Future Stamp (Bad).")


def main():
    rclpy.init()
    node = LatencyAuditor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
