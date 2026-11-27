#!/usr/bin/env python3
"""Monitor /odom pose, twist, planar displacement, and signed path distance."""

import math
import sys

import geometry_msgs.msg  # noqa: F401
import nav_msgs.msg  # noqa: F401
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomTester(Node):
    def __init__(self) -> None:
        super().__init__("odom_tester")
        self.start_x: float | None = None
        self.start_y: float | None = None
        self.prev_x: float | None = None
        self.prev_y: float | None = None
        self.signed_path_m = 0.0
        self.latest_msg: Odometry | None = None

        self.create_subscription(Odometry, "/odom", self._on_odom, 50)
        self.create_timer(0.25, self._print_status)

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.start_x is None or self.start_y is None:
            self.start_x = x
            self.start_y = y
            self.prev_x = x
            self.prev_y = y
        elif self.prev_x is not None and self.prev_y is not None:
            # Signed step along heading (positive forward, negative reverse)
            yaw = yaw_from_quat(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            dx = x - self.prev_x
            dy = y - self.prev_y
            step_signed = dx * math.cos(yaw) + dy * math.sin(yaw)
            self.signed_path_m += step_signed
            self.prev_x = x
            self.prev_y = y

        self.latest_msg = msg

    def _print_status(self) -> None:
        if self.latest_msg is None or self.start_x is None or self.start_y is None:
            return
        p = self.latest_msg.pose.pose.position
        t = self.latest_msg.twist.twist
        q = self.latest_msg.pose.pose.orientation
        yaw_deg = math.degrees(yaw_from_quat(q.x, q.y, q.z, q.w))
        net_from_start = math.hypot(p.x - self.start_x, p.y - self.start_y)
        print(
            f"\rpos=({p.x:+6.3f}, {p.y:+6.3f}) yaw={yaw_deg:+6.1f}deg "
            f"vx={t.linear.x:+5.2f}m/s net={net_from_start:6.3f}m signed_path={self.signed_path_m:+7.3f}m",
            end="",
            flush=True,
        )


def main() -> int:
    rclpy.init(args=sys.argv)
    node = OdomTester()
    print("Listening to /odom... Drive forward/reverse, then press Ctrl+C.", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped /odom monitor.", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
