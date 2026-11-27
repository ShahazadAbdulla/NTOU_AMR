#!/usr/bin/env python3
"""Measure total robot distance travelled from /joint_states wheel rotations."""

import argparse
import math
import sys

import geometry_msgs.msg  # noqa: F401
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

WHEEL_JOINTS = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_right_wheel_joint",
    "rear_left_wheel_joint",
)


def shortest_angle_delta(prev: float, curr: float) -> float:
    diff = curr - prev
    return math.atan2(math.sin(diff), math.cos(diff))


class WheelDistanceTester(Node):
    def __init__(self, wheel_radius: float, use_abs: bool) -> None:
        super().__init__("wheel_distance_tester")
        self.wheel_radius = wheel_radius
        self.use_abs = use_abs
        self.prev_pos: dict[str, float] = {}
        self.total_distance_m = 0.0
        self.samples = 0

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)
        self.create_timer(0.25, self._print_status)

    def _on_joint_state(self, msg: JointState) -> None:
        name_to_pos = {
            name: pos
            for name, pos in zip(msg.name, msg.position)
            if name in WHEEL_JOINTS and math.isfinite(pos)
        }
        if not name_to_pos:
            return

        if not self.prev_pos:
            self.prev_pos = dict(name_to_pos)
            return

        step_distances: list[float] = []
        for joint, pos in name_to_pos.items():
            if joint in self.prev_pos:
                d_rad = shortest_angle_delta(self.prev_pos[joint], pos)
                d_m = abs(d_rad) * self.wheel_radius if self.use_abs else d_rad * self.wheel_radius
                step_distances.append(d_m)
            self.prev_pos[joint] = pos

        if step_distances:
            self.total_distance_m += sum(step_distances) / len(step_distances)
            self.samples += 1

    def _print_status(self) -> None:
        print(f"\rRobot distance: {self.total_distance_m:7.3f} m", end="", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Test robot travel distance from wheel joints")
    parser.add_argument(
        "--wheel-radius",
        type=float,
        required=True,
        help="Wheel radius in meters (e.g., 0.065)",
    )
    parser.add_argument(
        "--signed",
        action="store_true",
        help="Accumulate signed forward/reverse distance instead of magnitude",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=sys.argv[:1] + ros_args)
    node = WheelDistanceTester(wheel_radius=args.wheel_radius, use_abs=not args.signed)
    print(
        f"Tracking /joint_states with wheel_radius={args.wheel_radius} m. "
        "Drive 3.0 m, stop, then press Ctrl+C.",
        flush=True,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\nFinal robot distance: {node.total_distance_m:.4f} m", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
