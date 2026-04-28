#!/usr/bin/env python3
"""Looping ROS2 joint test publisher for FFW teleop pipeline."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class LoopJointTest(Node):
    def __init__(self, hz: float) -> None:
        super().__init__("loop_joint_test")
        self.pub_left = self.create_publisher(
            JointTrajectory,
            "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
            10,
        )
        self.pub_right = self.create_publisher(
            JointTrajectory,
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
            10,
        )
        self.pub_lift = self.create_publisher(
            JointTrajectory,
            "/leader/joystick_controller_right/joint_trajectory",
            10,
        )
        self.t0 = time.monotonic()
        period = 1.0 / max(hz, 1.0)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(f"Loop joint test started ({hz:.1f} Hz). Ctrl-C to stop.")

    def _make_traj(self, names: list[str], positions: list[float]) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = names
        p = JointTrajectoryPoint()
        p.positions = positions
        msg.points = [p]
        return msg

    def tick(self) -> None:
        t = time.monotonic() - self.t0

        # Smooth visible motion for the left arm.
        l2 = 0.9 + 0.45 * math.sin(0.8 * t)
        l3 = -0.8 + 0.55 * math.sin(0.8 * t + 1.2)
        l4 = 0.6 + 0.35 * math.sin(0.8 * t + 2.1)
        self.pub_left.publish(
            self._make_traj(
                ["arm_l_joint2", "arm_l_joint3", "arm_l_joint4"],
                [l2, l3, l4],
            )
        )

        # Mirror-ish right arm movement.
        r2 = -0.9 + 0.45 * math.sin(0.8 * t + 0.6)
        r3 = 0.8 + 0.55 * math.sin(0.8 * t + 1.8)
        r4 = 0.6 + 0.35 * math.sin(0.8 * t + 2.6)
        self.pub_right.publish(
            self._make_traj(
                ["arm_r_joint2", "arm_r_joint3", "arm_r_joint4"],
                [r2, r3, r4],
            )
        )

        # Lift is authored in [-0.5, 0.0] in Scene.usda.
        lift = -0.25 + 0.2 * math.sin(0.4 * t)
        self.pub_lift.publish(self._make_traj(["lift_joint"], [lift]))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hz", type=float, default=100.0, help="Publish rate for looping test")
    args = parser.parse_args()

    rclpy.init()
    node = LoopJointTest(args.hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
