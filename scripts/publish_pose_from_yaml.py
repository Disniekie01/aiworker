#!/usr/bin/env python3
"""Publish multi-step arm poses from a YAML config."""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

FULL_JOINT_ORDER = [
    "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
    "arm_l_joint5", "arm_l_joint6", "arm_l_joint7", "gripper_l_joint1",
    "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
    "arm_r_joint5", "arm_r_joint6", "arm_r_joint7", "gripper_r_joint1",
    "lift_joint", "head_joint1", "head_joint2",
]


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError("Top-level YAML must be a mapping.")
    return data


def _extract_executor(root: dict[str, Any], key: str) -> dict[str, Any]:
    any_ns = root.get("/**", {})
    exe = any_ns.get(key, {})
    params = exe.get("ros__parameters", {})
    if not isinstance(params, dict):
        raise ValueError(f"Invalid ros__parameters for {key}")
    return params


def _traj(names: list[str], positions: list[float]) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = names
    point = JointTrajectoryPoint()
    point.positions = [float(x) for x in positions]
    msg.points = [point]
    return msg


class PosePublisher(Node):
    def __init__(self, cfg_path: Path) -> None:
        super().__init__("pose_yaml_publisher")
        data = _load_yaml(cfg_path)
        self.left = _extract_executor(data, "arm_l_joint_trajectory_executor")
        self.right = _extract_executor(data, "arm_r_joint_trajectory_executor")

        left_topic = str(self.left.get("action_topic", "")).strip()
        right_topic = str(self.right.get("action_topic", "")).strip()
        if not left_topic or not right_topic:
            raise ValueError("Both arm action_topic values are required.")

        self.pub_left = self.create_publisher(JointTrajectory, left_topic, 10)
        self.pub_right = self.create_publisher(JointTrajectory, right_topic, 10)
        self.pub_joint_targets = self.create_publisher(JointState, "/ffw_isaac/joint_targets", 10)

    def run(self) -> None:
        left_names = list(self.left.get("joint_names", []))
        right_names = list(self.right.get("joint_names", []))
        left_steps = list(self.left.get("step_names", []))
        right_steps = list(self.right.get("step_names", []))
        duration = float(self.left.get("duration", 5.0))

        if left_steps != right_steps:
            raise ValueError("Left/right step_names must match in this script.")

        for step in left_steps:
            lpos = list(self.left.get(step, []))
            rpos = list(self.right.get(step, []))
            if len(lpos) != len(left_names):
                raise ValueError(f"{step}: left positions length mismatch.")
            if len(rpos) != len(right_names):
                raise ValueError(f"{step}: right positions length mismatch.")

            self.pub_left.publish(_traj(left_names, lpos))
            self.pub_right.publish(_traj(right_names, rpos))
            # Also publish full joint targets directly for current relay/Isaac pipeline.
            cmd = {n: 0.0 for n in FULL_JOINT_ORDER}
            cmd["lift_joint"] = -0.17634518808722158
            for n, p in zip(left_names, lpos):
                cmd[n] = float(p)
            for n, p in zip(right_names, rpos):
                cmd[n] = float(p)
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.header.frame_id = "base_link"
            js.name = list(FULL_JOINT_ORDER)
            js.position = [cmd[n] for n in FULL_JOINT_ORDER]
            for _ in range(5):
                self.pub_joint_targets.publish(js)
                rclpy.spin_once(self, timeout_sec=0.02)
            self.get_logger().info(f"Published {step}; waiting {duration:.2f}s")
            t_end = time.monotonic() + max(duration, 0.0)
            while time.monotonic() < t_end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)


def main() -> None:
    script_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default=str(script_root / "config" / "arm_forward_pose.yaml"),
        help="YAML file with arm_l_joint_trajectory_executor and arm_r_joint_trajectory_executor",
    )
    args = parser.parse_args()

    rclpy.init()
    node = PosePublisher(Path(args.config))
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
