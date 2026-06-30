#!/usr/bin/env python3
# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""Bridge ROS2 JointState -> UDP JSON for Isaac Python 3.11 runtimes.

Normal mode: forward /ffw_isaac/joint_targets (VR relay or YAML publisher).

Hybrid mode (--hybrid): merge VR arms from /ffw_isaac/joint_targets with scripted
base/lift/head/shirt from /ffw_isaac/body_targets, then forward to Isaac over UDP.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
from pathlib import Path
from typing import Dict, List

if sys.version_info[:2] != (3, 12):
    raise RuntimeError(
        "joint_targets_udp_bridge.py must run with Python 3.12 for ROS Jazzy rclpy.\n"
        "Use:\n"
        "  /usr/bin/python3.12 scripts/joint_targets_udp_bridge.py\n"
        "or deactivate conda before running."
    )

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

FULL_JOINT_ORDER = [
    "arm_l_joint1",
    "arm_l_joint2",
    "arm_l_joint3",
    "arm_l_joint4",
    "arm_l_joint5",
    "arm_l_joint6",
    "arm_l_joint7",
    "gripper_l_joint1",
    "arm_r_joint1",
    "arm_r_joint2",
    "arm_r_joint3",
    "arm_r_joint4",
    "arm_r_joint5",
    "arm_r_joint6",
    "arm_r_joint7",
    "gripper_r_joint1",
    "lift_joint",
    "head_joint1",
    "head_joint2",
]

VR_JOINTS = [
    "arm_l_joint1",
    "arm_l_joint2",
    "arm_l_joint3",
    "arm_l_joint4",
    "arm_l_joint5",
    "arm_l_joint6",
    "arm_l_joint7",
    "gripper_l_joint1",
    "arm_r_joint1",
    "arm_r_joint2",
    "arm_r_joint3",
    "arm_r_joint4",
    "arm_r_joint5",
    "arm_r_joint6",
    "arm_r_joint7",
    "gripper_r_joint1",
]

BODY_KEYS = [
    "lift_joint",
    "head_joint1",
    "head_joint2",
    "isaac_base_x",
    "isaac_base_y",
    "isaac_base_z",
    "isaac_base_yaw",
    "shirt_tx",
    "shirt_ty",
    "shirt_tz",
    "shirt_rx",
    "shirt_ry",
    "shirt_rz",
    "crate_tx",
    "crate_ty",
    "crate_tz",
    "crate_rx",
    "crate_ry",
    "crate_rz",
    "grasp_attach_tx",
    "grasp_attach_ty",
    "grasp_attach_tz",
    "grasp_attach_rx",
    "grasp_attach_ry",
    "grasp_attach_rz",
    "motion_step",
]

GRASP_ATTACH_KEYS = [
    "grasp_attach_tx",
    "grasp_attach_ty",
    "grasp_attach_tz",
    "grasp_attach_rx",
    "grasp_attach_ry",
    "grasp_attach_rz",
]

UDP_OUTPUT_ORDER = FULL_JOINT_ORDER + [
    "isaac_base_x",
    "isaac_base_y",
    "isaac_base_z",
    "isaac_base_yaw",
    "shirt_tx",
    "shirt_ty",
    "shirt_tz",
    "shirt_rx",
    "shirt_ry",
    "shirt_rz",
    "crate_tx",
    "crate_ty",
    "crate_tz",
    "crate_rx",
    "crate_ry",
    "crate_rz",
    "grasp_attach_tx",
    "grasp_attach_ty",
    "grasp_attach_tz",
    "grasp_attach_rx",
    "grasp_attach_ry",
    "grasp_attach_rz",
    "motion_step",
]


def _msg_to_map(msg: JointState) -> Dict[str, float]:
    return {n: float(p) for n, p in zip(msg.name, msg.position)}


def _load_grasp_attach_offset_from_yaml(yaml_path: str) -> Dict[str, float]:
    out = {k: 0.0 for k in GRASP_ATTACH_KEYS}
    path = Path(yaml_path.strip())
    if not path.is_file():
        return out
    try:
        import yaml

        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        params = data.get("/**", {}).get("arm_l_joint_trajectory_executor", {}).get(
            "ros__parameters", {}
        )
        raw = params.get("physics_grasp_attach_offset")
        if isinstance(raw, (list, tuple)) and len(raw) >= 6:
            for i, key in enumerate(GRASP_ATTACH_KEYS):
                out[key] = float(raw[i])
        elif isinstance(raw, dict):
            for key in GRASP_ATTACH_KEYS:
                short = key.replace("grasp_attach_", "")
                if short in raw:
                    out[key] = float(raw[short])
    except Exception as exc:
        print(f"[udp_bridge] Could not load physics_grasp_attach_offset: {exc!r}")
    return out


class JointTargetsUdpBridge(Node):
    def __init__(
        self,
        topic: str,
        host: str,
        port: int,
        *,
        hybrid: bool = False,
        body_topic: str = "/ffw_isaac/body_targets",
        publish_hz: float = 60.0,
        grasp_offset_yaml: str = "",
    ) -> None:
        super().__init__("joint_targets_udp_bridge")
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._dst = (host, int(port))
        self._hybrid = bool(hybrid)
        self._lock = threading.Lock()
        self._body_has_msg = False
        self._vr_map: Dict[str, float] = {n: 0.0 for n in FULL_JOINT_ORDER}
        self._body_map: Dict[str, float] = {
            "lift_joint": -0.15,
            "head_joint1": 0.0,
            "head_joint2": 0.0,
            "isaac_base_x": 0.0,
            "isaac_base_y": 0.0,
            "isaac_base_z": 0.0,
            "isaac_base_yaw": 0.0,
            "motion_step": -1.0,
        }
        self._grasp_attach: Dict[str, float] = _load_grasp_attach_offset_from_yaml(
            grasp_offset_yaml
        )
        if grasp_offset_yaml.strip() and any(abs(v) > 1e-9 for v in self._grasp_attach.values()):
            print(
                "[udp_bridge] Whole-carry grasp attach offset from YAML:",
                ", ".join(f"{k}={self._grasp_attach[k]:.4f}" for k in GRASP_ATTACH_KEYS),
            )

        self.create_subscription(
            JointState, "/ffw_isaac/grasp_attach_offset", self._cb_grasp_attach, 10
        )

        if self._hybrid:
            self.create_subscription(JointState, topic, self._cb_vr, 10)
            self.create_subscription(JointState, body_topic, self._cb_body, 10)
            hz = max(1.0, float(publish_hz))
            self.create_timer(1.0 / hz, self._publish_merged)
            self.get_logger().info(
                f"Hybrid merge: VR arms from {topic} + body from {body_topic} "
                f"-> udp://{host}:{port} @ {hz:.0f} Hz"
            )
        else:
            self.create_subscription(JointState, topic, self._cb_forward, 10)
            self.get_logger().info(f"Forwarding {topic} -> udp://{host}:{port} (JSON name/position)")

    def _send_udp(self, names: List[str], positions: List[float]) -> None:
        payload = {"name": names, "position": [float(x) for x in positions]}
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self._sock.sendto(data, self._dst)

    def _cb_grasp_attach(self, msg: JointState) -> None:
        incoming = _msg_to_map(msg)
        with self._lock:
            for key in GRASP_ATTACH_KEYS:
                if key in incoming:
                    self._grasp_attach[key] = incoming[key]

    def _merge_grasp_attach(self, incoming: Dict[str, float]) -> Dict[str, float]:
        with self._lock:
            for key in GRASP_ATTACH_KEYS:
                if key in incoming:
                    self._grasp_attach[key] = incoming[key]
            out = dict(incoming)
            for key in GRASP_ATTACH_KEYS:
                out[key] = float(self._grasp_attach[key])
            return out

    def _cb_forward(self, msg: JointState) -> None:
        merged = self._merge_grasp_attach(_msg_to_map(msg))
        self._send_udp(list(merged.keys()), list(merged.values()))

    def _cb_vr(self, msg: JointState) -> None:
        incoming = _msg_to_map(msg)
        with self._lock:
            for key in VR_JOINTS:
                if key in incoming:
                    self._vr_map[key] = incoming[key]
            # Keep lift/head as fallback until body stream arrives.
            for key in ("lift_joint", "head_joint1", "head_joint2"):
                if key in incoming:
                    self._vr_map[key] = incoming[key]

    def _cb_body(self, msg: JointState) -> None:
        incoming = _msg_to_map(msg)
        with self._lock:
            self._body_has_msg = True
            for key in BODY_KEYS:
                if key in incoming:
                    self._body_map[key] = incoming[key]

    def _publish_merged(self) -> None:
        with self._lock:
            merged: Dict[str, float] = {}
            for key in VR_JOINTS:
                merged[key] = float(self._vr_map.get(key, 0.0))
            if self._body_has_msg:
                for key in BODY_KEYS:
                    if key in self._body_map:
                        merged[key] = float(self._body_map[key])
            for key in GRASP_ATTACH_KEYS:
                merged[key] = float(self._grasp_attach.get(key, 0.0))
            names = list(UDP_OUTPUT_ORDER)
            positions = [merged.get(n, 0.0) for n in names if n in merged]
            names = [n for n in names if n in merged]
        self._send_udp(names, positions)

    def destroy_node(self) -> bool:
        self._sock.close()
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--joint_state_topic", default="/ffw_isaac/joint_targets")
    parser.add_argument(
        "--body_topic",
        default="/ffw_isaac/body_targets",
        help="Scripted base/lift/head/shirt topic (hybrid mode only)",
    )
    parser.add_argument(
        "--hybrid",
        action="store_true",
        help="Merge VR arms (/ffw_isaac/joint_targets) with body script (--body_topic)",
    )
    parser.add_argument("--publish_hz", type=float, default=60.0)
    parser.add_argument("--udp_host", default="127.0.0.1")
    parser.add_argument("--udp_port", type=int, default=15000)
    parser.add_argument(
        "--grasp-offset-yaml",
        default="",
        help="Load physics_grasp_attach_offset for whole carry (merged into every UDP packet)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = JointTargetsUdpBridge(
        args.joint_state_topic,
        args.udp_host,
        args.udp_port,
        hybrid=bool(args.hybrid),
        body_topic=args.body_topic,
        publish_hz=args.publish_hz,
        grasp_offset_yaml=args.grasp_offset_yaml,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
