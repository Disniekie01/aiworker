#!/usr/bin/env python3
"""Live web UI to tune FFW joint poses while Isaac Sim is running.

Publishes sensor_msgs/JointState on /ffw_isaac/joint_targets (same as
publish_pose_from_yaml.py). Load/save steps compatible with pick_place.yaml.

Run (with Isaac stack + udp_bridge already up):

  source /opt/ros/jazzy/setup.bash
  source ros2_ws/install/setup.bash
  source env_local.bash
  export ROS_DOMAIN_ID=0
  /usr/bin/python3.12 scripts/joint_pose_web_tuner.py --config config/pick_place.yaml

Open http://127.0.0.1:8765 in a browser.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse

if sys.version_info[:2] != (3, 12):
    raise RuntimeError(
        "joint_pose_web_tuner.py must run with Python 3.12 for ROS Jazzy rclpy.\n"
        "Use: /usr/bin/python3.12 scripts/joint_pose_web_tuner.py"
    )

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml

FULL_JOINT_ORDER = [
    "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
    "arm_l_joint5", "arm_l_joint6", "arm_l_joint7", "gripper_l_joint1",
    "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
    "arm_r_joint5", "arm_r_joint6", "arm_r_joint7", "gripper_r_joint1",
    "lift_joint", "head_joint1", "head_joint2",
]
BASE_POSE_NAMES = ["isaac_base_x", "isaac_base_y", "isaac_base_z", "isaac_base_yaw"]
SHIRT_POSE_NAMES = ["shirt_tx", "shirt_ty", "shirt_tz", "shirt_rx", "shirt_ry", "shirt_rz"]
CRATE_POSE_NAMES = ["crate_tx", "crate_ty", "crate_tz", "crate_rx", "crate_ry", "crate_rz"]
GRASP_ATTACH_OFFSET_NAMES = [
    "grasp_attach_tx",
    "grasp_attach_ty",
    "grasp_attach_tz",
    "grasp_attach_rx",
    "grasp_attach_ry",
    "grasp_attach_rz",
]
DEFAULT_SHIRT_POSE = {
    "shirt_tx": -0.287,
    "shirt_ty": -3.512,
    "shirt_tz": 3.799,
    "shirt_rx": 0.0,
    "shirt_ry": 0.0,
    "shirt_rz": 0.0,
}
DEFAULT_CRATE_POSE = {
    "crate_tx": 0.1638139638914529,
    "crate_ty": -0.689979076385498,
    "crate_tz": 1.040000081062317,
    "crate_rx": 0.0,
    "crate_ry": 0.0,
    "crate_rz": 90.0,
}
DEFAULT_GRASP_ATTACH_OFFSET = {name: 0.0 for name in GRASP_ATTACH_OFFSET_NAMES}
PUBLISH_JOINT_ORDER = FULL_JOINT_ORDER + BASE_POSE_NAMES + SHIRT_POSE_NAMES + CRATE_POSE_NAMES
PHYSICS_GRASP_PUBLISH_ORDER = (
    FULL_JOINT_ORDER + BASE_POSE_NAMES + CRATE_POSE_NAMES + GRASP_ATTACH_OFFSET_NAMES
)

LEFT_ARM_JOINTS = FULL_JOINT_ORDER[:8]
RIGHT_ARM_JOINTS = FULL_JOINT_ORDER[8:16]

# URDF limits from assets/ffw_bg2_follower.urdf
JOINT_LIMITS: dict[str, dict[str, float]] = {
    "arm_l_joint1": {"min": -3.14, "max": 3.14, "step": 0.01},
    "arm_l_joint2": {"min": 0.0, "max": 3.14, "step": 0.01},
    "arm_l_joint3": {"min": -3.14, "max": 3.14, "step": 0.01},
    "arm_l_joint4": {"min": -2.9361, "max": 1.0786, "step": 0.01},
    "arm_l_joint5": {"min": -3.14, "max": 3.14, "step": 0.01},
    "arm_l_joint6": {"min": -1.57, "max": 1.57, "step": 0.01},
    "arm_l_joint7": {"min": -1.8201, "max": 1.5804, "step": 0.01},
    "gripper_l_joint1": {"min": 0.0, "max": 1.0, "step": 0.05},
    "arm_r_joint1": {"min": -3.14, "max": 3.14, "step": 0.01},
    "arm_r_joint2": {"min": -3.14, "max": 0.0, "step": 0.01},
    "arm_r_joint3": {"min": -3.14, "max": 3.14, "step": 0.01},
    "arm_r_joint4": {"min": -2.9361, "max": 1.0786, "step": 0.01},
    "arm_r_joint5": {"min": -3.14, "max": 3.14, "step": 0.01},
    "arm_r_joint6": {"min": -1.57, "max": 1.57, "step": 0.01},
    "arm_r_joint7": {"min": -1.5804, "max": 1.8201, "step": 0.01},
    "gripper_r_joint1": {"min": 0.0, "max": 1.0, "step": 0.05},
    "lift_joint": {"min": -0.5, "max": 0.0, "step": 0.01},
    "head_joint1": {"min": -1.57, "max": 1.57, "step": 0.01},
    "head_joint2": {"min": -1.8201, "max": 1.5804, "step": 0.01},
    "isaac_base_x": {"min": -2.0, "max": 2.0, "step": 0.001},
    "isaac_base_y": {"min": -2.0, "max": 2.0, "step": 0.001},
    "isaac_base_z": {"min": -0.5, "max": 0.5, "step": 0.001},
    "isaac_base_yaw": {"min": -3.14159, "max": 3.14159, "step": 0.01},
    "shirt_tx": {"min": -500.0, "max": 500.0, "step": 0.001, "unclamped": True},
    "shirt_ty": {"min": -500.0, "max": 500.0, "step": 0.001, "unclamped": True},
    "shirt_tz": {"min": -500.0, "max": 500.0, "step": 0.001, "unclamped": True},
    "shirt_rx": {"min": -500.0, "max": 500.0, "step": 0.01, "unclamped": True, "unit": "deg"},
    "shirt_ry": {"min": -500.0, "max": 500.0, "step": 0.01, "unclamped": True, "unit": "deg"},
    "shirt_rz": {"min": -500.0, "max": 500.0, "step": 0.01, "unclamped": True, "unit": "deg"},
    "crate_tx": {"min": -500.0, "max": 500.0, "step": 0.001, "unclamped": True},
    "crate_ty": {"min": -500.0, "max": 500.0, "step": 0.001, "unclamped": True},
    "crate_tz": {"min": -500.0, "max": 500.0, "step": 0.001, "unclamped": True},
    "crate_rx": {"min": -500.0, "max": 500.0, "step": 0.1, "unclamped": True, "unit": "deg"},
    "crate_ry": {"min": -500.0, "max": 500.0, "step": 0.1, "unclamped": True, "unit": "deg"},
    "crate_rz": {"min": -500.0, "max": 500.0, "step": 0.1, "unclamped": True, "unit": "deg"},
    "grasp_attach_tx": {"min": -1.0, "max": 1.0, "step": 0.001, "unit": "m", "unclamped": True},
    "grasp_attach_ty": {"min": -1.0, "max": 1.0, "step": 0.001, "unit": "m", "unclamped": True},
    "grasp_attach_tz": {"min": -1.0, "max": 1.0, "step": 0.001, "unit": "m", "unclamped": True},
    "grasp_attach_rx": {"min": -500.0, "max": 500.0, "step": 0.1, "unit": "deg", "unclamped": True},
    "grasp_attach_ry": {"min": -500.0, "max": 500.0, "step": 0.1, "unit": "deg", "unclamped": True},
    "grasp_attach_rz": {"min": -500.0, "max": 500.0, "step": 0.1, "unit": "deg", "unclamped": True},
}


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


def _clamp(name: str, value: float) -> float:
    if name in SHIRT_POSE_NAMES or name in CRATE_POSE_NAMES or name in GRASP_ATTACH_OFFSET_NAMES:
        return float(value)
    lim = JOINT_LIMITS.get(name, {"min": -3.14, "max": 3.14})
    return max(float(lim["min"]), min(float(lim["max"]), float(value)))


# Left-arm values derived from right arm (FFW follower symmetry).
# joint4 elbow: sign flip; other joints copy right value.
ARM_MIRROR_SIGN: dict[int, int] = {1: 1, 2: 1, 3: 1, 4: -1, 5: 1, 6: 1, 7: 1}


def _mirror_right_arm_to_left(pose: dict[str, float]) -> dict[str, float]:
    out = dict(pose)
    for i, sign in ARM_MIRROR_SIGN.items():
        r_name = f"arm_r_joint{i}"
        l_name = f"arm_l_joint{i}"
        out[l_name] = _clamp(l_name, sign * float(pose.get(r_name, 0.0)))
    out["gripper_l_joint1"] = _clamp(
        "gripper_l_joint1", float(pose.get("gripper_r_joint1", 0.0))
    )
    return out


def _parse_base_pose(raw: Any, fallback: dict[str, float]) -> dict[str, float]:
    if raw is None:
        return dict(fallback)
    if isinstance(raw, (list, tuple)) and len(raw) == 4:
        return {
            "isaac_base_x": float(raw[0]),
            "isaac_base_y": float(raw[1]),
            "isaac_base_z": float(raw[2]),
            "isaac_base_yaw": float(raw[3]),
        }
    if isinstance(raw, dict):
        return {
            "isaac_base_x": float(raw.get("x", fallback["isaac_base_x"])),
            "isaac_base_y": float(raw.get("y", fallback["isaac_base_y"])),
            "isaac_base_z": float(raw.get("z", fallback["isaac_base_z"])),
            "isaac_base_yaw": float(raw.get("yaw", fallback["isaac_base_yaw"])),
        }
    raise ValueError("base pose must be [x, y, z, yaw] or {x, y, z, yaw}")


def _head_from_step(raw: Any, fallback: dict[str, float]) -> dict[str, float]:
    head = dict(fallback)
    if raw is None:
        return head
    if isinstance(raw, dict):
        if "head_joint1" in raw:
            head["head_joint1"] = float(raw["head_joint1"])
        if "head_joint2" in raw:
            head["head_joint2"] = float(raw["head_joint2"])
        return head
    if isinstance(raw, (list, tuple)):
        if len(raw) >= 1:
            head["head_joint1"] = float(raw[0])
        if len(raw) >= 2:
            head["head_joint2"] = float(raw[1])
        return head
    raise ValueError("step_head must be [h1, h2] or dict")


def _parse_shirt_pose(raw: Any, fallback: dict[str, float]) -> dict[str, float]:
    if raw is None:
        return dict(fallback)
    if isinstance(raw, (list, tuple)) and len(raw) == 3:
        pose = dict(fallback)
        pose["shirt_tx"] = float(raw[0])
        pose["shirt_ty"] = float(raw[1])
        pose["shirt_tz"] = float(raw[2])
        return pose
    if isinstance(raw, (list, tuple)) and len(raw) >= 6:
        return {name: float(raw[i]) for i, name in enumerate(SHIRT_POSE_NAMES)}
    if isinstance(raw, dict):
        out = dict(fallback)
        for name in SHIRT_POSE_NAMES:
            if name in raw:
                out[name] = float(raw[name])
        return out
    raise ValueError("shirt pose must be [tx, ty, tz] or [tx, ty, tz, rx, ry, rz]")


def _parse_crate_pose(raw: Any, fallback: dict[str, float]) -> dict[str, float]:
    if raw is None:
        return dict(fallback)
    if isinstance(raw, (list, tuple)) and len(raw) == 3:
        pose = dict(fallback)
        pose["crate_tx"] = float(raw[0])
        pose["crate_ty"] = float(raw[1])
        pose["crate_tz"] = float(raw[2])
        return pose
    if isinstance(raw, (list, tuple)) and len(raw) >= 6:
        return {name: float(raw[i]) for i, name in enumerate(CRATE_POSE_NAMES)}
    if isinstance(raw, dict):
        out = dict(fallback)
        for name in CRATE_POSE_NAMES:
            if name in raw:
                out[name] = float(raw[name])
        return out
    raise ValueError("crate pose must be [tx, ty, tz] or [tx, ty, tz, rx, ry, rz]")


def _parse_grasp_attach_offset(raw: Any, fallback: dict[str, float]) -> dict[str, float]:
    if raw is None:
        return dict(fallback)
    if isinstance(raw, (list, tuple)) and len(raw) >= 6:
        return {name: float(raw[i]) for i, name in enumerate(GRASP_ATTACH_OFFSET_NAMES)}
    if isinstance(raw, dict):
        out = dict(fallback)
        for name in GRASP_ATTACH_OFFSET_NAMES:
            short = name.replace("grasp_attach_", "")
            if name in raw:
                out[name] = float(raw[name])
            elif short in raw:
                out[name] = float(raw[short])
        return out
    raise ValueError("physics_grasp_attach_offset must be [tx,ty,tz,rx,ry,rz] or dict")


class YamlPoseStore:
    def __init__(self, path: Path) -> None:
        self.path = path.resolve()
        self.data = _load_yaml(self.path)
        self.left = _extract_executor(self.data, "arm_l_joint_trajectory_executor")
        self.right = _extract_executor(self.data, "arm_r_joint_trajectory_executor")
        self.left_names = list(self.left.get("joint_names", LEFT_ARM_JOINTS))
        self.right_names = list(self.right.get("joint_names", RIGHT_ARM_JOINTS))
        self.steps = list(self.left.get("step_names", []))
        if list(self.right.get("step_names", [])) != self.steps:
            raise ValueError("Left/right step_names must match.")
        self._lock = threading.Lock()

    def default_pose(self) -> dict[str, float]:
        pose = {name: 0.0 for name in PUBLISH_JOINT_ORDER}
        lift = float(self.left.get("lift_start", self.left.get("lift_joint", 0.0)))
        pose["lift_joint"] = _clamp("lift_joint", lift)
        base = _parse_base_pose(
            self.left.get("base_start", [0.0, 0.0, 0.0, 0.0]),
            {n: 0.0 for n in BASE_POSE_NAMES},
        )
        pose.update(base)
        shirt_start = _parse_shirt_pose(self.left.get("shirt_start"), DEFAULT_SHIRT_POSE)
        pose.update(shirt_start)
        crate_start = _parse_crate_pose(self.left.get("crate_start"), DEFAULT_CRATE_POSE)
        pose.update(crate_start)
        grasp_offset = _parse_grasp_attach_offset(
            self.left.get("physics_grasp_attach_offset"),
            DEFAULT_GRASP_ATTACH_OFFSET,
        )
        pose.update(grasp_offset)
        if self.steps:
            return self.pose_for_step(self.steps[0])
        return pose

    def pose_for_step(self, step: str) -> dict[str, float]:
        if step not in self.steps:
            raise KeyError(f"Unknown step: {step}")
        lpos = list(self.left.get(step, []))
        rpos = list(self.right.get(step, []))
        if len(lpos) != len(self.left_names):
            raise ValueError(f"{step}: left positions length mismatch")
        if len(rpos) != len(self.right_names):
            raise ValueError(f"{step}: right positions length mismatch")

        pose = {name: 0.0 for name in PUBLISH_JOINT_ORDER}
        for name, val in zip(self.left_names, lpos):
            pose[name] = _clamp(name, val)
        for name, val in zip(self.right_names, rpos):
            pose[name] = _clamp(name, val)

        step_lift = dict(self.left.get("step_lift", {}) or {})
        if step in step_lift:
            pose["lift_joint"] = _clamp("lift_joint", float(step_lift[step]))

        base_fallback = _parse_base_pose(
            self.left.get("base_start", [0.0, 0.0, 0.0, 0.0]),
            {n: 0.0 for n in BASE_POSE_NAMES},
        )
        step_base = dict(self.left.get("step_base_pose", {}) or {})
        pose.update(_parse_base_pose(step_base.get(step), base_fallback))

        head_fallback = {"head_joint1": 0.0, "head_joint2": 0.0}
        step_head = dict(self.left.get("step_head", {}) or {})
        pose.update(_head_from_step(step_head.get(step), head_fallback))

        shirt_fallback = _parse_shirt_pose(self.left.get("shirt_start"), DEFAULT_SHIRT_POSE)
        step_shirt = dict(self.left.get("step_shirt_pose", {}) or {})
        pose.update(_parse_shirt_pose(step_shirt.get(step), shirt_fallback))
        crate_fallback = _parse_crate_pose(self.left.get("crate_start"), DEFAULT_CRATE_POSE)
        step_crate = dict(self.left.get("step_crate_pose", {}) or {})
        pose.update(_parse_crate_pose(step_crate.get(step), crate_fallback))
        pose.update(
            _parse_grasp_attach_offset(
                self.left.get("physics_grasp_attach_offset"),
                DEFAULT_GRASP_ATTACH_OFFSET,
            )
        )
        return pose

    def save_physics_grasp_attach_offset(self, pose: dict[str, float]) -> None:
        offset = [float(pose.get(name, 0.0)) for name in GRASP_ATTACH_OFFSET_NAMES]
        self.left["physics_grasp_attach_offset"] = offset
        self.data.setdefault("/**", {})["arm_l_joint_trajectory_executor"] = {
            "ros__parameters": self.left
        }

    def apply_pose_to_step(self, step: str, pose: dict[str, float]) -> None:
        self.left[step] = [_clamp(n, pose.get(n, 0.0)) for n in self.left_names]
        self.right[step] = [_clamp(n, pose.get(n, 0.0)) for n in self.right_names]

        step_lift = dict(self.left.get("step_lift", {}) or {})
        step_lift[step] = _clamp("lift_joint", pose.get("lift_joint", 0.0))
        self.left["step_lift"] = step_lift

        step_head = dict(self.left.get("step_head", {}) or {})
        step_head[step] = [
            _clamp("head_joint1", pose.get("head_joint1", 0.0)),
            _clamp("head_joint2", pose.get("head_joint2", 0.0)),
        ]
        self.left["step_head"] = step_head

        base = [
            _clamp("isaac_base_x", pose.get("isaac_base_x", 0.0)),
            _clamp("isaac_base_y", pose.get("isaac_base_y", 0.0)),
            _clamp("isaac_base_z", pose.get("isaac_base_z", 0.0)),
            _clamp("isaac_base_yaw", pose.get("isaac_base_yaw", 0.0)),
        ]
        step_base = dict(self.left.get("step_base_pose", {}) or {})
        step_base[step] = base
        self.left["step_base_pose"] = step_base

        shirt = [float(pose.get(name, DEFAULT_SHIRT_POSE[name])) for name in SHIRT_POSE_NAMES]
        step_shirt = dict(self.left.get("step_shirt_pose", {}) or {})
        step_shirt[step] = shirt
        self.left["step_shirt_pose"] = step_shirt

        crate = [float(pose.get(name, DEFAULT_CRATE_POSE[name])) for name in CRATE_POSE_NAMES]
        step_crate = dict(self.left.get("step_crate_pose", {}) or {})
        step_crate[step] = crate
        self.left["step_crate_pose"] = step_crate

        self.data.setdefault("/**", {})["arm_l_joint_trajectory_executor"] = {
            "ros__parameters": self.left
        }
        self.data["/**"]["arm_r_joint_trajectory_executor"] = {
            "ros__parameters": self.right
        }

    def add_step(self, name: str, pose: dict[str, float]) -> None:
        if not name or not name.replace("_", "").isalnum():
            raise ValueError("Step name must be alphanumeric/underscore")
        if name in self.steps:
            raise ValueError(f"Step already exists: {name}")
        self.steps.append(name)
        self.left["step_names"] = list(self.steps)
        self.right["step_names"] = list(self.steps)
        self.apply_pose_to_step(name, pose)

    def update_step(self, name: str, pose: dict[str, float]) -> None:
        if name not in self.steps:
            raise ValueError(f"Unknown step: {name}")
        self.apply_pose_to_step(name, pose)

    def write_yaml(self) -> str:
        backup = self.path.with_suffix(self.path.suffix + ".bak")
        shutil.copy2(self.path, backup)
        with self.path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(
                self.data,
                f,
                default_flow_style=None,
                sort_keys=False,
                allow_unicode=True,
            )
        return str(backup)

    def dump_yaml_text(self) -> str:
        return yaml.safe_dump(
            self.data,
            default_flow_style=None,
            sort_keys=False,
            allow_unicode=True,
        )


class JointPoseWebTuner(Node):
    def __init__(
        self,
        store: YamlPoseStore,
        publish_hz: float,
        joint_states_topic: str,
        *,
        physics_grasp: bool = False,
    ) -> None:
        super().__init__("joint_pose_web_tuner")
        self.store = store
        self.physics_grasp = bool(physics_grasp)
        self.publish_order = (
            PHYSICS_GRASP_PUBLISH_ORDER
            if self.physics_grasp
            else PUBLISH_JOINT_ORDER
        )
        self.publish_hz = max(1.0, float(publish_hz))
        self.pose_lock = threading.Lock()
        self.pose = store.default_pose()
        self._latest_joint_states: dict[str, float] | None = None
        self._publish_enabled = False

        self.pub = self.create_publisher(JointState, "/ffw_isaac/joint_targets", 10)
        self.grasp_pub = None
        if self.physics_grasp:
            self.grasp_pub = self.create_publisher(
                JointState, "/ffw_isaac/grasp_attach_offset", 10
            )
        self.create_subscription(JointState, joint_states_topic, self._on_joint_states, 10)
        period = 1.0 / self.publish_hz
        self.create_timer(period, self._timer_publish)
        self.get_logger().info(
            f"Joint targets idle until Live publish or slider move; "
            f"config={store.path}"
            + (
                "; grasp attach offset sliders (ATTACH frame, live while latched)"
                if self.physics_grasp
                else ""
            )
        )

    def enable_publish(self) -> None:
        self._publish_enabled = True

    def disable_publish(self) -> None:
        self._publish_enabled = False

    def _on_joint_states(self, msg: JointState) -> None:
        snap: dict[str, float] = {}
        for name, pos in zip(msg.name, msg.position):
            if name in PUBLISH_JOINT_ORDER:
                snap[name] = float(pos)
        if snap:
            self._latest_joint_states = snap

    def set_pose(self, raw: dict[str, float]) -> dict[str, float]:
        with self.pose_lock:
            for name in self.publish_order:
                if name in raw:
                    self.pose[name] = _clamp(name, raw[name])
            for name in CRATE_POSE_NAMES:
                if name in raw:
                    self.pose[name] = _clamp(name, raw[name])
            if self.physics_grasp:
                for name in GRASP_ATTACH_OFFSET_NAMES:
                    if name in raw:
                        self.pose[name] = _clamp(name, raw[name])
            return dict(self.pose)

    def get_pose(self) -> dict[str, float]:
        with self.pose_lock:
            return dict(self.pose)

    def save_physics_grasp_attach_offset_to_store(self, store: YamlPoseStore) -> None:
        with self.pose_lock:
            store.save_physics_grasp_attach_offset(dict(self.pose))

    def snap_from_robot(self) -> dict[str, float] | None:
        if not self._latest_joint_states:
            return None
        with self.pose_lock:
            for name, val in self._latest_joint_states.items():
                self.pose[name] = _clamp(name, val)
            return dict(self.pose)

    def publish_grasp_offset(self) -> None:
        if not self.physics_grasp or self.grasp_pub is None:
            return
        with self.pose_lock:
            cmd = {n: float(self.pose.get(n, 0.0)) for n in GRASP_ATTACH_OFFSET_NAMES}
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = "base_link"
        js.name = list(GRASP_ATTACH_OFFSET_NAMES)
        js.position = [cmd[n] for n in js.name]
        self.grasp_pub.publish(js)

    def publish_joint_targets_now(self) -> None:
        with self.pose_lock:
            names = list(self.publish_order)
            positions = [float(self.pose.get(n, 0.0)) for n in names]
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = "base_link"
        js.name = names
        js.position = positions
        self.pub.publish(js)
        if self.physics_grasp:
            self.publish_grasp_offset()

    def _timer_publish(self) -> None:
        if not self._publish_enabled:
            return
        self.publish_joint_targets_now()


def _make_handler(
    static_dir: Path,
    store: YamlPoseStore,
    node: JointPoseWebTuner,
) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:
            return

        def _json_response(self, code: int, payload: dict[str, Any]) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self) -> dict[str, Any]:
            length = int(self.headers.get("Content-Length", 0))
            if length <= 0:
                return {}
            raw = self.rfile.read(length)
            data = json.loads(raw.decode("utf-8"))
            return data if isinstance(data, dict) else {}

        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                html = (static_dir / "joint_tuner.html").read_text(encoding="utf-8")
                body = html.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if path == "/api/meta":
                self._json_response(
                    200,
                    {
                        "config_path": str(store.path),
                        "steps": list(store.steps),
                        "joints": JOINT_LIMITS,
                        "joint_groups": {
                            "leftArm": LEFT_ARM_JOINTS,
                            "rightArm": RIGHT_ARM_JOINTS,
                        },
                        "pose": node.get_pose(),
                        "publish_hz": node.publish_hz,
                        "publishing": node._publish_enabled,
                        "physics_grasp": node.physics_grasp,
                        "hidden_sections": (
                            ["shirt"] if node.physics_grasp else ["graspAttach"]
                        ),
                        "visible_sections": ["graspAttach"] if node.physics_grasp else [],
                    },
                )
                return

            if path.startswith("/api/steps/"):
                step = unquote(path.split("/api/steps/", 1)[1])
                try:
                    with store._lock:
                        pose = store.pose_for_step(step)
                    self._json_response(200, {"step": step, "pose": pose})
                except Exception as exc:
                    self._json_response(400, {"error": str(exc)})
                return

            if path == "/api/snap":
                pose = node.snap_from_robot()
                if pose is None:
                    self._json_response(503, {"error": "No /joint_states received yet"})
                    return
                self._json_response(200, {"pose": pose})
                return

            if path == "/api/mirror-right-arm":
                pose = node.get_pose()
                mirrored = _mirror_right_arm_to_left(pose)
                pose = node.set_pose(mirrored)
                self._json_response(200, {"pose": pose, "message": "Mirrored right arm to left (j4 sign flipped)"})
                return

            if path == "/api/yaml/download":
                with store._lock:
                    text = store.dump_yaml_text()
                body = text.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/x-yaml")
                self.send_header(
                    "Content-Disposition",
                    f'attachment; filename="{store.path.name}"',
                )
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            self._json_response(404, {"error": "not found"})

        def do_POST(self) -> None:
            path = urlparse(self.path).path
            body = self._read_json()

            if path == "/api/pose":
                pose_in = body.get("pose", body)
                if not isinstance(pose_in, dict):
                    self._json_response(400, {"error": "pose must be an object"})
                    return
                live = bool(body.get("live", False))
                if "live" in body:
                    if live:
                        node.enable_publish()
                    else:
                        node.disable_publish()
                pose = node.set_pose(pose_in)
                if node.physics_grasp and any(
                    k in pose_in for k in GRASP_ATTACH_OFFSET_NAMES
                ):
                    node.publish_grasp_offset()
                self._json_response(200, {"pose": pose, "publishing": node._publish_enabled})
                return

            if path == "/api/grasp_offset":
                pose_in = body.get("pose", body)
                if not isinstance(pose_in, dict):
                    self._json_response(400, {"error": "pose must be an object"})
                    return
                if not node.physics_grasp:
                    self._json_response(400, {"error": "physics_grasp mode required"})
                    return
                pose = node.set_pose(pose_in)
                node.publish_grasp_offset()
                self._json_response(200, {"pose": pose})
                return

            if path == "/api/crate_pose":
                pose_in = body.get("pose", body)
                if not isinstance(pose_in, dict):
                    self._json_response(400, {"error": "pose must be an object"})
                    return
                pose = node.set_pose(pose_in)
                node.enable_publish()
                node.publish_joint_targets_now()
                self._json_response(200, {"pose": pose, "publishing": True})
                return

            if path == "/api/publish/enable":
                node.enable_publish()
                self._json_response(200, {"publishing": True})
                return

            if path == "/api/publish/disable":
                node.disable_publish()
                self._json_response(200, {"publishing": False})
                return

            if path == "/api/steps":
                name = str(body.get("name", "")).strip()
                mode = str(body.get("mode", "new")).strip().lower()
                pose_in = body.get("pose", node.get_pose())
                if not isinstance(pose_in, dict):
                    self._json_response(400, {"error": "pose must be an object"})
                    return
                try:
                    with store._lock:
                        if mode == "update":
                            store.update_step(name, pose_in)
                            msg = f"Updated step '{name}' in memory"
                        else:
                            store.add_step(name, pose_in)
                            msg = f"Added step '{name}' (not written to disk yet)"
                    self._json_response(200, {"message": msg, "steps": list(store.steps)})
                except Exception as exc:
                    self._json_response(400, {"error": str(exc)})
                return

            if path == "/api/yaml/write":
                try:
                    with store._lock:
                        if node.physics_grasp:
                            node.save_physics_grasp_attach_offset_to_store(store)
                        backup = store.write_yaml()
                    self._json_response(
                        200,
                        {
                            "message": f"Wrote {store.path} (backup: {backup})",
                            "path": str(store.path),
                        },
                    )
                except Exception as exc:
                    self._json_response(500, {"error": str(exc)})
                return

            self._json_response(404, {"error": "not found"})

    return Handler


def main() -> None:
    script_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Web UI for live FFW joint pose tuning")
    parser.add_argument(
        "--config",
        default=str(script_root / "config" / "pick_place.yaml"),
        help="YAML config (pick_place format)",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--publish-hz", type=float, default=30.0)
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument(
        "--physics-grasp",
        action="store_true",
        help="Hide shirt mesh sliders; do not publish shirt_tx/ty/tz (dynamic rigid shirt in Isaac)",
    )
    args = parser.parse_args()

    cfg_path = Path(args.config).resolve()
    if not cfg_path.is_file():
        raise SystemExit(f"Config not found: {cfg_path}")

    store = YamlPoseStore(cfg_path)
    static_dir = Path(__file__).resolve().parent / "static"

    rclpy.init()
    node = JointPoseWebTuner(
        store, args.publish_hz, args.joint_states_topic, physics_grasp=args.physics_grasp
    )
    if args.physics_grasp:
        node.publish_grasp_offset()

    handler_cls = _make_handler(static_dir, store, node)
    httpd = ThreadingHTTPServer((args.host, args.port), handler_cls)
    http_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    http_thread.start()

    print(f"\n  Joint Pose Tuner: http://{args.host}:{args.port}\n")
    node.get_logger().info(f"Web UI at http://{args.host}:{args.port}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
