# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""Load FFW joint + Isaac base pose for a named step in pick_place-style YAML."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

FULL_JOINT_ORDER = [
    "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
    "arm_l_joint5", "arm_l_joint6", "arm_l_joint7", "gripper_l_joint1",
    "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
    "arm_r_joint5", "arm_r_joint6", "arm_r_joint7", "gripper_r_joint1",
    "lift_joint", "head_joint1", "head_joint2",
]
BASE_POSE_NAMES = ["isaac_base_x", "isaac_base_y", "isaac_base_z", "isaac_base_yaw"]
DEFAULT_SHIRT_POSE = {
    "shirt_tx": -0.287,
    "shirt_ty": -3.512,
    "shirt_tz": 3.799,
    "shirt_rx": 0.0,
    "shirt_ry": 0.0,
    "shirt_rz": 0.0,
}


def _extract_executor(root: dict[str, Any], key: str) -> dict[str, Any]:
    any_ns = root.get("/**", {})
    exe = any_ns.get(key, {})
    params = exe.get("ros__parameters", {})
    if not isinstance(params, dict):
        raise ValueError(f"Invalid ros__parameters for {key}")
    return params


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
    raise ValueError("base pose must be [x, y, z, yaw]")


def _head_from_step(raw: Any, fallback: dict[str, float]) -> dict[str, float]:
    if raw is None:
        return dict(fallback)
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        return {"head_joint1": float(raw[0]), "head_joint2": float(raw[1])}
    raise ValueError("step_head must be [h1, h2]")


def load_step_pose(yaml_path: str | Path, step: str = "Start") -> dict[str, float]:
    """Return joint/base dict for a sequence step (e.g. Start, Ready)."""
    path = Path(yaml_path)
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError("Top-level YAML must be a mapping.")

    left = _extract_executor(data, "arm_l_joint_trajectory_executor")
    right = _extract_executor(data, "arm_r_joint_trajectory_executor")
    left_names = list(left.get("joint_names", []))
    right_names = list(right.get("joint_names", []))
    if step not in left.get("step_names", []):
        raise KeyError(f"Unknown step '{step}' in {path}")

    lpos = list(left[step])
    rpos = list(right[step])
    if len(lpos) != len(left_names) or len(rpos) != len(right_names):
        raise ValueError(f"{step}: left/right position length mismatch")

    pose: dict[str, float] = {name: 0.0 for name in FULL_JOINT_ORDER}
    step_lift = dict(left.get("step_lift", {}) or {})
    if step in step_lift:
        pose["lift_joint"] = float(step_lift[step])
    elif "lift_joint" in left:
        pose["lift_joint"] = float(left["lift_joint"])

    base_fallback = {name: 0.0 for name in BASE_POSE_NAMES}
    step_base = dict(left.get("step_base_pose", {}) or {})
    pose.update(_parse_base_pose(step_base.get(step), base_fallback))

    head_fallback = {"head_joint1": 0.0, "head_joint2": 0.0}
    step_head = dict(left.get("step_head", {}) or {})
    pose.update(_head_from_step(step_head.get(step), head_fallback))

    for name, pos in zip(left_names, lpos):
        pose[name] = float(pos)
    for name, pos in zip(right_names, rpos):
        pose[name] = float(pos)
    return pose
