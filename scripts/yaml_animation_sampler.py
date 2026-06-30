#!/usr/bin/env python3
"""Sample a pick_place-style YAML into a JSON animation timeline (no ROS)."""

from __future__ import annotations

import argparse
import json
import math
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
SHIRT_TRANSLATE_NAMES = ["shirt_tx", "shirt_ty", "shirt_tz"]
SHIRT_ROTATE_NAMES = ["shirt_rx", "shirt_ry", "shirt_rz"]
SHIRT_POSE_NAMES = SHIRT_TRANSLATE_NAMES + SHIRT_ROTATE_NAMES
MOTION_STEP_NAME = "motion_step"
DEFAULT_SHIRT_POSE = {
    "shirt_tx": -0.287,
    "shirt_ty": -3.512,
    "shirt_tz": 3.799,
    "shirt_rx": 0.0,
    "shirt_ry": 0.0,
    "shirt_rz": 0.0,
}
PUBLISH_JOINT_ORDER = FULL_JOINT_ORDER + BASE_POSE_NAMES + SHIRT_POSE_NAMES + [MOTION_STEP_NAME]

LIFT_JOINT_LOWER = -0.5
LIFT_JOINT_UPPER = 0.0


def _clamp_lift(value: float) -> float:
    return max(LIFT_JOINT_LOWER, min(LIFT_JOINT_UPPER, float(value)))


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


def _ease(t: float, mode: str) -> float:
    t = max(0.0, min(1.0, float(t)))
    if mode == "linear":
        return t
    return t * t * (3.0 - 2.0 * t)


def _lerp_scalar(start: float, end: float, t: float) -> float:
    return (1.0 - t) * float(start) + t * float(end)


def _lerp_angle(start: float, end: float, t: float) -> float:
    delta = (float(end) - float(start) + math.pi) % (2.0 * math.pi) - math.pi
    return float(start) + t * delta


def _lerp_cmd(
    start: dict[str, float],
    end: dict[str, float],
    t_arm: float,
    t_base: float,
    easing: str = "smooth",
) -> dict[str, float]:
    u_arm = _ease(t_arm, easing)
    u_base = _ease(t_base, easing)
    out: dict[str, float] = {}
    for name in FULL_JOINT_ORDER:
        out[name] = _lerp_scalar(start.get(name, 0.0), end.get(name, 0.0), u_arm)
    out["isaac_base_x"] = _lerp_scalar(start.get("isaac_base_x", 0.0), end.get("isaac_base_x", 0.0), u_base)
    out["isaac_base_y"] = _lerp_scalar(start.get("isaac_base_y", 0.0), end.get("isaac_base_y", 0.0), u_base)
    out["isaac_base_z"] = _lerp_scalar(start.get("isaac_base_z", 0.0), end.get("isaac_base_z", 0.0), u_base)
    out["isaac_base_yaw"] = _lerp_angle(
        start.get("isaac_base_yaw", 0.0),
        end.get("isaac_base_yaw", 0.0),
        u_base,
    )
    for name in SHIRT_TRANSLATE_NAMES:
        out[name] = _lerp_scalar(start.get(name, 0.0), end.get(name, 0.0), u_arm)
    for name in SHIRT_ROTATE_NAMES:
        out[name] = _lerp_scalar(start.get(name, 0.0), end.get(name, 0.0), u_arm)
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


def sample_yaml_animation(cfg_path: Path, publish_hz: float | None = None) -> dict[str, Any]:
    data = _load_yaml(cfg_path)
    left = _extract_executor(data, "arm_l_joint_trajectory_executor")
    right = _extract_executor(data, "arm_r_joint_trajectory_executor")

    left_names = list(left.get("joint_names", []))
    right_names = list(right.get("joint_names", []))
    left_steps = list(left.get("step_names", []))
    right_steps = list(right.get("step_names", []))
    if left_steps != right_steps:
        raise ValueError("Left/right step_names must match.")

    duration = float(left.get("duration", 5.0))
    arm_duration_default = float(left.get("arm_duration", duration))
    hz = float(publish_hz if publish_hz is not None else left.get("publish_hz", 30.0))
    if hz <= 0.0:
        raise ValueError("publish_hz must be positive.")
    easing = str(left.get("easing", "smooth")).strip().lower() or "smooth"
    step_durations = dict(left.get("step_durations", {}) or {})
    step_arm_durations = dict(left.get("step_arm_durations", {}) or {})
    step_base_durations = dict(left.get("step_base_durations", {}) or {})

    lift_start = _clamp_lift(float(left.get("lift_start", left.get("lift_joint", LIFT_JOINT_UPPER))))
    step_lift = dict(left.get("step_lift", {}) or {})
    step_lift_delta = dict(left.get("step_lift_delta", {}) or {})
    step_head = dict(left.get("step_head", {}) or {})
    step_base_pose = dict(left.get("step_base_pose", {}) or {})
    step_shirt_pose = dict(left.get("step_shirt_pose", {}) or {})

    current_cmd = {name: 0.0 for name in PUBLISH_JOINT_ORDER}
    current_cmd["lift_joint"] = lift_start
    base_start = _parse_base_pose(
        left.get("base_start", [0.0, 0.0, 0.0, 0.0]),
        {name: 0.0 for name in BASE_POSE_NAMES},
    )
    current_cmd.update(base_start)
    current_cmd.update(_parse_shirt_pose(left.get("shirt_start"), DEFAULT_SHIRT_POSE))

    def lift_for_step(step: str) -> float:
        if step in step_lift_delta:
            return _clamp_lift(float(current_cmd["lift_joint"]) + float(step_lift_delta[step]))
        if step in step_lift:
            return _clamp_lift(float(step_lift[step]))
        return _clamp_lift(float(current_cmd["lift_joint"]))

    def base_pose_for_step(step: str) -> dict[str, float]:
        current = {n: float(current_cmd[n]) for n in BASE_POSE_NAMES}
        if step not in step_base_pose:
            return current
        return _parse_base_pose(step_base_pose[step], current)

    def head_for_step(step: str) -> dict[str, float]:
        head = {
            "head_joint1": float(current_cmd.get("head_joint1", 0.0)),
            "head_joint2": float(current_cmd.get("head_joint2", 0.0)),
        }
        raw = step_head.get(step)
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
        raise ValueError(f"{step}: step_head must be [head1, head2] or dict")

    def step_to_cmd(
        step: str,
        lpos: list[float],
        rpos: list[float],
    ) -> dict[str, float]:
        cmd = {name: 0.0 for name in PUBLISH_JOINT_ORDER}
        cmd["lift_joint"] = float(lift_for_step(step))
        cmd.update(base_pose_for_step(step))
        cmd.update(head_for_step(step))
        cmd.update(_parse_shirt_pose(step_shirt_pose.get(step), DEFAULT_SHIRT_POSE))
        for name, pos in zip(left_names, lpos):
            cmd[name] = float(pos)
        for name, pos in zip(right_names, rpos):
            cmd[name] = float(pos)
        return cmd

    frames: list[dict[str, float]] = []
    t = 0.0
    frames.append({k: float(current_cmd[k]) for k in PUBLISH_JOINT_ORDER if k in current_cmd})

    for step_idx, step in enumerate(left_steps):
        lpos = list(left.get(step, []))
        rpos = list(right.get(step, []))
        if len(lpos) != len(left_names) or len(rpos) != len(right_names):
            raise ValueError(f"{step}: arm position length mismatch")

        target = step_to_cmd(step, lpos, rpos)
        step_duration = float(step_durations.get(step, duration))
        base_duration = max(0.01, float(step_base_durations.get(step, step_duration)))
        arm_duration = max(0.01, float(step_arm_durations.get(step, arm_duration_default)))
        seg_duration = max(base_duration, arm_duration)
        steps = max(1, int(round(seg_duration * hz)))
        start = dict(current_cmd)

        for i in range(1, steps + 1):
            elapsed = i / steps
            t_base = min(1.0, elapsed * seg_duration / base_duration)
            t_arm = min(1.0, elapsed * seg_duration / arm_duration)
            cmd = _lerp_cmd(start, target, t_arm, t_base, easing)
            cmd[MOTION_STEP_NAME] = float(step_idx)
            frames.append({k: float(cmd[k]) for k in PUBLISH_JOINT_ORDER if k in cmd})
            t += seg_duration / steps

        current_cmd = dict(target)
        current_cmd[MOTION_STEP_NAME] = float(step_idx)

    return {
        "source_yaml": str(cfg_path.resolve()),
        "fps": hz,
        "duration_s": t,
        "frame_count": len(frames),
        "joint_names": PUBLISH_JOINT_ORDER,
        "step_names": left_steps,
        "frames": frames,
    }


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Sample YAML pick-place animation to JSON")
    parser.add_argument(
        "--config",
        default=str(root / "config" / "pick_place.yaml"),
        help="YAML config (same format as publish_pose_from_yaml.py)",
    )
    parser.add_argument(
        "--output",
        default=str(root / "exports" / "pick_place_animation.json"),
        help="Output JSON path",
    )
    parser.add_argument("--publish-hz", type=float, default=None)
    args = parser.parse_args()

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    anim = sample_yaml_animation(Path(args.config), publish_hz=args.publish_hz)
    out_path.write_text(json.dumps(anim, indent=2), encoding="utf-8")
    print(
        f"Wrote {out_path} ({anim['frame_count']} frames, "
        f"{anim['duration_s']:.2f}s @ {anim['fps']} Hz)"
    )


if __name__ == "__main__":
    main()
