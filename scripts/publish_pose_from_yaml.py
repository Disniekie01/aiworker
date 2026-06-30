#!/usr/bin/env python3
"""Publish multi-step arm poses from a YAML config."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Any

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
SHIRT_TRANSLATE_NAMES = ["shirt_tx", "shirt_ty", "shirt_tz"]
SHIRT_ROTATE_NAMES = ["shirt_rx", "shirt_ry", "shirt_rz"]
SHIRT_POSE_NAMES = SHIRT_TRANSLATE_NAMES + SHIRT_ROTATE_NAMES
CRATE_POSE_NAMES = ["crate_tx", "crate_ty", "crate_tz", "crate_rx", "crate_ry", "crate_rz"]
MOTION_STEP_NAME = "motion_step"
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
PUBLISH_JOINT_ORDER = (
    FULL_JOINT_ORDER + BASE_POSE_NAMES + SHIRT_POSE_NAMES + CRATE_POSE_NAMES + [MOTION_STEP_NAME]
)
BODY_PUBLISH_ORDER = (
    ["lift_joint", "head_joint1", "head_joint2"]
    + BASE_POSE_NAMES
    + SHIRT_POSE_NAMES
    + CRATE_POSE_NAMES
    + [MOTION_STEP_NAME]
)


LIFT_JOINT_LOWER = -0.5
LIFT_JOINT_UPPER = 0.0


def _clamp_lift(value: float) -> float:
    return max(LIFT_JOINT_LOWER, min(LIFT_JOINT_UPPER, float(value)))


def _read_current_lift(node: Node, topic: str, timeout_s: float = 2.0) -> float | None:
    from sensor_msgs.msg import JointState

    latest: float | None = None

    def _cb(msg: JointState) -> None:
        nonlocal latest
        if "lift_joint" not in msg.name:
            return
        idx = msg.name.index("lift_joint")
        if idx < len(msg.position):
            latest = float(msg.position[idx])

    sub = node.create_subscription(JointState, topic, _cb, 10)
    deadline = time.time() + max(0.1, float(timeout_s))
    while latest is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    return latest


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
    for name in CRATE_POSE_NAMES:
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


def _parse_shirt_pose(raw: Any, fallback: dict[str, float]) -> tuple[dict[str, float], bool]:
    """Return (pose dict, include_rotation). A 3-element list is translate-only."""
    if raw is None:
        return dict(fallback), True
    if isinstance(raw, (list, tuple)) and len(raw) == 3:
        pose = dict(fallback)
        pose["shirt_tx"] = float(raw[0])
        pose["shirt_ty"] = float(raw[1])
        pose["shirt_tz"] = float(raw[2])
        return pose, False
    if isinstance(raw, (list, tuple)) and len(raw) >= 6:
        return {name: float(raw[i]) for i, name in enumerate(SHIRT_POSE_NAMES)}, True
    if isinstance(raw, dict):
        out = dict(fallback)
        for name in SHIRT_POSE_NAMES:
            if name in raw:
                out[name] = float(raw[name])
        include_rotation = any(name in raw for name in SHIRT_ROTATE_NAMES)
        return out, include_rotation
    raise ValueError("shirt pose must be [tx, ty, tz] or [tx, ty, tz, rx, ry, rz]")


def _parse_crate_pose(raw: Any, fallback: dict[str, float]) -> tuple[dict[str, float], bool]:
    if raw is None:
        return dict(fallback), True
    if isinstance(raw, (list, tuple)) and len(raw) == 3:
        pose = dict(fallback)
        pose["crate_tx"] = float(raw[0])
        pose["crate_ty"] = float(raw[1])
        pose["crate_tz"] = float(raw[2])
        return pose, False
    if isinstance(raw, (list, tuple)) and len(raw) >= 6:
        return {name: float(raw[i]) for i, name in enumerate(CRATE_POSE_NAMES)}, True
    if isinstance(raw, dict):
        out = dict(fallback)
        for name in CRATE_POSE_NAMES:
            if name in raw:
                out[name] = float(raw[name])
        include_rotation = any(name in raw for name in ("crate_rx", "crate_ry", "crate_rz"))
        return out, include_rotation
    raise ValueError("crate pose must be [tx, ty, tz] or [tx, ty, tz, rx, ry, rz]")


class PosePublisher(Node):
    def __init__(
        self,
        cfg_path: Path,
        publish_hz: float,
        hybrid_body: bool = False,
        playback_speed: float | None = None,
    ) -> None:
        super().__init__("pose_yaml_publisher")
        self.hybrid_body = bool(hybrid_body)
        data = _load_yaml(cfg_path)
        self.left = _extract_executor(data, "arm_l_joint_trajectory_executor")
        self.right = _extract_executor(data, "arm_r_joint_trajectory_executor")
        self.default_lift_joint = _clamp_lift(float(self.left.get("lift_joint", LIFT_JOINT_UPPER)))
        self.lift_upper = _clamp_lift(float(self.left.get("lift_joint_upper", LIFT_JOINT_UPPER)))
        self.lift_lower = _clamp_lift(float(self.left.get("lift_joint_lower", LIFT_JOINT_LOWER)))
        use_current_lift = bool(self.left.get("use_current_lift", True))
        joint_states_topic = str(self.left.get("joint_states_topic", "/joint_states")).strip()
        configured_lift_start = self.left.get("lift_start")
        if use_current_lift and joint_states_topic:
            measured_lift = _read_current_lift(self, joint_states_topic)
            if measured_lift is not None:
                self.lift_start = _clamp_lift(measured_lift)
                self.get_logger().info(
                    f"Using current lift_joint from {joint_states_topic}: {self.lift_start:.4f}"
                )
            elif configured_lift_start is not None:
                self.lift_start = _clamp_lift(float(configured_lift_start))
                self.get_logger().warn(
                    f"No lift_joint on {joint_states_topic}; using lift_start={self.lift_start:.4f}"
                )
            else:
                self.lift_start = self.default_lift_joint
                self.get_logger().warn(
                    f"No lift_joint on {joint_states_topic}; using lift_joint={self.lift_start:.4f}"
                )
        elif configured_lift_start is not None:
            self.lift_start = _clamp_lift(float(configured_lift_start))
        else:
            self.lift_start = self.default_lift_joint
        self.step_lift = dict(self.left.get("step_lift", {}) or {})
        self.step_lift_delta = dict(self.left.get("step_lift_delta", {}) or {})
        self.step_head = dict(self.left.get("step_head", {}) or {})
        self.step_base_pose = dict(self.left.get("step_base_pose", {}) or {})
        self.step_shirt_pose = dict(self.left.get("step_shirt_pose", {}) or {})
        self.step_crate_pose = dict(self.left.get("step_crate_pose", {}) or {})
        self.publish_hz = float(publish_hz)
        if self.publish_hz <= 0.0:
            raise ValueError("publish_hz must be positive.")

        left_topic = str(self.left.get("action_topic", "")).strip()
        right_topic = str(self.right.get("action_topic", "")).strip()
        if not left_topic or not right_topic:
            raise ValueError("Both arm action_topic values are required.")

        self.pub_joint_targets = self.create_publisher(
            JointState,
            "/ffw_isaac/body_targets" if self.hybrid_body else "/ffw_isaac/joint_targets",
            10,
        )
        if self.hybrid_body:
            self.get_logger().info(
                "Hybrid body mode: publishing base/lift/head/shirt to /ffw_isaac/body_targets "
                "(arms from VR relay)"
            )
        self.current_cmd = {name: 0.0 for name in PUBLISH_JOINT_ORDER}
        self.current_cmd["lift_joint"] = self.lift_start
        self.current_cmd[MOTION_STEP_NAME] = -1.0
        base_start = _parse_base_pose(
            self.left.get("base_start", [0.0, 0.0, 0.0, 0.0]),
            {name: 0.0 for name in BASE_POSE_NAMES},
        )
        self.current_cmd.update(base_start)
        shirt_start, _ = _parse_shirt_pose(self.left.get("shirt_start"), DEFAULT_SHIRT_POSE)
        self.current_cmd.update(shirt_start)
        self.current_cmd["_shirt_include_rotation"] = True
        crate_start, _ = _parse_crate_pose(self.left.get("crate_start"), DEFAULT_CRATE_POSE)
        self.current_cmd.update(crate_start)
        self.current_cmd["_crate_include_rotation"] = True
        yaml_speed = float(self.left.get("playback_speed", 1.0))
        if playback_speed is not None:
            self.playback_speed = float(playback_speed)
        else:
            self.playback_speed = yaml_speed
        if self.playback_speed <= 0.0:
            raise ValueError("playback_speed must be positive.")
        self.easing = str(self.left.get("easing", "smooth")).strip().lower() or "smooth"

    def _lift_for_step(self, step: str) -> float:
        if step in self.step_lift_delta:
            target = float(self.current_cmd.get("lift_joint", self.lift_start)) + float(
                self.step_lift_delta[step]
            )
            return _clamp_lift(target)
        if step in self.step_lift:
            return _clamp_lift(float(self.step_lift[step]))
        return _clamp_lift(float(self.current_cmd.get("lift_joint", self.lift_start)))

    def _base_pose_for_step(self, step: str) -> dict[str, float]:
        current = {
            "isaac_base_x": float(self.current_cmd.get("isaac_base_x", 0.0)),
            "isaac_base_y": float(self.current_cmd.get("isaac_base_y", 0.0)),
            "isaac_base_z": float(self.current_cmd.get("isaac_base_z", 0.0)),
            "isaac_base_yaw": float(self.current_cmd.get("isaac_base_yaw", 0.0)),
        }
        if step not in self.step_base_pose:
            return current
        return _parse_base_pose(self.step_base_pose[step], current)

    def _head_for_step(self, step: str) -> dict[str, float]:
        head = {
            "head_joint1": float(self.current_cmd.get("head_joint1", 0.0)),
            "head_joint2": float(self.current_cmd.get("head_joint2", 0.0)),
        }
        raw = self.step_head.get(step)
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
        raise ValueError(f"{step}: step_head must be [head1, head2] or {{head_joint1, head_joint2}}")

    def _shirt_for_step(self, step: str) -> dict[str, float]:
        current = {name: float(self.current_cmd.get(name, DEFAULT_SHIRT_POSE[name])) for name in SHIRT_POSE_NAMES}
        current["_shirt_include_rotation"] = bool(self.current_cmd.get("_shirt_include_rotation", True))
        if step not in self.step_shirt_pose:
            return current
        pose, include_rotation = _parse_shirt_pose(self.step_shirt_pose[step], current)
        pose["_shirt_include_rotation"] = include_rotation
        return pose

    def _crate_for_step(self, step: str) -> dict[str, float]:
        current = {name: float(self.current_cmd.get(name, DEFAULT_CRATE_POSE[name])) for name in CRATE_POSE_NAMES}
        current["_crate_include_rotation"] = bool(self.current_cmd.get("_crate_include_rotation", True))
        if step not in self.step_crate_pose:
            return current
        pose, include_rotation = _parse_crate_pose(self.step_crate_pose[step], current)
        pose["_crate_include_rotation"] = include_rotation
        return pose

    def _step_to_cmd(
        self,
        left_names: list[str],
        right_names: list[str],
        lpos: list[float],
        rpos: list[float],
        lift_joint: float,
        base_pose: dict[str, float],
        head_pose: dict[str, float],
        shirt_pose: dict[str, float],
        crate_pose: dict[str, float],
    ) -> dict[str, float]:
        cmd = {name: 0.0 for name in PUBLISH_JOINT_ORDER}
        cmd["lift_joint"] = float(lift_joint)
        cmd.update(base_pose)
        cmd.update(head_pose)
        cmd.update(shirt_pose)
        cmd.update(crate_pose)
        for name, pos in zip(left_names, lpos):
            cmd[name] = float(pos)
        for name, pos in zip(right_names, rpos):
            cmd[name] = float(pos)
        return cmd

    def _publish_joint_targets(self, cmd: dict[str, float]) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = "base_link"
        order = BODY_PUBLISH_ORDER if self.hybrid_body else PUBLISH_JOINT_ORDER
        js.name = list(order)
        js.position = [float(cmd[n]) for n in order]
        self.pub_joint_targets.publish(js)

    def _lerp_body_cmd(
        self,
        start: dict[str, float],
        end: dict[str, float],
        t: float,
        easing: str,
    ) -> dict[str, float]:
        u = _ease(t, easing)
        out = dict(start)
        out["lift_joint"] = _lerp_scalar(start.get("lift_joint", 0.0), end.get("lift_joint", 0.0), u)
        out["head_joint1"] = _lerp_scalar(start.get("head_joint1", 0.0), end.get("head_joint1", 0.0), u)
        out["head_joint2"] = _lerp_scalar(start.get("head_joint2", 0.0), end.get("head_joint2", 0.0), u)
        out["isaac_base_x"] = _lerp_scalar(
            start.get("isaac_base_x", 0.0), end.get("isaac_base_x", 0.0), u
        )
        out["isaac_base_y"] = _lerp_scalar(
            start.get("isaac_base_y", 0.0), end.get("isaac_base_y", 0.0), u
        )
        out["isaac_base_z"] = _lerp_scalar(
            start.get("isaac_base_z", 0.0), end.get("isaac_base_z", 0.0), u
        )
        out["isaac_base_yaw"] = _lerp_angle(
            start.get("isaac_base_yaw", 0.0), end.get("isaac_base_yaw", 0.0), u
        )
        for name in SHIRT_POSE_NAMES:
            out[name] = _lerp_scalar(start.get(name, 0.0), end.get(name, 0.0), u)
        for name in CRATE_POSE_NAMES:
            out[name] = _lerp_scalar(start.get(name, 0.0), end.get(name, 0.0), u)
        out[MOTION_STEP_NAME] = float(end.get(MOTION_STEP_NAME, start.get(MOTION_STEP_NAME, -1.0)))
        out["_shirt_include_rotation"] = bool(end.get("_shirt_include_rotation", True))
        out["_crate_include_rotation"] = bool(end.get("_crate_include_rotation", True))
        return out

    def _move_to(
        self,
        step: str,
        target: dict[str, float],
        base_duration: float,
        arm_duration: float,
        motion_step: int,
    ) -> None:
        base_duration = max(0.01, float(base_duration))
        arm_duration = max(0.01, float(arm_duration))
        duration = base_duration if self.hybrid_body else max(base_duration, arm_duration)
        steps = max(1, int(round(duration * self.publish_hz)))
        dt = duration / steps
        start = dict(self.current_cmd)
        target = dict(target)
        target[MOTION_STEP_NAME] = float(motion_step)
        for i in range(1, steps + 1):
            elapsed = i / steps
            t_base = min(1.0, elapsed * duration / base_duration)
            t_arm = min(1.0, elapsed * duration / arm_duration)
            if self.hybrid_body:
                cmd = self._lerp_body_cmd(start, target, t_base, self.easing)
            else:
                cmd = _lerp_cmd(start, target, t_arm, t_base, self.easing)
            cmd[MOTION_STEP_NAME] = float(motion_step)
            self._publish_joint_targets(cmd)
            rclpy.spin_once(self, timeout_sec=0.0)
            if i < steps:
                time.sleep(dt)
        self.current_cmd = dict(target)
        move_bits = (
            f"body {base_duration:.2f}s"
            if self.hybrid_body
            else f"arms {arm_duration:.2f}s, base {base_duration:.2f}s"
        )
        self.get_logger().info(
            f"Moved to {step} over {duration:.2f}s ({move_bits}, "
            f"{steps} samples @ {self.publish_hz:.1f} Hz)"
        )

    def _scaled_duration(self, seconds: float) -> float:
        return max(0.01, float(seconds) / self.playback_speed)

    def run(self) -> None:
        left_names = list(self.left.get("joint_names", []))
        right_names = list(self.right.get("joint_names", []))
        left_steps = list(self.left.get("step_names", []))
        right_steps = list(self.right.get("step_names", []))
        duration = float(self.left.get("duration", 5.0))
        arm_duration_default = float(self.left.get("arm_duration", duration))
        if "publish_hz" in self.left:
            self.publish_hz = float(self.left["publish_hz"])
        step_durations = dict(self.left.get("step_durations", {}) or {})
        step_arm_durations = dict(self.left.get("step_arm_durations", {}) or {})
        step_base_durations = dict(self.left.get("step_base_durations", {}) or {})

        if left_steps != right_steps:
            raise ValueError("Left/right step_names must match in this script.")

        self.get_logger().info(
            f"Playback speed: {self.playback_speed:.0%} "
            f"(step durations scaled by {1.0 / self.playback_speed:.3f}x)"
        )

        # Hold the current sim pose before the first interpolated move.
        self._publish_joint_targets(self.current_cmd)
        rclpy.spin_once(self, timeout_sec=0.0)

        for step_idx, step in enumerate(left_steps):
            lpos = list(self.left.get(step, []))
            rpos = list(self.right.get(step, []))
            if len(lpos) != len(left_names):
                raise ValueError(f"{step}: left positions length mismatch.")
            if len(rpos) != len(right_names):
                raise ValueError(f"{step}: right positions length mismatch.")

            target = self._step_to_cmd(
                left_names,
                right_names,
                lpos,
                rpos,
                self._lift_for_step(step),
                self._base_pose_for_step(step),
                self._head_for_step(step),
                self._shirt_for_step(step),
                self._crate_for_step(step),
            )
            step_duration = float(step_durations.get(step, duration))
            base_duration = self._scaled_duration(
                float(step_base_durations.get(step, step_duration))
            )
            arm_duration = self._scaled_duration(
                float(step_arm_durations.get(step, arm_duration_default))
            )
            self._move_to(step, target, base_duration, arm_duration, motion_step=step_idx)


def main() -> None:
    script_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default=str(script_root / "config" / "arm_forward_pose.yaml"),
        help="YAML file with arm_l_joint_trajectory_executor and arm_r_joint_trajectory_executor",
    )
    parser.add_argument(
        "--publish-hz",
        type=float,
        default=30.0,
        help="Interpolation publish rate (default: 30)",
    )
    parser.add_argument(
        "--hybrid-body",
        action="store_true",
        help="Publish only base/lift/head/shirt to /ffw_isaac/body_targets (arms from VR)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=None,
        help="Playback speed multiplier (0.7 = 70%% speed, overrides YAML playback_speed)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = PosePublisher(
        Path(args.config),
        publish_hz=args.publish_hz,
        hybrid_body=args.hybrid_body,
        playback_speed=args.speed,
    )
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
