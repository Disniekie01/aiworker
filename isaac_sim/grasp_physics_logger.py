# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""Structured JSONL logging for physics-grasp investigation."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Optional, Sequence

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from shirt_grasp_controller import (
    _merged_world_bounds,
    _resolve_tip_paths,
    _world_matrix,
    _xform_cache,
)


def _resolve_physics_body_path(stage: Usd.Stage, start_path: str) -> str:
    path = start_path.rstrip("/")
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return path
    best = path
    current = prim
    while current and current.IsValid():
        p = current.GetPath().pathString
        if current.HasAPI(UsdPhysics.RigidBodyAPI):
            rb = UsdPhysics.RigidBodyAPI(current)
            if rb.GetRigidBodyEnabledAttr().Get():
                return p
        if current.HasAPI(UsdPhysics.CollisionAPI):
            best = p
        parent = current.GetParent()
        if parent and parent.IsValid() and parent.HasAPI(UsdPhysics.ArticulationRootAPI):
            return best if best != path else p
        current = parent
    return best


def _matrix_from_pose(pos, quat_wxyz) -> Gf.Matrix4d:
    q = Gf.Quatd(float(quat_wxyz[0]), float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3]))
    m = Gf.Matrix4d()
    m.SetRotate(Gf.Rotation(q))
    m.SetTranslateOnly(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    return m


def _physics_world_matrix(stage: Usd.Stage, prim_path: str, xform_cache: UsdGeom.XformCache) -> Optional[Gf.Matrix4d]:
    try:
        from isaacsim.core.utils.xforms import get_world_pose

        pos, quat = get_world_pose(prim_path, fabric=True)
        return _matrix_from_pose(pos, quat).RemoveScaleShear()
    except Exception:
        pass
    m = _world_matrix(stage, prim_path, xform_cache)
    if m is None:
        return None
    return Gf.Matrix4d(m).RemoveScaleShear()


def _vec3d(v: Gf.Vec3d | Gf.Vec3f) -> list[float]:
    return [float(v[0]), float(v[1]), float(v[2])]


def _quat_wxyz(q: Gf.Quatd | Gf.Quatf) -> list[float]:
    im = q.GetImaginary()
    return [float(q.GetReal()), float(im[0]), float(im[1]), float(im[2])]


def _matrix_pose(m: Gf.Matrix4d) -> dict[str, list[float]]:
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
    return {"pos": _vec3d(t), "quat_wxyz": _quat_wxyz(q)}


def _read_velocity(stage: Usd.Stage, body_path: str) -> Optional[dict[str, list[float]]]:
    prim = stage.GetPrimAtPath(body_path)
    if not prim or not prim.IsValid():
        return None
    out: dict[str, list[float]] = {}
    for key, attr_name in (("linear", "physics:velocity"), ("angular", "physics:angularVelocity")):
        attr = prim.GetAttribute(attr_name)
        if attr and attr.IsValid():
            v = attr.Get()
            if v is not None:
                out[key] = _vec3d(v)
    return out or None


def _pose_delta_m(a: Gf.Matrix4d, b: Gf.Matrix4d) -> float:
    return float((a.ExtractTranslation() - b.ExtractTranslation()).GetLength())


class GraspPhysicsLogger:
    """Append-only JSONL session log for physics grasp / latch debugging."""

    def __init__(self, log_dir: str | Path, *, session_tag: str = "") -> None:
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        suffix = f"_{session_tag}" if session_tag else ""
        self.session_path = self.log_dir / f"physics_grasp_{stamp}{suffix}.jsonl"
        self.latest_path = self.log_dir / "latest.jsonl"
        self.sim_step = 0
        self._last_shirt_fabric: Optional[Gf.Matrix4d] = None
        self._monitor_active = False
        self._anomaly_count = 0

        self.emit(
            "session_start",
            session_path=str(self.session_path),
            latest_path=str(self.latest_path),
        )
        if self.latest_path.exists():
            self.latest_path.unlink()
        print(f"[physics_grasp_log] Writing to {self.session_path}")

    def set_sim_step(self, step: int) -> None:
        self.sim_step = int(step)

    def emit(self, event: str, **fields: Any) -> None:
        record = {
            "t": round(time.time(), 6),
            "sim_step": self.sim_step,
            "event": event,
            **fields,
        }
        try:
            line = json.dumps(record, default=str) + "\n"
        except Exception as exc:
            record = {
                "t": round(time.time(), 6),
                "sim_step": self.sim_step,
                "event": "log_serialize_error",
                "original_event": event,
                "error": repr(exc),
            }
            line = json.dumps(record, default=str) + "\n"
        try:
            with open(self.session_path, "a", encoding="utf-8") as session_fp:
                session_fp.write(line)
            with open(self.latest_path, "a", encoding="utf-8") as latest_fp:
                latest_fp.write(line)
        except Exception as exc:
            print(f"[physics_grasp_log] Write failed ({event}): {exc!r}")

    def log_config(self, **config: Any) -> None:
        self.emit("config", **config)

    def motion_step_name(self, motion_step: Optional[int], names: Sequence[str]) -> Optional[str]:
        if motion_step is None:
            return None
        idx = int(motion_step)
        if 0 <= idx < len(names):
            return names[idx]
        return None

    def snapshot_body(
        self,
        stage: Usd.Stage,
        label: str,
        prim_path: str,
        *,
        xform_cache: Optional[UsdGeom.XformCache] = None,
    ) -> dict[str, Any]:
        cache = xform_cache or _xform_cache(stage)
        fabric = _physics_world_matrix(stage, prim_path, cache)
        usd = _world_matrix(stage, prim_path, cache)
        body_path = _resolve_physics_body_path(stage, prim_path)
        snap: dict[str, Any] = {
            "label": label,
            "prim": prim_path,
            "resolved_body": body_path,
        }
        if fabric is not None:
            snap["fabric"] = _matrix_pose(fabric)
        if usd is not None:
            snap["usd"] = _matrix_pose(Gf.Matrix4d(usd).RemoveScaleShear())
        if fabric is not None and usd is not None:
            snap["fabric_usd_pos_err_m"] = _pose_delta_m(fabric, Gf.Matrix4d(usd).RemoveScaleShear())
        vel = _read_velocity(stage, body_path)
        if vel:
            snap["velocity"] = vel
        return snap

    def snapshot_overlap(
        self,
        stage: Usd.Stage,
        attach_prim: str,
        object_mesh_path: str,
        *,
        overlap_padding: float,
        overlaps: bool,
    ) -> dict[str, Any]:
        cache = _xform_cache(stage)
        tip_paths = _resolve_tip_paths(stage, attach_prim) or [attach_prim]
        grip_bounds = _merged_world_bounds(stage, tip_paths, cache)
        shirt_bounds = _merged_world_bounds(stage, [object_mesh_path], cache)
        data: dict[str, Any] = {
            "overlaps": overlaps,
            "overlap_padding_m": float(overlap_padding),
            "grip_tip_paths": list(tip_paths),
        }
        if grip_bounds[1] is not None:
            _, g_min, g_max = grip_bounds
            data["grip_aabb_min"] = _vec3d(g_min)
            data["grip_aabb_max"] = _vec3d(g_max)
        if shirt_bounds[1] is not None:
            _, s_min, s_max = shirt_bounds
            data["shirt_aabb_min"] = _vec3d(s_min)
            data["shirt_aabb_max"] = _vec3d(s_max)
        if grip_bounds[0] is not None and shirt_bounds[0] is not None:
            data["center_sep_m"] = float((grip_bounds[0] - shirt_bounds[0]).GetLength())
        return data

    def log_attach_snapshot(
        self,
        stage: Usd.Stage,
        *,
        attach_prim: str,
        object_root: str,
        object_mesh_path: str,
        body0_path: str,
        body1_path: str,
        attach_world: Gf.Matrix4d,
        grip_world: Gf.Matrix4d,
        body0_world: Gf.Matrix4d,
        body1_world: Gf.Matrix4d,
        local_pos0,
        local_rot0,
        local_pos1,
        local_rot1,
        frame_err: float,
        latch_mode: str,
    ) -> None:
        cache = _xform_cache(stage)

        def _local_to_world(body_world: Gf.Matrix4d, pos, rot) -> Gf.Matrix4d:
            local = Gf.Matrix4d()
            local.SetRotate(Gf.Rotation(rot))
            local.SetTranslateOnly(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
            return (body_world * local).RemoveScaleShear()

        world_from_local0 = _local_to_world(body0_world, local_pos0, local_rot0)
        world_from_local1 = _local_to_world(body1_world, local_pos1, local_rot1)
        constraint_residual_m = _pose_delta_m(world_from_local0, world_from_local1)

        self.emit(
            "attach_snapshot",
            latch_mode=latch_mode,
            frame_err_m=float(frame_err),
            constraint_residual_m=constraint_residual_m,
            body0_path=body0_path,
            body1_path=body1_path,
            local_pos0=_vec3d(local_pos0),
            local_rot0_wxyz=_quat_wxyz(local_rot0),
            local_pos1=_vec3d(local_pos1),
            local_rot1_wxyz=_quat_wxyz(local_rot1),
            attach_world=_matrix_pose(attach_world),
            grip_world=_matrix_pose(grip_world),
            body0_fabric=_matrix_pose(body0_world),
            body1_fabric=_matrix_pose(body1_world),
            attach_prim=self.snapshot_body(stage, "attach_prim", attach_prim, xform_cache=cache),
            shirt_root=self.snapshot_body(stage, "shirt_root", object_root, xform_cache=cache),
            shirt_mesh=self.snapshot_body(stage, "shirt_mesh", object_mesh_path, xform_cache=cache),
            body0_vel=_read_velocity(stage, body0_path),
            body1_vel_before_zero=_read_velocity(stage, body1_path),
        )

    def set_monitor_active(self, active: bool) -> None:
        self._monitor_active = bool(active)
        if not active:
            self._last_shirt_fabric = None

    def monitor_shirt(
        self,
        stage: Usd.Stage,
        shirt_body_path: str,
        *,
        motion_step: Optional[int],
        motion_step_name: Optional[str],
        grip_cmd: Optional[float],
        grip_meas: Optional[float],
        gripper_closed: bool,
        attached: bool,
        attach_pending: bool,
        close_step: int,
        vel_warn_m_s: float = 0.75,
        jump_warn_m: float = 0.03,
        sample_every: int = 5,
    ) -> None:
        if not self._monitor_active:
            return
        if self.sim_step % max(1, int(sample_every)) != 0:
            return

        cache = _xform_cache(stage)
        fabric = _physics_world_matrix(stage, shirt_body_path, cache)
        if fabric is None:
            return

        pos = fabric.ExtractTranslation()
        vel = _read_velocity(stage, shirt_body_path)
        linear_speed = 0.0
        if vel and "linear" in vel:
            v = vel["linear"]
            linear_speed = float((v[0] ** 2 + v[1] ** 2 + v[2] ** 2) ** 0.5)

        jump_m = 0.0
        if self._last_shirt_fabric is not None:
            jump_m = _pose_delta_m(fabric, self._last_shirt_fabric)
        self._last_shirt_fabric = Gf.Matrix4d(fabric)

        fields: dict[str, Any] = {
            "motion_step": motion_step,
            "motion_step_name": motion_step_name,
            "grip_cmd": grip_cmd,
            "grip_meas": grip_meas,
            "gripper_closed": gripper_closed,
            "attached": attached,
            "attach_pending": attach_pending,
            "close_step": close_step,
            "shirt_body": shirt_body_path,
            "shirt_pos": _vec3d(pos),
            "linear_speed_m_s": linear_speed,
            "step_jump_m": jump_m,
            "velocity": vel,
        }

        anomaly = linear_speed >= vel_warn_m_s or jump_m >= jump_warn_m
        if anomaly:
            self._anomaly_count += 1
            fields["anomaly"] = True
            fields["anomaly_count"] = self._anomaly_count
            self.emit("shirt_anomaly", **fields)
        else:
            self.emit("shirt_sample", **fields)
