# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""PhysX grasp via FixedJoint between gripper finger and object rigid bodies.

Uses ATTACH (gripper) and Grippoint (object) frames. Joint is created after
physics step using fabric poses so local frames match simulation (no snap pop).
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Optional, Sequence

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

if TYPE_CHECKING:
    from grasp_physics_logger import GraspPhysicsLogger

from shirt_grasp_controller import (
    DEFAULT_ATTACH_PRIM,
    AttachTrigger,
    _aabb_overlap,
    _as_rigid_matrix,
    _ensure_matrix_xform_op,
    _freeze_rigid_for_grasp,
    _merged_world_bounds,
    _nudge_prim_translate_world,
    _resolve_tip_paths,
    _set_mesh_collision,
    _set_world_matrix,
    _unfreeze_rigid,
    _world_matrix,
    _world_point,
    _xform_cache,
    resolve_gripper_attach_prim,
    resolve_shirt_grip_point,
)

GRASP_JOINTS_ROOT = "/World/GraspJoints"
SUSTAIN_FIXED_JOINT = "fixed_joint"
SUSTAIN_KINEMATIC_FOLLOW = "kinematic_follow"
GRASP_ATTACH_OFFSET_KEYS = (
    "grasp_attach_tx",
    "grasp_attach_ty",
    "grasp_attach_tz",
    "grasp_attach_rx",
    "grasp_attach_ry",
    "grasp_attach_rz",
)


def _object_root_for_mesh(mesh_path: str) -> str:
    parts = [p for p in mesh_path.split("/") if p]
    if len(parts) >= 2:
        return f"/{parts[0]}/{parts[1]}"
    return mesh_path.rsplit("/", 1)[0] if "/" in mesh_path else mesh_path


def _find_object_mesh_paths(stage: Usd.Stage, root_path: str, fallback_mesh: str = "") -> list[str]:
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        if fallback_mesh and stage.GetPrimAtPath(fallback_mesh).IsValid():
            return [fallback_mesh.rstrip("/")]
        return []
    paths: list[str] = []
    for prim in Usd.PrimRange(root):
        if not prim.IsActive():
            continue
        if prim.GetTypeName() == "Mesh":
            paths.append(prim.GetPath().pathString)
    if not paths and fallback_mesh and stage.GetPrimAtPath(fallback_mesh).IsValid():
        return [fallback_mesh.rstrip("/")]
    return paths


def _resolve_physics_body_path(stage: Usd.Stage, start_path: str) -> str:
    """Walk up from a marker prim to an articulation link or rigid body."""
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
    """Prefer fabric (simulated) pose; fall back to USD xform cache."""
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


def _matrix_to_local_pose(body_world: Gf.Matrix4d, frame_world: Gf.Matrix4d) -> tuple[Gf.Vec3f, Gf.Quatf]:
    local = body_world.GetInverse() * frame_world
    local = local.RemoveScaleShear()
    t = local.ExtractTranslation()
    q = local.ExtractRotationQuat()
    return (
        Gf.Vec3f(float(t[0]), float(t[1]), float(t[2])),
        Gf.Quatf(float(q.GetReal()), float(q.GetImaginary()[0]), float(q.GetImaginary()[1]), float(q.GetImaginary()[2])),
    )


def _zero_object_velocity(stage: Usd.Stage, body_path: str) -> None:
    prim = stage.GetPrimAtPath(body_path)
    if not prim or not prim.IsValid():
        return
    for attr_name, value in (
        ("physics:velocity", Gf.Vec3f(0.0, 0.0, 0.0)),
        ("physics:angularVelocity", Gf.Vec3f(0.0, 0.0, 0.0)),
    ):
        attr = prim.GetAttribute(attr_name)
        if attr and attr.IsValid():
            attr.Set(value)


def _filter_object_robot_collision(stage: Usd.Stage, object_root: str, robot_root: str) -> None:
    if not robot_root:
        return
    obj = stage.GetPrimAtPath(object_root)
    robot = stage.GetPrimAtPath(robot_root)
    if not obj or not robot or not obj.IsValid() or not robot.IsValid():
        return
    obj_fp = UsdPhysics.FilteredPairsAPI.Apply(obj)
    rel = obj_fp.GetFilteredPairsRel()
    if not rel or not rel.IsValid():
        rel = obj_fp.CreateFilteredPairsRel()
    rel.AddTarget(Sdf.Path(robot_root.rstrip("/")))
    robot_fp = UsdPhysics.FilteredPairsAPI.Apply(robot)
    rel_r = robot_fp.GetFilteredPairsRel()
    if not rel_r or not rel_r.IsValid():
        rel_r = robot_fp.CreateFilteredPairsRel()
    rel_r.AddTarget(Sdf.Path(object_root.rstrip("/")))


def _nudge_object_root(
    stage: Usd.Stage,
    object_root: str,
    delta_world: Gf.Vec3d,
    xform_cache: UsdGeom.XformCache,
) -> bool:
    if not stage.GetPrimAtPath(object_root).IsValid():
        return False
    _ensure_matrix_xform_op(stage, object_root, xform_cache)
    if _nudge_prim_translate_world(stage, object_root, delta_world, xform_cache):
        return True
    follow_world = _world_matrix(stage, object_root, xform_cache)
    if follow_world is None:
        return False
    new_t = follow_world.ExtractTranslation() + delta_world
    new_world = Gf.Matrix4d(follow_world)
    new_world.SetTranslateOnly(Gf.Vec3d(float(new_t[0]), float(new_t[1]), float(new_t[2])))
    return _set_world_matrix(stage, object_root, new_world, xform_cache)


class GraspConstraintController:
    """Create/remove a FixedJoint after physics step (motion step or VR grip close)."""

    def __init__(
        self,
        stage: Usd.Stage,
        attach_prim: str,
        object_mesh_path: str,
        object_grip_point: str = "",
        link_from_step: int = 5,
        release_from_step: int = -1,
        robot_filter_root: str = "",
        snap_to_gripper: bool = True,
        latch_at_current_pose: bool = False,
        attach_trigger: str = AttachTrigger.MOTION_STEP.value,
        close_threshold: float = 0.90,
        open_threshold: float = 0.25,
        settle_steps: int = 10,
        overlap_padding: float = 0.04,
        physics_logger: Optional["GraspPhysicsLogger"] = None,
        sustain_mode: str = SUSTAIN_FIXED_JOINT,
    ) -> None:
        self._stage = stage
        self._attach_prim = attach_prim.rstrip("/")
        self._object_mesh_path = object_mesh_path.rstrip("/")
        self._object_root = _object_root_for_mesh(self._object_mesh_path)
        self._object_grip_point = ""
        if object_grip_point and stage.GetPrimAtPath(object_grip_point).IsValid():
            self._object_grip_point = object_grip_point.rstrip("/")
        else:
            resolved = resolve_shirt_grip_point(stage, self._object_mesh_path, object_grip_point)
            if stage.GetPrimAtPath(resolved).IsValid():
                self._object_grip_point = resolved.rstrip("/")
        self._link_from_step = int(link_from_step)
        self._release_from_step = int(release_from_step)
        self._robot_filter_root = robot_filter_root.rstrip("/")
        self._snap_to_gripper = bool(snap_to_gripper)
        self._latch_at_current_pose = bool(latch_at_current_pose)
        self._sustain_mode = sustain_mode.strip() or SUSTAIN_FIXED_JOINT
        self._joint_path: Optional[str] = None
        self._kinematic_latched = False
        self._shirt_local_in_attach: Optional[Gf.Matrix4d] = None
        self._body0_path = ""
        self._body1_path = ""
        self._collision_filtered = False
        self._object_mesh_paths = _find_object_mesh_paths(
            stage, self._object_root, self._object_mesh_path
        )
        self._collision_disabled = False
        self._motion_step: Optional[int] = None
        self._snap_applied = False
        self._attach_trigger = AttachTrigger(attach_trigger)
        self._close_threshold = float(close_threshold)
        self._open_threshold = float(open_threshold)
        self._settle_steps = max(1, int(settle_steps))
        self._overlap_padding = float(overlap_padding)
        self._gripper_closed = False
        self._close_step = 0
        self._attach_pending = False
        self._attach_attempted = False
        self._log = physics_logger
        self._post_attach_watch_steps = 0
        self._motion_step_names: list[str] = []
        self._last_grip_cmd: Optional[float] = None
        self._last_grip_meas: Optional[float] = None
        self._attach_offset = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        print(
            f"[grasp_constraint] attach={self._attach_prim} "
            f"grippoint={self._object_grip_point or 'object-root'} "
            f"object_root={self._object_root} "
            f"trigger={self._attach_trigger.value} "
            f"link_step={self._link_from_step} release_step={self._release_from_step} "
            f"snap={self._snap_to_gripper} latch_pose={'current' if self._latch_at_current_pose else 'markers'} "
            f"sustain={self._sustain_mode}"
        )
        if self._log is not None:
            self._log.log_config(
                attach_prim=self._attach_prim,
                object_mesh_path=self._object_mesh_path,
                object_root=self._object_root,
                object_grip_point=self._object_grip_point or None,
                attach_trigger=self._attach_trigger.value,
                link_from_step=self._link_from_step,
                release_from_step=self._release_from_step,
                snap_to_gripper=self._snap_to_gripper,
                latch_at_current_pose=self._latch_at_current_pose,
                sustain_mode=self._sustain_mode,
                settle_steps=self._settle_steps,
                overlap_padding_m=self._overlap_padding,
                close_threshold=self._close_threshold,
                open_threshold=self._open_threshold,
                robot_filter_root=self._robot_filter_root or None,
            )

    def _disable_object_collision(self) -> None:
        if self._collision_disabled or not self._object_mesh_paths:
            return
        for mesh_path in self._object_mesh_paths:
            _set_mesh_collision(self._stage, mesh_path, False)
        self._collision_disabled = True
        print(
            f"[grasp_constraint] Disabled collision on {len(self._object_mesh_paths)} "
            f"object mesh(es) while attached"
        )
        self._emit("collision_disabled_on_attach", mesh_count=len(self._object_mesh_paths))

    def _restore_object_collision(self) -> None:
        if not self._collision_disabled:
            return
        for mesh_path in self._object_mesh_paths:
            _set_mesh_collision(self._stage, mesh_path, True)
        self._collision_disabled = False
        print("[grasp_constraint] Restored object mesh collision after release")
        self._emit("collision_restored_on_release")

    def _emit(self, event: str, **fields) -> None:
        if self._log is None:
            return
        try:
            self._log.emit(event, **fields)
        except Exception as exc:
            print(f"[grasp_constraint] Log emit failed ({event}): {exc!r}")

    def set_motion_step_names(self, names: Sequence[str]) -> None:
        self._motion_step_names = list(names)

    def set_attach_offset(
        self,
        tx: float,
        ty: float,
        tz: float,
        rx_deg: float,
        ry_deg: float,
        rz_deg: float,
    ) -> None:
        self._attach_offset = (
            float(tx),
            float(ty),
            float(tz),
            float(rx_deg),
            float(ry_deg),
            float(rz_deg),
        )

    def update_attach_offset_from_names(
        self, msg_names: Sequence[str], msg_pos: Sequence[float]
    ) -> bool:
        """Update offset only when grasp_attach_* keys are present in the stream."""
        mapping = {n: float(p) for n, p in zip(msg_names, msg_pos)}
        if not any(k in mapping for k in GRASP_ATTACH_OFFSET_KEYS):
            return False
        self.set_attach_offset(
            mapping.get("grasp_attach_tx", self._attach_offset[0]),
            mapping.get("grasp_attach_ty", self._attach_offset[1]),
            mapping.get("grasp_attach_tz", self._attach_offset[2]),
            mapping.get("grasp_attach_rx", self._attach_offset[3]),
            mapping.get("grasp_attach_ry", self._attach_offset[4]),
            mapping.get("grasp_attach_rz", self._attach_offset[5]),
        )
        return True

    def _attach_offset_matrix(self) -> Gf.Matrix4d:
        tx, ty, tz, rx_deg, ry_deg, rz_deg = self._attach_offset
        rot = (
            Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), math.radians(rx_deg))
            * Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), math.radians(ry_deg))
            * Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), math.radians(rz_deg))
        )
        m = Gf.Matrix4d()
        m.SetRotate(rot)
        m.SetTranslateOnly(Gf.Vec3d(tx, ty, tz))
        return m.RemoveScaleShear()

    @property
    def is_attached(self) -> bool:
        if self._kinematic_latched:
            return True
        return self._joint_path is not None and self._stage.GetPrimAtPath(self._joint_path).IsValid()

    def _snap_grippoint_to_attach(self, xform_cache: UsdGeom.XformCache) -> bool:
        if not self._object_grip_point:
            return False
        attach_pt = _world_point(self._stage, self._attach_prim, xform_cache)
        grip_pt = _world_point(self._stage, self._object_grip_point, xform_cache)
        if attach_pt is None or grip_pt is None:
            return False
        delta = attach_pt - grip_pt
        if float(delta.GetLength()) < 1e-5:
            return True
        if _nudge_object_root(self._stage, self._object_root, delta, xform_cache):
            print(
                f"[grasp_constraint] Snapped Grippoint to ATTACH via {self._object_root} "
                f"(delta {float(delta[0]):+.3f}, {float(delta[1]):+.3f}, {float(delta[2]):+.3f}) m"
            )
            return True
        print("[grasp_constraint] Snap failed: could not move object root")
        return False

    def _create_kinematic_latch(self) -> bool:
        if self.is_attached:
            return True

        xform_cache = _xform_cache(self._stage)
        body1_path = _resolve_physics_body_path(self._stage, self._object_root)
        if not self._stage.GetPrimAtPath(body1_path).IsValid():
            print(f"[grasp_constraint] Shirt body not found: {body1_path}")
            self._emit("attach_failed", reason="shirt_body_not_found", body1_path=body1_path)
            return False

        attach_world = _physics_world_matrix(self._stage, self._attach_prim, xform_cache)
        shirt_world = _physics_world_matrix(self._stage, body1_path, xform_cache)
        if attach_world is None or shirt_world is None:
            print("[grasp_constraint] Could not read fabric transforms for kinematic latch")
            self._emit("attach_failed", reason="missing_fabric_transforms")
            return False

        vel_before = None
        if self._log is not None:
            from grasp_physics_logger import _read_velocity

            vel_before = _read_velocity(self._stage, body1_path)

        _zero_object_velocity(self._stage, body1_path)
        attach_rigid = _as_rigid_matrix(attach_world)
        shirt_rigid = _as_rigid_matrix(shirt_world)
        self._shirt_local_in_attach = shirt_rigid * attach_rigid.GetInverse()
        _freeze_rigid_for_grasp(self._stage, body1_path)
        self._body1_path = body1_path
        self._kinematic_latched = True
        self._post_attach_watch_steps = 30

        if self._log is not None:
            self._log.log_attach_snapshot(
                self._stage,
                attach_prim=self._attach_prim,
                object_root=self._object_root,
                object_mesh_path=self._object_mesh_path,
                body0_path=self._attach_prim,
                body1_path=body1_path,
                attach_world=attach_rigid,
                grip_world=shirt_rigid,
                body0_world=attach_rigid,
                body1_world=shirt_rigid,
                local_pos0=Gf.Vec3f(0.0, 0.0, 0.0),
                local_rot0=Gf.Quatf(1.0, 0.0, 0.0, 0.0),
                local_pos1=Gf.Vec3f(0.0, 0.0, 0.0),
                local_rot1=Gf.Quatf(1.0, 0.0, 0.0, 0.0),
                frame_err=0.0,
                latch_mode="kinematic_follow",
            )
            if vel_before is not None:
                self._emit("shirt_velocity_zeroed", body1_path=body1_path, velocity_before=vel_before)

        print(
            f"[grasp_constraint] Kinematic latch {body1_path} -> {self._attach_prim} "
            f"(follow fabric pose through Lift/Turn)"
        )
        self._emit(
            "attach_created",
            sustain_mode=SUSTAIN_KINEMATIC_FOLLOW,
            body1_path=body1_path,
            attach_prim=self._attach_prim,
            frame_err_m=0.0,
            latch_mode="kinematic_follow",
            motion_step=self._motion_step,
        )
        self._disable_object_collision()
        return True

    def _follow_kinematic(self) -> None:
        if not self._kinematic_latched or self._shirt_local_in_attach is None:
            return
        xform_cache = _xform_cache(self._stage)
        attach_world = _physics_world_matrix(self._stage, self._attach_prim, xform_cache)
        if attach_world is None:
            return
        follow_world = (
            self._shirt_local_in_attach
            * self._attach_offset_matrix()
            * _as_rigid_matrix(attach_world)
        )
        if not _set_world_matrix(self._stage, self._object_root, follow_world, xform_cache):
            print(f"[grasp_constraint] Kinematic follow failed for {self._object_root}")
            return
        if self._body1_path:
            _zero_object_velocity(self._stage, self._body1_path)

    def _create_fixed_joint(self) -> bool:
        if self.is_attached:
            return True

        xform_cache = _xform_cache(self._stage)
        body0_path = _resolve_physics_body_path(self._stage, self._attach_prim)
        body1_path = _resolve_physics_body_path(self._stage, self._object_root)
        body0 = self._stage.GetPrimAtPath(body0_path)
        body1 = self._stage.GetPrimAtPath(body1_path)
        if not body0 or not body0.IsValid() or not body1 or not body1.IsValid():
            print(f"[grasp_constraint] Physics bodies not found: body0={body0_path} body1={body1_path}")
            self._emit("attach_failed", reason="physics_bodies_not_found", body0_path=body0_path, body1_path=body1_path)
            return False

        vel_before = None
        if self._log is not None:
            from grasp_physics_logger import _read_velocity

            vel_before = _read_velocity(self._stage, body1_path)

        _zero_object_velocity(self._stage, body1_path)

        body0_world = _physics_world_matrix(self._stage, body0_path, xform_cache)
        body1_world = _physics_world_matrix(self._stage, body1_path, xform_cache)
        if body0_world is None or body1_world is None:
            print("[grasp_constraint] Could not read physics/world transforms")
            self._emit("attach_failed", reason="missing_fabric_transforms", body0_path=body0_path, body1_path=body1_path)
            return False

        latch_mode = "fabric_body1" if self._latch_at_current_pose else "markers"
        if self._latch_at_current_pose:
            attach_world = body1_world
            grip_world = body1_world
        else:
            attach_world = _physics_world_matrix(self._stage, self._attach_prim, xform_cache)
            grip_path = self._object_grip_point or self._object_root
            grip_world = _physics_world_matrix(self._stage, grip_path, xform_cache)
            if attach_world is None or grip_world is None:
                print("[grasp_constraint] Could not read physics/world transforms")
                self._emit("attach_failed", reason="missing_marker_transforms")
                return False

        pos0, rot0 = _matrix_to_local_pose(body0_world, attach_world)
        pos1, rot1 = _matrix_to_local_pose(body1_world, grip_world)

        frame_err = (
            attach_world.ExtractTranslation() - grip_world.ExtractTranslation()
        ).GetLength()

        if self._log is not None:
            self._log.log_attach_snapshot(
                self._stage,
                attach_prim=self._attach_prim,
                object_root=self._object_root,
                object_mesh_path=self._object_mesh_path,
                body0_path=body0_path,
                body1_path=body1_path,
                attach_world=attach_world,
                grip_world=grip_world,
                body0_world=body0_world,
                body1_world=body1_world,
                local_pos0=pos0,
                local_rot0=rot0,
                local_pos1=pos1,
                local_rot1=rot1,
                frame_err=float(frame_err),
                latch_mode=latch_mode,
            )
            if vel_before is not None:
                self._emit("shirt_velocity_zeroed", body1_path=body1_path, velocity_before=vel_before)

        if not self._stage.GetPrimAtPath(GRASP_JOINTS_ROOT).IsValid():
            UsdGeom.Xform.Define(self._stage, GRASP_JOINTS_ROOT)

        joint_name = f"grasp_{body1_path.rsplit('/', 1)[-1]}"
        joint_path = f"{GRASP_JOINTS_ROOT}/{joint_name}"
        if self._stage.GetPrimAtPath(joint_path).IsValid():
            self._stage.RemovePrim(Sdf.Path(joint_path))

        joint = UsdPhysics.FixedJoint.Define(self._stage, joint_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(body0_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(body1_path)])
        joint.CreateLocalPos0Attr().Set(pos0)
        joint.CreateLocalRot0Attr().Set(rot0)
        joint.CreateLocalPos1Attr().Set(pos1)
        joint.CreateLocalRot1Attr().Set(rot1)
        joint.CreateCollisionEnabledAttr().Set(False)
        joint.CreateBreakForceAttr().Set(1.0e20)
        joint.CreateBreakTorqueAttr().Set(1.0e20)

        if not self._collision_filtered and self._robot_filter_root:
            _filter_object_robot_collision(self._stage, body1_path, self._robot_filter_root)
            self._collision_filtered = True
            self._emit(
                "collision_filter_applied",
                object_root=body1_path,
                robot_root=self._robot_filter_root,
            )

        self._joint_path = joint_path
        self._body0_path = body0_path
        self._body1_path = body1_path
        self._post_attach_watch_steps = 30
        print(
            f"[grasp_constraint] FixedJoint {joint_path} "
            f"body0={body0_path} body1={body1_path} "
            f"frame_err={float(frame_err):.4f}m "
            f"mode={latch_mode}"
        )
        self._emit(
            "attach_created",
            joint_path=joint_path,
            body0_path=body0_path,
            body1_path=body1_path,
            frame_err_m=float(frame_err),
            latch_mode=latch_mode,
            motion_step=self._motion_step,
        )
        self._disable_object_collision()
        return True

    def _release_joint(self) -> None:
        self._restore_object_collision()
        if self._kinematic_latched:
            body = self._body1_path or self._object_root
            _unfreeze_rigid(self._stage, body)
            print(f"[grasp_constraint] Released kinematic latch on {body}")
            self._emit("attach_released", sustain_mode=SUSTAIN_KINEMATIC_FOLLOW, body1_path=body)
            self._kinematic_latched = False
            self._shirt_local_in_attach = None
            self._body1_path = ""
            self._snap_applied = False
        if not self._joint_path:
            return
        path = self._joint_path
        if self._stage.GetPrimAtPath(path).IsValid():
            self._stage.RemovePrim(Sdf.Path(path))
        if self._body1_path:
            _zero_object_velocity(self._stage, self._body1_path)
        print(f"[grasp_constraint] Released {self._body1_path or self._object_root} (removed {path})")
        self._emit("attach_released", joint_path=path, body1_path=self._body1_path or self._object_root)
        self._joint_path = None
        self._body0_path = ""
        if not self._kinematic_latched:
            self._body1_path = ""
        self._snap_applied = False

    def _gripper_shirt_separation_m(self) -> Optional[float]:
        xform_cache = _xform_cache(self._stage)
        tip_paths = _resolve_tip_paths(self._stage, self._attach_prim)
        if not tip_paths:
            tip_paths = [self._attach_prim]
        grip_bounds = _merged_world_bounds(self._stage, tip_paths, xform_cache)
        shirt_bounds = _merged_world_bounds(self._stage, [self._object_mesh_path], xform_cache)
        if grip_bounds[0] is None or shirt_bounds[0] is None:
            return None
        return float((grip_bounds[0] - shirt_bounds[0]).GetLength())

    def _overlaps_object(self) -> bool:
        xform_cache = _xform_cache(self._stage)
        tip_paths = _resolve_tip_paths(self._stage, self._attach_prim)
        if not tip_paths:
            tip_paths = [self._attach_prim]
        grip_bounds = _merged_world_bounds(self._stage, tip_paths, xform_cache)
        shirt_bounds = _merged_world_bounds(self._stage, [self._object_mesh_path], xform_cache)
        if grip_bounds[1] is None or shirt_bounds[1] is None:
            sep = self._gripper_shirt_separation_m()
            return sep is not None and sep <= 0.14
        _, g_min, g_max = grip_bounds
        _, s_min, s_max = shirt_bounds
        if _aabb_overlap(g_min, g_max, s_min, s_max, self._overlap_padding):
            return True
        sep = self._gripper_shirt_separation_m()
        return sep is not None and sep <= 0.14

    def _maybe_release_on_motion_step(self) -> bool:
        ms = self._motion_step
        if ms is None:
            return False
        if self._release_from_step >= 0 and ms >= self._release_from_step:
            if self.is_attached:
                self._release_joint()
            return True
        if self._attach_trigger == AttachTrigger.MOTION_STEP and ms < self._link_from_step:
            if self.is_attached:
                self._release_joint()
            return True
        return False

    def _update_grip_close(self, grip_cmd: Optional[float], grip_meas: Optional[float]) -> None:
        if grip_cmd is None and grip_meas is None:
            return
        grip = float(grip_cmd if grip_cmd is not None else grip_meas)

        if not self._gripper_closed and grip >= self._close_threshold:
            self._gripper_closed = True
            self._close_step = 0
            self._attach_attempted = False
            self._attach_pending = False
            self._snap_applied = False
            self._emit(
                "grip_close_start",
                grip_cmd=grip_cmd,
                grip_meas=grip_meas,
                grip_used=grip,
                motion_step=self._motion_step,
            )
            if self._log is not None:
                self._log.set_monitor_active(True)
        elif self._gripper_closed and grip <= self._open_threshold:
            self._gripper_closed = False
            self._attach_pending = False
            self._emit(
                "grip_open",
                grip_cmd=grip_cmd,
                grip_meas=grip_meas,
                grip_used=grip,
                motion_step=self._motion_step,
            )
            if self._log is not None:
                self._log.set_monitor_active(False)
            if self.is_attached:
                self._release_joint()
            return

        if self.is_attached or not self._gripper_closed:
            return
        if self._attach_pending:
            return

        self._close_step += 1
        if self._close_step < self._settle_steps:
            if self._close_step == 1 or self._close_step == self._settle_steps - 1:
                self._emit(
                    "grip_settle",
                    close_step=self._close_step,
                    settle_steps=self._settle_steps,
                    grip_used=grip,
                    motion_step=self._motion_step,
                )
            return

        # After settle, re-check overlap while grip stays closed (GRIPP → Lift).
        if (self._close_step - self._settle_steps) % 3 != 0:
            return

        overlaps = self._overlaps_object()
        separation_m = self._gripper_shirt_separation_m()
        overlap_info = {}
        if self._log is not None:
            overlap_info = self._log.snapshot_overlap(
                self._stage,
                self._attach_prim,
                self._object_mesh_path,
                overlap_padding=self._overlap_padding,
                overlaps=overlaps,
            )
        self._emit(
            "overlap_check",
            close_step=self._close_step,
            motion_step=self._motion_step,
            separation_m=separation_m,
            **overlap_info,
        )
        if overlaps:
            self._attach_pending = True
            self._emit("attach_pending", reason="grip_close", motion_step=self._motion_step)
        else:
            print(
                "[grasp_constraint] Grip close: no overlap with object"
                + (f" (sep={separation_m:.3f}m)" if separation_m is not None else "")
            )
            self._emit(
                "attach_skipped",
                reason="no_overlap",
                motion_step=self._motion_step,
                separation_m=separation_m,
            )

    def update(
        self,
        grip_cmd: Optional[float] = None,
        grip_meas: Optional[float] = None,
        motion_step: Optional[int] = None,
    ) -> None:
        if grip_cmd is not None:
            self._last_grip_cmd = float(grip_cmd)
        if grip_meas is not None:
            self._last_grip_meas = float(grip_meas)
        if motion_step is not None:
            self._motion_step = int(motion_step)

        if self._attach_trigger == AttachTrigger.GRIP_CLOSE:
            if self._maybe_release_on_motion_step():
                return
            self._update_grip_close(grip_cmd, grip_meas)

    def _try_attach_after_physics(self, reason: str) -> None:
        if not self.is_attached:
            if self._snap_to_gripper and not self._snap_applied and self._object_grip_point:
                xform_cache = _xform_cache(self._stage)
                self._snap_grippoint_to_attach(xform_cache)
                self._snap_applied = True
                return
            try:
                if self._sustain_mode == SUSTAIN_KINEMATIC_FOLLOW:
                    self._create_kinematic_latch()
                else:
                    self._create_fixed_joint()
            except Exception as exc:
                print(f"[grasp_constraint] Attach failed ({reason}): {exc!r}")
                self._emit("attach_failed", reason=reason, error=repr(exc))

    def log_post_physics(self) -> None:
        """Sample shirt state after physics step (called from teleop loop)."""
        if self._log is None:
            return
        body_path = self._body1_path or _resolve_physics_body_path(self._stage, self._object_root)
        if self._gripper_closed or self.is_attached or self._post_attach_watch_steps > 0:
            self._log.monitor_shirt(
                self._stage,
                body_path,
                motion_step=self._motion_step,
                motion_step_name=self._log.motion_step_name(self._motion_step, self._motion_step_names),
                grip_cmd=self._last_grip_cmd,
                grip_meas=self._last_grip_meas,
                gripper_closed=self._gripper_closed,
                attached=self.is_attached,
                attach_pending=self._attach_pending,
                close_step=self._close_step,
                sample_every=1 if self._post_attach_watch_steps > 0 else 5,
            )
        if self._post_attach_watch_steps > 0:
            self._post_attach_watch_steps -= 1

    def follow_after_physics(self) -> None:
        """Attach/release after world.step() so fabric poses match PhysX bodies."""
        if self._attach_trigger == AttachTrigger.GRIP_CLOSE:
            if self._attach_pending and not self.is_attached:
                self._attach_pending = False
                self._try_attach_after_physics("grip_close")
            if self._kinematic_latched:
                self._follow_kinematic()
            return

        ms = self._motion_step
        if ms is None:
            return

        if self._maybe_release_on_motion_step():
            return

        if ms < self._link_from_step:
            return

        self._try_attach_after_physics(f"motion_step={ms}")


def build_grasp_constraint_controller(
    stage: Usd.Stage,
    attach_prim: str = DEFAULT_ATTACH_PRIM,
    object_mesh_paths: Sequence[str] = (),
    object_grip_point: str = "",
    link_from_step: int = 5,
    release_from_step: int = -1,
    robot_filter_root: str = "",
    snap_to_gripper: bool = True,
    latch_at_current_pose: bool = False,
    attach_trigger: str = AttachTrigger.MOTION_STEP.value,
    settle_steps: int = 10,
    physics_logger: Optional["GraspPhysicsLogger"] = None,
    sustain_mode: str = SUSTAIN_FIXED_JOINT,
) -> GraspConstraintController:
    mesh = list(object_mesh_paths)[0] if object_mesh_paths else ""
    return GraspConstraintController(
        stage,
        attach_prim=attach_prim,
        object_mesh_path=mesh,
        object_grip_point=object_grip_point,
        link_from_step=link_from_step,
        release_from_step=release_from_step,
        robot_filter_root=robot_filter_root,
        snap_to_gripper=snap_to_gripper,
        latch_at_current_pose=latch_at_current_pose,
        attach_trigger=attach_trigger,
        settle_steps=settle_steps,
        physics_logger=physics_logger,
        sustain_mode=sustain_mode,
    )
