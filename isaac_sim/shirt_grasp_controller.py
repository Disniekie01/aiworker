# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""Pick soft shirts by welding the pile root to the gripper on a close event.

Design
------
* **Reach** and **hold** are separate problems.
* **Reach**: the right gripper subtree must overlap the shirt mesh (tune YAML / web tuner).
* **Hold**: on grip close, freeze the shirt world pose relative to the gripper and follow
  with a full 4x4 transform (no pull-in, no teleport, no PhysX attachments).
"""

from __future__ import annotations

from enum import Enum
from typing import Iterable, Optional, Sequence

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

DEFAULT_SHIRT_MESHES = (
    "/World/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell_01/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell/TShirts_Hanging_V_neck_04",
)
DEFAULT_SHIRT_STEM = (
    "/World/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell_01/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell"
)
DEFAULT_SHIRT_GRIP_POINT = f"{DEFAULT_SHIRT_STEM}/Grippoint"
DEFAULT_GRIPPER_BASE = "/World/Robot/ffw_sg2_follower/right_gripper"
DEFAULT_ATTACH_PRIM = (
    f"{DEFAULT_GRIPPER_BASE}/gripper_r_rh_p12_rn_r2"
)
GRIPPER_JOINT_CANDIDATES = (
    "/World/Robot/ffw_sg2_follower/joints/gripper_r_joint1",
    "/World/Robot/ffw_sg2_follower/gripper_r_joint1",
)
GRIPPER_CLOSE_STIFFNESS = 400.0
GRIPPER_CLOSE_MAX_FORCE = 250.0
GRIPPER_CLOSE_DAMPING = 15.0
GRIPPER_TIP_NAMES = (
    "gripper_r_rh_p12_rn_r2",
    "gripper_r_rh_p12_rn_l2",
)
GRIPPER_FINGER_PRIMS = {
    "r2": f"{DEFAULT_GRIPPER_BASE}/gripper_r_rh_p12_rn_r2",
    "l2": f"{DEFAULT_GRIPPER_BASE}/gripper_r_rh_p12_rn_l2",
    "attach": f"{DEFAULT_GRIPPER_BASE}/gripper_r_rh_p12_rn_r1/ATTACH",
}
GRIPPER_ATTACH_PRIM_CANDIDATES = (
    f"{DEFAULT_GRIPPER_BASE}/gripper_r_rh_p12_rn_r1/ATTACH",
    f"{DEFAULT_GRIPPER_BASE}/ATTACH",
)
DEFAULT_GRIPPER_PRIM = DEFAULT_ATTACH_PRIM  # backwards compat alias
SHIRT_ROOTS = (
    "/World/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell_01",
)


class GraspMode(str, Enum):
    """How to decide whether a grip close should pick the shirt."""

    OVERLAP = "overlap"  # gripper AABB intersects shirt AABB (recommended)
    DISTANCE = "distance"  # fingertip midpoint within max_grasp_distance of shirt surface
    DEMO = "demo"  # grip close always picks nearest shirt (for testing lift only)


class AttachTrigger(str, Enum):
    """When to weld the shirt to the gripper finger."""

    GRIP_CLOSE = "grip_close"
    MOTION_STEP = "motion_step"


def _shirt_root_for_mesh(mesh_path: str) -> str:
    parts = [p for p in mesh_path.split("/") if p]
    if len(parts) >= 2:
        return f"/{parts[0]}/{parts[1]}"
    return mesh_path


def _shirt_inner_prim_path(shirt_root: str) -> str:
    """Inner payload prim (e.g. ..._01/Folded_TShirt_... without _01 suffix)."""
    root_name = shirt_root.rstrip("/").rsplit("/", 1)[-1]
    if "_" in root_name:
        base = root_name.rsplit("_", 1)[0]
        return f"{shirt_root.rstrip('/')}/{base}"
    return f"{shirt_root.rstrip('/')}/{root_name}"


def _prim_has_complex_xform_stack(prim: Usd.Prim) -> bool:
    """True when translate+orient writes cannot reproduce the authored world pose."""
    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return False
    ops = xformable.GetOrderedXformOps()
    if not ops:
        return False
    if len(ops) == 1 and ops[0].GetOpType() == UsdGeom.XformOp.TypeTransform:
        return False
    if len(ops) <= 2 and all(
        op.GetOpType() in (UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.TypeOrient)
        for op in ops
    ):
        return False
    return True


def _ensure_matrix_xform_op(
    stage: Usd.Stage,
    prim_path: str,
    xform_cache: UsdGeom.XformCache,
) -> bool:
    """Collapse scale/rotate xform stacks into one matrix op without moving the prim."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False
    if not _prim_has_complex_xform_stack(prim):
        return True

    world_mtx = Gf.Matrix4d(xform_cache.GetLocalToWorldTransform(prim))
    parent = prim.GetParent()
    if parent and parent.IsValid():
        parent_world = Gf.Matrix4d(xform_cache.GetLocalToWorldTransform(parent))
        local_mtx = world_mtx * parent_world.GetInverse()
    else:
        local_mtx = world_mtx

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(local_mtx)
    print(f"[shirt_grasp] Baked complex xform stack to matrix op on {prim_path}")
    return True


def _resolve_gripper_joint(stage: Usd.Stage) -> Optional[str]:
    for path in GRIPPER_JOINT_CANDIDATES:
        if stage.GetPrimAtPath(path).IsValid():
            return path
    return None


def _boost_gripper_drive(
    stage: Usd.Stage,
    joint_path: str,
    stiffness: float = GRIPPER_CLOSE_STIFFNESS,
    max_force: float = GRIPPER_CLOSE_MAX_FORCE,
    damping: float = GRIPPER_CLOSE_DAMPING,
) -> bool:
    prim = stage.GetPrimAtPath(joint_path)
    if not prim or not prim.IsValid():
        return False
    updated = False
    for attr_name, value in (
        ("drive:angular:physics:stiffness", stiffness),
        ("drive:angular:physics:maxForce", max_force),
        ("drive:angular:physics:damping", damping),
    ):
        attr = prim.GetAttribute(attr_name)
        if attr and attr.IsValid():
            attr.Set(float(value))
            updated = True
    return updated


def _set_mesh_collision(stage: Usd.Stage, mesh_path: str, enabled: bool) -> None:
    prim = stage.GetPrimAtPath(mesh_path)
    if not prim or not prim.IsValid():
        return
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        if enabled:
            return
        UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(bool(enabled))


def _resolve_tip_paths(stage: Usd.Stage, attach_prim: str) -> list[str]:
    """Prim(s) used for reach tests and snap target; attach_prim is always included."""
    attach = attach_prim.rstrip("/")
    if attach.endswith("/ATTACH") and stage.GetPrimAtPath(attach).IsValid():
        paths: list[str] = [attach]
        attach_prim_obj = stage.GetPrimAtPath(attach)
        finger = attach_prim_obj.GetParent()
        gripper = finger.GetParent() if finger and finger.IsValid() else None
        if gripper and gripper.IsValid():
            grip_base = gripper.GetPath().pathString
            for name in GRIPPER_TIP_NAMES:
                path = f"{grip_base}/{name}"
                if stage.GetPrimAtPath(path).IsValid() and path not in paths:
                    paths.append(path)
        return paths
    paths: list[str] = []
    if stage.GetPrimAtPath(attach).IsValid():
        paths.append(attach)
    base = attach.rsplit("/", 1)[0] if "/" in attach else attach
    for name in GRIPPER_TIP_NAMES:
        path = f"{base.rstrip('/')}/{name}"
        if stage.GetPrimAtPath(path).IsValid() and path not in paths:
            paths.append(path)
    return paths


def resolve_shirt_grip_point(stage: Usd.Stage, shirt_mesh: str, preferred: str = "") -> str:
    """Resolve the shirt Grippoint Xform used for weld alignment."""
    stem = shirt_mesh.rsplit("/", 1)[0] if "/" in shirt_mesh else DEFAULT_SHIRT_STEM
    candidates: list[str] = []
    if preferred:
        candidates.append(preferred.rstrip("/"))
    candidates.extend([f"{stem}/Grippoint", DEFAULT_SHIRT_GRIP_POINT])
    seen: set[str] = set()
    for path in candidates:
        if path in seen:
            continue
        seen.add(path)
        if stage.GetPrimAtPath(path).IsValid():
            return path
    return preferred.rstrip("/") if preferred else DEFAULT_SHIRT_GRIP_POINT


def _stem_cell_for_mesh(mesh_path: str) -> str:
    if "/" in mesh_path:
        return mesh_path.rsplit("/", 1)[0]
    return DEFAULT_SHIRT_STEM


def _world_point(
    stage: Usd.Stage,
    prim_path: str,
    xform_cache: UsdGeom.XformCache,
) -> Optional[Gf.Vec3d]:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None
    return Gf.Vec3d(xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation())


def _nudge_prim_translate_world(
    stage: Usd.Stage,
    prim_path: str,
    delta_world: Gf.Vec3d,
    xform_cache: UsdGeom.XformCache,
) -> bool:
    if float(delta_world.GetLength()) < 1e-6:
        return False
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False
    parent = prim.GetParent()
    delta_local = delta_world
    if parent and parent.IsValid():
        parent_world = Gf.Matrix4d(xform_cache.GetLocalToWorldTransform(parent))
        parent_rot = parent_world.ExtractRotationMatrix()
        delta_local = parent_rot.GetInverse() * Gf.Vec3d(delta_world)

    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return False
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() != UsdGeom.XformOp.TypeTranslate:
            continue
        old = op.Get()
        op.Set(
            Gf.Vec3d(
                float(old[0]) + float(delta_local[0]),
                float(old[1]) + float(delta_local[1]),
                float(old[2]) + float(delta_local[2]),
            )
        )
        return True
    return False


def resolve_gripper_attach_prim(stage: Usd.Stage, preferred: str = "") -> str:
    """Pick the first valid ATTACH marker (user-placed weld point on the gripper)."""
    candidates: list[str] = []
    if preferred:
        candidates.append(preferred.rstrip("/"))
    candidates.extend(GRIPPER_ATTACH_PRIM_CANDIDATES)
    seen: set[str] = set()
    for path in candidates:
        if path in seen:
            continue
        seen.add(path)
        if stage.GetPrimAtPath(path).IsValid():
            return path
    return preferred.rstrip("/") if preferred else GRIPPER_ATTACH_PRIM_CANDIDATES[0]


def _xform_cache(stage: Usd.Stage) -> UsdGeom.XformCache:
    cache = UsdGeom.XformCache()
    try:
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        t = float(timeline.get_current_time())
        fps = float(stage.GetTimeCodesPerSecond()) or 24.0
        cache.SetTime(Usd.TimeCode(t * fps))
    except Exception:
        cache.SetTime(Usd.TimeCode.Default())
    return cache


def _world_bounds(
    stage: Usd.Stage,
    prim_path: str,
    xform_cache: UsdGeom.XformCache,
) -> tuple[Optional[Gf.Vec3d], Optional[Gf.Vec3d], Optional[Gf.Vec3d]]:
    """Return (center, min, max) world AABB for a prim subtree."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None, None, None
    imageable = UsdGeom.Imageable(prim)
    if not imageable:
        pos = Gf.Vec3d(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())
        return pos, pos, pos
    time_code = xform_cache.GetTime() if hasattr(xform_cache, "GetTime") else Usd.TimeCode.Default()
    bound = imageable.ComputeWorldBound(time_code, UsdGeom.Tokens.default_)
    box = bound.ComputeAlignedBox()
    if box.IsEmpty():
        return None, None, None
    mid = Gf.Vec3d(box.GetMidpoint())
    return mid, Gf.Vec3d(box.GetMin()), Gf.Vec3d(box.GetMax())


def _merged_world_bounds(
    stage: Usd.Stage,
    prim_paths: Sequence[str],
    xform_cache: UsdGeom.XformCache,
) -> tuple[Optional[Gf.Vec3d], Optional[Gf.Vec3d], Optional[Gf.Vec3d]]:
    mins: list[Gf.Vec3d] = []
    maxs: list[Gf.Vec3d] = []
    for path in prim_paths:
        _center, box_min, box_max = _world_bounds(stage, path, xform_cache)
        if box_min is not None and box_max is not None:
            mins.append(box_min)
            maxs.append(box_max)
    if not mins:
        return None, None, None
    merged_min = Gf.Vec3d(
        min(m[0] for m in mins),
        min(m[1] for m in mins),
        min(m[2] for m in mins),
    )
    merged_max = Gf.Vec3d(
        max(m[0] for m in maxs),
        max(m[1] for m in maxs),
        max(m[2] for m in maxs),
    )
    merged_center = (merged_min + merged_max) * 0.5
    return merged_center, merged_min, merged_max


def _aabb_overlap(
    min_a: Gf.Vec3d,
    max_a: Gf.Vec3d,
    min_b: Gf.Vec3d,
    max_b: Gf.Vec3d,
    padding: float = 0.0,
) -> bool:
    pad = float(padding)
    return (
        min_a[0] <= max_b[0] + pad
        and max_a[0] + pad >= min_b[0]
        and min_a[1] <= max_b[1] + pad
        and max_a[1] + pad >= min_b[1]
        and min_a[2] <= max_b[2] + pad
        and max_a[2] + pad >= min_b[2]
    )


def _aabb_separation(
    min_a: Gf.Vec3d,
    max_a: Gf.Vec3d,
    min_b: Gf.Vec3d,
    max_b: Gf.Vec3d,
) -> float:
    """Minimum distance between two axis-aligned boxes (0 if overlapping)."""
    dx = max(0.0, max(min_a[0] - max_b[0], min_b[0] - max_a[0]))
    dy = max(0.0, max(min_a[1] - max_b[1], min_b[1] - max_a[1]))
    dz = max(0.0, max(min_a[2] - max_b[2], min_b[2] - max_a[2]))
    return float((Gf.Vec3d(dx, dy, dz)).GetLength())


def _distance_point_to_aabb(point: Gf.Vec3d, box_min: Gf.Vec3d, box_max: Gf.Vec3d) -> float:
    cx = max(box_min[0], min(float(point[0]), box_max[0]))
    cy = max(box_min[1], min(float(point[1]), box_max[1]))
    cz = max(box_min[2], min(float(point[2]), box_max[2]))
    return float((point - Gf.Vec3d(cx, cy, cz)).GetLength())


def _as_rigid_matrix(m: Gf.Matrix4d) -> Gf.Matrix4d:
    """Translation + rotation only (strip any scale/shear from a 4x4)."""
    out = Gf.Matrix4d(1.0)
    out.SetRotate(m.ExtractRotationMatrix())
    out.SetTranslateOnly(Gf.Vec3d(m.ExtractTranslation()))
    return out


def _read_local_scale(prim: Usd.Prim) -> Optional[Gf.Vec3d]:
    scale_attr = prim.GetAttribute("xformOp:scale")
    if scale_attr and scale_attr.IsValid():
        s = scale_attr.Get()
        return Gf.Vec3d(float(s[0]), float(s[1]), float(s[2]))
    return None


def _apply_scale_to_rigid(rigid: Gf.Matrix4d, scale: Gf.Vec3d) -> Gf.Matrix4d:
    s_mtx = Gf.Matrix4d(1.0)
    s_mtx.SetScale(scale)
    return rigid * s_mtx


def _world_matrix(stage: Usd.Stage, prim_path: str, xform_cache: UsdGeom.XformCache) -> Optional[Gf.Matrix4d]:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None
    return Gf.Matrix4d(xform_cache.GetLocalToWorldTransform(prim))


def _set_world_matrix(
    stage: Usd.Stage,
    prim_path: str,
    world_mtx: Gf.Matrix4d,
    xform_cache: UsdGeom.XformCache,
    preserve_scale: bool = True,
) -> bool:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False
    parent = prim.GetParent()
    world_rigid = _as_rigid_matrix(world_mtx)
    local_mtx = world_rigid
    if parent and parent.IsValid():
        parent_world = Gf.Matrix4d(xform_cache.GetLocalToWorldTransform(parent))
        local_mtx = world_rigid * parent_world.GetInverse()

    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return False

    authored_scale = _read_local_scale(prim) if preserve_scale else None

    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTransform:
            local_out = local_mtx
            if preserve_scale:
                old = Gf.Matrix4d(op.Get())
                old_scale = Gf.Vec3d(
                    old.GetRow(0).GetLength(),
                    old.GetRow(1).GetLength(),
                    old.GetRow(2).GetLength(),
                )
                if old_scale.GetLength() > 1e-6:
                    local_out = _apply_scale_to_rigid(local_mtx, old_scale)
            op.Set(Gf.Matrix4d(local_out))
            return True

    translate = local_mtx.ExtractTranslation()
    rot = local_mtx.ExtractRotation()
    quat = rot.GetQuaternion()
    # Prefer existing ops when present.
    t_attr = prim.GetAttribute("xformOp:translate")
    if t_attr and t_attr.IsValid():
        t_attr.Set(Gf.Vec3d(translate))
    else:
        UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(translate))

    orient_attr = prim.GetAttribute("xformOp:orient")
    if orient_attr and orient_attr.IsValid():
        orient_attr.Set(Gf.Quatf(float(quat.GetReal()), *quat.GetImaginary()))
    else:
        rxyz = rot.Decompose(Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
        UsdGeom.XformCommonAPI(prim).SetRotate(
            Gf.Vec3f(float(rxyz[0]), float(rxyz[1]), float(rxyz[2])),
            UsdGeom.XformCommonAPI.RotationOrderXYZ,
        )

    if authored_scale is not None:
        pass  # keep authored scale on child meshes (e.g. test cube)
    return True


def _ensure_translate_orient_ops(stage: Usd.Stage, prim_path: str) -> bool:
    """Ensure a prim can receive world rotation via translate + orient ops."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False
    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return False
    ops = xformable.GetOrderedXformOps()
    has_translate = any(op.GetOpType() == UsdGeom.XformOp.TypeTranslate for op in ops)
    has_orient = any(
        op.GetOpType() in (UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.TypeRotateXYZ)
        for op in ops
    )
    if not has_translate:
        xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    if not has_orient:
        xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    return True


def _grip_local_offset(
    follow_world: Gf.Matrix4d,
    grip_world: Gf.Matrix4d,
    *,
    rotate_with_attach: bool,
) -> Gf.Matrix4d:
    """Local grip offset in follow frame. rotate_with_attach=True welds orientation to ATTACH."""
    local = _as_rigid_matrix(follow_world).GetInverse() * _as_rigid_matrix(grip_world)
    if rotate_with_attach:
        pos = Gf.Vec3d(local.ExtractTranslation())
        out = Gf.Matrix4d(1.0)
        out.SetTranslateOnly(pos)
        return out
    return local


def _follow_world_for_grip(
    attach_world: Gf.Matrix4d,
    grip_local: Gf.Matrix4d,
    *,
    follow_rot_frozen: Optional[Gf.Matrix4d] = None,
) -> Gf.Matrix4d:
    attach_rigid = _as_rigid_matrix(attach_world)
    if follow_rot_frozen is not None:
        grip_pos = Gf.Vec3d(grip_local.ExtractTranslation())
        rot = _as_rigid_matrix(follow_rot_frozen).ExtractRotationMatrix()
        attach_pos = Gf.Vec3d(attach_rigid.ExtractTranslation())
        follow_pos = attach_pos - Gf.Vec3d(rot * grip_pos)
        out = Gf.Matrix4d(1.0)
        out.SetRotate(rot)
        out.SetTranslateOnly(follow_pos)
        return out
    return attach_rigid * grip_local.GetInverse()


    """Hold current shape without stripping deformable APIs (removal pops rest pose)."""
    prim = stage.GetPrimAtPath(mesh_path)
    if not prim or not prim.IsValid():
        return
    if not prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI):
        return
    body_api = PhysxSchema.PhysxDeformableBodyAPI(prim)
    attr = body_api.GetKinematicEnabledAttr()
    if attr and attr.IsValid():
        attr.Set(True)
    else:
        body_api.CreateKinematicEnabledAttr(True)


def _freeze_rigid_for_grasp(stage: Usd.Stage, prim_path: str) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        return
    rb = UsdPhysics.RigidBodyAPI(prim)
    attr = rb.GetKinematicEnabledAttr()
    if attr and attr.IsValid():
        attr.Set(True)
    else:
        rb.CreateKinematicEnabledAttr(True)


def _unfreeze_deformable(stage: Usd.Stage, mesh_path: str) -> None:
    prim = stage.GetPrimAtPath(mesh_path)
    if not prim or not prim.IsValid():
        return
    if not prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI):
        return
    body_api = PhysxSchema.PhysxDeformableBodyAPI(prim)
    attr = body_api.GetKinematicEnabledAttr()
    if attr and attr.IsValid():
        attr.Set(False)


def _unfreeze_rigid(stage: Usd.Stage, prim_path: str) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        return
    if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
        for attr_name, value in (
            ("physics:velocity", Gf.Vec3f(0.0, 0.0, 0.0)),
            ("physics:angularVelocity", Gf.Vec3f(0.0, 0.0, 0.0)),
        ):
            attr = prim.GetAttribute(attr_name)
            if attr and attr.IsValid():
                attr.Set(value)
    rb = UsdPhysics.RigidBodyAPI(prim)
    attr = rb.GetKinematicEnabledAttr()
    if attr and attr.IsValid():
        attr.Set(False)
    else:
        rb.CreateKinematicEnabledAttr(False)


def _freeze_shirt_for_grasp(stage: Usd.Stage, follow_root: str, mesh_path: str) -> None:
    _freeze_deformable_for_grasp(stage, mesh_path)
    _freeze_rigid_for_grasp(stage, follow_root)


def _unfreeze_shirt(stage: Usd.Stage, follow_root: str, mesh_path: str) -> None:
    _unfreeze_deformable(stage, mesh_path)
    _unfreeze_rigid(stage, follow_root)


def _disable_deformable_sim(stage: Usd.Stage, mesh_path: str) -> None:
    """Legacy: prefer _freeze_deformable_for_grasp to avoid rest-pose snap."""
    _freeze_deformable_for_grasp(stage, mesh_path)


class ShirtGraspController:
    """Weld shirt pile to the fingertip (r2) once per grip close."""

    def __init__(
        self,
        stage: Usd.Stage,
        gripper_prim: str = DEFAULT_ATTACH_PRIM,
        shirt_mesh_paths: Sequence[str] = DEFAULT_SHIRT_MESHES,
        mode: str | GraspMode = GraspMode.OVERLAP,
        close_threshold: float = 0.90,
        open_threshold: float = 0.25,
        max_grasp_distance: float = 0.18,
        max_separation: float = 0.12,
        overlap_padding: float = 0.04,
        settle_steps: int = 10,
        follow_mesh: bool = False,
        attach_trigger: str | AttachTrigger = AttachTrigger.GRIP_CLOSE,
        link_from_step: int = 5,
        release_from_step: int = -1,
        snap_to_gripper: bool = False,
        shirt_grip_point: str = "",
        link_rotate: bool = True,
    ) -> None:
        self._stage = stage
        self._attach_prim = gripper_prim.rstrip("/")
        self._reach_prim_paths = _resolve_tip_paths(stage, self._attach_prim)
        if not self._reach_prim_paths:
            self._reach_prim_paths = [self._attach_prim]
        self._shirt_mesh_paths = list(shirt_mesh_paths)
        self._shirt_grip_point = ""
        if shirt_mesh_paths:
            resolved_grip = resolve_shirt_grip_point(
                stage, list(shirt_mesh_paths)[0], shirt_grip_point
            )
            if stage.GetPrimAtPath(resolved_grip).IsValid():
                self._shirt_grip_point = resolved_grip
        self._mode = GraspMode(mode)
        self._close_threshold = float(close_threshold)
        self._open_threshold = float(open_threshold)
        self._max_grasp_distance = float(max_grasp_distance)
        self._max_separation = float(max_separation)
        self._overlap_padding = float(overlap_padding)
        self._settle_steps = max(1, int(settle_steps))
        self._follow_mesh = bool(follow_mesh)
        self._attach_trigger = AttachTrigger(attach_trigger)
        self._link_from_step = int(link_from_step)
        self._release_from_step = int(release_from_step)
        self._snap_to_gripper = bool(snap_to_gripper)
        self._link_rotate = bool(link_rotate)
        self._follow_rot_frozen: Optional[Gf.Matrix4d] = None
        self._gripper_closed = False
        self._attach_attempted = False
        self._close_step = 0
        self._attached_shirt: Optional[str] = None
        self._follow_root: Optional[str] = None
        self._shirt_local_in_attach: Optional[Gf.Matrix4d] = None
        self._grip_local_in_follow: Optional[Gf.Matrix4d] = None
        self._logged_scene_hint = False
        self._gripper_joint = _resolve_gripper_joint(stage)
        self._gripper_drive_boosted = False
        self._shirt_collision_disabled = False

        print(
            f"[shirt_grasp] attach_prim={self._attach_prim} "
            f"gripper_joint={self._gripper_joint} "
            f"reach_prims={self._reach_prim_paths} mode={self._mode.value} "
            f"trigger={self._attach_trigger.value} link_from_step={self._link_from_step} "
            f"release_from_step={self._release_from_step} "
            f"snap={self._snap_to_gripper} link_rotate={self._link_rotate} "
            f"shirt_grip_point={self._shirt_grip_point or 'mesh-center'} "
            f"close>={self._close_threshold:.2f} "
            f"(overlap_pad={self._overlap_padding:.2f}m, max_sep={self._max_separation:.2f}m)"
        )

    @property
    def is_attached(self) -> bool:
        return self._shirt_local_in_attach is not None

    def _pick_target(self) -> tuple[Optional[str], dict[str, object]]:
        xform_cache = _xform_cache(self._stage)
        g_center, g_min, g_max = _merged_world_bounds(
            self._stage, self._reach_prim_paths, xform_cache
        )
        if g_center is None or g_min is None or g_max is None:
            return None, {"reason": "no_gripper_bounds"}

        best_path: Optional[str] = None
        best_metric = float("inf")
        best_info: dict[str, object] = {}

        for mesh_path in self._shirt_mesh_paths:
            if self._shirt_grip_point:
                s_center = _world_point(self._stage, self._shirt_grip_point, xform_cache)
                s_min, s_max = None, None
                if s_center is not None:
                    s_min = s_center
                    s_max = s_center
            else:
                s_center, s_min, s_max = _world_bounds(self._stage, mesh_path, xform_cache)
            if s_center is None or s_min is None or s_max is None:
                continue
            overlap = _aabb_overlap(g_min, g_max, s_min, s_max, self._overlap_padding)
            separation = _aabb_separation(g_min, g_max, s_min, s_max)
            surface_dist = _distance_point_to_aabb(g_center, s_min, s_max)
            metric = separation if self._mode != GraspMode.DISTANCE else surface_dist
            if metric < best_metric:
                best_metric = metric
                best_path = mesh_path
                best_info = {
                    "overlap": overlap,
                    "separation": separation,
                    "surface_dist": surface_dist,
                    "gripper_center": (g_center[0], g_center[1], g_center[2]),
                    "shirt_center": (s_center[0], s_center[1], s_center[2]),
                }
        return best_path, best_info

    def _should_pick(self, info: dict[str, object]) -> bool:
        if self._mode == GraspMode.DEMO:
            return True
        if self._mode == GraspMode.OVERLAP:
            if bool(info.get("overlap")):
                return True
            separation = float(info.get("separation", float("inf")))
            return separation <= self._max_separation
        surface_dist = float(info.get("surface_dist", float("inf")))
        return surface_dist <= self._max_grasp_distance

    def _log_scene_hint(self, info: dict[str, object]) -> None:
        if self._logged_scene_hint or not info:
            return
        self._logged_scene_hint = True
        gc = info.get("gripper_center")
        sc = info.get("shirt_center")
        if gc and sc:
            print(
                "[shirt_grasp] Scene hint — move gripper toward shirt:\n"
                f"  fingertip center ({gc[0]:.3f}, {gc[1]:.3f}, {gc[2]:.3f})\n"
                f"  shirt center   ({sc[0]:.3f}, {sc[1]:.3f}, {sc[2]:.3f})\n"
                f"  delta          ({sc[0]-gc[0]:+.3f}, {sc[1]-gc[1]:+.3f}, {sc[2]-gc[2]:+.3f})"
            )

    def _follow_root_for(self, shirt_mesh_path: str) -> str:
        if self._shirt_grip_point:
            return _shirt_root_for_mesh(shirt_mesh_path)
        if self._follow_mesh:
            return shirt_mesh_path
        return _shirt_root_for_mesh(shirt_mesh_path)

    def _nudge_shirt_group_for_grippoint(
        self,
        shirt_mesh_path: str,
        delta_world: Gf.Vec3d,
        xform_cache: UsdGeom.XformCache,
    ) -> str:
        """Move stem cell or pile root so Grippoint and mesh travel together."""
        for path in (
            _stem_cell_for_mesh(shirt_mesh_path),
            _shirt_root_for_mesh(shirt_mesh_path),
        ):
            if not self._stage.GetPrimAtPath(path).IsValid():
                continue
            _ensure_matrix_xform_op(self._stage, path, xform_cache)
            if _nudge_prim_translate_world(self._stage, path, delta_world, xform_cache):
                return path
            follow_world = _world_matrix(self._stage, path, xform_cache)
            if follow_world is None:
                continue
            new_t = follow_world.ExtractTranslation() + delta_world
            new_world = Gf.Matrix4d(follow_world)
            new_world.SetTranslateOnly(Gf.Vec3d(float(new_t[0]), float(new_t[1]), float(new_t[2])))
            if _set_world_matrix(self._stage, path, new_world, xform_cache):
                return path
        return ""

    def _snap_shirt_to_reach(
        self,
        shirt_mesh_path: str,
        xform_cache: UsdGeom.XformCache,
    ) -> bool:
        """Align shirt Grippoint (or mesh center) to the gripper ATTACH / fingertip."""
        g_center, _, _ = _merged_world_bounds(self._stage, self._reach_prim_paths, xform_cache)
        if g_center is None:
            g_center = _world_point(self._stage, self._attach_prim, xform_cache)
        if g_center is None:
            return False

        if self._shirt_grip_point:
            s_center = _world_point(self._stage, self._shirt_grip_point, xform_cache)
            if s_center is None:
                print(f"[shirt_grasp] Snap skipped: Grippoint not found at {self._shirt_grip_point}")
                return False
            delta = g_center - s_center
            moved = self._nudge_shirt_group_for_grippoint(shirt_mesh_path, delta, xform_cache)
            if moved:
                print(
                    f"[shirt_grasp] Snapped Grippoint to gripper via {moved} "
                    f"(world delta {float(delta[0]):+.3f}, {float(delta[1]):+.3f}, {float(delta[2]):+.3f}) m"
                )
                return True
            print("[shirt_grasp] Snap skipped: could not move shirt stem/pile root")
            return False

        s_center, s_min, s_max = _world_bounds(self._stage, shirt_mesh_path, xform_cache)
        if s_center is None or s_min is None or s_max is None:
            return False
        delta = g_center - s_center
        if _nudge_prim_translate_world(self._stage, shirt_mesh_path, delta, xform_cache):
            print(
                f"[shirt_grasp] Snapped shirt mesh center to gripper "
                f"(world delta {float(delta[0]):+.3f}, {float(delta[1]):+.3f}, {float(delta[2]):+.3f}) m"
            )
            return True
        print("[shirt_grasp] Snap skipped: no xformOp:translate on shirt mesh")
        return False

    def _attach(self, shirt_mesh_path: str, info: dict[str, object]) -> bool:
        follow_root = self._follow_root_for(shirt_mesh_path)
        if not self._stage.GetPrimAtPath(follow_root).IsValid():
            print(f"[shirt_grasp] Shirt follow prim not found: {follow_root}")
            return False

        xform_cache = _xform_cache(self._stage)

        if self._snap_to_gripper and (self._follow_mesh or self._shirt_grip_point):
            self._snap_shirt_to_reach(shirt_mesh_path, xform_cache)
            xform_cache = _xform_cache(self._stage)

        attach_world = _world_matrix(self._stage, self._attach_prim, xform_cache)
        follow_world = _world_matrix(self._stage, follow_root, xform_cache)
        mesh_world = _world_matrix(self._stage, shirt_mesh_path, xform_cache)
        if attach_world is None or follow_world is None:
            print("[shirt_grasp] Could not read attach prim or shirt transform")
            return False

        self._grip_local_in_follow = None
        self._follow_rot_frozen = None
        if self._shirt_grip_point:
            _ensure_translate_orient_ops(self._stage, follow_root)
            xform_cache = _xform_cache(self._stage)
            grip_world = _world_matrix(self._stage, self._shirt_grip_point, xform_cache)
            if grip_world is not None:
                self._grip_local_in_follow = _grip_local_offset(
                    follow_world,
                    grip_world,
                    rotate_with_attach=self._link_rotate,
                )
                if not self._link_rotate:
                    self._follow_rot_frozen = _as_rigid_matrix(follow_world)
                follow_world = _follow_world_for_grip(
                    attach_world,
                    self._grip_local_in_follow,
                    follow_rot_frozen=self._follow_rot_frozen,
                )
                _set_world_matrix(self._stage, follow_root, follow_world, xform_cache)
                xform_cache = _xform_cache(self._stage)
                follow_world = _world_matrix(self._stage, follow_root, xform_cache)
                grip_world = _world_matrix(self._stage, self._shirt_grip_point, xform_cache)
                if grip_world is not None and follow_world is not None:
                    self._grip_local_in_follow = _grip_local_offset(
                        follow_world,
                        grip_world,
                        rotate_with_attach=self._link_rotate,
                    )

        attach_world = _world_matrix(self._stage, self._attach_prim, xform_cache)
        follow_world = _world_matrix(self._stage, follow_root, xform_cache)
        if attach_world is None or follow_world is None:
            print("[shirt_grasp] Could not read transforms after grip alignment")
            return False

        self._shirt_local_in_attach = (
            _as_rigid_matrix(follow_world) * _as_rigid_matrix(attach_world).GetInverse()
        )
        self._follow_root = follow_root
        self._attached_shirt = shirt_mesh_path
        if self._follow_mesh or self._shirt_grip_point:
            _freeze_shirt_for_grasp(self._stage, follow_root, shirt_mesh_path)
        else:
            _freeze_shirt_for_grasp(self._stage, follow_root, shirt_mesh_path)

        gc = info.get("gripper_center", ("?", "?", "?"))
        sc = info.get("shirt_center", ("?", "?", "?"))
        mesh_t = mesh_world.ExtractTranslation() if mesh_world is not None else None
        if self._shirt_grip_point:
            target = "pile+Grippoint" + ("+rotate" if self._link_rotate else "")
        elif self._follow_mesh:
            target = "mesh"
        else:
            target = "pile"
        print(
            f"[shirt_grasp] Linked {follow_root} to {self._attach_prim} ({target}) "
            f"(overlap={info.get('overlap')} separation={float(info.get('separation', -1)):.3f}m "
            f"surface_dist={float(info.get('surface_dist', -1)):.3f}m)"
        )
        print(
            f"[shirt_grasp]   attach={self._attach_prim} "
            f"fingertip=({gc[0]:.3f},{gc[1]:.3f},{gc[2]:.3f}) "
            f"shirt=({sc[0]:.3f},{sc[1]:.3f},{sc[2]:.3f})"
        )
        if mesh_t is not None:
            print(
                f"[shirt_grasp]   mesh_world=({mesh_t[0]:.3f},{mesh_t[1]:.3f},{mesh_t[2]:.3f}) "
                f"(local pose on {shirt_mesh_path.rsplit('/', 1)[-1]} is preserved)"
            )
        return True

    def _prepare_for_grip_close(self) -> None:
        """Soft deformable shirts can push weak gripper drives open — boost close force."""
        if self._gripper_joint and not self._gripper_drive_boosted:
            if _boost_gripper_drive(self._stage, self._gripper_joint):
                self._gripper_drive_boosted = True
                print(
                    f"[shirt_grasp] Boosted {self._gripper_joint} drive "
                    f"(stiffness={GRIPPER_CLOSE_STIFFNESS}, maxForce={GRIPPER_CLOSE_MAX_FORCE})"
                )
        if not self._shirt_collision_disabled:
            for mesh_path in self._shirt_mesh_paths:
                _set_mesh_collision(self._stage, mesh_path, False)
            self._shirt_collision_disabled = True
            print("[shirt_grasp] Disabled shirt mesh collision during grip close")

    def _restore_after_release(self) -> None:
        if self._attached_shirt and self._follow_root:
            _unfreeze_shirt(self._stage, self._follow_root, self._attached_shirt)
        if self._shirt_collision_disabled:
            for mesh_path in self._shirt_mesh_paths:
                _set_mesh_collision(self._stage, mesh_path, True)
            self._shirt_collision_disabled = False
        self._gripper_drive_boosted = False

    def _release(self) -> None:
        if self._attached_shirt:
            print(f"[shirt_grasp] Released {self._attached_shirt}")
        self._restore_after_release()
        self._attached_shirt = None
        self._follow_root = None
        self._shirt_local_in_attach = None
        self._grip_local_in_follow = None
        self._follow_rot_frozen = None
        self._close_step = 0
        self._attach_attempted = False

    def _update_follow(self) -> None:
        if self._shirt_local_in_attach is None or not self._follow_root:
            return
        xform_cache = _xform_cache(self._stage)
        raw_attach = _world_matrix(self._stage, self._attach_prim, xform_cache)
        if raw_attach is None:
            return
        attach_world = _as_rigid_matrix(raw_attach)
        if self._grip_local_in_follow is not None and self._shirt_grip_point:
            _ensure_translate_orient_ops(self._stage, self._follow_root)
            follow_world = _follow_world_for_grip(
                attach_world,
                self._grip_local_in_follow,
                follow_rot_frozen=self._follow_rot_frozen,
            )
        else:
            follow_world = self._shirt_local_in_attach * attach_world
        if not _set_world_matrix(self._stage, self._follow_root, follow_world, xform_cache):
            print(f"[shirt_grasp] Failed to move {self._follow_root}")

    def _try_attach_on_grip_close(self) -> None:
        shirt_path, info = self._pick_target()
        if shirt_path is None:
            print("[shirt_grasp] Grip close: no shirt mesh found in stage")
            return
        self._log_scene_hint(info)
        if not self._should_pick(info):
            print(
                f"[shirt_grasp] Grip close: no pick — "
                f"overlap={info.get('overlap')} "
                f"separation={float(info.get('separation', -1)):.3f}m "
                f"surface_dist={float(info.get('surface_dist', -1)):.3f}m "
                f"(mode={self._mode.value})"
            )
            gc = info.get("gripper_center")
            sc = info.get("shirt_center")
            if gc and sc:
                print(
                    f"[shirt_grasp]   move gripper by "
                    f"({sc[0]-gc[0]:+.3f}, {sc[1]-gc[1]:+.3f}, {sc[2]-gc[2]:+.3f}) m"
                )
            return
        self._attach(shirt_path, info)

    def _try_force_attach(self, reason: str) -> None:
        if self.is_attached:
            return
        self._prepare_for_grip_close()
        shirt_path, info = self._pick_target()
        if shirt_path is None and self._shirt_mesh_paths:
            shirt_path = self._shirt_mesh_paths[0]
            info = {"reason": reason}
        if shirt_path is None:
            print(f"[shirt_grasp] {reason}: no shirt mesh found in stage")
            return
        try:
            if self._attach(shirt_path, info):
                print(f"[shirt_grasp] Attached on {reason}")
        except Exception as exc:
            print(f"[shirt_grasp] Attach error ({reason}): {exc!r}")

    def _update_by_motion_step(self, motion_step: Optional[int]) -> None:
        if motion_step is None:
            return
        if (
            self._release_from_step >= 0
            and motion_step >= self._release_from_step
        ):
            if self.is_attached:
                self._release()
            return
        if motion_step >= self._link_from_step:
            if not self.is_attached:
                self._try_force_attach(f"motion_step={motion_step}")
            return
        if self.is_attached:
            self._release()

    def follow_after_physics(self) -> None:
        """Update shirt world pose from gripper — call after world.step()."""
        if self.is_attached:
            self._update_follow()

    def update(
        self,
        gripper_r_joint1: Optional[float],
        gripper_r_joint1_measured: Optional[float] = None,
        motion_step: Optional[int] = None,
    ) -> None:
        if self._attach_trigger == AttachTrigger.MOTION_STEP:
            self._update_by_motion_step(motion_step)
            return
        # Use commanded grip only — measured joint can be pushed open by shirt contact.
        if gripper_r_joint1 is None:
            if gripper_r_joint1_measured is None:
                return
            grip = float(gripper_r_joint1_measured)
        else:
            grip = float(gripper_r_joint1)

        if not self._gripper_closed and grip >= self._close_threshold:
            self._gripper_closed = True
            self._close_step = 0
            self._attach_attempted = False
            self._prepare_for_grip_close()
        elif self._gripper_closed and grip <= self._open_threshold:
            self._gripper_closed = False
            self._release()
            return

        if self.is_attached:
            return

        if not self._gripper_closed or self._attach_attempted:
            return

        self._close_step += 1
        if self._close_step < self._settle_steps:
            return

        self._attach_attempted = True
        try:
            self._try_attach_on_grip_close()
        except Exception as exc:
            print(f"[shirt_grasp] Attach error: {exc!r}")


def build_shirt_grasp_controller(
    stage: Usd.Stage,
    gripper_prim: str = DEFAULT_ATTACH_PRIM,
    shirt_mesh_paths: Iterable[str] = DEFAULT_SHIRT_MESHES,
    mode: str = GraspMode.OVERLAP.value,
    follow_mesh: bool = False,
    attach_trigger: str = AttachTrigger.GRIP_CLOSE.value,
    link_from_step: int = 5,
    release_from_step: int = -1,
    snap_to_gripper: bool = False,
    shirt_grip_point: str = "",
    link_rotate: bool = True,
) -> ShirtGraspController:
    return ShirtGraspController(
        stage,
        gripper_prim=gripper_prim,
        shirt_mesh_paths=list(shirt_mesh_paths),
        mode=mode,
        follow_mesh=follow_mesh,
        attach_trigger=attach_trigger,
        link_from_step=link_from_step,
        release_from_step=release_from_step,
        snap_to_gripper=snap_to_gripper,
        shirt_grip_point=shirt_grip_point,
        link_rotate=link_rotate,
    )
