# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""Apply local crate pose from tuner / YAML (crate_tx … crate_rz on the crate root)."""

from __future__ import annotations

from typing import Mapping, Optional

from pxr import Gf, Usd, UsdGeom

DEFAULT_CRATE_PRIM = "/World/KB3D_CTS_Crate_A"
CRATE_POSE_KEYS = (
    "crate_tx",
    "crate_ty",
    "crate_tz",
    "crate_rx",
    "crate_ry",
    "crate_rz",
)
CRATE_TRANSLATE_KEYS = ("crate_tx", "crate_ty", "crate_tz")
CRATE_ROTATE_KEYS = ("crate_rx", "crate_ry", "crate_rz")
DEFAULT_CRATE_POSE = (
    0.1638139638914529,
    -0.689979076385498,
    1.040000081062317,
    0.0,
    0.0,
    90.0,
)


def crate_pose_from_mapping(
    values: Mapping[str, float],
) -> Optional[tuple[float, float, float, float, float, float]]:
    if not all(k in values for k in CRATE_TRANSLATE_KEYS):
        return None
    tx, ty, tz = (float(values[k]) for k in CRATE_TRANSLATE_KEYS)
    if all(k in values for k in CRATE_ROTATE_KEYS):
        return tx, ty, tz, float(values["crate_rx"]), float(values["crate_ry"]), float(values["crate_rz"])
    return tx, ty, tz, 0.0, 0.0, 90.0


def _set_existing_vec3_attr(prim: Usd.Prim, attr_name: str, value: Gf.Vec3d) -> bool:
    attr = prim.GetAttribute(attr_name)
    if not attr or not attr.IsValid():
        return False
    attr.Set(value)
    return True


def apply_crate_pose(
    stage: Usd.Stage,
    prim_path: str,
    tx: float,
    ty: float,
    tz: float,
    rx: float,
    ry: float,
    rz: float,
) -> bool:
    """Update translate + rotateXYZ only — preserve FBX scale/unitsResolve ops."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False

    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return False

    wrote = False
    for op in xformable.GetOrderedXformOps():
        op_type = op.GetOpType()
        if op_type == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(float(tx), float(ty), float(tz)))
            wrote = True
        elif op_type == UsdGeom.XformOp.TypeRotateXYZ:
            op.Set(Gf.Vec3d(float(rx), float(ry), float(rz)))
            wrote = True

    if wrote:
        return True

    # First-time setup: add translate/rotate before any existing scale ops.
    if _set_existing_vec3_attr(prim, "xformOp:translate", Gf.Vec3d(tx, ty, tz)):
        _set_existing_vec3_attr(prim, "xformOp:rotateXYZ", Gf.Vec3d(rx, ry, rz))
        return True

    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(tx, ty, tz))
    xformable.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(rx, ry, rz))
    return True


def apply_crate_pose_from_values(
    stage: Usd.Stage,
    prim_path: str,
    values: Mapping[str, float],
) -> bool:
    pose = crate_pose_from_mapping(values)
    if pose is None:
        return False
    return apply_crate_pose(stage, prim_path, *pose)
