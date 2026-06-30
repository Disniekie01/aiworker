# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""Apply local mesh pose from tuner / YAML (shirt_tx … shirt_rz on the shirt mesh)."""

from __future__ import annotations

from typing import Mapping, Optional

from pxr import Gf, Usd, UsdGeom

DEFAULT_SHIRT_ROOT = "/World/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell_01"
DEFAULT_SHIRT_MESH = (
    f"{DEFAULT_SHIRT_ROOT}/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell/TShirts_Hanging_V_neck_04"
)
SHIRT_POSE_KEYS = (
    "shirt_tx",
    "shirt_ty",
    "shirt_tz",
    "shirt_rx",
    "shirt_ry",
    "shirt_rz",
)
SHIRT_TRANSLATE_KEYS = ("shirt_tx", "shirt_ty", "shirt_tz")
SHIRT_ROTATE_KEYS = ("shirt_rx", "shirt_ry", "shirt_rz")
# Local translate / rotate XYZ (degrees) on the shirt mesh — matches Isaac property panel.
DEFAULT_SHIRT_POSE = (
    -0.287,
    -3.512,
    3.799,
    0.0,
    0.0,
    0.0,
)


def shirt_pose_from_mapping(
    values: Mapping[str, float],
) -> Optional[tuple[float, float, float, float, float, float]]:
    if not all(k in values for k in SHIRT_TRANSLATE_KEYS):
        return None
    tx, ty, tz = (float(values[k]) for k in SHIRT_TRANSLATE_KEYS)
    if all(k in values for k in SHIRT_ROTATE_KEYS):
        return tx, ty, tz, float(values["shirt_rx"]), float(values["shirt_ry"]), float(values["shirt_rz"])
    return tx, ty, tz, 0.0, 0.0, 0.0


def apply_shirt_pose(
    stage: Usd.Stage,
    prim_path: str,
    tx: float,
    ty: float,
    tz: float,
    rx: float,
    ry: float,
    rz: float,
) -> bool:
    """Set local translate + rotateXYZ (degrees) — same fields as Isaac property panel."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False

    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return False

    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(tx, ty, tz))
    xformable.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(rx, ry, rz))
    xformable.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(1.0, 1.0, 1.0))
    return True


def apply_shirt_pose_from_values(
    stage: Usd.Stage,
    prim_path: str,
    values: Mapping[str, float],
) -> bool:
    pose = shirt_pose_from_mapping(values)
    if pose is None:
        return False
    return apply_shirt_pose(stage, prim_path, *pose)
