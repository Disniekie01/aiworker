#!/usr/bin/env python3
"""Bake YAML/JSON animation into a USD layer (joint drives + base + shirt xform).

Requires OpenUSD (pxr) — run with Isaac Sim's python.sh, not system python3:

  ~/isaacsim/python.sh scripts/yaml_animation_to_usd.py \\
    --animation exports/pick_place_animation.json \\
    --scene /path/to/Scene_clean.usda \\
    --output exports/pick_place_anim.usda
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

try:
    from pxr import Gf, Sdf, Usd, UsdGeom
except ImportError as exc:  # pragma: no cover - env dependent
    raise SystemExit(
        "OpenUSD (pxr) is required. Run this script with Isaac Sim's python.sh:\n"
        "  ~/isaacsim/python.sh scripts/yaml_animation_to_usd.py --help"
    ) from exc

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from yaml_animation_sampler import FULL_JOINT_ORDER  # noqa: E402

DRIVE_REVOLUTE = "drive:angular:physics:targetPosition"
DRIVE_PRISMATIC = "drive:linear:physics:targetPosition"
PRISMATIC_JOINTS = {"lift_joint"}

DEFAULT_SCENE = Path("/home/disniekie/Robotis/Scene_clean.usda")
DEFAULT_JOINTS_ROOT = "/World/Robot/ffw_sg2_follower/joints"
DEFAULT_BASE_PRIM = "/World/Robot"
DEFAULT_SHIRT_MESH = (
    "/World/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell_01/"
    "Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell/TShirts_Hanging_V_neck_04"
)


def _as_abs(path: Path) -> str:
    return path.resolve().as_posix()


def _ensure_float_attr(prim: Usd.Prim, name: str) -> Usd.Attribute:
    attr = prim.GetAttribute(name)
    if attr and attr.IsValid():
        return attr
    return prim.CreateAttribute(name, Sdf.ValueTypeNames.Float)


def _ensure_shirt_ops(xformable: UsdGeom.Xformable) -> tuple[UsdGeom.XformOp, UsdGeom.XformOp]:
    translate = xformable.GetTranslateOp()
    rotate = None
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            rotate = op
            break
    if translate is None:
        translate = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    if rotate is None:
        rotate = xformable.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble)
    return translate, rotate


def _ensure_base_ops(xformable: UsdGeom.Xformable) -> tuple[UsdGeom.XformOp, UsdGeom.XformOp]:
    translate = xformable.GetTranslateOp()
    orient = xformable.GetOrientOp()
    if translate is None:
        translate = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    if orient is None:
        orient = xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    return translate, orient


def _apply_base_pose(
    translate_op: UsdGeom.XformOp,
    orient_op: UsdGeom.XformOp,
    frame: dict,
    time_code: Usd.TimeCode,
) -> None:
    """Match standalone_ffw_joint_teleop: translate + yaw about world Z (Z-up stage)."""
    x = float(frame.get("isaac_base_x", 0.0))
    y = float(frame.get("isaac_base_y", 0.0))
    z = float(frame.get("isaac_base_z", 0.0))
    yaw = float(frame.get("isaac_base_yaw", 0.0))
    translate_op.Set(Gf.Vec3d(x, y, z), time_code)
    rot = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), math.degrees(yaw))
    orient_op.Set(rot.GetQuat(), time_code)


def export_animation_usd(
    anim: dict,
    scene_path: Path,
    output_path: Path,
    *,
    joints_root: str = DEFAULT_JOINTS_ROOT,
    base_prim: str = DEFAULT_BASE_PRIM,
    shirt_mesh: str = DEFAULT_SHIRT_MESH,
    animate_base: bool = True,
    flatten: bool = False,
) -> None:
    frames = anim.get("frames", [])
    if not frames:
        raise ValueError("Animation JSON has no frames")

    fps = float(anim.get("fps", 30.0))
    output_path.parent.mkdir(parents=True, exist_ok=True)

    stage = Usd.Stage.CreateNew(_as_abs(output_path))
    root_layer = stage.GetRootLayer()
    root_layer.subLayerPaths.append(_as_abs(scene_path))

    stage.SetTimeCodesPerSecond(fps)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(len(frames) - 1)

    base = stage.OverridePrim(base_prim)
    base_xformable = UsdGeom.Xformable(base)
    base_ops: tuple[UsdGeom.XformOp, UsdGeom.XformOp] | None = None
    if animate_base:
        base_ops = _ensure_base_ops(base_xformable)

    shirt_xformable = UsdGeom.Xformable(stage.OverridePrim(shirt_mesh)) if shirt_mesh else None
    shirt_ops: tuple[UsdGeom.XformOp, UsdGeom.XformOp] | None = None
    if shirt_xformable:
        shirt_ops = _ensure_shirt_ops(shirt_xformable)

    missing_joints: list[str] = []
    for frame_idx, frame in enumerate(frames):
        time_code = Usd.TimeCode(frame_idx)

        if base_ops is not None:
            _apply_base_pose(base_ops[0], base_ops[1], frame, time_code)

        for joint_name in FULL_JOINT_ORDER:
            if joint_name not in frame:
                continue
            joint_path = f"{joints_root.rstrip('/')}/{joint_name}"
            joint_prim = stage.OverridePrim(joint_path)
            if not joint_prim or not joint_prim.IsValid():
                missing_joints.append(joint_path)
                continue
            value = float(frame[joint_name])
            if joint_name in PRISMATIC_JOINTS:
                _ensure_float_attr(joint_prim, DRIVE_PRISMATIC).Set(value, time_code)
            else:
                _ensure_float_attr(joint_prim, DRIVE_REVOLUTE).Set(value, time_code)

        if shirt_ops is not None:
            tx = float(frame.get("shirt_tx", 0.0))
            ty = float(frame.get("shirt_ty", 0.0))
            tz = float(frame.get("shirt_tz", 0.0))
            rx = float(frame.get("shirt_rx", 0.0))
            ry = float(frame.get("shirt_ry", 0.0))
            rz = float(frame.get("shirt_rz", 0.0))
            shirt_ops[0].Set(Gf.Vec3d(tx, ty, tz), time_code)
            shirt_ops[1].Set(Gf.Vec3d(rx, ry, rz), time_code)

    if missing_joints:
        unique = sorted(set(missing_joints))
        print(f"Warning: could not override {len(unique)} joint path(s), e.g. {unique[0]}")

    stage.GetRootLayer().Save()
    print(
        f"Wrote {_as_abs(output_path)} "
        f"({len(frames)} frames @ {fps} Hz, subLayer={_as_abs(scene_path)})"
    )

    if flatten:
        flat_path = output_path.with_name(output_path.stem + "_flat.usdc")
        flat_stage = Usd.Stage.Open(_as_abs(output_path))
        flat_stage.Export(_as_abs(flat_path), addSourceAssetHint=True)
        print(f"Wrote flattened {_as_abs(flat_path)}")


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Export YAML-sampled animation to animated USD")
    parser.add_argument("--animation", required=True, help="Animation JSON from yaml_animation_sampler.py")
    parser.add_argument("--scene", default=str(DEFAULT_SCENE), help="Base scene USD (referenced as subLayer)")
    parser.add_argument("--output", default=str(root / "exports" / "pick_place_anim.usda"))
    parser.add_argument("--joints-root", default=DEFAULT_JOINTS_ROOT)
    parser.add_argument("--base-prim", default=DEFAULT_BASE_PRIM)
    parser.add_argument("--shirt-mesh", default=DEFAULT_SHIRT_MESH)
    parser.add_argument(
        "--skip-base",
        action="store_true",
        help="Do not animate /World/Robot translate/yaw (crate no longer appears to spin)",
    )
    parser.add_argument(
        "--flatten",
        action="store_true",
        help="Also write a flattened .usdc (large; self-contained)",
    )
    args = parser.parse_args()

    anim_path = Path(args.animation).expanduser()
    scene_path = Path(args.scene).expanduser()
    output_path = Path(args.output).expanduser()

    if not anim_path.is_file():
        raise FileNotFoundError(f"Animation JSON not found: {anim_path}")
    if not scene_path.is_file():
        raise FileNotFoundError(f"Scene USD not found: {scene_path}")

    anim = json.loads(anim_path.read_text(encoding="utf-8"))
    export_animation_usd(
        anim,
        scene_path,
        output_path,
        joints_root=args.joints_root,
        base_prim=args.base_prim,
        shirt_mesh=args.shirt_mesh,
        animate_base=not args.skip_base,
        flatten=args.flatten,
    )


if __name__ == "__main__":
    main()
