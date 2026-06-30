# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""
Standalone Isaac Sim: load FFW USD and mirror /ffw_isaac/joint_targets (sensor_msgs/JointState).

Run inside Isaac Sim's Python (not system python), e.g.:
  ./python.sh standalone_ffw_joint_teleop.py --usd_path /path/to/FFW_SG2.usd --articulation_prim /World/Robot

Requires rclpy in the same environment as Isaac (Jazzy ROS 2). Match ROS_DOMAIN_ID with the Vuer DDS relay.
"""
from __future__ import annotations

import argparse
import contextlib
import json
import os
import socket
import sys
import threading
import time
from typing import Dict, List, Optional

# If launched from terminals that sourced env_local.bash, PYTHONPATH may include
# Python 3.12 packages that are incompatible with Isaac's Python 3.11 runtime.
_BAD_SITE_HINT = "/.venv/lib/python3.12/site-packages"
if _BAD_SITE_HINT in os.environ.get("PYTHONPATH", ""):
    py_entries = [p for p in os.environ["PYTHONPATH"].split(":") if _BAD_SITE_HINT not in p]
    os.environ["PYTHONPATH"] = ":".join(py_entries)
sys.path = [p for p in sys.path if _BAD_SITE_HINT not in p]

import numpy as np
import torch

with contextlib.suppress(ModuleNotFoundError):
    import isaacsim  # noqa: F401

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--usd_path", type=str, required=True, help="Absolute path to FFW SG2/BG2 USD")
parser.add_argument(
    "--articulation_prim",
    type=str,
    default="",
    help="Articulation root prim path. If empty, auto-detect under --spawn_prim.",
)
parser.add_argument(
    "--start-pose-yaml",
    type=str,
    default="",
    help="YAML config for initial robot pose (default: pick_place Start step when set)",
)
parser.add_argument(
    "--start-pose-step",
    type=str,
    default="Start",
    help="Sequence step name to apply at startup (default: Start)",
)
parser.add_argument(
    "--spawn_prim",
    type=str,
    default="/World/Robot",
    help="Prim path where USD is spawned",
)
parser.add_argument(
    "--joint_state_topic",
    type=str,
    default="/ffw_isaac/joint_targets",
    help="sensor_msgs/JointState from ffw_vuer_dds_relay",
)
parser.add_argument(
    "--input_mode",
    type=str,
    default="auto",
    choices=["auto", "ros2", "udp"],
    help="Command source: ROS2 subscription (needs rclpy in Isaac env) or UDP bridge",
)
parser.add_argument(
    "--udp_host",
    type=str,
    default="127.0.0.1",
    help="UDP host for joint target bridge in UDP mode",
)
parser.add_argument(
    "--udp_port",
    type=int,
    default=15000,
    help="UDP port for joint target bridge in UDP mode",
)
parser.add_argument("--headless", action="store_true")
parser.add_argument(
    "--command_smoothing",
    type=float,
    default=0.0,
    help="Exponential smoothing on joint targets (0=off, 0.15-0.3 recommended)",
)
parser.add_argument(
    "--base_move_prim",
    type=str,
    default="/World/Robot",
    help="USD prim whose world pose is set from isaac_base_* fields in joint targets",
)
parser.add_argument(
    "--soft-shirts",
    action=argparse.BooleanOptionalAction,
    default=True,
    help="GPU deformable t-shirt bodies (default: on). Use --no-soft-shirts for rigid.",
)
parser.add_argument(
    "--shirt-link-gripper",
    action=argparse.BooleanOptionalAction,
    default=False,
    help="Link shirt mesh to gripper finger at GRIPP sequence step (motion_step)",
)
parser.add_argument(
    "--shirt-link-finger",
    type=str,
    default="attach",
    choices=["attach", "r2", "l2"],
    help="Shirt weld target: attach=ATTACH marker on gripper (default), or r2/l2 finger",
)
parser.add_argument(
    "--shirt-link-step",
    type=str,
    default="GRIPP",
    help="Sequence step name that triggers shirt link (default: GRIPP)",
)
parser.add_argument(
    "--shirt-link-snap",
    action=argparse.BooleanOptionalAction,
    default=True,
    help="At link time, move shirt mesh center to gripper fingertip center (default: on)",
)
parser.add_argument(
    "--shirt-release-step",
    type=str,
    default="Drop",
    help="Sequence step that releases the shirt link (default: Drop). Use 'none' to keep welded until rewind.",
)
parser.add_argument(
    "--shirt-link-rotate",
    action=argparse.BooleanOptionalAction,
    default=True,
    help="On link, rotate the object with the gripper ATTACH frame (default: on)",
)
parser.add_argument(
    "--grasp-test-cube",
    action=argparse.BooleanOptionalAction,
    default=False,
    help="Hide shirt pile and spawn a flat rigid cube for grip-link testing",
)
parser.add_argument(
    "--grasp-mode",
    type=str,
    default="",
    choices=["", "xform", "constraint"],
    help="Grasp hold: constraint=PhysX FixedJoint (default with --grasp-test-cube); "
    "xform=kinematic USD follow (soft shirts / fallback)",
)
parser.add_argument(
    "--physics-grasp",
    action=argparse.BooleanOptionalAction,
    default=False,
    help="Dynamic rigid shirt + gripper friction; no attach weld or FixedJoint (real PhysX contact)",
)
parser.add_argument(
    "--physics-grasp-log-dir",
    type=str,
    default="",
    help="JSONL investigation log for --physics-grasp (default: <project>/logs/physics_grasp)",
)
parser.add_argument(
    "--shirt-grasp",
    action=argparse.BooleanOptionalAction,
    default=False,
    help="Legacy overlap-based shirt weld (or use --shirt-link-gripper)",
)
parser.add_argument(
    "--shirt-prim",
    type=str,
    default="/World/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell_01/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell/TShirts_Hanging_V_neck_04",
    help="USD mesh prim for local shirt_tx/ty/tz/rx/ry/rz (degrees)",
)
parser.add_argument(
    "--crate-prim",
    type=str,
    default="/World/KB3D_CTS_Crate_A",
    help="USD root prim for local crate_tx/ty/tz/rx/ry/rz (degrees)",
)
parser.add_argument(
    "--shirt-root",
    type=str,
    default="",
    help="Extra shirt root for scene physics (e.g. /World/Shirt for VR pickup)",
)
parser.add_argument(
    "--shirt-grip-point",
    type=str,
    default="/World/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell_01/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell/Grippoint",
    help="Shirt Grippoint Xform aligned to gripper ATTACH on link (default: .../Grippoint)",
)
parser.add_argument(
    "--shirt-grasp-mode",
    type=str,
    default="overlap",
    choices=["overlap", "distance", "demo"],
    help="overlap=gripper/shirt AABB touch; distance=fingertip proximity; demo=always pick (test lift)",
)
parser.add_argument(
    "--gripper-r-prim",
    type=str,
    default="/World/Robot/ffw_sg2_follower/right_gripper/gripper_r_rh_p12_rn_r2",
    help="Right fingertip prim to weld the shirt to (gripper_r_rh_p12_rn_r2)",
)
SEQUENCE_STEP_NAMES = [
    "Start",
    "Ready",
    "Rotatetocrate",
    "GripShirt",
    "Gripdown",
    "GRIPP",
    "Lift",
    "Turn",
    "Drop",
    "Button",
    "press",
    "Rest",
]
args_cli = parser.parse_args()
shirt_link_from_step = 5
if args_cli.shirt_link_step in SEQUENCE_STEP_NAMES:
    shirt_link_from_step = SEQUENCE_STEP_NAMES.index(args_cli.shirt_link_step)
shirt_release_from_step = -1
_release_step = args_cli.shirt_release_step.strip()
if _release_step.lower() not in ("", "none", "off"):
    if _release_step in SEQUENCE_STEP_NAMES:
        shirt_release_from_step = SEQUENCE_STEP_NAMES.index(_release_step)
    else:
        print(
            f"[isaac_ffw_teleop] WARN: unknown shirt_release_step '{_release_step}' "
            f"(expected one of {SEQUENCE_STEP_NAMES})"
        )
if args_cli.shirt_link_gripper:
    args_cli.shirt_grasp = True
    args_cli.shirt_grasp_mode = "demo"
    _gripper_base = "/World/Robot/ffw_sg2_follower/right_gripper"
    _finger_prims = {
        "attach": f"{_gripper_base}/gripper_r_rh_p12_rn_r1/ATTACH",
        "r2": f"{_gripper_base}/gripper_r_rh_p12_rn_r2",
        "l2": f"{_gripper_base}/gripper_r_rh_p12_rn_l2",
    }
    finger = args_cli.shirt_link_finger.strip().lower()
    args_cli.gripper_r_prim = _finger_prims.get(finger, _finger_prims["attach"])

if not args_cli.grasp_mode:
    if args_cli.grasp_test_cube:
        args_cli.grasp_mode = "constraint"
    elif args_cli.soft_shirts:
        args_cli.grasp_mode = "xform"
    else:
        args_cli.grasp_mode = "xform"
elif args_cli.soft_shirts and args_cli.grasp_mode == "constraint":
    print(
        "[isaac_ffw_teleop] WARN: --grasp-mode constraint ignored with soft shirts; using xform"
    )
    args_cli.grasp_mode = "xform"

simulation_app = SimulationApp({"headless": args_cli.headless})

_rclpy_err: Optional[Exception] = None
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except Exception as e:  # pragma: no cover - runtime env dependent
    rclpy = None  # type: ignore
    Node = object  # type: ignore
    JointState = object  # type: ignore
    _rclpy_err = e

import isaacsim.core.utils.prims as prim_utils
from isaacsim.core.api.world import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics


class _JointSub(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("isaac_ffw_joint_mirror")
        self._lock = threading.Lock()
        self._names: List[str] = []
        self._pos: List[float] = []
        self.create_subscription(JointState, topic, self._cb, 10)

    def _cb(self, msg: JointState) -> None:
        with self._lock:
            self._names = list(msg.name)
            self._pos = [float(x) for x in msg.position]

    def get_targets(self) -> tuple[list[str], list[float]]:
        with self._lock:
            return list(self._names), list(self._pos)


class _UdpJointSub:
    """Receives JSON packets from joint_targets_udp_bridge.py.

    Packet format: {"name": [...], "position": [...]}
    """

    def __init__(self, host: str, port: int) -> None:
        self._lock = threading.Lock()
        self._names: List[str] = []
        self._pos: List[float] = []
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, int(port)))
        self._sock.setblocking(False)

    def poll(self) -> None:
        while True:
            try:
                data, _ = self._sock.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError:
                break
            try:
                msg = json.loads(data.decode("utf-8"))
                names = list(msg.get("name", []))
                pos = [float(x) for x in msg.get("position", [])]
                with self._lock:
                    self._names = names
                    self._pos = pos
            except Exception:
                # Keep running even if one datagram is malformed.
                continue

    def get_targets(self) -> tuple[list[str], list[float]]:
        self.poll()
        with self._lock:
            return list(self._names), list(self._pos)

    def close(self) -> None:
        self._sock.close()


BASE_POSE_KEYS = ("isaac_base_x", "isaac_base_y", "isaac_base_z", "isaac_base_yaw")
SHIRT_POSE_KEYS = ("shirt_tx", "shirt_ty", "shirt_tz", "shirt_rx", "shirt_ry", "shirt_rz")
CRATE_POSE_KEYS = ("crate_tx", "crate_ty", "crate_tz", "crate_rx", "crate_ry", "crate_rz")
GRASP_ATTACH_OFFSET_KEYS = (
    "grasp_attach_tx",
    "grasp_attach_ty",
    "grasp_attach_tz",
    "grasp_attach_rx",
    "grasp_attach_ry",
    "grasp_attach_rz",
)
MOTION_STEP_KEY = "motion_step"
TELEOP_POSE_KEYS = BASE_POSE_KEYS + SHIRT_POSE_KEYS + CRATE_POSE_KEYS
NON_JOINT_TARGET_KEYS = TELEOP_POSE_KEYS + GRASP_ATTACH_OFFSET_KEYS + (MOTION_STEP_KEY,)


def _load_grasp_attach_offset_from_yaml(yaml_path: str) -> tuple[float, float, float, float, float, float]:
    if not yaml_path.strip():
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    try:
        import yaml

        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        params = data.get("/**", {}).get("arm_l_joint_trajectory_executor", {}).get(
            "ros__parameters", {}
        )
        raw = params.get("physics_grasp_attach_offset")
        if raw is None:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if isinstance(raw, (list, tuple)) and len(raw) >= 6:
            return tuple(float(raw[i]) for i in range(6))
    except Exception as exc:
        print(f"[isaac_ffw_teleop] Could not load physics_grasp_attach_offset: {exc!r}")
    return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def _extract_base_pose(msg_names: List[str], msg_pos: List[float]) -> Optional[tuple[float, float, float, float]]:
    mapping = {n: float(p) for n, p in zip(msg_names, msg_pos)}
    if not all(k in mapping for k in BASE_POSE_KEYS):
        return None
    return (
        mapping["isaac_base_x"],
        mapping["isaac_base_y"],
        mapping["isaac_base_z"],
        mapping["isaac_base_yaw"],
    )


def _extract_shirt_pose(msg_names: List[str], msg_pos: List[float]) -> Optional[tuple[float, float, float, float, float, float]]:
    mapping = {n: float(p) for n, p in zip(msg_names, msg_pos)}
    if not all(k in mapping for k in ("shirt_tx", "shirt_ty", "shirt_tz")):
        return None
    tx, ty, tz = mapping["shirt_tx"], mapping["shirt_ty"], mapping["shirt_tz"]
    rx = float(mapping.get("shirt_rx", 0.0))
    ry = float(mapping.get("shirt_ry", 0.0))
    rz = float(mapping.get("shirt_rz", 0.0))
    return tx, ty, tz, rx, ry, rz


def _extract_crate_pose(msg_names: List[str], msg_pos: List[float]) -> Optional[tuple[float, float, float, float, float, float]]:
    mapping = {n: float(p) for n, p in zip(msg_names, msg_pos)}
    if not all(k in mapping for k in ("crate_tx", "crate_ty", "crate_tz")):
        return None
    tx, ty, tz = mapping["crate_tx"], mapping["crate_ty"], mapping["crate_tz"]
    rx = float(mapping.get("crate_rx", 0.0))
    ry = float(mapping.get("crate_ry", 0.0))
    rz = float(mapping.get("crate_rz", 90.0))
    return tx, ty, tz, rx, ry, rz


def _extract_grasp_attach_offset(
    msg_names: List[str], msg_pos: List[float]
) -> tuple[float, float, float, float, float, float]:
    mapping = {n: float(p) for n, p in zip(msg_names, msg_pos)}
    return tuple(float(mapping.get(k, 0.0)) for k in GRASP_ATTACH_OFFSET_KEYS)


def _extract_motion_step(msg_names: List[str], msg_pos: List[float]) -> Optional[int]:
    mapping = {n: float(p) for n, p in zip(msg_names, msg_pos)}
    if MOTION_STEP_KEY not in mapping:
        return None
    return int(round(mapping[MOTION_STEP_KEY]))


def _joint_targets_without_base(
    msg_names: List[str], msg_pos: List[float]
) -> tuple[list[str], list[float]]:
    names: List[str] = []
    pos: List[float] = []
    for n, p in zip(msg_names, msg_pos):
        if n in NON_JOINT_TARGET_KEYS:
            continue
        names.append(n)
        pos.append(float(p))
    return names, pos


def _apply_base_pose(prim_path: str, x: float, y: float, z: float, yaw_rad: float) -> bool:
    import math
    import omni.usd  # Isaac runtime

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return False
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False

    xform_api = UsdGeom.XformCommonAPI(prim)
    if xform_api:
        xform_api.SetTranslate((float(x), float(y), float(z)))
        xform_api.SetRotate((0.0, 0.0, math.degrees(float(yaw_rad))))
        return True

    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return False
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(float(x), float(y), float(z)))
    rot = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), math.degrees(float(yaw_rad)))
    xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(rot.GetQuat())
    return True


def _map_to_articulation(
    sim_names: List[str], sim_q: np.ndarray, msg_names: List[str], msg_pos: List[float]
) -> np.ndarray:
    idx: Dict[str, int] = {n: i for i, n in enumerate(sim_names)}
    out = np.array(sim_q, dtype=np.float64).copy()
    for n, p in zip(msg_names, msg_pos):
        if n in idx:
            out[idx[n]] = float(p)
    return out


def _find_articulation_root(spawn_prim: str) -> Optional[str]:
    """Find first articulation root prim under spawn_prim in current stage."""
    import omni.usd  # Isaac runtime

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return None

    # Check the spawn prim itself first.
    p = stage.GetPrimAtPath(spawn_prim)
    if p and p.IsValid() and (
        UsdPhysics.ArticulationRootAPI(p) or PhysxSchema.PhysxArticulationAPI(p)
    ):
        return spawn_prim

    prefix = spawn_prim.rstrip("/") + "/"
    for prim in stage.Traverse():
        if not prim.IsValid():
            continue
        path = prim.GetPath().pathString
        if not path.startswith(prefix):
            continue
        if UsdPhysics.ArticulationRootAPI(prim) or PhysxSchema.PhysxArticulationAPI(prim):
            return path
    return None


def _list_articulation_candidates(limit: int = 20) -> List[str]:
    import omni.usd  # Isaac runtime

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return []

    out: List[str] = []
    for prim in stage.Traverse():
        if not prim.IsValid():
            continue
        if UsdPhysics.ArticulationRootAPI(prim) or PhysxSchema.PhysxArticulationAPI(prim):
            out.append(prim.GetPath().pathString)
            if len(out) >= limit:
                break
    return out


def _lookup_joint(names: List[str], pos: List[float], joint_name: str) -> Optional[float]:
    for n, p in zip(names, pos):
        if n == joint_name:
            return float(p)
    return None


def _apply_start_pose_from_yaml(
    world,
    robot,
    sim_joint_names: List[str],
    base_move_prim: str,
    yaml_path: str,
    step: str,
) -> None:
    if not yaml_path.strip():
        return
    import sys
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from yaml_pose_loader import load_step_pose

    pose = load_step_pose(yaml_path, step)
    base = (
        float(pose["isaac_base_x"]),
        float(pose["isaac_base_y"]),
        float(pose["isaac_base_z"]),
        float(pose["isaac_base_yaw"]),
    )
    _apply_base_pose(base_move_prim, *base)
    print(
        f"[isaac_ffw_teleop] Start pose from {yaml_path} step={step} "
        f"base=({base[0]:.3f}, {base[1]:.3f}, {base[2]:.3f}, yaw={base[3]:.3f})"
    )

    names = list(FULL_JOINT_ORDER) if "FULL_JOINT_ORDER" in dir() else [
        "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
        "arm_l_joint5", "arm_l_joint6", "arm_l_joint7", "gripper_l_joint1",
        "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
        "arm_r_joint5", "arm_r_joint6", "arm_r_joint7", "gripper_r_joint1",
        "lift_joint", "head_joint1", "head_joint2",
    ]
    joint_names = [n for n in names if n in pose]
    joint_pos = [float(pose[n]) for n in joint_names]
    if joint_names and sim_joint_names:
        q_tgt = _map_to_articulation(sim_joint_names, np.zeros(len(sim_joint_names)), joint_names, joint_pos)
        q_tgt_t = torch.as_tensor(q_tgt, dtype=torch.float32).reshape(1, -1)
        if hasattr(robot, "set_joint_position_targets"):
            robot.set_joint_position_targets(q_tgt_t)
        if hasattr(robot, "set_joint_positions"):
            robot.set_joint_positions(q_tgt_t)
        for _ in range(8):
            world.step(render=False)


def main() -> None:
    import omni.usd  # Isaac runtime

    world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, backend="torch", device="cpu")
    set_camera_view([2.2, 2.2, 1.5], [0.0, 0.0, 0.8])
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    existing_spawn = stage.GetPrimAtPath(args_cli.spawn_prim) if stage is not None else None
    if existing_spawn and existing_spawn.IsValid():
        # Reuse existing prim path (e.g. /World). Only add reference if empty.
        if not existing_spawn.HasAuthoredReferences():
            existing_spawn.GetReferences().AddReference(args_cli.usd_path)
    else:
        prim_utils.create_prim(args_cli.spawn_prim, usd_path=args_cli.usd_path, translation=(0.0, 0.0, 0.0))

    grasp_test_shirt_roots = None
    if args_cli.grasp_test_cube:
        from spawn_grasp_test_cube import (
            GRASP_TEST_CUBE_GRIP,
            GRASP_TEST_CUBE_MESH,
            GRASP_TEST_CUBE_ROOT,
            spawn_grasp_test_cube,
        )

        spawn_grasp_test_cube(stage)
        args_cli.shirt_prim = GRASP_TEST_CUBE_MESH
        args_cli.shirt_grip_point = GRASP_TEST_CUBE_GRIP
        grasp_test_shirt_roots = (GRASP_TEST_CUBE_ROOT,)
        print(
            "[isaac_ffw_teleop] Grasp test cube mode:",
            GRASP_TEST_CUBE_MESH,
            "grip=",
            GRASP_TEST_CUBE_GRIP,
            "(YAML shirt_tx/ty/tz ignored — cube uses physics + grip constraint)",
        )

    _constraint_grasp = (
        args_cli.grasp_mode == "constraint"
        and not args_cli.soft_shirts
        and not args_cli.physics_grasp
    )
    _physics_grasp = bool(args_cli.physics_grasp and not args_cli.soft_shirts)
    try:
        from configure_scene_physics import SHIRT_ROOTS, configure_scene_physics

        shirt_roots = list(grasp_test_shirt_roots or SHIRT_ROOTS)
        extra_root = args_cli.shirt_root.strip()
        if extra_root and extra_root not in shirt_roots:
            shirt_roots.append(extra_root)

        configure_scene_physics(
            stage,
            shirt_roots=shirt_roots,
            soft_shirts=bool(args_cli.soft_shirts and not args_cli.grasp_test_cube),
            constraint_grasp=_constraint_grasp,
            physics_grasp=_physics_grasp,
            world=world,
            robot_filter_root=args_cli.articulation_prim.strip() or args_cli.spawn_prim.strip(),
        )
        if _physics_grasp:
            try:
                from shirt_grasp_controller import _boost_gripper_drive, _resolve_gripper_joint

                gj = _resolve_gripper_joint(stage)
                if gj and _boost_gripper_drive(
                    stage, gj, stiffness=900.0, max_force=600.0, damping=30.0
                ):
                    print(
                        f"[isaac_ffw_teleop] Physics grasp: boosted gripper drive on {gj} "
                        "(stiffness=900, maxForce=600)"
                    )
            except Exception as exc:
                print(f"[isaac_ffw_teleop] Physics grasp gripper tune skipped: {exc!r}")
    except Exception as exc:
        print("[isaac_ffw_teleop] Scene physics setup skipped:", repr(exc))

    print(
        "[isaac_ffw_teleop] Shirt pose teleop on",
        args_cli.shirt_prim,
        "via shirt_tx/ty/tz + local rotateXYZ deg on mesh",
    )
    if _physics_grasp:
        print(
            "[isaac_ffw_teleop] Physics grasp mode: friction pick + kinematic sustain on turn/lift"
        )

    art_prim = args_cli.articulation_prim.strip() or _find_articulation_root(args_cli.spawn_prim)
    if not art_prim:
        cands = _list_articulation_candidates()
        cands_msg = ", ".join(cands) if cands else "<none found>"
        raise RuntimeError(
            f"Could not find articulation root under spawn prim '{args_cli.spawn_prim}'. "
            f"Pass --articulation_prim explicitly to a valid articulation root. "
            f"Detected articulation candidates in stage: {cands_msg}"
        )

    robot = Articulation(art_prim, name="ffw")
    world.scene.add(robot)
    world.reset()
    robot.initialize()
    # Auto-play so users don't need to press the Isaac UI Play button.
    world.play()

    use_ros2 = args_cli.input_mode == "ros2" or (
        args_cli.input_mode == "auto" and rclpy is not None
    )
    if args_cli.input_mode == "ros2" and rclpy is None:
        raise RuntimeError(f"--input_mode ros2 requested but rclpy failed import: {_rclpy_err}")

    executor = None
    ros_sub = None
    udp_sub = None
    if use_ros2:
        rclpy.init()
        ros_sub = _JointSub(args_cli.joint_state_topic)
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(ros_sub)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        print("[isaac_ffw_teleop] Input mode: ROS2 topic", args_cli.joint_state_topic)
    else:
        udp_sub = _UdpJointSub(args_cli.udp_host, args_cli.udp_port)
        print(
            "[isaac_ffw_teleop] Input mode: UDP bridge",
            f"{args_cli.udp_host}:{args_cli.udp_port}",
        )
        if _rclpy_err is not None:
            print("[isaac_ffw_teleop] rclpy unavailable in Isaac env:", repr(_rclpy_err))
        print(
            "[isaac_ffw_teleop] Run bridge in ROS terminal:",
            "python /home/disniekie/Robotis/robotis_vr_isaac/scripts/joint_targets_udp_bridge.py",
        )

    sim_joint_names: Optional[List[str]] = None
    if hasattr(robot, "joint_names"):
        sim_joint_names = list(robot.joint_names)
    elif hasattr(robot, "dof_names"):
        sim_joint_names = list(robot.dof_names)

    print("[isaac_ffw_teleop] Spawn prim:", args_cli.spawn_prim)
    print("[isaac_ffw_teleop] Articulation prim:", art_prim)
    print("[isaac_ffw_teleop] Base move prim:", args_cli.base_move_prim)
    print("[isaac_ffw_teleop] Joint names from view:", sim_joint_names)
    if not sim_joint_names:
        raise RuntimeError(
            "Articulation initialized but no joint names were found. "
            "Verify --articulation_prim points to the articulation root (not just parent Xform)."
        )

    _apply_start_pose_from_yaml(
        world,
        robot,
        sim_joint_names,
        args_cli.base_move_prim,
        args_cli.start_pose_yaml.strip(),
        args_cli.start_pose_step.strip() or "Start",
    )

    last_apply_log_s = 0.0
    smooth_alpha = float(np.clip(args_cli.command_smoothing, 0.0, 1.0))
    smoothed_q: Optional[np.ndarray] = None
    smoothed_base: Optional[tuple[float, float, float, float]] = None
    if smooth_alpha > 0.0:
        print(f"[isaac_ffw_teleop] Command smoothing alpha={smooth_alpha:.3f}")
    lift_idx = sim_joint_names.index("lift_joint") if sim_joint_names and "lift_joint" in sim_joint_names else -1
    shirt_grasp = None
    physics_grasp_logger = None
    if args_cli.shirt_grasp and not args_cli.physics_grasp:
        try:
            from shirt_grasp_controller import (
                resolve_gripper_attach_prim,
                resolve_shirt_grip_point,
            )

            gripper_prim = args_cli.gripper_r_prim.strip()
            if args_cli.shirt_link_gripper and args_cli.shirt_link_finger.strip().lower() == "attach":
                gripper_prim = resolve_gripper_attach_prim(stage, gripper_prim)
                if not stage.GetPrimAtPath(gripper_prim).IsValid():
                    print(
                        f"[isaac_ffw_teleop] WARN: ATTACH prim not found at {gripper_prim} "
                        f"(expected under gripper_r_rh_p12_rn_r1/ATTACH or right_gripper/ATTACH)"
                    )
                else:
                    print(f"[isaac_ffw_teleop] Using gripper ATTACH prim: {gripper_prim}")

            shirt_grip_point = resolve_shirt_grip_point(
                stage,
                args_cli.shirt_prim.strip(),
                args_cli.shirt_grip_point.strip(),
            )
            if stage.GetPrimAtPath(shirt_grip_point).IsValid():
                print(f"[isaac_ffw_teleop] Using shirt Grippoint: {shirt_grip_point}")
            elif args_cli.shirt_link_gripper:
                print(f"[isaac_ffw_teleop] WARN: shirt Grippoint not found at {shirt_grip_point}")

            robot_filter = args_cli.articulation_prim.strip() or args_cli.spawn_prim.strip()
            if args_cli.grasp_mode == "constraint":
                from grasp_constraint_controller import build_grasp_constraint_controller
                from shirt_grasp_controller import AttachTrigger

                shirt_grasp = build_grasp_constraint_controller(
                    stage,
                    attach_prim=gripper_prim,
                    object_mesh_paths=[args_cli.shirt_prim.strip()],
                    object_grip_point=shirt_grip_point,
                    link_from_step=shirt_link_from_step,
                    release_from_step=shirt_release_from_step,
                    robot_filter_root=robot_filter,
                    snap_to_gripper=bool(args_cli.shirt_link_snap),
                    attach_trigger=(
                        AttachTrigger.MOTION_STEP.value
                        if args_cli.shirt_link_gripper
                        else AttachTrigger.GRIP_CLOSE.value
                    ),
                )
                grasp_label = "PhysX FixedJoint constraint"
            else:
                from shirt_grasp_controller import AttachTrigger, build_shirt_grasp_controller

                shirt_grasp = build_shirt_grasp_controller(
                    stage,
                    gripper_prim=gripper_prim,
                    shirt_mesh_paths=[args_cli.shirt_prim.strip()],
                    mode=args_cli.shirt_grasp_mode,
                    follow_mesh=bool(args_cli.shirt_link_gripper),
                    attach_trigger=(
                        AttachTrigger.MOTION_STEP.value
                        if args_cli.shirt_link_gripper
                        else AttachTrigger.GRIP_CLOSE.value
                    ),
                    link_from_step=shirt_link_from_step,
                    release_from_step=shirt_release_from_step,
                    snap_to_gripper=bool(args_cli.shirt_link_snap and args_cli.shirt_link_gripper),
                    shirt_grip_point=shirt_grip_point,
                    link_rotate=bool(args_cli.shirt_link_rotate),
                )
                grasp_label = f"xform weld mode={args_cli.shirt_grasp_mode}"

            link_msg = (
                f" (mesh link on {args_cli.shirt_link_finger} at step {args_cli.shirt_link_step}"
                f"{f', release at {args_cli.shirt_release_step}' if shirt_release_from_step >= 0 else ''})"
                if args_cli.shirt_link_gripper
                else ""
            )
            print(
                "[isaac_ffw_teleop] Shirt grasp enabled on",
                gripper_prim,
                f"grasp_mode={args_cli.grasp_mode} {grasp_label}{link_msg}",
            )
        except Exception as exc:
            print("[isaac_ffw_teleop] Shirt grasp disabled:", repr(exc))
    elif args_cli.physics_grasp:
        try:
            from grasp_constraint_controller import (
                SUSTAIN_KINEMATIC_FOLLOW,
                build_grasp_constraint_controller,
            )
            from grasp_physics_logger import GraspPhysicsLogger
            from shirt_grasp_controller import (
                AttachTrigger,
                resolve_gripper_attach_prim,
                resolve_shirt_grip_point,
            )

            log_dir = args_cli.physics_grasp_log_dir.strip()
            if not log_dir:
                project_root = os.environ.get("ROBOTIS_VR_ISAAC_ROOT") or os.path.dirname(
                    os.path.dirname(os.path.abspath(__file__))
                )
                log_dir = os.path.join(project_root, "logs", "physics_grasp")
            physics_grasp_logger = GraspPhysicsLogger(log_dir)
            physics_grasp_logger.log_config(
                usd_path=args_cli.usd_path,
                shirt_prim=args_cli.shirt_prim.strip(),
                shirt_root=args_cli.shirt_root.strip() or None,
                articulation_prim=args_cli.articulation_prim.strip() or None,
            )

            gripper_prim = resolve_gripper_attach_prim(stage, "")
            shirt_grip_point = resolve_shirt_grip_point(
                stage,
                args_cli.shirt_prim.strip(),
                args_cli.shirt_grip_point.strip(),
            )
            drop_idx = (
                shirt_release_from_step
                if shirt_release_from_step >= 0
                else SEQUENCE_STEP_NAMES.index("Drop")
            )
            robot_filter = ""
            shirt_grasp = build_grasp_constraint_controller(
                stage,
                attach_prim=gripper_prim,
                object_mesh_paths=[args_cli.shirt_prim.strip()],
                object_grip_point=shirt_grip_point,
                link_from_step=shirt_link_from_step,
                release_from_step=drop_idx,
                robot_filter_root=robot_filter,
                snap_to_gripper=False,
                latch_at_current_pose=True,
                attach_trigger=AttachTrigger.GRIP_CLOSE.value,
                settle_steps=25,
                physics_logger=physics_grasp_logger,
                sustain_mode=SUSTAIN_KINEMATIC_FOLLOW,
            )
            shirt_grasp.set_motion_step_names(SEQUENCE_STEP_NAMES)
            yaml_off = _load_grasp_attach_offset_from_yaml(args_cli.start_pose_yaml.strip())
            shirt_grasp.set_attach_offset(*yaml_off)
            if any(abs(v) > 1e-9 for v in yaml_off):
                print(
                    "[isaac_ffw_teleop] Grasp attach offset from YAML:",
                    ", ".join(f"{k}={yaml_off[i]:.4f}" for i, k in enumerate(GRASP_ATTACH_OFFSET_KEYS)),
                )
            print(
                "[isaac_ffw_teleop] Physics grasp: friction pick, then kinematic follow "
                "at gripper ATTACH (release at Drop)"
            )
            print(
                f"[isaac_ffw_teleop] Physics grasp log: {physics_grasp_logger.session_path}"
            )
        except Exception as exc:
            print("[isaac_ffw_teleop] Physics grasp latch disabled:", repr(exc))

    sim_step = 0
    grip_monitor_from_step = SEQUENCE_STEP_NAMES.index("GripShirt")

    while simulation_app.is_running():
        if world.is_stopped():
            world.play()
        if not world.is_playing():
            world.step(render=not args_cli.headless)
            continue

        q = robot.get_joint_positions()
        if isinstance(q, torch.Tensor):
            q_np = q.detach().cpu().numpy().astype(np.float64).flatten()
        elif isinstance(q, np.ndarray):
            q_np = q.astype(np.float64).flatten()
        else:
            q_np = np.array(q, dtype=np.float64).flatten()

        if ros_sub is not None:
            names, pos = ros_sub.get_targets()
        elif udp_sub is not None:
            names, pos = udp_sub.get_targets()
        else:
            names, pos = [], []
        joint_names, joint_pos = _joint_targets_without_base(names, pos)
        base_pose = _extract_base_pose(names, pos)
        if base_pose is not None:
            if smooth_alpha > 0.0:
                if smoothed_base is None:
                    smoothed_base = base_pose
                else:
                    sb = list(smoothed_base)
                    tgt = list(base_pose)
                    sb[0] = (1.0 - smooth_alpha) * sb[0] + smooth_alpha * tgt[0]
                    sb[1] = (1.0 - smooth_alpha) * sb[1] + smooth_alpha * tgt[1]
                    sb[2] = (1.0 - smooth_alpha) * sb[2] + smooth_alpha * tgt[2]
                    dyaw = (tgt[3] - sb[3] + np.pi) % (2.0 * np.pi) - np.pi
                    sb[3] = sb[3] + smooth_alpha * dyaw
                    smoothed_base = (sb[0], sb[1], sb[2], sb[3])
                base_pose = smoothed_base
            _apply_base_pose(args_cli.base_move_prim, *base_pose)

        shirt_pose = _extract_shirt_pose(names, pos)
        crate_pose = _extract_crate_pose(names, pos)
        motion_step = _extract_motion_step(names, pos)

        q_tgt = None
        if joint_names and sim_joint_names is not None:
            q_tgt = _map_to_articulation(sim_joint_names, q_np, joint_names, joint_pos)
            if smooth_alpha > 0.0:
                if smoothed_q is None:
                    smoothed_q = q_tgt.copy()
                else:
                    smoothed_q = (1.0 - smooth_alpha) * smoothed_q + smooth_alpha * q_tgt
                q_tgt = smoothed_q
            q_tgt_t = torch.as_tensor(q_tgt, dtype=torch.float32).reshape(1, -1)
            # Use articulation drive targets only to avoid jitter from fighting
            # between target controllers and hard position writes.
            if hasattr(robot, "set_joint_position_targets"):
                robot.set_joint_position_targets(q_tgt_t)
        elif joint_names and len(joint_pos) == len(q_np):
            q_tgt = np.array(joint_pos, dtype=np.float64)
            q_tgt_t = torch.as_tensor(joint_pos, dtype=torch.float32).reshape(1, -1)
            if hasattr(robot, "set_joint_position_targets"):
                robot.set_joint_position_targets(q_tgt_t)

        now = time.monotonic()
        if (joint_names or base_pose is not None) and (now - last_apply_log_s) > 1.0:
            q_after = robot.get_joint_positions()
            if isinstance(q_after, torch.Tensor):
                q_after_np = q_after.detach().cpu().numpy().astype(np.float64).flatten()
            elif isinstance(q_after, np.ndarray):
                q_after_np = q_after.astype(np.float64).flatten()
            else:
                q_after_np = np.array(q_after, dtype=np.float64).flatten()

            lift_cmd = None
            if q_tgt is not None and lift_idx >= 0 and lift_idx < len(q_tgt):
                lift_cmd = float(q_tgt[lift_idx])
            lift_meas = None
            if lift_idx >= 0 and lift_idx < len(q_after_np):
                lift_meas = float(q_after_np[lift_idx])
            # Visible heartbeat that commands are being applied.
            print(
                f"[isaac_ffw_teleop] applying {len(joint_names)} joints "
                f"(lift_cmd={lift_cmd}, lift_meas={lift_meas}, base={base_pose})"
            )
            last_apply_log_s = now

        if shirt_grasp is not None:
            grip_cmd = _lookup_joint(names, pos, "gripper_r_joint1")
            grip_meas = None
            if sim_joint_names and "gripper_r_joint1" in sim_joint_names:
                gi = sim_joint_names.index("gripper_r_joint1")
                if gi < len(q_np):
                    grip_meas = float(q_np[gi])
            if args_cli.physics_grasp and shirt_grasp is not None:
                shirt_grasp.update_attach_offset_from_names(names, pos)
            shirt_grasp.update(grip_cmd, grip_meas, motion_step=motion_step)
            if (
                physics_grasp_logger is not None
                and motion_step is not None
                and int(motion_step) >= grip_monitor_from_step
            ):
                physics_grasp_logger.set_monitor_active(True)

        if crate_pose is not None and stage is not None:
            try:
                from crate_pose_controller import apply_crate_pose

                if not apply_crate_pose(stage, args_cli.crate_prim.strip(), *crate_pose):
                    if sim_step == 0:
                        print(
                            f"[isaac_ffw_teleop] Crate pose not applied: prim missing "
                            f"{args_cli.crate_prim.strip()}"
                        )
            except Exception as exc:
                print(f"[isaac_ffw_teleop] Crate pose apply failed: {exc!r}")

        world.step(render=not args_cli.headless)

        sim_step += 1
        if physics_grasp_logger is not None:
            physics_grasp_logger.set_sim_step(sim_step)

        shirt_pose_blocked = bool(args_cli.grasp_test_cube) or bool(args_cli.physics_grasp) or (
            shirt_grasp is not None
            and shirt_grasp.is_attached
        ) or (
            args_cli.shirt_link_gripper
            and motion_step is not None
            and motion_step >= shirt_link_from_step
            and (
                shirt_release_from_step < 0
                or motion_step < shirt_release_from_step
            )
        )
        if shirt_grasp is not None:
            if args_cli.grasp_mode == "constraint" or args_cli.physics_grasp:
                try:
                    shirt_grasp.follow_after_physics()
                    shirt_grasp.log_post_physics()
                except Exception as exc:
                    print(f"[isaac_ffw_teleop] Grasp constraint follow failed: {exc!r}")
            elif shirt_grasp.is_attached:
                try:
                    shirt_grasp.follow_after_physics()
                except Exception as exc:
                    print(f"[isaac_ffw_teleop] Shirt follow failed: {exc!r}")
        elif shirt_pose is not None and stage is not None and not shirt_pose_blocked:
            try:
                from shirt_pose_controller import apply_shirt_pose

                apply_shirt_pose(stage, args_cli.shirt_prim.strip(), *shirt_pose)
            except Exception as exc:
                print(f"[isaac_ffw_teleop] Shirt pose apply failed: {exc!r}")

    if executor is not None and ros_sub is not None:
        executor.shutdown()
        ros_sub.destroy_node()
        rclpy.shutdown()
    if udp_sub is not None:
        udp_sub.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
