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
args_cli = parser.parse_args()

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
from pxr import PhysxSchema, UsdPhysics


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
    print("[isaac_ffw_teleop] Joint names from view:", sim_joint_names)
    if not sim_joint_names:
        raise RuntimeError(
            "Articulation initialized but no joint names were found. "
            "Verify --articulation_prim points to the articulation root (not just parent Xform)."
        )

    last_apply_log_s = 0.0
    lift_idx = sim_joint_names.index("lift_joint") if sim_joint_names and "lift_joint" in sim_joint_names else -1
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
        q_tgt = None
        if names and sim_joint_names is not None:
            q_tgt = _map_to_articulation(sim_joint_names, q_np, names, pos)
            q_tgt_t = torch.as_tensor(q_tgt, dtype=torch.float32).reshape(1, -1)
            # Use articulation drive targets only to avoid jitter from fighting
            # between target controllers and hard position writes.
            if hasattr(robot, "set_joint_position_targets"):
                robot.set_joint_position_targets(q_tgt_t)
        elif names and len(pos) == len(q_np):
            q_tgt = np.array(pos, dtype=np.float64)
            q_tgt_t = torch.as_tensor(pos, dtype=torch.float32).reshape(1, -1)
            if hasattr(robot, "set_joint_position_targets"):
                robot.set_joint_position_targets(q_tgt_t)

        now = time.monotonic()
        if names and (now - last_apply_log_s) > 1.0:
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
                f"[isaac_ffw_teleop] applying {len(names)} joints "
                f"(lift_cmd={lift_cmd}, lift_meas={lift_meas})"
            )
            last_apply_log_s = now

        world.step(render=not args_cli.headless)

    if executor is not None and ros_sub is not None:
        executor.shutdown()
        ros_sub.destroy_node()
        rclpy.shutdown()
    if udp_sub is not None:
        udp_sub.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
