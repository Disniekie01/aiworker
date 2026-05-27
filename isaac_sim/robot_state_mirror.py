"""
Mirror real robot /joint_states into Isaac Sim.
Robot joints are driven directly by ROS topic values (set_joint_positions).
Full physics simulation runs for all other scene objects (rigid bodies, collisions, gravity).
Uses Isaac Sim's built-in ROS2 bridge extension — no system rclpy required.
"""
from __future__ import annotations

import argparse
import threading
import traceback
from typing import Dict, List, Optional

import numpy as np

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Mirror robot joint states into Isaac Sim")
parser.add_argument("--usd_path", type=str, required=True, help="USD scene file path")
parser.add_argument("--topic", type=str, default="/joint_states", help="sensor_msgs/JointState topic to mirror")
parser.add_argument("--headless", action="store_true")
args_cli = parser.parse_args()

simulation_app = SimulationApp({"headless": args_cli.headless})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import omni.usd
import omni.timeline
import omni.physx
from isaacsim.core.api.world import World
from isaacsim.core.prims import Articulation
from pxr import PhysxSchema, UsdPhysics  # PhysxSchema used in _find_articulation_root


class JointStateSub(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("isaac_robot_mirror")
        self._lock = threading.Lock()
        self._names: List[str] = []
        self._pos: List[float] = []
        self.create_subscription(JointState, topic, self._cb, 10)

    def _cb(self, msg: JointState) -> None:
        with self._lock:
            self._names = list(msg.name)
            self._pos = [float(x) for x in msg.position]

    def get(self) -> tuple[list[str], list[float]]:
        with self._lock:
            return list(self._names), list(self._pos)


def _map_joints(
    sim_names: List[str], names: List[str], pos: List[float]
) -> np.ndarray:
    idx: Dict[str, int] = {n: p for n, p in zip(names, pos)}
    return np.array([idx.get(n, 0.0) for n in sim_names], dtype=np.float32)


def _find_articulation_root() -> Optional[str]:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return None
    for prim in stage.Traverse():
        if not prim.IsValid():
            continue
        if UsdPhysics.ArticulationRootAPI(prim) or PhysxSchema.PhysxArticulationAPI(prim):
            return prim.GetPath().pathString
    return None


MIMIC_GEARING_OVERRIDE: Dict[str, float] = {
    "gripper_l_joint2": -1.0,
    "gripper_l_joint3": -1.0,
    "gripper_l_joint4": -1.0,
    "gripper_r_joint2": -1.0,
    "gripper_r_joint3": -1.0,
    "gripper_r_joint4": -1.0,
}


def _fix_mimic_gearing(stage) -> None:
    """Override incorrect gearing values on mimic joints before simulation starts."""
    for prim in stage.Traverse():
        if not prim.IsValid() or prim.GetName() not in MIMIC_GEARING_OVERRIDE:
            continue
        mimic = PhysxSchema.PhysxMimicJointAPI.Get(prim, "rotX")
        if mimic:
            correct = MIMIC_GEARING_OVERRIDE[prim.GetName()]
            mimic.GetGearingAttr().Set(correct)
            print(f"[robot_mirror] Fixed gearing {prim.GetName()} → {correct}")


def _find_joints() -> Dict[str, str]:
    """Return {joint_name: prim_path} for every physics joint in the stage."""
    stage = omni.usd.get_context().get_stage()
    joints: Dict[str, str] = {}
    if stage is None:
        return joints
    for prim in stage.Traverse():
        if not prim.IsValid():
            continue
        if (UsdPhysics.RevoluteJoint(prim)
                or UsdPhysics.PrismaticJoint(prim)
                or UsdPhysics.SphericalJoint(prim)
                or UsdPhysics.FixedJoint(prim)):
            joints[prim.GetName()] = prim.GetPath().pathString
    return joints


def main() -> None:
    # Open the USD file directly as the stage.
    omni.usd.get_context().open_stage(args_cli.usd_path)
    simulation_app.update()

    _fix_mimic_gearing(omni.usd.get_context().get_stage())

    # [STEP 2] Play via timeline — matches GUI Play button behavior, no Python overhead.
    omni.timeline.get_timeline_interface().play()

    joints = _find_joints()
    print(f"[robot_mirror] Found {len(joints)} joints:")
    for name, path in joints.items():
        print(f"  {name:30s}  {path}")

    # [STEP 2+3] World-based init (required by Articulation, but adds overhead).
    # world = World(physics_dt=1.0 / 60.0, backend="numpy")
    # art_prim = _find_articulation_root()
    # if not art_prim:
    #     raise RuntimeError("No articulation root found in stage.")
    # robot = Articulation(art_prim, name="robot_mirror")
    # world.scene.add(robot)
    # world.reset()
    # robot.initialize()

    # sim_joint_names: List[str] = list(
    #     getattr(robot, "joint_names", None) or getattr(robot, "dof_names", [])
    # )
    # if not sim_joint_names:
    #     raise RuntimeError("No joint names found.")

    # print(f"[robot_mirror] Articulation : {art_prim}")
    # print(f"[robot_mirror] Joints ({len(sim_joint_names)}): {sim_joint_names}")
    # print(f"[robot_mirror] Subscribing  : {args_cli.topic}")

    # [STEP 4] ROS2 joint mirroring via low-level PhysX callback (no World needed).
    rclpy.init()
    sub = JointStateSub(args_cli.topic)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(sub)

    # Build {joint_name: (prim, drive_type)} for joints that have a drive configured.
    stage = omni.usd.get_context().get_stage()
    joint_drive_map: Dict[str, tuple] = {}
    for name, path in joints.items():
        prim = stage.GetPrimAtPath(path)
        if UsdPhysics.DriveAPI.Get(prim, "angular"):
            joint_drive_map[name] = (prim, "angular")
        elif UsdPhysics.DriveAPI.Get(prim, "linear"):
            joint_drive_map[name] = (prim, "linear")
    print(f"[robot_mirror] Controllable joints: {list(joint_drive_map.keys())}")

    def _on_physics_step(step_size: float) -> None:
        executor.spin_once(timeout_sec=0)
        names, pos = sub.get()
        for joint_name, joint_pos in zip(names, pos):
            if joint_name not in joint_drive_map:
                continue
            prim, drive_type = joint_drive_map[joint_name]
            drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
            if drive_type == "angular":
                drive.GetTargetPositionAttr().Set(float(np.degrees(joint_pos)))
            else:
                drive.GetTargetPositionAttr().Set(float(joint_pos))

    physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_physics_step)

    while simulation_app.is_running():
        simulation_app.update()

    physx_sub = None
    executor.shutdown()
    sub.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        simulation_app.close()
        raise SystemExit(1)
