"""
Mirror real robot /joint_states into Isaac Sim for visualization.
Uses Isaac Sim's built-in ROS2 bridge extension — no system rclpy required.
Physics is disabled on the robot (kinematic mode) — pure visualization only.
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


def main() -> None:
    # Open the USD file directly as the stage.
    omni.usd.get_context().open_stage(args_cli.usd_path)
    simulation_app.update()

    # Physics at 60Hz, rendering at 30Hz to reduce GPU load.
    world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 30.0, backend="torch", device="cpu")

    art_prim = _find_articulation_root()
    if not art_prim:
        raise RuntimeError("No articulation root found in stage.")

    robot = Articulation(art_prim, name="robot_mirror")
    world.scene.add(robot)
    world.reset()
    robot.initialize()
    world.play()

    sim_joint_names: List[str] = list(
        getattr(robot, "joint_names", None) or getattr(robot, "dof_names", [])
    )
    if not sim_joint_names:
        raise RuntimeError("No joint names found.")

    print(f"[robot_mirror] Articulation : {art_prim}")
    print(f"[robot_mirror] Joints ({len(sim_joint_names)}): {sim_joint_names}")
    print(f"[robot_mirror] Subscribing  : {args_cli.topic}")

    rclpy.init()
    sub = JointStateSub(args_cli.topic)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(sub)

    while simulation_app.is_running():
        if not world.is_playing():
            world.step(render=not args_cli.headless)
            continue

        executor.spin_once(timeout_sec=0)
        names, pos = sub.get()
        if names:
            q_tgt = _map_joints(sim_joint_names, names, pos)
            robot.set_joint_positions(torch.as_tensor(q_tgt).reshape(1, -1))

        world.step(render=not args_cli.headless)

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
