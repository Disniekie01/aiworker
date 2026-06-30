# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""
Bridge ROBOTIS Vuer (ROS 2) teleop to Cyclone DDS topics consumed by FFWSG2Sdk / record_demos.

Subscribes to the same JointTrajectory topics vr_publisher_sg2 publishes on ROS 2, converts
them to DDS, and republishes. Subscribes to /l_goal_pose and /r_goal_pose and runs PyBullet
IK so arm joint trajectories exist on the DDS broadcaster topics (what the SDK expects).

Also publishes sensor_msgs/JointState on /ffw_isaac/joint_targets for standalone Isaac Sim
scripts that drive articulation directly (no Isaac Lab).
"""
from __future__ import annotations

import os
import threading
from typing import Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory

# ROBOTIS DDS (install: pip install -e robotis_lab/third_party/robotis_dds_python)
try:
    from robotis_dds_python.idl.trajectory_msgs.msg import JointTrajectory_ as DdsJointTrajectory_
    from robotis_dds_python.tools.topic_manager import TopicManager
except ImportError as e:
    TopicManager = None  # type: ignore
    DdsJointTrajectory_ = None  # type: ignore
    _IMPORT_ERR = e
else:
    _IMPORT_ERR = None

from ffw_vuer_dds_relay.dds_convert import make_dds_joint_traj, pose_to_numpy
from ffw_vuer_dds_relay.pybullet_ik import FfwBg2PyBulletIk

# Same order as FFWSG2Sdk.joint_names
FULL_JOINT_ORDER: List[str] = [
    "arm_l_joint1",
    "arm_l_joint2",
    "arm_l_joint3",
    "arm_l_joint4",
    "arm_l_joint5",
    "arm_l_joint6",
    "arm_l_joint7",
    "gripper_l_joint1",
    "arm_r_joint1",
    "arm_r_joint2",
    "arm_r_joint3",
    "arm_r_joint4",
    "arm_r_joint5",
    "arm_r_joint6",
    "arm_r_joint7",
    "gripper_r_joint1",
    "lift_joint",
    "head_joint1",
    "head_joint2",
]

ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]


def _traj_to_map(msg: JointTrajectory) -> Dict[str, float]:
    if not msg.points or not msg.joint_names:
        return {}
    last = msg.points[-1]
    return {n: float(p) for n, p in zip(msg.joint_names, last.positions)}


class VuerDdsRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("ffw_vuer_dds_relay")
        if TopicManager is None:
            raise RuntimeError(
                "robotis_dds_python not importable. "
                "Set PYTHONPATH to robotis_lab/third_party/robotis_dds_python "
                f"or pip install -e that package. Original: {_IMPORT_ERR}"
            )

        self.declare_parameter("urdf_path", "")
        self.declare_parameter("enable_pose_ik", True)
        self.declare_parameter("ik_smoothing", 0.35)
        self.declare_parameter("control_rate_hz", 60.0)
        self.declare_parameter("isaac_joint_topic", "/ffw_isaac/joint_targets")
        self.declare_parameter("start_pose_yaml", "")
        self.declare_parameter("start_pose_step", "Start")

        urdf = str(self.get_parameter("urdf_path").value).strip()
        if not urdf:
            urdf = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
                "assets",
                "ffw_bg2_follower_nomesh.urdf",
            )
        if not os.path.isfile(urdf):
            self.get_logger().error(f"URDF not found: {urdf}. Run scripts/strip_urdf_meshes.py.")
            raise FileNotFoundError(urdf)

        self._enable_ik = bool(self.get_parameter("enable_pose_ik").value)
        self._alpha = float(np.clip(float(self.get_parameter("ik_smoothing").value), 0.0, 1.0))
        self._ik: Optional[FfwBg2PyBulletIk] = None
        if self._enable_ik:
            try:
                self._ik = FfwBg2PyBulletIk(urdf)
                self.get_logger().info(f"PyBullet IK using {urdf}")
            except Exception as e:
                self.get_logger().error(f"Pose IK disabled (PyBullet init failed): {e}")
                self._ik = None

        domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
        self._tm = TopicManager(domain_id=domain_id)
        self._w_left = self._tm.topic_writer(
            "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
            DdsJointTrajectory_,
        )
        self._w_right = self._tm.topic_writer(
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
            DdsJointTrajectory_,
        )
        self._w_head = self._tm.topic_writer(
            "/leader/joystick_controller_left/joint_trajectory",
            DdsJointTrajectory_,
        )
        self._w_lift = self._tm.topic_writer(
            "/leader/joystick_controller_right/joint_trajectory",
            DdsJointTrajectory_,
        )

        self._lock = threading.Lock()
        self._lift_map: Dict[str, float] = {}
        self._head_map: Dict[str, float] = {}
        self._grip_l_map: Dict[str, float] = {}
        self._grip_r_map: Dict[str, float] = {}
        self._l_pose: Optional[PoseStamped] = None
        self._r_pose: Optional[PoseStamped] = None

        self._cmd_prev = {n: 0.0 for n in FULL_JOINT_ORDER}
        self._cmd_prev["lift_joint"] = -0.15
        self._vr_active = False
        self._load_start_pose_from_yaml()

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            JointTrajectory,
            "/leader/joystick_controller_right/joint_trajectory",
            self._cb_lift_ros,
            qos,
        )
        self.create_subscription(
            JointTrajectory,
            "/leader/joystick_controller_left/joint_trajectory",
            self._cb_head_ros,
            qos,
        )
        self.create_subscription(
            JointTrajectory,
            "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
            self._cb_grip_l_ros,
            qos,
        )
        self.create_subscription(
            JointTrajectory,
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
            self._cb_grip_r_ros,
            qos,
        )
        self.create_subscription(PoseStamped, "/l_goal_pose", self._cb_l_pose, qos)
        self.create_subscription(PoseStamped, "/r_goal_pose", self._cb_r_pose, qos)
        self.create_subscription(Bool, "/reactivate", self._cb_reactivate, 10)

        self._pub_isaac = self.create_publisher(
            JointState, str(self.get_parameter("isaac_joint_topic").value), 10
        )

        hz = float(self.get_parameter("control_rate_hz").value)
        self._timer = self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self.get_logger().info(
            f"Vuer DDS relay running (domain {domain_id}). "
            f"Isaac joint targets: {self.get_parameter('isaac_joint_topic').value}. "
            f"Arm IK gated on /reactivate (X left + A right while squeezing)."
        )

    def _load_start_pose_from_yaml(self) -> None:
        yaml_path = str(self.get_parameter("start_pose_yaml").value).strip()
        if not yaml_path:
            return
        step = str(self.get_parameter("start_pose_step").value).strip() or "Start"
        try:
            import sys
            from pathlib import Path

            yaml_p = Path(yaml_path).resolve()
            root = os.environ.get("ROBOTIS_VR_ISAAC_ROOT", "").strip()
            if not root:
                root = str(yaml_p.parent.parent)
            if str(root) not in sys.path:
                sys.path.insert(0, str(root))
            from yaml_pose_loader import load_step_pose

            pose = load_step_pose(yaml_path, step)
            for n in FULL_JOINT_ORDER:
                if n in pose:
                    self._cmd_prev[n] = float(pose[n])
            self.get_logger().info(
                f"Start pose from {yaml_path} step={step} "
                f"(base_x={pose.get('isaac_base_x', 0):.3f})"
            )
        except Exception as exc:
            self.get_logger().warn(f"Could not load start pose from {yaml_path}: {exc!r}")

    def destroy_node(self) -> bool:
        if self._ik is not None:
            self._ik.close()
        return super().destroy_node()

    def _cb_lift_ros(self, msg: JointTrajectory) -> None:
        with self._lock:
            self._lift_map = _traj_to_map(msg)

    def _cb_head_ros(self, msg: JointTrajectory) -> None:
        with self._lock:
            self._head_map = _traj_to_map(msg)

    def _cb_grip_l_ros(self, msg: JointTrajectory) -> None:
        with self._lock:
            self._grip_l_map = _traj_to_map(msg)

    def _cb_grip_r_ros(self, msg: JointTrajectory) -> None:
        with self._lock:
            self._grip_r_map = _traj_to_map(msg)

    def _cb_l_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._l_pose = msg

    def _cb_r_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._r_pose = msg

    def _cb_reactivate(self, msg: Bool) -> None:
        with self._lock:
            active = bool(msg.data)
            if active == self._vr_active:
                return
            self._vr_active = active
            if not active:
                self._l_pose = None
                self._r_pose = None
        state = "active" if active else "paused"
        self.get_logger().info(f"VR arm IK {state} (/reactivate={active})")

    def _tick(self) -> None:
        with self._lock:
            cmd = dict(self._cmd_prev)
            cmd.update(self._lift_map)
            cmd.update(self._head_map)
            cmd.update(self._grip_l_map)
            cmd.update(self._grip_r_map)

            seed = dict(cmd)
            if self._vr_active:
                if self._ik is not None and self._l_pose is not None:
                    pos, quat = pose_to_numpy(self._l_pose)
                    sol = self._ik.solve("left", pos, quat, seed)
                    if sol:
                        cmd.update(sol)
                if self._ik is not None and self._r_pose is not None:
                    pos, quat = pose_to_numpy(self._r_pose)
                    sol = self._ik.solve("right", pos, quat, seed)
                    if sol:
                        cmd.update(sol)

            a = self._alpha
            for n in FULL_JOINT_ORDER:
                if n in cmd:
                    cmd[n] = float((1.0 - a) * self._cmd_prev.get(n, 0.0) + a * cmd[n])

            self._cmd_prev = dict(cmd)

            try:
                lj = ARM_L + ["gripper_l_joint1"]
                rj = ARM_R + ["gripper_r_joint1"]
                dds_l = make_dds_joint_traj(lj, [cmd.get(n, 0.0) for n in lj])
                dds_r = make_dds_joint_traj(rj, [cmd.get(n, 0.0) for n in rj])
                self._w_left.write(dds_l)
                self._w_right.write(dds_r)
                if self._head_map:
                    hj = sorted(self._head_map.keys())
                    self._w_head.write(
                        make_dds_joint_traj(hj, [self._head_map[k] for k in hj])
                    )
                if self._lift_map:
                    lj = sorted(self._lift_map.keys())
                    self._w_lift.write(
                        make_dds_joint_traj(lj, [self._lift_map[k] for k in lj])
                    )
            except Exception as e:
                self.get_logger().warn(f"DDS trajectory write: {e}")

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.header.frame_id = "base_link"
            js.name = list(FULL_JOINT_ORDER)
            js.position = [float(cmd.get(n, 0.0)) for n in FULL_JOINT_ORDER]
            self._pub_isaac.publish(js)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VuerDdsRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
