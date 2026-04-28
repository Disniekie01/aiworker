# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""ROS 2 trajectory_msgs -> ROBOTIS DDS JointTrajectory_."""
from __future__ import annotations

from builtin_interfaces.msg import Duration as RosDuration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header as RosHeader
from trajectory_msgs.msg import JointTrajectory as RosJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint as RosJointTrajectoryPoint

from robotis_dds_python.idl.builtin_interfaces.msg import Duration_
from robotis_dds_python.idl.builtin_interfaces.msg import Time_
from robotis_dds_python.idl.std_msgs.msg import Header_
from robotis_dds_python.idl.trajectory_msgs.msg import JointTrajectoryPoint_
from robotis_dds_python.idl.trajectory_msgs.msg import JointTrajectory_


def ros_time_to_dds(t) -> Time_:
    return Time_(sec=int(t.sec), nanosec=int(t.nanosec))


def ros_duration_to_dds(d: RosDuration) -> Duration_:
    return Duration_(sec=int(d.sec), nanosec=int(d.nanosec))


def ros_header_to_dds(h: RosHeader) -> Header_:
    return Header_(stamp=ros_time_to_dds(h.stamp), frame_id=h.frame_id)


def ros_joint_trajectory_to_dds(msg: RosJointTrajectory) -> JointTrajectory_:
    pts = []
    for p in msg.points:
        pts.append(
            JointTrajectoryPoint_(
                positions=list(p.positions),
                velocities=list(p.velocities) if p.velocities else [],
                accelerations=list(p.accelerations) if p.accelerations else [],
                effort=list(p.effort) if p.effort else [],
                time_from_start=ros_duration_to_dds(p.time_from_start),
            )
        )
    return JointTrajectory_(
        header=ros_header_to_dds(msg.header),
        joint_names=list(msg.joint_names),
        points=pts,
    )


def make_dds_joint_traj(
    joint_names: list,
    positions: list,
    frame_id: str = "base_link",
    *,
    sec: int = 0,
    nanosec: int = 0,
) -> JointTrajectory_:
    """Single-point trajectory (what FFWSG2Sdk consumes)."""
    import time

    now = time.time()
    s, ns = int(now), int((now - int(now)) * 1e9)
    pt = JointTrajectoryPoint_(
        positions=[float(x) for x in positions],
        velocities=[],
        accelerations=[],
        effort=[],
        time_from_start=Duration_(sec=sec, nanosec=nanosec),
    )
    return JointTrajectory_(
        header=Header_(stamp=Time_(sec=s, nanosec=ns), frame_id=frame_id),
        joint_names=list(joint_names),
        points=[pt],
    )


def pose_to_numpy(p: PoseStamped):
    import numpy as np

    pos = np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z], dtype=np.float64)
    q = np.array(
        [
            p.pose.orientation.x,
            p.pose.orientation.y,
            p.pose.orientation.z,
            p.pose.orientation.w,
        ],
        dtype=np.float64,
    )
    return pos, q
