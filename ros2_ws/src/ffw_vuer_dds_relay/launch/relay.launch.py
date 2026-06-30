from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("ffw_vuer_dds_relay")
    start_yaml = os.environ.get("ROBOTIS_START_POSE_YAML", "")
    return LaunchDescription(
        [
            Node(
                package="ffw_vuer_dds_relay",
                executable="vuer_dds_relay_node",
                name="ffw_vuer_dds_relay",
                output="screen",
                parameters=[
                    {"urdf_path": f"{pkg_share}/assets/ffw_bg2_follower_nomesh.urdf"},
                    {"enable_pose_ik": True},
                    {"ik_smoothing": 0.35},
                    {"control_rate_hz": 60.0},
                    {"isaac_joint_topic": "/ffw_isaac/joint_targets"},
                    {"start_pose_yaml": start_yaml},
                    {"start_pose_step": "Start"},
                ],
            ),
        ]
    )
