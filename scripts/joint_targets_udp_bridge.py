#!/usr/bin/env python3
# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""Bridge ROS2 JointState -> UDP JSON for Isaac Python 3.11 runtimes.

Run in a ROS/Jazzy terminal (Python 3.12):
  source /opt/ros/jazzy/setup.bash
  source /home/disniekie/Robotis/robotis_vr_isaac/ros2_ws/install/setup.bash
  python /home/disniekie/Robotis/robotis_vr_isaac/scripts/joint_targets_udp_bridge.py
"""

from __future__ import annotations

import argparse
import json
import socket
import sys

if sys.version_info[:2] != (3, 12):
    raise RuntimeError(
        "joint_targets_udp_bridge.py must run with Python 3.12 for ROS Jazzy rclpy.\n"
        "Use:\n"
        "  /usr/bin/python3.12 /home/disniekie/Robotis/robotis_vr_isaac/scripts/joint_targets_udp_bridge.py\n"
        "or deactivate conda before running."
    )

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointTargetsUdpBridge(Node):
    def __init__(self, topic: str, host: str, port: int) -> None:
        super().__init__("joint_targets_udp_bridge")
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._dst = (host, int(port))
        self.create_subscription(JointState, topic, self._cb, 10)
        self.get_logger().info(
            f"Forwarding {topic} -> udp://{host}:{port} (JSON name/position)"
        )

    def _cb(self, msg: JointState) -> None:
        payload = {
            "name": list(msg.name),
            "position": [float(x) for x in msg.position],
        }
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self._sock.sendto(data, self._dst)

    def destroy_node(self) -> bool:
        self._sock.close()
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--joint_state_topic", default="/ffw_isaac/joint_targets")
    parser.add_argument("--udp_host", default="127.0.0.1")
    parser.add_argument("--udp_port", type=int, default=15000)
    args = parser.parse_args()

    rclpy.init()
    node = JointTargetsUdpBridge(args.joint_state_topic, args.udp_host, args.udp_port)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
