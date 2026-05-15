#!/usr/bin/env bash
# Extracted from run_stack.sh cmd_start isaac block.
# Launches the original standalone_ffw_joint_teleop.py for comparison.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_ROOT="${ISAAC_ROOT:-/isaac-sim}"
USD_PATH="${ROOT}/scenes/Scene_clean.usda"
SPAWN_PRIM="/World"
ARTICULATION_PRIM="/World/Robot/ffw_sg2_follower"
DOMAIN_ID=30

source /opt/ros/jazzy/setup.bash
source "${ROOT}/ros2_ws/install/setup.bash"
source "${ROOT}/env_local.bash"
export ROS_DOMAIN_ID="${DOMAIN_ID}"

cd "${ISAAC_ROOT}"
exec ./python.sh "${ROOT}/isaac_sim/standalone_ffw_joint_teleop.py" \
  --usd_path          "${USD_PATH}" \
  --spawn_prim        "${SPAWN_PRIM}" \
  --articulation_prim "${ARTICULATION_PRIM}" \
  --input_mode        udp \
  --udp_host          127.0.0.1 \
  --udp_port          15000
