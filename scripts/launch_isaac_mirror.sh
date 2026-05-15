#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_ROOT="${ISAAC_ROOT:-/isaac-sim}"

USD_PATH="${ROOT}/scenes/Scene_clean.usda"
ARTICULATION_PRIM="/World/Robot/ffw_sg2_follower"
SPAWN_PRIM="/World"
TOPIC="/joint_states"
HEADLESS=0

usage() {
  cat <<'EOF'
Usage: launch_isaac_mirror.sh [options]

Options:
  --usd-path <path>           USD scene file (default: scenes/Scene_clean.usda)
  --articulation-prim <path>  Articulation root prim (default: /World/Robot/ffw_sg2_follower)
  --spawn-prim <path>         Spawn prim path (default: /World)
  --topic <topic>             JointState topic to mirror (default: /joint_states)
  --headless                  Run without GUI
  --isaac-root <path>         Isaac Sim root dir (default: /isaac-sim or $ISAAC_ROOT)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --usd-path)          USD_PATH="$2";          shift 2 ;;
    --articulation-prim) ARTICULATION_PRIM="$2"; shift 2 ;;
    --spawn-prim)        SPAWN_PRIM="$2";        shift 2 ;;
    --topic)             TOPIC="$2";             shift 2 ;;
    --headless)          HEADLESS=1;             shift   ;;
    --isaac-root)        ISAAC_ROOT="$2";        shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 1 ;;
  esac
done

if [[ ! -f "${USD_PATH}" ]]; then
  echo "Error: USD file not found: ${USD_PATH}"
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"

# Prioritize Isaac Sim's own ROS2 bridge libraries over the system ROS2 ones.
# System ROS2 (Jazzy) ships Python 3.12 compiled .so files which crash Isaac's
# Python 3.11 rclpy at assertion time during node init.
export LD_LIBRARY_PATH="${ISAAC_ROOT}/exts/isaacsim.ros2.bridge/jazzy/lib:${LD_LIBRARY_PATH:-}"

# Isaac Sim runs Python 3.11. Strip Python 3.12 venv paths so Isaac doesn't
# pick up incompatible compiled extensions (e.g. numpy built for 3.12).
# This only affects the subshell spawned by this script.
if [[ -n "${PYTHONPATH:-}" ]]; then
  PYTHONPATH="$(echo "${PYTHONPATH}" | tr ':' '\n' | grep -v 'python3\.12' | paste -sd: -)"
  export PYTHONPATH
fi

ARGS=(
  "${ROOT}/isaac_sim/robot_state_mirror.py"
  --usd_path         "${USD_PATH}"
  --spawn_prim       "${SPAWN_PRIM}"
  --articulation_prim "${ARTICULATION_PRIM}"
  --topic            "${TOPIC}"
)
[[ "${HEADLESS}" == "1" ]] && ARGS+=(--headless)

echo "[isaac_mirror] Isaac root : ${ISAAC_ROOT}"
echo "[isaac_mirror] USD        : ${USD_PATH}"
echo "[isaac_mirror] Prim       : ${ARTICULATION_PRIM}"
echo "[isaac_mirror] Topic      : ${TOPIC}"
echo "[isaac_mirror] Domain ID  : ${ROS_DOMAIN_ID}"

exec "${ISAAC_ROOT}/python.sh" "${ARGS[@]}"
