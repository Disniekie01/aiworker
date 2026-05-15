#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT}/.run_logs"
PID_DIR="${ROOT}/.run_pids"
mkdir -p "${LOG_DIR}" "${PID_DIR}"

DOMAIN_ID=30
WITH_VUER=0
WITH_ISAAC=0
WITH_LOOP_TEST=0
LOOP_HZ=100
VUER_INSECURE=0
VUER_DISABLE_SQUEEZE_GATE=0
USD_PATH=""
ISAAC_ROOT="${ISAAC_ROOT:-/isaac-sim}"
SPAWN_PRIM="/World"
ARTICULATION_PRIM="/World/Robot/ffw_sg2_follower"
HW_MODEL="sg2"
HW_ALL_IN_ONE=0

usage() {
  cat <<'EOF'
Usage:
  run_stack.sh start [options]
  run_stack.sh start-hardware [options]
  run_stack.sh start-ui [options]
  run_stack.sh start-tmux [options]
  run_stack.sh stop
  run_stack.sh status

Options for start:
  --domain-id <id>                ROS_DOMAIN_ID (default: 0)
  --with-vuer                     Launch robotis_vuer SG2 publisher
  --insecure-vuer                Launch Vuer in insecure ws/http mode (testing only)
  --disable-squeeze-gate         Disable VR squeeze deadman gate (testing only)
  --with-isaac                    Launch Isaac teleop script
  --usd-path <path>               USD path (required with --with-isaac)
  --isaac-root <path>             Isaac install root (default: $HOME/isaacsim or ISAAC_ROOT env)
  --spawn-prim <path>             Spawn prim for Isaac script (default: /World)
  --articulation-prim <path>      Articulation prim for Isaac (default: /World/Robot/ffw_sg2_follower)
  --with-loop-test                Launch looping joint test publisher
  --loop-hz <hz>                  Loop test publish rate (default: 100)

Options for start-hardware:
  --domain-id <id>                ROS_DOMAIN_ID (default: 0)
  --hw-model <sg2|bg2>            Hardware model for bringup (default: sg2)
  --hw-all-in-one                 Launch follower+leader with single bringup launch
EOF
}

pick_terminal_cmd() {
  if command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal"
    return
  fi
  if command -v x-terminal-emulator >/dev/null 2>&1; then
    echo "x-terminal-emulator"
    return
  fi
  echo ""
}

launch_ui_proc() {
  local term_bin="$1"
  local title="$2"
  local cmd="$3"
  # Run in interactive shell and keep it open with exec bash.
  local wrapped="set +e; echo \"[${title}] starting...\"; ${cmd}; rc=\$?; echo \"[${title}] exited with code \$rc\"; exec bash"
  if [[ "${term_bin}" == "gnome-terminal" ]]; then
    gnome-terminal --title="${title}" -- bash -ic "${wrapped}"
  else
    x-terminal-emulator -e bash -ic "${wrapped}"
  fi
}

start_proc() {
  local name="$1"
  local cmd="$2"
  local log="${LOG_DIR}/${name}.log"
  local pidf="${PID_DIR}/${name}.pid"

  if [[ -f "${pidf}" ]] && kill -0 "$(cat "${pidf}")" 2>/dev/null; then
    echo "[skip] ${name} already running (pid $(cat "${pidf}"))"
    return
  fi

  nohup bash -lc "${cmd}" >"${log}" 2>&1 &
  echo $! > "${pidf}"
  echo "[start] ${name} (pid $(cat "${pidf}")) -> ${log}"
}

stop_proc() {
  local name="$1"
  local pidf="${PID_DIR}/${name}.pid"
  if [[ -f "${pidf}" ]]; then
    local pid
    pid="$(cat "${pidf}")"
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" || true
      sleep 0.3
      kill -9 "${pid}" 2>/dev/null || true
      echo "[stop] ${name} (pid ${pid})"
    else
      echo "[stale] ${name} pid file existed, process missing"
    fi
    rm -f "${pidf}"
  else
    echo "[skip] ${name} not running"
  fi
}

status_proc() {
  local name="$1"
  local label="${2:-${name}}"
  local pidf="${PID_DIR}/${name}.pid"
  if [[ -f "${pidf}" ]] && kill -0 "$(cat "${pidf}")" 2>/dev/null; then
    echo "[up]   ${label} (pid $(cat "${pidf}"))"
  else
    echo "[down] ${label}"
  fi
}

cmd_start() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --domain-id) DOMAIN_ID="$2"; shift 2 ;;
      --with-vuer) WITH_VUER=1; shift ;;
      --insecure-vuer) VUER_INSECURE=1; shift ;;
      --disable-squeeze-gate) VUER_DISABLE_SQUEEZE_GATE=1; shift ;;
      --with-isaac) WITH_ISAAC=1; shift ;;
      --usd-path) USD_PATH="$2"; shift 2 ;;
      --isaac-root) ISAAC_ROOT="$2"; shift 2 ;;
      --spawn-prim) SPAWN_PRIM="$2"; shift 2 ;;
      --articulation-prim) ARTICULATION_PRIM="$2"; shift 2 ;;
      --with-loop-test) WITH_LOOP_TEST=1; shift ;;
      --loop-hz) LOOP_HZ="$2"; shift 2 ;;
      -h|--help) usage; exit 0 ;;
      *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
  done

  local base_env="source /opt/ros/jazzy/setup.bash && source \"${ROOT}/ros2_ws/install/setup.bash\" && source \"${ROOT}/env_local.bash\" && export ROS_DOMAIN_ID=\"${DOMAIN_ID}\""

  start_proc relay "${base_env} && ros2 launch ffw_vuer_dds_relay relay.launch.py"
  start_proc udp_bridge "${base_env} && /usr/bin/python3.12 \"${ROOT}/scripts/joint_targets_udp_bridge.py\" --udp_host 127.0.0.1 --udp_port 15000"

  if [[ "${WITH_VUER}" == "1" ]]; then
    local docker_env="-e ROS_DOMAIN_ID=${DOMAIN_ID}"
    [[ "${VUER_INSECURE}" == "1" ]] && docker_env="${docker_env} -e ROBOTIS_VUER_INSECURE=1"
    [[ "${VUER_DISABLE_SQUEEZE_GATE}" == "1" ]] && docker_env="${docker_env} -e ROBOTIS_VUER_DISABLE_SQUEEZE_GATE=1"
    start_proc vuer "docker exec ${docker_env} robotis-applications bash -ic 'ros2 launch robotis_vuer vr.launch.py model:=sg2'"
  fi

  if [[ "${WITH_LOOP_TEST}" == "1" ]]; then
    start_proc loop_test "${base_env} && /usr/bin/python3.12 \"${ROOT}/scripts/loop_joint_test.py\" --hz \"${LOOP_HZ}\""
  fi

  if [[ "${WITH_ISAAC}" == "1" ]]; then
    if [[ -z "${USD_PATH}" ]]; then
      if [[ -f "${ROOT}/scenes/Scene_clean.usda" ]]; then
        USD_PATH="${ROOT}/scenes/Scene_clean.usda"
      else
        echo "Error: --with-isaac requires --usd-path (or provide ${ROOT}/scenes/Scene_clean.usda)"
        exit 1
      fi
    fi
    start_proc isaac "
      cd \"${ISAAC_ROOT}\" &&
      ./python.sh \"${ROOT}/isaac_sim/standalone_ffw_joint_teleop.py\" \
        --usd_path \"${USD_PATH}\" \
        --spawn_prim \"${SPAWN_PRIM}\" \
        --articulation_prim \"${ARTICULATION_PRIM}\" \
        --input_mode udp \
        --udp_host 127.0.0.1 \
        --udp_port 15000
    "
  fi
}

cmd_start_ui() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --domain-id) DOMAIN_ID="$2"; shift 2 ;;
      --with-vuer) WITH_VUER=1; shift ;;
      --insecure-vuer) VUER_INSECURE=1; shift ;;
      --disable-squeeze-gate) VUER_DISABLE_SQUEEZE_GATE=1; shift ;;
      --with-isaac) WITH_ISAAC=1; shift ;;
      --usd-path) USD_PATH="$2"; shift 2 ;;
      --isaac-root) ISAAC_ROOT="$2"; shift 2 ;;
      --spawn-prim) SPAWN_PRIM="$2"; shift 2 ;;
      --articulation-prim) ARTICULATION_PRIM="$2"; shift 2 ;;
      --with-loop-test) WITH_LOOP_TEST=1; shift ;;
      --loop-hz) LOOP_HZ="$2"; shift 2 ;;
      -h|--help) usage; exit 0 ;;
      *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
  done

  local term_bin
  term_bin="$(pick_terminal_cmd)"
  if [[ -z "${term_bin}" ]]; then
    echo "No terminal launcher found (gnome-terminal/x-terminal-emulator)."
    exit 1
  fi

  local base_env="source /opt/ros/jazzy/setup.bash && source \"${ROOT}/ros2_ws/install/setup.bash\" && source \"${ROOT}/env_local.bash\" && export ROS_DOMAIN_ID=\"${DOMAIN_ID}\""

  launch_ui_proc "${term_bin}" "relay" "${base_env} && ros2 launch ffw_vuer_dds_relay relay.launch.py"
  launch_ui_proc "${term_bin}" "udp_bridge" "${base_env} && /usr/bin/python3.12 \"${ROOT}/scripts/joint_targets_udp_bridge.py\" --udp_host 127.0.0.1 --udp_port 15000"

  if [[ "${WITH_VUER}" == "1" ]]; then
    local vuer_env=""
    if [[ "${VUER_INSECURE}" == "1" ]]; then
      vuer_env="export ROBOTIS_VUER_INSECURE=1 && "
    fi
    if [[ "${VUER_DISABLE_SQUEEZE_GATE}" == "1" ]]; then
      vuer_env="${vuer_env}export ROBOTIS_VUER_DISABLE_SQUEEZE_GATE=1 && "
    fi
    launch_ui_proc "${term_bin}" "vuer" "${base_env} && source \"${ROOT}/robotis_applications/install/setup.bash\" && ${vuer_env}ros2 launch robotis_vuer vr.launch.py model:=sg2"
  fi
  if [[ "${WITH_LOOP_TEST}" == "1" ]]; then
    launch_ui_proc "${term_bin}" "loop_test" "${base_env} && /usr/bin/python3.12 \"${ROOT}/scripts/loop_joint_test.py\" --hz \"${LOOP_HZ}\""
  fi
  if [[ "${WITH_ISAAC}" == "1" ]]; then
    if [[ -z "${USD_PATH}" ]]; then
      if [[ -f "${ROOT}/scenes/Scene_clean.usda" ]]; then
        USD_PATH="${ROOT}/scenes/Scene_clean.usda"
      else
        echo "Error: --with-isaac requires --usd-path (or provide ${ROOT}/scenes/Scene_clean.usda)"
        exit 1
      fi
    fi
    launch_ui_proc "${term_bin}" "isaac" "cd \"${ISAAC_ROOT}\" && ./python.sh \"${ROOT}/isaac_sim/standalone_ffw_joint_teleop.py\" --usd_path \"${USD_PATH}\" --spawn_prim \"${SPAWN_PRIM}\" --articulation_prim \"${ARTICULATION_PRIM}\" --input_mode udp --udp_host 127.0.0.1 --udp_port 15000"
  fi
}

cmd_start_tmux() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --domain-id) DOMAIN_ID="$2"; shift 2 ;;
      --with-vuer) WITH_VUER=1; shift ;;
      --insecure-vuer) VUER_INSECURE=1; shift ;;
      --disable-squeeze-gate) VUER_DISABLE_SQUEEZE_GATE=1; shift ;;
      --with-isaac) WITH_ISAAC=1; shift ;;
      --usd-path) USD_PATH="$2"; shift 2 ;;
      --isaac-root) ISAAC_ROOT="$2"; shift 2 ;;
      --spawn-prim) SPAWN_PRIM="$2"; shift 2 ;;
      --articulation-prim) ARTICULATION_PRIM="$2"; shift 2 ;;
      --with-loop-test) WITH_LOOP_TEST=1; shift ;;
      --loop-hz) LOOP_HZ="$2"; shift 2 ;;
      -h|--help) usage; exit 0 ;;
      *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
  done

  if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux not found. Install tmux or use start/start-ui."
    exit 1
  fi

  local session="robotis_stack"
  tmux kill-session -t "${session}" 2>/dev/null || true
  tmux new-session -d -s "${session}" -n relay

  local base_env="source /opt/ros/jazzy/setup.bash && source \"${ROOT}/ros2_ws/install/setup.bash\" && source \"${ROOT}/env_local.bash\" && export ROS_DOMAIN_ID=\"${DOMAIN_ID}\""

  tmux send-keys -t "${session}:relay" "${base_env} && ros2 launch ffw_vuer_dds_relay relay.launch.py" C-m

  tmux new-window -t "${session}" -n udp_bridge
  tmux send-keys -t "${session}:udp_bridge" "${base_env} && /usr/bin/python3.12 \"${ROOT}/scripts/joint_targets_udp_bridge.py\" --udp_host 127.0.0.1 --udp_port 15000" C-m

  if [[ "${WITH_LOOP_TEST}" == "1" ]]; then
    tmux new-window -t "${session}" -n loop_test
    tmux send-keys -t "${session}:loop_test" "${base_env} && /usr/bin/python3.12 \"${ROOT}/scripts/loop_joint_test.py\" --hz \"${LOOP_HZ}\"" C-m
  fi

  if [[ "${WITH_VUER}" == "1" ]]; then
    local vuer_env=""
    if [[ "${VUER_INSECURE}" == "1" ]]; then
      vuer_env="export ROBOTIS_VUER_INSECURE=1 && "
    fi
    if [[ "${VUER_DISABLE_SQUEEZE_GATE}" == "1" ]]; then
      vuer_env="${vuer_env}export ROBOTIS_VUER_DISABLE_SQUEEZE_GATE=1 && "
    fi
    tmux new-window -t "${session}" -n vuer
    tmux send-keys -t "${session}:vuer" "${base_env} && source \"${ROOT}/robotis_applications/install/setup.bash\" && ${vuer_env}ros2 launch robotis_vuer vr.launch.py model:=sg2" C-m
  fi

  if [[ "${WITH_ISAAC}" == "1" ]]; then
    if [[ -z "${USD_PATH}" ]]; then
      if [[ -f "${ROOT}/scenes/Scene_clean.usda" ]]; then
        USD_PATH="${ROOT}/scenes/Scene_clean.usda"
      else
        echo "Error: --with-isaac requires --usd-path (or provide ${ROOT}/scenes/Scene_clean.usda)"
        exit 1
      fi
    fi
    tmux new-window -t "${session}" -n isaac
    tmux send-keys -t "${session}:isaac" "cd \"${ISAAC_ROOT}\" && ./python.sh \"${ROOT}/isaac_sim/standalone_ffw_joint_teleop.py\" --usd_path \"${USD_PATH}\" --spawn_prim \"${SPAWN_PRIM}\" --articulation_prim \"${ARTICULATION_PRIM}\" --input_mode udp --udp_host 127.0.0.1 --udp_port 15000" C-m
  fi

  echo "tmux session started: ${session}"
  echo "Attach with: tmux attach -t ${session}"
}

cmd_start_hardware() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --domain-id) DOMAIN_ID="$2"; shift 2 ;;
      --hw-model) HW_MODEL="$2"; shift 2 ;;
      --hw-all-in-one) HW_ALL_IN_ONE=1; shift ;;
      -h|--help) usage; exit 0 ;;
      *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
  done

  local base_env="source /opt/ros/jazzy/setup.bash && source \"${ROOT}/env_local.bash\" && export ROS_DOMAIN_ID=\"${DOMAIN_ID}\""
  local pkg_setup="source \"${ROOT}/robotis_applications/install/setup.bash\""

  if [[ "${HW_MODEL}" != "sg2" && "${HW_MODEL}" != "bg2" ]]; then
    echo "Error: --hw-model must be 'sg2' or 'bg2'"
    exit 1
  fi

  if [[ "${HW_ALL_IN_ONE}" == "1" ]]; then
    start_proc hw_all_in_one "${base_env} && ${pkg_setup} && ros2 launch ffw_bringup ffw_${HW_MODEL}_ai.launch.py"
    return
  fi

  start_proc hw_follower "${base_env} && ${pkg_setup} && ros2 launch ffw_bringup ffw_${HW_MODEL}_follower_ai.launch.py"
  # Leader type follows ROBOTIS guide for hardware teleoperation.
  start_proc hw_leader "${base_env} && ${pkg_setup} && ros2 launch ffw_bringup ffw_lg2_leader_ai.launch.py"
}

cmd_stop() {
  stop_proc hw_all_in_one
  stop_proc hw_leader
  stop_proc hw_follower
  stop_proc loop_test
  stop_proc isaac
  if docker exec robotis-applications pkill -f "[r]os2 launch" 2>/dev/null; then
    echo "[stop] vuer/launch  (container: ros2 launch)"
  else
    echo "[skip] vuer/launch  (container: ros2 launch not found or no container)"
  fi
  sleep 1
  if docker exec robotis-applications pkill -f "[r]obotis_vuer" 2>/dev/null; then
    echo "[stop] vuer/node    (container: robotis_vuer)"
  else
    echo "[skip] vuer/node    (container: robotis_vuer not found or no container)"
  fi
  sleep 1
  docker exec robotis-applications pkill -9 -f "[r]obotis_vuer" 2>/dev/null || true
  stop_proc vuer
  stop_proc udp_bridge
  stop_proc relay
}

cmd_status() {
  status_proc hw_all_in_one
  status_proc hw_follower
  status_proc hw_leader
  status_proc relay
  status_proc udp_bridge
  status_proc vuer "vuer/docker_exec"
  if docker ps --format '{{.Names}}' | grep -qx "robotis-applications" 2>/dev/null; then
    if docker exec robotis-applications bash -c 'pgrep -f "[r]os2 launch" > /dev/null 2>&1'; then
      echo "[up]   vuer/launch  (container: ros2 launch)"
    else
      echo "[down] vuer/launch  (container: ros2 launch)"
    fi
    if docker exec robotis-applications bash -c 'pgrep -f "[r]obotis_vuer" > /dev/null 2>&1'; then
      echo "[up]   vuer/node    (container: vr_publisher_sg2)"
    else
      echo "[down] vuer/node    (container: vr_publisher_sg2)"
    fi
  else
    echo "[down] vuer/launch  (robotis-applications container not running)"
    echo "[down] vuer/node    (robotis-applications container not running)"
  fi
  status_proc isaac
  status_proc loop_test
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

subcmd="$1"
shift

case "${subcmd}" in
  start) cmd_start "$@" ;;
  start-hardware) cmd_start_hardware "$@" ;;
  start-ui) cmd_start_ui "$@" ;;
  start-tmux) cmd_start_tmux "$@" ;;
  stop) cmd_stop ;;
  status) cmd_status ;;
  -h|--help) usage ;;
  *) echo "Unknown command: ${subcmd}"; usage; exit 1 ;;
esac
