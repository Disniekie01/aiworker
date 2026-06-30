#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT}/.run_logs"
PID_DIR="${ROOT}/.run_pids"
mkdir -p "${LOG_DIR}" "${PID_DIR}"

DOMAIN_ID=0
WITH_VUER=0
WITH_ISAAC=0
WITH_LOOP_TEST=0
WITHOUT_RELAY=0
SOFT_SHIRTS=1
GRASP_TEST_CUBE=0
GRASP_MODE=""
HYBRID_VR=PICK_PLACE=0
PHYSICS_GRASP=0
VR_SHIRT_GRASP=0
VR_SHIRT_PRIM="/World/Shirt/Meshes/Sketchfab_model/simple_folded_shirt_obj_cleaner_materialmerger_gles/Object_2/Object_0"
VR_SHIRT_ROOT="/World/Shirt"
LOOP_HZ=100
VUER_INSECURE=0
VUER_DISABLE_SQUEEZE_GATE=0
USD_PATH=""
ISAAC_ROOT="${ISAAC_ROOT:-$HOME/isaacsim}"
SPAWN_PRIM="/World"
ARTICULATION_PRIM="/World/Robot/ffw_sg2_follower"
BASE_MOVE_PRIM="/World/Robot"
HW_MODEL="sg2"
START_POSE_YAML=""

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
  --without-relay                 Skip ffw_vuer_dds_relay (use for scripted Isaac poses)
  --soft-shirts                   GPU deformable t-shirts in Isaac (default: on)
  --no-soft-shirts                Use rigid shirt meshes instead of deformables
  --grasp-test-cube               Hide shirt pile; spawn flat rigid cube for grip testing
  --grasp-mode <xform|constraint> PhysX FixedJoint grasp (default: constraint with test cube)
  --pick-place                      Scripted pick_place.yaml (no VR; implies --without-relay)
  --physics-grasp                   Rigid dynamic shirt + gripper friction (no attach/FixedJoint)
  --hybrid-vr                       VR arms + scripted base/lift (UDP bridge merge mode)
  --vr-shirt-grasp                  VR trigger pickup of /World/Shirt (rigid + constraint)
  --vr-shirt-prim <path>            Shirt mesh prim (default: /World/Shirt/.../Object_0)
  --vr-shirt-root <path>            Shirt root for physics (default: /World/Shirt)
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
  local pidf="${PID_DIR}/${name}.pid"
  if [[ -f "${pidf}" ]] && kill -0 "$(cat "${pidf}")" 2>/dev/null; then
    echo "[up]   ${name} (pid $(cat "${pidf}"))"
  else
    echo "[down] ${name}"
  fi
}

udp_bridge_args() {
  local args="--udp_host 127.0.0.1 --udp_port 15000"
  if [[ "${HYBRID_VR}" == "1" ]]; then
    args="${args} --hybrid"
  fi
  if [[ "${PHYSICS_GRASP}" == "1" && -n "${START_POSE_YAML}" ]]; then
    args="${args} --grasp-offset-yaml \"${START_POSE_YAML}\""
  fi
  echo "${args}"
}

isaac_grasp_flags() {
  if [[ "${GRASP_TEST_CUBE}" == "1" ]]; then
    echo "--shirt-link-gripper --shirt-link-finger attach --shirt-release-step Drop"
    return
  fi
  if [[ "${PICK_PLACE}" == "1" ]]; then
    return
  fi
  if [[ "${VR_SHIRT_GRASP}" == "1" ]]; then
    echo "--shirt-grasp --shirt-grasp-mode overlap --shirt-prim \"${VR_SHIRT_PRIM}\" --shirt-root \"${VR_SHIRT_ROOT}\""
  fi
}

ensure_grasp_mode_for_isaac() {
  if [[ "${GRASP_TEST_CUBE}" == "1" && -z "${GRASP_MODE}" ]]; then
    GRASP_MODE="constraint"
  fi
  if [[ "${VR_SHIRT_GRASP}" == "1" && -z "${GRASP_MODE}" ]]; then
    GRASP_MODE="constraint"
  fi
}

isaac_start_pose_flags() {
  if [[ -n "${START_POSE_YAML}" ]]; then
    echo "--start-pose-yaml \"${START_POSE_YAML}\" --start-pose-step Start"
  fi
}

# Isaac's python.sh breaks when conda is active (wrong argparse / stdlib).
isaac_env_preamble() {
  cat <<'EOF'
if command -v conda >/dev/null 2>&1 && [[ -n "${CONDA_DEFAULT_ENV:-}" ]]; then
  eval "$(conda shell.bash hook)" && conda deactivate
fi
EOF
}

physics_grasp_flag() {
  if [[ "${PHYSICS_GRASP}" == "1" ]]; then
    echo "--physics-grasp --shirt-root \"${VR_SHIRT_ROOT}\" --shirt-prim \"${VR_SHIRT_PRIM}\""
  fi
}

default_usd_path() {
  if [[ -f "${ROOT}/scenes/newscene/newscene.usda" && -f "${ROOT}/scenes/newscene/Scene.usda" ]]; then
    echo "${ROOT}/scenes/newscene/newscene.usda"
  elif [[ -f "${ROOT}/scenes/Scene_clean.usda" ]]; then
    echo "${ROOT}/scenes/Scene_clean.usda"
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
      --without-relay) WITHOUT_RELAY=1; shift ;;
      --soft-shirts) SOFT_SHIRTS=1; shift ;;
      --no-soft-shirts) SOFT_SHIRTS=0; shift ;;
      --grasp-test-cube) GRASP_TEST_CUBE=1; shift ;;
      --grasp-mode) GRASP_MODE="$2"; shift 2 ;;
      --pick-place) PICK_PLACE=1; WITHOUT_RELAY=1; SOFT_SHIRTS=0; PHYSICS_GRASP=1; shift ;;
      --physics-grasp) PHYSICS_GRASP=1; SOFT_SHIRTS=0; shift ;;
      --hybrid-vr) HYBRID_VR=1; VUER_DISABLE_SQUEEZE_GATE=1; shift ;;
      --vr-shirt-grasp) VR_SHIRT_GRASP=1; SOFT_SHIRTS=0; shift ;;
      --vr-shirt-prim) VR_SHIRT_PRIM="$2"; shift 2 ;;
      --vr-shirt-root) VR_SHIRT_ROOT="$2"; shift 2 ;;
      --loop-hz) LOOP_HZ="$2"; shift 2 ;;
      -h|--help) usage; exit 0 ;;
      *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
  done

  local base_env="source /opt/ros/jazzy/setup.bash && source \"${ROOT}/ros2_ws/install/setup.bash\" && source \"${ROOT}/env_local.bash\" && export ROS_DOMAIN_ID=\"${DOMAIN_ID}\" && export ROBOTIS_VR_ISAAC_ROOT=\"${ROOT}\""

  if [[ "${HYBRID_VR}" == "1" && "${WITHOUT_RELAY}" == "1" ]]; then
    echo "Error: --hybrid-vr requires the VR relay (do not use --without-relay)"
    exit 1
  fi
  if [[ -z "${START_POSE_YAML}" ]]; then
    if [[ "${HYBRID_VR}" == "1" && -f "${ROOT}/config/vr_ready_pose.yaml" ]]; then
      START_POSE_YAML="${ROOT}/config/vr_ready_pose.yaml"
    elif [[ -f "${ROOT}/config/pick_place.yaml" ]]; then
      START_POSE_YAML="${ROOT}/config/pick_place.yaml"
    fi
  fi
  if [[ "${HYBRID_VR}" == "1" && "${WITH_VUER}" != "1" ]]; then
    echo "[warn] --hybrid-vr without --with-vuer: start Vuer separately or arms will not move"
  fi
  if [[ "${HYBRID_VR}" == "1" && -n "${START_POSE_YAML}" ]]; then
    echo "[hybrid-vr] Start pose: ${START_POSE_YAML} (match this arm pose in VR before X+A)"
  fi

  if [[ "${WITHOUT_RELAY}" != "1" ]]; then
    start_proc relay "${base_env} && export ROBOTIS_START_POSE_YAML=\"${START_POSE_YAML}\" && ros2 launch ffw_vuer_dds_relay relay.launch.py"
  fi
  start_proc udp_bridge "${base_env} && /usr/bin/python3.12 \"${ROOT}/scripts/joint_targets_udp_bridge.py\" $(udp_bridge_args)"

  if [[ "${WITH_VUER}" == "1" ]]; then
    local vuer_env=""
    if [[ "${VUER_INSECURE}" == "1" ]]; then
      vuer_env="export ROBOTIS_VUER_INSECURE=1 && "
    fi
    if [[ "${VUER_DISABLE_SQUEEZE_GATE}" == "1" ]]; then
      vuer_env="${vuer_env}export ROBOTIS_VUER_DISABLE_SQUEEZE_GATE=1 && "
    fi
    : > "${LOG_DIR}/quest.log"
    start_proc vuer "${base_env} && source \"${ROOT}/robotis_applications/install/setup.bash\" && ${vuer_env}ros2 launch robotis_vuer vr.launch.py model:=sg2"
    start_proc quest_log "\"${ROOT}/scripts/quest_log.sh\""
  fi

  if [[ "${WITH_LOOP_TEST}" == "1" ]]; then
    start_proc loop_test "${base_env} && /usr/bin/python3.12 \"${ROOT}/scripts/loop_joint_test.py\" --hz \"${LOOP_HZ}\""
  fi

  if [[ "${WITH_ISAAC}" == "1" ]]; then
    if [[ -z "${USD_PATH}" ]]; then
      USD_PATH="$(default_usd_path)"
    fi
    if [[ -z "${USD_PATH}" ]]; then
      echo "Error: --with-isaac requires --usd-path (install scenes/newscene assets or scenes/Scene_clean.usda)"
      exit 1
    fi
    local soft_shirts_flag=""
    if [[ "${SOFT_SHIRTS}" == "1" ]]; then
      soft_shirts_flag="--soft-shirts"
    else
      soft_shirts_flag="--no-soft-shirts"
    fi
    local grasp_test_cube_flag=""
    local grasp_mode_flag=""
    if [[ "${GRASP_TEST_CUBE}" == "1" ]]; then
      grasp_test_cube_flag="--grasp-test-cube"
    fi
    ensure_grasp_mode_for_isaac
    if [[ -n "${GRASP_MODE}" ]]; then
      grasp_mode_flag="--grasp-mode ${GRASP_MODE}"
    fi
    local isaac_grasp_flags
    isaac_grasp_flags="$(isaac_grasp_flags)"
    local isaac_physics_grasp_flag
    isaac_physics_grasp_flag="$(physics_grasp_flag)"
    local isaac_start_flags
    isaac_start_flags="$(isaac_start_pose_flags)"
    start_proc isaac "
      $(isaac_env_preamble)
      cd \"${ISAAC_ROOT}\" &&
      ./python.sh \"${ROOT}/isaac_sim/standalone_ffw_joint_teleop.py\" \
        --usd_path \"${USD_PATH}\" \
        --spawn_prim \"${SPAWN_PRIM}\" \
        --articulation_prim \"${ARTICULATION_PRIM}\" \
        --input_mode udp \
        --udp_host 127.0.0.1 \
        --udp_port 15000 \
        --command_smoothing 0.2 \
        --base_move_prim \"${BASE_MOVE_PRIM}\" \
        ${isaac_start_flags} \
        ${soft_shirts_flag} \
        ${grasp_test_cube_flag} \
        ${grasp_mode_flag} \
        ${isaac_grasp_flags} \
        ${isaac_physics_grasp_flag}
    "
  fi

  if [[ "${PICK_PLACE}" == "1" ]]; then
    echo ""
    echo "Pick-place (YAML, no VR): after Isaac loads, run:"
    echo "  /usr/bin/python3.12 \"${ROOT}/scripts/publish_pose_from_yaml.py\" \\"
    echo "    --config \"${ROOT}/config/pick_place.yaml\""
    if [[ "${PHYSICS_GRASP}" == "1" ]]; then
      echo ""
      echo "Physics grasp: shirt is a dynamic rigid body; close gripper on contact (no attach weld)."
      echo "Investigation log (JSONL): ${ROOT}/logs/physics_grasp/latest.jsonl"
      echo "After a run, summarize with:"
      echo "  /usr/bin/python3.12 \"${ROOT}/scripts/analyze_physics_grasp_log.py\""
      echo "Web tuner (arms/base + grasp attach offset sliders):"
      echo "  /usr/bin/python3.12 \"${ROOT}/scripts/joint_pose_web_tuner.py\" \\"
      echo "    --config \"${ROOT}/config/pick_place.yaml\" --physics-grasp"
    fi
  fi
  if [[ "${HYBRID_VR}" == "1" ]]; then
    echo ""
    echo "Hybrid VR: Quest controls arms; run body sequence in another terminal:"
    echo "  /usr/bin/python3.12 \"${ROOT}/scripts/publish_pose_from_yaml.py\" \\"
    echo "    --config \"${ROOT}/config/pick_place.yaml\" --hybrid-body"
    if [[ "${VR_SHIRT_GRASP}" == "1" ]]; then
      echo ""
      echo "VR shirt grasp: squeeze trigger when gripper overlaps ${VR_SHIRT_PRIM}; release trigger to drop."
    fi
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
      --without-relay) WITHOUT_RELAY=1; shift ;;
      --soft-shirts) SOFT_SHIRTS=1; shift ;;
      --no-soft-shirts) SOFT_SHIRTS=0; shift ;;
      --grasp-test-cube) GRASP_TEST_CUBE=1; shift ;;
      --grasp-mode) GRASP_MODE="$2"; shift 2 ;;
      --pick-place) PICK_PLACE=1; WITHOUT_RELAY=1; SOFT_SHIRTS=0; PHYSICS_GRASP=1; shift ;;
      --physics-grasp) PHYSICS_GRASP=1; SOFT_SHIRTS=0; shift ;;
      --hybrid-vr) HYBRID_VR=1; VUER_DISABLE_SQUEEZE_GATE=1; shift ;;
      --vr-shirt-grasp) VR_SHIRT_GRASP=1; SOFT_SHIRTS=0; shift ;;
      --vr-shirt-prim) VR_SHIRT_PRIM="$2"; shift 2 ;;
      --vr-shirt-root) VR_SHIRT_ROOT="$2"; shift 2 ;;
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

  local base_env="source /opt/ros/jazzy/setup.bash && source \"${ROOT}/ros2_ws/install/setup.bash\" && source \"${ROOT}/env_local.bash\" && export ROS_DOMAIN_ID=\"${DOMAIN_ID}\" && export ROBOTIS_VR_ISAAC_ROOT=\"${ROOT}\""

  if [[ "${HYBRID_VR}" == "1" && "${WITHOUT_RELAY}" == "1" ]]; then
    echo "Error: --hybrid-vr requires the VR relay (do not use --without-relay)"
    exit 1
  fi
  if [[ -z "${START_POSE_YAML}" ]]; then
    if [[ "${HYBRID_VR}" == "1" && -f "${ROOT}/config/vr_ready_pose.yaml" ]]; then
      START_POSE_YAML="${ROOT}/config/vr_ready_pose.yaml"
    elif [[ -f "${ROOT}/config/pick_place.yaml" ]]; then
      START_POSE_YAML="${ROOT}/config/pick_place.yaml"
    fi
  fi
  if [[ "${WITHOUT_RELAY}" != "1" ]]; then
    launch_ui_proc "${term_bin}" "relay" "${base_env} && ros2 launch ffw_vuer_dds_relay relay.launch.py"
  fi
  launch_ui_proc "${term_bin}" "udp_bridge" "${base_env} && /usr/bin/python3.12 \"${ROOT}/scripts/joint_targets_udp_bridge.py\" $(udp_bridge_args)"

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
      USD_PATH="$(default_usd_path)"
    fi
    if [[ -z "${USD_PATH}" ]]; then
      echo "Error: --with-isaac requires --usd-path (install scenes/newscene assets or scenes/Scene_clean.usda)"
      exit 1
    fi
    local soft_shirts_flag=""
    if [[ "${SOFT_SHIRTS}" == "1" ]]; then
      soft_shirts_flag="--soft-shirts"
    else
      soft_shirts_flag="--no-soft-shirts"
    fi
    local grasp_test_cube_flag=""
    local grasp_mode_flag=""
    if [[ "${GRASP_TEST_CUBE}" == "1" ]]; then
      grasp_test_cube_flag="--grasp-test-cube"
    fi
    ensure_grasp_mode_for_isaac
    if [[ -n "${GRASP_MODE}" ]]; then
      grasp_mode_flag="--grasp-mode ${GRASP_MODE}"
    fi
    local isaac_grasp_flags
    isaac_grasp_flags="$(isaac_grasp_flags)"
    local isaac_physics_grasp_flag
    isaac_physics_grasp_flag="$(physics_grasp_flag)"
    local isaac_start_flags
    isaac_start_flags="$(isaac_start_pose_flags)"
    launch_ui_proc "${term_bin}" "isaac" "$(isaac_env_preamble) cd \"${ISAAC_ROOT}\" && ./python.sh \"${ROOT}/isaac_sim/standalone_ffw_joint_teleop.py\" --usd_path \"${USD_PATH}\" --spawn_prim \"${SPAWN_PRIM}\" --articulation_prim \"${ARTICULATION_PRIM}\" --base_move_prim \"${BASE_MOVE_PRIM}\" --input_mode udp --udp_host 127.0.0.1 --udp_port 15000 --command_smoothing 0.2 ${isaac_start_flags} ${soft_shirts_flag} ${grasp_test_cube_flag} ${grasp_mode_flag} ${isaac_grasp_flags} ${isaac_physics_grasp_flag}"
  fi
  if [[ "${HYBRID_VR}" == "1" ]]; then
    echo ""
    echo "Hybrid VR: run body sequence:"
    echo "  /usr/bin/python3.12 \"${ROOT}/scripts/publish_pose_from_yaml.py\" --config \"${ROOT}/config/pick_place.yaml\" --hybrid-body"
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
      --without-relay) WITHOUT_RELAY=1; shift ;;
      --soft-shirts) SOFT_SHIRTS=1; shift ;;
      --no-soft-shirts) SOFT_SHIRTS=0; shift ;;
      --grasp-test-cube) GRASP_TEST_CUBE=1; shift ;;
      --grasp-mode) GRASP_MODE="$2"; shift 2 ;;
      --pick-place) PICK_PLACE=1; WITHOUT_RELAY=1; SOFT_SHIRTS=0; PHYSICS_GRASP=1; shift ;;
      --physics-grasp) PHYSICS_GRASP=1; SOFT_SHIRTS=0; shift ;;
      --hybrid-vr) HYBRID_VR=1; VUER_DISABLE_SQUEEZE_GATE=1; shift ;;
      --vr-shirt-grasp) VR_SHIRT_GRASP=1; SOFT_SHIRTS=0; shift ;;
      --vr-shirt-prim) VR_SHIRT_PRIM="$2"; shift 2 ;;
      --vr-shirt-root) VR_SHIRT_ROOT="$2"; shift 2 ;;
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

  local base_env="source /opt/ros/jazzy/setup.bash && source \"${ROOT}/ros2_ws/install/setup.bash\" && source \"${ROOT}/env_local.bash\" && export ROS_DOMAIN_ID=\"${DOMAIN_ID}\" && export ROBOTIS_VR_ISAAC_ROOT=\"${ROOT}\""

  if [[ "${HYBRID_VR}" == "1" && "${WITHOUT_RELAY}" == "1" ]]; then
    echo "Error: --hybrid-vr requires the VR relay (do not use --without-relay)"
    exit 1
  fi
  if [[ -z "${START_POSE_YAML}" ]]; then
    if [[ "${HYBRID_VR}" == "1" && -f "${ROOT}/config/vr_ready_pose.yaml" ]]; then
      START_POSE_YAML="${ROOT}/config/vr_ready_pose.yaml"
    elif [[ -f "${ROOT}/config/pick_place.yaml" ]]; then
      START_POSE_YAML="${ROOT}/config/pick_place.yaml"
    fi
  fi

  if [[ "${WITHOUT_RELAY}" != "1" ]]; then
    tmux send-keys -t "${session}:relay" "${base_env} && ros2 launch ffw_vuer_dds_relay relay.launch.py" C-m
  else
    tmux send-keys -t "${session}:relay" "echo relay skipped (--without-relay); exec bash" C-m
  fi

  tmux new-window -t "${session}" -n udp_bridge
  tmux send-keys -t "${session}:udp_bridge" "${base_env} && /usr/bin/python3.12 \"${ROOT}/scripts/joint_targets_udp_bridge.py\" $(udp_bridge_args)" C-m

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
      USD_PATH="$(default_usd_path)"
    fi
    if [[ -z "${USD_PATH}" ]]; then
      echo "Error: --with-isaac requires --usd-path (install scenes/newscene assets or scenes/Scene_clean.usda)"
      exit 1
    fi
    local soft_shirts_flag=""
    if [[ "${SOFT_SHIRTS}" == "1" ]]; then
      soft_shirts_flag="--soft-shirts"
    else
      soft_shirts_flag="--no-soft-shirts"
    fi
    local grasp_test_cube_flag=""
    local grasp_mode_flag=""
    if [[ "${GRASP_TEST_CUBE}" == "1" ]]; then
      grasp_test_cube_flag="--grasp-test-cube"
    fi
    ensure_grasp_mode_for_isaac
    if [[ -n "${GRASP_MODE}" ]]; then
      grasp_mode_flag="--grasp-mode ${GRASP_MODE}"
    fi
    local isaac_grasp_flags
    isaac_grasp_flags="$(isaac_grasp_flags)"
    local isaac_physics_grasp_flag
    isaac_physics_grasp_flag="$(physics_grasp_flag)"
    local isaac_start_flags
    isaac_start_flags="$(isaac_start_pose_flags)"
    tmux new-window -t "${session}" -n isaac
    tmux send-keys -t "${session}:isaac" "$(isaac_env_preamble) cd \"${ISAAC_ROOT}\" && ./python.sh \"${ROOT}/isaac_sim/standalone_ffw_joint_teleop.py\" --usd_path \"${USD_PATH}\" --spawn_prim \"${SPAWN_PRIM}\" --articulation_prim \"${ARTICULATION_PRIM}\" --base_move_prim \"${BASE_MOVE_PRIM}\" --input_mode udp --udp_host 127.0.0.1 --udp_port 15000 --command_smoothing 0.2 ${isaac_start_flags} ${soft_shirts_flag} ${grasp_test_cube_flag} ${grasp_mode_flag} ${isaac_grasp_flags} ${isaac_physics_grasp_flag}" C-m
  fi

  echo "tmux session started: ${session}"
  echo "Attach with: tmux attach -t ${session}"
  if [[ "${HYBRID_VR}" == "1" ]]; then
    echo "Hybrid VR: run body sequence in another terminal with publish_pose_from_yaml.py --hybrid-body"
  fi
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
  stop_proc quest_log
  stop_proc vuer
  stop_proc udp_bridge
  stop_proc relay

  # Orphan children survive stop_proc (launch/python grandchildren).
  pkill -f "[s]tandalone_ffw_joint_teleop.py" 2>/dev/null || true
  pkill -f "[v]r_publisher_sg2" 2>/dev/null || true
  pkill -f "ros2 launch robotis_vuer" 2>/dev/null || true
  pkill -f "ros2 launch ffw_vuer_dds_relay" 2>/dev/null || true
  pkill -f "vuer_dds_relay_node" 2>/dev/null || true
  pkill -f "joint_targets_udp_bridge.py" 2>/dev/null || true
  pkill -f "${ROOT}/scripts/quest_log.sh" 2>/dev/null || true
}

cmd_status() {
  status_proc hw_all_in_one
  status_proc hw_follower
  status_proc hw_leader
  status_proc relay
  status_proc udp_bridge
  status_proc vuer
  status_proc quest_log
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
