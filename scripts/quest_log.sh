#!/usr/bin/env bash
# Monitor Quest / Vuer connection signals -> .run_logs/quest.log
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT}/.run_logs"
LOG="${LOG_DIR}/quest.log"
VUER_LOG="${LOG_DIR}/vuer.log"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

mkdir -p "${LOG_DIR}"
exec >>"${LOG}" 2>&1

echo "=== quest monitor started $(date -Is) ROS_DOMAIN_ID=${DOMAIN_ID} ==="

for _ in $(seq 1 60); do
  if [[ -f "${VUER_LOG}" ]]; then
    url="$(rg -m1 "Network:" "${VUER_LOG}" 2>/dev/null | sed 's/.*Network://' | sed 's/\x1b\[[0-9;]*m//g' | tr -d '[:space:]' || true)"
    if [[ -n "${url}" ]]; then
      echo "QUEST_URL: ${url}"
      break
    fi
  fi
  sleep 1
done

tail_vuer() {
  [[ -f "${VUER_LOG}" ]] || touch "${VUER_LOG}"
  tail -n 0 -F "${VUER_LOG}" 2>/dev/null | while IFS= read -r line; do
    if echo "${line}" | rg -qi \
      'websocket|connected|disconnect|Network:|Controller data received|Starting controller|controller/body tracking|Error in VR|address already|Errno|VR Trajectory server|motion-controller'; then
      echo "$(date -Is) [vuer] ${line}"
    fi
  done
}

tail_vuer &
TAIL_PID=$!
trap 'kill "${TAIL_PID}" 2>/dev/null || true; echo "=== quest monitor stopped $(date -Is) ==="; exit 0' TERM INT

base_env="source /opt/ros/jazzy/setup.bash && source \"${ROOT}/ros2_ws/install/setup.bash\" && source \"${ROOT}/env_local.bash\" && export ROS_DOMAIN_ID=\"${DOMAIN_ID}\""

last_summary=""
while true; do
  conns="$(ss -tnH sport = :8012 2>/dev/null | wc -l | tr -d ' ')"
  estabs="$(ss -tnH state established sport = :8012 2>/dev/null | wc -l | tr -d ' ')"

  squeeze_line="$(bash -lc "${base_env} && timeout 2 ros2 topic hz /vr_controller/left_squeeze 2>/dev/null" | tail -n 1 || true)"
  goal_line="$(bash -lc "${base_env} && timeout 2 ros2 topic hz /l_goal_pose 2>/dev/null" | tail -n 1 || true)"

  if [[ -n "${squeeze_line}" ]]; then
    squeeze="hz ${squeeze_line}"
  else
    squeeze="no /vr_controller/left_squeeze msgs (2s)"
  fi

  if [[ -n "${goal_line}" ]]; then
    goal="hz ${goal_line}"
  else
    goal="no /l_goal_pose msgs (2s)"
  fi

  summary="tcp_listeners_or_pending=${conns} established=${estabs} | ${squeeze} | ${goal}"
  if [[ "${summary}" != "${last_summary}" ]]; then
    echo "$(date -Is) [status] ${summary}"
    if [[ "${estabs}" != "0" ]]; then
      echo "$(date -Is) [quest] TCP connection on :8012 (headset browser likely connected)"
    fi
    last_summary="${summary}"
  fi

  sleep 5
done
