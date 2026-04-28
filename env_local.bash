# Local (no-sudo) Cyclone DDS + venv for robotis_dds_python / relay.
# Usage after building ros2_ws:
#   source /opt/ros/jazzy/setup.bash
#   source /path/to/robotis_vr_isaac/ros2_ws/install/setup.bash
#   source /path/to/robotis_vr_isaac/env_local.bash

_ROBOTIS_VR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export CYCLONEDDS_HOME="${_ROBOTIS_VR_ROOT}/deps/cyclonedds-install"
export LD_LIBRARY_PATH="${CYCLONEDDS_HOME}/lib:${LD_LIBRARY_PATH:-}"
_VENV_SITE=""
for _d in "${_ROBOTIS_VR_ROOT}/.venv/lib"/python*/site-packages; do
  if [[ -d "${_d}" ]]; then _VENV_SITE="${_d}"; break; fi
done
if [[ -z "${_VENV_SITE}" ]]; then
  echo "env_local.bash: no .venv site-packages; run: python3.12 -m venv ${_ROBOTIS_VR_ROOT}/.venv" >&2
fi
_DDS_PY="${_ROBOTIS_VR_ROOT}/../robotis_lab/third_party/robotis_dds_python"
export PYTHONPATH="${_DDS_PY}:${_VENV_SITE}:${PYTHONPATH:-}"
