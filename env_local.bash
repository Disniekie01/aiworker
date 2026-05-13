# Local (no-sudo) Cyclone DDS + venv for robotis_dds_python / relay.
# Usage after building ros2_ws:
#   source /opt/ros/jazzy/setup.bash
#   source /path/to/robotis_vr_isaac/ros2_ws/install/setup.bash
#   source /path/to/robotis_vr_isaac/env_local.bash
#
# robotis_dds_python: clone into third_party/ (see third_party/README.md)
#   or keep robotis_lab next to this repo. Cyclone C libs: build into deps/
#   (deps/ is gitignored — see INSTALL.txt).

_ROBOTIS_VR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export CYCLONEDDS_HOME="${_ROBOTIS_VR_ROOT}/deps/cyclonedds-install"
export LD_LIBRARY_PATH="${CYCLONEDDS_HOME}/lib:${LD_LIBRARY_PATH:-}"

if ! compgen -G "${CYCLONEDDS_HOME}/lib/libddsc.so"* >/dev/null 2>&1; then
  echo "env_local.bash: no libddsc under ${CYCLONEDDS_HOME}/lib — build Cyclone DDS C here or install cyclonedds-dev (see INSTALL.txt)." >&2
fi

_VENV_SITE=""
for _d in "${_ROBOTIS_VR_ROOT}/.venv/lib"/python*/site-packages; do
  if [[ -d "${_d}" ]]; then _VENV_SITE="${_d}"; break; fi
done
if [[ -z "${_VENV_SITE}" ]]; then
  echo "env_local.bash: no .venv site-packages; run: python3.12 -m venv ${_ROBOTIS_VR_ROOT}/.venv" >&2
fi

_DDS_PY=""
if [[ -d "${_ROBOTIS_VR_ROOT}/third_party/robotis_dds_python" ]]; then
  _DDS_PY="${_ROBOTIS_VR_ROOT}/third_party/robotis_dds_python"
elif [[ -d "${_ROBOTIS_VR_ROOT}/../robotis_lab/third_party/robotis_dds_python" ]]; then
  _DDS_PY="${_ROBOTIS_VR_ROOT}/../robotis_lab/third_party/robotis_dds_python"
else
  echo "env_local.bash: robotis_dds_python not found. Clone it: see third_party/README.md" >&2
fi

if [[ -n "${_DDS_PY}" ]]; then
  export PYTHONPATH="${_DDS_PY}:${_VENV_SITE}:${PYTHONPATH:-}"
else
  export PYTHONPATH="${_VENV_SITE}:${PYTHONPATH:-}"
fi
