#!/usr/bin/env bash
# Install Python dependencies for this project into .venv
# Run once from the repo root: bash install_python_deps.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="${REPO_ROOT}/.venv"
CYCLONEDDS_HOME="${REPO_ROOT}/deps/cyclonedds-install"

# ── Checks ──────────────────────────────────────────────────────────────────
if [[ ! -f "${CYCLONEDDS_HOME}/lib/libddsc.so" ]]; then
  echo "ERROR: libddsc.so not found under ${CYCLONEDDS_HOME}/lib"
  echo "  Build CycloneDDS C first (see INSTALL.txt), then re-run this script."
  exit 1
fi

if [[ ! -d "${VENV}" ]]; then
  echo "Creating venv at ${VENV} ..."
  python3.12 -m venv "${VENV}"
fi

# ── Install ──────────────────────────────────────────────────────────────────
echo "Installing cyclonedds==0.10.2 (CYCLONEDDS_HOME=${CYCLONEDDS_HOME}) ..."
CYCLONEDDS_HOME="${CYCLONEDDS_HOME}" "${VENV}/bin/pip" install cyclonedds==0.10.2

DDS_PY="${REPO_ROOT}/third_party/robotis_dds_python"
if [[ ! -d "${DDS_PY}" ]]; then
  echo "ERROR: ${DDS_PY} not found."
  echo "  Clone it first: git clone https://github.com/ROBOTIS-GIT/robotis_dds_python.git third_party/robotis_dds_python"
  exit 1
fi
echo "Installing robotis_dds_python (editable) ..."
"${VENV}/bin/pip" install -e "${DDS_PY}"

# ── Verify ───────────────────────────────────────────────────────────────────
echo "Verifying imports ..."
CYCLONEDDS_HOME="${CYCLONEDDS_HOME}" LD_LIBRARY_PATH="${CYCLONEDDS_HOME}/lib:${LD_LIBRARY_PATH:-}" \
  "${VENV}/bin/python" -c "import cyclonedds; import robotis_dds_python; print('OK: cyclonedds and robotis_dds_python imported successfully')"

echo ""
echo "Done. In each terminal that runs the relay, source:"
echo "  source \"${REPO_ROOT}/env_local.bash\""
