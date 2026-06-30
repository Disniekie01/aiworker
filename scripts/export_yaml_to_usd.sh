#!/usr/bin/env bash
# Sample pick_place.yaml and bake animation into a USD layer (Isaac Sim / Omniverse).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${ROOT}/config/pick_place.yaml"
SCENE="${SCENE_USD:-/home/disniekie/Robotis/Scene_clean.usda}"
EXPORT_DIR="${ROOT}/exports"
ANIM_JSON="${EXPORT_DIR}/pick_place_animation.json"
OUT_USD="${EXPORT_DIR}/pick_place_anim.usda"

find_isaac_python() {
  if [[ -n "${ISAAC_PYTHON:-}" ]]; then
    echo "${ISAAC_PYTHON}"
    return
  fi
  for c in "${HOME}/isaacsim/python.sh" "/home/disniekie/isaacsim/python.sh"; do
    if [[ -x "${c}" ]]; then
      echo "${c}"
      return
    fi
  done
  echo ""
}

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Sample pick_place.yaml and export an animated USD layer for Isaac Sim.

The output .usda references your scene as a subLayer and time-samples:
  - joint drive:angular/linear:physics:targetPosition
  - /World/Robot base translate + yaw
  - shirt mesh local xform (if present in scene)

Options:
  --config <yaml>       Input YAML (default: config/pick_place.yaml)
  --scene <usd>         Base scene (default: Scene_clean.usda)
  --output <usd>        Output animation layer (default: exports/pick_place_anim.usda)
  --json-only           Only write animation JSON, skip USD bake
  --flatten             Also write a flattened .usdc (large)
  --skip-base           Keep robot base fixed (no world-spin effect from base yaw)
  --isaac-python <path> Isaac Sim python.sh (or set ISAAC_PYTHON)

Play in Isaac Sim:
  Open the output .usda, press Play on the timeline (30 fps).

EOF
}

JSON_ONLY=0
FLATTEN=0
SKIP_BASE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) CONFIG="$2"; shift 2 ;;
    --scene) SCENE="$2"; shift 2 ;;
    --output) OUT_USD="$2"; shift 2 ;;
    --json-only) JSON_ONLY=1; shift ;;
    --flatten) FLATTEN=1; shift ;;
    --skip-base) SKIP_BASE=1; shift ;;
    --isaac-python) ISAAC_PYTHON="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 1 ;;
  esac
done

ANIM_JSON="${EXPORT_DIR}/pick_place_animation.json"
mkdir -p "$(dirname "${OUT_USD}")"

echo "[1/2] Sampling YAML -> ${ANIM_JSON}"
python3 "${ROOT}/scripts/yaml_animation_sampler.py" \
  --config "${CONFIG}" \
  --output "${ANIM_JSON}"

if [[ "${JSON_ONLY}" == "1" ]]; then
  echo "Done (JSON only)."
  exit 0
fi

ISAAC_PY="$(find_isaac_python)"
if [[ -z "${ISAAC_PY}" ]]; then
  echo ""
  echo "Isaac Sim python.sh not found. Set ISAAC_PYTHON=/path/to/isaacsim/python.sh"
  echo "Animation JSON is ready at: ${ANIM_JSON}"
  exit 1
fi

ISAAC_ROOT="$(dirname "${ISAAC_PY}")"
echo "[2/2] USD bake -> ${OUT_USD}"
echo "Using: ${ISAAC_PY}"

FLATTEN_FLAG=()
if [[ "${FLATTEN}" == "1" ]]; then
  FLATTEN_FLAG=(--flatten)
fi
SKIP_BASE_FLAG=()
if [[ "${SKIP_BASE}" == "1" ]]; then
  SKIP_BASE_FLAG=(--skip-base)
fi

# Isaac pxr is available after SimulationApp starts — use the headless exporter.
env -u PYTHONPATH -u PYTHONHOME -u VIRTUAL_ENV \
  "${ISAAC_PY}" "${ROOT}/isaac_sim/export_yaml_animation_usd.py" \
  --animation "${ANIM_JSON}" \
  --scene "${SCENE}" \
  --output "${OUT_USD}" \
  "${FLATTEN_FLAG[@]}" \
  "${SKIP_BASE_FLAG[@]}"

if [[ ! -s "${OUT_USD}" ]]; then
  echo "ERROR: USD export failed (missing or empty: ${OUT_USD})" >&2
  exit 1
fi

echo "Done."
echo "  JSON: ${ANIM_JSON}"
echo "  USD:  ${OUT_USD}"
echo ""
echo "Open in Isaac Sim and press Play on the timeline."
