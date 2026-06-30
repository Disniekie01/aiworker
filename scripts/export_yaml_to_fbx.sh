#!/usr/bin/env bash
# Sample pick_place.yaml and export skeletal FBX via Blender.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${ROOT}/config/pick_place.yaml"
URDF="${ROOT}/assets/ffw_bg2_follower_nomesh.urdf"
EXPORT_DIR="${ROOT}/exports"
ANIM_JSON="${EXPORT_DIR}/pick_place_animation.json"
OUT_FBX="${EXPORT_DIR}/pick_place.fbx"

find_blender() {
  if [[ -n "${BLENDER_BIN:-}" ]]; then
    echo "${BLENDER_BIN}"
    return
  fi
  # Prefer Flatpak when installed (avoids broken/partial apt blender + PYTHONPATH issues).
  if command -v flatpak >/dev/null 2>&1 && flatpak list 2>/dev/null | grep -q "org.blender.Blender"; then
    echo "flatpak:org.blender.Blender"
    return
  fi
  if command -v blender >/dev/null 2>&1; then
    echo "$(command -v blender)"
    return
  fi
  for c in /usr/bin/blender /snap/bin/blender /var/lib/flatpak/exports/bin/blender; do
    if [[ -x "${c}" ]]; then
      echo "${c}"
      return
    fi
  done
  echo ""
}

run_blender() {
  local bin="$1"
  shift
  # Blender ships its own Python; strip venv/conda paths that break FBX export (numpy).
  env -u PYTHONPATH -u PYTHONHOME -u VIRTUAL_ENV -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
    -u CONDA_SHLVL -u CONDA_PROMPT_MODIFIER \
    bash -lc '
      bin="$1"
      shift
      if [[ "${bin}" == flatpak:* ]]; then
        flatpak run "${bin#flatpak:}" "$@"
      else
        "${bin}" "$@"
      fi
    ' _ "${bin}" "$@"
}

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Sample pick_place.yaml and export FBX animation for Blender.

Options:
  --config <yaml>     Input YAML (default: config/pick_place.yaml)
  --urdf <path>       Robot URDF (default: assets/ffw_bg2_follower_nomesh.urdf)
  --output <fbx>      Output FBX path (default: exports/pick_place.fbx)
  --json-only         Only write animation JSON, skip Blender
  --open              After export, open FBX in Blender GUI
  --blender <path>    Blender binary (or set BLENDER_BIN)

Install Blender (if missing):
  # Pop!_OS / Ubuntu apt (if dependencies are healthy):
  sudo apt update && sudo apt install -y blender
  # Or Flatpak (often works when apt is broken):
  flatpak install -y flathub org.blender.Blender

EOF
}

JSON_ONLY=0
OPEN_BLENDER=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) CONFIG="$2"; shift 2 ;;
    --urdf) URDF="$2"; shift 2 ;;
    --output) OUT_FBX="$2"; shift 2 ;;
    --json-only) JSON_ONLY=1; shift ;;
    --open) OPEN_BLENDER=1; shift ;;
    --blender) BLENDER_BIN="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 1 ;;
  esac
done

ANIM_JSON="$(dirname "${OUT_FBX}")/$(basename "${OUT_FBX}" .fbx)_animation.json"
mkdir -p "$(dirname "${OUT_FBX}")"

echo "[1/2] Sampling YAML -> ${ANIM_JSON}"
python3 "${ROOT}/scripts/yaml_animation_sampler.py" \
  --config "${CONFIG}" \
  --output "${ANIM_JSON}"

if [[ "${JSON_ONLY}" == "1" ]]; then
  echo "Done (JSON only)."
  exit 0
fi

BLENDER="$(find_blender)"
if [[ -z "${BLENDER}" ]]; then
  echo ""
  echo "Blender not found. Options:"
  echo "  flatpak install -y flathub org.blender.Blender"
  echo "  sudo apt --fix-broken install && sudo apt install -y blender"
  echo ""
  echo "Then re-run this script, or set BLENDER_BIN=/path/to/blender"
  echo "Animation JSON is ready at: ${ANIM_JSON}"
  exit 1
fi

echo "[2/2] Blender export -> ${OUT_FBX}"
echo "Using: ${BLENDER}"
run_blender "${BLENDER}" --background --python "${ROOT}/scripts/blender_export_yaml_fbx.py" -- \
  --animation "${ANIM_JSON}" \
  --urdf "${URDF}" \
  --output "${OUT_FBX}"

if [[ ! -s "${OUT_FBX}" ]]; then
  echo "ERROR: FBX export failed (missing or empty: ${OUT_FBX})" >&2
  exit 1
fi

echo "Done."
echo "  JSON: ${ANIM_JSON}"
echo "  FBX:  ${OUT_FBX}"

if [[ "${OPEN_BLENDER}" == "1" ]]; then
  echo "Opening in Blender..."
  run_blender "${BLENDER}" --python "${ROOT}/scripts/blender_open_fbx.py" -- "${OUT_FBX}" &
fi
