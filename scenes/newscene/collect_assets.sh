#!/usr/bin/env bash
# Bundle external assets into scenes/newscene/ with relative USD paths.
# Usage: ./collect_assets.sh [link|copy]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${ROOT}/../.." && pwd)"
MODE="${1:-link}"

log() { echo "[collect] $*"; }

expand_path() {
  local p="$1"
  p="${p//\$\{ROBOTIS_VR_ROOT\}/${REPO}}"
  p="${p//\$\{HOME\}/${HOME}}"
  printf '%s' "${p}"
}

copy_or_link() {
  local src="$1" dst="$2"
  mkdir -p "$(dirname "${dst}")"
  if [[ -e "${dst}" && ! -L "${dst}" ]]; then
    log "skip (exists): ${dst}"
    return
  fi
  rm -f "${dst}" 2>/dev/null || true
  if [[ "${MODE}" == "copy" ]]; then
    if [[ -d "${src}" ]]; then
      cp -a "${src}" "${dst}"
    else
      cp -a "${src}" "${dst}"
    fi
    log "copied: ${dst}"
  else
    ln -sf "${src}" "${dst}"
    log "linked: ${dst} -> ${src}"
  fi
}

ROBOT_SRC="${NEWSCENE_ROBOT_USD:-${REPO}/../Scene.usda}"
AUSTRIA_SRC="${NEWSCENE_AUSTRIA_DIR:-$(expand_path '${HOME}/Downloads/Austria/Austria')}"
CRATE_SRC="${NEWSCENE_CRATE_FBX:-$(expand_path '${HOME}/Downloads/KB3D_CTS_Crate_A.fbx')}"
SHIRT_SRC="${NEWSCENE_SHIRT_USDZ_DIR:-$(expand_path '${HOME}/Downloads/Folded_Shirt_lowpoly_game_asset (Copy 1).usdz')}"

for need in "${AUSTRIA_SRC}/SceneRobot.usd" "${CRATE_SRC}" "${SHIRT_SRC}/scene.usdc" "${ROBOT_SRC}"; do
  if [[ ! -e "${need}" ]]; then
    echo "Missing: ${need}" >&2
    echo "Set NEWSCENE_* env vars or see manifest.json" >&2
    exit 1
  fi
done

log "mode=${MODE}"

copy_or_link "${ROBOT_SRC}" "${ROOT}/Scene.usda"
copy_or_link "${AUSTRIA_SRC}" "${ROOT}/assets/environment/Austria"
copy_or_link "${CRATE_SRC}" "${ROOT}/assets/crate/KB3D_CTS_Crate_A.fbx"
if [[ -f "$(dirname "${CRATE_SRC}")/KB3D_CTS_Crate_A.mtl" ]]; then
  copy_or_link "$(dirname "${CRATE_SRC}")/KB3D_CTS_Crate_A.mtl" "${ROOT}/assets/crate/KB3D_CTS_Crate_A.mtl"
fi
copy_or_link "${SHIRT_SRC}/scene.usdc" "${ROOT}/assets/shirt/scene.usdc"
if [[ -d "${SHIRT_SRC}/0" ]]; then
  copy_or_link "${SHIRT_SRC}/0" "${ROOT}/assets/shirt/0"
fi

TEMPLATE="${ROOT}/newscene.usda"
if [[ ! -f "${TEMPLATE}" ]]; then
  echo "Missing ${TEMPLATE} (should be in git)" >&2
  exit 1
fi

python3 - "${TEMPLATE}" <<'PY'
import pathlib
import re
import sys

path = pathlib.Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
replacements = [
    (r"@\.\./Downloads/Austria/Austria/SceneRobot\.usd@", "@./assets/environment/Austria/SceneRobot.usd@"),
    (r"@\.\./Downloads/KB3D_CTS_Crate_A\.fbx@", "@./assets/crate/KB3D_CTS_Crate_A.fbx@"),
    (r"@\.\./Downloads/Folded_Shirt_lowpoly_game_asset \(Copy 1\)\.usdz/scene\.usdc@", "@./assets/shirt/scene.usdc@"),
    (r"@\./Scene\.usda@", "@./Scene.usda@"),
]
for pattern, repl in replacements:
    text = re.sub(pattern, repl, text)
path.write_text(text, encoding="utf-8")
print(f"[collect] ensured relative paths in {path}")
PY

cp "${TEMPLATE}" "${ROOT}/Scene_clean.usda"

log "done — entry: ${ROOT}/newscene.usda"
log "verify: ./verify_assets.sh"
