#!/usr/bin/env bash
# Maintainer tool: rebuild scenes/newscene/ binaries from external source files.
# Normal installs use Git LFS: git lfs pull && ./verify_assets.sh
#
# Requires all NEWSCENE_* paths (see manifest.json). No machine-local defaults.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${ROOT}/../.." && pwd)"
MODE="${1:-copy}"

log() { echo "[collect] $*"; }

if [[ -z "${NEWSCENE_ROBOT_USD:-}" || -z "${NEWSCENE_AUSTRIA_DIR:-}" \
   || -z "${NEWSCENE_CRATE_FBX:-}" || -z "${NEWSCENE_SHIRT_USDZ_DIR:-}" ]]; then
  cat >&2 <<EOF
collect_assets.sh is for maintainers rebuilding scene meshes from source files.

On a fresh clone, use Git LFS instead:
  git lfs install
  git lfs pull
  ./verify_assets.sh

To rebuild manually, set all of:
  NEWSCENE_ROBOT_USD       FFW SG2 Scene.usda
  NEWSCENE_AUSTRIA_DIR     folder with SceneRobot.usd
  NEWSCENE_CRATE_FBX       KB3D crate FBX
  NEWSCENE_SHIRT_USDZ_DIR  extracted shirt USDZ folder (contains scene.usdc)

See manifest.json
EOF
  exit 1
fi

ROBOT_SRC="${NEWSCENE_ROBOT_USD}"
AUSTRIA_SRC="${NEWSCENE_AUSTRIA_DIR}"
CRATE_SRC="${NEWSCENE_CRATE_FBX}"
SHIRT_SRC="${NEWSCENE_SHIRT_USDZ_DIR}"

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

for need in "${AUSTRIA_SRC}/SceneRobot.usd" "${CRATE_SRC}" "${SHIRT_SRC}/scene.usdc" "${ROBOT_SRC}"; do
  if [[ ! -e "${need}" ]]; then
    echo "Missing: ${need}" >&2
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

log "done — run ./verify_assets.sh and commit LFS objects if changed"
