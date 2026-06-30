#!/usr/bin/env bash
# Create a portable tarball of all binary scene assets (~1 GB).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIST="${ROOT}/dist"
STAMP="$(date +%Y%m%d)"
ARCHIVE="${DIST}/newscene-assets-${STAMP}.tar.gz"

mkdir -p "${DIST}"

echo "[package] collecting assets (copy mode)..."
"${ROOT}/collect_assets.sh" copy

echo "[package] verifying..."
"${ROOT}/verify_assets.sh"

echo "[package] archiving..."
tar -czf "${ARCHIVE}" -C "${ROOT}" \
  Scene.usda \
  newscene.usda \
  Scene_clean.usda \
  assets

ls -lh "${ARCHIVE}"
echo ""
echo "Transfer to another machine:"
echo "  scp ${ARCHIVE} user@host:/path/to/aiworker/scenes/newscene/dist/"
echo "  ssh user@host 'cd aiworker/scenes/newscene && ./install_assets.sh dist/$(basename "${ARCHIVE}")'"
