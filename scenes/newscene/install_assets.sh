#!/usr/bin/env bash
# Install scene binaries from package_assets.sh tarball on a new machine.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -ge 1 ]]; then
  ARCHIVE="$1"
else
  ARCHIVE=""
fi
if [[ -n "${ARCHIVE}" && ! -f "${ARCHIVE}" ]]; then
  ARCHIVE="${ROOT}/${ARCHIVE}"
fi
if [[ -z "${ARCHIVE}" || ! -f "${ARCHIVE}" ]]; then
  ARCHIVE="$(ls -t "${ROOT}"/dist/newscene-assets-*.tar.gz 2>/dev/null | head -1 || true)"
fi
if [[ -z "${ARCHIVE}" || ! -f "${ARCHIVE}" ]]; then
  echo "Archive not found." >&2
  echo "Usage: $0 path/to/newscene-assets.tar.gz" >&2
  echo "  or place dist/newscene-assets-*.tar.gz and run without arguments" >&2
  exit 1
fi

echo "[install] extracting ${ARCHIVE}..."
rm -rf "${ROOT}/assets" "${ROOT}/Scene.usda"
tar -xzf "${ARCHIVE}" -C "${ROOT}"

"${ROOT}/verify_assets.sh"
echo "[install] ready — use scenes/newscene/newscene.usda"
