#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCENE="${ROOT}/newscene.usda"
MANIFEST="${ROOT}/manifest.json"

if [[ ! -f "${SCENE}" ]]; then
  echo "Missing ${SCENE}" >&2
  exit 1
fi

missing=0
while IFS= read -r rel; do
  rel="${rel#\"}"
  rel="${rel%\"}"
  target="${ROOT}/${rel}"
  if [[ -e "${target}" ]]; then
    echo "[ok]   ${rel}"
  else
    echo "[MISS] ${rel}"
    missing=$((missing + 1))
  fi
done < <(python3 - "${MANIFEST}" <<'PY'
import json, sys
data = json.load(open(sys.argv[1]))
for p in data["required_paths"] + data.get("optional_paths", []):
    print(p)
PY
)

while IFS= read -r ref; do
  rel="${ref#@}"
  rel="${rel%@}"
  target="${ROOT}/${rel#./}"
  if [[ -e "${target}" ]]; then
    echo "[ok]   ${rel} (usd ref)"
  else
    echo "[MISS] ${rel} (usd ref)"
    missing=$((missing + 1))
  fi
done < <(rg -o '@[^@]+@' "${SCENE}" | sort -u)

if [[ "${missing}" -gt 0 ]]; then
  echo "" >&2
  echo "${missing} missing. Run: git lfs pull" >&2
  exit 1
fi
echo "All assets present."
