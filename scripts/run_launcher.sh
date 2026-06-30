#!/usr/bin/env bash
# Open the pick-place web launcher (Isaac + YAML + tuner controls).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec /usr/bin/python3.12 "${ROOT}/scripts/pick_place_launcher.py" "$@"
