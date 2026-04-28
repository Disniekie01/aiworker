#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

echo "[setup] Syncing submodule metadata..."
git submodule sync --recursive

echo "[setup] Initializing/updating submodules..."
git submodule update --init --recursive

echo "[setup] Done."
