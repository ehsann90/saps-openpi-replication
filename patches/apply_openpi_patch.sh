#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OPENPI_DIR="$PROJECT_ROOT/third_party/openpi"
PATCH_FILE="$PROJECT_ROOT/patches/openpi-libero-python38-build.patch"

if [[ ! -d "$OPENPI_DIR/.git" && ! -f "$OPENPI_DIR/.git" ]]; then
    echo "OpenPI submodule is missing."
    echo "Run: git submodule update --init --recursive"
    exit 1
fi

if git -C "$OPENPI_DIR" apply --check "$PATCH_FILE" 2>/dev/null; then
    git -C "$OPENPI_DIR" apply "$PATCH_FILE"
    echo "Applied OpenPI LIBERO Python 3.8 compatibility patch."
elif git -C "$OPENPI_DIR" apply --reverse --check "$PATCH_FILE" 2>/dev/null; then
    echo "OpenPI compatibility patch is already applied."
else
    echo "Patch cannot be applied cleanly to the current OpenPI revision."
    exit 1
fi
