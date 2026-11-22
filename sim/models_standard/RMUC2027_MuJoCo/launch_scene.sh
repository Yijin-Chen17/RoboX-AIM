#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd "$DIR/../.." && pwd)"
REPO_ROOT="$(cd "$SIM_DIR/.." && pwd)"

if [ -f "$DIR/.venv/bin/python3" ]; then
    PYTHON="$DIR/.venv/bin/python3"
elif [ -f "$SIM_DIR/.venv/bin/python3" ]; then
    PYTHON="$SIM_DIR/.venv/bin/python3"
elif [ -f "$REPO_ROOT/.venv/bin/python3" ]; then
    PYTHON="$REPO_ROOT/.venv/bin/python3"
else
    PYTHON="python3"
fi

cd "$DIR"
exec "$PYTHON" view_scene.py "$@"
