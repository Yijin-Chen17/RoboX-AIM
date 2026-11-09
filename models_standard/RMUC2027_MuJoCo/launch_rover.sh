#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$DIR/../.." && pwd)"

if [ -f "$DIR/.venv/bin/python3" ]; then
    PYTHON="$DIR/.venv/bin/python3"
elif [ -f "$ROOT_DIR/.venv/bin/python3" ]; then
    PYTHON="$ROOT_DIR/.venv/bin/python3"
else
    PYTHON="python3"
fi

cd "$DIR"
exec "$PYTHON" view_scene.py --rover --route road_red "$@"
