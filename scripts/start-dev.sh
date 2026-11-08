#!/bin/bash
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker rm -f mobed-container 2>/dev/null; \
docker run --rm -it \
  --name mobed-container \
  -v "${PROJECT_DIR}":/workspace \
  ros2-dev \
  /bin/bash
