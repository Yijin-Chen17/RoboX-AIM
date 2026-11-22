#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$SIM_DIR/.." && pwd)"
ROBOX_CONTROL_DIR="$REPO_ROOT/RoboX_ws/src/chassis_controllers"
OUT_DIR="$SIM_DIR/build_native"

# Target output directory
mkdir -p "$OUT_DIR"

echo "==> 正在编译 MobED 原生 C++ 控制算法动态库 (libmobed_control.dylib)..."

clang++ -O3 -std=c++17 -shared -fPIC \
  -I "$SIM_DIR/native_core/include" \
  -I "$ROBOX_CONTROL_DIR/include" \
  -I /opt/homebrew/include/eigen3 \
  -I /opt/homebrew/opt/eigen/include/eigen3 \
  "$SIM_DIR/native_core/src/mobed_controller_core.cpp" \
  "$ROBOX_CONTROL_DIR/src/utils/robox/kinematics/mobed_kinematics.cpp" \
  "$ROBOX_CONTROL_DIR/src/utils/robox/controllers/driving_controller.cpp" \
  "$ROBOX_CONTROL_DIR/src/utils/robox/controllers/balance_controller.cpp" \
  "$ROBOX_CONTROL_DIR/src/utils/robox/dynamics/srbd_model.cpp" \
  "$ROBOX_CONTROL_DIR/src/utils/robox/dynamics/grf_optimizer.cpp" \
  "$ROBOX_CONTROL_DIR/src/utils/robox/estimation/state_estimator.cpp" \
  "$ROBOX_CONTROL_DIR/src/utils/robox/estimation/terrain_estimator.cpp" \
  "$ROBOX_CONTROL_DIR/src/utils/robox/estimation/contact_detector.cpp" \
  "$ROBOX_CONTROL_DIR/src/utils/robox/estimation/climb_geometry_estimator.cpp" \
  "$ROBOX_CONTROL_DIR/src/utils/robox/tasks/normal_climb_controller.cpp" \
  "$ROBOX_CONTROL_DIR/src/utils/robox/tasks/task_controller.cpp" \
  "$ROBOX_CONTROL_DIR/src/utils/robox/tasks/trajectory_generator.cpp" \
  -o "$OUT_DIR/libmobed_control.dylib"

echo "==> 编译完成: $OUT_DIR/libmobed_control.dylib"
ls -lh "$OUT_DIR/libmobed_control.dylib"
