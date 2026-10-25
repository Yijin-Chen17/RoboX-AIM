# Standard Simulation Models (models_standard)

This directory contains the standard, calibrated, and active models for the MobED / ROBOX V2 simulation environment:

1. **`robox_v2/`**: Standard ROBOX V2 robot model (Flat Ground)
   - `robox_v2.xml`: Fully calibrated 30 kg robot model with accurate chassis/wheel collision geoms, bottom slider rods, mass properties, and 12-DOF actuators (steering, posture, driving).
   - `meshes/`: Visual OBJ meshes (`part_01.obj` - `part_13.obj`).

2. **`rmuc2027/`**: RoboMaster University Championship 2027 Arena
   - `robot_rmuc2027_lite.xml`: High-performance 120 FPS battlefield simulation with ROBOX V2 robot.
   - `rmuc2027_arena_lite.xml`: Arena environment with analytically decomposed convex collision hulls (28x floor, ring road, center gantry, ramps, dart stations).
   - `robot_rmuc2027.xml` / `rmuc2027_arena.xml`: Full lighting arena configuration.
   - `meshes_lite/`: Decimated visual STL meshes and calibrated collision hull assets.

3. **`playground/`**: Obstacle & Terrain Testbench
   - `robot_playground.xml`: Multi-obstacle terrain testbench (5cm/10cm/20cm curbs, stairs, ramps, steps) with ROBOX V2 robot.
   - `playground_arena.xml`: Analytical geometric collision definitions for all playground terrain obstacles.

> Note: All models use direct relative paths to shared assets (`robox_v2/meshes`). No symbolic links are used.
