# Standard Simulation Models (models_standard)

This directory contains the standard, calibrated, and active models for the MobED / ROBOX V2 simulation environment:

1. **`robox_v2/`**: Standard ROBOX V2 robot model (Flat Ground)
   - `robox_v2.xml`: Fully calibrated 30 kg robot model with accurate chassis/wheel collision geoms, bottom slider rods, mass properties, and 12-DOF actuators (steering, posture, driving).
   - `meshes/`: Visual OBJ meshes (`part_01.obj` - `part_13.obj`).

2. **`RMUC2027_MuJoCo/`**: RoboMaster University Championship 2027 Arena (Official V3 Approved Scene)
   - `robot_rmuc2027.xml`: Full lighting arena configuration with ROBOX V2 robot.
   - `robot_rmuc2027_physics.xml`: High-performance collision-only simulation with ROBOX V2 robot (`discardvisual="true"`).
   - `field.xml`: Complete RMUC 2027 static arena collision hulls (1,365 geoms) and visual assets.
   - `scene.xml` / `scene_physics.xml`: Standalone arena view configurations.
   - `launch_robox.sh` / `.cmd`: Quick launchers for interactive ROBOX V2 simulation.

3. **`playground/`**: Obstacle & Terrain Testbench
   - `robot_playground.xml`: Multi-obstacle terrain testbench (5cm/10cm/20cm curbs, RMUC 2027 central highland 200mm+150mm two-step stair, ramps, bumps) with ROBOX V2 robot.
   - `playground_arena.xml`: Analytical and CAD-derived geometric collision definitions for all playground terrain obstacles.

> Note: All models use direct relative paths to shared assets (`robox_v2/meshes`). No symbolic links are used.
