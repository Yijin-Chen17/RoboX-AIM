"""Open the RMUC 2027 field or its ROBOX V2 scene."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import mujoco

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robox', action='store_true', help='Add the ROBOX V2 robot in RMUC 2027')
    parser.add_argument('--physics', action='store_true', help='Discard display meshes; show collision shapes')
    parser.add_argument('--headless', action='store_true', help='Run without a window, then print state')
    parser.add_argument('--duration', type=float, help='Stop after this many simulated seconds')
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error('--duration must be positive')
    if args.robox:
        entry = ROOT / ('robot_rmuc2027_physics.xml' if args.physics else 'robot_rmuc2027.xml')
    else:
        entry = ROOT / ('scene_physics.xml' if args.physics else 'scene.xml')
    model = mujoco.MjModel.from_xml_path(str(entry))
    data = mujoco.MjData(model)

    def reset():
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)

    def advance(target):
        while data.time < target:
            mujoco.mj_step(model, data)

    reset()
    duration = args.duration or (20. if args.headless else float('inf'))
    if args.headless:
        advance(duration)
    else:
        from mujoco import viewer as mj_viewer
        paused = False
        reset_requested = False

        def key_callback(key):
            nonlocal paused, reset_requested
            if key == 32:
                paused = not paused
            elif key in (ord('R'), ord('r')):
                reset_requested = True

        with mj_viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
            viewer.opt.geomgroup[3] = bool(args.physics)
            viewer.opt.geomgroup[1] = not args.physics
            viewer.cam.lookat[:] = [0, 0, .7]
            viewer.cam.distance = 34
            viewer.cam.azimuth = 90
            viewer.cam.elevation = -62
            if args.robox:
                chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
                if chassis_id >= 0:
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                    viewer.cam.trackbodyid = chassis_id
                    viewer.cam.distance = 3.5
                    viewer.cam.azimuth = 135
                    viewer.cam.elevation = -25
            print('Space: pause/resume. R: reset. Close the window to exit.', flush=True)
            while viewer.is_running() and data.time < duration:
                frame_start = time.perf_counter()
                if reset_requested:
                    reset()
                    reset_requested = False
                if not paused:
                    advance(min(data.time + 1/60, duration))
                viewer.sync()
                time.sleep(max(0., 1/60 - (time.perf_counter() - frame_start)))
    warnings = data.warning.number.astype(int).tolist()
    print(json.dumps({'simulated_s': float(data.time),
                      'physics_only': args.physics, 'ngeom': model.ngeom,
                      'contacts': int(data.ncon), 'warnings': warnings}), flush=True)
    if sum(warnings):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
