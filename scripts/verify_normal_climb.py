"""Headless small-curb and independent stair-mode regression.

    .venv/bin/python scripts/verify_normal_climb.py
    .venv/bin/python scripts/verify_normal_climb.py --baseline-library /tmp/before.dylib

The baseline comparison checks every sampled pose, actuator command and FSM state
on 150/200 mm stairs. Normal uses the same terrain probe as the native viewer.
"""

import argparse
import ctypes
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from mobed_sim_native import ActuatorOutputs, CommandInputs, NormalCurbProbe, SensorInputs


def run(library, height, mode=1, speed=0.25, duration=25.0, *, scene=None, keyframe="home"):
    library = Path(library).resolve()
    lib = ctypes.CDLL(str(library))
    lib.mobed_create.restype = ctypes.c_void_p
    lib.mobed_destroy.argtypes = [ctypes.c_void_p]
    has_terrain_entry = hasattr(lib, "mobed_update_spin_with_terrain")
    if mode == 1 and not has_terrain_entry:
        raise RuntimeError("Rebuild the library to test the normal terrain gate")
    update = lib.mobed_update_spin_with_terrain if has_terrain_entry else lib.mobed_update_spin
    update.restype = ctypes.c_int
    update.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(SensorInputs), ctypes.POINTER(CommandInputs),
        ctypes.c_double, ctypes.POINTER(ActuatorOutputs), ctypes.c_int,
    ]
    if has_terrain_entry:
        update.argtypes += [ctypes.c_int]
    if scene is None:
        model = make_test_step(height)
    else:
        model = mujoco.MjModel.from_xml_path(str(scene))
        model.opt.timestep = 0.002
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key(keyframe).id)
    mujoco.mj_forward(model, data)
    return simulate(lib, update, has_terrain_entry, library, model, data, height, mode, speed, duration)


def make_test_step(height):
    root = ET.parse(REPO / "models_standard/robox_v2/robox_v2.xml").getroot()
    root.find("compiler").set("meshdir", str(REPO / "models_standard/robox_v2/meshes"))
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", "0.002")
    if height > 0:
        ET.SubElement(root.find("worldbody"), "geom", name="test_step", type="box",
                      pos=f"10.8 0 {height / 2}", size=f"10 1 {height / 2}",
                      friction="1 0.005 0.0001")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def simulate(lib, update, has_terrain_entry, library, model, data, height, mode, speed, duration):
    joints = model.actuator_trnid[:, 0]
    qpos_address = model.jnt_qposadr[joints]
    dof_address = model.jnt_dofadr[joints]
    wheel_ids = [model.geom("wheel_col_" + name).id for name in ("LF", "RF", "LB", "RB")]
    chassis = model.body("chassis").id
    curb_probe = NormalCurbProbe(model, chassis) if mode == 1 else None
    controller = lib.mobed_create()
    assert controller, "failed to create controller"
    sensor, cmd, out = SensorInputs(), CommandInputs(), ActuatorOutputs()
    cmd.target_height = 0.205
    cmd.climb_mode = mode
    previous_velocity = np.zeros(3)
    previous_state = 0
    events, samples = [], []
    max_tilt = 0.0
    try:
        for step in range(round(duration / 0.002)):
            time = step * 0.002
            if step % 5 == 0:
                for prefix, start in (("steer", 0), ("ecc", 4), ("wheel", 8)):
                    for i in range(4):
                        getattr(sensor, prefix + "_pos")[i] = data.qpos[qpos_address[start + i]]
                        getattr(sensor, prefix + "_vel")[i] = data.qvel[dof_address[start + i]]
                        getattr(sensor, prefix + "_effort")[i] = data.actuator_force[start + i]
                rotation = data.xmat[chassis].reshape(3, 3)
                velocity = data.qvel[:3].copy()
                sensor.imu_accel[:] = rotation.T @ (
                    (velocity - previous_velocity) / 0.01 - np.array([0, 0, -9.81])
                )
                previous_velocity = velocity
                sensor.imu_gyro[:] = data.qvel[3:6]
                sensor.imu_quat[:] = data.qpos[3:7]
                cmd.vx = min(speed, max(0, (time - 4) * 0.5))
                args = (controller, ctypes.byref(sensor), ctypes.byref(cmd),
                        0.01, ctypes.byref(out), 0)
                if has_terrain_entry:
                    allowed = curb_probe is not None and cmd.vx >= 0.10 and curb_probe.allows_entry(data)
                    update(*args, int(allowed))
                else:
                    update(*args)
                data.ctrl[:4] = out.steer_pos
                data.ctrl[4:8] = out.ecc_pos
                data.ctrl[8:12] = out.wheel_vel
                if out.fsm_state != previous_state:
                    events.append([round(time, 2), out.fsm_state])
                    previous_state = out.fsm_state
                max_tilt = max(max_tilt, abs(np.arcsin(np.clip(-rotation[2, 0], -1, 1))))
                samples.append([time, *data.qpos[:7], *data.ctrl, out.fsm_state, *out.ecc_effort])
                assert np.isfinite(data.qpos).all(), "non-finite simulation state"
            mujoco.mj_step(model, data)
        wheel_x = data.geom_xpos[wheel_ids, 0]
        wheel_z = data.geom_xpos[wheel_ids, 2] - 0.1
        result = {
            "library": library.name, "height_mm": round(height * 1000), "mode": mode,
            "speed_mps": speed, "events": events, "final_x": round(float(data.qpos[0]), 3),
            "wheel_height_mm": (wheel_z * 1000).round(1).tolist(),
            "max_pitch_deg": round(float(np.rad2deg(max_tilt)), 2),
        }
        print(json.dumps(result), flush=True)
        return np.array(samples), result, wheel_x.copy(), wheel_z.copy()
    finally:
        lib.mobed_destroy(controller)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, default=REPO / "build_native/libmobed_control.dylib")
    parser.add_argument("--baseline-library", type=Path)
    args = parser.parse_args()
    cases = [(mm / 1000, 0.25) for mm in range(0, 81, 5)]
    cases += [(height, speed) for speed in (0.15, 0.4, 0.6) for height in (0.03, 0.06, 0.08)]
    for height, speed in cases:
        _, result, wheel_x, wheel_z = run(args.library, height, speed=speed, duration=30)
        assert (wheel_x > 0.9).all(), f"{height}: rear axle failed to clear the edge"
        assert np.max(np.abs(wheel_z - height)) < 0.004, f"{height}: wheels did not settle on platform"
        states = [event[1] for event in result["events"]]
        assert all(state == 0 or 20 <= state <= 24 for state in states), "normal entered a stair state"
        if states:
            assert states in ([20, 21, 22, 23, 24, 0], [21, 22, 23, 24, 0],
                              [20, 21, 23, 24, 0], [21, 23, 24, 0]), \
                f"{height}: normal climb aborted/incomplete: {states}"
            # RECOVER must reach its stance target, rather than exit by timeout.
            recovery = next(event[0] for event in result["events"] if event[1] == 24)
            assert result["events"][-1][0] - recovery < 10.0, "posture recovery timed out"
        if height == 0:
            assert not states, "normal climb falsely triggered on flat ground"
    for height in (0.081, 0.15, 0.20):
        _, result, wheel_x, _ = run(args.library, height, duration=12)
        assert not result["events"], f"normal attempted a {height} m climb"
        assert (wheel_x[:2] > 0.65).all(), "robot did not reach the obstacle"
    playground = REPO / "models_standard/playground/robot_playground.xml"
    trace, result, _, wheel_z = run(args.library, 0.20, duration=10,
                                    scene=playground, keyframe="rmuc_step")
    assert trace[-1, 2] > 3.15, "robot did not reach the playground step"
    assert not result["events"], "normal attempted the playground's 200 mm mesh step"
    assert np.max(wheel_z) < 0.02, "normal mounted the playground's high step"
    for height in (0.15, 0.20):
        current, result, wheel_x, wheel_z = run(args.library, height, mode=2, speed=0.6, duration=12)
        assert all(0 <= event[1] <= 7 for event in result["events"]), "stair mode entered a normal state"
        assert (wheel_x > 0.9).all() and np.max(np.abs(wheel_z - height)) < 0.004
        if args.baseline_library:
            baseline, _, _, _ = run(args.baseline_library, height, mode=2, speed=0.6, duration=12)
            np.testing.assert_array_equal(current, baseline, err_msg="stair behavior changed")
    print("Small-curb coverage and independent stair checks passed")


if __name__ == "__main__":
    main()
