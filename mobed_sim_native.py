#!/usr/bin/env python3
"""
MobED Native 120 FPS Simulation Runner (Direct GLFW + MuJoCo Rendering)
========================================================================
Runs high-fidelity physics + hardware-accelerated 3D rendering directly via GLFW.
Links in-memory to libmobed_control.dylib via ctypes (0 ms latency, 0 packet loss).

Key Advantages:
- True Simultaneous Multi-Key Holding: Hardware-level polling via glfw.get_key().
  Holding [W + A] or [W + D] simultaneously is 100% supported with 0 ms latency!
- ZERO MuJoCo Hotkey Conflicts: Bypasses MuJoCo's built-in Simulate GUI hotkeys.
  [Space] is a pure smooth brake and NEVER freezes physics; [A/E/J] NEVER trigger UI badges.
- 100% Clean 3D Viewport: Pure 3D simulation with zero intrusive text boxes or debug clutter.
- Real-time mouse orbit (left drag), pan (right drag), and zoom (scroll).

Usage:
    ./.venv/bin/python3 mobed_sim_native.py [--arena rmuc|playground|flat] [--climb on|off]
"""

import os
import sys
import time
import argparse
import ctypes
import numpy as np
import glfw
import mujoco

# Import RoboMaster teleoperation planner
from teleop_commander import RoboMasterTeleopCommander


# -----------------------------------------------------------------------------
# C-Types Definitions matching mobed_controller_core.hpp
# -----------------------------------------------------------------------------
class SensorInputs(ctypes.Structure):
    _fields_ = [
        ('imu_accel',    ctypes.c_double * 3),
        ('imu_gyro',     ctypes.c_double * 3),
        ('imu_quat',     ctypes.c_double * 4),
        ('steer_pos',    ctypes.c_double * 4),
        ('steer_vel',    ctypes.c_double * 4),
        ('steer_effort', ctypes.c_double * 4),
        ('ecc_pos',      ctypes.c_double * 4),
        ('ecc_vel',      ctypes.c_double * 4),
        ('ecc_effort',   ctypes.c_double * 4),
        ('wheel_pos',    ctypes.c_double * 4),
        ('wheel_vel',    ctypes.c_double * 4),
        ('wheel_effort', ctypes.c_double * 4),
    ]

class CommandInputs(ctypes.Structure):
    _fields_ = [
        ('vx',            ctypes.c_double),
        ('vy',            ctypes.c_double),
        ('wz',            ctypes.c_double),
        ('target_height', ctypes.c_double),
        ('target_roll',   ctypes.c_double),
        ('target_pitch',  ctypes.c_double),
        ('posture_mode',  ctypes.c_int),
        ('climb_mode',    ctypes.c_int),
        ('e_stop',        ctypes.c_int),
    ]

class ActuatorOutputs(ctypes.Structure):
    _fields_ = [
        ('steer_pos',     ctypes.c_double * 4),
        ('ecc_pos',       ctypes.c_double * 4),
        ('wheel_vel',     ctypes.c_double * 4),
        ('steer_effort',  ctypes.c_double * 4),
        ('ecc_effort',    ctypes.c_double * 4),
        ('wheel_effort',  ctypes.c_double * 4),
        ('fsm_state',     ctypes.c_int),
        ('bank_angle',    ctypes.c_double),
        ('terrain_pitch', ctypes.c_double),
        ('terrain_roll',  ctypes.c_double),
    ]


# -----------------------------------------------------------------------------
# Arena & Joint Constants
# -----------------------------------------------------------------------------
PLAYGROUND_XML        = 'models_standard/playground/robot_playground.xml'
RMUC2027_XML         = 'models_standard/RMUC2027_MuJoCo/robot_rmuc2027.xml'
RMUC2027_PHYSICS_XML = 'models_standard/RMUC2027_MuJoCo/robot_rmuc2027_physics.xml'
RMUC2027_LITE_XML    = RMUC2027_PHYSICS_XML
RMUC2027_HD_XML      = RMUC2027_XML
ROBOX_V2_XML          = 'models_standard/robox_v2/robox_v2.xml'
FLAT_XML              = ROBOX_V2_XML
DYLIB_PATH            = 'build_native/libmobed_control.dylib'

JOINT_NAMES = [
    'Steering_joint_LF', 'Steering_joint_RF', 'Steering_joint_LB', 'Steering_joint_RB',
    'Posture_control_joint_LF', 'Posture_control_joint_RF', 'Posture_control_joint_LB', 'Posture_control_joint_RB',
    'Wheel_joint_LF', 'Wheel_joint_RF', 'Wheel_joint_LB', 'Wheel_joint_RB',
]

FSM_STATE_NAMES = [
    "IDLE",
    "FRONT_VAULT_180",
    "STRADDLE_LEVELING",
    "REAR_BELLY_REST",
    "REAR_VAULT_LOOP",
    "POST_CLIMB_RESET",
    "CLIMBED_RUN",
]


# -----------------------------------------------------------------------------
# Interactive Mouse Camera Controller
# -----------------------------------------------------------------------------
class InteractiveCamera:
    """Standard orbit, pan, and zoom camera controller with chassis tracking."""
    def __init__(self, model, chassis_id):
        self.model = model
        self.chassis_id = chassis_id
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()

        # Hide Group 3 (collision meshes) by default to eliminate transparent overdraw
        self.opt.geomgroup[3] = 0

        # Configure initial chassis tracking camera
        if chassis_id != -1:
            self.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.cam.trackbodyid = chassis_id
            self.cam.distance = 2.8
            self.cam.elevation = -18.0
            self.cam.azimuth = 145.0

        self.last_x = 0.0
        self.last_y = 0.0
        self.button_left = False
        self.button_middle = False
        self.button_right = False

    def on_mouse_button(self, window, button, action, mods):
        self.button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        self.button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        self.button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)
        xpos, ypos = glfw.get_cursor_pos(window)
        self.last_x = xpos
        self.last_y = ypos

    def on_cursor_pos(self, window, xpos, ypos):
        dx = xpos - self.last_x
        dy = ypos - self.last_y
        self.last_x = xpos
        self.last_y = ypos

        if not (self.button_left or self.button_middle or self.button_right):
            return

        w, h = glfw.get_framebuffer_size(window)
        if h <= 0:
            return

        mod_shift = (glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS or
                     glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)

        if self.button_right:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_MOVE_V
        elif self.button_left:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM

        mujoco.mjv_moveCamera(self.model, action, dx / h, dy / h, self.cam)

    def on_scroll(self, window, xoffset, yoffset):
        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, self.cam)


# -----------------------------------------------------------------------------
# Hardware-Polled Simultaneous Keyboard State
# -----------------------------------------------------------------------------
def get_hardware_keyboard_state(window) -> dict:
    """Directly polls GLFW hardware key states. 100% supports multi-key simultaneous holding."""
    # Forward / Backward
    fwd = (glfw.get_key(window, glfw.KEY_W) == glfw.PRESS or
           glfw.get_key(window, glfw.KEY_UP) == glfw.PRESS)
    bwd = (glfw.get_key(window, glfw.KEY_S) == glfw.PRESS or
           glfw.get_key(window, glfw.KEY_DOWN) == glfw.PRESS)

    # Turn / Strafe Left / Right
    left = (glfw.get_key(window, glfw.KEY_A) == glfw.PRESS or
            glfw.get_key(window, glfw.KEY_LEFT) == glfw.PRESS)
    rite = (glfw.get_key(window, glfw.KEY_D) == glfw.PRESS or
            glfw.get_key(window, glfw.KEY_RIGHT) == glfw.PRESS)

    # In-place Spin
    spin_l = (glfw.get_key(window, glfw.KEY_Q) == glfw.PRESS)
    spin_r = (glfw.get_key(window, glfw.KEY_E) == glfw.PRESS)

    # Smooth Brake
    brake = (glfw.get_key(window, glfw.KEY_SPACE) == glfw.PRESS or
             glfw.get_key(window, glfw.KEY_X) == glfw.PRESS)

    return {
        'fwd': fwd, 'bwd': bwd, 'left': left, 'rite': rite,
        'spin_l': spin_l, 'spin_r': spin_r, 'brake': brake
    }


# -----------------------------------------------------------------------------
# Main Simulation Runner
# -----------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='MobED / ROBOX V2 120 FPS Direct GLFW MuJoCo Simulator')
    parser.add_argument('--arena', type=str, default='rmuc2027',
                        choices=['rmuc2027', 'rmuc2027_physics', 'rmuc2027_lite', 'rmuc2027_hd', 'robox', 'playground', 'flat'],
                        help='Arena scene: rmuc2027 (default: RMUC 2027 V3 + ROBOX V2), rmuc2027_physics/rmuc2027_lite (Fast Collision-only), robox/flat (ROBOX V2 Flat Ground), playground (ROBOX V2 Obstacle testbench)')
    parser.add_argument('--robot', type=str, default=None, choices=['robox', 'mobed'],
                        help='Robot model override: robox (ROBOX V2, 30kg) or mobed (legacy MobED). Auto-detected if omitted.')
    parser.add_argument('--keyframe', type=str, default='home',
                        help='Spawn location keyframe (rmuc2027: home/home_red, home_blue, central_passage, test_center, road_red, road_blue, ramp_red, ramp_blue, test_platform_top; playground: home, curb_5cm, curb_10cm, curb_20cm, plaza, ramp, bumps, rmuc_step)')
    parser.add_argument('--climb', type=str, default='normal', choices=['normal', 'stair', 'on', 'off'],
                        help='Climb mode: normal (Mode 1: Paper standard, default), stair (Mode 2: 200mm stair), off')
    parser.add_argument('--model', type=str, default=None, help='Custom MJCF model path')
    args = parser.parse_args()

    # 1. Select scene XML
    if args.model:
        xml_path = args.model
    elif args.arena in ['rmuc2027', 'rmuc2027_hd'] and os.path.exists(RMUC2027_XML):
        xml_path = RMUC2027_XML
    elif args.arena in ['rmuc2027_physics', 'rmuc2027_lite'] and os.path.exists(RMUC2027_PHYSICS_XML):
        xml_path = RMUC2027_PHYSICS_XML
    elif args.arena in ['robox', 'flat'] and os.path.exists(ROBOX_V2_XML):
        xml_path = ROBOX_V2_XML
    elif args.arena == 'playground' and os.path.exists(PLAYGROUND_XML):
        xml_path = PLAYGROUND_XML
    elif os.path.exists(RMUC2027_XML):
        xml_path = RMUC2027_XML
    elif os.path.exists(PLAYGROUND_XML):
        xml_path = PLAYGROUND_XML
    else:
        xml_path = ROBOX_V2_XML

    # Determine robot model type: 0 = ROBOX_V2, 1 = MOBED_LEGACY
    if args.robot == 'robox':
        robot_model_type = 0
    elif args.robot == 'mobed':
        robot_model_type = 1
    elif 'robox' in xml_path.lower() or 'rmuc2027' in xml_path.lower() or 'playground' in xml_path.lower():
        robot_model_type = 0
    else:
        robot_model_type = 1

    robot_name_str = "ROBOX V2 (30.00 kg Standard)" if robot_model_type == 0 else "MobED Legacy"

    # Climb FSM mode: default PAPER_NORMAL (1), STAIR_200MM (2), or DISABLED (0)
    if args.climb == 'off':
        climb_mode_val = 0
    elif args.climb == 'stair':
        climb_mode_val = 2
    else:
        climb_mode_val = 1

    print(f'==> Loading MuJoCo Model: {xml_path}')
    print(f'==> Robot Model Target:   {robot_name_str}')
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 2. Apply keyframe
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, args.keyframe)
    if key_id != -1:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        for i, name in enumerate(JOINT_NAMES[:8]):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1:
                data.ctrl[i] = float(data.qpos[model.jnt_qposadr[jid]])
        applied = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, key_id)
        print(f"==> Keyframe '{applied}' applied successfully.")
    else:
        if model.nkey > 0:
            avail_keys = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, k) for k in range(model.nkey)]
            print(f"==> Notice: Keyframe '{args.keyframe}' not found. Available: {avail_keys}. Using default initial pose.")
        else:
            print(f"==> Notice: Keyframe '{args.keyframe}' not found. Using default initial pose.")
    mujoco.mj_forward(model, data)

    # 3. Load native C++ algorithm dylib
    if not os.path.exists(DYLIB_PATH):
        raise FileNotFoundError(f"Native library {DYLIB_PATH} not found! Run scripts/build_native_dylib.sh first.")

    print(f'==> Loading MobED C++ Control Core from {DYLIB_PATH} ...')
    lib = ctypes.CDLL(DYLIB_PATH)
    lib.mobed_create.restype = ctypes.c_void_p
    lib.mobed_destroy.argtypes = [ctypes.c_void_p]
    lib.mobed_reset.argtypes = [ctypes.c_void_p]
    lib.mobed_set_robot_model.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.mobed_update_spin.restype = ctypes.c_int
    lib.mobed_update_spin.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(SensorInputs),
        ctypes.POINTER(CommandInputs),
        ctypes.c_double,
        ctypes.POINTER(ActuatorOutputs),
        ctypes.c_int,
    ]

    controller = lib.mobed_create()
    if not controller:
        raise RuntimeError("Failed to allocate MobedControllerCore instance!")
    lib.mobed_set_robot_model(controller, robot_model_type)
    lib.mobed_reset(controller)

    # 4. Initialize GLFW window on Main Thread
    if not glfw.init():
        raise RuntimeError("Failed to initialize GLFW!")

    # Disable heavy MSAA on Retina displays to eliminate pixel fill-rate bottleneck
    glfw.window_hint(glfw.SAMPLES, 0)

    window_width, window_height = 1280, 800
    window = glfw.create_window(
        window_width, window_height,
        f"MobED / ROBOX V2 Simulation — [{args.arena.upper()}] ({robot_name_str})",
        None, None
    )
    if not window:
        glfw.terminate()
        raise RuntimeError("Failed to create native GLFW window!")

    glfw.make_context_current(window)
    glfw.swap_interval(1)  # Enable VSync for rock-solid 60/120 FPS Metal sync

    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
    camera_ctrl = InteractiveCamera(model, chassis_id)

    # Register mouse callbacks
    glfw.set_mouse_button_callback(window, camera_ctrl.on_mouse_button)
    glfw.set_cursor_pos_callback(window, camera_ctrl.on_cursor_pos)
    glfw.set_scroll_callback(window, camera_ctrl.on_scroll)

    # Initialize MuJoCo visualization context
    scn = mujoco.MjvScene(model, maxgeom=max(4000, model.ngeom * 2))
    con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

    # 5. Initialize Teleop Commander
    teleop_commander = RoboMasterTeleopCommander(
        max_lin_vel=1.0,     # 1.0 m/s agile cruising
        max_strafe_vel=0.8,  # 0.8 m/s lateral strafe
        max_ang_vel=1.5,     # 1.5 rad/s turning
        spin_ang_vel=1.8     # 1.8 rad/s in-place spin
    )

    # Pre-allocate C structs
    s_in = SensorInputs()
    c_in = CommandInputs()
    c_in.target_height = 0.228 if robot_model_type == 0 else 0.205
    c_in.target_roll   = 0.0
    c_in.target_pitch  = 0.0
    c_in.posture_mode  = 0
    c_in.climb_mode    = climb_mode_val
    c_in.e_stop        = 0

    out = ActuatorOutputs()

    # Joint lookups
    joint_qpos_adrs = []
    joint_dof_adrs = []
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        joint_qpos_adrs.append(model.jnt_qposadr[jid])
        joint_dof_adrs.append(model.jnt_dofadr[jid])

    g_world = np.array([0.0, 0.0, -9.81])
    prev_lin_vel = np.array(data.qvel[0:3], dtype=np.float64)

    mode_str = "Mode 1: PAPER_NORMAL (Paper Standard)" if climb_mode_val == 1 else ("Mode 2: STAIR_200MM (Dedicated Stair)" if climb_mode_val == 2 else "Mode 0: DISABLED")

    print("\n" + "="*70)
    print("  MobED Direct GLFW Native Simulation Ready (100% Conflict-Free)")
    print("="*70)
    print("  Controls (Directly in 3D Window, Full Multi-Key Support):")
    print("    [W / S]       Drive Forward / Backward (1.0 m/s)")
    print("    [W + A / D]   Agile Driving Curve + Motorcycle Lean (SIMULTANEOUS!)")
    print("    [A / D]       Pure Lateral Strafe (when stopped, 391mm Wide Stance)")
    print("    [V / B]       Toggle continuous spin CCW / CW with field-relative WASD")
    print("    [Q / E]       In-place Spin CCW / CW (1.8 rad/s)")
    print("    [Space / X]   Brake / Soft Stop (Release keys to coast to stop)")
    print("    [H]           Toggle Collision Overlay (Group 3: ON / OFF)")
    print("    [L]           Lower chassis to lowest stance (eccentric arms highest)")
    print("    [T / G]       Power test: away / toward chassis (starts from L pose)")
    print("    [C]           Toggle Mode (Mode 1: Paper Normal <-> Mode 2: Stair 200mm)")
    print("    [1 / 2 / 0]   Direct Mode Select: [1] Paper Normal | [2] Stair 200mm | [0] Disabled")
    print("    Mouse Drag    Left: Rotate around car | Right: Pan | Scroll: Zoom")
    print(f"  Active Mode : {mode_str}")
    print("="*70 + "\n")

    # Mode tracking variables
    current_climb_mode = climb_mode_val
    prev_key_c = False
    prev_key_1 = False
    prev_key_2 = False
    prev_key_0 = False
    prev_key_h = False
    prev_key_l = False
    prev_key_t = False
    prev_key_g = False
    prev_spin_keys = (False, False)
    spin_request = 0
    spin_phase = 0

    # ---------------------------------------------------------------
    # Eccentric-arm power measurement test
    # T: move all four arms 180 deg physically away from the chassis
    # G: move all four arms 180 deg in the opposite direction
    # The signs account for the mirrored joint axes in the MJCF model.
    # ---------------------------------------------------------------
    ecc_test_active = False
    ecc_test_to_lowest = False
    ecc_test_pending_direction = 0
    ecc_lowest_reached = False
    ecc_pose_hold = False
    ecc_pose_hold_targets = np.zeros(4, dtype=np.float64)
    ecc_test_direction = 0
    ecc_test_elapsed = 0.0
    ecc_test_start = np.zeros(4, dtype=np.float64)
    ecc_lowest_targets = np.full(4, -np.pi, dtype=np.float64)
    ecc_test_max_power = np.zeros(12, dtype=np.float64)
    ecc_test_last_power = np.zeros(12, dtype=np.float64)
    live_joint_power = np.zeros(12, dtype=np.float64)
    live_electrical_power = np.zeros(12, dtype=np.float64)
    ecc_test_max_torque = np.zeros(12, dtype=np.float64)
    ecc_test_max_speed = np.zeros(12, dtype=np.float64)
    # FL, FR, RL, RR: same physical radial direction from the chassis.
    ecc_away_signs = np.array([1.0, -1.0, -1.0, 1.0], dtype=np.float64)
    ecc_test_duration = np.pi / 3.5
    # "Lowest" means the chassis lowest stance: the eccentric arms point
    # upward.  In the MJCF actuator coordinates that pose is q=0.
    ecc_lowest_angle = 0.0
    ecc_lowest_tolerance = 0.04

    # Motor electrical model from the supplied datasheets.  MuJoCo reports
    # joint-side torque and speed, so convert through the output reduction.
    # Electrical estimate is motor copper loss plus back-EMF power:
    #   I = |tau_motor| / Kt
    #   P_elec = I * (Ke*|omega_motor| + I*R_phase)
    # Driver and gearbox efficiencies are not specified, therefore they are
    # intentionally not invented or folded into this displayed estimate.
    # Joint order: steering / eccentric posture / drive wheel.
    # Motor order: DM-J4310P / DM-J4340P / MS9010 v3.
    motor_gear = np.array([10.0] * 4 + [40.0] * 4 + [1.0] * 4)
    motor_kt = np.array([3.5 / (4.9 * 10.0)] * 4 +
                        [12.0 / (4.11 * 40.0)] * 4 +
                        [0.54] * 4)
    motor_ke = np.array([
        60.0 / (2.0 * np.pi * (200.0 * 10.0 / 24.0)),
        60.0 / (2.0 * np.pi * (200.0 * 10.0 / 24.0)),
        60.0 / (2.0 * np.pi * (200.0 * 10.0 / 24.0)),
        60.0 / (2.0 * np.pi * (200.0 * 10.0 / 24.0)),
        60.0 / (2.0 * np.pi * (56.0 * 40.0 / 24.0)),
        60.0 / (2.0 * np.pi * (56.0 * 40.0 / 24.0)),
        60.0 / (2.0 * np.pi * (56.0 * 40.0 / 24.0)),
        60.0 / (2.0 * np.pi * (56.0 * 40.0 / 24.0)),
        60.0 / (2.0 * np.pi * 6.7),
        60.0 / (2.0 * np.pi * 6.7),
        60.0 / (2.0 * np.pi * 6.7),
        60.0 / (2.0 * np.pi * 6.7),
    ])
    motor_r = np.array([0.58] * 4 + [0.72] * 4 + [8.2] * 4)

    # Timing variables
    last_sim_time = time.perf_counter()
    sim_time_debt = 0.0
    last_hud_time = 0.0
    control_substep_timer = 0.0
    CONTROL_DT = 0.010  # 100 Hz control loop
    PHYSICS_DT = model.opt.timestep

    try:
        while not glfw.window_should_close(window):
            # Process window events (mouse, keyboard, window resize)
            glfw.poll_events()

            now_perf = time.perf_counter()
            dt_wall = min(now_perf - last_sim_time, 0.05)
            last_sim_time = now_perf
            sim_time_debt += dt_wall

            # Step physics in 1:1 real-time accumulator
            while sim_time_debt >= PHYSICS_DT:
                control_substep_timer += PHYSICS_DT

                # ---------------- 100 Hz Control Update ----------------
                if control_substep_timer >= CONTROL_DT:
                    control_substep_timer -= CONTROL_DT

                    # 1. Read joints
                    for i in range(4):
                        s_in.steer_pos[i]    = float(data.qpos[joint_qpos_adrs[i]])
                        s_in.steer_vel[i]    = float(data.qvel[joint_dof_adrs[i]])
                        s_in.steer_effort[i] = float(data.actuator_force[i])

                        s_in.ecc_pos[i]      = float(data.qpos[joint_qpos_adrs[4 + i]])
                        s_in.ecc_vel[i]      = float(data.qvel[joint_dof_adrs[4 + i]])
                        s_in.ecc_effort[i]   = float(data.actuator_force[4 + i])

                        s_in.wheel_pos[i]    = float(data.qpos[joint_qpos_adrs[8 + i]])
                        s_in.wheel_vel[i]    = float(data.qvel[joint_dof_adrs[8 + i]])
                        s_in.wheel_effort[i] = float(data.actuator_force[8 + i])

                    # 2. Read IMU (finite-difference linear acceleration + gyro + quaternion)
                    chassis_xmat = data.xmat[chassis_id].reshape(3, 3)
                    curr_lin_vel = np.array(data.qvel[0:3], dtype=np.float64)
                    acc_world = (curr_lin_vel - prev_lin_vel) / CONTROL_DT
                    prev_lin_vel = curr_lin_vel.copy()
                    specific_force_body = chassis_xmat.T @ (acc_world - g_world)

                    s_in.imu_quat[0] = float(data.qpos[3])
                    s_in.imu_quat[1] = float(data.qpos[4])
                    s_in.imu_quat[2] = float(data.qpos[5])
                    s_in.imu_quat[3] = float(data.qpos[6])

                    s_in.imu_gyro[0] = float(data.qvel[3])
                    s_in.imu_gyro[1] = float(data.qvel[4])
                    s_in.imu_gyro[2] = float(data.qvel[5])

                    s_in.imu_accel[0] = float(specific_force_body[0])
                    s_in.imu_accel[1] = float(specific_force_body[1])
                    s_in.imu_accel[2] = float(specific_force_body[2])

                    # 3. Poll multi-key hardware state (W + A simultaneous supported!)
                    key_state = get_hardware_keyboard_state(window)
                    # Mode toggle keys (edge detection)
                    key_c = (glfw.get_key(window, glfw.KEY_C) == glfw.PRESS)
                    key_1 = (glfw.get_key(window, glfw.KEY_1) == glfw.PRESS)
                    key_2 = (glfw.get_key(window, glfw.KEY_2) == glfw.PRESS)
                    key_0 = (glfw.get_key(window, glfw.KEY_0) == glfw.PRESS)
                    key_h = (glfw.get_key(window, glfw.KEY_H) == glfw.PRESS)
                    key_l = (glfw.get_key(window, glfw.KEY_L) == glfw.PRESS)
                    key_t = (glfw.get_key(window, glfw.KEY_T) == glfw.PRESS)
                    key_g = (glfw.get_key(window, glfw.KEY_G) == glfw.PRESS)
                    start_lowest = key_l and not prev_key_l
                    start_away_test = key_t and not prev_key_t
                    start_toward_test = key_g and not prev_key_g

                    if key_c and not prev_key_c:
                        current_climb_mode = 1 if current_climb_mode == 2 else 2
                    elif key_1 and not prev_key_1:
                        current_climb_mode = 1
                    elif key_2 and not prev_key_2:
                        current_climb_mode = 2
                    elif key_0 and not prev_key_0:
                        current_climb_mode = 0

                    if key_h and not prev_key_h:
                        camera_ctrl.opt.geomgroup[3] = 1 - camera_ctrl.opt.geomgroup[3]
                        col_state = "SHOWN" if camera_ctrl.opt.geomgroup[3] else "HIDDEN"
                        print(f"\n==> [H] Collision Overlay (Group 3): {col_state}")

                    prev_key_c = key_c
                    prev_key_1 = key_1
                    prev_key_2 = key_2
                    prev_key_0 = key_0
                    prev_key_h = key_h
                    prev_key_l = key_l
                    prev_key_t = key_t
                    prev_key_g = key_g

                    spin_keys = (
                        glfw.get_key(window, glfw.KEY_V) == glfw.PRESS,
                        glfw.get_key(window, glfw.KEY_B) == glfw.PRESS,
                    )
                    focused = bool(glfw.get_window_attrib(window, glfw.FOCUSED))
                    diagnostic = (ecc_test_active or ecc_test_to_lowest or
                                  start_lowest or start_away_test or start_toward_test)
                    if key_state['brake'] or not focused or current_climb_mode == 2 or diagnostic:
                        spin_request = 0
                    elif spin_keys in ((True, False), (False, True)) and not any(prev_spin_keys):
                        direction = 1 if spin_keys[0] else -1
                        spin_request = 0 if spin_request == direction else direction
                        if spin_request:
                            ecc_pose_hold = False
                    prev_spin_keys = spin_keys

                    if spin_request or spin_phase:
                        teleop_commander.reset()
                        c_in.vx = 0.20 * (int(key_state['fwd']) - int(key_state['bwd']))
                        c_in.vy = 0.20 * (int(key_state['left']) - int(key_state['rite']))
                        c_in.wz = 0.0
                    else:
                        if not focused:
                            key_state = {'brake': True}
                        c_in.vx, c_in.vy, c_in.wz = teleop_commander.update(key_state, CONTROL_DT)

                    c_in.climb_mode = current_climb_mode

                    # 4. Invoke native C++ control core
                    previous_spin_phase = spin_phase
                    spin_phase = lib.mobed_update_spin(
                        controller, ctypes.byref(s_in), ctypes.byref(c_in), CONTROL_DT,
                        ctypes.byref(out), spin_request)
                    if previous_spin_phase and not spin_phase:
                        spin_request = 0
                    if spin_phase:
                        start_lowest = start_away_test = start_toward_test = False

                    # Start a standalone 180-degree eccentric-arm power test.
                    # The test does not alter any controller/model parameter;
                    # it only replaces the eccentric position target while
                    # active so both directions use the same measured setup.
                    if start_lowest:
                        ecc_test_to_lowest = True
                        ecc_test_pending_direction = 0
                        ecc_test_active = False
                        ecc_pose_hold = False
                        # Use the nearest equivalent zero representation for
                        # each actuator, avoiding an unnecessary 2*pi jump.
                        ecc_lowest_targets[:] = (
                            2.0 * np.pi * np.round(
                                np.array([s_in.ecc_pos[i] for i in range(4)]) /
                                (2.0 * np.pi)))
                        print("\n==> Moving chassis to lowest stance (eccentric arms highest)")

                    if start_away_test or start_toward_test:
                        ecc_test_to_lowest = True
                        ecc_test_pending_direction = 1 if start_away_test else -1
                        ecc_test_active = False
                        ecc_pose_hold = False
                        ecc_lowest_targets[:] = (
                            2.0 * np.pi * np.round(
                                np.array([s_in.ecc_pos[i] for i in range(4)]) /
                                (2.0 * np.pi)))
                        label = "AWAY" if start_away_test else "TOWARD"
                        print(f"\n==> Preparing eccentric power test: {label}; moving to lowest chassis stance first")

                    if ecc_test_to_lowest:
                        # Use the measured joint positions as the approach
                        # state, then start the requested 180-degree test only
                        # after all four arms are physically at q=0 (up).
                        lowest_error = np.array([
                            np.arctan2(np.sin(s_in.ecc_pos[i] - ecc_lowest_targets[i]),
                                      np.cos(s_in.ecc_pos[i] - ecc_lowest_targets[i]))
                            for i in range(4)
                        ])
                        if np.max(np.abs(lowest_error)) <= ecc_lowest_tolerance:
                            ecc_test_to_lowest = False
                            ecc_lowest_reached = True
                            if ecc_test_pending_direction != 0:
                                ecc_test_active = True
                                ecc_test_direction = ecc_test_pending_direction
                                ecc_test_elapsed = 0.0
                                ecc_test_start[:] = ecc_lowest_targets
                                ecc_test_max_power.fill(0.0)
                                ecc_test_last_power.fill(0.0)
                                ecc_test_max_torque.fill(0.0)
                                ecc_test_max_speed.fill(0.0)
                                label = "AWAY" if ecc_test_direction > 0 else "TOWARD"
                                print(f"==> Eccentric power test started from lowest chassis stance: {label}")
                                ecc_test_pending_direction = 0
                            else:
                                ecc_pose_hold = True
                                ecc_pose_hold_targets[:] = ecc_lowest_targets

                    if ecc_test_to_lowest:
                        for i in range(4):
                            out.ecc_pos[i] = float(ecc_lowest_targets[i])

                    if ecc_test_active:
                        ecc_test_elapsed = min(ecc_test_duration,
                                               ecc_test_elapsed + CONTROL_DT)
                        progress = min(np.pi, 3.5 * ecc_test_elapsed)
                        # These are actuator-space directions.  They are
                        # mirrored because the MJCF hinge axes are mirrored;
                        # this makes all four wheels move radially in the
                        # same physical direction.
                        targets = ecc_test_start + ecc_test_direction * ecc_away_signs * progress
                        for i in range(4):
                            out.ecc_pos[i] = float(targets[i])

                    # Hold the standalone test pose after L/T/G completes.
                    # This is deliberately applied after the native controller
                    # update so the normal FSM cannot restore the old pose.
                    if ecc_pose_hold and not ecc_test_to_lowest and not ecc_test_active:
                        for i in range(4):
                            out.ecc_pos[i] = float(ecc_pose_hold_targets[i])

                    # 5. Apply commands to MuJoCo actuators
                    for i in range(4):
                        data.ctrl[i]     = out.steer_pos[i]
                        data.ctrl[4 + i] = out.ecc_pos[i]
                        data.ctrl[8 + i] = out.wheel_vel[i]

                # Step physics
                mujoco.mj_step(model, data)
                sim_time_debt -= PHYSICS_DT

                # Read instantaneous mechanical power directly from MuJoCo
                # on every physics step, independent of the test mode.
                joint_power = np.zeros(12, dtype=np.float64)
                for i in range(12):
                    omega = float(data.qvel[joint_dof_adrs[i]])
                    torque = float(data.actuator_force[i])
                    joint_power[i] = torque * omega
                live_joint_power[:] = joint_power
                joint_torque = np.abs(np.array(data.actuator_force[:12], dtype=np.float64))
                joint_speed = np.abs(np.array(
                    [data.qvel[joint_dof_adrs[i]] for i in range(12)],
                    dtype=np.float64))
                motor_torque = joint_torque / motor_gear
                motor_current = motor_torque / motor_kt
                motor_speed = joint_speed * motor_gear
                live_electrical_power[:] = motor_current * (
                    motor_ke * motor_speed + motor_current * motor_r)

                if ecc_test_active:
                    ecc_test_last_power[:] = live_electrical_power
                    ecc_test_max_power[:] = np.maximum(ecc_test_max_power,
                                                        np.abs(live_electrical_power))
                    ecc_test_max_torque[:] = np.maximum(
                        ecc_test_max_torque, np.abs(data.actuator_force[:12]))
                    ecc_test_max_speed[:] = np.maximum(
                        ecc_test_max_speed,
                        np.abs(data.qvel[np.array(joint_dof_adrs[:12])]))

                    if ecc_test_elapsed >= ecc_test_duration:
                        label = "AWAY" if ecc_test_direction > 0 else "TOWARD"
                        print(f"\n==> Eccentric power test complete: {label}")
                        for i, name in enumerate(JOINT_NAMES):
                            print(f"    {name:24s} max |P|={ecc_test_max_power[i]:8.3f} W  "
                                  f"max |tau|={ecc_test_max_torque[i]:7.3f} Nm  "
                                  f"max |omega|={ecc_test_max_speed[i]:7.3f} rad/s")
                        ecc_pose_hold_targets[:] = (
                            ecc_test_start +
                            ecc_test_direction * ecc_away_signs * np.pi)
                        ecc_pose_hold = True
                        ecc_test_active = False

            # ---------------- Render 3D Scene directly via OpenGL/Metal ----------------
            fb_w, fb_h = glfw.get_framebuffer_size(window)
            viewport = mujoco.MjrRect(0, 0, fb_w, fb_h)

            mujoco.mjv_updateScene(
                model, data, camera_ctrl.opt, None,
                camera_ctrl.cam, mujoco.mjtCatBit.mjCAT_ALL, scn
            )
            mujoco.mjr_render(viewport, scn, con)

            # In-window power dashboard.  Terminal telemetry is intentionally
            # kept separate; this overlay is visible in the graphical window.
            if True:
                test_label = ("RUNNING AWAY" if ecc_test_active and ecc_test_direction > 0 else
                              "RUNNING TOWARD" if ecc_test_active else
                              "MOVING TO LOWEST" if ecc_test_to_lowest else
                              "HOLD / LIVE")
                rows = [f"ELECTRICAL POWER EST. [{test_label}]",
                        "T: away  G: toward  L: chassis lowest / arms up (q=0)",
                        "joint       P_elec live / test max (W)"]
                short_names = ["S_FL", "S_FR", "S_RL", "S_RR",
                               "E_FL", "E_FR", "E_RL", "E_RR",
                               "W_FL", "W_FR", "W_RL", "W_RR"]
                rows.extend(
                    f"{short_names[i]:5s} {live_electrical_power[i]:+7.1f} / {ecc_test_max_power[i]:7.1f} W"
                    for i in range(12)
                )
                overlay_text = "\n".join(rows)
                mujoco.mjr_overlay(
                    mujoco.mjtFontScale.mjFONTSCALE_100,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    viewport, overlay_text, "", con
                )
            glfw.swap_buffers(window)

            # ---------------- Dynamic Single-Line Terminal Telemetry (10 Hz) ----------------
            now = time.time()
            if now - last_hud_time > 0.10:
                last_hud_time = now
                fsm_idx = out.fsm_state
                fsm_str = FSM_STATE_NAMES[fsm_idx] if 0 <= fsm_idx < len(FSM_STATE_NAMES) else f"STATE_{fsm_idx}"
                is_strafing = (abs(c_in.vy) > 0.05 and abs(c_in.vx) < 0.1)
                mode_tag = "STRAFE(391mm)" if is_strafing else "ROAD(Turn+Bank)"
                if spin_phase:
                    mode_tag = ('', 'SPIN ALIGNING', 'SPIN ACTIVE', 'SPIN STOPPING')[spin_phase]
                elif spin_request:
                    mode_tag = 'SPIN WAITING'
                active_mode_label = "PAPER_NORMAL" if current_climb_mode == 1 else ("STAIR_200MM" if current_climb_mode == 2 else "DISABLED")

                dashboard = (
                    f"\r\033[K[MobED] {mode_tag} | Mode=[{active_mode_label}] | "
                    f"Vx={c_in.vx:+.2f} m/s | Vy={c_in.vy:+.2f} m/s | "
                    f"Wz={c_in.wz:+.2f} rad/s | Bank={np.degrees(out.bank_angle):+.1f}° | FSM={fsm_str}"
                )
                if ecc_test_active or np.any(ecc_test_max_power > 0.0):
                    dashboard += " | Power W current/max: " + ", ".join(
                        f"{JOINT_NAMES[i].replace('Steering_joint_', 'S_').replace('Posture_control_joint_', 'E_').replace('Wheel_joint_', 'W_')}="
                        f"{ecc_test_last_power[i]:+.1f}/{ecc_test_max_power[i]:.1f}"
                        for i in range(12)
                    )
                sys.stdout.write(dashboard)
                sys.stdout.flush()

    finally:
        print("\n\n==> Cleaning up resources...")
        lib.mobed_destroy(controller)
        con.free()
        glfw.destroy_window(window)
        glfw.terminate()
        print("==> Simulation terminated cleanly.")


if __name__ == '__main__':
    main()
