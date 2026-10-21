#!/usr/bin/env python3
"""
Self-Balancing Robot Simulation — PyBullet
==========================================
Physics-correct inverted-pendulum balancing bot.

HOW THE PHYSICS WORK
--------------------
The chassis CoM is at z=+0.15 m above the wheel axle. Gravity pulls it down.
Any forward pitch (pitch > 0) creates a gravitational torque that accelerates
the fall further — classic unstable inverted pendulum.

The PID controller measures the pitch angle and drives wheel torque to
move the wheel contact point *under* the CoM, restoring balance.

Sign convention (confirmed by headless test):
  pitch > 0  → chassis tilted forward (+X)
  To cancel this, wheels must accelerate in +X (forward)
  In PyBullet with our URDF: positive joint torque → forward motion
  So:  tau = +Kp * pitch  (positive feedback on forward lean → push forward)

If Kp = 0  → no corrective torque → robot falls under gravity (visible!)
If Kp too low → robot slowly falls
If Kp tuned correctly (≈ 15-25) → robot balances upright
"""

import math
import os
import sys
import time

import pybullet as p
import pybullet_data

# ==============================================================================
# SIMULATION PARAMETERS
# ==============================================================================
SIM_HZ          = 240                    # Physics update rate (Hz)
SIM_DT          = 1.0 / SIM_HZ          # Physics timestep (s)
WHEEL_RADIUS    = 0.05                   # Must match URDF (m)
MAX_TORQUE      = 10.0                   # Per-wheel torque limit (N·m)
FALL_ANGLE_DEG  = 55.0                   # Auto-detect fall beyond this pitch

# Default PID gains  (Kp~20, Ki~0.4, Kd~1.8 gives stable balance)
DEFAULT_KP      = 20.0
DEFAULT_KI      = 0.4
DEFAULT_KD      = 1.8
DEFAULT_KSPEED  = 0.04   # Speed-damping gain (outer loop)

# CoM height from axle — used for physics display only
COM_HEIGHT      = 0.15   # m  (matches URDF inertial origin)

# Impulse disturbance tests
IMPULSE_CONFIGS = {
    ord('1'): {"force": +10.0, "steps": 8, "label": "LIGHT  (+10 N)"},
    ord('2'): {"force": -20.0, "steps": 8, "label": "MEDIUM (-20 N)"},
    ord('3'): {"force": +35.0, "steps": 8, "label": "HEAVY  (+35 N)"},
}


# ==============================================================================
# HELPERS
# ==============================================================================

def load_robot(urdf_path, tilt_rad=0.03):
    """Spawn robot at a slight tilt so PID must immediately react."""
    orn = p.getQuaternionFromEuler([0.0, tilt_rad, 0.0])
    rid = p.loadURDF(
        urdf_path,
        basePosition=[0.0, 0.0, 0.05],   # axle height = wheel radius
        baseOrientation=orn,
        flags=p.URDF_USE_INERTIA_FROM_FILE
    )
    return rid


def resolve_wheel_joints(robot_id):
    left_j = right_j = None
    for j in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, j)[1].decode()
        if name == "left_wheel_joint":
            left_j = j
        elif name == "right_wheel_joint":
            right_j = j
    if left_j is None or right_j is None:
        left_j, right_j = 0, 1
    return left_j, right_j


def configure_joints(robot_id, joints):
    """Unlock joints for torque control (disable built-in velocity servo)."""
    for j in joints:
        p.changeDynamics(robot_id, j,
                         lateralFriction=2.0,
                         spinningFriction=0.001,
                         rollingFriction=0.001)
        # Remove position/velocity hold so we can apply raw torque
        p.setJointMotorControl2(robot_id, j,
                                controlMode=p.VELOCITY_CONTROL,
                                force=0.0)


def apply_torque(robot_id, left_j, right_j, tau_l, tau_r):
    """Clamp and apply wheel torques."""
    tau_l = max(-MAX_TORQUE, min(MAX_TORQUE, tau_l))
    tau_r = max(-MAX_TORQUE, min(MAX_TORQUE, tau_r))
    p.setJointMotorControl2(robot_id, left_j,
                            controlMode=p.TORQUE_CONTROL, force=tau_l)
    p.setJointMotorControl2(robot_id, right_j,
                            controlMode=p.TORQUE_CONTROL, force=tau_r)
    return tau_l, tau_r


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    # -- 1. Physics & GUI ---------------------------------------------------------
    client = p.connect(p.GUI)
    if client < 0:
        print("[FATAL] Cannot open PyBullet GUI.")
        sys.exit(1)

    p.setGravity(0, 0, -9.81)
    p.setTimeStep(SIM_DT)
    p.setRealTimeSimulation(0)

    # Clean up default PyBullet chrome
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,   0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

    p.resetDebugVisualizerCamera(
        cameraDistance=1.2,
        cameraYaw=40.0,
        cameraPitch=-12.0,
        cameraTargetPosition=[0.0, 0.0, 0.18]
    )

    # -- 2. Load Ground & Robot ---------------------------------------------------
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF("plane.urdf")
    p.changeDynamics(plane, -1, lateralFriction=1.5, restitution=0.0)

    urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "balancing_bot.urdf")
    if not os.path.exists(urdf_path):
        print(f"[FATAL] URDF not found at: {urdf_path}")
        sys.exit(1)

    robot = load_robot(urdf_path, tilt_rad=0.03)   # ~1.7 deg initial lean
    left_j, right_j = resolve_wheel_joints(robot)
    configure_joints(robot, [left_j, right_j])

    # Base chassis damping (small, mimics real bearings / air drag)
    p.changeDynamics(robot, -1, linearDamping=0.01, angularDamping=0.01)

    # -- 3. GUI Sliders -----------------------------------------------------------
    sl_kp    = p.addUserDebugParameter("Kp  [Tilt P-gain]",   0.0,  60.0, DEFAULT_KP)
    sl_ki    = p.addUserDebugParameter("Ki  [Tilt I-gain]",   0.0,   5.0, DEFAULT_KI)
    sl_kd    = p.addUserDebugParameter("Kd  [Tilt D-gain]",   0.0,   8.0, DEFAULT_KD)
    sl_ks    = p.addUserDebugParameter("Kspd [Speed damp]",   0.0,   0.2, DEFAULT_KSPEED)
    sl_tilt  = p.addUserDebugParameter("Target Lean (deg)",  -8.0,   8.0, 0.0)
    sl_push  = p.addUserDebugParameter("Live Push (N)",     -15.0,  15.0, 0.0)

    # Static help text
    p.addUserDebugText(
        "Arrow Up/Down: drive   Arrow Left/Right: steer\n"
        "SPACE: nudge   1/2/3: impulse   r: reset   q: quit",
        textPosition=[-0.9, -0.85, 0.45],
        textColorRGB=[0.7, 0.9, 1.0],
        textSize=1.1,
        lifeTime=0
    )

    # -- 4. Controller State ------------------------------------------------------
    integral      = 0.0
    vel_smooth    = 0.0   # low-pass filtered forward speed (m/s)
    last_kp = DEFAULT_KP
    last_ki = DEFAULT_KI
    last_kd = DEFAULT_KD
    last_ks = DEFAULT_KSPEED

    imp_force  = 0.0
    imp_steps  = 0
    imp_label  = ""

    fallen     = False
    status     = "BALANCING"
    step       = 0
    hud_id     = -1
    # placeholder so they exist before first HUD render
    P = I = D = 0.0
    tau_l = tau_r = 0.0
    target_pitch  = 0.0
    fx = 0.0

    print("=" * 60)
    print("  Self-Balancing Robot — PyBullet")
    print(f"  Default gains: Kp={DEFAULT_KP} Ki={DEFAULT_KI} Kd={DEFAULT_KD}")
    print("  Set Kp=0 -> robot falls immediately (no control)")
    print("  Increase Kp from 0 -> robot starts fighting gravity")
    print("=" * 60)

    # -- 5. Main Loop -------------------------------------------------------------
    try:
        while True:
            loop_t = time.perf_counter()

            # Read sliders
            kp   = p.readUserDebugParameter(sl_kp)
            ki   = p.readUserDebugParameter(sl_ki)
            kd   = p.readUserDebugParameter(sl_kd)
            ks   = p.readUserDebugParameter(sl_ks)
            lean = p.readUserDebugParameter(sl_tilt)    # degrees
            push = p.readUserDebugParameter(sl_push)    # Newtons

            # Flush integral if gains changed (prevent windup carry-over)
            if (abs(kp - last_kp) > 1e-4 or abs(ki - last_ki) > 1e-4 or
                    abs(kd - last_kd) > 1e-4 or abs(ks - last_ks) > 1e-4):
                integral = 0.0
                last_kp, last_ki, last_kd, last_ks = kp, ki, kd, ks
                status = f"GAIN UPDATE  Kp={kp:.1f}  Ki={ki:.2f}  Kd={kd:.2f}"

            # Keyboard
            keys = p.getKeyboardEvents()

            if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
                print("[INFO] Quit.")
                break

            if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                p.removeBody(robot)
                robot = load_robot(urdf_path, tilt_rad=0.03)
                left_j, right_j = resolve_wheel_joints(robot)
                configure_joints(robot, [left_j, right_j])
                p.changeDynamics(robot, -1, linearDamping=0.01, angularDamping=0.01)
                integral   = 0.0
                vel_smooth = 0.0
                fallen     = False
                imp_steps  = 0
                status     = "RESET"
                hud_id     = -1
                step       = 0
                P = I = D = tau_l = tau_r = 0.0
                fx = 0.0
                target_pitch = 0.0
                continue

            # Space-bar nudge
            if p.B3G_SPACE in keys and keys[p.B3G_SPACE] & p.KEY_WAS_TRIGGERED:
                imp_force = 10.0
                imp_steps = 10
                imp_label = "NUDGE +10 N"
                status    = imp_label

            # 1/2/3 impulses
            for code, cfg in IMPULSE_CONFIGS.items():
                if code in keys and keys[code] & p.KEY_WAS_TRIGGERED:
                    imp_force = cfg["force"]
                    imp_steps = cfg["steps"]
                    imp_label = cfg["label"]
                    status    = f"IMPULSE {imp_label}"

            # Arrow key lean commands (drive / steer)
            key_lean   = 0.0
            steer_diff = 0.0
            if p.B3G_UP_ARROW   in keys and keys[p.B3G_UP_ARROW]   & p.KEY_IS_DOWN:
                key_lean  += math.radians(3.0)
                status = "FORWARD"
            if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
                key_lean  -= math.radians(3.0)
                status = "BACKWARD"
            if p.B3G_LEFT_ARROW  in keys and keys[p.B3G_LEFT_ARROW]  & p.KEY_IS_DOWN:
                steer_diff += 0.5
            if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
                steer_diff -= 0.5

            # Sense
            pos, orn     = p.getBasePositionAndOrientation(robot)
            lin_v, ang_v = p.getBaseVelocity(robot)
            euler        = p.getEulerFromQuaternion(orn)

            # Pitch = rotation about Y-axis (index 1 in PyBullet Euler)
            # +ve pitch = tilted forward (+X direction)
            pitch      = euler[1]
            pitch_rate = ang_v[1]

            # Wheel speeds -> forward linear speed
            w_l = p.getJointState(robot, left_j)[1]
            w_r = p.getJointState(robot, right_j)[1]
            fwd_speed = 0.5 * (w_l + w_r) * WHEEL_RADIUS  # m/s

            # Low-pass on speed for outer loop
            vel_smooth = 0.9 * vel_smooth + 0.1 * fwd_speed

            # Fall Detection
            if abs(math.degrees(pitch)) > FALL_ANGLE_DEG and not fallen:
                fallen = True
                status = f"FALLEN  pitch={math.degrees(pitch):.1f} deg  (press r to reset)"

            # Control Law
            if fallen:
                apply_torque(robot, left_j, right_j, 0.0, 0.0)
            else:
                # Target pitch = user lean command - speed damping
                target_pitch = (math.radians(lean) + key_lean
                                - ks * vel_smooth)
                target_pitch = max(-math.radians(10), min(math.radians(10), target_pitch))

                # PID error
                err = pitch - target_pitch

                # Derivative: use measured angular velocity (more stable than finite diff)
                d_err = pitch_rate

                # Integral with anti-windup
                integral += err * SIM_DT
                integral  = max(-0.5, min(0.5, integral))

                # ---------------------------------------------------------------
                # SIGN:  pitch > 0 (forward tilt) -> err > 0
                #        We need positive torque to drive wheels forward and
                #        bring the CoM back over the axle.
                #        Verified: positive joint torque -> forward (+X) motion.
                # ---------------------------------------------------------------
                P = kp * err
                I = ki * integral
                D = kd * d_err

                tau = P + I + D

                # Yaw damping
                yaw_rate  = ang_v[2]
                tau_steer = -0.15 * yaw_rate + steer_diff

                tau_l, tau_r = apply_torque(robot, left_j, right_j,
                                            tau - tau_steer,
                                            tau + tau_steer)

                if imp_steps == 0 and not fallen:
                    if abs(lean) > 0.05 or abs(key_lean) > 0.001:
                        status = f"LEAN CMD {lean:+.1f} deg"
                    else:
                        status = "BALANCING"

            # External Disturbances
            fx = push
            if imp_steps > 0:
                fx     += imp_force
                imp_steps -= 1
                if imp_steps == 0:
                    imp_label = ""

            if abs(fx) > 0.01:
                p.applyExternalForce(
                    robot, -1,
                    forceObj=[fx, 0.0, 0.0],
                    posObj=[0.0, 0.0, COM_HEIGHT],
                    flags=p.LINK_FRAME
                )

            # Step Physics
            p.stepSimulation()
            step += 1

            # HUD (~60 Hz)
            if step % 4 == 0:
                pd = math.degrees(pitch)
                td = math.degrees(target_pitch)

                if not fallen:
                    hud = (
                        "== TELEMETRY ===================\n"
                        f"Pitch :  {pd:+6.2f} deg  (target {td:+5.2f} deg)\n"
                        f"FwdSpd:  {vel_smooth:+6.3f} m/s\n"
                        f"PID   :  P={P:+6.2f}  I={I:+5.2f}  D={D:+5.2f}\n"
                        f"Torque:  L={tau_l:+5.2f}  R={tau_r:+5.2f}  Nm\n"
                        f"Gains :  Kp={kp:.1f}  Ki={ki:.2f}  Kd={kd:.2f}\n"
                        f"Push  :  {fx:+5.1f} N\n"
                        f"Status:  {status}"
                    )
                    color = [0.4, 1.0, 0.5]
                else:
                    hud = (
                        "== FALLEN ======================\n"
                        f"Pitch: {pd:+6.1f} deg\n"
                        f"Gains: Kp={kp:.1f}  Ki={ki:.2f}  Kd={kd:.2f}\n"
                        "Press  r  to reset\n"
                        "Tip: Kp~20 gives stable balance"
                    )
                    color = [1.0, 0.35, 0.25]

                hud_pos = [pos[0], pos[1], pos[2] + 0.45]

                if hud_id == -1:
                    hud_id = p.addUserDebugText(hud, hud_pos,
                                                textColorRGB=color,
                                                textSize=1.05,
                                                lifeTime=0)
                else:
                    hud_id = p.addUserDebugText(hud, hud_pos,
                                                textColorRGB=color,
                                                textSize=1.05,
                                                lifeTime=0,
                                                replaceItemUniqueId=hud_id)

                cam = p.getDebugVisualizerCamera()
                p.resetDebugVisualizerCamera(
                    cameraDistance=cam[10],
                    cameraYaw=cam[8],
                    cameraPitch=cam[9],
                    cameraTargetPosition=[pos[0], pos[1], 0.15]
                )

            # Real-time pacing
            dt_left = SIM_DT - (time.perf_counter() - loop_t)
            if dt_left > 0:
                time.sleep(dt_left)

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
    finally:
        p.disconnect()
        print("[INFO] Disconnected.")


if __name__ == "__main__":
    main()
