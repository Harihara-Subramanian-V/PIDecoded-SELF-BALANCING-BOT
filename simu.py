import math
import os
import sys
import time
import pybullet as p
import pybullet_data

SIM_DT = 1.0 / 240.0
WHEEL_RADIUS = 0.05
MAX_TORQUE = 10.0
FALL_DEG = 55.0
IMPULSES = {ord('1'): (10.0, 8), ord('2'): (-20.0, 8), ord('3'): (35.0, 8)}


def spawn_robot(urdf_path):
    orn = p.getQuaternionFromEuler([0.0, 0.03, 0.0])
    rid = p.loadURDF(urdf_path, [0.0, 0.0, 0.05], orn, flags=p.URDF_USE_INERTIA_FROM_FILE)
    left_j = right_j = 0
    for j in range(p.getNumJoints(rid)):
        name = p.getJointInfo(rid, j)[1].decode()
        if name == "left_wheel_joint":
            left_j = j
        elif name == "right_wheel_joint":
            right_j = j

    for j in (left_j, right_j):
        p.changeDynamics(rid, j, lateralFriction=2.0, spinningFriction=0.001, rollingFriction=0.001)
        p.setJointMotorControl2(rid, j, controlMode=p.VELOCITY_CONTROL, force=0.0)

    p.changeDynamics(rid, -1, linearDamping=0.01, angularDamping=0.01)
    return rid, left_j, right_j


def main():
    if p.connect(p.GUI) < 0:
        sys.exit(1)

    p.setGravity(0, 0, -9.81)
    p.setTimeStep(SIM_DT)
    p.setRealTimeSimulation(0)

    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.resetDebugVisualizerCamera(1.2, 40.0, -12.0, [0.0, 0.0, 0.18])

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF("plane.urdf")
    p.changeDynamics(plane, -1, lateralFriction=1.5)

    urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "balancing_bot.urdf")
    robot, left_j, right_j = spawn_robot(urdf_path)

    sl_kp = p.addUserDebugParameter("Kp", 0.0, 60.0, 20.0)
    sl_ki = p.addUserDebugParameter("Ki", 0.0, 5.0, 0.4)
    sl_kd = p.addUserDebugParameter("Kd", 0.0, 8.0, 1.8)
    sl_ks = p.addUserDebugParameter("Kspd", 0.0, 0.2, 0.04)
    sl_lean = p.addUserDebugParameter("Target Lean (deg)", -8.0, 8.0, 0.0)
    sl_push = p.addUserDebugParameter("Live Push (N)", -15.0, 15.0, 0.0)

    p.addUserDebugText("Arrows: drive/steer | Space: nudge | 1/2/3: push | r: reset | q: quit",
                       [-0.9, -0.85, 0.45], [0.7, 0.9, 1.0], 1.1, 0)

    integral = vel_smooth = 0.0
    last_gains = (20.0, 0.4, 1.8, 0.04)
    imp_force = 0.0
    imp_steps = 0
    fallen = False
    hud_id = -1
    step = 0
    tau_l = tau_r = 0.0

    while True:
        t_start = time.perf_counter()

        kp = p.readUserDebugParameter(sl_kp)
        ki = p.readUserDebugParameter(sl_ki)
        kd = p.readUserDebugParameter(sl_kd)
        ks = p.readUserDebugParameter(sl_ks)
        lean = p.readUserDebugParameter(sl_lean)
        push = p.readUserDebugParameter(sl_push)

        if (kp, ki, kd, ks) != last_gains:
            integral = 0.0
            last_gains = (kp, ki, kd, ks)

        keys = p.getKeyboardEvents()
        if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
            break

        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
            p.removeBody(robot)
            robot, left_j, right_j = spawn_robot(urdf_path)
            integral = vel_smooth = imp_force = 0.0
            imp_steps = 0
            fallen = False
            continue

        if p.B3G_SPACE in keys and keys[p.B3G_SPACE] & p.KEY_WAS_TRIGGERED:
            imp_force, imp_steps = 10.0, 10

        for key, (f, s) in IMPULSES.items():
            if key in keys and keys[key] & p.KEY_WAS_TRIGGERED:
                imp_force, imp_steps = f, s

        key_lean = 0.0
        steer_diff = 0.0
        if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
            key_lean += math.radians(3.0)
        if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
            key_lean -= math.radians(3.0)
        if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
            steer_diff += 0.5
        if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
            steer_diff -= 0.5

        pos, orn = p.getBasePositionAndOrientation(robot)
        lin_v, ang_v = p.getBaseVelocity(robot)
        pitch = p.getEulerFromQuaternion(orn)[1]
        pitch_rate = ang_v[1]

        w_avg = 0.5 * (p.getJointState(robot, left_j)[1] + p.getJointState(robot, right_j)[1])
        vel_smooth = 0.9 * vel_smooth + 0.1 * (w_avg * WHEEL_RADIUS)

        if abs(math.degrees(pitch)) > FALL_DEG and not fallen:
            fallen = True

        if fallen:
            tau_l = tau_r = 0.0
        else:
            target_pitch = max(-0.17, min(0.17, math.radians(lean) + key_lean - ks * vel_smooth))
            err = pitch - target_pitch
            integral = max(-0.5, min(0.5, integral + err * SIM_DT))

            tau = kp * err + kd * pitch_rate + ki * integral
            tau_steer = -0.15 * ang_v[2] + steer_diff
            tau_l = max(-MAX_TORQUE, min(MAX_TORQUE, tau - tau_steer))
            tau_r = max(-MAX_TORQUE, min(MAX_TORQUE, tau + tau_steer))

        p.setJointMotorControl2(robot, left_j, controlMode=p.TORQUE_CONTROL, force=tau_l)
        p.setJointMotorControl2(robot, right_j, controlMode=p.TORQUE_CONTROL, force=tau_r)

        fx = push + (imp_force if imp_steps > 0 else 0.0)
        if imp_steps > 0:
            imp_steps -= 1

        if abs(fx) > 0.01:
            p.applyExternalForce(robot, -1, [fx, 0.0, 0.0], [0.0, 0.0, 0.15], p.LINK_FRAME)

        p.stepSimulation()
        step += 1

        if step % 4 == 0:
            pd = math.degrees(pitch)
            if not fallen:
                hud = (f"Pitch: {pd:+5.1f} deg | Spd: {vel_smooth:+4.2f} m/s\n"
                       f"Torque: L={tau_l:+4.1f} R={tau_r:+4.1f} Nm\n"
                       f"Kp={kp:.1f} Ki={ki:.2f} Kd={kd:.2f}")
                color = [0.4, 1.0, 0.5]
            else:
                hud = f"FALLEN ({pd:+5.1f} deg)\nPress 'r' to reset"
                color = [1.0, 0.3, 0.3]

            hud_pos = [pos[0], pos[1], pos[2] + 0.4]
            hud_id = p.addUserDebugText(hud, hud_pos, color, 1.05, 0, replaceItemUniqueId=hud_id)

            cam = p.getDebugVisualizerCamera()
            p.resetDebugVisualizerCamera(cam[10], cam[8], cam[9], [pos[0], pos[1], 0.15])

        dt_left = SIM_DT - (time.perf_counter() - t_start)
        if dt_left > 0:
            time.sleep(dt_left)

    p.disconnect()


if __name__ == "__main__":
    main()
