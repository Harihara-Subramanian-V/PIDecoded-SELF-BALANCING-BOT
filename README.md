# PIDecoded: Two-Wheeled Self-Balancing Robot

A physics-correct, real-time simulation of an inverted-pendulum two-wheeled self-balancing robot built using **PyBullet** and **Python**.

![Python](https://img.shields.io/badge/Python-3.10%2B-blue)
![PyBullet](https://img.shields.io/badge/Physics-PyBullet-orange)
![Control](https://img.shields.io/badge/Control-Dual--Loop%20PID-green)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

---

## 📌 Overview

This project simulates a two-wheeled inverted pendulum robot with authentic rigid-body dynamics, contact mechanics, and realistic actuator limits. The center of mass (CoM) is positioned $15\text{ cm}$ above the wheel axle, rendering the uncontrolled system open-loop unstable under gravity.

The robot is stabilized using a **dual-loop PID controller**:
- **Inner Loop (Fast Tilt Stabilization)**: Regulates the pitch angle $\theta$ and angular velocity $\dot{\theta}$ to drive corrective wheel torque.
- **Outer Loop (Speed Damping & Braking)**: Filters wheel linear velocity to prevent runaway drift and maintain a stationary balance point.

---

## ⚙️ Physics & Control Principles

### 1. Inverted Pendulum Dynamics
When the robot tilts forward ($\theta > 0$), gravity produces a positive torque about the wheel axle that accelerates the chassis toward the floor. To counteract this toppling torque:
$$\tau_{\text{motor}} = K_p (\theta - \theta_{\text{target}}) + K_d \dot{\theta} + K_i \int (\theta - \theta_{\text{target}})\,dt$$

- **Positive Pitch ($\theta > 0$)**: Chassis leans forward (+X). The controller commands positive motor torque ($\tau > 0$) to drive the wheels forward, rapidly moving the ground contact point back directly under the center of mass.
- **$K_p = 0$ (No Feedback)**: The robot will fall flat immediately under gravity (demonstrated by dragging the slider or spawning with zero gain).
- **Correct Tuning ($K_p \approx 20, K_i \approx 0.4, K_d \approx 1.8$)**: The robot catches itself within fractions of a second and remains upright even when subjected to external impulses.

---

## 🚀 Features

- **True Physics Simulation**: Powered by PyBullet at 240 Hz with friction and contact dynamics.
- **Real-Time PID Gain Tuning**: In-GUI sliders for $K_p$, $K_i$, $K_d$, and $K_{\text{speed}}$ with immediate memory flush to eliminate windup lag.
- **Live 3D Telemetry HUD**: Displays real-time pitch angle, speed, individual PID term contributions ($P, I, D$), and motor torque.
- **Fall Detection**: Automatically flags falls when $|\theta| > 55^\circ$, cuts torque, and alerts the user to reset.
- **Disturbance Rejection Testing**: Calibrated external pushes (Light, Medium, Heavy) to evaluate controller robustness.
- **Interactive Drive & Steer**: Use arrow keys to manually lean and steer the robot around the plane.

---

## 🎮 Interactive Controls

| Key / Control | Action | Description |
|---|---|---|
| **$K_p, K_i, K_d$ Sliders** | Live Tuning | Adjust proportional, integral, and derivative gains in real-time |
| **$K_{\text{spd}}$ Slider** | Speed Damping | Adjust outer-loop braking gain to eliminate steady-state drift |
| **`Target Lean` Slider** | Lean Command | Command a steady-state tilt angle to make the robot cruise |
| **`Live Push` Slider** | Continuous Disturbance | Apply constant external lateral force (N) |
| **`↑` / `↓` Arrow Keys** | Drive Forward / Reverse | Dynamic pitch offset commanding forward/backward motion |
| **`←` / `→` Arrow Keys** | Steer Left / Right | Differential wheel torque for yaw rotation |
| **`Spacebar`** | Nudge (+10 N) | Tap the chassis to test dynamic catch response |
| **`1`** | Light Impulse (+10 N) | Quick calibrated forward disturbance pulse |
| **`2`** | Medium Impulse (-20 N) | Quick calibrated backward disturbance pulse |
| **`3`** | Heavy Impulse (+35 N) | High-magnitude force pulse |
| **`r`** | Reset Robot | Respawn robot at origin with a slight lean to test PID catch |
| **`q`** | Quit | Exit simulation cleanly |

---

## 📁 Repository Structure

```
.
├── balancing_bot.urdf   # URDF model definition (links, joints, inertia, visuals)
├── simu.py              # Main simulation script with PyBullet GUI & PID logic
├── requirements.txt     # Python package dependencies
├── .gitignore           # Ignored files (caches, virtualenvs)
└── README.md            # Project documentation
```

---

## 🛠️ Installation & Usage

### 1. Clone the Repository
```bash
git clone https://github.com/Harihara-Subramanian-V/PIDecoded-SELF-BALANCING-BOT.git
cd PIDecoded-SELF-BALANCING-BOT
```

### 2. Install Dependencies
Ensure you have Python 3.10+ installed:
```bash
pip install -r requirements.txt
```

### 3. Run the Simulation
```bash
python simu.py
```

---

## 📊 Tuning Guide

1. **Start with $K_p$**: With $K_i=0$ and $K_d=0$, increase $K_p$ from $0$ until the robot begins resisting gravity and oscillates around vertical.
2. **Add $K_d$**: Increase $K_d$ to dampen the oscillations and eliminate rapid jitter.
3. **Add $K_i$**: If there is an offset error (e.g. slight steady tilt due to asymmetry), introduce a small $K_i$ to eliminate steady-state error.
4. **Adjust $K_{\text{spd}}$**: Tune the speed damping parameter so the robot naturally comes to a stop rather than drifting across the floor.

---

## 📄 License
This project is open-source under the [MIT License](LICENSE).
