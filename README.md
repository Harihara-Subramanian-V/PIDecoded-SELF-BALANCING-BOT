# PIDecoded: Two-Wheeled Self-Balancing Robot

A physics-correct, real-time simulation of an inverted-pendulum two-wheeled self-balancing robot built using **PyBullet** and **Python**.

![Python](https://img.shields.io/badge/Python-3.12-blue)
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

### Inverted Pendulum Dynamics
When the robot tilts forward ($\theta > 0$), gravity produces a positive torque about the wheel axle that accelerates the chassis toward the floor. To counteract this toppling torque:
$$\tau_{\text{motor}} = K_p (\theta - \theta_{\text{target}}) + K_d \dot{\theta} + K_i \int (\theta - \theta_{\text{target}})\,dt$$

- **Positive Pitch ($\theta > 0$)**: Chassis leans forward (+X). The controller commands positive motor torque ($\tau > 0$) to drive the wheels forward, rapidly moving the ground contact point back directly under the center of mass.
- **$K_p = 0$ (No Feedback)**: The robot will fall flat immediately under gravity (demonstrated by dragging the slider or spawning with zero gain).
- **Correct Tuning ($K_p \approx 20, K_i \approx 0.4, K_d \approx 1.8$)**: The robot catches itself within fractions of a second and remains upright even when subjected to external impulses.

---

## 🚀 Features

- **True Physics Simulation**: Powered by PyBullet at 240 Hz with friction and contact dynamics.
- **Real-Time PID Gain Tuning**: In-GUI sliders for $K_p$, $K_i$, $K_d$, and $K_{\text{spd}}$ with immediate memory flush to eliminate windup lag.
- **Live 3D Telemetry HUD**: Displays real-time pitch angle, speed, individual PID term contributions, and motor torque.
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

## 📋 Prerequisites

Before proceeding with installation, ensure your environment meets the following requirements:

- **Operating System**: Windows 10 or Windows 11 (64-bit).
- **Python**: [Python 3.12 (64-bit)](https://www.python.org/downloads/) is the recommended tested version.
  > **Note on Python Versions**: Experimental releases such as Python 3.14 may not have prebuilt binary wheels (`.whl`) available on PyPI for all dependencies. If you use an unsupported Python version, `pip` may attempt to compile PyBullet from source and request Microsoft Visual C++ Build Tools. Sticking to Python 3.12 avoids any compiler requirements.
- **Git**: [Git for Windows](https://git-scm.com/download/win) for cloning the repository.
- **Network**: Internet connection to download required packages from PyPI.
- **Hardware**: No physical robot, microcontroller, or Arduino required; this is a self-contained software simulation.

### Verifying Prerequisites
Open a terminal (PowerShell or Command Prompt) and verify your installations:
```bash
py --version
git --version
```
Both commands should return their respective version numbers without errors.

---

## 🛠️ Guided Setup & Installation

Follow these steps sequentially to set up and run the simulation:

### Step 1 — Download the Project
Clone the repository using Git and navigate into the project directory:
```bash
git clone https://github.com/Harihara-Subramanian-V/PIDecoded-SELF-BALANCING-BOT.git
cd PIDecoded-SELF-BALANCING-BOT
```
> *Explanation*: The `cd` command changes your current working directory to the newly downloaded project folder.

### Step 2 — Create an Isolated Python Environment
Create a dedicated virtual environment using Python 3.12 and activate it:
```bash
py -3.12 -m venv .venv
.venv\Scripts\activate
```
> *Explanation*: The `.venv` folder isolates the project's libraries and Python binaries from your global system environment, preventing dependency conflicts across projects.
> 
> **Expected Result**: Your terminal prompt will be prefixed with `(.venv)`.

### Step 3 — Upgrade the Package Installer
Ensure you have the latest `pip` installed inside your virtual environment:
```bash
python -m pip install --upgrade pip
```
> *Explanation*: Invoking `python -m pip` targets the active virtual environment's Python interpreter directly. This avoids common path mismatches caused by running a bare `pip` command.

### Step 4 — Install Dependencies
Install all required packages from `requirements.txt`:
```bash
python -m pip install -r requirements.txt
```
> *Explanation*: This installs PyBullet and NumPy. Installation typically takes 1–2 minutes depending on your internet connection.
>
> **Troubleshooting Compiler Errors**:
> PyBullet provides prebuilt binary wheels for 64-bit Python 3.10 through 3.12 on Windows. If `pip` starts compiling C++ files and errors out with `Microsoft Visual C++ 14.0 or greater is required`, verify that you created your virtual environment using a supported 64-bit Python version (`py -3.12 -m venv .venv`). Switching to Python 3.12 installs the official prebuilt binary wheel without requiring any C++ build tools.

### Step 5 — Verify the Installation
Confirm that PyBullet, PyBullet Data, and NumPy are imported and functioning properly before running the visual environment:
```bash
python -c "import pybullet; import pybullet_data; import numpy; print('All dependencies installed successfully')"
```
**Expected Output**:
```text
pybullet build time: ...
All dependencies installed successfully
```
*(The specific build timestamp message may vary depending on the release).*

### Step 6 — Launch the Simulation
Start the self-balancing robot simulation:
```bash
python simu.py
```
A 3D PyBullet GUI window will open displaying the balancing robot on a ground plane, interactive gain tuning sliders on the side panel, and real-time telemetry HUD overlaying the robot.

---

## 📊 Tuning Guide

1. **Start with $K_p$**: With $K_i=0$ and $K_d=0$, increase $K_p$ from $0$ until the robot begins resisting gravity and oscillates around vertical.
2. **Add $K_d$**: Increase $K_d$ to dampen the oscillations and eliminate rapid jitter.
3. **Add $K_i$**: If there is an offset error (e.g. slight steady tilt due to asymmetry), introduce a small $K_i$ to eliminate steady-state error.
4. **Adjust $K_{\text{spd}}$**: Tune the speed damping parameter so the robot naturally comes to a stop rather than drifting across the floor.

---

## 📄 License
This project is open-source under the [MIT License](LICENSE).
