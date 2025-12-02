# Gesture-Based Pick & Place – Panda Arm Simulator 🖐️🤖

Control a **Franka Panda robot arm** in PyBullet using just your **hand in front of a webcam**.

- Move your hand → the robot’s end-effector follows.
- Open your hand → the gripper opens.
- Make a fist → the gripper closes and can grab a cube.
- Pick up the cube, move it, and drop it somewhere else – fully in simulation.

---

## 🎯 Project Overview

This project is a **gesture-controlled robotic pick-and-place simulator** built with:

- **Python**
- **MediaPipe + OpenCV** (for hand tracking & gesture recognition)
- **PyBullet** (for physics simulation & Panda arm)

---

## ⚙️ Tech Stack

- **Python 3.9+**
- **OpenCV** – webcam capture & display
- **MediaPipe Hands** – 21-point hand landmark detection
- **PyBullet** – Franka Panda arm + cube + physics
- **NumPy** – math & smoothing

---

## 🧠 Core Ideas

1. **Hand Tracking:**  
   MediaPipe detects the hand and gives 21 landmarks per frame.

2. **Gesture Classification:**  
   Uses distances between fingertips and wrist to detect:
   - **Open hand** → `OPEN`
   - **Closed hand / fist** → `FIST`

3. **Smooth Control:**  
   The hand center `(cx, cy)` is **smoothed** using an exponential moving average to avoid jitter.

4. **Robot Control:**  
   - `(cx, cy)` offset from the screen center → controls the **end-effector target position** in Y/Z.
   - PyBullet’s **inverse kinematics** gives joint angles for the Panda arm.
   - Gripper joints open/close based on gesture.

---

## 🕹️ Controls

In front of your **webcam**:

| Gesture / Motion             | Effect on Robot                          |
|-----------------------------|------------------------------------------|
| ✋ Open hand                | Gripper opens + arm moves with your hand |
| ✊ Fist                     | Gripper closes (try to grab the cube)    |
| Move hand left / right      | End-effector moves left / right (Y axis) |
| Move hand up / down         | End-effector moves up / down (Z axis)    |
| Hand near center            | Arm stays mostly still (deadzone)        |

> Tip: Move slowly and smoothly – the EMA filter will keep the arm motion clean.
