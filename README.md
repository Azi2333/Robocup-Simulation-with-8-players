# 🤖 NAO Humanoid Soccer Simulation (Webots)

A multi-agent robot soccer system built in Webots. Three roles (Striker, Defender, Goalkeeper) are implemented with individual controllers and motion strategies. Defenders are split into forward and backfield units for zone control.

---

## 🎯 What It Does

- 🟢 Striker tracks the ball, aligns with goal, and shoots.
- 🔵 Defenders intercept in forward or backfield zones.
- 🟠 Goalkeeper blocks shots and clears the ball.

Each robot runs an independent FSM and reacts to local sensing.

---

## 🔍 Key Features

- Finite State Machine (FSM) for behavior control
- Predefined `.motion` files for walk, kick, turn, stand up, etc.
- Simulated GPS + IMU + dual-camera system
- Object detection using **YOLOv4-tiny** (offline via MATLAB)
  - Detects ball, teammates, opponents
  - Goal detection via HSV and contour filtering

---

## 🗂 Structure

```bash
striker_opponent/         # Striker controller
forward_defender/         # Midline defender
backfield_defender/       # Defensive-zone defender
goalkeeper_controller/    # Goalkeeper logic
*.motion                  # All NAO movement primitives
README.md
