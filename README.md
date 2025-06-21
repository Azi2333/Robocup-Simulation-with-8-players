# 🤖 NAO Humanoid Soccer Simulation (Webots)

A multi-agent robot soccer system built in Webots. Three roles (Striker, Defender, Goalkeeper) are implemented with individual controllers and motion strategies. Defenders are split into forward and backfield units for zone control.

## Pre-requisites
- Webots
- Matlab
- MATLAB Support for MinGW-w64 C/C++/Fortran Compiler https://uk.mathworks.com/matlabcentral/fileexchange/52848-matlab-support-for-mingw-w64-c-c-fortran-compiler
- Matlab Toolboxes: Computer Vision, Deep Learning Toolbox, Computer Vision Toolbox Model for YOLO v4 Object Detection

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

## 🖼️ Demo
👉 https://www.bilibili.com/video/BV1ZkMszuEdw/?spm_id_from=333.1387.upload.video_card.click](https://www.bilibili.com/video/BV1FiNLzkERb/?vd_source=c1da2cac35f64bc74121091ae2ad3517)
