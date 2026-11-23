# Dum-E — Robotic Arm

**A 6-DOF robotic arm built with Arduino UNO Q, Python, and computer vision.**

## Overview

Dum-E is a personal robotics project focused on designing, programming, and controlling a six-degree-of-freedom robotic arm.

The project combines embedded programming, Python-based control, servo calibration, sensor integration, and computer vision to explore robotic manipulation and automation.

## Features

- **6-DOF Robotic Arm:** Independent control of six servo motors.
- **Python Control Interface:** Serial communication between a computer and the Arduino UNO Q.
- **Servo Calibration:** Position calibration and mechanical backlash compensation to improve movement consistency.
- **Distance Sensing:** VL53L1X Time-of-Flight sensor integration.
- **Computer Vision:** Experimentation with OpenCV, MediaPipe, and YOLO for visual detection and tracking.
- **Automated GitHub Synchronization:** Changes made in Arduino App Lab are automatically committed and pushed to GitHub.

## Hardware

| Component | Purpose |
|---|---|
| Arduino UNO Q | Main controller |
| PCA9685 | 16-channel PWM servo driver |
| 3 × MG995 | High-torque servo motors |
| 3 × MG90S | Compact servo motors |
| VL53L1X | Time-of-Flight distance sensor |
| USB Camera | Computer vision input |
| External 6V Power Supply | Servo motor power |

## Software & Technologies

- **Languages:** Python, C++
- **Embedded Systems:** Arduino UNO Q, Arduino App Lab
- **Communication:** Serial communication, I²C
- **Computer Vision:** OpenCV, MediaPipe, YOLO
- **Version Control:** Git, GitHub
- **Automation:** Linux, Bash, systemd
- **Mechanical Design:** FreeCAD, 3D printing

## System Architecture

```text
        Computer / Python
                |
         Serial Commands
                |
         Arduino UNO Q
                |
       +--------+--------+
       |                 |
    PCA9685           VL53L1X
       |            Distance Sensor
       |
   6 Servo Motors
       |
   Robotic Arm

Camera → Computer Vision Processing
```

## Communication Protocol

The robotic arm supports serial commands for controlling and monitoring its components.

| Command | Description |
|---|---|
| `PING` | Check controller connectivity |
| `STATUS` | Request system status |
| `SERVO:<id>:<angle>` | Set a servo position |
| `GET_SERVO` | Retrieve servo positions |
| `GET_DISTANCE` | Read the distance sensor |
| `STOP` | Stop the controlled operation |

## Calibration and Motion Control

A key challenge was achieving consistent movement using hobby servo motors with mechanical backlash.

To improve repeatability, I developed a calibration approach using experimentally measured servo positions and interpolation.

I also implemented a directional approach strategy for wrist positioning to compensate for mechanical backlash.

## Performance Evaluation

The project includes ongoing testing of:

- End-effector positioning accuracy (mm)
- Movement repeatability
- Servo response and calibration
- Communication latency (ms)

Quantitative performance results will be added after controlled measurements.

## Repository Structure

```text
Dum-E/
├── sketch/
│   ├── sketch.ino
│   └── sketch.yaml
├── python/
│   └── main.py
├── app.yaml
├── Main.py
├── arm_calibration.py
├── test.py
└── README.md
```

## Future Improvements

- Integrate object detection with robotic arm movements.
- Improve automated object tracking and positioning.
- Develop inverse kinematics for Cartesian movement.
- Implement automated pick-and-place operations.
- Measure and optimize positioning accuracy and response time.

## Project Status

**Active development** — core servo control, serial communication, sensor integration, and calibration have been implemented. Additional autonomous capabilities are under development.

## Author

**Idan Kislev**

Personal robotics and embedded systems project.

GitHub: [JoeyMan09/Dum-E](https://github.com/JoeyMan09/Dum-E)




