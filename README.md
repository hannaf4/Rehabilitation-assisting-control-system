# Rehabilitation Assisting Control System

An embedded real-time motion correction platform that uses inertial sensing and closed-loop control to provide adaptive human posture assistance. The system continuously measures body orientation through an IMU and drives an actuator using a PID controller to compensate for undesired motion.

# Project Overview

This project was developed to explore embedded control systems for rehabilitation and assistive technology. It integrates real-time sensor acquisition, state estimation, control algorithms, and actuator feedback on a bare-metal embedded platform.

# Key Features
- Real-time IMU data acquisition over I²C
- PID-based closed-loop control with tunable gains
- PWM motor control for actuator positioning
- Sensor calibration and offset compensation
- GPIO-based hardware interfacing
- Continuous feedback loop for adaptive posture correction
- Bare-metal firmware with direct register-level peripheral programming
- Real-time debugging and control-loop optimization

# Hardware
- Texas Instruments MSP432E4 Microcontroller
- Inertial Measurement Unit (IMU)
- DC Motor / Linear Actuator
- PWM Motor Driver

# Firmware

The firmware was developed in bare-metal C using direct register-level programming to maximize performance and deterministic timing.

Implemented peripherals include:

- GPIO
- PWM
- I²C
- Timers

# Control Algorithm

The controller implements a PID feedback loop:

### Proportional: Corrects instantaneous posture error
### Integral: Eliminates steady-state error
### Derivative: Dampens oscillations and improves stability

Gain tuning and calibration were performed experimentally to achieve smooth actuator response while minimizing overshoot and jitter.

# Future Improvements

- Sensor fusion using complementary or Kalman filtering
- Adaptive PID gain scheduling
- RTOS implementation for task scheduling
- BLE connectivity for remote monitoring
Data logging and telemetry
