# Multi-Controller Robotics System

## Overview

This repository contains the source code, simulation files, and supporting resources for a robotics and automation project utilizing:

- ESP32-CAM
- ESP32
- Arduino Nano
- MATLAB Simulation

The project integrates machine vision, wireless communication, and autonomous line-following capabilities.

---

## Project Structure

---

## Modules

### 1. ESP32-CAM Module

Files:

- CAM_MT4.cpp
- CAM_MT4.h
- CAM_MT4.ino

Functions:

- Image acquisition
- Camera initialization
- Visual data processing
- Communication with the main controller

Hardware:

- ESP32-CAM
- OV2640 Camera

---

### 2. ESP32 Integrated Controller

Files:

- ESP32_INTEGRATED.cpp
- ESP32_INTEGRATED.h
- ESP32_INTEGRATED.ino

Functions:

- Central processing unit
- Sensor integration
- Communication management
- Decision making and control

Hardware:

- ESP32 Development Board

---

### 3. Arduino Nano Controller

Files:

- NANO_MT5.cpp
- NANO_MT5.h
- NANO_MT5.ino

Functions:

- Motor control
- Sensor interfacing
- Real-time actuator operation

Hardware:

- Arduino Nano

---

### 4. Universal Line Follower

File:

- line_follower_universal.ino

Features:

- PID-based line tracking
- Multi-sensor support
- Autonomous navigation
- Adjustable speed control

---

### 5. MATLAB Simulation

File:

- lfr_simulation_1.m

Purpose:

- Testing line-following algorithms
- Performance analysis
- Controller tuning
- Path tracking evaluation

---

### 6. Track Layout

File:

- track.png

Purpose:

- Simulation track
- Testing environment
- Algorithm validation

---

## Software Requirements

### Arduino IDE

Required Libraries:

- WiFi
- ESP32 Board Package
- ESP32 Camera Libraries
- Wire
- EEPROM

### MATLAB

Recommended Version:

- MATLAB R2021a or newer

---

## Hardware Requirements

- ESP32-CAM
- ESP32 Development Board
- Arduino Nano
- DC Motors
- Motor Driver
- Line Sensors
- Power Supply/Battery Pack

---

## Installation

1. Clone the repository

```bash
git clone https://github.com/username/repository-name.git
