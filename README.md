# Self-Stabilizing Smart Spoon for Parkinson's Patients

## Overview
This is my 3rd-year Embedded Systems Laboratory project — a **low-cost self-stabilizing smart spoon** designed to help people with **Parkinson's disease** eat independently by reducing the effect of hand tremors.

## Features
- **ATmega328P Microcontroller**
- **MPU6050** (Gyroscope + Accelerometer)
- **Dual Servo Motors** for stabilization
- **Complementary Filter** for sensor fusion
- **PWM Control** for precise servo positioning
- PCB designed using **EasyEDA**
- Firmware written in **AVR C**, with Arduino-based MPU6050 calibration

## How It Works
The MPU6050 detects involuntary hand movements (roll, pitch, yaw). The microcontroller processes the data, applies a complementary filter, and controls the servo motors to keep the spoon bowl stable.

## Accuracy
- Roll Error: ~4.2°
- Pitch Error: ~2.8°

## Folder Structure
```
src/            # Source code for the project
README.md       # Project documentation
```

## Getting Started
1. Clone the repository
2. Compile the `smart_spoon.c` file using AVR-GCC
3. Upload the compiled HEX file to an ATmega328P microcontroller
4. Connect the MPU6050 and servo motors as per the circuit diagram
5. Power on and test

## License
This project is for educational purposes only.

---
