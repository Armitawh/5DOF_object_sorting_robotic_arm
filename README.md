# 5-DOF Object Sorting Robotic Arm

## About
This project demonstrates a **5-DOF robotic arm** capable of **sorting objects by color and size** using **computer vision and sensor data**. The system integrates **machine learning, computer vision, and embedded control** on a **Raspberry Pi** platform. 

The robotic arm is designed as a **proof-of-concept for automated sorting systems**, showcasing capabilities in **real-time object detection, color classification, and inverse kinematics control**.

---

## Features
- **5 Degrees-of-Freedom robotic arm** for object manipulation.
- **Raspberry Pi 4** as the main controller.
- **Pi Camera** for visual detection of objects.
- **RGB color sensor** for precise color classification.
- **Ultrasonic sensor** for object distance measurement and collision avoidance.
- **Python-based control system** using:
  - OpenCV for computer vision
  - NumPy for kinematics calculations
  - Adafruit ServoKit for servo control
- **Inverse kinematics** implementation for precise positioning.
- Automated sorting of objects based on **color (Red/Blue)** and **size**.

---

## Hardware Requirements
- Raspberry Pi 4 
- Pi Camera Module v2
- Adafruit 16-Channel Servo Driver (PCA9685)
- 5 Servos (standard 180°)
- RGB Color Sensor (TCS34725)
- Ultrasonic Distance Sensor (HC-SR04)
- 5-DOF Robotic Arm
- Jumper wires, power supply, and breadboard

---

## Software Requirements
- Python 3.9+
- OpenCV
- NumPy
- Adafruit CircuitPython libraries:
  - `adafruit_servokit`
  - `adafruit_tcs34725`
- Picamera
- Pillow (PIL)
- Jupyter Notebook (optional, for testing and visualization)

