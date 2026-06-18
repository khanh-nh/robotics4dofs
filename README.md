# 4DOF Humanoid Arm with Tools Changing Mechanism - Robotics and Automation

A 4-Degree-of-Freedom (4DOF) robotic humanoid arm controlled via an ESP32 microcontroller. This project features full Forward Kinematics (FK) and Inverse Kinematics (IK) solvers, serial command controls, an end-effector with changeable end-effector tools, and multiple pre-programmed complex motions (like handshakes, waving, and pick-and-place).

## Demo Video


https://github.com/user-attachments/assets/3dcf2fd9-d3b4-4252-ae77-8185305c0198

## Features

- **Inverse & Forward Kinematics:** Custom IK/FK engine to move the arm using Cartesian coordinates (`X, Y, Z`) or explicit joint angles (`Pitch, Roll, Yaw, Elbow`).
- **Pre-programmed Motions:** Ready-to-use commands for waving, handshaking, table picking, and stroking.
- **ESP32 Servo Control:** Employs the `ESP32Servo` library for smooth and precise hardware PWM servo management.
- **Stepper Integration:** Includes support for an auxiliary stepper motor.
- **Interactive Serial CLI:** Send coordinate targets or trigger animations instantly via the Arduino Serial Monitor.

## Hardware Configuration

The robot's hardware architecture relies on an ESP32 microcontroller to coordinate various actuators and end-effectors. Below are the key components used in the setup:

### Servos
The arm joints are actuated by PWM servos. Ensure you have a dedicated external power supply for the servos (e.g., a 5V/6V high-current buck converter), as drawing power directly from the ESP32 will cause brownouts and erratic behavior.
- `S0` (Shoulder Pitch) = **GPIO25**
- `S1` (Shoulder Roll) = **GPIO26**
- `S2` (Shoulder Yaw) = **GPIO32**
- `S3` (Elbow Pitch) = **GPIO33**
- `Tool Servo` = **GPIO27**

### Stepper Motor & Driver
An auxiliary stepper motor is included, driven by a standard stepper motor driver (such as an A4988, DRV8825, or TMC2209).
- `Step` = **GPIO14**
- `Dir` = **GPIO12**

### End-Effectors (Pogo Pins & Electromagnet)
The arm is designed to support modular end-effectors for different automation tasks:
- **Electromagnet:** Used for magnetic pick-and-place operations. Because it draws higher current, it must be controlled via a relay module or a MOSFET circuit triggered by an ESP32 GPIO pin.
- **Pogo Pins:** Attached to the end-effector to establish temporary, spring-loaded electrical connections. They are perfect for the tool-changing mechanism.

## Usage & Serial Commands

Upload the code to your ESP32 and open the Serial Monitor at **115200 baud** (with 'Newline' enabled). You can use the following commands:

### Core Movement Commands
- `go x y z` : Move the end-effector to absolute Cartesian coordinates in mm (e.g., `go 120 160 250`).
- `q pitch roll yaw elbow` : Move joints to specific angles (e.g., `q 90 80 70 60`).
- `fk pitch roll yaw elbow` : Calculate and print the expected end-effector position for the given angles.
- `tool` : Briefly run the end-effector/tool servo.
- `rest` : Return the arm to the default rest pose.
- `speed <value>` : Change motion speed (smaller value = faster, default is 15).

### Pre-programmed Motions
- `wave` : Performs a waving animation.
- `hello` : Performs a greeting animation.
- `handshake` : Reaches out and shakes a hand.
- `pickplace` : Performs a generic pick-and-place sequence.
- `frontpick` : Reaches down to pick a low object from a table in front.
- `raise` / `side` / `bend` : Simple predefined postural tests.

### Utility Commands
- `help` : Displays the full list of available commands and coordinate frame details.
- `status` : Prints current joint angles and calculates the current end-effector position.
- `relay on` / `relay off` : Toggles the configured relay.
- `stepper <degrees>` : Rotates the stepper motor by a set amount of degrees (e.g., `stepper 360`).

## Coordinate System

- **Origin:** Center of the hanging base/body
- **+X axis:** Forward
- **+Y axis:** Left/Outward
- **+Z axis:** Upward
