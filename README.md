# Rat in a Maze

This project implements a wall-following robot using ultrasonic sensors and PID control for precise navigation. The robot can autonomously follow walls on either side and handle turns based on sensor input.


## Table of Content
1. Introduction   
2. Features  
3. Hardware Requirements  
4. Software Requirements  
5. Circuit Diagram  
6. Code Explanation  
7. PID Control Mechanism  
8. Setup and Usage  
9. Testing and Debugging  
10. Future Enhancements  



# Introduction
The Wall-Following Robot is designed to navigate autonomously by following walls and making decisions based on distance measurements from ultrasonic sensors. The robot uses a PID (Proportional-Integral-Derivative) control algorithm to adjust motor speeds and maintain consistent wall-following behavior.




## Features
- Follows walls on the left or right side.
- Detects obstacles and adjusts its path.
- Configurable PID control for precise movement.
- LED indicators for debugging wall-following logic.
- Real-time distance measurement and error calculation.

## Hardware Requirements

The following components are required to assemble and operate the wall-following robot:

- *Microcontroller:*
  - Arduino Uno (or any compatible microcontroller)
- *Sensors:*
  - 3 Ultrasonic Sensors (e.g., HC-SR04)
    - Left Sensor
    - Right Sensor
    - Front Sensor
- *Motor Driver:*
  - L298N Motor Driver Module (or compatible)
- *Motors:*
  - 2 DC Motors for differential drive
- *Power Supply:*
  - Battery Pack (sufficient voltage and current for the motors and microcontroller)
- *Miscellaneous:*
  - Jumper Wires
  - Breadboard (optional for prototyping)
  - Chassis for mounting components
  - LEDs for status indicators
## Software Requirements

To successfully program and operate the wall-following robot, you will need the following software and tools:

- *Arduino IDE:*
  - Version 1.8.19 or newer for programming the microcontroller.
  - [Download here](https://www.arduino.cc/en/software)
- *Arduino Libraries:*
  - NewPing Library for controlling ultrasonic sensors.
  - [Installation Guide](https://www.arduino.cc/en/guide/libraries)
- *Serial Monitor:*
  - Built into the Arduino IDE for debugging and monitoring sensor values.
- *Operating System:*
  - Compatible with Windows, macOS, or Linux.
- *Additional Tools (Optional):*
  - Fritzing (or any circuit diagram tool) for visualizing hardware connections.
  - Git/GitHub for version control (optional for collaborative development).

## Circuit Diagram

![MAZE](https://github.com/user-attachments/assets/fd520509-b03e-4619-a304-14d79865aa1d)
- *Ultrasonic Sensors:*
  - Left Sensor: Trig (A3), Echo (A0)
  - Front Sensor: Trig (A4), Echo (A1)
  - Right Sensor: Trig (A5), Echo (A2)
- *Motor Driver:*
  - Left Motor: Pins 2, 3
  - Right Motor: Pins 4, 5
- *Power Supply:*
  - Connect to motor driver and Arduino VIN/GND.


## Code Explanation

### Sensor Readings:
- readSensorValues() collects and averages data from the ultrasonic sensors.

### Wall Detection:
- detectWalls() identifies the walls based on distance thresholds.

### PID Control:
- Adjusts motor speeds to minimize errors between sensor readings.

### Motor Control:
- setMotorDirection() defines motor states for movement directions (e.g., FORWARD, BACKWARD).
## PID Control Mechanism

- *Proportional Term (P):* Directly proportional to the error.
- *Integral Term (I):* Accumulates error over time.
- *Derivative Term (D):* Predicts future error trends.
- Adjusting the constants kp, ki, and kd tunes the robot's responsiveness.

## Setup and Usage

### Hardware Assembly
- Assemble the circuit using the provided hardware.
- Attach sensors and motors to the chassis.

### Code Upload
- Open Arduino IDE and upload the code.

### Operation
- Place the robot near a wall.
- Power it on.
- Observe LEDs for debugging:
  - *Status LED* (pin 13): Initialization successful.
  - *Indicator LEDs* (pins 8, 9): Shows side being followed.

## Testing and Debugging

### Sensor Calibration
- Test ultrasonic sensors via the Serial Monitor.

### PID Tuning
- Adjust constants *kp, **ki, **kd*:
  - *High kp*: Increases responsiveness but may cause oscillations.
  - *High ki*: Corrects steady-state errors.
  - *High kd*: Dampens rapid error changes.

### LED Indicators
- Debug wall-following behavior based on LED states.
## Future Enhancements

- Dynamic PID tuning for real-time adaptability.
- Distance-based stopping mechanism for better obstacle avoidance.
- Bluetooth/Wi-Fi control for manual overrides.
- Enhanced motor driver for smoother operation.
