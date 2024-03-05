
# Intelligent Path Following Car


## Introduction
The Intelligent Vehicle Congestion Control System is a project based on the Arduino platform. It aims to achieve intelligent navigation and congestion control of vehicles through sensor data fusion and the PID control algorithm. The system uses infrared sensors to detect the environment, adjusts the wheel speed to achieve intelligent obstacle avoidance, and controls the vehicle to travel safely in congested environments.

## Key Features
- Reads environmental data using infrared sensors and performs data fusion to obtain the distance to obstacles in front of the vehicle.
- Adjusts the wheel speed based on the PID control algorithm to enable the vehicle to travel safely in congested environments and achieve intelligent obstacle avoidance.
- Gradually adjusts the vehicle speed to reduce the load on the plastic gear transmission system and lower the motor failure rate.
- Implements autonomous navigation and intelligent parking functions to enhance driving safety and stability.

## Technology Stack
- Arduino platform
- C/C++ programming language
- Infrared sensor data acquisition and data fusion
- PID control algorithm
- PWM wheel control

## Installation
1. Clone the repository: `git clone <repository-url>`
2. Upload the Arduino sketch to your Arduino board using the Arduino IDE or any compatible tool.

## Usage
1. Connect the infrared sensors and motor control pins according to the provided schematic.
2. Power on the Arduino board and observe the behavior of the vehicle in different environments.
3. Adjust parameters in the code to fine-tune the performance of the vehicle as needed.
