# Webots-ESP32-Linefollowing-Dijkstra

## Overview

This project is the implementation of Hardware-in-the-Loop (HiL) Simulation for the Robotics and Python course. It involves integrating a robot simulation in Webots with real-time control logic running on an ESP32 using MicroPython. The robot is capable of navigating a predefined map using Dijkstra's algorithm for path planning and handles dynamic re-planning upon obstacle detection.

The system demonstrates:

- Path planning using Dijkstra's algorithm
- Serial communication between Webots and ESP32
- Obstacle detection and automatic path re-planning
- Real-time robot movement and debug visualization

## Installation

Before you begin, ensure you have the following software installed:

Webots: (Link to Webots download page, e.g., https://cyberbotics.com/)

Thonny IDE: (Link to Thonny download page, e.g., https://thonny.org/)

## Requirements

### MicroPython (ESP32)

- MicroPython v1.25.0
- Built-in modules: ujson, machine, time, math

### Webots Controller (Python)

- Python 3.10 or later
- Webots R2023a or later
- pyserial module (install with pip install pyserial)

## Setup Instructions

1. Flash the ESP32 with MicroPython and upload the Thonny_code.py file using Thonny IDE or a similar tool.
2. Disconnect any active REPL or serial monitor on the ESP32.
3. In Webots_code.py, make sure the SERIAL_PORT value matches your computer's COM port (e.g., COM3).
4. Open the Webots simulation environment.
5. Run the Webots controller.
6. Press the left button on the ESP32 to start the simulation.

## Features Implemented

Path Planning  
- Implements Dijkstra's algorithm for computing the shortest path  
- Automatically re-plans the route when obstacles are detected

State Machine Control  
- Manages robot behavior using a structured state machine  
- Handles transitions between line-following, intersection handling, turning, and stopping

Obstacle Detection  
- Uses proximity sensors to detect obstacles  
- Updates the grid map and recalculates the path if an obstacle is found

Serial Communication  
- Uses UART serial protocol for bidirectional communication between Webots and the ESP32

Debugging and Visualization  
- Sends debug information (sensor values, current node, intersection counter) to Webots for live feedback


## How to Run the Simulation

1. Upload the ESP32 code and reset the board
2. Close Thonny after oploading the code
3. Press the left button on the ESP32 to initialize communication
4. Run Webots and start the simulation world
5. Run the Python controller script within Webots
6. The robot will compute the path, navigate, and re-plan if obstacles appear


