# DC Motor Control System with PID and Quadrature Encoder

## Overview

This Practice 3 project implements a complete DC motor control system with position and velocity feedback using a quadrature encoder. The system consists of:

1. **Arduino Implementation**: Handles encoder reading, PID control calculations, and motor control
2. **MATLAB Simulation**: Provides system modeling, PID tuning, and performance analysis
3. **Python GUI Interface**: Offers real-time monitoring, control, and data visualization capabilities

This comprehensive solution demonstrates practical implementation of control theory concepts, data acquisition techniques, and human-machine interface design.

## System Requirements

### Hardware
- Arduino board (Uno, Mega, or compatible)
- DC motor with quadrature encoder
- Motor driver (L298N or similar)
- Jumper wires and breadboard
- USB cable for connecting Arduino to computer

### Software
- Arduino IDE (1.8.x or newer)
- MATLAB (R2018a or newer) with Control System Toolbox
- Python 3.6+ with the following packages:
  - PyQt5
  - pyserial
  - matplotlib
  - numpy

## Quick Start Guide

1. **Hardware Setup**
   - Connect the hardware components according to the [circuit diagram](circuit_diagram.md)
   - Ensure the motor and encoder are properly mounted

2. **Software Installation**
   - Install Arduino IDE and upload the sketch from the `Arduino/motor_encoder_control` folder
   - Install required Python packages: `pip install pyqt5 pyserial matplotlib numpy`

3. **Running the System**
   - Upload the Arduino sketch to your board
   - Run the MATLAB simulation to determine optimal PID parameters
   - Start the Python GUI: `python motor_control_gui.py`
   - Connect to the Arduino through the GUI and begin control operations

## Directory Structure

```
Practica3/
├── Arduino/
│   └── motor_encoder_control/
│       └── motor_encoder_control.ino
├── MATLAB/
│   ├── motor_pid_simulation.m
│   ├── pid_params.csv
│   └── pid_params.mat
├── Docs/
│   ├── README.md
│   ├── circuit_diagram.md
│   ├── user_manual.md
│   ├── theory_of_operation.md
│   └── troubleshooting.md
└── motor_control_gui.py
```

## Documentation

- [Circuit Diagram and Pinout](circuit_diagram.md): Hardware connections and pin assignments
- [User Manual](user_manual.md): Detailed usage instructions for the entire system
- [Theory of Operation](theory_of_operation.md): Technical explanation of PID control and quadrature encoders
- [Troubleshooting Guide](troubleshooting.md): Common issues and solutions

## License

This project is provided for educational purposes as part of the Data Acquisition and Control course.

## Acknowledgements

Special thanks to:
- The Arduino community for open-source libraries and examples
- The MATLAB and Python developer communities

