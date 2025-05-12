# Stepper Motor Control System

## Project Overview

This project provides a complete system for controlling a stepper motor via an M542C driver using an Arduino and a Python GUI interface. The system allows control of three axes (X, Y, and R) with features including:

- Speed control
- Direction control
- Position tracking
- Real-time status updates
- Emergency stop
- Multi-axis support

The system consists of two main components:
1. **Arduino Firmware**: Controls the M542C stepper motor driver with precise timing
2. **Python GUI Application**: Provides a user-friendly interface for motor control

## Hardware Setup

### Components Required

- Arduino board (Uno, Mega, or similar)
- M542C stepper motor driver
- Stepper motor (compatible with M542C)
- Power supply for the M542C driver (typically 24-48V DC)
- USB cable for Arduino-computer connection
- Appropriate power and signal wiring
- Potentiometers or analog sensors for X, Y, and R axis inputs (optional)

### M542C Driver Connections

Connect the M542C driver to the Arduino as follows:

| M542C Pin | Arduino Pin | Description |
|-----------|-------------|-------------|
| PUL+ | 3 | Step pulse signal |
| PUL- | GND | Step pulse ground |
| DIR+ | 2 | Direction signal |
| DIR- | GND | Direction ground |
| ENA+ | 4 | Enable signal |
| ENA- | GND | Enable ground |

### Axis Input Connections

Connect analog inputs for the three axes:

| Input | Arduino Pin | Description |
|-------|-------------|-------------|
| X-axis | A0 | X-axis position/speed input |
| Y-axis | A1 | Y-axis position/speed input |
| R-axis | A2 | R-axis position/speed input |

### Power Connections

- The Arduino can be powered via USB or using an external power supply
- The M542C driver requires a separate power supply (24-48V DC depending on your motor)
- Ensure all grounds are connected to establish a common reference

## Software Requirements

### Arduino

- Arduino IDE (1.8.x or newer)
- Standard Arduino libraries (no additional libraries required)

### Python

- Python 3.6 or newer
- Required Python packages:
  - `tkinter` (included with standard Python installation)
  - `pyserial` (for serial communication)

## Installation

### Arduino Setup

1. Connect your Arduino to your computer
2. Open the Arduino IDE
3. Open the `arduino/stepper_control.ino` file
4. Select the correct board and port from the Tools menu
5. Click Upload to install the firmware on the Arduino

### Python Setup

1. Ensure Python 3.6+ is installed on your system
2. Install the required Python packages:
   ```
   pip install pyserial
   ```
3. No additional setup is needed as Tkinter is included with Python

## Usage Instructions

### Starting the Application

1. Connect the Arduino to your computer via USB
2. Navigate to the project directory
3. Run the Python GUI application:
   ```
   cd python
   python motor_control_gui.py
   ```

### Using the GUI

The GUI is divided into three main sections:

#### Connection Control

- **Serial Port**: Select the correct port from the dropdown
- **Refresh**: Update the list of available ports
- **Connect/Disconnect**: Connect to or disconnect from the Arduino
- **Emergency Stop**: Immediately stops the motor

#### Axis Controls

For each axis (X, Y, R):
1. **Select Axis**: Click the "Select X/Y/R" button to make this the active axis
2. **Speed Control**: Adjust the speed using the slider or by entering a value
3. **Direction**: Toggle between forward and reverse
4. **Position**: View the current position
5. **Input Value**: Shows the current analog input value for the axis

#### Status Display

Shows the current state of the system:
- Connection status
- Active axis
- Motor state (enabled/disabled)
- Current speed
- Current direction

### Basic Operation

1. Connect to the Arduino using the Connect button
2. Select the axis you want to control
3. Set the desired speed using the slider
4. Set the direction (Forward/Reverse)
5. The motor will begin rotating at the specified speed and direction
6. Use the Emergency Stop button if you need to stop immediately

## Troubleshooting

### Arduino Connection Issues

- **Arduino not found in port list**:
  - Ensure the Arduino is connected properly to your computer
  - Check if drivers are installed (especially for clone Arduino boards)
  - Try a different USB cable or port

- **Cannot connect to selected port**:
  - Ensure no other application is using the port
  - Check if you have permission to access the port
  - Restart the application and/or computer

### Motor Control Issues

- **Motor doesn't move**:
  - Check all wiring connections
  - Verify power supply is connected and turned on
  - Ensure the Enable pin is properly connected
  - Check the motor is not disabled in software

- **Motor moves erratically**:
  - Check for loose connections
  - Verify the proper microstepping settings on the M542C
  - Ensure the power supply provides adequate current

- **Unexpected direction**:
  - Reverse the motor wires or change the direction in software

### Software Issues

- **GUI doesn't start**:
  - Check Python installation and version
  - Verify all required packages are installed
  - Check console for error messages

- **Status not updating**:
  - Check serial connection
  - Verify Arduino is running the correct firmware
  - Restart the application

## Additional Information

### Pin Configuration

The default pin configuration in the Arduino sketch is:
- Step Pulse: Pin 3
- Direction: Pin 2
- Enable: Pin 4
- X-axis Input: A0
- Y-axis Input: A1
- R-axis Input: A2

These can be modified in the Arduino code if needed.

### Communication Protocol

The Arduino and Python application communicate using a simple text-based protocol:
- Commands from Python to Arduino: `<axis>,<speed>,<direction>`
  - Example: `X,1000,1` (X-axis, 1000 steps/sec, forward direction)
- Status updates from Arduino to Python: `STATUS,<axis_index>,<enabled>,<direction>,<speed>,<position>,<x_value>,<y_value>,<r_value>`

## License

This project is provided as-is for educational purposes.

## Acknowledgments

- Thanks to the Arduino and Python communities for their excellent documentation and libraries.

