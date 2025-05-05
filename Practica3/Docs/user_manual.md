# User Manual - DC Motor Control System

This user manual provides detailed instructions for setting up and operating the DC Motor Control System with PID control and encoder feedback.

## Table of Contents

1. [System Setup](#system-setup)
2. [Arduino Configuration](#arduino-configuration)
3. [MATLAB Simulation](#matlab-simulation)
4. [Python GUI Operation](#python-gui-operation)
5. [Data Logging and Analysis](#data-logging-and-analysis)
6. [Configuration Management](#configuration-management)

## System Setup

### Hardware Assembly

1. **Connect the DC motor to the motor driver**:
   - Connect the motor's positive terminal to the driver's OUT1
   - Connect the motor's negative terminal to the driver's OUT2

2. **Connect the quadrature encoder**:
   - Connect encoder VCC to Arduino 5V
   - Connect encoder GND to Arduino GND
   - Connect encoder output A to Arduino pin 2
   - Connect encoder output B to Arduino pin 3

3. **Connect the motor driver to Arduino**:
   - Connect IN1 to Arduino pin 7
   - Connect IN2 to Arduino pin 8
   - Connect ENA to Arduino pin 9
   - Connect driver GND to Arduino GND
   - Connect driver 5V (logic power) to Arduino 5V (if needed)

4. **Connect external power**:
   - Connect an appropriate power supply (7-12V depending on your motor) to the motor driver
   - **IMPORTANT**: Ensure common ground between Arduino and motor driver power supply

5. **Verify connections** according to the [circuit diagram](circuit_diagram.md) before powering on the system.

### Software Installation

1. **Arduino IDE**:
   - Install the latest Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software)
   - No additional libraries are required for the basic implementation

2. **MATLAB**:
   - Ensure MATLAB with Control System Toolbox is installed
   - No additional toolboxes are required for basic simulation

3. **Python Environment**:
   - Install Python 3.6 or newer
   - Install required packages:
     ```
     pip install pyqt5 pyserial matplotlib numpy
     ```

## Arduino Configuration

1. **Upload the sketch**:
   - Open Arduino IDE
   - Load the `motor_encoder_control.ino` sketch from the Arduino folder
   - Select the correct board and port
   - Upload the sketch to your Arduino

2. **Serial Monitor Testing** (optional):
   - Open the Serial Monitor in Arduino IDE (set baud rate to 115200)
   - You should see initialization messages
   - You can manually test commands:
     - `PID,1.0,0.1,0.01` - set PID parameters
     - `SET,100` - set position/speed setpoint
     - `MOTOR,128` - directly control motor (values -255 to 255)
     - `RESET` - reset encoder position to 0
     - `SETMODE,P` or `SETMODE,S` - set position or speed control mode

## MATLAB Simulation

The MATLAB script provides a simulation environment to determine optimal PID parameters before implementing them on hardware.

1. **Run the simulation**:
   - Open MATLAB
   - Navigate to the MATLAB folder in the project
   - Open and run `motor_pid_simulation.m`

2. **Adjust motor parameters**:
   - Modify the motor parameters at the top of the script to match your specific motor
   - Common parameters to adjust:
     - `J` (moment of inertia)
     - `b` (friction coefficient)
     - `Kt` (torque constant)
     - `Ke` (back-EMF constant)
     - `R` (armature resistance)

3. **PID Tuning**:
   - Adjust the PID parameters in the script to achieve desired response
   - Key parameters to observe:
     - Rise time
     - Settling time
     - Overshoot
     - Steady-state error

4. **Simulation Output**:
   - Examine the generated plots for step response, control signal, and parameter effects
   - Use these insights to set initial PID values for the real system
   - The script automatically saves tuned parameters to `pid_params.csv` for reference

## Python GUI Operation

The Python GUI provides a user-friendly interface for controlling and monitoring the motor system.

### Connection Setup

1. **Starting the application**:
   - Navigate to the project directory in terminal/command prompt
   - Run: `python motor_control_gui.py`

2. **Connect to Arduino**:
   - Select the appropriate COM port from the dropdown menu
   - Set the baud rate to 115200
   - Click "Connect"
   - The status bar should show "Connected" and real-time data should begin to appear

### Control Features

1. **Control Mode Selection**:
   - Select "Position Control" for controlling exact positions
   - Select "Speed Control" for controlling velocity

2. **PID Parameter Adjustment**:
   - Set Kp, Ki, and Kd values based on MATLAB simulation results
   - Click "Apply PID Parameters" to send to Arduino

3. **Setpoint Control**:
   - Enter desired position (in encoder counts) or speed (in counts/second)
   - Click "Apply Setpoint" to send command

4. **Manual Motor Control**:
   - Use the slider to set motor power directly (-255 to 255)
   - Click "Apply Manual" to send command
   - Click "Zero Motor" to stop the motor immediately

5. **Additional Controls**:
   - "Reset Encoder" - Sets encoder position to zero
   - "EMERGENCY STOP" - Immediately halts the motor (safety feature)

### Real-time Visualization

1. **Plot Window**:
   - The top graph shows position in encoder counts
   - The middle graph shows speed in counts per second
   - The bottom graph shows control output (PWM value)

2. **Plot Controls**:
   - Adjust "Plot Window" value to change time scale (5-60 seconds)
   - Plots auto-scale to fit the data range

## Data Logging and Analysis

1. **Data Logging**:
   - Click "Start Logging" to begin recording data
   - The status will show the number of data points collected
   - Click "Stop Logging" to end recording

2. **Exporting Data**:
   - Click "Export Data" to save logged data
   - Choose a location and filename in the dialog
   - Data is saved in CSV format with columns:
     - time (seconds since start)
     - timestamp (Arduino millisecond count)
     - position (encoder counts)
     - speed (counts per second)
     - setpoint
     - error (difference between setpoint and measurement)
     - control (PWM output)
     - kp, ki

