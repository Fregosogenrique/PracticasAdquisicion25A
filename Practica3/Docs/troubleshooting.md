# Troubleshooting Guide

This guide helps identify and resolve common issues with the DC motor control system. Follow the step-by-step procedures to diagnose and fix problems.

## Table of Contents
1. [Hardware Issues](#hardware-issues)
2. [Software Problems](#software-problems)
3. [Communication Errors](#communication-errors)
4. [Control System Problems](#control-system-problems)
5. [Debugging Procedures](#debugging-procedures)

## Hardware Issues

### Motor Does Not Turn

**Symptoms:**
- Motor doesn't move when commands are sent
- No motor noise or movement

**Possible Causes and Solutions:**

1. **Power Issues:**
   - Check external power supply is connected to motor driver
   - Verify power supply voltage (7-12V depending on motor)
   - Measure voltage at motor driver input terminals

2. **Wiring Problems:**
   - Verify motor connections to driver outputs
   - Check driver control pins (IN1, IN2, ENA) connections to Arduino
   - Ensure common ground between Arduino and motor driver

3. **Driver Issues:**
   - Check if driver is getting hot (possible short circuit)
   - Verify driver configuration (jumper settings)
   - Test driver with direct connections to bypass Arduino

4. **Motor Problems:**
   - Disconnect motor and test with direct power connection
   - Check motor for physical obstruction
   - Measure motor winding resistance (should not be open circuit)

### Encoder Not Responding

**Symptoms:**
- No position change detected when motor turns
- Erratic position readings

**Possible Causes and Solutions:**

1. **Connection Issues:**
   - Verify encoder A and B connections to Arduino pins 2 and 3
   - Check encoder power connections (5V and GND)
   - Ensure encoder is properly mounted to motor shaft

2. **Signal Problems:**
   - Use oscilloscope to verify encoder signals while manually rotating
   - Add pull-up resistors (10kΩ) if signals are weak
   - Add decoupling capacitors to reduce noise

3. **Mechanical Issues:**
   - Check encoder disk for damage or contamination
   - Verify encoder mounting alignment
   - Ensure encoder shaft is properly coupled to motor

## Software Problems

### Arduino Upload Failures

**Symptoms:**
- Error messages during sketch upload
- Upload appears successful but system doesn't function

**Possible Causes and Solutions:**

1. **Port Selection:**
   - Verify correct board type selected in Arduino IDE
   - Check correct port selected in Arduino IDE
   - Try different USB cable

2. **Compilation Errors:**
   - Check error messages in Arduino IDE
   - Verify all required libraries are installed
   - Look for syntax errors in code

3. **Upload Issues:**
   - Press reset button on Arduino just before upload
   - Check if bootloader is functioning
   - Try slower upload speed (Tools > Upload Speed)

### Python GUI Won't Start

**Symptoms:**
- Error message when starting application
- GUI starts but crashes immediately

**Possible Causes and Solutions:**

1. **Installation Issues:**
   - Verify Python and required packages are installed
   - Check package versions match requirements
   - Try reinstalling packages: `pip install --upgrade pyqt5 pyserial matplotlib numpy`

2. **Path Problems:**
   - Ensure running from correct directory
   - Check file paths in code
   - Verify file permissions

3. **Compatibility Issues:**
   - Test with different Python version
   - Check OS compatibility
   - Run from terminal to see detailed error messages

### MATLAB Simulation Errors

**Symptoms:**
- Errors when running MATLAB script
- Simulation produces unexpected results

**Possible Causes and Solutions:**

1. **Toolbox Issues:**
   - Verify Control System Toolbox is installed
   - Check MATLAB version compatibility
   - Ensure all required functions are available

2. **Parameter Problems:**
   - Check for reasonable parameter values
   - Look for division by zero or other mathematical issues
   - Verify units are consistent

## Communication Errors

### Cannot Connect to Arduino

**Symptoms:**
- "Connection Error" message in GUI
- No response from Arduino

**Possible Causes and Solutions:**

1. **Port Issues:**
   - Refresh port list in GUI
   - Verify Arduino is listed in available ports
   - Check if other applications are using the port
   - On Windows, check Device Manager for COM port issues
   - On Mac/Linux, check permissions on /dev/tty*

2. **Baud Rate Mismatch:**
   - Ensure GUI baud rate matches Arduino sketch (115200)
   - Try different baud rates if uncertain

3. **Arduino State:**
   - Press reset button on Arduino
   - Check if Arduino is in bootloader mode (blinking L LED)
   - Verify power to Arduino (PWR LED should be on)

### Data Communication Problems

**Symptoms:**
- Connected but no data received
- Garbled or incomplete data

**Possible Causes and Solutions:**

1. **Protocol Issues:**
   - Check Arduino serial output format matches GUI expectations
   - Verify command format sent from GUI
   - Monitor serial traffic with Serial Monitor or another tool

2. **Timing Problems:**
   - Reduce data transmission rate
   - Check for buffer overflows
   - Verify handshaking if implemented

3. **Noise and Interference:**
   - Ensure USB cable is away from motor and power lines
   - Try shorter USB cable
   - Add ferrite beads to USB cable

## Control System Problems

### Motor Oscillates or Unstable

**Symptoms:**
- Motor continuously oscillates around setpoint
- System becomes unstable at higher gains

**Possible Causes and Solutions:**

1. **PID Tuning Issues:**
   - Reduce Kp if oscillations occur
   - Decrease Ki if system has slow oscillations
   - Increase Kd to dampen oscillations
   - Start with P-only control (Ki=0, Kd=0), then add I and D

2. **Mechanical Problems:**
   - Check for mechanical play or backlash
   - Ensure system is not overloaded
   - Verify motor and encoder are firmly mounted

3. **Electrical Issues:**
   - Check for noise in encoder signals
   - Ensure power supply can handle current demands
   - Add capacitors to smooth power supply

### Poor Control Performance

**Symptoms:**
- Slow response to setpoint changes
- Persistent steady-state error
- Excessive overshoot

**Possible Causes and Solutions:**

1. **PID Parameter Adjustment:**
   - Increase Kp for faster response (but watch for oscillations)
   - Increase Ki to eliminate steady-state error
   - Adjust Kd to control overshoot
   - Use MATLAB simulation to find better parameters

2. **System Limitations:**
   - Check if motor has enough power for the application
   - Verify power supply voltage is adequate
   - Consider mechanical constraints (friction, inertia)

3. **Implementation Issues:**
   - Check control loop timing
   - Verify calculations in PID implementation
   - Consider derivative filtering if noise is an issue

## Debugging Procedures

### Systematic Debugging Approach

1. **Isolate the Problem:**
   - Determine if issue is hardware, software, or communication
   - Test components individually when possible
   - Simplify the system to minimum configuration

2. **Hardware Debugging:**
   - Measure voltages at key points
   - Use LED indicators for visual

