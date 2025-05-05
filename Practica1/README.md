# DC Motor Control System with H-Bridge

## Project Overview

This project implements a DC motor control system using an Arduino connected to an H-bridge circuit for bidirectional motor control. A Python-based graphical user interface provides an intuitive way to control the motor's speed and direction in real-time.

The system offers:
- PWM-based speed control
- Forward/reverse direction control through H-bridge
- Real-time status monitoring
- Emergency stop functionality
- Over-current protection (if supported by the H-bridge)

This project was developed as part of Practica1 for data acquisition and control lab exercises.

## Hardware Requirements

- Arduino Uno or compatible board
- DC motor (5-12V, depending on your power supply)
- H-bridge driver module (e.g., L298N, L293D)
- Power supply suitable for your motor
- USB cable for Arduino connection
- Jumper wires
- (Optional) Current sensing resistor if implementing over-current protection

## Software Dependencies

### Arduino
- Arduino IDE 1.8.x or newer

### Python
- Python 3.6 or newer
- PyQt5
- pyserial

Install Python dependencies with:

```bash
pip install PyQt5 pyserial
```

## Hardware Setup

### Pin Connections

| Arduino Pin | Connection                   | Purpose                     |
|-------------|------------------------------|----------------------------|
| Pin 7       | H-bridge IN1                 | Direction control 1        |
| Pin 8       | H-bridge IN2                 | Direction control 2        |
| Pin 9       | H-bridge EN_A                | PWM speed control          |
| A0          | Current sense (if available) | Over-current protection    |
| 5V          | H-bridge VCC (logic)         | Power for H-bridge logic   |
| GND         | H-bridge GND                 | Common ground              |

### H-bridge Connections

| H-bridge Pin | Connection                          |
|--------------|-------------------------------------|
| VCC          | Arduino 5V (logic power)            |
| GND          | Arduino GND                         |
| IN1          | Arduino Pin 7                       |
| IN2          | Arduino Pin 8                       |
| ENA          | Arduino Pin 9                       |
| OUT1         | Motor terminal 1                    |
| OUT2         | Motor terminal 2                    |
| VS           | External power supply (motor power) |

**Note**: Connect the motor power supply (VS) directly to the H-bridge's power input, not through the Arduino. The Arduino and H-bridge should share a common ground.

### External Power

The motor should be powered by an external power supply connected to the H-bridge's power inputs, not through the Arduino. This prevents damaging the Arduino due to high current draw.

## Software Setup

### Arduino

1. Open the Arduino IDE
2. Load the `Arduino/dc_motor_control.ino` sketch
3. Connect Arduino via USB
4. Select the correct board and port from the Tools menu
5. Upload the sketch to the Arduino

### Python GUI

1. Ensure all dependencies are installed
2. Navigate to the project directory
3. Run the Python GUI with:

```bash
python Interface/motor_control_gui.py
```

## Usage Instructions

### Using the Python GUI

1. **Connection Setup**:
   - Select the appropriate COM port from the dropdown menu
   - Click "Connect" to establish a connection with the Arduino
   - The status indicator will turn green when connected successfully

2. **Motor Control**:
   - Set the direction using the "Forward" or "Reverse" buttons
   - Adjust speed using the slider (0-255)
   - Click "Start" to run the motor, "Stop" to halt it
   - Use the "EMERGENCY STOP" button for immediate halt in case of emergency

3. **Status Monitoring**:
   - Current direction, speed, and operation status are displayed in real-time
   - The log area shows command history and responses from the Arduino

4. **After Emergency Stop**:
   - Use the "Reset E-Stop" button to reset after an emergency stop
   - This doesn't restart the motor but allows it to be started again

### LED Indicators on Arduino (if implemented)

- Solid LED: Motor running
- Blinking LED: Emergency stop activated
- No LED: Motor stopped or in standby

## Troubleshooting

### Common Issues

1. **Cannot connect to Arduino**:
   - Verify the correct COM port is selected
   - Check if Arduino is properly connected via USB
   - Ensure no other program is using the selected port
   - Try unplugging and replugging the Arduino

2. **Motor doesn't respond**:
   - Check all connections between Arduino and H-bridge
   - Verify the H-bridge is receiving appropriate power
   - Ensure the motor connections to the H-bridge are secure
   - Check the Arduino serial monitor for error messages

3. **Erratic motor behavior**:
   - Verify the H-bridge connections are correct
   - Check for loose wires or poor connections
   - Ensure the power supply is adequate for the motor

4. **Communication errors**:
   - Reset both the Arduino and the Python application
   - Check if the correct baud rate (9600) is set in both programs
   - Examine the log area for specific error messages

## Safety Considerations

1. **Power supply**:
   - Never connect the motor power directly to the Arduino
   - Ensure the power supply voltage matches your motor's rating
   - Use an appropriate fuse to protect against short circuits

2. **Emergency stop**:
   - Test the emergency stop functionality before operating
   - Consider adding a physical emergency stop button if needed

3. **Mechanical safety**:
   - Secure the motor properly before testing
   - Keep fingers and loose clothing away from moving parts
   - Consider adding a protective enclosure around moving parts

4. **Electrical safety**:
   - Disconnect power before making any wiring changes
   - Check for short circuits before applying power
   - Use appropriate wire gauge for the motor current

## Extending the Project

This project can be extended in several ways:

1. Add encoder feedback for precise speed control
2. Implement PID control for better speed regulation
3. Add temperature monitoring for the motor and H-bridge
4. Implement SPWM (Sinusoidal PWM) for more efficient motor control
5. Add a web interface for remote control

## License

This project is provided as-is for educational purposes.

## Acknowledgments

- Based on fundamental motor control principles and Arduino-Python communication
- Developed for educational purposes in a data acquisition and control lab context

