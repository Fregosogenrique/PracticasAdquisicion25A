# Circuit Diagram and Connection Details

This document describes the hardware connections required for the DC motor control system with quadrature encoder.

## System Block Diagram

```
                                     +----------------+
                                     |                |
                    +------------+   |                |   +---------------+
Computer <--USB--> |  Arduino   |<--|   Motor Driver |<--|  DC Motor     |
(Python GUI)       |  Board     |   |   (L298N)      |   |  with Encoder |
                    +------------+   |                |   +---------------+
                         ^           |                |          |
                         |           +----------------+          |
                         |                                       |
                         +---------------------------------------+
                                    Encoder Feedback
```

## Connection Pinout

### Arduino Connections

| Arduino Pin | Connected To | Purpose |
|-------------|-------------|---------|
| Pin 2 | Encoder Phase A | Interrupt input for encoder pulse counting |
| Pin 3 | Encoder Phase B | Interrupt input for encoder direction detection |
| Pin 7 | Motor Driver Direction 1 | Direction control |
| Pin 8 | Motor Driver Direction 2 | Direction control |
| Pin 9 | Motor Driver PWM | Speed control via PWM |
| 5V | Motor Driver 5V, Encoder VCC | Power for driver logic and encoder |
| GND | Motor Driver GND, Encoder GND | Common ground |

### Motor Driver (L298N) Connections

| L298N Pin | Connected To | Purpose |
|-----------|-------------|---------|
| IN1 | Arduino Pin 7 | Direction control input 1 |
| IN2 | Arduino Pin 8 | Direction control input 2 |
| ENA | Arduino Pin 9 | PWM input for speed control |
| OUT1 | Motor Positive | Motor terminal 1 |
| OUT2 | Motor Negative | Motor terminal 2 |
| 5V | Arduino 5V | Logic power (if jumper is removed) |
| 12V | External Power Supply | Motor power (7-12V depending on motor) |
| GND | Arduino GND, Power Supply GND | Common ground |

### Encoder Connections

| Encoder Pin | Connected To | Purpose |
|-------------|-------------|---------|
| VCC | Arduino 5V | Power for encoder |
| GND | Arduino GND | Ground reference |
| A | Arduino Pin 2 | Quadrature phase A output |
| B | Arduino Pin 3 | Quadrature phase B output |

## Detailed Wiring Diagram

```
                       Arduino                           L298N Motor Driver
                    +------------+                     +----------------+
                    |            |                     |                |
                    |        5V o|--------------------|  5V            |
                    |       GND o|--------------------|  GND           |
                    |            |                     |                |
                    |        D7 o|--------------------|  IN1           |
                    |        D8 o|--------------------|  IN2           |
                    |        D9 o|--------------------|  ENA           |
                    |            |                     |                |
                    |            |                     |  OUT1 o--------)--------------+
                    |            |                     |  OUT2 o--------)--------------+
                    |            |                     |                |              |
                    |            |                     +----------------+              |
                    |            |                                                    |
                    |            |                        DC Motor with Encoder       |
                    |            |                     +----------------+              |
                    |        D2 o|--------------------|  Encoder A      |              |
                    |        D3 o|--------------------|  Encoder B      |              |
                    |       5V o|--+                  |  VCC            |              |
                    |      GND o|--+------------------|  GND            |              |
                    |            |  |                 |                 |              |
                    +------------+  |                 |       Motor     |<-------------+
                                    |                 |       Terminals |<-------------+
                                    |                 |                 |
                                    |                 +----------------+
                                    |
                                    |
                                External
                                Power Supply
                              (7-12V depending
                                 on motor)
```

## Important Notes

1. **Power Supply**: 
   - The motor driver should be powered by an external power supply appropriate for your motor (typically 7-12V).
   - Keep the motor power separate from the Arduino's USB power to prevent voltage drops and noise issues.

2. **Encoder Connection**:
   - Some encoders have pull-up resistors built-in, but if yours doesn't, add 10kΩ pull-up resistors from encoder outputs A and B to 5V.
   - If the encoder doesn't have built-in filtering capacitors, consider adding 0.1μF capacitors from A and B to ground to reduce noise.

3. **Motor Driver Configuration**:
   - If using an L298N, ensure the power jumper is configured correctly (removed if providing separate 5V logic power).
   - The enable pins (ENA/ENB) must be driven with PWM signals to control motor speed.

4. **Shielding and Wire Length**:
   - Keep encoder wires as short as possible and away from motor wires to reduce electromagnetic interference.
   - Consider using shielded cables for the encoder if experiencing noise issues.

5. **Additional Considerations**:
   - For higher precision applications, consider adding optical isolation between the encoder and Arduino.
   - For motors drawing more than 1A, ensure adequate cooling for the motor driver.

