# Theory of Operation

This document explains the theoretical principles behind the DC motor control system with PID and quadrature encoder feedback.

## Table of Contents
1. [PID Control Theory](#pid-control-theory)
2. [Quadrature Encoder Principles](#quadrature-encoder-principles)
3. [DC Motor Characteristics](#dc-motor-characteristics)
4. [System Modeling](#system-modeling)
5. [Digital Implementation](#digital-implementation)

## PID Control Theory

The Proportional-Integral-Derivative (PID) controller is one of the most widely used control algorithms in industry. It calculates an error value as the difference between a measured process variable and a desired setpoint, then applies a correction based on proportional, integral, and derivative terms.

### Basic PID Equation

The PID controller output is calculated using the following equation:

```
u(t) = Kp * e(t) + Ki * ∫e(τ)dτ + Kd * de(t)/dt
```

Where:
- `u(t)` is the control signal
- `e(t)` is the error (setpoint - measured value)
- `Kp` is the proportional gain
- `Ki` is the integral gain
- `Kd` is the derivative gain

### The Three Terms

1. **Proportional Term** (`Kp * e(t)`)
   - Produces an output proportional to the current error
   - Reduces rise time and steady-state error
   - Higher values increase response speed but may cause overshoot
   - Too high can cause instability

2. **Integral Term** (`Ki * ∫e(τ)dτ`)
   - Accounts for accumulated past errors
   - Eliminates steady-state error for constant setpoints
   - Can cause "windup" if the actuator saturates
   - Too high can cause oscillation

3. **Derivative Term** (`Kd * de(t)/dt`)
   - Based on the rate of error change
   - Improves settling time and stability
   - Reduces overshoot
   - Sensitive to noise and can amplify it
   - Often implemented with filtering

### PID Tuning

Tuning involves adjusting the three parameters to achieve optimal control performance:
- **Ziegler-Nichols Method**: Experimental method that sets Kp to cause sustained oscillations, then derives the other parameters
- **Manual Tuning**: Iteratively adjusting parameters and observing system response
- **Model-Based Tuning**: Using a mathematical model of the system to derive parameters
- **Auto-Tuning**: Automated algorithms that determine parameters based on system response

The MATLAB script in our project uses model-based tuning to help find appropriate PID values.

## Quadrature Encoder Principles

A quadrature encoder provides position and direction feedback by generating two out-of-phase signals as the shaft rotates.

### Basic Operation

1. **Physical Structure**:
   - The encoder consists of a disk with alternating transparent and opaque segments
   - Two optical sensors (A and B) are positioned to detect light passing through the disk
   - The sensors are placed at a phase offset of 90° (hence "quadrature")

2. **Signal Generation**:
   - As the disk rotates, each sensor generates a square wave
   - The two signals (A and B) are 90° out of phase
   - This phase relationship enables direction detection

3. **Position and Direction**:
   - The number of pulses corresponds to angular position
   - The phase relationship between A and B determines direction:
     - If A leads B (A rises before B), rotation is in one direction
     - If B leads A (B rises before A), rotation is in the opposite direction

4. **Resolution**:
   - Basic resolution is determined by the number of segments on the disk
   - Resolution can be multiplied by detecting both rising and falling edges (2x)
   - Detecting all edges of both signals provides 4x resolution (quadrature decoding)

### Quadrature Decoding

Our Arduino implementation uses interrupt-based quadrature decoding:
1. Changes on either channel trigger an interrupt
2. The state of both channels is read
3. Position counter is incremented or decremented based on the state transition

## DC Motor Characteristics

DC motors convert electrical energy to mechanical rotation through electromagnetic principles.

### Basic Motor Equations

1. **Electrical Characteristics**:
   ```
   V = IR + Ke * ω
   ```
   Where:
   - `V` is the applied voltage
   - `I` is the current
   - `R` is the armature resistance
   - `Ke` is the back-EMF constant
   - `ω` is the angular velocity

2. **Mechanical Characteristics**:
   ```
   T = Kt * I
   T = J * dω/dt + b * ω + TL
   ```
   Where:
   - `T` is the motor torque
   - `Kt` is the torque constant
   - `J` is the rotor inertia
   - `b` is the viscous friction coefficient
   - `TL` is the load torque

### Motor Behavior

1. **Speed Control**:
   - Motor speed is approximately proportional to applied voltage
   - Under load, speed decreases (due to increased current and voltage drop)
   - PWM is used to control average voltage and thus speed

2. **Torque-Speed Relationship**:
   - Maximum torque at zero speed (stall torque)
   - Torque decreases linearly as speed increases
   - Maximum speed at zero torque (no-load speed)

3. **Efficiency**:
   - Maximum efficiency typically at 70-80% of no-load speed
   - Efficiency drops at very low and very high speeds

## System Modeling

The DC motor with encoder can be modeled as a dynamic system for control design.

### Transfer Function

The transfer function relating input voltage to output position is:

```
G(s) = θ(s)/V(s) = Kt/[s * (J*s + b) * (L*s + R) + Kt*Ke]
```

For velocity output:

```
G(s) = ω(s)/V(s) = Kt/[(J*s + b) * (L*s + R) + Kt*Ke]
```

### State-Space Representation

For motor position and velocity:

```
dx/dt = Ax + Bu
y = Cx

Where:
x = [θ, ω, i]^T  (position, velocity, current)
```

The matrices A, B, C depend on motor parameters J, b, Kt, Ke, R, and L.

### Simplified Model

For control design, often a simplified second-order model is used:

```
G(s) = K / [s * (τ*s + 1)]
```

Where:
- K is the steady-state gain
- τ is the mechanical time constant

This simplification works because electrical dynamics are typically much faster than mechanical dynamics.

## Digital Implementation

Converting continuous PID control to discrete-time implementation requires several considerations.

### Discrete PID Algorithm

The discrete-time PID implementation used in our Arduino code:

```
error = setpoint - input
integral += error
derivative = error - lastError
output = Kp * error + Ki * integral + Kd * derivative
lastError = error
```

### Sampling Considerations

1. **Sampling Rate**:
   - Must be fast enough to capture system dynamics (rule of thumb: 10x bandwidth)
   - Our implementation uses a fast main loop with timer-based sampling

2. **Integral Windup Protection**:
   - The integral term is limited to prevent excessive accumulation
   - Implemented via clamping: `if (integral > maxValue) integral = maxValue`

3. **Derivative Filtering**:
   - Derivatives amplify noise
   - A low-pass filter is applied to the derivative term

4. **PWM Control**:
   - Arduino generates PWM signals to control motor voltage
   - 8-bit PWM provides values from 0-255
   - Direction control requires two digital pins

### Encoder Interrupt Handling

Accurate position tracking requires interrupt-based encoder reading:
1. External interrupts trigger on signal edge detection
2. Interrupt service routines update position counter
3. Main loop calculates velocity from position changes

### Communication Protocol

The system uses a simple serial protocol for communication between Arduino and the Python GUI:
1. Commands are sent as ASCII strings followed by newline
2. Data is sent from Arduino in CSV format with specific fields
3. Periodic updates ensure real-time visualization

The protocol balances simplicity with efficiency for real-time control.

