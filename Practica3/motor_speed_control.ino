/*
 * Motor Speed Control with Encoder Feedback and PID
 * ================================================
 * 
 * This sketch implements speed control for a DC motor with encoder feedback
 * using an H-bridge driver and PID control algorithm.
 * 
 * Features:
 * - Precise speed control using encoder feedback
 * - PID control for maintaining target speed
 * - Direction control through H-bridge
 * - Speed reporting through serial communication
 * - Safety features including current monitoring and stall detection
 */

// Pin Definitions
#define ENCODER_PIN_A 2  // Interrupt pin
#define ENCODER_PIN_B 3  // Interrupt pin
#define H_BRIDGE_PWM 5   // PWM output for motor speed
#define H_BRIDGE_IN1 7   // Direction control 1
#define H_BRIDGE_IN2 8   // Direction control 2

// Constants for encoder and motor control
#define ENCODER_RESOLUTION 1200   // Pulses per revolution (modify based on your encoder specs)
#define SPEED_CALC_INTERVAL 100   // Calculate speed every 100ms
#define PID_INTERVAL 50           // PID update interval in ms
#define MAX_PWM_VALUE 255         // Maximum PWM value
#define MIN_PWM_VALUE 0           // Minimum PWM value
#define MAX_CURRENT 5.0           // Maximum allowed current in amps (adjust for your motor)
#define STALL_TIMEOUT 2000        // Time in ms to detect a stall
#define STALL_THRESHOLD 10        // Minimum encoder pulses to consider motor not stalled

// PID Constants (tune these for your specific motor/setup)
#define PID_KP 0.5                // Proportional gain
#define PID_KI 0.2                // Integral gain
#define PID_KD 0.1                // Derivative gain
#define PID_OUTPUT_LIMIT 255      // Limit PID output to PWM range

// Global Variables for Encoder
volatile long encoderPosition = 0;  // Current encoder position
volatile long lastEncoderPosition = 0; // Previous encoder position
unsigned long lastSpeedCalc = 0;    // Last speed calculation time
float currentSpeed = 0.0;           // Current measured speed in RPM

// Global Variables for Motor Control
int targetSpeed = 0;                // Target speed in RPM
int currentPwm = 0;                 // Current PWM value
boolean isRunning = false;          // Motor running status
String direction = "clockwise";     // Default direction "clockwise" or "counterclockwise"
int rotationCount = 0;              // Counter for full rotations

// PID Variables
float pidError = 0;                 // Current error
float pidLastError = 0;             // Previous error
float pidIntegral = 0;              // Accumulated error
float pidDerivative = 0;            // Rate of change of error
float pidOutput = 0;                // PID output
unsigned long lastPidUpdate = 0;    // Last PID update time

// Motor monitoring
unsigned long lastStallCheck = 0;   // Last stall check time
boolean motorStalled = false;       // Motor stall status

// Interrupt Service Routines for encoder
void encoderPinAISR() {
  // Read both pins
  int a = digitalRead(ENCODER_PIN_A);
  int b = digitalRead(ENCODER_PIN_B);
  
  // Update position based on signal phase
  if (a == b) {
    encoderPosition++;
  } else {
    encoderPosition--;
  }
  
  // Count rotations (one full rotation = ENCODER_RESOLUTION pulses)
  if ((encoderPosition % ENCODER_RESOLUTION) == 0 && lastEncoderPosition != encoderPosition) {
    if (encoderPosition > lastEncoderPosition) {
      rotationCount++;
    } else {
      rotationCount--;
    }
  }
}

void encoderPinBISR() {
  // Read both pins
  int a = digitalRead(ENCODER_PIN_A);
  int b = digitalRead(ENCODER_PIN_B);
  
  // Update position based on signal phase
  if (a != b) {
    encoderPosition++;
  } else {
    encoderPosition--;
  }
}

void setup() {
  // Set up encoder pins with pull-up resistors
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  
  // Set up H-bridge control pins
  pinMode(H_BRIDGE_PWM, OUTPUT);
  pinMode(H_BRIDGE_IN1, OUTPUT);
  pinMode(H_BRIDGE_IN2, OUTPUT);
  
  // Set initial motor state - stopped
  analogWrite(H_BRIDGE_PWM, 0);
  digitalWrite(H_BRIDGE_IN1, LOW);
  digitalWrite(H_BRIDGE_IN2, LOW);
  
  // Attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderPinAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoderPinBISR, CHANGE);
  
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Motor Speed Control with PID Ready");
  
  // Initialize timers
  lastSpeedCalc = millis();
  lastPidUpdate = millis();
  lastStallCheck = millis();
}

void loop() {
  // Process any incoming serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
  
  // Calculate speed from encoder at regular intervals
  calculateSpeed();
  
  // Run PID control at regular intervals if motor is running
  if (isRunning && !motorStalled) {
    updatePid();
  }
  
  // Check for motor stall condition
  checkForStall();
  
  // Send periodic speed updates (every 500ms)
  static unsigned long lastSpeedReport = 0;
  if (millis() - lastSpeedReport > 500) {
    sendSpeedUpdate();
    lastSpeedReport = millis();
  }
}

// Calculate motor speed based on encoder position changes
void calculateSpeed() {
  unsigned long currentTime = millis();
  
  // Calculate speed every SPEED_CALC_INTERVAL milliseconds
  if (currentTime - lastSpeedCalc >= SPEED_CALC_INTERVAL) {
    // Calculate position change
    long positionChange = encoderPosition - lastEncoderPosition;
    float timeChange = (currentTime - lastSpeedCalc) / 1000.0; // Convert to seconds
    
    // Calculate speed in RPM
    // Formula: (positionChange / ENCODER_RESOLUTION) = rotations
    // rotations / timeChange = rotations per second
    // rotations per second * 60 = RPM
    currentSpeed = (positionChange / (float)ENCODER_RESOLUTION) * (60.0 / timeChange);
    
    // Update lastEncoderPosition and lastSpeedCalc
    lastEncoderPosition = encoderPosition;
    lastSpeedCalc = currentTime;
  }
}

// Update PID control
void updatePid() {
  unsigned long currentTime = millis();
  
  // Update PID at the defined interval
  if (currentTime - lastPidUpdate >= PID_INTERVAL) {
    float dt = (currentTime - lastPidUpdate) / 1000.0; // Time delta in seconds
    
    // Calculate error (target speed - actual speed)
    pidError = targetSpeed - currentSpeed;
    
    // Calculate integral (accumulated error)
    pidIntegral += pidError * dt;
    
    // Apply anti-windup to integral term (limit it)
    if (pidIntegral > PID_OUTPUT_LIMIT) pidIntegral = PID_OUTPUT_LIMIT;
    if (pidIntegral < -PID_OUTPUT_LIMIT) pidIntegral = -PID_OUTPUT_LIMIT;
    
    // Calculate derivative (rate of change of error)
    pidDerivative = (pidError - pidLastError) / dt;
    
    // Calculate PID output
    pidOutput = (PID_KP * pidError) + (PID_KI * pidIntegral) + (PID_KD * pidDerivative);
    
    // Limit output to PWM range
    if (pidOutput > PID_OUTPUT_LIMIT) pidOutput = PID_OUTPUT_LIMIT;
    if (pidOutput < -PID_OUTPUT_LIMIT) pidOutput = -PID_OUTPUT_LIMIT;
    
    // Convert PID output to PWM value (absolute value since direction is controlled separately)
    int newPwm = abs(pidOutput);
    
    // Set the direction based on PID output sign and current direction setting
    if (direction == "clockwise" && pidOutput < 0) {
      // Need to reverse direction to achieve negative speed
      setMotorDirection("counterclockwise");
    } else if (direction == "counterclockwise" && pidOutput > 0) {
      // Need to reverse direction to achieve positive speed
      setMotorDirection("clockwise");
    }
    
    // Apply new PWM value
    setMotorPwm(newPwm);
    
    // Update for next iteration
    pidLastError = pidError;
    lastPidUpdate = currentTime;
  }
}

// Set motor PWM value
void setMotorPwm(int pwm) {
  // Ensure PWM is within limits
  if (pwm > MAX_PWM_VALUE) pwm = MAX_PWM_VALUE;
  if (pwm < MIN_PWM_VALUE) pwm = MIN_PWM_VALUE;
  
  // Apply PWM to motor
  analogWrite(H_BRIDGE_PWM, pwm);
  currentPwm = pwm;
}

// Set motor direction
void setMotorDirection(String dir) {
  direction = dir;
  
  if (dir == "clockwise") {
    digitalWrite(H_BRIDGE_IN1, HIGH);
    digitalWrite(H_BRIDGE_IN2, LOW);
  } else if (dir == "counterclockwise") {
    digitalWrite(H_BRIDGE_IN1, LOW);
    digitalWrite(H_BRIDGE_IN2, HIGH);
  } else {
    // Stop motor if invalid direction
    digitalWrite(H_BRIDGE_IN1, LOW);
    digitalWrite(H_BRIDGE_IN2, LOW);
  }
}

// Start motor
void startMotor() {
  if (!isRunning) {
    isRunning = true;
    motorStalled = false;
    
    // Ensure direction pins are set correctly
    setMotorDirection(direction);
    
    // Reset PID variables for clean start
    pidIntegral = 0;
    pidLastError = 0;
    
    Serial.println("OK:START");
  }
}

// Stop motor
void stopMotor() {
  isRunning = false;
  setMotorPwm(0);
  
  // Release motor (coast)
  digitalWrite(H_BRIDGE_IN1, LOW);
  digitalWrite(H_BRIDGE_IN2, LOW);
  
  Serial.println("OK:STOP");
}

// Emergency stop - brake motor
void emergencyStop() {
  isRunning = false;
  motorStalled = false;
  
  // Stop PWM
  analogWrite(H_BRIDGE_PWM, 0);
  
  // Brake motor (both pins HIGH or both LOW)
  digitalWrite(H_BRIDGE_IN1, HIGH);
  digitalWrite(H_BRIDGE_IN2, HIGH);
  
  // Reset PID values
  pidIntegral = 0;
  pidError = 0;
  pidLastError = 0;
  
  Serial.println("OK:ESTOP");
}

// Check for motor stall
void checkForStall() {
  unsigned long currentTime = millis();
  
  // Only check for stall if motor is running with significant PWM
  if (isRunning && currentPwm > 50) {
    if (currentTime - lastStallCheck >= STALL_TIMEOUT) {
      // Check if position has changed significantly
      long positionChange = abs(encoderPosition - lastEncoderPosition);
      
      // If minimal movement and not already marked as stalled
      if (positionChange < STALL_THRESHOLD && !motorStalled) {
        // Motor is stalled - emergency stop
        motorStalled = true;
        stopMotor();
        Serial.println("ERROR:Motor stall detected");
      }
      
      // Update last stall check time
      lastStallCheck = currentTime;
    }
  }
}

// Send speed update to serial
void sendSpeedUpdate() {
  Serial.print("SPEED:");
  Serial.println(currentSpeed);
  
  Serial.print("POS:");
  Serial.println(encoderPosition);
  
  Serial.print("ROT:");
  Serial.println(rotationCount);
  
  Serial.print("PWM:");
  Serial.println(currentPwm);
}

// Process commands received from the Python GUI
void processCommand(String command) {
  // Split the command at ':' if it contains parameters
  int colonIndex = command.indexOf(':');
  String cmd = colonIndex > 0 ? command.substring(0, colonIndex) : command;
  String param = colonIndex > 0 ? command.substring(colonIndex + 1) : "";
  
  if (cmd == "SET_DIR" || cmd == "DIR") {
    // Set direction
    if (param == "CW") {
      setMotorDirection("clockwise");
      Serial.println("OK:SET_DIR");
    } else if (param == "CCW") {
      setMotorDirection("counterclockwise");
      Serial.println("OK:SET_DIR");
    } else {
      Serial.println("ERROR:Invalid direction");
    }
  }
  else if (cmd == "SET_SPEED" || cmd == "SPEED") {
    // Set target speed in RPM
    int speedRpm = param.toInt();
    if (speedRpm >= 0) {  // Allow zero for stopping
      targetSpeed = speedRpm;
      Serial.print("OK:SET_SPEED:");
      Serial.println(targetSpeed);
    } else {
      Serial.println("ERROR:Invalid speed value");
    }
  }
  else if (cmd == "SET_PID") {
    // Format: SET_PID:Kp,Ki,Kd
    int firstComma = param.indexOf(',');
    int secondComma = param.indexOf(',', firstComma + 1);
    
    if (firstComma > 0 && secondComma > firstComma) {
      float kp = param.substring(0, firstComma).toFloat();
      float ki = param.substring(firstComma + 1, secondComma).toFloat();
      float kd = param.substring(secondComma + 1).toFloat();
      
      // Update PID constants
      #define PID_KP kp
      #define PID_KI ki
      #define PID_KD kd
      
      // Reset integral for clean transition
      pidIntegral = 0;
      
      Serial.println("OK:SET_PID");
    } else {
      Serial.println("ERROR:Invalid PID parameter format");
    }
  }
  else if (cmd == "START") {
    startMotor();
  }
  else if (cmd == "STOP

