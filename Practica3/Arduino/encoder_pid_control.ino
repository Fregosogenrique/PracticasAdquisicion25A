/*
 * Encoder-based PID Motor Control
 * ==============================
 * This sketch implements a PID control system for a DC motor using quadrature encoder feedback.
 * It communicates with MATLAB to receive control parameters and send real-time data.
 * 
 * Features:
 * - Interrupt-based quadrature encoder reading
 * - PID control implementation
 * - Serial communication protocol for MATLAB interface
 * - Motor direction and speed control
 * - Safety measures for motor protection
 */

// Pin Definitions
// Encoder pins - must use interrupt pins (2 and 3 on most Arduino boards)
#define ENCODER_A_PIN 2     // Encoder channel A
#define ENCODER_B_PIN 3     // Encoder channel B

// Motor control pins
#define MOTOR_PWM_PIN 9     // PWM output for motor speed
#define MOTOR_DIR_PIN_1 7   // Direction control 1
#define MOTOR_DIR_PIN_2 8   // Direction control 2
#define MOTOR_ENABLE_PIN 6  // Enable pin for the motor driver (optional)

// Optional safety pins
#define CURRENT_SENSE_PIN A0 // Analog input for current sensing
#define EMERGENCY_STOP_PIN 4 // Digital input for external emergency stop button

// Constants and Configuration
#define SERIAL_BAUD_RATE 115200      // Higher baud rate for faster communication with MATLAB
#define ENCODER_PPR 360              // Pulses Per Revolution of the encoder (adjust to your encoder)
#define MAX_PWM_VALUE 255            // Maximum PWM value
#define MIN_PWM_VALUE 0              // Minimum PWM value
#define CONTROL_INTERVAL_MS 10       // PID control loop interval in milliseconds
#define CURRENT_THRESHOLD 900        // Over-current threshold (adjust based on your setup)
#define COMMAND_TIMEOUT_MS 1000      // Timeout for serial commands
#define POSITION_UPDATE_INTERVAL 50  // Interval to send position updates to MATLAB (ms)

// PID Variables
volatile long encoderPosition = 0;    // Current encoder position
long targetPosition = 0;              // Target position
long previousPosition = 0;            // Previous position for velocity calculation

// PID coefficients - initial values
float Kp = 1.0;       // Proportional gain
float Ki = 0.0;       // Integral gain
float Kd = 0.0;       // Derivative gain

// PID computation variables
float integral = 0.0;
float previousError = 0.0;
int controlOutput = 0;
unsigned long lastControlTime = 0;
unsigned long lastDataSendTime = 0;

// System state flags
bool motorEnabled = false;            // Motor enabled flag
bool emergencyStop = false;           // Emergency stop flag
bool pidEnabled = false;              // PID controller enabled flag
unsigned long lastCommandTime = 0;    // Time of last received command

// Communication buffer
String inputBuffer = "";
bool commandComplete = false;

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("Encoder PID Motor Control System");
  Serial.println("Ready for MATLAB communication");
  
  // Set up encoder pins with pull-up resistors
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  
  // Set up motor control pins
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN_1, OUTPUT);
  pinMode(MOTOR_DIR_PIN_2, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  
  // Set up safety pins
  pinMode(CURRENT_SENSE_PIN, INPUT);
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);  // Normally HIGH, LOW when button pressed
  
  // Initialize motor control - initially disabled
  stopMotor();
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  
  // Attach interrupts for encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderISR, CHANGE);
  
  // Initialize timers
  lastControlTime = millis();
  lastCommandTime = millis();
  lastDataSendTime = millis();
  
  // Ready to receive commands
  Serial.println("READY");
}

void loop() {
  // Check for emergency stop (external button or software command)
  if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    triggerEmergencyStop("External emergency stop triggered");
  }
  
  // Check for command timeout (safety feature)
  if (motorEnabled && (millis() - lastCommandTime > COMMAND_TIMEOUT_MS)) {
    stopMotor();
    Serial.println("ERROR:Command timeout - motor stopped");
  }
  
  // Process incoming serial commands
  processSerialInput();
  if (commandComplete) {
    processCommand();
    inputBuffer = "";
    commandComplete = false;
  }
  
  // Check for over-current condition
  if (motorEnabled) {
    checkCurrentLimit();
  }
  
  // Execute PID control loop at regular intervals
  if (pidEnabled && motorEnabled && !emergencyStop && 
      (millis() - lastControlTime >= CONTROL_INTERVAL_MS)) {
    executeControlLoop();
    lastControlTime = millis();
  }
  
  // Send position data to MATLAB at regular intervals
  if (millis() - lastDataSendTime >= POSITION_UPDATE_INTERVAL) {
    sendPositionData();
    lastDataSendTime = millis();
  }
}

// Encoder interrupt service routine
void encoderISR() {
  // Read the current state of encoder pins
  int A = digitalRead(ENCODER_A_PIN);
  int B = digitalRead(ENCODER_B_PIN);
  
  // Determine direction based on the state of both pins
  // This is a simple quadrature decoder - can be expanded for better performance
  if (A == B) {
    encoderPosition++;
  } else {
    encoderPosition--;
  }
}

// PID control loop
void executeControlLoop() {
  // Calculate error
  long error = targetPosition - encoderPosition;
  
  // Calculate the derivative (velocity error)
  long derivative = error - previousError;
  
  // Update integral with error
  integral += error;
  
  // Apply anti-windup - limit the integral term
  if (integral > 1000) integral = 1000;
  if (integral < -1000) integral = -1000;
  
  // Calculate PID output
  controlOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // Limit output to valid PWM range
  controlOutput = constrain(controlOutput, -MAX_PWM_VALUE, MAX_PWM_VALUE);
  
  // Set motor direction based on control output sign
  if (controlOutput >= 0) {
    setMotorDirection(true);  // Forward
  } else {
    setMotorDirection(false); // Reverse
    controlOutput = -controlOutput; // Make positive for PWM
  }
  
  // Apply PWM to motor
  analogWrite(MOTOR_PWM_PIN, controlOutput);
  
  // Save error for next iteration
  previousError = error;
}

// Set motor direction
void setMotorDirection(bool forward) {
  if (forward) {
    digitalWrite(MOTOR_DIR_PIN_1, HIGH);
    digitalWrite(MOTOR_DIR_PIN_2, LOW);
  } else {
    digitalWrite(MOTOR_DIR_PIN_1, LOW);
    digitalWrite(MOTOR_DIR_PIN_2, HIGH);
  }
}

// Stop motor
void stopMotor() {
  analogWrite(MOTOR_PWM_PIN, 0);
  motorEnabled = false;
  pidEnabled = false;
  integral = 0;  // Reset integral term
  previousError = 0;
}

// Enable motor
void enableMotor() {
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  motorEnabled = true;
  Serial.println("Motor enabled");
}

// Process serial input character by character
void processSerialInput() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    // Process line when newline received
    if (inChar == '\n') {
      commandComplete = true;
      break;
    } else {
      // Add character to buffer
      inputBuffer += inChar;
    }
  }
}

// Process complete commands
void processCommand() {
  inputBuffer.trim();  // Remove any whitespace
  
  // Update command time for timeout checking
  lastCommandTime = millis();
  
  // Parse command
  if (inputBuffer.startsWith("SET_PID")) {
    // Format: SET_PID,P,I,D
    int firstComma = inputBuffer.indexOf(',');
    int secondComma = inputBuffer.indexOf(',', firstComma + 1);
    int thirdComma = inputBuffer.indexOf(',', secondComma + 1);
    
    if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
      float newKp = inputBuffer.substring(firstComma + 1, secondComma).toFloat();
      float newKi = inputBuffer.substring(secondComma + 1, thirdComma).toFloat();
      float newKd = inputBuffer.substring(thirdComma + 1).toFloat();
      
      // Update PID parameters
      Kp = newKp;
      Ki = newKi;
      Kd = newKd;
      
      // Reset PID state for smooth transition
      integral = 0;
      previousError = 0;
      
      Serial.print("PID parameters set: P=");
      Serial.print(Kp);
      Serial.print(" I=");
      Serial.print(Ki);
      Serial.print(" D=");
      Serial.println(Kd);
    } else {
      Serial.println("ERROR:Invalid PID parameter format");
    }
  }
  else if (inputBuffer.startsWith("SET_TARGET")) {
    // Format: SET_TARGET,position
    int comma = inputBuffer.indexOf(',');
    
    if (comma > 0) {
      long newTarget = inputBuffer.substring(comma + 1).toInt();
      targetPosition = newTarget;
      Serial.print("Target position set to: ");
      Serial.println(targetPosition);
    } else {
      Serial.println("ERROR:Invalid target position format");
    }
  }
  else if (inputBuffer == "START_PID") {
    if (!emergencyStop) {
      pidEnabled = true;
      enableMotor();
      Serial.println("PID control started");
    } else {
      Serial.println("ERROR:Cannot start - emergency stop active");
    }
  }
  else if (inputBuffer == "STOP_PID") {
    pidEnabled = false;
    Serial.println("PID control stopped");
  }
  else if (inputBuffer == "STOP_MOTOR") {
    stopMotor();
    Serial.println("Motor stopped");
  }
  else if (inputBuffer == "ENABLE_MOTOR") {
    if (!emergencyStop) {
      enableMotor();
    } else {
      Serial.println("ERROR:Cannot enable - emergency stop active");
    }
  }
  else if (inputBuffer == "RESET_ENCODER") {
    // Reset position counters
    noInterrupts();  // Temporarily disable interrupts
    encoderPosition = 0;
    previousPosition = 0;
    interrupts();    // Re-enable interrupts
    Serial.println("Encoder position reset to zero");
  }
  else if (inputBuffer == "RESET_EMERGENCY") {
    if (emergencyStop) {
      emergencyStop = false;
      Serial.println("Emergency stop reset");
    }
  }
  else if (inputBuffer == "GET_STATUS") {
    sendStatusData();
  }
  else if (inputBuffer == "ESTOP") {
    triggerEmergencyStop("Software emergency stop triggered");
  }
  else if (inputBuffer == "DIRECT_PWM") {
    // Format: DIRECT_PWM,value
    int comma = inputBuffer.indexOf(',');
    
    if (comma > 0) {
      int pwmValue = inputBuffer.substring(comma + 1).toInt();
      
      // Limit to valid range
      pwmValue = constrain(pwmValue, MIN_PWM_VALUE, MAX_PWM_VALUE);
      
      if (!emergencyStop && motorEnabled) {
        analogWrite(MOTOR_PWM_PIN, pwmValue);
        Serial.print("Direct PWM set to: ");
        Serial.println(pwmValue);
      } else {
        Serial.println("ERROR:Cannot set PWM - motor disabled or emergency stop active");
      }
    } else {
      Serial.println("ERROR:Invalid PWM format");
    }
  }
  else {
    Serial.print("ERROR:Unknown command: ");
    Serial.println(inputBuffer);
  }
}

// Check for over-current condition
void checkCurrentLimit() {
  int currentValue = analogRead(CURRENT_SENSE_PIN);
  
  if (currentValue > CURRENT_THRESHOLD) {
    triggerEmergencyStop("Over-current detected");
  }
}

// Trigger emergency stop
void triggerEmergencyStop(String reason) {
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  motorEnabled = false;
  pidEnabled = false;
  emergencyStop = true;
  
  Serial.print("EMERGENCY STOP: ");
  Serial.println(reason);
}

// Send position data to MATLAB
void sendPositionData() {
  // Calculate approximate speed (change in position since last update)
  long positionChange = encoderPosition - previousPosition;
  previousPosition = encoderPosition;
  
  // Send formatted data string
  // Format: DATA,timestamp,position,speed,target,output
  Serial.print("DATA,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(encoderPosition);
  Serial.print(",");
  Serial.print(positionChange);
  Serial.print(",");
  Serial.print(targetPosition);
  Serial.print(",");
  Serial.println(controlOutput);
}

// Send full status data to MATLAB
void sendStatusData() {
  // Format: STATUS,enabled,emergency,pid_enabled,p,i,d,position,target
  Serial.print("STATUS,");
  Serial.print(motorEnabled ? "1" : "0");
  Serial.print(",");
  Serial.print(emergencyStop ? "1" : "0");
  Serial.print(",");
  Serial.print(pidEnabled ? "1" : "0");
  Serial.print(",");
  Serial.print(Kp);
  Serial.print(",");
  Serial.print(Ki);
  Serial.print(",");
  Serial.print(Kd);
  Serial.print(",");
  Serial.print(encoderPosition);
  Serial.print(",");
  Serial.println(targetPosition);
}

