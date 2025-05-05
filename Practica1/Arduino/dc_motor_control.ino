/*
 * Arduino DC Motor Control with H-Bridge
 * =====================================
 * This sketch controls a DC motor using PWM through an H-bridge circuit.
 * It establishes a serial communication protocol to interact with a Python GUI.
 * 
 * Features:
 * - PWM speed control
 * - Direction control (forward/reverse) using H-bridge
 * - Serial command interface
 * - Safety features (emergency stop, current limiting)
 */

// Pin Definitions for H-Bridge
#define EN_A_PIN 9    // PWM output for motor speed (must be a PWM pin)
#define IN1_PIN 7     // Direction control 1
#define IN2_PIN 8     // Direction control 2

// Optional current sensing pin (if H-bridge supports it)
#define CURRENT_SENSE_PIN A0  // Analog input for current sensing

// Motor control constants
#define MAX_PWM_VALUE 255
#define MIN_PWM_VALUE 0
#define CURRENT_THRESHOLD 900  // Adjust based on your motor and current sensing circuit

// Motor state variables
int motorSpeed = 0;            // Current motor speed (0-255)
bool isRunning = false;        // Motor running status
bool isForward = true;         // Direction flag: true = forward, false = reverse
bool emergencyStopped = false; // Emergency stop flag

// Command processing variables
String inputBuffer = "";       // Buffer to store incoming command
bool commandComplete = false;  // Flag to indicate a complete command

void setup() {
  // Configure pins
  pinMode(EN_A_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(CURRENT_SENSE_PIN, INPUT);
  
  // Initialize motor to stopped state
  analogWrite(EN_A_PIN, 0);    // Set speed to 0
  setDirection(true);          // Default to forward direction
  
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Arduino DC Motor Control Ready");
  Serial.println("Commands: SET_SPEED <0-255>, SET_DIR <F/R>, START, STOP, ESTOP");
}

void loop() {
  // Process serial commands
  processSerialInput();
  
  // Check for over-current condition if motor is running
  if (isRunning && !emergencyStopped) {
    checkCurrentLimit();
  }
  
  // Execute commands when complete
  if (commandComplete) {
    processCommand();
    // Reset for next command
    inputBuffer = "";
    commandComplete = false;
  }
}

// Read and buffer serial input
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

// Process buffered command
void processCommand() {
  // Convert to uppercase for easier comparison
  inputBuffer.trim();
  
  // Parse command and parameters
  if (inputBuffer.startsWith("SET_SPEED")) {
    // Extract speed value
    int spaceIndex = inputBuffer.indexOf(' ');
    if (spaceIndex != -1) {
      String speedValue = inputBuffer.substring(spaceIndex + 1);
      int newSpeed = speedValue.toInt();
      
      // Validate and set speed
      if (newSpeed >= MIN_PWM_VALUE && newSpeed <= MAX_PWM_VALUE) {
        setMotorSpeed(newSpeed);
        Serial.print("OK:SPEED:");
        Serial.println(newSpeed);
      } else {
        Serial.println("ERROR:Invalid speed value (0-255)");
      }
    } else {
      Serial.println("ERROR:Missing speed value");
    }
  }
  else if (inputBuffer.startsWith("SET_DIR")) {
    // Extract direction value
    int spaceIndex = inputBuffer.indexOf(' ');
    if (spaceIndex != -1) {
      String dirValue = inputBuffer.substring(spaceIndex + 1);
      
      // Set direction based on F or R
      if (dirValue.equals("F")) {
        setDirection(true);
        Serial.println("OK:DIR:FORWARD");
      } 
      else if (dirValue.equals("R")) {
        setDirection(false);
        Serial.println("OK:DIR:REVERSE");
      }
      else {
        Serial.println("ERROR:Invalid direction (use F or R)");
      }
    } else {
      Serial.println("ERROR:Missing direction value");
    }
  }
  else if (inputBuffer.equals("START")) {
    // Start the motor if not in emergency stop
    if (!emergencyStopped) {
      startMotor();
      Serial.println("OK:MOTOR_STARTED");
    } else {
      Serial.println("ERROR:Emergency stop active");
    }
  }
  else if (inputBuffer.equals("STOP")) {
    // Stop the motor
    stopMotor();
    Serial.println("OK:MOTOR_STOPPED");
  }
  else if (inputBuffer.equals("ESTOP")) {
    // Emergency stop
    emergencyStop();
    Serial.println("OK:EMERGENCY_STOP_ACTIVATED");
  }
  else if (inputBuffer.equals("RESET")) {
    // Reset after emergency stop
    resetEmergencyStop();
    Serial.println("OK:EMERGENCY_STOP_RESET");
  }
  else {
    // Unknown command
    Serial.print("ERROR:Unknown command: ");
    Serial.println(inputBuffer);
  }
}

// Set the motor speed via PWM
void setMotorSpeed(int speed) {
  motorSpeed = speed;
  
  if (isRunning) {
    analogWrite(EN_A_PIN, motorSpeed);
  }
  
  Serial.print("Speed set to: ");
  Serial.println(motorSpeed);
}

// Set the motor direction
void setDirection(bool forward) {
  isForward = forward;
  
  if (isForward) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }
  
  Serial.print("Direction set to: ");
  Serial.println(isForward ? "FORWARD" : "REVERSE");
}

// Start the motor
void startMotor() {
  if (!isRunning && !emergencyStopped) {
    isRunning = true;
    analogWrite(EN_A_PIN, motorSpeed);
    Serial.println("Motor started");
  }
}

// Stop the motor
void stopMotor() {
  isRunning = false;
  analogWrite(EN_A_PIN, 0);
  Serial.println("Motor stopped");
}

// Emergency stop - immediately cuts power
void emergencyStop() {
  isRunning = false;
  emergencyStopped = true;
  analogWrite(EN_A_PIN, 0);
  Serial.println("EMERGENCY STOP!");
}

// Reset emergency stop state
void resetEmergencyStop() {
  emergencyStopped = false;
  Serial.println("Emergency stop reset");
}

// Check for over-current condition
void checkCurrentLimit() {
  int currentValue = analogRead(CURRENT_SENSE_PIN);
  
  // If current exceeds threshold, trigger emergency stop
  if (currentValue > CURRENT_THRESHOLD) {
    Serial.print("ALERT:Over-current detected: ");
    Serial.println(currentValue);
    emergencyStop();
  }
}

// Optional: Can add SPWM functionality here if needed instead of standard PWM

