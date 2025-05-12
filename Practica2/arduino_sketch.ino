/*
 * Arduino Encoder Motor Control with Serial Commands
 * =================================================
 * This sketch controls a DC motor with encoder feedback using PWM
 * and adds a serial command interface for integration with the Python GUI.
 * 
 * Features:
 * - Encoder position tracking using interrupts
 * - Continuous rotation mode with direction control
 * - Real-time speed calculation from encoder feedback
 * - Safety features including acceleration limiting and stall detection
 */

// Pin Definitions
#define ENCODER_PIN_A 2  // Interrupt pin
#define ENCODER_PIN_B 3  // Interrupt pin
#define MOTOR_PWM_PIN 5  // PWM output for motor speed
#define MOTOR_DIR_PIN 7  // Direction control pin

// Encoder Constants
#define ENCODER_RESOLUTION 1200  // Pulses per revolution (modify based on your encoder specs)
#define SPEED_CALC_INTERVAL 200  // Calculate speed every 200ms
#define MAX_PWM_VALUE 255       // Maximum PWM value
#define MIN_PWM_VALUE 0         // Minimum PWM value
#define ACCEL_RATE 5            // Acceleration rate limiter (PWM change per interval)
#define STALL_TIMEOUT 2000      // Time in ms to detect a stall
#define STALL_THRESHOLD 10      // Minimum encoder pulses to consider motor not stalled

// Global Variables
volatile long encoderPosition = 0;  // Current encoder position
volatile long lastPosition = 0;     // Previous encoder position
unsigned long lastSpeedCalc = 0;    // Last speed calculation time
unsigned long lastStallCheck = 0;   // Last stall check time
float currentSpeed = 0.0;           // Current calculated speed in RPM
int targetPwm = 0;                  // Target PWM value (0-255)
int currentPwm = 0;                 // Current PWM value with acceleration limiting
boolean isRunning = false;          // Motor running status
boolean continuousMode = false;     // Continuous rotation mode
String direction = "clockwise";     // Default direction
int rotationCount = 0;              // Counter for full rotations

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
  if ((encoderPosition % ENCODER_RESOLUTION) == 0 && lastPosition != encoderPosition) {
    if (encoderPosition > lastPosition) {
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
  
  // Set up motor control pins
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  
  // Set initial motor state
  digitalWrite(MOTOR_DIR_PIN, HIGH);  // Set initial direction (HIGH = clockwise)
  analogWrite(MOTOR_PWM_PIN, 0);      // Start with motor stopped
  
  // Attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderPinAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoderPinBISR, CHANGE);
  
  // Initialize serial communication for commands
  Serial.begin(9600);
  Serial.println("Arduino Encoder Motor Control Ready");
}

void loop() {
  // Process any incoming serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
  
  // Update motor control with acceleration limiting
  updateMotorControl();
  
  // Calculate speed from encoder at regular intervals
  calculateSpeed();
  
  // Check for motor stall condition
  checkForStall();
}

// Calculate motor speed based on encoder position changes
void calculateSpeed() {
  unsigned long currentTime = millis();
  
  // Calculate speed every SPEED_CALC_INTERVAL milliseconds
  if (currentTime - lastSpeedCalc >= SPEED_CALC_INTERVAL) {
    // Calculate position change
    long positionChange = encoderPosition - lastPosition;
    float timeChange = (currentTime - lastSpeedCalc) / 1000.0; // Convert to seconds
    
    // Calculate speed in RPM
    // Formula: (positionChange / ENCODER_RESOLUTION) = rotations
    // rotations / timeChange = rotations per second
    // rotations per second * 60 = RPM
    currentSpeed = (positionChange / (float)ENCODER_RESOLUTION) * (60.0 / timeChange);
    
    // Update lastPosition and lastSpeedCalc
    lastPosition = encoderPosition;
    lastSpeedCalc = currentTime;
  }
}

// Update motor control with acceleration limiting
void updateMotorControl() {
  // Apply direction
  if (direction == "clockwise") {
    digitalWrite(MOTOR_DIR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_DIR_PIN, LOW);
  }
  
  // Motor control logic
  if (isRunning) {
    // Apply acceleration limiting
    if (currentPwm < targetPwm) {
      currentPwm = min(currentPwm + ACCEL_RATE, targetPwm);
    } else if (currentPwm > targetPwm) {
      currentPwm = max(currentPwm - ACCEL_RATE, targetPwm);
    }
    
    // Apply PWM to motor
    analogWrite(MOTOR_PWM_PIN, currentPwm);
  } else {
    // Gradually stop motor if not running
    if (currentPwm > 0) {
      currentPwm = max(currentPwm - ACCEL_RATE, 0);
      analogWrite(MOTOR_PWM_PIN, currentPwm);
    } else {
      analogWrite(MOTOR_PWM_PIN, 0);
    }
  }
}

// Check for motor stall condition
void checkForStall() {
  unsigned long currentTime = millis();
  
  // Only check for stall if motor is running
  if (isRunning && currentPwm > 50) {
    if (currentTime - lastStallCheck >= STALL_TIMEOUT) {
      // Check if position has changed significantly
      long positionChange = abs(encoderPosition - lastPosition);
      
      // If minimal or no movement, consider it stalled
      if (positionChange < STALL_THRESHOLD) {
        // Motor is stalled - emergency stop
        isRunning = false;
        targetPwm = 0;
        currentPwm = 0;
        analogWrite(MOTOR_PWM_PIN, 0);
        Serial.println("ERROR:Motor stall detected");
      }
      
      // Update last stall check time
      lastStallCheck = currentTime;
      lastPosition = encoderPosition;
    }
  }
}

// Process commands received from the Python GUI
void processCommand(String command) {
  // Split the command at ':' if it contains parameters
  int colonIndex = command.indexOf(':');
  String cmd = colonIndex > 0 ? command.substring(0, colonIndex) : command;
  String param = colonIndex > 0 ? command.substring(colonIndex + 1) : "";
  
  if (cmd == "SET_DIR" || cmd == "CW" || cmd == "CCW") {
    // Set direction
    if (cmd == "CW" || param == "CW") {
      direction = "clockwise";
      Serial.println("OK:SET_DIR");
      Serial.println("clockwise");
    } else if (cmd == "CCW" || param == "CCW") {
      direction = "counterclockwise";
      Serial.println("OK:SET_DIR");
      Serial.println("counterclockwise");
    } else {
      Serial.println("ERROR:Invalid direction");
    }
  }
  else if (cmd == "SET_SPEED" || cmd == "SPEED") {
    // Set motor speed (via PWM value)
    int newPwm = param.toInt();
    if (newPwm >= MIN_PWM_VALUE && newPwm <= MAX_PWM_VALUE) {
      targetPwm = newPwm;
      Serial.println("OK:SET_SPEED");
    } else {
      Serial.println("ERROR:Invalid speed value (0-255)");
    }
  }
  else if (cmd == "SET_CONTINUOUS") {
    // Set continuous mode
    if (param == "ON") {
      continuousMode = true;
      Serial.println("OK:SET_CONTINUOUS");
    } else if (param == "OFF") {
      continuousMode = false;
      Serial.println("OK
