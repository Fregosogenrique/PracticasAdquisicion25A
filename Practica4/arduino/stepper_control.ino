/*
 * Stepper Motor Control with M542C Driver
 * 
 * Controls a stepper motor using M542C driver with 3 axis inputs
 * Communication with Python GUI via Serial
 * 
 * X, Y, and R axis inputs are read to control motor speed and direction
 */

// Pin Definitions for M542C Driver
const int STEP_PIN = 3;    // Pulse signal pin
const int DIR_PIN = 2;     // Direction control pin
const int ENA_PIN = 4;     // Enable pin (LOW to enable, HIGH to disable)

// Analog Input Pins for Axes
const int X_AXIS_PIN = A0; // X axis input
const int Y_AXIS_PIN = A1; // Y axis input
const int R_AXIS_PIN = A2; // R axis input

// Motor Control Variables
unsigned long previousMicros = 0;  // For tracking pulse timing
int stepState = LOW;               // Current state of step pin
bool motorEnabled = false;         // Motor enable state
int motorDirection = 1;            // 1 for forward, 0 for reverse
unsigned int stepInterval = 1000;  // Microseconds between steps (controls speed)
unsigned int maxSpeed = 3000;      // Maximum speed in steps per second
unsigned int minSpeed = 100;       // Minimum speed in steps per second

// Serial Communication
String inputBuffer = "";          // Buffer for incoming serial data
bool stringComplete = false;      // Flag for complete serial message
const long BAUD_RATE = 115200;    // Serial baud rate

// Status Variables
int activeAxis = 0;               // 0 = X, 1 = Y, 2 = R
int xAxisValue = 0;               // Current X axis value
int yAxisValue = 0;               // Current Y axis value
int rAxisValue = 0;               // Current R axis value
long currentPosition = 0;         // Keep track of motor position

void setup() {
  // Initialize pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  
  // Initialize motor state - disabled by default
  digitalWrite(ENA_PIN, HIGH);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(STEP_PIN, LOW);
  
  // Start serial communication
  Serial.begin(BAUD_RATE);
  Serial.println("Stepper Motor Control System Ready");
  
  // Initialize input buffer
  inputBuffer.reserve(64);
}

void loop() {
  // Handle serial communication
  readSerialCommands();
  
  // Read analog inputs
  readAxisInputs();
  
  // Generate step pulses if motor is enabled
  if (motorEnabled) {
    generateStepPulse();
  }
  
  // Send status updates every 500ms
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate > 500) {
    sendStatusUpdate();
    lastStatusUpdate = millis();
  }
}

// Read axis inputs and calculate motor parameters
void readAxisInputs() {
  // Read raw values
  xAxisValue = analogRead(X_AXIS_PIN);
  yAxisValue = analogRead(Y_AXIS_PIN);
  rAxisValue = analogRead(R_AXIS_PIN);
  
  // Get the active axis value based on currently selected axis
  int currentAxisValue = 0;
  switch(activeAxis) {
    case 0: currentAxisValue = xAxisValue; break;
    case 1: currentAxisValue = yAxisValue; break;
    case 2: currentAxisValue = rAxisValue; break;
  }
  
  // Calculate direction (values below 512 are reverse, above are forward)
  if (currentAxisValue < 512) {
    motorDirection = 0;  // Reverse
    currentAxisValue = 512 - currentAxisValue;  // Convert to positive range
  } else {
    motorDirection = 1;  // Forward
    currentAxisValue = currentAxisValue - 512;  // Convert to 0-based range
  }
  
  // Apply direction to DIR pin
  digitalWrite(DIR_PIN, motorDirection);
  
  // Calculate speed (step interval) from axis value
  // Map 0-512 axis input to min-max speed
  if (currentAxisValue < 20) {
    // Deadzone to prevent motor drift
    motorEnabled = false;
    digitalWrite(ENA_PIN, HIGH);  // Disable motor
  } else {
    motorEnabled = true;
    digitalWrite(ENA_PIN, LOW);   // Enable motor
    
    // Calculate steps per second based on axis value (0-512 maps to min-max speed)
    int stepsPerSecond = map(currentAxisValue, 20, 512, minSpeed, maxSpeed);
    
    // Convert steps per second to step interval in microseconds
    stepInterval = 1000000 / stepsPerSecond;
  }
}

// Generate non-blocking step pulses with precise timing
void generateStepPulse() {
  unsigned long currentMicros = micros();
  
  // Check if it's time for the next pulse action
  if (currentMicros - previousMicros >= (stepState == HIGH ? 10 : stepInterval - 10)) {
    previousMicros = currentMicros;
    
    // Toggle step pin
    stepState = (stepState == LOW) ? HIGH : LOW;
    digitalWrite(STEP_PIN, stepState);
    
    // Count steps for position tracking (only when step goes from LOW to HIGH)
    if (stepState == HIGH) {
      currentPosition += (motorDirection == 1) ? 1 : -1;
    }
  }
}

// Process incoming serial commands
void readSerialCommands() {
  while (Serial.available() > 0 && !stringComplete) {
    char inChar = (char)Serial.read();
    
    // Process the command when newline received
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      // Add character to buffer
      inputBuffer += inChar;
    }
  }
  
  // Process completed command
  if (stringComplete) {
    processCommand(inputBuffer);
    
    // Clear buffer for next command
    inputBuffer = "";
    stringComplete = false;
  }
}

// Process command from serial interface
void processCommand(String command) {
  // Format: <axis>,<speed>,<direction>
  // Examples: 
  // "X,1000,1" - X axis, 1000 steps/sec, forward
  // "Y,500,0" - Y axis, 500 steps/sec, reverse
  // "STOP" - Emergency stop
  
  if (command == "STOP") {
    // Emergency stop command
    motorEnabled = false;
    digitalWrite(ENA_PIN, HIGH);  // Disable motor
    Serial.println("Motor stopped");
    return;
  }
  
  // Split command by commas
  int firstComma = command.indexOf(',');
  int secondComma = command.indexOf(',', firstComma + 1);
  
  if (firstComma > 0 && secondComma > firstComma) {
    // Extract axis
    String axis = command.substring(0, firstComma);
    
    // Set active axis
    if (axis == "X" || axis == "x") {
      activeAxis = 0;
    } else if (axis == "Y" || axis == "y") {
      activeAxis = 1;
    } else if (axis == "R" || axis == "r") {
      activeAxis = 2;
    } else {
      Serial.println("Invalid axis");
      return;
    }
    
    // Extract and set speed
    String speedStr = command.substring(firstComma + 1, secondComma);
    int speed = speedStr.toInt();
    if (speed < minSpeed) speed = minSpeed;
    if (speed > maxSpeed) speed = maxSpeed;
    
    // Convert speed to step interval
    if (speed > 0) {
      stepInterval = 1000000 / speed;
      motorEnabled = true;
      digitalWrite(ENA_PIN, LOW);  // Enable motor
    } else {
      motorEnabled = false;
      digitalWrite(ENA_PIN, HIGH); // Disable motor
    }
    
    // Extract and set direction
    String dirStr = command.substring(secondComma + 1);
    motorDirection = (dirStr == "1" || dirStr == "1\r") ? 1 : 0;
    digitalWrite(DIR_PIN, motorDirection);
    
    // Send confirmation
    Serial.print("Set axis: ");
    Serial.print(axis);
    Serial.print(", speed: ");
    Serial.print(speed);
    Serial.print(" steps/sec, direction: ");
    Serial.println(motorDirection);
  } else {
    Serial.println("Invalid command format. Use: <axis>,<speed>,<direction>");
  }
}

// Send status update to the Python GUI
void sendStatusUpdate() {
  Serial.print("STATUS,");
  Serial.print(activeAxis);
  Serial.print(",");
  Serial.print(motorEnabled ? "ENABLED" : "DISABLED");
  Serial.print(",");
  Serial.print(motorDirection);
  Serial.print(",");
  Serial.print(1000000 / stepInterval); // Convert interval back to steps/sec
  Serial.print(",");
  Serial.print(currentPosition);
  Serial.print(",");
  Serial.print(xAxisValue);
  Serial.print(",");
  Serial.print(yAxisValue);
  Serial.print(",");
  Serial.println(rAxisValue);
}

