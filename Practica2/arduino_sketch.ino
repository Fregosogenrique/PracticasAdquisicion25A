/*
 * Arduino Stepper Motor Control with Serial Commands
 * ================================================
 * This sketch controls a stepper motor using the Stepper library and
 * adds a serial command interface for integration with the Python GUI.
 * 
 * Original stepper code based on:
 * Stepper Motor Control - one revolution
 * Modified by Andres Cruz, ELECTRONILAB.CO
 * 
 * Enhanced with serial communication for GUI control.
 */

#include <Stepper.h>

// Default values - can be changed through serial commands
int stepsPerRevolution = 48;  // Default from original code
int motorSpeed = 60;          // Default speed in RPM
boolean isRunning = false;    // Motor running status
String direction = "clockwise"; // Default direction

// Initialize the stepper library on pins 8 through 11
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

void setup() {
  // Set the speed in RPM
  myStepper.setSpeed(motorSpeed);
  
  // Initialize serial communication for commands
  Serial.begin(9600);
  Serial.println("Arduino Stepper Control Ready");
}

void loop() {
  // Process any incoming serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
  
  // If the motor is running, step in the current direction
  if (isRunning) {
    if (direction == "clockwise") {
      myStepper.step(1);
    } else {
      myStepper.step(-1);
    }
  }
}

// Process commands received from the Python GUI
void processCommand(String command) {
  // Split the command at ':' if it contains parameters
  int colonIndex = command.indexOf(':');
  String cmd = colonIndex > 0 ? command.substring(0, colonIndex) : command;
  String param = colonIndex > 0 ? command.substring(colonIndex + 1) : "";
  
  if (cmd == "CW") {
    // Set direction to clockwise
    direction = "clockwise";
    Serial.println("OK:CW");
    Serial.println("clockwise");
  }
  else if (cmd == "CCW") {
    // Set direction to counterclockwise
    direction = "counterclockwise";
    Serial.println("OK:CCW");
    Serial.println("counterclockwise");
  }
  else if (cmd == "SPEED") {
    // Set motor speed
    int newSpeed = param.toInt();
    if (newSpeed > 0) {
      motorSpeed = newSpeed;
      myStepper.setSpeed(motorSpeed);
      Serial.println("OK:SPEED");
    } else {
      Serial.println("ERROR:Invalid speed value");
    }
  }
  else if (cmd == "STEPS") {
    // Set steps per revolution
    int newSteps = param.toInt();
    if (newSteps > 0) {
      stepsPerRevolution = newSteps;
      myStepper = Stepper(stepsPerRevolution, 8, 9, 10, 11);
      myStepper.setSpeed(motorSpeed);
      Serial.println("OK:STEPS");
    } else {
      Serial.println("ERROR:Invalid steps value");
    }
  }
  else if (cmd == "START") {
    // Start the motor - optional param for direction
    isRunning = true;
    if (param == "CW") {
      direction = "clockwise";
      Serial.println("clockwise");
    } else if (param == "CCW") {
      direction = "counterclockwise";
      Serial.println("counterclockwise");
    }
    Serial.println("OK:START");
  }
  else if (cmd == "STOP") {
    // Stop the motor
    isRunning = false;
    Serial.println("OK:STOP");
  }
  else if (cmd == "ESTOP") {
    // Emergency stop - immediately halt
    isRunning = false;
    Serial.println("OK:ESTOP");
  }
  else {
    // Unknown command
    Serial.println("ERROR:Unknown command");
  }
}

