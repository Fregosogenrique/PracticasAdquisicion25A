/**
 * Motor Encoder Control with PID
 * 
 * This sketch implements a PID control system for a DC motor using quadrature encoder feedback.
 * It includes:
 * - Quadrature encoder decoding using interrupts
 * - PID control algorithm for position and speed
 * - Motor control via PWM
 * - Serial communication for command input and data output
 * 
 * Connections:
 * - Encoder Phase A: Pin 2 (interrupt)
 * - Encoder Phase B: Pin 3 (interrupt)
 * - Motor Driver PWM: Pin 9
 * - Motor Driver Direction: Pins 7, 8
 */

// Pin Definitions
const int ENCODER_PIN_A = 2;  // Interrupt pin for encoder phase A
const int ENCODER_PIN_B = 3;  // Interrupt pin for encoder phase B
const int MOTOR_PWM = 9;      // PWM control for motor speed
const int MOTOR_DIR_A = 7;    // Direction control pin 1
const int MOTOR_DIR_B = 8;    // Direction control pin 2

// Encoder variables
volatile long encoderPosition = 0;  // Current position of the encoder
volatile long lastEncoderPosition = 0; // Last position for speed calculation
unsigned long lastTime = 0; // For speed calculation
long encoderSpeed = 0; // Speed in counts per second

// Motor control variables
int motorPwm = 0;       // Motor PWM value (0-255)
bool motorDirection = true; // true = forward, false = reverse

// PID control variables
double setpoint = 0;      // Desired position or speed
double input = 0;         // Current position or speed (from encoder)
double output = 0;        // Control output to motor
double error = 0;         // Current error
double lastError = 0;     // Previous error for derivative calculation
double integral = 0;      // Integral accumulator
double derivative = 0;    // Rate of change of error

// PID parameters (tunable)
double Kp = 1.0;  // Proportional gain
double Ki = 0.1;  // Integral gain
double Kd = 0.01; // Derivative gain

// Control mode
bool positionMode = true; // true = position control, false = speed control

// Serial communication variables
const int BUFFER_SIZE = 64;
char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;
unsigned long lastDataSent = 0;
const int DATA_INTERVAL = 100; // Send data every 100ms

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize motor control pins
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR_A, OUTPUT);
  pinMode(MOTOR_DIR_B, OUTPUT);
  
  // Set initial motor state - stopped
  analogWrite(MOTOR_PWM, 0);
  digitalWrite(MOTOR_DIR_A, LOW);
  digitalWrite(MOTOR_DIR_B, LOW);
  
  // Initialize encoder pins with pull-up resistors
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  
  // Attach interrupts for quadrature encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), handleEncoderB, CHANGE);
  
  // Initialize timing variables
  lastTime = millis();
  lastDataSent = millis();
  
  Serial.println("Motor Encoder Control with PID initialized");
  Serial.println("Commands:");
  Serial.println("- SETMODE,P: Set position control mode");
  Serial.println("- SETMODE,S: Set speed control mode");
  Serial.println("- SET,value: Set the target position/speed");
  Serial.println("- PID,p,i,d: Set PID parameters");
  Serial.println("- MOTOR,value: Directly set motor PWM (-255 to 255)");
  Serial.println("- RESET: Reset encoder position to 0");
}

void loop() {
  // Process incoming serial commands
  processSerialCommands();
  
  // Calculate speed
  calculateSpeed();
  
  // Update PID input based on mode
  if (positionMode) {
    input = encoderPosition;
  } else {
    input = encoderSpeed;
  }
  
  // Calculate PID
  updatePID();
  
  // Apply PID output to motor
  setMotor((int)output);
  
  // Send data to serial port at regular intervals
  if (millis() - lastDataSent >= DATA_INTERVAL) {
    sendData();
    lastDataSent = millis();
  }
}

// Encoder interrupt handlers
void handleEncoderA() {
  // XOR the two encoder pins - if they're the same, count up, otherwise count down
  if (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B)) {
    encoderPosition++;
  } else {
    encoderPosition--;
  }
}

void handleEncoderB() {
  // XOR the two encoder pins - if they're different, count up, otherwise count down
  if (digitalRead(ENCODER_PIN_A) != digitalRead(ENCODER_PIN_B)) {
    encoderPosition++;
  } else {
    encoderPosition--;
  }
}

// Calculate motor speed from encoder position changes
void calculateSpeed() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;
  
  // Calculate speed every 50ms
  if (deltaTime >= 50) {
    // Calculate speed in counts per second
    encoderSpeed = (encoderPosition - lastEncoderPosition) * 1000 / deltaTime;
    
    lastEncoderPosition = encoderPosition;
    lastTime = currentTime;
  }
}

// Update PID calculation
void updatePID() {
  // Calculate error
  error = setpoint - input;
  
  // Calculate integral
  integral += error;
  
  // Apply integral windup guard
  if (integral > 1000) integral = 1000;
  if (integral < -1000) integral = -1000;
  
  // Calculate derivative
  derivative = error - lastError;
  
  // Calculate output
  output = Kp * error + Ki * integral + Kd * derivative;
  
  // Limit output to motor range
  if (output > 255) output = 255;
  if (output < -255) output = -255;
  
  // Save current error for next iteration
  lastError = error;
}

// Set motor speed and direction
void setMotor(int value) {
  // Set direction
  if (value > 0) {
    digitalWrite(MOTOR_DIR_A, HIGH);
    digitalWrite(MOTOR_DIR_B, LOW);
    motorDirection = true;
  } else if (value < 0) {
    digitalWrite(MOTOR_DIR_A, LOW);
    digitalWrite(MOTOR_DIR_B, HIGH);
    motorDirection = false;
    value = -value; // Make value positive for PWM
  } else {
    // Stop motor
    digitalWrite(MOTOR_DIR_A, LOW);
    digitalWrite(MOTOR_DIR_B, LOW);
  }
  
  // Set PWM value
  motorPwm = value;
  analogWrite(MOTOR_PWM, motorPwm);
}

// Process serial commands
void processSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Add character to buffer
    if (bufferIndex < BUFFER_SIZE - 1) {
      inputBuffer[bufferIndex++] = c;
    }
    
    // Process command when newline is received
    if (c == '\n') {
      inputBuffer[bufferIndex] = 0; // Null terminate the string
      parseCommand(inputBuffer);
      bufferIndex = 0; // Reset buffer
    }
  }
}

// Parse and execute a command
void parseCommand(char* command) {
  char* token = strtok(command, ",");
  
  if (token != NULL) {
    if (strcmp(token, "SETMODE") == 0) {
      token = strtok(NULL, ",");
      if (token != NULL) {
        if (token[0] == 'P') {
          positionMode = true;
          integral = 0; // Reset integral when changing modes
          Serial.println("Mode: Position Control");
        } else if (token[0] == 'S') {
          positionMode = false;
          integral = 0; // Reset integral when changing modes
          Serial.println("Mode: Speed Control");
        }
      }
    } else if (strcmp(token, "SET") == 0) {
      token = strtok(NULL, ",");
      if (token != NULL) {
        setpoint = atof(token);
        Serial.print("Setpoint: ");
        Serial.println(setpoint);
      }
    } else if (strcmp(token, "PID") == 0) {
      token = strtok(NULL, ",");
      if (token != NULL) {
        Kp = atof(token);
        token = strtok(NULL, ",");
        if (token != NULL) {
          Ki = atof(token);
          token = strtok(NULL, ",");
          if (token != NULL) {
            Kd = atof(token);
          }
        }
        Serial.print("PID parameters set: Kp=");
        Serial.print(Kp);
        Serial.print(", Ki=");
        Serial.print(Ki);
        Serial.print(", Kd=");
        Serial.println(Kd);
      }
    } else if (strcmp(token, "MOTOR") == 0) {
      token = strtok(NULL, ",");
      if (token != NULL) {
        int value = atoi(token);
        // Limit value to motor range
        if (value > 255) value = 255;
        if (value < -255) value = -255;
        
        setMotor(value);
        Serial.print("Motor direct control: ");
        Serial.println(value);
      }
    } else if (strcmp(token, "RESET") == 0) {
      encoderPosition = 0;
      lastEncoderPosition = 0;
      integral = 0;
      Serial.println("Encoder position reset to 0");
    }
  }
}

// Send data to serial port
void sendData() {
  Serial.print("DATA,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(encoderPosition);
  Serial.print(",");
  Serial.print(encoderSpeed);
  Serial.print(",");
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(error);
  Serial.print(",");
  Serial.print(output);
  Serial.print(",");
  Serial.print(Kp);
  Serial.print(",");
  Serial.print(Ki);
  Serial.print(",");
  Serial.print(Kd);
  Serial.print(",");
  Serial.print(positionMode ? "P" : "S");
  Serial.println();
}

