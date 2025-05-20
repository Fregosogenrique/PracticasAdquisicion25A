// --- PINES ---
// Motor Driver (ej. L298N)
const int ENA_PIN = 9;  // PWM para velocidad (Debe ser un pin PWM: 3, 5, 6, 9, 10, 11 en UNO)
const int IN1_PIN = 8;  // Control de dirección 1
const int IN2_PIN = 7;  // Control de dirección 2

// Encoder
const int ENCODER_A_PIN = 2; // Canal A del encoder (Debe ser un pin de interrupción: 2 o 3 en UNO)
const int ENCODER_B_PIN = 4; // Canal B del encoder (opcional, para detectar dirección si es necesario)

// --- CONSTANTES DEL MOTOR Y ENCODER ---
const float PULSES_PER_REVOLUTION = 600.0; // ¡AJUSTA ESTO! Pulsos del encoder por cada vuelta completa del eje del motor
const int SAMPLE_TIME_MS = 50;             // Tiempo en milisegundos para calcular velocidad y aplicar PID

// --- VARIABLES PID ---
double Kp = 2.0;  // Ganancia Proporcional ¡AJUSTAR!
double Ki = 5.0;  // Ganancia Integral ¡AJUSTAR!
double Kd = 1.0;  // Ganancia Derivativa ¡AJUSTAR!

double setpointRPM = 0.0;      // Velocidad deseada en RPM (Recibida de Python)
double currentRPM = 0.0;       // Velocidad actual medida
double pwmOutput = 0;          // Salida PWM para el motor (0-255)

double integralError = 0.0;
double previousError = 0.0;

// --- VARIABLES DEL ENCODER ---
volatile long encoderPulses = 0; // Contador de pulsos (volátil por ser usado en ISR)
long lastEncoderPulses = 0;

// --- TEMPORIZACIÓN ---
unsigned long lastPidTime = 0;
unsigned long lastSerialReportTime = 0;
const int SERIAL_REPORT_INTERVAL_MS = 200; // Intervalo para enviar datos a Python

// --- ESTADO DEL MOTOR ---
bool motorRunning = false;
int motorDirection = 1; // 1: Adelante, -1: Atrás, 0: Parado

void setup() {
  Serial.begin(115200); // Comunicación con Python

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP); // Usar pull-up interno si no hay externo
  pinMode(ENCODER_B_PIN, INPUT_PULLUP); // Opcional

  // Configurar interrupción para el canal A del encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), countPulseA, RISING);

  stopMotor(); // Asegurar que el motor esté parado al inicio

  Serial.println("Arduino listo. Esperando comandos...");
}

void loop() {
  handleSerialCommands(); // Procesar comandos de Python

  unsigned long currentTime = millis();

  // Calcular velocidad y ejecutar PID a intervalos regulares
  if (currentTime - lastPidTime >= SAMPLE_TIME_MS) {
    lastPidTime = currentTime;

    // 1. Calcular RPM actuales
    long pulsesNow = encoderPulses;
    long deltaPulses = pulsesNow - lastEncoderPulses;
    lastEncoderPulses = pulsesNow;

    // Convertir pulsos/sample_time a RPM
    // (pulsos / PPR) = revoluciones
    // revoluciones / (SAMPLE_TIME_MS / 1000.0 / 60.0) = RPM (rev/min)
    currentRPM = ( (double)deltaPulses / PULSES_PER_REVOLUTION ) / ( (double)SAMPLE_TIME_MS / 60000.0 );

    if (!motorRunning || motorDirection == 0) {
      currentRPM = 0; // Si está parado, RPM es 0
      integralError = 0; // Resetear integral si está parado
      pwmOutput = 0;
    } else {
      // 2. Calcular PID
      double error = setpointRPM - currentRPM;
      integralError += error * (SAMPLE_TIME_MS / 1000.0); // Acumular error en el tiempo (segundos)

      // Anti-windup para el término integral (limitar su valor)
      integralError = constrain(integralError, -100.0, 100.0); // ¡Ajustar límites!

      double derivativeError = (error - previousError) / (SAMPLE_TIME_MS / 1000.0);
      previousError = error;

      pwmOutput = Kp * error + Ki * integralError + Kd * derivativeError;
    }

    // 3. Aplicar salida PWM al motor
    // Limitar PWM entre 0 y 255
    pwmOutput = constrain(pwmOutput, 0, 255);
    if (motorRunning && motorDirection != 0) {
      applyMotorSpeed(motorDirection, (int)pwmOutput);
    } else {
      stopMotor();
    }
  }

  // Enviar datos a Python periódicamente
  if (currentTime - lastSerialReportTime >= SERIAL_REPORT_INTERVAL_MS) {
    lastSerialReportTime = currentTime;
    Serial.print("R:");
    Serial.print(currentRPM, 2); // Enviar RPM actuales con 2 decimales
    Serial.print(",S:");
    Serial.print(setpointRPM, 2); // Enviar Setpoint
    Serial.print(",P:");
    Serial.println(pwmOutput, 0); // Enviar PWM
  }
}

// --- FUNCIONES AUXILIARES ---

// ISR (Interrupt Service Routine) para contar pulsos del encoder
void countPulseA() {
  // Opcional: leer ENCODER_B_PIN para determinar dirección si es un encoder cuadrático
  // if (digitalRead(ENCODER_B_PIN) == LOW) {
  //   encoderPulses++;
  // } else {
  //   encoderPulses--;
  // }
  encoderPulses++; // Simplificado: solo cuenta en una dirección, la dirección se establece por comando
}

void applyMotorSpeed(int direction, int speedVal) {
  if (direction == 1) { // Adelante
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else if (direction == -1) { // Atrás
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else { // Parar
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }
  analogWrite(ENA_PIN, speedVal);
}

void stopMotor() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  motorRunning = false;
  motorDirection = 0;
  //setpointRPM = 0; // Opcional: resetear setpoint al parar
  encoderPulses = 0;     // Resetear contador de pulsos
  lastEncoderPulses = 0;
  integralError = 0;     // Resetear integral del PID
  previousError = 0;
  pwmOutput = 0;
  currentRPM = 0;
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Quitar espacios en blanco

    if (command.startsWith("SET_RPM:")) {
      float rpm = command.substring(8).toFloat();
      setpointRPM = abs(rpm); // Usar valor absoluto para el setpoint de velocidad
      motorRunning = true;
      Serial.print("Setpoint RPM recibido: ");
      Serial.println(setpointRPM);
      // La dirección se establece por separado
    } else if (command.startsWith("SET_DIR:")) {
      int dir = command.substring(8).toInt();
      if (dir == 1) { // Adelante
        motorDirection = 1;
        motorRunning = true;
        Serial.println("Dirección: Adelante");
      } else if (dir == -1) { // Atrás
        motorDirection = -1;
        motorRunning = true;
        Serial.println("Dirección: Atrás");
      } else { // Parar (dir == 0)
        stopMotor();
        Serial.println("Motor Parado por comando DIR 0");
      }
    } else if (command.equals("STOP")) {
      stopMotor();
      Serial.println("Motor Parado por comando STOP");
    } else if (command.startsWith("KP:")) {
      Kp = command.substring(3).toFloat();
      Serial.print("Nuevo Kp: "); Serial.println(Kp);
    } else if (command.startsWith("KI:")) {
      Ki = command.substring(3).toFloat();
      Serial.print("Nuevo Ki: "); Serial.println(Ki);
    } else if (command.startsWith("KD:")) {
      Kd = command.substring(3).toFloat();
      Serial.print("Nuevo Kd: "); Serial.println(Kd);
    }
  }
}