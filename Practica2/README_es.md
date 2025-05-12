# Práctica 2: Control de Motor con Encoder y Arduino

## Descripción General

Esta práctica implementa un sistema de control para un motor con encoder utilizando Arduino y una interfaz gráfica en Python. El sistema permite la rotación continua del motor con cambios de orientación en tiempo real, mientras monitorea la posición y velocidad a través de un encoder.

A diferencia de los motores paso a paso que se controlan por pasos discretos, esta implementación utiliza un motor DC con encoder que permite:
- Rotación continua con monitoreo preciso de posición
- Cambios de dirección durante la operación
- Medición de velocidad en tiempo real
- Detección de fallos como estancamiento del motor

## Características Principales

- **Control de Dirección**: Cambio de sentido horario/antihorario en tiempo real
- **Rotación Continua**: Modo de operación continua sin paradas
- **Monitoreo de Posición**: Seguimiento preciso mediante encoder
- **Medición de Velocidad**: Cálculo de RPM en tiempo real desde el encoder
- **Contador de Rotaciones**: Seguimiento del número de rotaciones completas
- **Control de Velocidad**: Ajuste mediante PWM (0-255)
- **Características de Seguridad**: 
  - Detección de estancamiento del motor
  - Aceleración/desaceleración controlada
  - Parada de emergencia
  - Manejo robusto de errores

## Requisitos de Hardware

- Arduino (UNO, Mega o compatible)
- Motor DC con encoder incremental
- Driver para motor DC (L298N, TB6612 o similar)
- Fuente de alimentación adecuada para el motor
- Cables para conexiones
- Computadora con puerto USB

### Conexiones de Hardware

- **Encoder**:
  - Pin A del encoder → Pin 2 del Arduino (Interrupción)
  - Pin B del encoder → Pin 3 del Arduino (Interrupción)
  - VCC del encoder → 5V del Arduino
  - GND del encoder → GND del Arduino

- **Driver del Motor**:
  - Pin PWM del driver → Pin 5 del Arduino
  - Pin de dirección del driver → Pin 7 del Arduino
  - Conexiones de alimentación según especificaciones del driver

## Componentes de Software

El sistema consta de dos componentes principales:

### 1. Firmware Arduino (arduino_sketch.ino)

El firmware se encarga de:
- Leer señales del encoder mediante interrupciones
- Controlar el motor mediante PWM
- Calcular la posición y velocidad en tiempo real
- Ejecutar las instrucciones recibidas por puerto serie
- Implementar medidas de seguridad

### 2. Interfaz Gráfica Python (arduino_stepper_control.py)

La interfaz proporciona:
- Panel de conexión para gestionar la comunicación serial
- Controles para dirección, velocidad y modo continuo
- Visualización de posición, velocidad y conteo de rotaciones
- Log de eventos para monitoreo y depuración
- Manejo de errores y reconexión automática

## Instalación y Configuración

### Software Requerido

- Arduino IDE (para cargar el firmware)
- Python 3.6 o superior
- Bibliotecas Python: PyQt5, pyserial

### Pasos de Instalación

1. **Configuración del Arduino**:
   - Conecta el Arduino a tu computadora
   - Abre arduino_sketch.ino en Arduino IDE
   - Modifica la constante `ENCODER_RESOLUTION` si tu encoder tiene diferente resolución
   - Carga el sketch en el Arduino

2. **Configuración Python**:
   - Instala las dependencias:
   ```bash
   pip install PyQt5 pyserial
   ```
   - Navega al directorio de la práctica y ejecuta la interfaz:
   ```bash
   python arduino_stepper_control.py
   ```

## Instrucciones de Uso

1. **Conexión**:
   - Selecciona el puerto COM correcto del desplegable
   - Haz clic en "Conectar"
   - Verifica que el estado cambie a "Connected" (verde)

2. **Control Básico**:
   - **Dirección**: Selecciona "Horario" o "Antihorario"
   - **Velocidad**: Ajusta el deslizador para controlar la velocidad (0-255)
   - **Inicio/Parada**: Presiona "Iniciar" para comenzar la rotación, "Detener" para parar

3. **Modo Continuo**:
   - Haz clic en "Activar" para iniciar el modo continuo
   - El motor mantendrá una rotación constante hasta que se desactive

4. **Monitoreo**:
   - El panel de estado muestra:
     - Posición actual del encoder
     - Velocidad real (RPM)
     - Recuento de rotaciones completas
     - Estado del modo continuo
     - Dirección actual

5. **Parada de Emergencia**:
   - Presiona "PARADA DE EMERGENCIA" para detener inmediatamente el motor

## Detalles Técnicos del Encoder

### Funcionamiento del Encoder

Esta implementación utiliza un encoder incremental con dos canales (A y B) en cuadratura para determinar:
- Posición exacta mediante conteo de pulsos
- Dirección de rotación mediante detección de fase entre canales
- Velocidad por cálculo de tiempo entre pulsos

### Cálculo de Posición y Velocidad

- **Posición**: Se determina contando los pulsos del encoder. Cada flanco (rising/falling) en los canales genera un incremento/decremento según la dirección.

- **Velocidad**: Se calcula mediante la fórmula:
  ```
  RPM = (cambio_posición / resolución_encoder) * (60 / tiempo_segundos)
  ```
  donde:
  - `cambio_posición` es la diferencia de pulsos en el intervalo
  - `resolución_encoder` es el número de pulsos por revolución
  - `tiempo_segundos` es el intervalo de muestreo

- **Conteo de Rotaciones**: Se incrementa cada vez que la posición alcanza un múltiplo de la resolución del encoder.

## Características de Seguridad

### Limitación de Aceleración

El sistema implementa una aceleración controlada que:
- Limita la tasa de cambio de PWM a `ACCEL_RATE` por ciclo
- Previene cambios bruscos que podrían dañar el motor o la mecánica
- Proporciona arranques y paradas suaves

### Detección de Estancamiento

El sistema monitorea constantemente el movimiento del motor:
- Verifica que la posición del encoder cambie cuando el motor está energizado
- Si no se detectan cambios durante `STALL_TIMEOUT` milisegundos, se asume estancamiento
- Ante un estancamiento, se detiene el motor y se notifica el error

### Comandos de Emergencia

La implementación incluye un comando de parada de emergencia (ESTOP):
- Detiene el motor inmediatamente con máxima prioridad
- Ignora la cola de comandos para respuesta inmediata
- Reinicia todos los estados de operación

### Manejo de Errores de Comunicación

La interfaz incluye un sistema robusto de manejo de errores:
- Detección de desconexiones
- Reintentos automáticos de conexión
- Timeout para comandos sin respuesta
- Recuperación de estado tras reconexión

## Comunicación Serial

### Protocolo de Comunicación

La comunicación entre Arduino y Python utiliza un protocolo basado en texto:

**Comandos enviados al Arduino**:
- `SET_DIR:CW/CCW` - Establece dirección (horario/antihorario)
- `SET_SPEED:<0-255>` - Establece velocidad por PWM
- `SET_CONTINUOUS:ON/OFF` - Activa/desactiva rotación continua
- `GET_POS` - Solicita posición actual
- `GET_SPEED` - Solicita velocidad actual
- `STOP` - Detiene el motor normalmente
- `ESTOP` - Parada de emergencia

**Respuestas del Arduino**:
- `POS:<valor>` - Posición actual del encoder
- `SPEED:<valor>` - Velocidad actual en RPM
- `OK:<comando>` - Confirmación de comando
- `ERROR:<mensaje>` - Notificación de error

## Resolución de Problemas

- **El motor no gira**: Verifica conexiones físicas y nivel de PWM
- **Lecturas erráticas del encoder**: Revisa ruido en señales y considera usar filtro
- **Errores de comunicación**: Verifica el puerto COM y velocidad de baudios
- **Estancamiento frecuente**: Ajusta `STALL_THRESHOLD` según el encoder
- **Aceleración demasiado lenta/rápida**: Modifica `ACCEL_RATE` en el firmware

---

Esta práctica demuestra la aplicación de conceptos fundamentales de adquisición de datos y control, implementando un sistema de retroalimentación cerrado con encoder para el control preciso de posición y velocidad de un motor.

