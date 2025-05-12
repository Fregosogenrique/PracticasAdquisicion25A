# Control de Velocidad de Motor con Encoder y PID

## Introducción

Este proyecto implementa un sistema de control de velocidad para un motor DC utilizando retroalimentación de encoder y un algoritmo de control PID. El sistema está compuesto por un Arduino que maneja el hardware del motor y un encoder, y una interfaz gráfica en Python que permite controlar y monitorear la operación del motor en tiempo real.

### Características Principales

- **Control Preciso de Velocidad**: Mantiene la velocidad deseada incluso bajo carga variable
- **Retroalimentación en Tiempo Real**: Monitoreo continuo de posición y velocidad a través del encoder
- **Ajuste PID Dinámico**: Interfaz para sintonizar los parámetros PID durante la operación
- **Visualización Gráfica**: Gráfico en tiempo real de velocidad objetivo vs. velocidad actual
- **Control de Dirección**: Cambio de dirección en horario y antihorario
- **Funciones de Seguridad**: Detección de estancamiento del motor y parada de emergencia
- **Registro de Eventos**: Historial detallado de operaciones y errores

## Hardware Requerido

### Componentes Necesarios

- Arduino UNO o compatible
- Motor DC con encoder incremental
- Puente H (L298N, TB6612 o similar)
- Fuente de alimentación para el motor (según especificaciones del motor)
- Cables de conexión
- Computadora con puerto USB

### Diagrama de Conexiones

```
+------------+       +------------+      +------------+
|            |       |            |      |            |
|  Arduino   |<----->| Puente H   |<---->|   Motor    |
|            |       |            |      | con Encoder|
+------------+       +------------+      +------------+
      ^
      |
      v
+------------+
|            |
| Computadora|
|            |
+------------+
```

### Conexiones Específicas

| Arduino | Dispositivo | Función |
|---------|-------------|---------|
| Pin 2   | Encoder Pin A | Señal encoder (Interrupción) |
| Pin 3   | Encoder Pin B | Señal encoder (Interrupción) |
| Pin 5   | Puente H PWM | Control de velocidad |
| Pin 7   | Puente H IN1 | Control de dirección 1 |
| Pin 8   | Puente H IN2 | Control de dirección 2 |
| 5V      | Alimentación | Alimentación de encoder |
| GND     | GND | Tierra común |

## Instalación del Software

### Requisitos Previos

- Arduino IDE (1.8.0 o superior)
- Python 3.6 o superior
- Bibliotecas Python:
  - PyQt5
  - pyserial
  - matplotlib

### Instalación de Dependencias de Python

```bash
pip install pyqt5 pyserial matplotlib
```

### Configuración del Arduino

1. Abrir el archivo `motor_speed_control.ino` en Arduino IDE
2. Conectar la placa Arduino mediante USB
3. Seleccionar la placa y puerto correctos en Arduino IDE
4. Modificar la constante `ENCODER_RESOLUTION` según las especificaciones de tu encoder
5. Cargar el sketch en la placa Arduino

### Ejecución de la Interfaz Python

```bash
python motor_speed_control.py
```

## Guía de Uso

### Conexión Inicial

1. Lanzar la aplicación Python
2. En el panel de "Configuración de Conexión", seleccionar el puerto COM correcto
3. Hacer clic en "Conectar"
4. Verificar que el estado cambie a "Conectado" en verde

### Control del Motor

1. **Control de Dirección**:
   - Seleccionar "Horario (CW)" o "Antihorario (CCW)" según la dirección deseada

2. **Control de Velocidad**:
   - Ajustar la velocidad usando el deslizador o el campo numérico
   - La velocidad se expresa en RPM (Revoluciones Por Minuto)

3. **Inicio/Parada**:
   - Hacer clic en "Iniciar" para comenzar la operación del motor
   - Usar "Detener" para parar el motor de manera controlada
   - En caso de emergencia, usar el botón rojo "PARADA DE EMERGENCIA"

### Monitoreo

- El panel de monitoreo muestra en tiempo real:
  - Velocidad actual (RPM)
  - Posición del encoder
  - Contador de rotaciones
  - Valor de PWM aplicado
  - Estado de operación

- El gráfico de velocidad muestra:
  - Línea azul: Velocidad actual
  - Línea roja punteada: Velocidad objetivo

- El panel de registro muestra eventos e información de diagnóstico

## Ajuste del Controlador PID

### Parámetros PID

En la pestaña "Sintonización PID", se pueden modificar los siguientes parámetros:

- **Ganancia Proporcional (Kp)**: Respuesta proporcional al error actual
- **Ganancia Integral (Ki)**: Respuesta a la acumulación de errores pasados
- **Ganancia Derivativa (Kd)**: Respuesta a la tasa de cambio del error

### Método de Sintonización Recomendado

1. Establecer todos los valores (Kp, Ki, Kd) a cero
2. Aumentar Kp gradualmente hasta obtener una respuesta rápida pero con ligeras oscilaciones
3. Aumentar Kd para reducir las oscilaciones
4. Aumentar Ki para eliminar el error en estado estacionario
5. Refinar los valores mientras observa el gráfico de velocidad

### Efectos de los Parámetros

| Parámetro | Aumentar Valor | Disminuir Valor |
|-----------|----------------|-----------------|
| Kp | Mayor respuesta, posible inestabilidad | Respuesta más lenta, menor corrección |
| Ki | Elimina error estacionario, posible sobreimpulso | Menor corrección de errores acumulados |
| Kd | Amortiguación, menor oscilación | Menor amortiguación, más sensible al ruido |

## Solución de Problemas

### Problemas Comunes

| Problema | Posibles Causas | Solución |
|----------|----------------|----------|
| No se detectan puertos COM | Arduino no conectado, drivers no instalados | Verificar conexión USB, reinstalar drivers |
| Error de conexión | Puerto incorrecto, Arduino en uso por otro programa | Seleccionar puerto correcto, cerrar otros programas usando Arduino |
| Motor no gira | Conexiones incorrectas, PWM muy bajo | Verificar cableado, aumentar velocidad |
| Oscilación excesiva | Valores PID no óptimos | Reducir Kp, aumentar Kd |
| Detección de estancamiento frecuente | Carga excesiva, umbral muy sensible | Verificar carga del motor, ajustar STALL_THRESHOLD |
| Velocidad imprecisa | Resolución de encoder incorrecta | Ajustar ENCODER_RESOLUTION en el sketch |
| Gráfico no se actualiza | Problema de comunicación | Reiniciar aplicación, verificar conexión |

### Diagnóstico

- Revisar el panel de registro para mensajes de error detallados
- Verificar que los valores de PWM sean suficientes para mover el motor
- Comprobar que las conexiones del encoder sean correctas y estables

## Notas Adicionales y Advertencias de Seguridad

### Advertencias

- **¡PRECAUCIÓN!** Los motores pueden arrancar inesperadamente. Mantener objetos y dedos alejados de partes móviles.
- No desconectar cables mientras el motor está en funcionamiento.
- Asegurar que el motor esté montado de forma segura antes de operarlo a alta velocidad.
- La detección de estancamiento no reemplaza adecuadas protecciones físicas.
- El freno de emergencia (ambos pines del puente H activos) puede generar corrientes elevadas.

### Notas sobre el Rendimiento

- La resolución del encoder afecta directamente la precisión del control de velocidad.
- El rendimiento del PID puede variar según las características del motor y la carga.
- Para aplicaciones de precisión, se recomienda calibrar el sistema para su motor específico.
- El ajuste óptimo de parámetros PID puede requerir experimentación.

### Extensiones Posibles

- Agregar control de posición absoluta
- Implementar perfiles de aceleración/desaceleración
- Añadir soporte para múltiples motores
- Incorporar comunicación remota o IoT
- Guardar y cargar configuraciones de PID

---

Este proyecto es parte de las prácticas de adquisición de datos y control utilizando Arduino y Python, demostrando conceptos de retroalimentación, control en tiempo real y procesamiento de señales.

