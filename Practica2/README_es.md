# Control de Motor Paso a Paso con Arduino y PyQt5

## Descripción del Proyecto

Esta aplicación proporciona una interfaz gráfica de usuario (GUI) basada en PyQt5 para controlar un motor paso a paso conectado a Arduino. La aplicación permite controlar la dirección, velocidad y pasos por revolución del motor, además de proporcionar funciones de inicio/parada y parada de emergencia.

## Características

- **Panel de Conexión:** Selección del puerto COM, botones para conectar/desconectar y actualizar puertos disponibles.
- **Panel de Control:** 
  - Botones de dirección (Horario/Antihorario)
  - Control de velocidad (0-100 RPM)
  - Configuración de pasos por revolución (predeterminado: 48)
  - Botones de Inicio/Parada y Parada de Emergencia
- **Panel de Estado:** 
  - Indicadores de dirección actual y velocidad
  - Estado de operación
  - Área de registro de mensajes para monitorizar la comunicación con Arduino

## Requisitos

- Python 3.6 o superior
- PyQt5
- pyserial
- Un Arduino (Uno, Nano, Mega, etc.)
- Motor paso a paso y hardware de control (compatible con el sketch de Arduino proporcionado)

## Instalación

1. **Instalar las dependencias necesarias:**

```bash
pip install pyqt5 pyserial
```

2. **Cargar el sketch de Arduino:**
   - Abra el archivo `arduino_sketch.ino` en el IDE de Arduino
   - Conéctese a su placa Arduino
   - Compile y cargue el sketch en la placa

3. **Ejecutar la aplicación Python:**

```bash
python arduino_stepper_control.py
```

## Uso

1. **Conexión con Arduino:**
   - Seleccione el puerto COM correspondiente a su Arduino en el menú desplegable
   - Haga clic en "Conectar" para establecer la comunicación
   - El indicador de estado cambiará a verde si la conexión es exitosa

2. **Control del Motor:**
   - Establezca la dirección deseada usando los botones "Horario" o "Antihorario"
   - Ajuste la velocidad usando el control deslizante (en RPM)
   - Configure los pasos por revolución según su motor específico
   - Haga clic en "Iniciar" para comenzar la rotación del motor
   - Use "Parar" para detener el motor normalmente
   - Use "PARADA DE EMERGENCIA" para detener inmediatamente en caso de emergencia

3. **Monitoreo:**
   - El panel de estado muestra la dirección actual, velocidad y estado de operación
   - El área de registro muestra mensajes detallados sobre las operaciones y respuestas del Arduino

## Protocolo de Comunicación

La comunicación entre la GUI y Arduino utiliza comandos seriales simples:

- `CW` - Establece dirección horaria
- `CCW` - Establece dirección antihoraria
- `SPEED:X` - Establece velocidad a X RPM
- `STEPS:X` - Configura X pasos por revolución
- `START:DIR` - Inicia el motor (DIR puede ser CW o CCW)
- `STOP` - Detiene el motor
- `ESTOP` - Parada de emergencia

Arduino responde con mensajes como:
- `OK:COMANDO` - Confirma que un comando se completó correctamente
- `ERROR:mensaje` - Indica un error con el mensaje explicativo
- `clockwise` o `counterclockwise` - Confirmación de cambio de dirección

## Estructura del Proyecto

- `arduino_stepper_control.py` - Aplicación principal de la GUI en Python
- `arduino_sketch.ino` - Código para cargar en la placa Arduino

## Solución de Problemas

1. **No se detectan puertos COM:**
   - Verifique que el Arduino esté conectado correctamente
   - Reinstale los drivers del adaptador USB si es necesario
   - Haga clic en "Actualizar" para buscar puertos disponibles

2. **Error de conexión:**
   - Verifique que el puerto COM seleccionado sea correcto
   - Asegúrese de que no haya otras aplicaciones usando el mismo puerto
   - Reinicie el Arduino y la aplicación

3. **El motor no responde:**
   - Verifique las conexiones del hardware
   - Asegúrese de que el sketch correcto esté cargado en Arduino
   - Revise los mensajes de registro para detectar errores de comunicación

## Desarrollo y Mejoras Futuras

- Añadir soporte para controlar múltiples motores
- Implementar perfiles de movimiento (aceleración/desaceleración)
- Guardar y cargar configuraciones predefinidas
- Visualización gráfica de la posición del motor

## Licencia

Este proyecto está disponible bajo la licencia MIT. Para más detalles, consulte el archivo LICENSE.

