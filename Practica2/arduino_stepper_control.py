#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Interfaz Gráfica para Control de Motor con Encoder y Arduino
===========================================================
Una interfaz gráfica de usuario basada en PyQt5 para controlar un motor con encoder
conectado a Arduino. Esta aplicación proporciona controles para la dirección del motor,
velocidad, modo continuo y monitoreo de posición del encoder.

Desarrollado para prácticas de adquisición de datos y control de dispositivos.
Permite la comunicación serial con un Arduino para enviar comandos de control.

Características principales:
- Conexión y desconexión con Arduino mediante puerto serie
- Control de dirección (horario/antihorario)
- Ajuste de velocidad mediante slider
- Monitoreo en tiempo real de la posición del encoder
- Modo de rotación continua
- Indicador de velocidad actual desde el encoder
- Inicio/Parada y Parada de Emergencia
- Monitoreo de estado y registro de mensajes
"""

import sys
import serial
import serial.tools
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGridLayout, QLabel, QPushButton, 
                            QComboBox, QSlider, QSpinBox, QGroupBox, 
                            QTextEdit, QFrame, QSplitter)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont, QIcon, QColor
import time
import threading

class MotorEncoderGUI(QMainWindow):
    """
    Ventana principal de la aplicación para el control de motor con encoder.
    
    Esta clase implementa la interfaz gráfica completa con todos los controles
    necesarios para la comunicación con Arduino y el control del motor con encoder.
    Incluye paneles para conexión, control del motor y monitoreo de estado y posición.
    """
    
    def __init__(self):
        """
        Inicializa la ventana principal y configura todos los componentes de la GUI.
        
        Crea los tres paneles principales (conexión, control y estado), establece
        la configuración inicial y prepara los temporizadores para la comunicación serial.
        """
        super().__init__()
        
        # Inicializa propiedades de la interfaz
        self.setWindowTitle("Control de Motor con Encoder y Arduino")
        self.setMinimumSize(800, 600)
        
        # Configura el widget central y el layout principal
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        
        # Crea los tres paneles (panel de estado primero para que log_text esté disponible)
        self.setup_status_panel()
        self.setup_connection_panel()
        self.setup_control_panel()
        
        # Registra mensaje inicial después de crear el panel de estado
        self.log_message("Aplicación iniciada. Por favor conecte el Arduino.")
        
        # Inicializa la conexión serial (aún no conectada)
        self.serial_port = None
        self.is_connected = False
        
        # Inicializa variables de control del motor
        self.current_direction = None  # Dirección actual: None, "clockwise" o "counterclockwise"
        self.current_speed = 60        # Velocidad predeterminada (PWM 0-255)
        self.is_running = False        # Estado de funcionamiento del motor
        self.continuous_mode = False   # Modo de rotación continua
        self.encoder_position = 0      # Posición actual del encoder
        self.actual_speed = 0.0        # Velocidad real medida del encoder (RPM)
        self.rotation_count = 0        # Contador de rotaciones completas
        
        # Configura el temporizador para lectura serial
        self.serial_timer = QTimer(self)
        self.serial_timer.timeout.connect(self.read_serial)
        self.serial_timer.setInterval(100)  # Verifica datos seriales cada 100ms
        
        # Configura el temporizador para actualizar la información del encoder
        self.encoder_timer = QTimer(self)
        self.encoder_timer.timeout.connect(self.update_encoder_status)
        self.encoder_timer.setInterval(500)  # Actualiza la posición del encoder cada 500ms
        
        # Configura el temporizador para timeout de respuesta de comandos
        self.response_timer = QTimer(self)
        self.response_timer.timeout.connect(self.handle_response_timeout)
        self.response_timer.setSingleShot(True)  # Temporizador de un solo disparo
        
        # Cola de comandos para operaciones seriales
        self.command_queue = []        # Cola para almacenar comandos pendientes
        self.awaiting_response = False # Indica si se espera respuesta de un comando
        self.last_command = None       # Último comando enviado
        
        # Actualiza la interfaz para reflejar el estado de conexión
        self.update_connection_status()
        
    def setup_connection_panel(self):
        """
        Crea y configura el panel de conexión.
        
        Este panel contiene controles para:
        - Selección del puerto COM
        - Botón para actualizar puertos disponibles
        - Botón para conectar/desconectar
        - Indicador de estado de conexión
        """
        # Crea un grupo para el panel de conexión
        connection_group = QGroupBox("Configuración de Conexión")
        connection_layout = QHBoxLayout()
        
        # Menú desplegable para selección de puerto COM
        self.port_label = QLabel("Puerto COM:")
        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("Actualizar")
        self.refresh_button.clicked.connect(self.refresh_ports)
        
        # Botón para conectar/desconectar
        self.connect_button = QPushButton("Conectar")
        self.connect_button.clicked.connect(self.toggle_connection)
        
        # Indicador de estado de conexión
        self.status_label = QLabel("Desconectado")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        
        # Add widgets to the connection layout
        connection_layout.addWidget(self.port_label)
        connection_layout.addWidget(self.port_combo)
        connection_layout.addWidget(self.refresh_button)
        connection_layout.addWidget(self.connect_button)
        connection_layout.addWidget(self.status_label)
        connection_layout.addStretch()
        
        # Set the layout for the connection group
        connection_group.setLayout(connection_layout)
        
        # Add the connection group to the main layout
        self.main_layout.addWidget(connection_group)
        
        # Populate the ports dropdown initially
        self.refresh_ports()
        
    def setup_control_panel(self):
        """
        Crea y configura el panel de control del motor.
        
        Este panel contiene controles para:
        - Dirección del motor (horario/antihorario)
        - Control de velocidad mediante slider
        - Toggle para modo continuo
        - Botones de inicio/parada y parada de emergencia
        """
        # Crea un grupo para el panel de control
        control_group = QGroupBox("Control del Motor")
        control_layout = QGridLayout()
        
        # Controles de dirección
        direction_label = QLabel("Dirección:")
        self.cw_button = QPushButton("Horario")
        self.cw_button.clicked.connect(lambda: self.set_direction("clockwise"))
        self.ccw_button = QPushButton("Antihorario")
        self.ccw_button.clicked.connect(lambda: self.set_direction("counterclockwise"))
        
        # Control de velocidad
        speed_label = QLabel("Velocidad (PWM):")
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(255)  # PWM range 0-255
        self.speed_slider.setValue(60)     # Default speed
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(25)
        self.speed_slider.valueChanged.connect(self.update_speed)
        
        self.speed_value_label = QLabel("60")
        
        # Toggle de modo continuo
        continuous_label = QLabel("Modo Continuo:")
        self.continuous_toggle = QPushButton("Activar")
        self.continuous_toggle.setCheckable(True)
        self.continuous_toggle.clicked.connect(self.toggle_continuous_mode)
        
        # Start/Stop and Emergency Stop buttons
        # Botones de inicio/parada y parada de emergencia
        self.start_stop_button = QPushButton("Iniciar")
        self.start_stop_button.clicked.connect(self.toggle_motor)
        
        self.emergency_stop_button = QPushButton("PARADA DE EMERGENCIA")
        self.emergency_stop_button.clicked.connect(self.emergency_stop)
        
        # Add widgets to the control layout
        control_layout.addWidget(direction_label, 0, 0)
        control_layout.addWidget(self.cw_button, 0, 1)
        control_layout.addWidget(self.ccw_button, 0, 2)
        
        control_layout.addWidget(speed_label, 1, 0)
        control_layout.addWidget(self.speed_slider, 1, 1, 1, 2)
        control_layout.addWidget(self.speed_value_label, 1, 3)
        
        control_layout.addWidget(continuous_label, 2, 0)
        control_layout.addWidget(self.continuous_toggle, 2, 1)
        
        control_layout.addWidget(self.start_stop_button, 3, 0, 1, 2)
        control_layout.addWidget(self.emergency_stop_button, 3, 2, 1, 2)
        
        # Set the layout for the control group
        control_group.setLayout(control_layout)
        
        # Add the control group to the main layout
        self.main_layout.addWidget(control_group)
        
    def setup_status_panel(self):
        """
        Crea y configura"""
        # Create a group box for the status panel
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        
        # Create a grid for status indicators
        indicators_layout = QGridLayout()
        
        # Current direction indicator
        direction_status_label = QLabel("Current Direction:")
        self.direction_value_label = QLabel("Not Set")
        
        # Current speed indicator
        speed_status_label = QLabel("Target Speed (PWM):")
        self.speed_status_value = QLabel("0")
        
        # Actual speed from encoder
        actual_speed_label = QLabel("Actual Speed (RPM):")
        self.actual_speed_value = QLabel("0.0")
        
        # Encoder position
        encoder_pos_label = QLabel("Encoder Position:")
        self.encoder_pos_value = QLabel("0")
        
        # Rotation count
        rotation_count_label = QLabel("Rotation Count:")
        self.rotation_count_value = QLabel("0")
        
        # Continuous mode status
        continuous_status_label = QLabel("Continuous Mode:")
        self.continuous_status_value = QLabel("OFF")
        
        # Operation status
        operation_status_label = QLabel("Operation Status:")
        self.operation_status_value = QLabel("Idle")
        
        # Add indicators to the layout
        indicators_layout.addWidget(direction_status_label, 0, 0)
        indicators_layout.addWidget(self.direction_value_label, 0, 1)
        
        indicators_layout.addWidget(speed_status_label, 1, 0)
        indicators_layout.addWidget(self.speed_status_value, 1, 1)
        
        indicators_layout.addWidget(actual_speed_label, 2, 0)
        indicators_layout.addWidget(self.actual_speed_value, 2, 1)
        
        indicators_layout.addWidget(encoder_pos_label, 3, 0)
        indicators_layout.addWidget(self.encoder_pos_value, 3, 1)
        
        indicators_layout.addWidget(rotation_count_label, 4, 0)
        indicators_layout.addWidget(self.rotation_count_value, 4, 1)
        
        indicators_layout.addWidget(continuous_status_label, 5, 0)
        indicators_layout.addWidget(self.continuous_status_value, 5, 1)
        
        indicators_layout.addWidget(operation_status_label, 6, 0)
        indicators_layout.addWidget(self.operation_status_value, 6, 1)
        
        # Add the indicators layout to the status layout
        status_layout.addLayout(indicators_layout)
        
        # Add a separator line
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        status_layout.addWidget(separator)
        
        # Add a text area for logging
        log_label = QLabel("Log Messages:")
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        
        status_layout.addWidget(log_label)
        status_layout.addWidget(self.log_text)
        
        # Set the layout for the status group
        status_group.setLayout(status_layout)
        
        # Add the status group to the main layout
        self.main_layout.addWidget(status_group)
        
        # Panel is now ready for messages
        
    def refresh_ports(self):
        """Refresh the list of available serial ports."""
        self.port_combo.clear()
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            self.port_combo.addItem(port.device)
        
        if len(ports) == 0:
            self.log_message("No serial ports found. Please connect Arduino.")
        else:
            self.log_message(f"Found {len(ports)} serial ports.")
            
    def toggle_connection(self):
        """Connect to or disconnect from the selected serial port."""
        if not self.is_connected:
            # Get the selected port
            if self.port_combo.currentText() == "":
                self.log_message("Error: No port selected.")
                return
                
            port = self.port_combo.currentText()
            
            try:
                # Try to open the serial connection
                self.serial_port = serial.Serial(port, 9600, timeout=1)
                # Wait for Arduino to reset (typical for Arduino boards)
                time.sleep(2)
                self.is_connected = True
                self.log_message(f"Connected to {port} at 9600 baud.")
                
                # Start the serial reading timer
                self.serial_timer.start()
                
                # Send initial commands to set up the Arduino
                self.send_command(f"SET_SPEED:{self.current_speed}")
                # Start the encoder update timer
                self.encoder_timer.start()
            except serial.SerialException as e:
                self.log_message(f"Error connecting to {port}: {str(e)}")
                return
        else:
            # Disconnect from the serial port
            # Stop the serial reading timer first
            self.serial_timer.stop()
            self.response_timer.stop()
            # Stop the encoder update timer
            self.encoder_timer.stop()
            
            # If motor is running, stop it before disconnecting
            if self.is_running:
                self.send_command("STOP", wait_response=False)
                self.is_running = False
                
            # Now close the serial port
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.is_connected = False
            self.serial_port = None
            self.log_message("Disconnected from serial port.")
            
        # Update the UI to reflect the connection status
        self.update_connection_status()
            
    def update_connection_status(self):
        """Update the UI elements based on connection status."""
        if self.is_connected:
            self.connect_button.setText("Disconnect")
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            
            # Enable control elements
            self.cw_button.setEnabled(True)
            self.ccw_button.setEnabled(True)
            self.speed_slider.setEnabled(True)
            self.continuous_toggle.setEnabled(True)
            self.start_stop_button.setEnabled(True)
            self.emergency_stop_button.setEnabled(True)
        else:
            self.connect_button.setText("Connect")
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            
            # Disable control elements
            self.cw_button.setEnabled(False)
            self.ccw_button.setEnabled(False)
            self.speed_slider.setEnabled(False)
            self.continuous_toggle.setEnabled(False)
            self.start_stop_button.setEnabled(False)
            self.emergency_stop_button.setEnabled(False)
            
            # Reset status indicators
            self.direction_value_label.setText("Not Set")
            self.speed_status_value.setText("0")
            self.actual_speed_value.setText("0.0")
            self.encoder_pos_value.setText("0")
            self.rotation_count_value.setText("0")
            self.continuous_status_value.setText("OFF")
            self.operation_status_value.setText("Idle")
    
    def set_direction(self, direction):
        """Set the motor direction."""
        if not self.is_connected:
            return
            
        self.current_direction = direction
        self.direction_value_label.setText(direction.capitalize())
        self.log_message(f"Direction set to {direction}.")
        
        # Send direction command to Arduino
        if direction == "clockwise":
            self.send_command("SET_DIR:CW")
        else:
            self.send_command("SET_DIR:CCW")
        
    def update_speed(self):
        """Update the speed display when the slider is moved."""
        speed = self.speed_slider.value()
        self.current_speed = speed
        self.speed_value_label.setText(f"{speed}")
        self.speed_status_value.setText(f"{speed}")
        
        # Send the speed command to Arduino
        self.send_command(f"SET_SPEED:{speed}")
        
    def toggle_continuous_mode(self):
        """Toggle continuous rotation mode."""
        if not self.is_connected:
            return
            
        # Toggle mode
        self.continuous_mode = not self.continuous_mode
        
        if self.continuous_mode:
            self.continuous_toggle.setText("Desactivar")
            self.continuous_status_value.setText("ON")
            self.log_message("Continuous rotation mode activated.")
            self.send_command("SET_CONTINUOUS:ON")
        else:
            self.continuous_toggle.setText("Activar")
            self.continuous_status_value.setText("OFF")
            self.log_message("Continuous rotation mode deactivated.")
            self.send_command("SET_CONTINUOUS:OFF")
            
    def update_encoder_status(self):
        """Request and update encoder position and speed information."""
        if not self.is_connected:
            return
            
        # Request current position
        self.send_command("GET_POS")
        
        # Request current speed
        self.send_command("GET_SPEED")
    
    def toggle_motor(self):
        """Start or stop the motor."""
        if not self.is_connected:
            return
            
        # Toggle button state
        if self.start_stop_button.text() == "Iniciar":
            self.start_stop_button.setText("Detener")
            self.operation_status_value.setText("Running")
            self.log_message("Motor started.")
            self.is_running = True
            
            # Send start command with current direction
            if self.current_direction == "clockwise":
                self.send_command("SET_DIR:CW")
            elif self.current_direction == "counterclockwise":
                self.send_command("SET_DIR:CCW")
            else:
                # Default to clockwise if no direction set
                self.send_command("SET_DIR:CW")
                self.current_direction = "clockwise"
                self.direction_value_label.setText("Clockwise")
                
            # Start the motor with current speed
            self.send_command(f"SET_SPEED:{self.current_speed}")
        else:
            self.start_stop_button.setText("Iniciar")
            self.operation_status_value.setText("Stopped")
            self.log_message("Motor stopped.")
            self.is_running = False
            
            # Send stop command to Arduino
            self.send_command("STOP")
    
    def emergency_stop(self):
        """Emergency stop the motor."""
        if not self.is_connected:
            return
            
        self.start_stop_button.setText("Iniciar")
        self.operation_status_value.setText("EMERGENCY STOPPED")
        self.log_message("EMERGENCY STOP activated!", "red")
        self.is_running = False
        
        # Send emergency stop command to Arduino with high priority
        self.send_command("ESTOP", wait_response=False)
        
        # To ensure it's processed immediately, clear any pending commands
        self.command_queue = []
        self.awaiting_response = False
    
    def log_message(self, message, color="black"):
        """Add a message to the log with optional color."""
        self.log_text.append(f"<span style='color:{color};'>{message}</span>")
        
    def closeEvent(self, event):
        """Handle application close event."""
        # Stop timers
        self.serial_timer.stop()
        self.response_timer.stop()
        
        # Close serial port
        if self.serial_port and self.serial_port.is_open:
            try:
                # Try to stop the motor before closing
                if self.is_running:
                    self.serial_port.write("STOP\n".encode())
                    time.sleep(0.1)  # Small delay to allow command to be processed
            except:
                pass  # Ignore errors during close
            self.serial_port.close()
        event.accept()

    def send_command(self, command, wait_response=True, timeout=2000):
        """
        Send a command to the Arduino.
        
        Args:
            command (str): The command to send
            wait_response (bool): Whether to wait for a response
            timeout (int): Timeout in milliseconds for response
        """
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            self.log_message("Cannot send command - not connected", "red")
            return False
        
        # Add command to queue
        if self.awaiting_response and wait_response:
            self.command_queue.append((command, wait_response))
            self.log_message(f"Command '{command}' queued")
            return True
            
        try:
            # Format command with newline terminator
            cmd_str = f"{command}\n"
            self.serial_port.write(cmd_str.encode())
            self.log_message(f"Sent: {command}")
            
            if wait_response:
                self.last_command = command
                self.awaiting_response = True
                # Start timeout timer
                self.response_timer.start(timeout)
            
            return True
            
        except serial.SerialException as e:
            self.log_message(f"Error sending command: {str(e)}", "red")
            # If there was an error, try to reconnect
            self.handle_connection_error()
            return False
    
    def read_serial(self):
        """Read data from the serial port and process it."""
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            return
            
        try:
            # Check if data is available to read
            if self.serial_port.in_waiting > 0:
                # Read a line (until \n)
                data = self.serial_port.readline().decode('utf-8').strip()
                
                if data:
                    self.log_message(f"Received: {data}")
                    self.process_response(data)
                    
        except serial.SerialException as e:
            self.log_message(f"Error reading from serial port: {str(e)}", "red")
            self.handle_connection_error()
    
    def process_response(self, response):
        """Process a response received from the Arduino."""
        # Cancel the response timeout timer
        self.response_timer.stop()
        
        # Mark that we're no longer waiting for a response
        self.awaiting_response = False
        
        # Process encoder position and speed feedback
        if response.startswith("POS:"):
            # Update encoder position display
            position = response[4:]
            try:
                pos_value = int(position)
                self.encoder_position = pos_value
                self.encoder_pos_value.setText(str(pos_value))
                
                # Calculate rotation count (from encoder position)
                encoder_resolution = 1200  # This should match ENCODER_RESOLUTION in Arduino
                self.rotation_count = abs(pos_value) // encoder_resolution
                self.rotation_count_value.setText(str(self.rotation_count))
            except ValueError:
                self.log_message(f"Invalid position value: {position}", "red")
        
        elif response.startswith("ROT:"):
            # Direct update of rotation count from Arduino
            rot_count = response[4:]
            try:
                self.rotation_count = int(rot_count)
                self.rotation_count_value.setText(str(self.rotation_count))
            except ValueError:
                self.log_message(f"Invalid rotation count: {rot_count}", "red")
        
        elif response.startswith("SPEED:"):
            # Update actual speed display
            speed = response[6:]
            try:
                speed_value = float(speed)
                self.actual_speed = speed_value
                self.actual_speed_value.setText(f"{speed_value:.1f}")
            except ValueError:
                self.log_message(f"Invalid speed value: {speed}", "red")
            
        elif response.startswith("ERROR:"):
            # Command error
            error = response[6:]  # Extract error message
            self.log_message(f"Command error: {error}", "red")
            
        elif response == "clockwise" or response == "counterclockwise":
            # Direction update from Arduino
            self.current_direction = response
            self.direction_value_label.setText(response.capitalize())
            
        # Process any queued commands now that we've received a response
        self.process_command_queue()
            
    def process_command_queue(self):
        """Process the next command in the queue if any."""
        if not self.command_queue or self.awaiting_response:
            return
            
        # Get the next command
        command, wait_response = self.command_queue.pop(0)
        
        # Send it
        self.send_command(command, wait_response)
    
    def handle_response_timeout(self):
        """Handle a timeout waiting for a response from Arduino."""
        if not self.awaiting_response:
            return
            
        self.log_message(f"Timeout waiting for response to command: {self.last_command}", "red")
        
        # We're no longer waiting for a response
        self.awaiting_response = False
        
        # Try to process any queued commands
        self.process_command_queue()
    
    def handle_connection_error(self):
        """Handle a serial connection error."""
        self.log_message("Connection error detected. Attempting to reconnect...", "red")
        
        # Stop timers
        self.serial_timer.stop()
        self.response_timer.stop()
        
        # Close the serial port if it's open
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except:
                pass
                
        # Mark as disconnected
        self.is_connected = False
        self.serial_port = None
        
        # Update the UI
        self.update_connection_status()
        
        # Attempt to reconnect after a short delay
        QTimer.singleShot(3000, self.attempt_reconnect)
    
    def attempt_reconnect(self):
        """Attempt to reconnect to the last used port."""
        if self.is_connected:
            return  # Already reconnected
            
        port = self.port_combo.currentText()
        if not port:
            self.log_message("Cannot reconnect - no port selected", "red")
            return
            
        self.log_message(f"Attempting to reconnect to {port}...")
        
        try:
            # Try to reopen the serial connection
            self.serial_port = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            self.is_connected = True
            self.log_message(f"Reconnected to {port} at 9600 baud.", "green")
            
            # Restart the serial reading timer
            self.serial_timer.start()
            
            # Resend configuration commands
            # Resend configuration commands if previously set
            if self.current_speed > 0:
                self.send_command(f"SET_SPEED:{self.current_speed}")
                
            if self.current_direction:
                if self.current_direction == "clockwise":
                    self.send_command("SET_DIR:CW")
                else:
                    self.send_command("SET_DIR:CCW")
                    
            if self.continuous_mode:
                self.send_command("SET_CONTINUOUS:ON")
                
            # Restart encoder updates
            self.encoder_timer.start()
            
            # Update the UI
            self.update_connection_status()
            
        except serial.SerialException as e:
            self.log_message(f"Failed to reconnect: {str(e)}", "red")
            # Try again after a delay
            QTimer.singleShot(5000, self.attempt_reconnect)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorEncoderGUI()
    window.show()
    sys.exit(app.exec_())
