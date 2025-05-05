#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
DC Motor Control GUI
===================
A graphical user interface for controlling a DC motor through an Arduino
and H-bridge circuit. This application provides controls for motor speed,
direction, and operation status.

Developed for Practica1 of the data acquisition lab.
"""

import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGridLayout, QLabel, QPushButton, 
                            QComboBox, QSlider, QGroupBox, QTextEdit, QFrame)
from PyQt5.QtCore import Qt, QTimer, pyqtSlot
from PyQt5.QtGui import QFont, QColor
import time
import threading

class DCMotorControlGUI(QMainWindow):
    """
    Main window of the DC motor control application.
    
    This class implements the complete graphical interface with all controls
    needed for serial communication with the Arduino and control of a DC motor
    through an H-bridge. It includes panels for connection, motor control,
    and status monitoring.
    """
    
    def __init__(self):
        """Initialize the main window and set up all GUI components."""
        super().__init__()
        
        # Window properties
        self.setWindowTitle("DC Motor Control with H-Bridge")
        self.setMinimumSize(800, 600)
        
        # Set up the central widget and main layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        
        # Create panels (status panel first to initialize log_text)
        self.setup_status_panel()
        self.setup_connection_panel()
        self.setup_control_panel()
        
        # Log initial message
        self.log_message("Application started. Please connect to Arduino.")
        
        # Initialize connection state
        self.serial_port = None
        self.is_connected = False
        
        # Initialize motor control state variables
        self.motor_running = False
        self.current_speed = 0
        self.current_direction = "F"  # 'F' for forward, 'R' for reverse
        
        # Set up timer for serial reading
        self.serial_timer = QTimer(self)
        self.serial_timer.timeout.connect(self.read_serial)
        self.serial_timer.setInterval(100)  # Check for serial data every 100ms
        
        # Set up command timeout timer
        self.response_timer = QTimer(self)
        self.response_timer.timeout.connect(self.handle_response_timeout)
        self.response_timer.setSingleShot(True)
        
        # Command handling variables
        self.command_queue = []
        self.awaiting_response = False
        self.last_command = None
        
        # Update interface to show initial connection state
        self.update_connection_status()
    
    def setup_connection_panel(self):
        """Create and set up the connection control panel."""
        # Create a group box for connection settings
        connection_group = QGroupBox("Connection Settings")
        connection_layout = QHBoxLayout()
        
        # Port selection dropdown
        self.port_label = QLabel("COM Port:")
        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh_ports)
        
        # Connect/Disconnect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        
        # Connection status label
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        
        # Add widgets to the layout
        connection_layout.addWidget(self.port_label)
        connection_layout.addWidget(self.port_combo)
        connection_layout.addWidget(self.refresh_button)
        connection_layout.addWidget(self.connect_button)
        connection_layout.addWidget(self.status_label)
        connection_layout.addStretch()
        
        # Set the layout for the group box
        connection_group.setLayout(connection_layout)
        
        # Add the group box to the main layout
        self.main_layout.addWidget(connection_group)
        
        # Initially populate the ports dropdown
        self.refresh_ports()
    
    def setup_control_panel(self):
        """Create and set up the motor control panel."""
        # Create a group box for motor controls
        control_group = QGroupBox("Motor Control")
        control_layout = QGridLayout()
        
        # Direction control
        direction_label = QLabel("Direction:")
        self.forward_button = QPushButton("Forward")
        self.forward_button.clicked.connect(lambda: self.set_direction("F"))
        self.reverse_button = QPushButton("Reverse")
        self.reverse_button.clicked.connect(lambda: self.set_direction("R"))
        
        # Speed control
        speed_label = QLabel("Speed:")
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(255)
        self.speed_slider.setValue(0)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(25)
        self.speed_slider.valueChanged.connect(self.update_speed)
        
        self.speed_value_label = QLabel("0")
        
        # Start/Stop and Emergency Stop buttons
        self.start_stop_button = QPushButton("Start")
        self.start_stop_button.clicked.connect(self.toggle_motor)
        
        self.emergency_stop_button = QPushButton("EMERGENCY STOP")
        self.emergency_stop_button.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.emergency_stop_button.clicked.connect(self.emergency_stop)
        
        self.reset_button = QPushButton("Reset E-Stop")
        self.reset_button.clicked.connect(self.reset_emergency_stop)
        
        # Add widgets to the layout
        control_layout.addWidget(direction_label, 0, 0)
        control_layout.addWidget(self.forward_button, 0, 1)
        control_layout.addWidget(self.reverse_button, 0, 2)
        
        control_layout.addWidget(speed_label, 1, 0)
        control_layout.addWidget(self.speed_slider, 1, 1, 1, 2)
        control_layout.addWidget(self.speed_value_label, 1, 3)
        
        control_layout.addWidget(self.start_stop_button, 2, 0, 1, 2)
        control_layout.addWidget(self.emergency_stop_button, 3, 0, 1, 3)
        control_layout.addWidget(self.reset_button, 4, 0, 1, 2)
        
        # Set the layout for the group box
        control_group.setLayout(control_layout)
        
        # Add the group box to the main layout
        self.main_layout.addWidget(control_group)
    
    def setup_status_panel(self):
        """Create and set up the status and logging panel."""
        # Create a group box for status information
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        
        # Status indicators layout
        indicators_layout = QGridLayout()
        
        # Direction status
        direction_status_label = QLabel("Current Direction:")
        self.direction_value_label = QLabel("Not Set")
        
        # Speed status
        speed_status_label = QLabel("Current Speed:")
        self.speed_status_value = QLabel("0")
        
        # Operation status
        operation_status_label = QLabel("Operation Status:")
        self.operation_status_value = QLabel("Idle")
        
        # Add indicators to layout
        indicators_layout.addWidget(direction_status_label, 0, 0)
        indicators_layout.addWidget(self.direction_value_label, 0, 1)
        
        indicators_layout.addWidget(speed_status_label, 1, 0)
        indicators_layout.addWidget(self.speed_status_value, 1, 1)
        
        indicators_layout.addWidget(operation_status_label, 2, 0)
        indicators_layout.addWidget(self.operation_status_value, 2, 1)
        
        # Add indicators to main status layout
        status_layout.addLayout(indicators_layout)
        
        # Add separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        status_layout.addWidget(separator)
        
        # Log area
        log_label = QLabel("Log Messages:")
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        
        status_layout.addWidget(log_label)
        status_layout.addWidget(self.log_text)
        
        # Set the layout for the group box
        status_group.setLayout(status_layout)
        
        # Add the group box to the main layout
        self.main_layout.addWidget(status_group)
    
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
                # Open serial connection
                self.serial_port = serial.Serial(port, 9600, timeout=1)
                # Wait for Arduino reset
                time.sleep(2)
                self.is_connected = True
                self.log_message(f"Connected to {port} at 9600 baud.")
                
                # Start the serial reading timer
                self.serial_timer.start()
                
            except serial.SerialException as e:
                self.log_message(f"Error connecting to {port}: {str(e)}")
                return
        else:
            # Disconnect
            self.serial_timer.stop()
            self.response_timer.stop()
            
            # If motor is running, stop it before disconnecting
            if self.motor_running:
                self.send_command("STOP", wait_response=False)
                self.motor_running = False
            
            # Close the serial port
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            
            self.is_connected = False
            self.serial_port = None
            self.log_message("Disconnected from serial port.")
        
        # Update UI to reflect connection state
        self.update_connection_status()
    
    def update_connection_status(self):
        """Update UI elements based on connection status."""
        if self.is_connected:
            self.connect_button.setText("Disconnect")
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            
            # Enable control elements
            self.forward_button.setEnabled(True)
            self.reverse_button.setEnabled(True)
            self.speed_slider.setEnabled(True)
            self.start_stop_button.setEnabled(True)
            self.emergency_stop_button.setEnabled(True)
            self.reset_button.setEnabled(True)
        else:
            self.connect_button.setText("Connect")
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            
            # Disable control elements
            self.forward_button.setEnabled(False)
            self.reverse_button.setEnabled(False)
            self.speed_slider.setEnabled(False)
            self.start_stop_button.setEnabled(False)
            self.emergency_stop_button.setEnabled(False)
            self.reset_button.setEnabled(False)
            
            # Reset status indicators
            self.direction_value_label.setText("Not Set")
            self.speed_status_value.setText("0")
            self.operation_status_value.setText("Idle")
    
    def set_direction(self, direction):
        """Set the motor direction."""
        if not self.is_connected:
            return
        
        self.current_direction = direction
        dir_text = "Forward" if direction == "F" else "Reverse"
        self.direction_value_label.setText(dir_text)
        self.log_message(f"Direction set to {dir_text}.")
        
        # Send direction command to Arduino
        self.send_command(f"SET_DIR {direction}")
    
    def update_speed(self):
        """Update the speed display when the slider is moved."""
        speed = self.speed_slider.value()
        self.current_speed = speed
        self.speed_value_label.setText(str(speed))
        self.speed_status_value.setText(str(speed))
        
        # Send the speed command to Arduino
        self.send_command(f"SET_SPEED {speed}")
    
    def toggle_motor(self):
        """Start or stop the motor."""
        if not self.is_connected:
            return
        
        if self.start_stop_button.text() == "Start":
            self.start_stop_button.setText("Stop")
            self.operation_status_value.setText("Running")
            self.log_message("Motor started.")
            self.motor_running = True
            
            # Send start command
            self.send_command("START")
        else:
            self.start_stop_button.setText("Start")
            self.operation_status_value.setText("Stopped")
            self.log_message("Motor stopped.")
            self.motor_running = False
            
            # Send stop command
            self.send_command("STOP")
    
    def emergency_stop(self):
        """Trigger an emergency stop."""
        if not self.is_connected:
            return
        
        self.start_stop_button.setText("Start")
        self.operation_status_value.setText("EMERGENCY STOPPED")
        self.operation_status_value.setStyleSheet("color: red; font-weight: bold;")
        self.log_message("EMERGENCY STOP activated!", "red")
        self.motor_running = False
        
        # Send emergency stop command with high priority
        self.send_command("ESTOP", wait_response=False)
        
        # Clear any pending commands
        self.command_queue = []
        self.awaiting_response = False
    
    def reset_emergency_stop(self):
        """Reset the emergency stop state."""
        if not self.is_connected:
            return
            
        self.operation_status_value.setText("Idle")
        self.operation_status_value.setStyleSheet("")
        self.log_message("Emergency stop reset.")
        
        # Send reset command to Arduino
        self.send_command("RESET")
    
    def read_serial(self):
        """Read data from the serial port and process it."""
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            return
            
        try:
            # Check if data is available
            if self.serial_port.in_waiting > 0:
                # Read a line until newline
                data = self.serial_port.readline().decode('utf-8').strip()
                
                if data:
                    self.log_message(f"Received: {data}")
                    self.process_response(data)
        except serial.SerialException as e:
            self.log_message(f"Error reading from serial port: {str(e)}", "red")
            self.handle_connection_error()
    
    def send_command(self, command, wait_response=True, timeout=2000):
        """
        Send a command to the Arduino.
        
        Args:
            command (str): Command to send
            wait_response (bool): Whether to wait for a response
            timeout (int): Timeout in milliseconds for response
        
        Returns:
            bool: Success of sending command
        """
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            self.log_message("Cannot send command - not connected", "red")
            return False
        
        # If already waiting for a response and this command needs a response,
        # queue it to be sent later
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
    
    def process_response(self, response):
        """Process a response received from the Arduino."""
        # Cancel the response timeout timer
        self.response_timer.stop()
        
        # Mark that we're no longer waiting for a response
        self.awaiting_response = False
        
        # Process specific responses
        if response.startswith("OK:"):
            # Command successful
            command = response[3:]  # Extract the command part
            self.log_message(f"Command completed successfully: {command}", "green")
            
            # Update interface based on specific successful commands
            if response.startswith("OK:SPEED:"):
                speed_val = response[9:]
                self.speed_status_value.setText(speed_val)
                
            elif response.startswith("OK:DIR:"):
                dir_val = response[7:]
                self.direction_value_label.setText(dir_val)
                
            elif response == "OK:MOTOR_STARTED":
                self.operation_status_value.setText("Running")
                self.motor_running = True
                
            elif response == "OK:MOTOR_STOPPED":
                self.operation_status_value.setText("Stopped")
                self.motor_running = False
                
            elif response == "OK:EMERGENCY_STOP_ACTIVATED":
                self.operation_status_value.setText("EMERGENCY STOPPED")
                self.operation_status_value.setStyleSheet("color: red; font-weight: bold;")
                self.motor_running = False
                
            elif response == "OK:EMERGENCY_STOP_RESET":
                self.operation_status_value.setText("Idle")
                self.operation_status_value.setStyleSheet("")
                
        elif response.startswith("ERROR:"):
            # Command error
            error_msg = response[6:]  # Extract error message
            self.log_message(f"Command error: {error_msg}", "red")
            
        elif response.startswith("ALERT:"):
            # Alert message
            alert_msg = response[6:]  # Extract alert message
            self.log_message(f"Alert: {alert_msg}", "orange")
        
        # Process any queued commands
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
            # Try to reopen the connection
            self.serial_port = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            self.is_connected = True
            self.log_message(f"Reconnected to {port} at 9600 baud.", "green")
            
            # Restart the serial reading timer
            self.serial_timer.start()
            
            # Resend current configuration
            self.send_command(f"SET_SPEED {self.current_speed}")
            self.send_command(f"SET_DIR {self.current_direction}")
            
            # Update the UI
            self.update_connection_status()
            
        except serial.SerialException as e:
            self.log_message(f"Failed to reconnect: {str(e)}", "red")
            # Try again after a delay
            QTimer.singleShot(5000, self.attempt_reconnect)
    
    def log_message(self, message, color="black"):
        """
        Add a message to the log with optional color.
        
        Args:
            message (str): The message to log
            color (str): Color name for the message
        """
        self.log_text.append(f"<span style='color:{color};'>{message}</span>")
    
    def closeEvent(self, event):
        """
        Handle application close event.
        
        Ensures proper cleanup when the application is closed.
        """
        # Stop timers
        self.serial_timer.stop()
        self.response_timer.stop()
        
        # Stop the motor and close the serial port
        if self.is_connected:
            try:
                # Try to stop the motor before closing
                if self.motor_running:
                    self.serial_port.write("STOP\n".encode())
                    time.sleep(0.1)  # Small delay to allow command to be processed
            except:
                pass  # Ignore errors during close
                
            # Close the port
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
        
        # Accept the close event
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DCMotorControlGUI()
    window.show()
    sys.exit(app.exec_())

