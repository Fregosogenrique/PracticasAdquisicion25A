#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Arduino Stepper Motor Control GUI
==================================
A PyQt5-based graphical user interface for controlling an Arduino stepper motor.
This application provides controls for motor direction, speed, and steps per revolution.
"""

import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGridLayout, QLabel, QPushButton, 
                            QComboBox, QSlider, QSpinBox, QGroupBox, 
                            QTextEdit, QFrame, QSplitter)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont, QIcon, QColor
import time
import threading

class StepperMotorGUI(QMainWindow):
    """Main application window for the Arduino Stepper Motor Control GUI."""
    
    def __init__(self):
        super().__init__()
        
        # Initialize UI properties
        self.setWindowTitle("Arduino Stepper Motor Control")
        self.setMinimumSize(800, 600)
        
        # Set up the central widget and main layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        
        # Create the three panels (status panel first so log_text is available)
        self.setup_status_panel()
        self.setup_connection_panel()
        self.setup_control_panel()
        
        # Log initial message after status panel is created
        self.log_message("Application started. Please connect to Arduino.")
        
        # Initialize the serial connection (not connected yet)
        self.serial_port = None
        self.is_connected = False
        
        # Initialize motor control variables
        self.current_direction = None
        self.current_speed = 60  # Default from Arduino code
        self.steps_per_revolution = 48  # Default from Arduino code
        self.is_running = False
        
        # Set up serial reading timer
        self.serial_timer = QTimer(self)
        self.serial_timer.timeout.connect(self.read_serial)
        self.serial_timer.setInterval(100)  # Check for serial data every 100ms
        
        # Set up command response timeout timer
        self.response_timer = QTimer(self)
        self.response_timer.timeout.connect(self.handle_response_timeout)
        self.response_timer.setSingleShot(True)
        
        # Command queue for serial operations
        self.command_queue = []
        self.awaiting_response = False
        self.last_command = None
        
        # Update the UI to reflect the connection status
        self.update_connection_status()
        
    def setup_connection_panel(self):
        """Create and configure the connection panel."""
        # Create a group box for the connection panel
        connection_group = QGroupBox("Connection Settings")
        connection_layout = QHBoxLayout()
        
        # COM Port selection dropdown
        self.port_label = QLabel("COM Port:")
        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh_ports)
        
        # Connect/Disconnect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        
        # Connection status indicator
        self.status_label = QLabel("Disconnected")
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
        """Create and configure the motor control panel."""
        # Create a group box for the control panel
        control_group = QGroupBox("Motor Control")
        control_layout = QGridLayout()
        
        # Direction controls
        direction_label = QLabel("Direction:")
        self.cw_button = QPushButton("Clockwise")
        self.cw_button.clicked.connect(lambda: self.set_direction("clockwise"))
        self.ccw_button = QPushButton("Counterclockwise")
        self.ccw_button.clicked.connect(lambda: self.set_direction("counterclockwise"))
        
        # Speed control
        speed_label = QLabel("Speed:")
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(100)
        self.speed_slider.setValue(60)  # Default speed of 60 RPM
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(10)
        self.speed_slider.valueChanged.connect(self.update_speed)
        
        self.speed_value_label = QLabel("60 RPM")
        
        # Steps per revolution
        steps_label = QLabel("Steps per Revolution:")
        self.steps_spinbox = QSpinBox()
        self.steps_spinbox.setMinimum(1)
        self.steps_spinbox.setMaximum(1000)
        self.steps_spinbox.setValue(48)  # Default value from Arduino code
        self.steps_spinbox.valueChanged.connect(self.update_steps)
        
        # Start/Stop and Emergency Stop buttons
        self.start_stop_button = QPushButton("Start")
        self.start_stop_button.clicked.connect(self.toggle_motor)
        
        self.emergency_stop_button = QPushButton("EMERGENCY STOP")
        self.emergency_stop_button.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.emergency_stop_button.clicked.connect(self.emergency_stop)
        
        # Add widgets to the control layout
        control_layout.addWidget(direction_label, 0, 0)
        control_layout.addWidget(self.cw_button, 0, 1)
        control_layout.addWidget(self.ccw_button, 0, 2)
        
        control_layout.addWidget(speed_label, 1, 0)
        control_layout.addWidget(self.speed_slider, 1, 1, 1, 2)
        control_layout.addWidget(self.speed_value_label, 1, 3)
        
        control_layout.addWidget(steps_label, 2, 0)
        control_layout.addWidget(self.steps_spinbox, 2, 1)
        
        control_layout.addWidget(self.start_stop_button, 3, 0, 1, 2)
        control_layout.addWidget(self.emergency_stop_button, 3, 2, 1, 2)
        
        # Set the layout for the control group
        control_group.setLayout(control_layout)
        
        # Add the control group to the main layout
        self.main_layout.addWidget(control_group)
        
    def setup_status_panel(self):
        """Create and configure the status panel."""
        # Create a group box for the status panel
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        
        # Create a grid for status indicators
        indicators_layout = QGridLayout()
        
        # Current direction indicator
        direction_status_label = QLabel("Current Direction:")
        self.direction_value_label = QLabel("Not Set")
        
        # Current speed indicator
        speed_status_label = QLabel("Current Speed:")
        self.speed_status_value = QLabel("0 RPM")
        
        # Operation status
        operation_status_label = QLabel("Operation Status:")
        self.operation_status_value = QLabel("Idle")
        
        # Add indicators to the layout
        indicators_layout.addWidget(direction_status_label, 0, 0)
        indicators_layout.addWidget(self.direction_value_label, 0, 1)
        
        indicators_layout.addWidget(speed_status_label, 1, 0)
        indicators_layout.addWidget(self.speed_status_value, 1, 1)
        
        indicators_layout.addWidget(operation_status_label, 2, 0)
        indicators_layout.addWidget(self.operation_status_value, 2, 1)
        
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
                self.send_command(f"STEPS:{self.steps_per_revolution}")
                self.send_command(f"SPEED:{self.current_speed}")
            except serial.SerialException as e:
                self.log_message(f"Error connecting to {port}: {str(e)}")
                return
        else:
            # Disconnect from the serial port
            # Stop the serial reading timer first
            self.serial_timer.stop()
            self.response_timer.stop()
            
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
            self.steps_spinbox.setEnabled(True)
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
            self.steps_spinbox.setEnabled(False)
            self.start_stop_button.setEnabled(False)
            self.emergency_stop_button.setEnabled(False)
            
            # Reset status indicators
            self.direction_value_label.setText("Not Set")
            self.speed_status_value.setText("0 RPM")
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
            self.send_command("CW")
        else:
            self.send_command("CCW")
        
    def update_speed(self):
        """Update the speed display when the slider is moved."""
        speed = self.speed_slider.value()
        self.current_speed = speed
        self.speed_value_label.setText(f"{speed} RPM")
        self.speed_status_value.setText(f"{speed} RPM")
        
        # Send the speed command to Arduino
        self.send_command(f"SPEED:{speed}")
        
    def update_steps(self):
        """Update the steps per revolution."""
        steps = self.steps_spinbox.value()
        self.steps_per_revolution = steps
        self.log_message(f"Steps per revolution set to {steps}.")
        
        # Send steps per revolution command to Arduino
        self.send_command(f"STEPS:{steps}")
    
    def toggle_motor(self):
        """Start or stop the motor."""
        if not self.is_connected:
            return
            
        # Toggle button state
        if self.start_stop_button.text() == "Start":
            self.start_stop_button.setText("Stop")
            self.operation_status_value.setText("Running")
            self.log_message("Motor started.")
            self.is_running = True
            
            # Send start command with current direction
            if self.current_direction == "clockwise":
                self.send_command("START:CW")
            elif self.current_direction == "counterclockwise":
                self.send_command("START:CCW")
            else:
                # Default to clockwise if no direction set
                self.send_command("START:CW")
                self.current_direction = "clockwise"
                self.direction_value_label.setText("Clockwise")
        else:
            self.start_stop_button.setText("Start")
            self.operation_status_value.setText("Stopped")
            self.log_message("Motor stopped.")
            self.is_running = False
            
            # Send stop command to Arduino
            self.send_command("STOP")
    
    def emergency_stop(self):
        """Emergency stop the motor."""
        if not self.is_connected:
            return
            
        self.start_stop_button.setText("Start")
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
        
        # Process specific responses
        if response.startswith("OK:"):
            # Command was successful
            command = response[3:]  # Extract the command that was acknowledged
            self.log_message(f"Command '{command}' completed successfully", "green")
            
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
            self.send_command(f"STEPS:{self.steps_per_revolution}")
            self.send_command(f"SPEED:{self.current_speed}")
            
            # Update the UI
            self.update_connection_status()
            
        except serial.SerialException as e:
            self.log_message(f"Failed to reconnect: {str(e)}", "red")
            # Try again after a delay
            QTimer.singleShot(5000, self.attempt_reconnect)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = StepperMotorGUI()
    window.show()
    sys.exit(app.exec_())
