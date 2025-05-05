#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motor Control GUI - Practice 3
------------------------------
A PyQt-based interface for controlling a DC motor with encoder feedback using PID control.
Features:
- Real-time plotting of position, speed, and control signals
- PID parameter adjustment
- Mode selection (Position/Speed control)
- Manual motor control
- Data logging and export
- Configuration save/load
- Emergency stop
"""

import sys
import os
import time
import csv
import json
import serial
import serial.tools.list_ports
import numpy as np
from datetime import datetime
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                           QLabel, QComboBox, QPushButton, QSlider, QDoubleSpinBox, 
                           QSpinBox, QCheckBox, QGroupBox, QFileDialog, QTabWidget, 
                           QMessageBox, QGridLayout, QSplitter, QFrame, QRadioButton, 
                           QButtonGroup, QStatusBar, QAction, QTextEdit)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread, QSettings

class SerialThread(QThread):
    """Thread for handling serial communication without blocking the GUI"""
    data_received = pyqtSignal(str)
    connection_error = pyqtSignal(str)
    
    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.serial = None
    
    def run(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            
            while self.running:
                if self.serial.in_waiting > 0:
                    data = self.serial.readline().decode('utf-8').strip()
                    if data:
                        self.data_received.emit(data)
                time.sleep(0.01)  # Small delay to prevent CPU overuse
                
        except serial.SerialException as e:
            self.connection_error.emit(f"Serial connection error: {str(e)}")
            self.running = False
        except Exception as e:
            self.connection_error.emit(f"Error: {str(e)}")
            self.running = False
        finally:
            if self.serial and self.serial.is_open:
                self.serial.close()
    
    def send_command(self, command):
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(f"{command}\n".encode('utf-8'))
                return True
            except Exception as e:
                self.connection_error.emit(f"Failed to send command: {str(e)}")
                return False
        return False
    
    def stop(self):
        self.running = False
        self.wait()

class PlotCanvas(FigureCanvas):
    """Matplotlib canvas for real-time plotting"""
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.fig.tight_layout()
        
        # Position plot
        self.position_ax = self.fig.add_subplot(311)
        self.position_ax.set_title('Position')
        self.position_ax.set_ylabel('Position (counts)')
        self.position_ax.grid(True)
        
        # Speed plot
        self.speed_ax = self.fig.add_subplot(312, sharex=self.position_ax)
        self.speed_ax.set_title('Speed')
        self.speed_ax.set_ylabel('Speed (counts/s)')
        self.speed_ax.grid(True)
        
        # Control signal plot
        self.control_ax = self.fig.add_subplot(313, sharex=self.position_ax)
        self.control_ax.set_title('Control Signal')
        self.control_ax.set_xlabel('Time (s)')
        self.control_ax.set_ylabel('PWM (-255 to 255)')
        self.control_ax.grid(True)
        
        # Initialize data
        self.time_data = np.array([])
        self.position_data = np.array([])
        self.speed_data = np.array([])
        self.setpoint_data = np.array([])
        self.control_data = np.array([])
        
        # Initialize plot lines
        self.position_line, = self.position_ax.plot([], [], 'b-', label='Position')
        self.position_setpoint, = self.position_ax.plot([], [], 'r--', label='Setpoint')
        self.speed_line, = self.speed_ax.plot([], [], 'g-', label='Speed')
        self.speed_setpoint, = self.speed_ax.plot([], [], 'r--', label='Setpoint')
        self.control_line, = self.control_ax.plot([], [], 'k-', label='Control')
        
        # Add legends
        self.position_ax.legend(loc='upper right')
        self.speed_ax.legend(loc='upper right')
        
        # Set window size (in seconds)
        self.window_size = 10.0
        
        super(PlotCanvas, self).__init__(self.fig)
        self.setMinimumSize(400, 300)
        
    def update_plot(self, time_val, position, speed, setpoint, control_signal, control_mode):
        # Add new data points
        self.time_data = np.append(self.time_data, time_val)
        self.position_data = np.append(self.position_data, position)
        self.speed_data = np.append(self.speed_data, speed)
        self.setpoint_data = np.append(self.setpoint_data, setpoint)
        self.control_data = np.append(self.control_data, control_signal)
        
        # Remove old data points beyond the window size
        if len(self.time_data) > 0 and (self.time_data[-1] - self.time_data[0]) > self.window_size:
            # Find index of oldest data to keep
            idx = np.searchsorted(self.time_data, self.time_data[-1] - self.window_size)
            
            # Trim data arrays
            self.time_data = self.time_data[idx:]
            self.position_data = self.position_data[idx:]
            self.speed_data = self.speed_data[idx:]
            self.setpoint_data = self.setpoint_data[idx:]
            self.control_data = self.control_data[idx:]
        
        # Update plot data
        self.position_line.set_data(self.time_data, self.position_data)
        self.speed_line.set_data(self.time_data, self.speed_data)
        self.control_line.set_data(self.time_data, self.control_data)
        
        # Update setpoint line depending on control mode
        if control_mode == 'P':  # Position mode
            self.position_setpoint.set_data(self.time_data, self.setpoint_data)
            self.speed_setpoint.set_data([], [])
        else:  # Speed mode
            self.position_setpoint.set_data([], [])
            self.speed_setpoint.set_data(self.time_data, self.setpoint_data)
        
        # Auto-scale axes
        if len(self.time_data) > 0:
            self.position_ax.set_xlim(self.time_data[0], self.time_data[-1])
            self.position_ax.set_ylim(min(min(self.position_data), min(self.setpoint_data)) - 10, 
                                     max(max(self.position_data), max(self.setpoint_data)) + 10)
            self.speed_ax.set_ylim(min(min(self.speed_data), min(self.setpoint_data)) - 10, 
                                  max(max(self.speed_data), max(self.setpoint_data)) + 10)
            self.control_ax.set_ylim(min(self.control_data) - 10, max(self.control_data) + 10)
        
        # Redraw
        self.fig.canvas.draw()
    
    def set_window_size(self, seconds):
        """Set the time window size in seconds"""
        self.window_size = seconds
        
    def clear_data(self):
        """Clear all plot data"""
        self.time_data = np.array([])
        self.position_data = np.array([])
        self.speed_data = np.array([])
        self.setpoint_data = np.array([])
        self.control_data = np.array([])
        
        # Clear plot lines
        self.position_line.set_data([], [])
        self.position_setpoint.set_data([], [])
        self.speed_line.set_data([], [])
        self.speed_setpoint.set_data([], [])
        self.control_line.set_data([], [])
        
        # Redraw
        self.fig.canvas.draw()

class MotorControlGUI(QMainWindow):
    """Main window for motor control interface"""
    def __init__(self):
        super().__init__()
        
        # Set window title and initial size
        self.setWindowTitle("Motor Control with PID - Practice 3")
        self.resize(1000, 800)
        
        # Initialize data storage
        self.log_data = []
        self.is_logging = False
        self.start_time = None
        self.control_mode = 'P'  # Default to position mode
        
        # Create main widget and layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        
        # Create splitter to allow resizing sections
        self.splitter = QSplitter(Qt.Vertical)
        self.main_layout.addWidget(self.splitter)
        
        # Top section - Control panel
        self.control_panel = QWidget()
        self.control_layout = QVBoxLayout(self.control_panel)
        
        # Add control panel sections
        self.create_connection_section()
        self.create_control_mode_section()
        self.create_pid_control_section()
        self.create_motor_control_section()
        self.create_logging_section()
        
        # Bottom section - Plot area
        self.plot_canvas = PlotCanvas(width=5, height=5, dpi=100)
        
        # Add widgets to splitter
        self.splitter.addWidget(self.control_panel)
        self.splitter.addWidget(self.plot_canvas)
        
        # Set initial splitter sizes (control panel: 40%, plot: 60%)
        self.splitter.setSizes([400, 600])
        
        # Create status bar
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("Disconnected")
        
        # Initialize serial connection
        self.serial_thread = None
        
        # Timer for updating UI elements
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)  # Update every 100ms
        
        # Create menu bar
        self.create_menu_bar()
        
        # Load settings from last session
        self.load_settings()
        
    def create_connection_section(self):
        """Create the serial connection control group"""
        connection_group = QGroupBox("Serial Connection")
        connection_layout = QGridLayout()
        
        # Port selection
        self.port_label = QLabel("Port:")
        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh_ports)
        
        # Baudrate selection
        self.baud_label = QLabel("Baudrate:")
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        
        # Connect/Disconnect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        
        # Add widgets to layout
        connection_layout.addWidget(self.port_label, 0, 0)
        connection_layout.addWidget(self.port_combo, 0, 1, 1, 2)
        connection_layout.addWidget(self.refresh_button, 0, 3)
        connection_layout.addWidget(self.baud_label, 1, 0)
        connection_layout.addWidget(self.baud_combo, 1, 1, 1, 2)
        connection_layout.addWidget(self.connect_button, 1, 3)
        
        connection_group.setLayout(connection_layout)
        self.control_layout.addWidget(connection_group)
        
        # Initially populate the port list
        self.refresh_ports()
        
    def create_control_mode_section(self):
        """Create control mode selection group"""
        mode_group = QGroupBox("Control Mode")
        mode_layout = QHBoxLayout()
        
        # Radio buttons for control mode
        self.position_radio = QRadioButton("Position Control")
        self.speed_radio = QRadioButton("Speed Control")
        self.position_radio.setChecked(True)
        
        # Button group for mutual exclusion
        self.mode_group = QButtonGroup()
        self.mode_group.addButton(self.position_radio, 0)
        self.mode_group.addButton(self.speed_radio, 1)
        self.mode_group.buttonClicked.connect(self.change_control_mode)
        
        # Add widgets to layout
        mode_layout.addWidget(self.position_radio)
        mode_layout.addWidget(self.speed_radio)
        mode_group.setLayout(mode_layout)
        self.control_layout.addWidget(mode_group)
        
    def create_pid_control_section(self):
        """Create PID parameter control group"""
        pid_group = QGroupBox("PID Parameters")
        pid_layout = QGridLayout()
        
        # Kp control
        self.kp_label = QLabel("Kp:")
        self.kp_spin = QDoubleSpinBox()
        self.kp_spin.setRange(0, 1000)
        self.kp_spin.setSingleStep(0.1)
        self.kp_spin.setValue(1.0)
        
        # Ki control
        self.ki_label = QLabel("Ki:")
        self.ki_spin = QDoubleSpinBox()
        self.ki_spin.setRange(0, 1000)
        self.ki_spin.setSingleStep(0.1)
        self.ki_spin.setValue(0.1)
        
        # Kd control
        self.kd_label = QLabel("Kd:")
        self.kd_spin = QDoubleSpinBox()
        self.kd_spin.setRange(0, 1000)
        self.kd_spin.setSingleStep(0.01)
        self.kd_spin.setValue(0.01)
        
        # Apply PID parameters button
        self.apply_pid_button = QPushButton("Apply PID Parameters")
        self.apply_pid_button.clicked.connect(self.apply_pid_parameters)
        
        # Add widgets to layout
        pid_layout.addWidget(self.kp_label, 0, 0)
        pid_layout.addWidget(self.kp_spin, 0, 1)
        pid_layout.addWidget(self.ki_label, 1, 0)
        pid_layout.addWidget(self.ki_spin, 1, 1)
        pid_layout.addWidget(self.kd_label, 2, 0)
        pid_layout.addWidget(self.kd_spin, 2, 1)
        pid_layout.addWidget(self.apply_pid_button, 3, 0, 1, 2)
        
        pid_group.setLayout(pid_layout)
        self.control_layout.addWidget(pid_group)
        
    def create_motor_control_section(self):
        """Create motor control section"""
        motor_group = QGroupBox("Motor Control")
        motor_layout = QGridLayout()
        
        # Setpoint control
        self.setpoint_label = QLabel("Setpoint:")
        self.setpoint_spin = QSpinBox()
        self.setpoint_spin.setRange(-10000, 10000)
        self.setpoint_spin.setValue(0)
        
        # Apply setpoint button
        self.apply_setpoint_button = QPushButton("Apply Setpoint")
        self.apply_setpoint_button.clicked.connect(self.apply_setpoint)
        
        # Manual motor control
        self.manual_control_label = QLabel("Manual Control:")
        self.manual_slider = QSlider(Qt.Horizontal)
        self.manual_slider.setRange(-255, 255)
        self.manual_slider.setValue(0)
        self.manual_slider.setTickPosition(QSlider.TicksBelow)
        self.manual_slider.setTickInterval(64)
        self.manual_slider.valueChanged.connect(self.update_manual_value)
        
        # Manual value display and apply
        self.manual_value_label = QLabel("0")
        self.apply_manual_button = QPushButton("Apply Manual")
        self.apply_manual_button.clicked.connect(self.apply_manual_control)
        
        # Zero button
        self.zero_button = QPushButton("Zero Motor")
        self.zero_button.clicked.connect(self.zero_motor)
        
        # Emergency stop button (large and red)
        self.emergency_stop_button = QPushButton("EMERGENCY STOP")
        self.emergency_stop_button.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 10px;")
        self.emergency_stop_button.clicked.connect(self.emergency_stop)
        
        # Reset encoder button
        self.reset_encoder_button = QPushButton("Reset Encoder")
        self.reset_encoder_button.clicked.connect(self.reset_encoder)
        
        # Add widgets to layout
        motor_layout.addWidget(self.setpoint_label, 0, 0)
        motor_layout.addWidget(self.setpoint_spin, 0, 1)
        motor_layout.addWidget(self.apply_setpoint_button, 0, 2)
        
        motor_layout.addWidget(self.manual_control_label, 1, 0)
        motor_layout.addWidget(self.manual_slider, 1, 1)
        motor_layout.addWidget(self.manual_value_label, 1, 2)
        
        motor_layout.addWidget(self.apply_manual_button, 2, 0)
        motor_layout.addWidget(self.zero_button, 2, 1)
        motor_layout.addWidget(self.reset_encoder_button, 2, 2)
        
        motor_layout.addWidget(self.emergency_stop_button, 3, 0, 1, 3)
        
        motor_group.setLayout(motor_layout)
        self.control_layout.addWidget(motor_group)
        
    def create_logging_section(self):
        """Create data logging control section"""
        logging_group = QGroupBox("Data Logging")
        logging_layout = QGridLayout()
        
        # Start/Stop logging button
        self.logging_button = QPushButton("Start Logging")
        self.logging_button.clicked.connect(self.toggle_logging)
        
        # Export data button
        self.export_button = QPushButton("Export Data")
        self.export_button.clicked.connect(self.export_data)
        self.export_button.setEnabled(False)  # Disabled until data is logged
        
        # Clear data button
        self.clear_button = QPushButton("Clear Data")
        self.clear_button.clicked.connect(self.clear_data)
        
        # Status label
        self.logging_status = QLabel("Not logging")
        
        # Plot window size control
        self.window_label = QLabel("Plot Window (s):")
        self.window_spin = QSpinBox()
        self.window_spin.setRange(5, 60)
        self.window_spin.setValue(10)
        self.window_spin.valueChanged.connect(self.change_plot_window)
        
        # Add widgets to layout
        logging_layout.addWidget(self.logging_button, 0, 0)
        logging_layout.addWidget(self.export_button, 0, 1)
        logging_layout.addWidget(self.clear_button, 0, 2)
        logging_layout.addWidget(self.logging_status, 1, 0, 1, 2)
        logging_layout.addWidget(self.window_label, 2, 0)
        logging_layout.addWidget(self.window_spin, 2, 1, 1, 2)
        
        logging_group.setLayout(logging_layout)
        self.control_layout.addWidget(logging_group)
    
    def create_menu_bar(self):
        """Create application menu bar"""
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu('File')
        
        # Save configuration action
        save_config_action = QAction('Save Configuration', self)
        save_config_action.setShortcut('Ctrl+S')
        save_config_action.triggered.connect(self.save_configuration)
        file_menu.addAction(save_config_action)
        
        # Load configuration action
        load_config_action = QAction('Load Configuration', self)
        load_config_action.setShortcut('Ctrl+L')
        load_config_action.triggered.connect(self.load_configuration)
        file_menu.addAction(load_config_action)
        
        file_menu.addSeparator()
        
        # Export data action
        export_data_action = QAction('Export Data...', self)
        export_data_action.setShortcut('Ctrl+E')
        export_data_action.triggered.connect(self.export_data)
        file_menu.addAction(export_data_action)
        
        file_menu.addSeparator()
        
        # Exit action
        exit_action = QAction('Exit', self)
        exit_action.setShortcut('Ctrl+Q')
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # Help menu
        help_menu = menubar.addMenu('Help')
        
        # About action
        about_action = QAction('About', self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
    
    # Serial Communication Methods
    def refresh_ports(self):
        """Refresh the list of available serial ports"""
        self.port_combo.clear()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        if ports:
            self.port_combo.addItems(ports)
        else:
            self.statusBar.showMessage("No serial ports found")
    
    def toggle_connection(self):
        """Connect or disconnect from the selected serial port"""
        if self.serial_thread is None or not self.serial_thread.running:
            # Connect
            port = self.port_combo.currentText()
            if not port:
                QMessageBox.warning(self, "Connection Error", "No serial port selected")
                return
                
            baudrate = int(self.baud_combo.currentText())
            
            try:
                self.serial_thread = SerialThread(port, baudrate)
                self.serial_thread.data_received.connect(self.process_serial_data)
                self.serial_thread.connection_error.connect(self.handle_connection_error)
                self.serial_thread.start()
                
                self.connect_button.setText("Disconnect")
                self.statusBar.showMessage(f"Connected to {port} at {baudrate} baud")
                
                # Enable control widgets
                self.setEnabledControls(True)
                
                # Start time reference
                self.start_time = time.time()
                
            except Exception as e:
                QMessageBox.critical(self, "Connection Error", f"Failed to connect: {str(e)}")
        else:
            # Disconnect
            self.disconnect_serial()
    
    def disconnect_serial(self):
        """Disconnect from the serial port"""
        if self.serial_thread and self.serial_thread.running:
            # Send stop command to motor
            self.serial_thread.send_command("MOTOR,0")
            
            # Stop the thread
            self.serial_thread.stop()
            self.serial_thread = None
            
            # Update UI
            self.connect_button.setText("Connect")
            self.statusBar.showMessage("Disconnected")
            
            # Disable control widgets
            self.setEnabledControls(False)
            
            # Stop logging if active
            if self.is_logging:
                self.toggle_logging()
    
    def handle_connection_error(self, error_message):
        """Handle serial connection errors"""
        QMessageBox.warning(self, "Connection Error", error_message)
        self.disconnect_serial()
    
    def process_serial_data(self, data):
        """Process data received from Arduino"""
        try:
            # Check if it's a data packet
            if data.startswith("DATA"):
                parts = data.split(',')
                if len(parts) >= 10:  # Ensure we have enough data elements
                    # Extract values
                    timestamp = int(parts[1]) / 1000.0  # Convert ms to seconds
                    position = int(parts[2])
                    speed = int(parts[3])
                    setpoint = float(parts[4])
                    error = float(parts[5])
                    control = float(parts[6])
                    kp = float(parts[7])
                    ki = float(parts[8])
                    kd = float(parts[9])
                    mode = parts[10] if len(parts) > 10 else 'P'
                    
                    # Calculate relative time (seconds since start)
                    rel_time = time.time() - self.start_time
                    
                    # Update plot
                    self.plot_canvas.update_plot(rel_time, position, speed, setpoint, control, mode)
                    
                    # Log data if enabled
                    if self.is_logging:
                        self.log_data.append({
                            

