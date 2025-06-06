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

class MotorSpeedControlGUI(QMainWindow):
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
        
        # Crear primero el log_text para evitar errores
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)

        # Crear el grupo del log
        log_group = QGroupBox("Registro de Eventos")
        log_layout = QVBoxLayout()
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        self.main_layout.addWidget(log_group)
        
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
        
        # Now that all UI elements are created, refresh the ports list
        self.refresh_ports()
        
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
        
        # Initially populate the port list after creating all UI elements
        # Moved to the end of __init__ to ensure all widgets are created
        
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
            if hasattr(self, 'log_text'):
                self.log_message(f"Se encontraron {len(ports)} puertos seriales.")
        else:
            self.statusBar.showMessage("No serial ports found")
            if hasattr(self, 'log_text'):
                self.log_message("No se encontraron puertos seriales. Conecte el Arduino.")
    
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
                            'time': rel_time,
                            'timestamp': timestamp,
                            'position': position,
                            'speed': speed,
                            'setpoint': setpoint,
                            'error': error,
                            'control': control,
                            'kp': kp,
                            'ki': ki,
                            'kd': kd,
                            'mode': mode
                        })
                        self.export_button.setEnabled(True)
                    
                    # Update status bar
                    self.statusBar.showMessage(f"Position: {position} | Speed: {speed} | Error: {error:.2f}")
            
            # Check for other responses (e.g., confirmations)
            else:
                # Log any non-data messages to status bar
                self.statusBar.showMessage(data, 3000)  # Show for 3 seconds
                
        except Exception as e:
            self.statusBar.showMessage(f"Error processing data: {str(e)}", 3000)
    
    # Control Methods
    def apply_pid_parameters(self):
        """Send new PID parameters to Arduino"""
        if self.serial_thread and self.serial_thread.running:
            kp = self.kp_spin.value()
            ki = self.ki_spin.value()
            kd = self.kd_spin.value()
            
            # Send the command
            command = f"PID,{kp},{ki},{kd}"
            self.serial_thread.send_command(command)
            self.statusBar.showMessage(f"Applied PID parameters: Kp={kp}, Ki={ki}, Kd={kd}")
        else:
            QMessageBox.warning(self, "Not Connected", "Please connect to Arduino first")
    
    def change_control_mode(self, button):
        """Change control mode (Position/Speed)"""
        if self.serial_thread and self.serial_thread.running:
            if button == self.position_radio:
                mode = 'P'  # Position mode
                self.control_mode = 'P'
                self.serial_thread.send_command("SETMODE,P")
                self.statusBar.showMessage("Switched to Position Control mode")
            else:
                mode = 'S'  # Speed mode
                self.control_mode = 'S'
                self.serial_thread.send_command("SETMODE,S")
                self.statusBar.showMessage("Switched to Speed Control mode")
            
            # Clear the plot data on mode change
            self.plot_canvas.clear_data()
        else:
            QMessageBox.warning(self, "Not Connected", "Please connect to Arduino first")
            # Reset radio buttons to match current mode
            if self.control_mode == 'P':
                self.position_radio.setChecked(True)
            else:
                self.speed_radio.setChecked(True)
    
    def apply_setpoint(self):
        """Send new setpoint to Arduino"""
        if self.serial_thread and self.serial_thread.running:
            setpoint = self.setpoint_spin.value()
            command = f"SET,{setpoint}"
            self.serial_thread.send_command(command)
            
            mode_text = "position" if self.control_mode == 'P' else "speed"
            self.statusBar.showMessage(f"Set {mode_text} setpoint to {setpoint}")
        else:
            QMessageBox.warning(self, "Not Connected", "Please connect to Arduino first")
    
    def update_manual_value(self):
        """Update the manual control value label"""
        value = self.manual_slider.value()
        self.manual_value_label.setText(str(value))
    
    def apply_manual_control(self):
        """Send manual control value to Arduino"""
        if self.serial_thread and self.serial_thread.running:
            value = self.manual_slider.value()
            command = f"MOTOR,{value}"
            self.serial_thread.send_command(command)
            self.statusBar.showMessage(f"Manual motor control: {value}")
        else:
            QMessageBox.warning(self, "Not Connected", "Please connect to Arduino first")
    
    def zero_motor(self):
        """Stop the motor (set PWM to 0)"""
        if self.serial_thread and self.serial_thread.running:
            self.serial_thread.send_command("MOTOR,0")
            self.manual_slider.setValue(0)
            self.statusBar.showMessage("Motor stopped")
        else:
            QMessageBox.warning(self, "Not Connected", "Please connect to Arduino first")
    
    def reset_encoder(self):
        """Reset the encoder position to 0"""
        if self.serial_thread and self.serial_thread.running:
            self.serial_thread.send_command("RESET")
            self.statusBar.showMessage("Encoder position reset to 0")
        else:
            QMessageBox.warning(self, "Not Connected", "Please connect to Arduino first")
    
    def emergency_stop(self):
        """Emergency stop - immediately stop the motor and reset"""
        if self.serial_thread and self.serial_thread.running:
            # Stop the motor
            self.serial_thread.send_command("MOTOR,0")
            self.manual_slider.setValue(0)
            
            # Reset the setpoint
            self.setpoint_spin.setValue(0)
            self.serial_thread.send_command("SET,0")
            
            # Show message
            QMessageBox.warning(self, "Emergency Stop", "Motor stopped. Reset your system as needed.")
            self.statusBar.showMessage("EMERGENCY STOP activated")
        else:
            QMessageBox.warning(self, "Not Connected", "Not connected to motor controller")
    
    # Data Logging Methods
    def toggle_logging(self):
        """Start or stop data logging"""
        self.is_logging = not self.is_logging
        
        if self.is_logging:
            # Start logging
            self.logging_button.setText("Stop Logging")
            self.logging_status.setText("Logging data...")
            # Reset log data if already full
            if len(self.log_data) > 0:
                if QMessageBox.question(self, "Confirm", "Clear existing log data?", 
                                       QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
                    self.log_data = []
            self.statusBar.showMessage("Data logging started")
        else:
            # Stop logging
            self.logging_button.setText("Start Logging")
            self.logging_status.setText(f"Logging stopped. {len(self.log_data)} data points collected.")
            self.statusBar.showMessage("Data logging stopped")
            
            # Enable export if we have data
            if len(self.log_data) > 0:
                self.export_button.setEnabled(True)
            else:
                self.export_button.setEnabled(False)
    
    def export_data(self):
        """Export logged data to CSV file"""
        if len(self.log_data) == 0:
            QMessageBox.warning(self, "No Data", "No data to export")
            return
            
        # Get filename from dialog
        filename, _ = QFileDialog.getSaveFileName(self, "Export Data", "", "CSV Files (*.csv);;All Files (*)")
        if not filename:
            return
            
        try:
            with open(filename, 'w', newline='') as csvfile:
                # Define fields to export
                fieldnames = ['time', 'timestamp', 'position', 'speed', 'setpoint', 
                             'error', 'control', 'kp', 'ki', 'kd', 'mode']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                # Write header
                writer.writeheader()
                
                # Write data rows
                for row in self.log_data:
                    writer.writerow(row)
                    
                self.statusBar.showMessage(f"Data exported to {filename}")
        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"Failed to export data: {str(e)}")
    
    def clear_data(self):
        """Clear all logged data and reset plots"""
        if len(self.log_data) > 0:
            if QMessageBox.question(self, "Confirm", "Clear all logged data?", 
                                   QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
                self.log_data = []
                self.plot_canvas.clear_data()
                self.export_button.setEnabled(False)
                self.logging_status.setText("Not logging")
                self.statusBar.showMessage("All data cleared")
        else:
            self.plot_canvas.clear_data()
            self.statusBar.showMessage("Plot cleared")
    
    def change_plot_window(self, value):
        """Change the plot time window size"""
        self.plot_canvas.set_window_size(float(value))
        self.statusBar.showMessage(f"Plot window set to {value} seconds")
    
    # Configuration Methods
    def save_configuration(self):
        """Save current configuration to a JSON file"""
        filename, _ = QFileDialog.getSaveFileName(self, "Save Configuration", "", "JSON Files (*.json);;All Files (*)")
        if not filename:
            return
            
        try:
            config = {
                'pid': {
                    'kp': self.kp_spin.value(),
                    'ki': self.ki_spin.value(),
                    'kd': self.kd_spin.value()
                },
                'control_mode': 'P' if self.position_radio.isChecked() else 'S',
                'setpoint': self.setpoint_spin.value(),
                'plot_window': self.window_spin.value(),
                'baudrate': self.baud_combo.currentText()
            }
            
            with open(filename, 'w') as f:
                json.dump(config, f, indent=4)
                
            self.statusBar.showMessage(f"Configuration saved to {filename}")
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save configuration: {str(e)}")
    
    def load_configuration(self):
        """Load configuration from a JSON file"""
        filename, _ = QFileDialog.getOpenFileName(self, "Load Configuration", "", "JSON Files (*.json);;All Files (*)")
        if not filename:
            return
            
        try:
            with open(filename, 'r') as f:
                config = json.load(f)
                
            # Apply loaded configuration
            if 'pid' in config:
                self.kp_spin.setValue(config['pid'].get('kp', 1.0))
                self.ki_spin.setValue(config['pid'].get('ki', 0.1))
                self.kd_spin.setValue(config['pid'].get('kd', 0.01))
                
                # Apply to Arduino if connected
                if self.serial_thread and self.serial_thread.running:
                    self.apply_pid_parameters()
            
            if 'control_mode' in config:
                if config['control_mode'] == 'P':
                    self.position_radio.setChecked(True)
                else:
                    self.speed_radio.setChecked(True)
                    
                # Apply to Arduino if connected
                if self.serial_thread and self.serial_thread.running:
                    self.change_control_mode(self.position_radio if config['control_mode'] == 'P' else self.speed_radio)
            
            if 'setpoint' in config:
                self.setpoint_spin.setValue(config['setpoint'])
                
                # Apply to Arduino if connected
                if self.serial_thread and self.serial_thread.running:
                    self.apply_setpoint()
            
            if 'plot_window' in config:
                self.window_spin.setValue(config['plot_window'])
                self.change_plot_window(config['plot_window'])
            
            if 'baudrate' in config:
                index = self.baud_combo.findText(config['baudrate'])
                if index >= 0:
                    self.baud_combo.setCurrentIndex(index)
                    
            self.statusBar.showMessage(f"Configuration loaded from {filename}")
        except Exception as e:
            QMessageBox.critical(self, "Load Error", f"Failed to load configuration: {str(e)}")
    
    # Utility Methods
    def setEnabledControls(self, enabled):
        """Enable or disable control widgets based on connection status"""
        # PID control widgets
        self.kp_spin.setEnabled(enabled)
        self.ki_spin.setEnabled(enabled)
        self.kd_spin.setEnabled(enabled)
        self.apply_pid_button.setEnabled(enabled)
        
        # Motor control widgets
        self.setpoint_spin.setEnabled(enabled)
        self.apply_setpoint_button.setEnabled(enabled)
        self.manual_slider.setEnabled(enabled)
        self.apply_manual_button.setEnabled(enabled)
        self.zero_button.setEnabled(enabled)
        self.reset_encoder_button.setEnabled(enabled)
        
        # Control mode widgets
        self.position_radio.setEnabled(enabled)
        self.speed_radio.setEnabled(enabled)
        
        # Emergency stop is always enabled if there's a connection
        self.emergency_stop_button.setEnabled(enabled)
    
    def update_ui(self):
        """Timer-based UI updates"""
        # Update UI based on connection status
        is_connected = self.serial_thread is not None and self.serial_thread.running
        
        # Update port selection enabled state
        self.port_combo.setEnabled(not is_connected)
        self.baud_combo.setEnabled(not is_connected)
        self.refresh_button.setEnabled(not is_connected)
    
    def show_about(self):
        """Show about dialog"""
        about_text = """
<h1>Motor Control with PID - Practice 3</h1>
<p>Version 1.0</p>
<p>A PyQt-based interface for controlling a DC motor with encoder feedback using PID control.</p>
<p>Features:</p>
<ul>
<li>Real-time plotting of position, speed, and control signals</li>
<li>PID parameter adjustment</li>
<li>Mode selection (Position/Speed control)</li>
<li>Manual motor control</li>
<li>Data logging and export</li>
<li>Configuration save/load</li>
<li>Emergency stop</li>
</ul>
<p>&copy; 2025 - Data Acquisition and Control Course</p>
"""
        QMessageBox.about(self, "About Motor Control GUI", about_text)
    
    def save_settings(self):
        """Save application settings to QSettings"""
        settings = QSettings("DataAcquisition", "MotorSpeedControlGUI")
        
        # Save PID parameters
        settings.setValue("pid/kp", self.kp_spin.value())
        settings.setValue("pid/ki", self.ki_spin.value())
        settings.setValue("pid/kd", self.kd_spin.value())
        
        # Save control mode
        settings.setValue("control_mode", "P" if self.position_radio.isChecked() else "S")
        
        # Save plot window size
        settings.setValue("plot_window", self.window_spin.value())
        
        # Save baudrate
        settings.setValue("baudrate", self.baud_combo.currentText())
        
        # Save window geometry
        settings.setValue("geometry", self.saveGeometry())
        settings.setValue("windowState", self.saveState())
        
        # Save splitter state
        settings.setValue("splitter", self.splitter.saveState())
    
    def load_settings(self):
        """Load application settings from QSettings"""
        settings = QSettings("DataAcquisition", "MotorSpeedControlGUI")
        
        # Load PID parameters
        self.kp_spin.setValue(float(settings.value("pid/kp", 1.0)))
        self.ki_spin.setValue(float(settings.value("pid/ki", 0.1)))
        self.kd_spin.setValue(float(settings.value("pid/kd", 0.01)))
        
        # Load control mode
        mode = settings.value("control_mode", "P")
        if mode == "P":
            self.position_radio.setChecked(True)
            self.control_mode = "P"
        else:
            self.speed_radio.setChecked(True)
            self.control_mode = "S"
        
        # Load plot window size
        window_size = settings.value("plot_window", 10)
        if window_size:
            self.window_spin.setValue(int(window_size))
            self.plot_canvas.set_window_size(float(window_size))
        
        # Load baudrate
        baudrate = settings.value("baudrate", "115200")
        index = self.baud_combo.findText(baudrate)
        if index >= 0:
            self.baud_combo.setCurrentIndex(index)
        
        # Load window geometry if available
        if settings.contains("geometry"):
            self.restoreGeometry(settings.value("geometry"))
        if settings.contains("windowState"):
            self.restoreState(settings.value("windowState"))
        
        # Load splitter state if available
        if settings.contains("splitter"):
            self.splitter.restoreState(settings.value("splitter"))
    
    def closeEvent(self, event):
        """Handle window close event"""
        # Disconnect from serial port if connected
        if self.serial_thread and self.serial_thread.running:
            # Ask user if they want to stop the motor before exiting
            reply = QMessageBox.question(self, "Exit Confirmation",
                                        "Stop the motor before exiting?",
                                        QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
            
            if reply == QMessageBox.Cancel:
                event.ignore()
                return
            
            if reply == QMessageBox.Yes:
                self.serial_thread.send_command("MOTOR,0")
            
            # Disconnect
            self.disconnect_serial()
        
        # Check if data logging is active and offer to export
        if len(self.log_data) > 0 and not self.export_button.isEnabled():
            reply = QMessageBox.question(self, "Export Data",
                                        "Would you like to export the logged data before exiting?",
                                        QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
            
            if reply == QMessageBox.Cancel:
                event.ignore()
                return
            
            if reply == QMessageBox.Yes:
                self.export_data()
        
        # Save application settings
        self.save_settings()
        
        # Accept the event and close the window
        event.accept()
        
    def log_message(self, message, message_type="info"):
        """
        Add a message to the log with color formatting.
        
        Args:
            message (str): Message to log
            message_type (str): Type of message ('info', 'success', 'warning', 'error')
        """
        if not hasattr(self, 'log_text'):
            print(f"Log: {message}")  # Fall back to console if log_text doesn't exist
            return
            
        # Set color based on message type
        color = {
            "info": "black",
            "success": "green",
            "warning": "orange",
            "error": "red"
        }.get(message_type, "black")
        
        # Add timestamp
        timestamp = time.strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}"
        
        # Add to log with color
        self.log_text.append(f"<span style='color:{color};'>{formatted_message}</span>")
        
        # Ensure the most recent message is visible
        self.log_text.ensureCursorVisible()


def main():
    """Main application entry point"""
    # Create application
    app = QApplication(sys.argv)
    app.setApplicationName("Motor Control GUI")
    app.setOrganizationName("DataAcquisition")
    app.setOrganizationDomain("dataacquisition.edu")
    
    # Create and show main window
    main_window = MotorSpeedControlGUI()
    main_window.show()
    
    # Run application event loop
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
