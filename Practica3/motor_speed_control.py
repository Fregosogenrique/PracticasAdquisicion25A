#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motor Speed Control - Practice 3
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
        """Set the plot time window size"""
        self.window_size = float(seconds)
    
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
        
        # Configuración de la ventana principal
        self.setWindowTitle("Control de Velocidad de Motor con Encoder y PID")
        self.setMinimumSize(900, 700)
        
        # Widget central y layout principal
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        
        # Crear el control_layout
        self.control_layout = QVBoxLayout()
        control_widget = QWidget()
        control_widget.setLayout(self.control_layout)
        
        # Crear el log_text primero
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        
        # Crear panel de log
        log_group = QGroupBox("Registro de Eventos")
        log_layout = QVBoxLayout()
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        
        # Añadir paneles en orden
        self.main_layout.addWidget(log_group)
        self.main_layout.addWidget(control_widget)
        
        # Setup remaining panels
        self.setup_connection_panel()
        self.setup_tabs()
        
        # Initialize data storage
        self.log_data = []
        self.is_logging = False
        self.start_time = None
        self.control_mode = 'P'  # Default to position mode
        
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
        
        # Refresh port list
        self.refresh_ports()
        
    def setup_connection_panel(self):
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
    
    def setup_tabs(self):
        """Create and setup the tab widgets for different control panels"""
        # Create tab widget
        self.tabs = QTabWidget()
        
        # Create tabs
        self.control_tab = QWidget()
        self.pid_tab = QWidget()
        self.plotting_tab = QWidget()
        
        # Add tabs to widget
        self.tabs.addTab(self.control_tab, "Control")
        self.tabs.addTab(self.pid_tab, "PID Settings")
        self.tabs.addTab(self.plotting_tab, "Plotting")
        
        # Setup tab contents
        self.setup_control_tab()
        self.setup_pid_tab()
        self.setup_plotting_tab()
        
        # Add to main layout
        self.main_layout.addWidget(self.tabs)
    
    def setup_control_tab(self):
        """Setup the motor control tab"""
        control_layout = QVBoxLayout(self.control_tab)
        
        # Create motor controls
        motor_group = QGroupBox("Motor Control")
        motor_layout = QGridLayout()
        
        # Direction control
        self.direction_label = QLabel("Direction:")
        self.cw_button = QPushButton("Clockwise (CW)")
        self.cw_button.setCheckable(True)
        self.cw_button.setChecked(True)
        self.cw_button.clicked.connect(lambda: self.set_direction("clockwise"))
        
        self.ccw_button = QPushButton("Counter-Clockwise (CCW)")
        self.ccw_button.setCheckable(True)
        self.ccw_button.clicked.connect(lambda: self.set_direction("counterclockwise"))
        
        # Speed control
        self.speed_label = QLabel("Speed (RPM):")
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(200)
        self.speed_slider.setValue(0)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(20)
        self.speed_slider.valueChanged.connect(self.update_target_speed)
        
        self.speed_spinbox = QSpinBox()
        self.speed_spinbox.setMinimum(0)
        self.speed_spinbox.setMaximum(200)
        self.speed_spinbox.setValue(0)
        self.speed_spinbox.valueChanged.connect(self.update_target_speed_from_spinbox)
        
        # Control buttons
        self.start_stop_button = QPushButton("Start")
        self.start_stop_button.clicked.connect(self.toggle_motor)
        
        self.emergency_stop_button = QPushButton("EMERGENCY STOP")
        self.emergency_stop_button.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.emergency_stop_button.clicked.connect(self.emergency_stop)
        
        # Add widgets to layout
        motor_layout.addWidget(self.direction_label, 0, 0)
        motor_layout.addWidget(self.cw_button, 0, 1)
        motor_layout.addWidget(self.ccw_button, 0, 2)
        
        motor_layout.addWidget(self.speed_label, 1, 0)
        motor_layout.addWidget(self.speed_slider, 1, 1, 1, 2)
        motor_layout.addWidget(self.speed_spinbox, 1, 3)
        
        motor_layout.addWidget(self.start_stop_button, 2, 0, 1, 2)
        motor_layout.addWidget(self.emergency_stop_button, 2, 2, 1, 2)
        
        motor_group.setLayout(motor_layout)
        control_layout.addWidget(motor_group)
        
        # Add status indicators
        status_group = QGroupBox("Status")
        status_layout = QGridLayout()
        
        # Speed indicator
        current_speed_label = QLabel("Current Speed (RPM):")
        self.current_speed_value = QLabel("0.0")
        self.current_speed_value.setStyleSheet("font-size: 16px; font-weight: bold;")
        
        # Encoder position
        encoder_pos_label = QLabel("Encoder Position:")
        self.encoder_pos_value = QLabel("0")
        
        # Rotation counter
        rotation_count_label = QLabel("Rotation Count:")
        self.rotation_count_value = QLabel("0")
        
        # PWM value
        pwm_value_label = QLabel("PWM Value:")
        self.pwm_value = QLabel("0")
        
        # Operation status
        operation_label = QLabel("Operation Status:")
        self.operation_value = QLabel("Stopped")
        
        # Add indicators to layout
        status_layout.addWidget(current_speed_label, 0, 0)
        status_layout.addWidget(self.current_speed_value, 0, 1)
        
        status_layout.addWidget(encoder_pos_label, 1, 0)
        status_layout.addWidget(self.encoder_pos_value, 1, 1)
        
        status_layout.addWidget(rotation_count_label, 2, 0)
        status_layout.addWidget(self.rotation_count_value, 2, 1)
        
        status_layout.addWidget(pwm_value_label, 3, 0)
        status_layout.addWidget(self.pwm_value, 3, 1)
        
        status_layout.addWidget(operation_label, 4, 0)
        status_layout.addWidget(self.operation_value, 4, 1)
        
        status_group.setLayout(status_layout)
        control_layout.addWidget(status_group)
    
    def setup_pid_tab(self):
        """Setup the PID parameters tab"""
        pid_layout = QVBoxLayout(self.pid_tab)
        
        # Description
        description = QLabel(
            "Adjust PID parameters to optimize speed control. "
            "Changes will be applied immediately to the controller."
        )
        description.setWordWrap(True)
        pid_layout.addWidget(description)
        
        # PID parameters
        params_layout = QGridLayout()
        
        # Kp - Proportional gain
        kp_label = QLabel("Proportional Gain (Kp):")
        self.kp_spinbox = QDoubleSpinBox()
        self.kp_spinbox.setMinimum(0.0)
        self.kp_spinbox.setMaximum(10.0)
        self.kp_spinbox.setSingleStep(0.1)
        self.kp_spinbox.setValue(0.5)  # Default value
        self.kp_spinbox.setToolTip("Controls the response proportional to the current error")
        
        # Ki - Integral gain
        ki_label = QLabel("Integral Gain (Ki):")
        self.ki_spinbox = QDoubleSpinBox()
        self.ki_spinbox.setMinimum(0.0)
        self.ki_spinbox.setMaximum(10.0)
        self.ki_spinbox.setSingleStep(0.1)
        self.ki_spinbox.setValue(0.2)  # Default value
        self.ki_spinbox.setToolTip("Controls the response to the accumulation of past errors")
        
        # Kd - Derivative gain
        kd_label = QLabel("Derivative Gain (Kd):")
        self.kd_spinbox = QDoubleSpinBox()
        self.kd_spinbox.setMinimum(0.0)
        self.kd_spinbox.setMaximum(10.0)
        self.kd_spinbox.setSingleStep(0.1)
        self.kd_spinbox.setValue(0.1)  # Default value
        self.kd_spinbox.setToolTip("Controls the response to the rate of change of the error")
        
        # Apply button
        self.apply_pid_button = QPushButton("Apply PID Parameters")
        self.apply_pid_button.clicked.connect(self.apply_pid_parameters)
        
        # Add widgets to layout
        params_layout.addWidget(kp_label, 0, 0)
        params_layout.addWidget(self.kp_spinbox, 0, 1)
        
        params_layout.addWidget(ki_label, 1, 0)
        params_layout.addWidget(self.ki_spinbox, 1, 1)
        
        params_layout.addWidget(kd_label, 2, 0)
        params_layout.addWidget(self.kd_spinbox, 2, 1)
        
        # Add apply button
        params_layout.addWidget(self.apply_pid_button, 3, 0, 1, 2)
        
        pid_layout.addLayout(params_layout)
        
        # Explanation of PID effects 
        pid_effects = QTextEdit()
        pid_effects.setReadOnly(True)
        pid_effects.setHtml("""
            <h3>PID Parameter Effects</h3>
            <p><b>Proportional Gain (Kp):</b> Determines the response strength to current error. High values may cause oscillations.</p>
            <p><b>Integral Gain (Ki):</b> Eliminates steady-state error. High values may cause overshoot.</p>
            <p><b>Derivative Gain (Kd):</b> Improves response time and reduces oscillations. Sensitive to signal noise.</p>
            <h4>Recommended Tuning:</h4>
            <ol>
                <li>Set all values to zero</li>
                <li>Increase Kp until you get a fast response with slight oscillations</li>
                <li>Increase Kd to reduce oscillations</li>
                <li>Increase Ki to eliminate steady-state error</li>
            </ol>
        """)
        
        pid_layout.addWidget(pid_effects)
    
    def setup_plotting_tab(self):
        """Setup the plotting tab"""
        plot_layout = QVBoxLayout(self.plotting_tab)
        
        # Add the plot canvas
        self.plot_canvas = PlotCanvas(self, width=5, height=5, dpi=100)
        plot_layout.addWidget(self.plot_canvas)
        
        # Plot controls
        controls_layout = QHBoxLayout()
        
        # Window size selector
        window_label = QLabel("Plot Window (seconds):")
        self.window_spinner = QSpinBox()
        self.window_spinner.setRange(5, 60)
        self.window_spinner.setValue(10)
        self.window_spinner.valueChanged.connect(lambda value: self.plot_canvas.set_window_size(value))
        
        # Clear button
        clear_button = QPushButton("Clear Plot")
        clear_button.clicked.connect(self.plot_canvas.clear_data)
        
        # Add controls to layout
        controls_layout.addWidget(window_label)
        controls_layout.addWidget(self.window_spinner)
        controls_layout.addStretch()
        controls_layout.addWidget(clear_button)
        
        plot_layout.addLayout(controls_layout)
    
    def refresh_ports(self):
        """Refresh the list of available serial ports"""
        self.port_combo.clear()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        if ports:
            self.port_combo.addItems(ports)
            self.log_message(f"Se encontraron {len(ports)} puertos seriales.")
        else:
            self.log_message("No se encontraron puertos seriales. Conecte el Arduino.")
    
    def toggle_connection(self):
        """Connect or disconnect from the selected serial port"""
        if not self.serial_thread or not self.serial_thread.running:
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
                self.log_message(f"Connected to {port} at {baudrate} baud", "success")
                
                # Enable control widgets
                self.cw_button.setEnabled(True)
                self.ccw_button.setEnabled(True)
                self.speed_slider.setEnabled(True)
                self.speed_spinbox.setEnabled(True)
                self.start_stop_button.setEnabled(True)
                self.emergency_stop_button.setEnabled(True)
                self.apply_pid_button.setEnabled(True)
                
                # Start time reference
                self.start_time = time.time()
                
            except Exception as e:
                QMessageBox.critical(self, "Connection Error", f"Failed to connect: {str(e)}")
        else:
            # Disconnect
            # Stop the motor if running
            if hasattr(self, 'is_running') and self.is_running:
                self.serial_thread.send_command("STOP")
                
            # Stop the thread
            self.serial_thread.stop()
            self.serial_thread = None
            
            # Update UI
            self.connect_button.setText("Connect")
            self.log_message("Disconnected from serial port")
            
            # Disable control widgets
            self.cw_button.setEnabled(False)
            self.ccw_button.setEnabled(False)
            self.speed_slider.setEnabled(False)
            self.speed_spinbox.setEnabled(False)
            self.start_stop_button.setEnabled(False)
            self.emergency_stop_button.setEnabled(False)
            self.apply_pid_button.setEnabled(False)
    
    def process_serial_data(self, data):
        """Process data received from Arduino"""
        try:
            # Check if it's a data packet
            if data.startswith("DATA"):
                parts = data.split(',')
                if len(parts) >= 5:  # Ensure we have enough data elements
                    # Extract values
                    timestamp = float(parts[1])
                    position = int(parts[2])
                    speed = int(parts[3])
                    setpoint = float(parts[4])
                    control = float(parts[5]) if len(parts) > 5 else 0
                    mode = parts[6] if len(parts) > 6 else 'P'
                    
                    # Calculate relative time
                    rel_time = time.time() - self.start_time
                    
                    # Update plot
                    self.plot_canvas.update_plot(rel_time, position, speed, setpoint, control, mode)
                    
                    # Update status indicators
                    self.current_speed_value.setText(f"{speed}")
                    self.encoder_pos_value.setText(f"{position}")
                    self.pwm_value.setText(f"{control}")
                    
            # Process other responses
            else:
                self.log_message(f"Arduino: {data}")
                
        except Exception as e:
            self.log_message(f"Error processing data: {str(e)}", "error")
    
    def handle_connection_error(self, error_message):
        """Handle serial connection errors"""
        QMessageBox.warning(self, "Connection Error", error_message)
        self.disconnect_serial()
    
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

    def update_target_speed(self, value):
        """Update target speed from slider"""
        self.speed_spinbox.setValue(value)
        if self.serial_thread and self.serial_thread.running:
            self.serial_thread.send_command(f"SPEED,{value}")
            self.log_message(f"Setting speed to {value} RPM")

    def update_target_speed_from_spinbox(self, value):
        """Update target speed from spinbox"""
        self.speed_slider.setValue(value)
        if self.serial_thread and self.serial_thread.running:
            self.serial_thread.send_command(f"SPEED,{value}")
            self.log_message(f"Setting speed to {value} RPM")

    def set_direction(self, direction):
        """Set motor direction"""
        if direction == "clockwise":
            self.cw_button.setChecked(True)
            self.ccw_button.setChecked(False)
            self.log_message("Setting direction to clockwise")
        else:
            self.cw_button.setChecked(False)
            self.ccw_button.setChecked(True)
            self.log_message("Setting direction to counter-clockwise")
        
        if self.serial_thread and self.serial_thread.running:
            self.serial_thread.send_command(f"DIR,{direction}")

    def toggle_motor(self):
        """Start or stop the motor"""
        if not self.serial_thread or not self.serial_thread.running:
            return
            
        if self.start_stop_button.text() == "Start":
            self.start_stop_button.setText("Stop")
            self.operation_value.setText("Running")
            self.serial_thread.send_command("START")
            self.log_message("Motor started", "success")
        else:
            self.start_stop_button.setText("Start")
            self.operation_value.setText("Stopped")
            self.serial_thread.send_command("STOP")
            self.log_message("Motor stopped")

    def emergency_stop(self):
        """Emergency stop the motor"""
        if self.serial_thread and self.serial_thread.running:
            self.serial_thread.send_command("ESTOP")
            self.start_stop_button.setText("Start")
            self.operation_value.setText("Emergency Stopped")
            self.operation_value.setStyleSheet("color: red; font-weight: bold;")
            self.log_message("EMERGENCY STOP activated!", "error")

    def apply_pid_parameters(self):
        """Apply PID parameters to the controller"""
        if self.serial_thread and self.serial_thread.running:
            kp = self.kp_spinbox.value()
            ki = self.ki_spinbox.value()
            kd = self.kd_spinbox.value()
            self.serial_thread.send_command(f"PID,{kp},{ki},{kd}")
            self.log_message(f"Applied PID parameters: Kp={kp}, Ki={ki}, Kd={kd}", "success")

    def save_configuration(self):
        """Save configuration to a file"""
        filename, _ = QFileDialog.getSaveFileName(self, "Save Configuration", "", 
                                            "JSON Files (*.json);;All Files (*)")
        if not filename:
            return
            
        try:
            config = {
                'pid': {
                    'kp': self.kp_spinbox.value(),
                    'ki': self.ki_spinbox.value(),
                    'kd': self.kd_spinbox.value()
                },
                'plot_window': self.window_spinner.value(),
                'baudrate': self.baud_combo.currentText()
            }
            
            with open(filename, 'w') as f:
                json.dump(config, f, indent=4)
                
            self.log_message(f"Configuration saved to {filename}", "success")
                
        except Exception as e:
            self.log_message(f"Error saving configuration: {str(e)}", "error")
            QMessageBox.critical(self, "Save Error", f"Failed to save configuration: {str(e)}")

    def load_configuration(self):
        """Load configuration from a file"""
        filename, _ = QFileDialog.getOpenFileName(self, "Load Configuration", "", 
                                             "JSON Files (*.json);;All Files (*)")
        if not filename:
            return
            
        try:
            with open(filename, 'r') as f:
                config = json.load(f)
                
            # Apply PID parameters
            if 'pid' in config:
                self.kp_spinbox.setValue(float(config['pid'].get('kp', 0.5)))
                self.ki_spinbox.setValue(float(config['pid'].get('ki', 0.2)))
                self.kd_spinbox.setValue(float(config['pid'].get('kd', 0.1)))
                
                # Apply to Arduino if connected
                if self.serial_thread and self.serial_thread.running:
                    self.apply_pid_parameters()
            
            # Apply plot window size
            if 'plot_window' in config:
                window_size = int(config['plot_window'])
                self.window_spinner.setValue(window_size)
                self.plot_canvas.set_window_size(float(window_size))
            
            # Set baudrate selection
            if 'baudrate' in config:
                index = self.baud_combo.findText(config['baudrate'])
                if index >= 0:
                    self.baud_combo.setCurrentIndex(index)
                    
            self.log_message(f"Configuration loaded from {filename}", "success")
        except Exception as e:
            self.log_message(f"Error loading configuration: {str(e)}", "error")
            QMessageBox.critical(self, "Load Error", f"Failed to load configuration: {str(e)}")

    def show_about(self):
        """Show the about dialog"""
        about_text = """
<h1>Motor Speed Control - Practice 3</h1>
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
        QMessageBox.about(self, "About Motor Speed Control", about_text)
    
    def update_ui(self):
        """Timer-based UI updates"""
        # Update UI based on connection status
        is_connected = self.serial_thread is not None and self.serial_thread.running
        
        # Update port selection enabled state
        self.port_combo.setEnabled(not is_connected)
        self.baud_combo.setEnabled(not is_connected)
        self.refresh_button.setEnabled(not is_connected)
    
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
                fieldnames = ['time', 'position', 'speed', 'setpoint', 'control']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                # Write header
                writer.writeheader()
                
                # Write data rows
                for row in self.log_data:
                    writer.writerow(row)
                    
                self.log_message(f"Data exported to {filename}", "success")
        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"Failed to export data: {str(e)}")

    def closeEvent(self, event):
        """Handle window close event"""
        # Disconnect from serial port if connected
        if self.serial_thread and self.serial_thread.running:
            # Send stop command to motor
            self.serial_thread.send_command("STOP")
            
            # Stop the thread
            self.serial_thread.stop()
        
        # Save application settings
        self.save_settings()
        
        # Accept the event and close the window
        event.accept()

    def save_settings(self):
        """Save application settings to QSettings"""
        settings = QSettings("DataAcquisition", "MotorSpeedControlGUI")
        
        # Save PID parameters
        settings.setValue("pid/kp", self.kp_spinbox.value())
        settings.setValue("pid/ki", self.ki_spinbox.value())
        settings.setValue("pid/kd", self.kd_spinbox.value())
        
        # Save plot window size
        settings.setValue("plot_window", self.window_spinner.value())
        
        # Save baudrate
        settings.setValue("baudrate", self.baud_combo.currentText())
        
        # Save window geometry
        settings.setValue("geometry", self.saveGeometry())
        settings.setValue("windowState", self.saveState())

    def load_settings(self):
        """Load application settings from QSettings"""
        settings = QSettings("DataAcquisition", "MotorSpeedControlGUI")
        
        # Load PID parameters
        self.kp_spinbox.setValue(float(settings.value("pid/kp", 0.5)))
        self.ki_spinbox.setValue(float(settings.value("pid/ki", 0.2)))
        self.kd_spinbox.setValue(float(settings.value("pid/kd", 0.1)))
        
        # Load plot window size
        window_size = settings.value("plot_window", 10)
        if window_size:
            try:
                self.window_spinner.setValue(int(window_size))
                if hasattr(self.plot_canvas, 'set_window_size'):
                    self.plot_canvas.set_window_size(float(window_size))
                else:
                    self.plot_canvas.window_size = float(window_size)
            except (ValueError, AttributeError) as e:
                self.log_message(f"Error setting window size: {str(e)}", "warning")
        
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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorSpeedControlGUI()
    window.show()
    sys.exit(app.exec_())
