"""
Motor Control GUI - Interface for controlling stepper motors via Arduino.
Provides controls for X, Y, and R axes with real-time status feedback.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import threading
import time
import logging
import sys

# Import local modules
import config
from serial_handler import SerialHandler

# Configure logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("MotorControlGUI")

class MotorControlGUI:
    """GUI application for controlling stepper motors via Arduino."""
    
    def __init__(self, root):
        """
        Initialize the motor control GUI.
        
        Args:
            root: The tkinter root window
        """
        self.root = root
        self.root.title(config.WINDOW_TITLE)
        self.root.geometry(f"{config.WINDOW_WIDTH}x{config.WINDOW_HEIGHT}")
        self.root.configure(bg=config.BACKGROUND_COLOR)
        
        # Initialize serial handler
        self.serial_handler = SerialHandler()
        
        # Initialize state variables
        self.connected = False
        self.axis_speeds = {axis: config.DEFAULT_SPEED for axis in config.AXES}
        self.axis_directions = {axis: config.DIRECTION_FORWARD for axis in config.AXES}
        self.current_axis = config.AXES[0]  # Default to X axis
        
        # Create and layout widgets
        self._create_widgets()
        self._layout_widgets()
        
        # Register callbacks with serial handler
        self.serial_handler.register_status_callback(self._update_status)
        self.serial_handler.register_connection_callback(self._connection_changed)
        
        # Setup regular port refresh
        self._refresh_ports()
        
        # Bind window close event
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
        
        # Refresh status every 100ms
        self._schedule_status_refresh()
    
    def _create_widgets(self):
        """Create all GUI widgets."""
        # Create main frames
        self.connection_frame = ttk.LabelFrame(self.root, text="Connection")
        self.axes_frame = ttk.Frame(self.root)
        self.status_frame = ttk.LabelFrame(self.root, text="Status")
        
        # Create connection controls
        self.port_label = ttk.Label(self.connection_frame, text="Serial Port:")
        self.port_combobox = ttk.Combobox(self.connection_frame, width=20)
        self.refresh_button = ttk.Button(self.connection_frame, text="Refresh", 
                                         command=self._refresh_ports)
        self.connect_button = ttk.Button(self.connection_frame, text="Connect", 
                                         command=self._toggle_connection)
        self.stop_button = ttk.Button(self.connection_frame, text="EMERGENCY STOP", 
                                      command=self._emergency_stop)
        self.stop_button.configure(style="Emergency.TButton")
        
        # Create a custom style for the emergency button
        self.style = ttk.Style()
        self.style.configure("Emergency.TButton", background=config.DISABLED_COLOR, 
                            foreground="white", font=("Arial", 12, "bold"))
        
        # Create axis control frames
        self.axis_frames = {}
        self.axis_labels = {}
        self.speed_labels = {}
        self.speed_scales = {}
        self.speed_entries = {}
        self.direction_buttons = {}
        self.position_labels = {}
        self.position_values = {}
        self.value_labels = {}
        self.value_bars = {}
        
        for axis in config.AXES:
            # Create a frame for this axis
            axis_frame = ttk.LabelFrame(self.axes_frame, text=f"{axis} Axis")
            self.axis_frames[axis] = axis_frame
            
            # Axis selection button
            select_button = ttk.Button(axis_frame, text=f"Select {axis}", 
                                      command=lambda a=axis: self._select_axis(a))
            select_button.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
            
            # Speed controls
            speed_label = ttk.Label(axis_frame, text="Speed:")
            speed_scale = ttk.Scale(axis_frame, from_=config.MIN_SPEED, to=config.MAX_SPEED,
                                    orient=tk.HORIZONTAL, value=config.DEFAULT_SPEED,
                                    command=lambda val, a=axis: self._update_speed(a, val))
            
            speed_var = tk.StringVar(value=str(config.DEFAULT_SPEED))
            speed_entry = ttk.Entry(axis_frame, textvariable=speed_var, width=6)
            speed_entry.bind("<Return>", lambda event, a=axis, var=speed_var: 
                             self._speed_entry_update(a, var))
            
            speed_label.grid(row=1, column=0, padx=5, pady=5, sticky="w")
            speed_scale.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
            speed_entry.grid(row=1, column=2, padx=5, pady=5)
            
            self.speed_labels[axis] = speed_label
            self.speed_scales[axis] = speed_scale
            self.speed_entries[axis] = speed_entry
            
            # Direction control
            direction_button = ttk.Button(axis_frame, text="Forward", width=12,
                                         command=lambda a=axis: self._toggle_direction(a))
            direction_button.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="w")
            self.direction_buttons[axis] = direction_button
            
            # Position display
            position_label = ttk.Label(axis_frame, text="Position:")
            position_value = ttk.Label(axis_frame, text="0", width=12, 
                                       background="white", anchor="e")
            position_label.grid(row=3, column=0, padx=5, pady=5, sticky="w")
            position_value.grid(row=3, column=1, padx=5, pady=5, sticky="ew")
            
            self.position_labels[axis] = position_label
            self.position_values[axis] = position_value
            
            # Input value display
            value_label = ttk.Label(axis_frame, text="Input Value:")
            value_bar = ttk.Progressbar(axis_frame, orient=tk.HORIZONTAL, length=100, mode='determinate')
            value_bar["maximum"] = 1023
            value_bar["value"] = 0
            
            value_label.grid(row=4, column=0, padx=5, pady=5, sticky="w")
            value_bar.grid(row=4, column=1, columnspan=2, padx=5, pady=5, sticky="ew")
            
            self.value_labels[axis] = value_label
            self.value_bars[axis] = value_bar
        
        # Create status display widgets
        self.connection_status_label = ttk.Label(self.status_frame, text="Connection:")
        self.connection_status_value = ttk.Label(self.status_frame, text="Disconnected",
                                                foreground=config.DISABLED_COLOR)
        
        self.active_axis_label = ttk.Label(self.status_frame, text="Active Axis:")
        self.active_axis_value = ttk.Label(self.status_frame, text=config.AXES[0])
        
        self.motor_state_label = ttk.Label(self.status_frame, text="Motor State:")
        self.motor_state_value = ttk.Label(self.status_frame, text="Disabled",
                                          foreground=config.DISABLED_COLOR)
        
        self.current_speed_label = ttk.Label(self.status_frame, text="Current Speed:")
        self.current_speed_value = ttk.Label(self.status_frame, text="0 steps/sec")
        
        self.current_direction_label = ttk.Label(self.status_frame, text="Direction:")
        self.current_direction_value = ttk.Label(self.status_frame, 
                                               text=config.DIRECTION_LABELS[config.DIRECTION_FORWARD])
    
    def _layout_widgets(self):
        """Layout all GUI widgets."""
        # Configure grid
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)
        
        # Place main frames
        self.connection_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        self.axes_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        self.status_frame.grid(row=2, column=0, padx=10, pady=10, sticky="ew")
        
        # Configure connection frame grid
        self.connection_frame.columnconfigure(1, weight=1)
        
        # Layout connection controls
        self.port_label.grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.port_combobox.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.refresh_button.grid(row=0, column=2, padx=5, pady=5)
        self.connect_button.grid(row=0, column=3, padx=5, pady=5)
        self.stop_button.grid(row=1, column=0, columnspan=4, padx=5, pady=5, sticky="ew")
        
        # Configure axes frame grid
        self.axes_frame.columnconfigure(0, weight=1)
        self.axes_frame.columnconfigure(1, weight=1)
        self.axes_frame.columnconfigure(2, weight=1)
        
        # Layout axis frames
        for i, axis in enumerate(config.AXES):
            self.axis_frames[axis].grid(row=0, column=i, padx=10, pady=10, sticky="nsew")
            self.axis_frames[axis].columnconfigure(1, weight=1)
            
            # Add color indication for active axis
            self.axis_frames[axis].configure(style=f"{axis}.TLabelframe")
            self.style.configure(f"{axis}.TLabelframe", background=config.AXIS_COLORS[axis])
            self.style.configure(f"{axis}.TLabelframe.Label", 
                               foreground=config.AXIS_COLORS[axis],
                               font=("Arial", 12, "bold"))
        
        # Layout status display
        self.connection_status_label.grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.connection_status_value.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        self.active_axis_label.grid(row=0, column=2, padx=5, pady=5, sticky="w")
        self.active_axis_value.grid(row=0, column=3, padx=5, pady=5, sticky="w")
        
        self.motor_state_label.grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.motor_state_value.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        
        self.current_speed_label.grid(row=1, column=2, padx=5, pady=5, sticky="w")
        self.current_speed_value.grid(row=1, column=3, padx=5, pady=5, sticky="w")
        
        self.current_direction_label.grid(row=2, column=0, padx=5, pady=5, sticky="w")
        self.current_direction_value.grid(row=2, column=1, padx=5, pady=5, sticky="w")
    
    def _refresh_ports(self):
        """Refresh the list of available serial ports."""
        ports = self.serial_handler.list_ports()
        self.port_combobox['values'] = ports
        
        # Select the first port if available and none is selected
        if ports and not self.port_combobox.get():
            self.port_combobox.current(0)
    
    def _toggle_connection(self):
        """Connect to or disconnect from the Arduino."""
        if self.connected:
            # Disconnect
            self.serial_handler.disconnect()
        else:
            # Connect
            port = self.port_combobox.get()
            if not port:
                messagebox.showerror("Connection Error", 
                                   "Please select a serial port.")
                return
            
            # Attempt to connect
            if self.serial_handler.connect(port):
                logger.info(f"Connected to {port}")
            else:
                messagebox.showerror("Connection Error", 
                                   f"Failed to connect to {port}. Check connections and try again.")
    
    def _connection_changed(self, connected):
        """
        Handle connection status changes.
        
        Args:
            connected: True if connected, False otherwise
        """
        self.connected = connected
        
        # Update button text
        self.connect_button.config(text="Disconnect" if connected else "Connect")
        
        # Update status display
        status_text = "Connected" if connected else "Disconnected"
        status_color = config.ENABLED_COLOR if connected else config.DISABLED_COLOR
        self.connection_status_value.config(text=status_text, foreground=status_color)
        
        # Enable/disable controls based on connection status
        state = "normal" if connected else "disabled"
        for axis in config.AXES:
            self.speed_scales[axis].config(state=state)
            self.speed_entries[axis].config(state=state)
            self.direction_buttons[axis].config(state=state)
        
        self.stop_button.config(state=state)
    
    def _select_axis(self, axis):
        """
        Select an axis for control.
        
        Args:
            axis: The axis to select (X, Y, or R)
        """
        if not self.connected:
            return
        
        self.current_axis = axis
        self._highlight_active_axis()
        
        # Send command to Arduino to set active axis
        speed = self.axis_speeds[axis]
        direction = self.axis_directions[axis]
        self.serial_handler.send_axis_command(axis, speed, direction)
    
    def _update_speed(self, axis, value):
        """
        Update the speed for an axis from slider movement.
        
        Args:
            axis: The axis to update (X, Y, or R)
            value: The new speed value from the slider
        """
        try:
            # Convert slider value to integer
            speed = int(float(value))
            
            # Update stored speed and entry widget
            self.axis_speeds[axis] = speed
            self.speed_entries[axis].delete(0, tk.END)
            self.speed_entries[axis].insert(0, str(speed))
            
            # If this is the active axis and we're connected, send command
            if self.connected and axis == self.current_axis:
                direction = self.axis_directions[axis]
                self.serial_handler.send_axis_command(axis, speed, direction)
        except ValueError:
            logger.error(f"Invalid speed value: {value}")
    
    def _speed_entry_update(self, axis, var):
        """
        Update the speed for an axis from manual entry.
        
        Args:
            axis: The axis to update (X, Y, or R)
            var: StringVar containing the new speed value
        """
        try:
            # Get and validate speed value
            speed_str = var.get()
            speed = int(speed_str)
            
            # Ensure speed is within limits
            if speed < config.MIN_SPEED:
                speed = config.MIN_SPEED
            elif speed > config.MAX_SPEED:
                speed = config.MAX_SPEED
            
            # Update stored speed and slider widget
            self.axis_speeds[axis] = speed
            self.speed_scales[axis].set(speed)
            
            # Update entry with clamped value if needed
            if str(speed) != speed_str:
                var.set(str(speed))
            
            # If this is the active axis and we're connected, send command
            if self.connected and axis == self.current_axis:
                direction = self.axis_directions[axis]
                self.serial_handler.send_axis_command(axis, speed, direction)
        except ValueError:
            # Reset to previous valid value if input is invalid
            var.set(str(self.axis_speeds[axis]))
            logger.error(f"Invalid speed value entered: {var.get()}")
    
    def _toggle_direction(self, axis):
        """
        Toggle the direction for an axis.
        
        Args:
            axis: The axis to update (X, Y, or R)
        """
        if not self.connected:
            return
        
        # Toggle direction between forward and reverse
        current = self.axis_directions[axis]
        new_direction = config.DIRECTION_REVERSE if current == config.DIRECTION_FORWARD else config.DIRECTION_FORWARD
        self.axis_directions[axis] = new_direction
        
        # Update button text
        label = config.DIRECTION_LABELS[new_direction]
        self.direction_buttons[axis].config(text=label)
        
        # If this is the active axis, send command
        if axis == self.current_axis:
            speed = self.axis_speeds[axis]
            self.serial_handler.send_axis_command(axis, speed, new_direction)
    
    def _emergency_stop(self):
        """Send emergency stop command to Arduino."""
        if self.connected:
            self.serial_handler.send_stop_command()
            messagebox.showinfo("Emergency Stop", "Emergency stop command sent to motor.")
    
    def _update_status(self, status):
        """
        Update the GUI with status information from Arduino.
        
        Args:
            status: Dictionary containing status information
        """
        try:
            # Update active axis display
            active_axis = status.get('active_axis', 'Unknown')
            self.active_axis_value.config(text=active_axis)
            
            # Update current axis in application state if different
            if active_axis in config.AXES and active_axis != self.current_axis:
                self.current_axis = active_axis
                self._highlight_active_axis()
            
            # Update motor state
            if status.get('enabled', False):
                self.motor_state_value.config(text="Enabled", foreground=config.ENABLED_COLOR)
            else:
                self.motor_state_value.config(text="Disabled", foreground=config.DISABLED_COLOR)
            
            # Update speed display
            speed = status.get('speed', 0)
            self.current_speed_value.config(text=f"{speed} steps/sec")
            
            # Update direction display
            direction = status.get('direction', config.DIRECTION_FORWARD)
            direction_label = status.get('direction_label', config.DIRECTION_LABELS[config.DIRECTION_FORWARD])
            self.current_direction_value.config(text=direction_label)
            
            # Update position display for all axes
            # In a real application, you'd track position per axis
            # Here we're just showing the current position for simplicity
            position = status.get('position', 0)
            for axis in config.AXES:
                if axis == active_axis:
                    self.position_values[axis].config(text=str(position))
            
            # Update input values
            axis_values = status.get('axis_values', {})
            for axis in config.AXES:
                if axis in axis_values:
                    value = axis_values[axis]
                    self.value_bars[axis]["value"] = value
        
        except Exception as e:
            logger.error(f"Error updating status: {str(e)}")
    
    def _highlight_active_axis(self):
        """Update GUI to highlight the active axis."""
        for axis in config.AXES:
            if axis == self.current_axis:
                # Add visual indicator for active axis
                self.axis_frames[axis].configure(style=f"{axis}Active.TLabelframe")
                self.style.configure(f"{axis}Active.TLabelframe", 
                                  background=config.AXIS_COLORS[axis])
                self.style.configure(f"{axis}Active.TLabelframe.Label", 
                                  foreground=config.AXIS_COLORS[axis],
                                  font=("Arial", 12, "bold"))
            else:
                # Reset non-active axes
                self.axis_frames[axis].configure(style=f"{axis}.TLabelframe")
    
    def _schedule_status_refresh(self):
        """Schedule regular updates of the GUI status."""
        # This is a simple way to update the GUI at regular intervals
        # without blocking the main thread or using a separate thread
        self.root.after(100, self._schedule_status_refresh)
    
    def _on_closing(self):
        """Handle application shutdown."""
        if self.connected:
            # Ask user to confirm if still connected
            if messagebox.askyesno("Quit", "Are you sure you want to quit? This will disconnect from the motor."):
                # Disconnect and close
                self.serial_handler.disconnect()
                self.root.destroy()
        else:
            # Just close if not connected
            self.root.destroy()


def main():
    """Main entry point for the application."""
    try:
        # Create the root window
        root = tk.Tk()
        
        # Create the motor control GUI
        app = MotorControlGUI(root)
        
        # Start the main event loop
        root.mainloop()
    
    except Exception as e:
        logging.error(f"Error starting application: {str(e)}")
        messagebox.showerror("Application Error", 
                           f"An error occurred starting the application:\n{str(e)}")
    finally:
        # Ensure everything is cleaned up
        logging.info("Application shutting down")


if __name__ == "__main__":
    main()
