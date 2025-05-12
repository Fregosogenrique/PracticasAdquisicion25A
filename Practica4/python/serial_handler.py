"""
Serial communication handler for motor control system.
Manages serial connection with Arduino and processes commands/status updates.
"""

import serial
import serial.tools.list_ports
import threading
import time
import queue
import logging
from typing import Callable, Dict, List, Optional, Tuple, Union
import config

# Configure logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("SerialHandler")

class SerialHandler:
    """
    Handles serial communication with the Arduino motor controller.
    Provides methods for sending commands and processing status updates.
    """
    
    def __init__(self):
        """Initialize the serial handler with default values."""
        self.serial_port = None
        self.is_connected = False
        self.port_name = None
        self.status_callbacks = []
        self.connection_callbacks = []
        self.read_thread = None
        self.stop_thread = False
        self.command_queue = queue.Queue()
        self.command_thread = None
        self.last_status = {}
        
    def list_ports(self) -> List[str]:
        """
        Get a list of available serial ports.
        
        Returns:
            List of port names as strings
        """
        ports = list(serial.tools.list_ports.comports())
        return [port.device for port in ports]
    
    def connect(self, port_name: str) -> bool:
        """
        Connect to the specified serial port.
        
        Args:
            port_name: Name of the serial port to connect to
            
        Returns:
            True if connection successful, False otherwise
        """
        if self.is_connected:
            self.disconnect()
            
        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=config.BAUD_RATE,
                timeout=config.TIMEOUT
            )
            
            # Wait for Arduino to reset after connection
            time.sleep(2)
            
            self.is_connected = True
            self.port_name = port_name
            
            # Start read thread
            self.stop_thread = False
            self.read_thread = threading.Thread(target=self._read_serial)
            self.read_thread.daemon = True
            self.read_thread.start()
            
            # Start command thread
            self.command_thread = threading.Thread(target=self._process_commands)
            self.command_thread.daemon = True
            self.command_thread.start()
            
            # Notify connection callbacks
            self._notify_connection_callbacks(True)
            
            logger.info(f"Connected to {port_name}")
            return True
            
        except serial.SerialException as e:
            logger.error(f"Failed to connect to {port_name}: {str(e)}")
            self._notify_connection_callbacks(False)
            return False
    
    def disconnect(self) -> None:
        """Disconnect from the serial port."""
        if self.is_connected and self.serial_port:
            # Stop threads
            self.stop_thread = True
            
            # Wait for threads to finish
            if self.read_thread and self.read_thread.is_alive():
                self.read_thread.join(timeout=1.0)
            
            if self.command_thread and self.command_thread.is_alive():
                self.command_thread.join(timeout=1.0)
            
            # Close the port
            try:
                self.serial_port.close()
            except serial.SerialException as e:
                logger.error(f"Error closing port: {str(e)}")
            
            self.is_connected = False
            self.port_name = None
            self.serial_port = None
            
            # Notify connection callbacks
            self._notify_connection_callbacks(False)
            
            logger.info("Disconnected from serial port")
    
    def send_axis_command(self, axis: str, speed: int, direction: int) -> None:
        """
        Send an axis control command to the Arduino.
        
        Args:
            axis: Axis identifier ('X', 'Y', or 'R')
            speed: Speed in steps per second
            direction: Direction (0 for reverse, 1 for forward)
        """
        if not self.is_connected:
            logger.warning("Cannot send command: Not connected")
            return
        
        # Validate parameters
        if axis not in config.AXES:
            logger.error(f"Invalid axis: {axis}")
            return
        
        if not (config.MIN_SPEED <= speed <= config.MAX_SPEED):
            speed = max(min(speed, config.MAX_SPEED), config.MIN_SPEED)
            logger.warning(f"Speed adjusted to limits: {speed}")
        
        if direction not in [config.DIRECTION_FORWARD, config.DIRECTION_REVERSE]:
            logger.error(f"Invalid direction: {direction}")
            return
        
        # Format command
        command = config.COMMAND_FORMAT.format(
            axis=axis,
            speed=speed,
            direction=direction
        )
        
        # Queue the command for sending
        self.command_queue.put(command)
        logger.debug(f"Queued command: {command}")
    
    def send_stop_command(self) -> None:
        """Send emergency stop command to Arduino."""
        if not self.is_connected:
            logger.warning("Cannot send stop command: Not connected")
            return
        
        # Queue the stop command with highest priority
        # Clear existing commands
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
            except queue.Empty:
                break
        
        # Add stop command
        self.command_queue.put(config.STOP_COMMAND)
        logger.info("Emergency stop command sent")
    
    def register_status_callback(self, callback: Callable[[Dict], None]) -> None:
        """
        Register a callback function to be called when status updates are received.
        
        Args:
            callback: Function that takes a status dictionary parameter
        """
        if callback not in self.status_callbacks:
            self.status_callbacks.append(callback)
    
    def unregister_status_callback(self, callback: Callable[[Dict], None]) -> None:
        """
        Unregister a previously registered status callback function.
        
        Args:
            callback: The callback function to remove
        """
        if callback in self.status_callbacks:
            self.status_callbacks.remove(callback)
    
    def register_connection_callback(self, callback: Callable[[bool], None]) -> None:
        """
        Register a callback function to be called when connection status changes.
        
        Args:
            callback: Function that takes a boolean parameter (True if connected)
        """
        if callback not in self.connection_callbacks:
            self.connection_callbacks.append(callback)
    
    def unregister_connection_callback(self, callback: Callable[[bool], None]) -> None:
        """
        Unregister a previously registered connection callback function.
        
        Args:
            callback: The callback function to remove
        """
        if callback in self.connection_callbacks:
            self.connection_callbacks.remove(callback)
    
    def get_last_status(self) -> Dict:
        """
        Get the last received status information.
        
        Returns:
            Dictionary containing the last status update
        """
        return self.last_status
    
    def _read_serial(self) -> None:
        """
        Read data from the serial port and process it.
        This method runs in a separate thread.
        """
        if not self.serial_port:
            return
        
        buffer = ""
        
        while not self.stop_thread and self.is_connected:
            try:
                # Read data if available
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='replace')
                    buffer += data
                    
                    # Process complete lines
                    if '\n' in buffer:
                        lines = buffer.split('\n')
                        # Keep the last incomplete line in the buffer
                        buffer = lines[-1]
                        
                        # Process all complete lines
                        for line in lines[:-1]:
                            line = line.strip()
                            if line:
                                self._process_message(line)
                
                # Small delay to prevent high CPU usage
                time.sleep(0.01)
                
            except serial.SerialException as e:
                logger.error(f"Serial read error: {str(e)}")
                self.disconnect()
                break
            except Exception as e:
                logger.error(f"Unexpected error in read thread: {str(e)}")
    
    def _process_commands(self) -> None:
        """
        Process and send commands from the queue.
        This method runs in a separate thread.
        """
        while not self.stop_thread and self.is_connected:
            try:
                # Get command from queue with timeout
                try:
                    command = self.command_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                
                # Send command to Arduino
                if self.serial_port and self.is_connected:
                    try:
                        self.serial_port.write(f"{command}\n".encode('utf-8'))
                        self.serial_port.flush()
                        logger.debug(f"Sent command: {command}")
                    except serial.SerialException as e:
                        logger.error(f"Error sending command: {str(e)}")
                        self.disconnect()
                        break
                
                # Mark task as done
                self.command_queue.task_done()
                
                # Small delay between commands
                time.sleep(0.05)
                
            except Exception as e:
                logger.error(f"Unexpected error in command thread: {str(e)}")
    
    def _process_message(self, message: str) -> None:
        """
        Process a message received from the Arduino.
        
        Args:
            message: The message string to process
        """
        # Check if it's a status update
        if message.startswith(config.STATUS_PREFIX):
            self._parse_status_message(message)
        else:
            # Just log other messages
            logger.info(f"Arduino: {message}")
    
    def _parse_status_message(self, message: str) -> None:
        """
        Parse a status message from the Arduino and update status information.
        
        Args:
            message: The status message to parse
        """
        try:
            # Split the message into parts
            parts = message.split(',')
            
            if len(parts) < 9:
                logger.warning(f"Invalid status message format: {message}")
                return
            
            # Parse the status information
            active_axis = int(parts[config.STATUS_AXIS_INDEX])
            enabled = parts[config.STATUS_ENABLED_INDEX] == "ENABLED"
            direction = int(parts[config.STATUS_DIRECTION_INDEX])
            speed = int(parts[config.STATUS_SPEED_INDEX])
            position = int(parts[config.STATUS_POSITION_INDEX])
            x_value = int(parts[config.STATUS_X_VALUE_INDEX])
            y_value = int(parts[config.STATUS_Y_VALUE_INDEX])
            r_value = int(parts[config.STATUS_R_VALUE_INDEX])
            
            # Create status dictionary
            status = {
                'active_axis': config.AXES[active_axis] if 0 <= active_axis < len(config.AXES) else "Unknown",
                'enabled': enabled,
                'direction': direction,
                'direction_label': config.DIRECTION_LABELS.get(direction, "Unknown"),
                'speed': speed,
                'position': position,
                'axis_values': {
                    'X': x_value,
                    'Y': y_value,
                    'R': r_value
                }
            }
            
            # Update last status
            self.last_status = status
            
            # Notify callbacks
            self._notify_status_callbacks(status)
            
        except (ValueError, IndexError) as e:
            logger.error(f"Error parsing status message: {str(e)}")
    
    def _notify_status_callbacks(self, status: Dict) -> None:
        """
        Notify all registered status callbacks with the current status.
        
        Args:
            status: Dictionary containing status information
        """
        for callback in self.status_callbacks[:]:  # Use a copy of the list to avoid issues if callbacks modify the list
            try:
                callback(status)
            except Exception as e:
                logger.error(f"Error in status callback: {str(e)}")
    
    def _notify_connection_callbacks(self, connected: bool) -> None:
        """
        Notify all registered connection callbacks with the current connection status.
        
        Args:
            connected: True if connected, False otherwise
        """
        for callback in self.connection_callbacks[:]:  # Use a copy of the list
            try:
                callback(connected)
            except Exception as e:
                logger.error(f"Error in connection callback: {str(e)}")

