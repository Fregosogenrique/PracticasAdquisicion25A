"""
Configuration settings for the motor control system.
Contains constants for serial communication, GUI layout, and motor parameters.
"""

# Serial Communication Settings
BAUD_RATE = 115200
TIMEOUT = 0.1  # seconds
UPDATE_INTERVAL = 50  # milliseconds - how often to check for serial data
STATUS_INTERVAL = 500  # milliseconds - how often Arduino sends status updates

# Command Protocol Definitions
COMMAND_FORMAT = "{axis},{speed},{direction}"  # Format for commands sent to Arduino
STOP_COMMAND = "STOP"  # Emergency stop command
STATUS_PREFIX = "STATUS"  # Prefix for status messages from Arduino

# GUI Constants
WINDOW_TITLE = "Motor Control System"
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
BACKGROUND_COLOR = "#f0f0f0"
LABEL_WIDTH = 10
ENTRY_WIDTH = 10
BUTTON_WIDTH = 15
PADDING = 10

# GUI Color Definitions
ENABLED_COLOR = "#4CAF50"  # Green
DISABLED_COLOR = "#F44336"  # Red
WARNING_COLOR = "#FF9800"  # Orange
HEADER_COLOR = "#2196F3"  # Blue
TEXT_COLOR = "#000000"  # Black

# Axis Configuration
AXES = ["X", "Y", "R"]
AXIS_COLORS = {
    "X": "#E91E63",  # Pink
    "Y": "#2196F3",  # Blue
    "R": "#FF9800",  # Orange
}

# Motor Parameters
MIN_SPEED = 100  # Minimum speed in steps per second
MAX_SPEED = 3000  # Maximum speed in steps per second
DEFAULT_SPEED = 500  # Default speed in steps per second

# Direction Constants
DIRECTION_FORWARD = 1
DIRECTION_REVERSE = 0
DIRECTION_LABELS = {
    DIRECTION_FORWARD: "Forward",
    DIRECTION_REVERSE: "Reverse"
}

# Status Message Indices
STATUS_AXIS_INDEX = 1
STATUS_ENABLED_INDEX = 2
STATUS_DIRECTION_INDEX = 3
STATUS_SPEED_INDEX = 4
STATUS_POSITION_INDEX = 5
STATUS_X_VALUE_INDEX = 6
STATUS_Y_VALUE_INDEX = 7
STATUS_R_VALUE_INDEX = 8

