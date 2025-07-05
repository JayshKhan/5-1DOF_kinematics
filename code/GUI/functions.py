"""
ROT3U Robotic Arm - GUI Support Functions

This module contains utility functions for the ROT3U GUI application.
It provides functions for Arduino communication, validation, and data processing.

Author: Jaysh Khan
"""

import subprocess
import serial
from tkinter import messagebox
from typing import List, Optional


def open_port(app) -> bool:
    """
    Open serial communication port with Arduino.
    
    Args:
        app: GUI application instance
        
    Returns:
        bool: True if successful, False otherwise
    """
    # Grant permission to access the port (Linux specific)
    try:
        cmd = ['chmod', '777', f'{app.COM_PORT}']
        subprocess.check_output(['sudo', '-S'] + cmd, input='password\n', encoding='utf-8')
    except subprocess.CalledProcessError as e:
        print(f"Permission error: {e}")
        # Continue anyway, permissions might already be set
    
    # Attempt to open serial connection
    try:
        if app.arduino:
            app.arduino.close()
            
        app.arduino = serial.Serial(app.COM_PORT, app.BAUD_RATE, timeout=1)
        print(f"Connected to Arduino at {app.COM_PORT}")
        return True
        
    except serial.SerialException as e:
        print(f"Serial connection error: {e}")
        messagebox.showerror("Connection Error", 
                           f"Could not connect to Arduino at {app.COM_PORT}.\n"
                           f"Please check the connection and port settings.")
        return False


def send_to_arduino(data: str, app) -> bool:
    """
    Send data to Arduino via serial communication.
    
    Args:
        data: String data to send
        app: GUI application instance
        
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        if not app.arduino or not app.arduino.is_open:
            if not open_port(app):
                return False
        
        # Send data
        app.arduino.write(data.encode())
        app.arduino.flush()
        
        # Read response
        response = app.arduino.readline().decode().strip()
        print(f"Sent: {data.strip()}")
        print(f"Response: {response}")
        
        return True
        
    except serial.SerialException as e:
        print(f"Communication error: {e}")
        messagebox.showerror("Communication Error", 
                           f"Failed to send data to Arduino: {e}")
        return False


def send_joint_angles(angles: List[float], app) -> bool:
    """
    Send joint angles to Arduino in the expected format.
    
    Args:
        angles: List of joint angles in degrees
        app: GUI application instance
        
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        # Validate angles
        if not app.kinematic.validate_joint_angles(angles):
            messagebox.showerror("Invalid Angles", 
                               "Joint angles are outside valid ranges!")
            return False
        
        # Prepare data for Arduino
        # Format: angle1,angle2,angle3,angle4,angle5,angle6,mode
        servo_mode = app.servo_mode
        mode = "s" if servo_mode.startswith("Seq") else "c"
        
        # Round angles to avoid precision issues
        rounded_angles = [round(angle, 1) for angle in angles]
        
        # Create data string
        data = ",".join(str(angle) for angle in rounded_angles) + f",{mode}\n"
        
        return send_to_arduino(data, app)
        
    except Exception as e:
        print(f"Error preparing data: {e}")
        return False


def validate_workspace_position(x: float, y: float, z: float, app) -> bool:
    """
    Validate if a position is within the robot's workspace.
    
    Args:
        x, y, z: Target coordinates
        app: GUI application instance
        
    Returns:
        bool: True if position is valid, False otherwise
    """
    # Check table collision (z too low)
    if z < -10:
        messagebox.showerror("Collision Error", 
                           "Position is too low - robot would collide with table!")
        return False
    
    # Check workspace limits
    position = [x, y, z]
    if not app.kinematic.is_position_reachable(position):
        response = messagebox.askyesno("Workspace Warning", 
                                     "Target position may be outside robot workspace.\n"
                                     "Continue anyway?")
        return response
    
    return True


def format_angle_display(angles: List[float]) -> str:
    """
    Format joint angles for display in the GUI.
    
    Args:
        angles: List of joint angles in degrees
        
    Returns:
        str: Formatted string for display
    """
    joint_names = ["Base", "Shoulder", "Elbow", "Wrist", "Roll", "Gripper"]
    formatted_lines = []
    
    for i, (name, angle) in enumerate(zip(joint_names, angles)):
        if i < len(angles):
            formatted_lines.append(f"{name}: {angle:.1f}°")
    
    return "\n".join(formatted_lines)


def calculate_trajectory_time(angles_start: List[float], angles_end: List[float], 
                            max_speed: float = 30.0) -> float:
    """
    Calculate estimated time for trajectory execution.
    
    Args:
        angles_start: Starting joint angles
        angles_end: Ending joint angles
        max_speed: Maximum angular velocity in degrees/second
        
    Returns:
        float: Estimated time in seconds
    """
    max_angle_change = max(abs(end - start) 
                          for start, end in zip(angles_start, angles_end))
    
    return max_angle_change / max_speed


def emergency_stop(app) -> bool:
    """
    Send emergency stop command to Arduino.
    
    Args:
        app: GUI application instance
        
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        # Send stop command
        stop_command = "STOP\n"
        return send_to_arduino(stop_command, app)
        
    except Exception as e:
        print(f"Emergency stop error: {e}")
        return False


def get_arduino_status(app) -> Optional[str]:
    """
    Request and return Arduino status information.
    
    Args:
        app: GUI application instance
        
    Returns:
        Optional[str]: Status string or None if failed
    """
    try:
        if not app.arduino or not app.arduino.is_open:
            return None
        
        # Send status request
        app.arduino.write(b"STATUS\n")
        app.arduino.flush()
        
        # Read response
        response = app.arduino.readline().decode().strip()
        return response if response else None
        
    except Exception as e:
        print(f"Status request error: {e}")
        return None


def close_arduino_connection(app) -> bool:
    """
    Safely close Arduino connection.
    
    Args:
        app: GUI application instance
        
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        if app.arduino and app.arduino.is_open:
            app.arduino.close()
            print("Arduino connection closed")
        return True
        
    except Exception as e:
        print(f"Error closing connection: {e}")
        return False


def validate_serial_settings(port: str, baud_rate: int) -> bool:
    """
    Validate serial port settings.
    
    Args:
        port: Serial port name
        baud_rate: Baud rate
        
    Returns:
        bool: True if settings are valid, False otherwise
    """
    # Check port name format
    if not port or not isinstance(port, str):
        return False
    
    # Check baud rate
    valid_baud_rates = [9600, 19200, 38400, 57600, 115200]
    if baud_rate not in valid_baud_rates:
        return False
    
    return True


def create_angle_command(angles: List[float], move_type: str = "linear") -> str:
    """
    Create a formatted command string for Arduino.
    
    Args:
        angles: List of joint angles in degrees
        move_type: Type of movement ("linear", "joint", "arc")
        
    Returns:
        str: Formatted command string
    """
    # Round angles to avoid precision issues
    rounded_angles = [round(angle, 2) for angle in angles]
    
    # Create command based on move type
    if move_type == "linear":
        command = "MOVE_LINEAR:"
    elif move_type == "joint":
        command = "MOVE_JOINT:"
    elif move_type == "arc":
        command = "MOVE_ARC:"
    else:
        command = "MOVE:"
    
    # Add angles
    command += ",".join(str(angle) for angle in rounded_angles)
    command += "\n"
    
    return command


# Utility functions for backward compatibility
def text_to_speech(text: str) -> None:
    """
    Text-to-speech function (placeholder).
    
    Args:
        text: Text to speak
    """
    # Placeholder for text-to-speech functionality
    print(f"TTS: {text}")


def log_action(action: str, details: str = "") -> None:
    """
    Log an action for debugging purposes.
    
    Args:
        action: Action description
        details: Additional details
    """
    print(f"[LOG] {action}: {details}")

