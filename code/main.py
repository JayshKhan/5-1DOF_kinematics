"""
ROT3U Robotic Arm - Main Application Entry Point

This is the main entry point for the ROT3U robotic arm control application.
It initializes the GUI and handles the main application loop.

Usage:
    python main.py

Author: Jaysh Khan
"""

import sys
import os
import traceback
from tkinter import messagebox

# Add the current directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from GUI.gui import ROT3UControlApp
    from GUI import functions
except ImportError as e:
    print(f"Import error: {e}")
    print("Please make sure all dependencies are installed.")
    print("Run: pip install -r requirements.txt")
    sys.exit(1)


def main():
    """
    Main application function.
    
    Initializes and runs the ROT3U control application.
    """
    try:
        # Create and configure the application
        app = ROT3UControlApp()
        
        # Set up proper window closing behavior
        app.protocol("WM_DELETE_WINDOW", lambda: cleanup_and_exit(app))
        
        print("ROT3U Robotic Arm Control Application Started")
        print("GUI initialized successfully")
        
        # Start the main event loop
        app.mainloop()
        
    except Exception as e:
        error_msg = f"Critical error starting application: {str(e)}\n\n{traceback.format_exc()}"
        print(error_msg)
        
        # Try to show error dialog if possible
        try:
            messagebox.showerror("Application Error", 
                               f"Failed to start ROT3U application:\n{str(e)}\n\n"
                               "Check the console for detailed error information.")
        except:
            pass
        
        sys.exit(1)


def cleanup_and_exit(app):
    """
    Clean up resources and exit the application safely.
    
    Args:
        app: The application instance
    """
    try:
        print("Shutting down application...")
        
        # Close Arduino connection if open
        functions.close_arduino_connection(app)
        
        # Destroy the application window
        app.destroy()
        
    except Exception as e:
        print(f"Error during cleanup: {e}")
    
    finally:
        print("Application closed")
        sys.exit(0)


def check_dependencies():
    """
    Check if all required dependencies are available.
    
    Returns:
        bool: True if all dependencies are available, False otherwise
    """
    required_modules = [
        'numpy',
        'matplotlib',
        'customtkinter',
        'serial'
    ]
    
    missing_modules = []
    
    for module in required_modules:
        try:
            __import__(module)
        except ImportError:
            missing_modules.append(module)
    
    if missing_modules:
        print("Missing required dependencies:")
        for module in missing_modules:
            print(f"  - {module}")
        print("\nPlease install missing dependencies:")
        print("pip install -r requirements.txt")
        return False
    
    return True


def show_help():
    """Display help information."""
    help_text = """
ROT3U Robotic Arm Control Application

USAGE:
    python main.py              Start the GUI application
    python main.py --help       Show this help message

FEATURES:
    - Forward Kinematics: Calculate end-effector position from joint angles
    - Inverse Kinematics: Calculate joint angles from target position
    - Real-time robot arm visualization
    - Arduino communication for hardware control
    - Trajectory planning and execution

REQUIREMENTS:
    - Python 3.8 or higher
    - All dependencies listed in requirements.txt
    - Arduino with compatible firmware (optional)

CONFIGURATION:
    - Set COM port and baud rate in the Settings tab
    - Default: /dev/ttyACM0 at 9600 baud (Linux)
    - Windows users should use COM ports (e.g., COM3)

For more information, see the README.md file.
"""
    print(help_text)


if __name__ == "__main__":
    # Handle command line arguments
    if len(sys.argv) > 1:
        if sys.argv[1] in ['--help', '-h']:
            show_help()
            sys.exit(0)
        else:
            print(f"Unknown argument: {sys.argv[1]}")
            print("Use --help for usage information")
            sys.exit(1)
    
    # Check dependencies before starting
    if not check_dependencies():
        sys.exit(1)
    
    # Start the application
    main()
