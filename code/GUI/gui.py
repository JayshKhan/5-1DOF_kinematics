"""
ROT3U Robotic Arm - GUI Application

This file contains the main GUI application for controlling the ROT3U robotic arm.
The GUI provides three main tabs:
- Forward Kinematics: Enter joint angles and see end-effector position
- Inverse Kinematics: Enter target position and get joint angles
- Settings: Configure communication and control parameters

The GUI uses CustomTkinter for modern styling and provides real-time visualization
of the robot arm configuration.

Author: Jaysh Khan
"""

import tkinter as tk
from tkinter import ttk, messagebox
from typing import List, Optional
import math

import customtkinter as ctk
from kinematics.Kinematics import Kinematic
from . import functions

# Set GUI theme
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("green")

# Color constants
COLOR_GREEN = "#26d663"
COLOR_RED = "#dd0202"
COLOR_BLUE = "#2196F3"


class ROT3UControlApp(ctk.CTk):
    """
    Main application class for the ROT3U robotic arm control GUI.
    
    This class provides a comprehensive interface for controlling the robotic arm
    including forward/inverse kinematics, trajectory planning, and hardware communication.
    """
    
    def __init__(self):
        super().__init__()
        
        # Initialize robot kinematics
        self.kinematic = Kinematic()
        
        # Communication settings
        self.COM_PORT = '/dev/ttyACM0'
        self.BAUD_RATE = 9600
        self.arduino = None
        self.servo_mode = "Sequential"
        
        # Current joint angles in degrees
        self.current_angles = [0, 0, 0, 0, 0, 0]
        
        # GUI elements
        self.notebook = None
        self.matplotlib_plot = None
        self.matplotlib_canvas = None
        
        # Forward kinematics elements
        self.angle_entries = []
        self.angle_labels = []
        self.fk_position_label = None
        
        # Inverse kinematics elements
        self.x_entry = None
        self.y_entry = None
        self.z_entry = None
        self.phi_entry = None
        self.ik_angles_label = None
        self.ik_verify_label = None
        
        # Settings elements
        self.port_entry = None
        self.baud_entry = None
        self.servo_mode_entry = None
        
        self._setup_gui()
        self._setup_styles()
        
    def _setup_gui(self):
        """Initialize the main GUI components."""
        self.title("ROT3U Robotic Arm Control")
        self.geometry("1000x700")
        
        # Create main notebook for tabs
        self.notebook = ttk.Notebook(self, style="primary.TNotebook")
        
        # Create tabs
        self._create_forward_kinematics_tab()
        self._create_inverse_kinematics_tab()
        self._create_settings_tab()
        
        # Create connection button
        self._create_connection_button()
        
        self.notebook.pack(expand=True, fill="both", padx=10, pady=10)
        
    def _setup_styles(self):
        """Configure the GUI styling."""
        style = ttk.Style()
        
        # Create custom theme
        style.theme_create(themename="rot3u", parent="alt", settings={
            "TNotebook": {
                "configure": {
                    "tabmargins": [2, 5, 2, 0],
                    "background": COLOR_GREEN,
                    "foreground": "black"
                }
            },
            "TNotebook.Tab": {
                "configure": {"padding": [10, 5]},
                "map": {
                    "background": [("selected", COLOR_GREEN)],
                    "expand": [("selected", [1, 1, 1, 0])]
                }
            }
        })
        
        style.theme_use(themename="rot3u")
        style.configure(
            style="primary.TNotebook",
            tabposition="n",
            background="grey",
            foreground="black",
            borderwidth=2
        )
        
    def _create_forward_kinematics_tab(self):
        """Create the forward kinematics tab."""
        fk_frame = tk.Frame(self.notebook)
        self.notebook.add(fk_frame, text="Forward Kinematics")
        
        # Configure grid weights
        fk_frame.grid_columnconfigure(0, weight=1)
        fk_frame.grid_columnconfigure(1, weight=2)
        fk_frame.grid_rowconfigure(0, weight=1)
        
        # Left panel for controls
        left_panel = ctk.CTkFrame(fk_frame)
        left_panel.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        # Right panel for visualization
        right_panel = ctk.CTkFrame(fk_frame)
        right_panel.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        
        # Create joint angle inputs
        self._create_joint_angle_inputs(left_panel)
        
        # Create position display
        self._create_position_display(right_panel)
        
    def _create_joint_angle_inputs(self, parent):
        """Create input fields for joint angles."""
        title = ctk.CTkLabel(parent, text="Joint Angles (degrees)", 
                           font=ctk.CTkFont(size=16, weight="bold"))
        title.pack(pady=10)
        
        # Joint names
        joint_names = ["Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"]
        
        self.angle_entries = []
        self.angle_labels = []
        
        for i, name in enumerate(joint_names):
            frame = ctk.CTkFrame(parent)
            frame.pack(fill="x", padx=10, pady=5)
            
            label = ctk.CTkLabel(frame, text=f"{name}:", width=100)
            label.pack(side="left", padx=5)
            
            # Get joint limits
            limits = self.kinematic.joint_limits.get(i, (0, 180))
            
            entry = ctk.CTkEntry(frame, width=100, 
                               validate="key",
                               validatecommand=(self.register(self._validate_angle), '%P'))
            entry.insert(0, str(self.current_angles[i]))
            entry.pack(side="left", padx=5)
            
            limit_label = ctk.CTkLabel(frame, text=f"({limits[0]}°-{limits[1]}°)", 
                                     text_color="gray")
            limit_label.pack(side="left", padx=5)
            
            self.angle_entries.append(entry)
            self.angle_labels.append(label)
            
        # Submit button
        submit_btn = ctk.CTkButton(parent, text="Calculate Forward Kinematics",
                                 command=self._calculate_forward_kinematics,
                                 width=200, height=40)
        submit_btn.pack(pady=20)
        
    def _create_position_display(self, parent):
        """Create display for end-effector position."""
        title = ctk.CTkLabel(parent, text="End-Effector Position", 
                           font=ctk.CTkFont(size=16, weight="bold"))
        title.pack(pady=10)
        
        # Position display
        self.fk_position_label = ctk.CTkLabel(parent, text="X: 0.00 cm\nY: 0.00 cm\nZ: 0.00 cm",
                                            font=ctk.CTkFont(size=14))
        self.fk_position_label.pack(pady=20)
        
        # Workspace info
        workspace = self.kinematic.get_workspace_limits()
        workspace_text = f"Workspace Info:\nMax Reach: {workspace['max_reach']:.1f} cm\nMin Reach: {workspace['min_reach']:.1f} cm"
        
        workspace_label = ctk.CTkLabel(parent, text=workspace_text, 
                                     font=ctk.CTkFont(size=12), 
                                     text_color="gray")
        workspace_label.pack(pady=10)
        
    def _create_inverse_kinematics_tab(self):
        """Create the inverse kinematics tab."""
        ik_frame = tk.Frame(self.notebook)
        self.notebook.add(ik_frame, text="Inverse Kinematics")
        
        # Configure grid weights
        ik_frame.grid_columnconfigure(0, weight=1)
        ik_frame.grid_columnconfigure(1, weight=2)
        ik_frame.grid_rowconfigure(0, weight=1)
        
        # Left panel for controls
        left_panel = ctk.CTkFrame(ik_frame)
        left_panel.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        # Right panel for visualization
        right_panel = ctk.CTkFrame(ik_frame)
        right_panel.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        
        # Set up matplotlib plot frame
        self.matplotlib_plot = right_panel
        
        # Create position inputs
        self._create_position_inputs(left_panel)
        
    def _create_position_inputs(self, parent):
        """Create input fields for target position."""
        title = ctk.CTkLabel(parent, text="Target Position", 
                           font=ctk.CTkFont(size=16, weight="bold"))
        title.pack(pady=10)
        
        # Position inputs
        coords = [("X", "x_entry"), ("Y", "y_entry"), ("Z", "z_entry")]
        
        for coord, attr in coords:
            frame = ctk.CTkFrame(parent)
            frame.pack(fill="x", padx=10, pady=5)
            
            label = ctk.CTkLabel(frame, text=f"{coord} (cm):", width=80)
            label.pack(side="left", padx=5)
            
            entry = ctk.CTkEntry(frame, width=100,
                               validate="key",
                               validatecommand=(self.register(self._validate_coordinate), '%P'))
            entry.insert(0, "0")
            entry.pack(side="left", padx=5)
            
            setattr(self, attr, entry)
            
        # Phi angle input
        phi_frame = ctk.CTkFrame(parent)
        phi_frame.pack(fill="x", padx=10, pady=5)
        
        phi_label = ctk.CTkLabel(phi_frame, text="Phi (°):", width=80)
        phi_label.pack(side="left", padx=5)
        
        self.phi_entry = ctk.CTkEntry(phi_frame, width=100,
                                    validate="key",
                                    validatecommand=(self.register(self._validate_angle), '%P'))
        self.phi_entry.insert(0, "0")
        self.phi_entry.pack(side="left", padx=5)
        
        # Calculate button
        calc_btn = ctk.CTkButton(parent, text="Calculate Inverse Kinematics",
                               command=self._calculate_inverse_kinematics,
                               width=200, height=40)
        calc_btn.pack(pady=20)
        
        # Results display
        self.ik_angles_label = ctk.CTkLabel(parent, text="Joint Angles:\n(Not calculated)", 
                                          font=ctk.CTkFont(size=12))
        self.ik_angles_label.pack(pady=10)
        
        self.ik_verify_label = ctk.CTkLabel(parent, text="Verification:\n(Not calculated)", 
                                          font=ctk.CTkFont(size=12), 
                                          text_color="gray")
        self.ik_verify_label.pack(pady=10)
        
    def _create_settings_tab(self):
        """Create the settings tab."""
        settings_frame = tk.Frame(self.notebook)
        self.notebook.add(settings_frame, text="Settings")
        
        main_frame = ctk.CTkFrame(settings_frame)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        title = ctk.CTkLabel(main_frame, text="Communication Settings", 
                           font=ctk.CTkFont(size=16, weight="bold"))
        title.pack(pady=10)
        
        # COM Port setting
        port_frame = ctk.CTkFrame(main_frame)
        port_frame.pack(fill="x", padx=10, pady=5)
        
        port_label = ctk.CTkLabel(port_frame, text="COM Port:", width=100)
        port_label.pack(side="left", padx=5)
        
        self.port_entry = ctk.CTkEntry(port_frame, width=150)
        self.port_entry.insert(0, self.COM_PORT)
        self.port_entry.pack(side="left", padx=5)
        
        # Baud Rate setting
        baud_frame = ctk.CTkFrame(main_frame)
        baud_frame.pack(fill="x", padx=10, pady=5)
        
        baud_label = ctk.CTkLabel(baud_frame, text="Baud Rate:", width=100)
        baud_label.pack(side="left", padx=5)
        
        self.baud_entry = ctk.CTkEntry(baud_frame, width=150)
        self.baud_entry.insert(0, str(self.BAUD_RATE))
        self.baud_entry.pack(side="left", padx=5)
        
        # Servo Mode setting
        mode_frame = ctk.CTkFrame(main_frame)
        mode_frame.pack(fill="x", padx=10, pady=5)
        
        mode_label = ctk.CTkLabel(mode_frame, text="Servo Mode:", width=100)
        mode_label.pack(side="left", padx=5)
        
        self.servo_mode_entry = ctk.CTkOptionMenu(mode_frame, 
                                                values=["Sequential", "Simultaneous"],
                                                width=150)
        self.servo_mode_entry.set(self.servo_mode)
        self.servo_mode_entry.pack(side="left", padx=5)
        
        # Save button
        save_btn = ctk.CTkButton(main_frame, text="Save Settings",
                               command=self._save_settings,
                               width=200, height=40)
        save_btn.pack(pady=20)
        
    def _create_connection_button(self):
        """Create the connection button at the bottom."""
        conn_btn = ctk.CTkButton(self, text="Connect to Arduino",
                               command=self._connect_arduino,
                               width=200, height=40)
        conn_btn.pack(side="bottom", pady=10)
        
    def _validate_angle(self, value: str) -> bool:
        """Validate angle input (0-180 degrees)."""
        if value == "" or value == "-":
            return True
        try:
            angle = float(value)
            return 0 <= angle <= 180
        except ValueError:
            return False
            
    def _validate_coordinate(self, value: str) -> bool:
        """Validate coordinate input (any float)."""
        if value == "" or value == "-":
            return True
        try:
            float(value)
            return True
        except ValueError:
            return False
            
    def _calculate_forward_kinematics(self):
        """Calculate and display forward kinematics."""
        try:
            # Get joint angles
            angles = []
            for entry in self.angle_entries:
                angle = float(entry.get() or 0)
                angles.append(math.radians(angle))  # Convert to radians
                
            # Validate angles
            angles_deg = [math.degrees(a) for a in angles]
            if not self.kinematic.validate_joint_angles(angles_deg):
                messagebox.showerror("Error", "Joint angles are outside valid ranges!")
                return
                
            # Calculate forward kinematics
            transform = self.kinematic.forward_kinematics(angles)
            x, y, z = transform[0, 3], transform[1, 3], transform[2, 3]
            
            # Update display
            self.fk_position_label.configure(text=f"X: {x:.2f} cm\nY: {y:.2f} cm\nZ: {z:.2f} cm")
            
            # Update current angles
            self.current_angles = angles_deg
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to calculate forward kinematics: {str(e)}")
            
    def _calculate_inverse_kinematics(self):
        """Calculate and display inverse kinematics."""
        try:
            # Get target position
            x = float(self.x_entry.get() or 0)
            y = float(self.y_entry.get() or 0) 
            z = float(self.z_entry.get() or 0)
            
            target_position = [x, y, z]
            
            # Check if position is reachable
            if not self.kinematic.is_position_reachable(target_position):
                messagebox.showwarning("Warning", "Target position may be outside workspace!")
                
            # Calculate inverse kinematics
            solutions = self.kinematic.inverse_kinematics(target_position)
            
            if not solutions:
                messagebox.showerror("Error", "No solution found for target position!")
                return
                
            # Find best solution
            best_solution = self.kinematic.find_best_solution(solutions, self.current_angles)
            
            if best_solution is None:
                messagebox.showerror("Error", "No valid solution within joint limits!")
                return
                
            # Display solution
            angles_text = "Joint Angles:\n"
            joint_names = ["Base", "Shoulder", "Elbow", "Wrist"]
            for i, (name, angle) in enumerate(zip(joint_names, best_solution)):
                angles_text += f"{name}: {angle:.1f}°\n"
                
            self.ik_angles_label.configure(text=angles_text)
            
            # Verify solution using forward kinematics
            angles_rad = [math.radians(a) for a in ([0] + best_solution + [0])]
            verify_transform = self.kinematic.forward_kinematics(angles_rad)
            vx, vy, vz = verify_transform[0, 3], verify_transform[1, 3], verify_transform[2, 3]
            
            verify_text = f"Verification:\nX: {vx:.2f} cm\nY: {vy:.2f} cm\nZ: {vz:.2f} cm"
            self.ik_verify_label.configure(text=verify_text)
            
            # Plot robot configuration
            self.kinematic.plot_robot_arm(solutions, self)
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to calculate inverse kinematics: {str(e)}")
            
    def _save_settings(self):
        """Save communication settings."""
        try:
            self.COM_PORT = self.port_entry.get()
            self.BAUD_RATE = int(self.baud_entry.get())
            self.servo_mode = self.servo_mode_entry.get()
            messagebox.showinfo("Success", "Settings saved successfully!")
        except ValueError:
            messagebox.showerror("Error", "Invalid baud rate! Please enter a number.")
            
    def _connect_arduino(self):
        """Connect to Arduino."""
        try:
            functions.open_port(self)
            messagebox.showinfo("Success", "Connected to Arduino successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to connect to Arduino: {str(e)}")


# For backward compatibility
App = ROT3UControlApp


if __name__ == "__main__":
    app = ROT3UControlApp()
    app.mainloop()
