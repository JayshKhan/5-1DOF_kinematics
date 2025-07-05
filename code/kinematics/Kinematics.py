"""
ROT3U Robotic Arm - Kinematics Module

This module contains the Kinematic class which handles forward and inverse kinematics
calculations for the ROT3U robotic arm. It provides methods for calculating end-effector
position from joint angles (forward kinematics) and joint angles from end-effector
position (inverse kinematics).

The robot arm has 5 degrees of freedom plus a gripper:
- Joint 0: Base rotation (around Z-axis)
- Joint 1: Shoulder pitch
- Joint 2: Elbow pitch  
- Joint 3: Wrist pitch
- Joint 4: Wrist roll
- Joint 5: Gripper

Author: Jaysh Khan
"""

import math
from typing import List, Tuple, Optional, Union
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class Kinematic:
    """
    Handles forward and inverse kinematics for the ROT3U robotic arm.
    
    The robot arm uses Denavit-Hartenberg (DH) parameters for kinematic calculations.
    Link lengths are specified in centimeters.
    """
    
    def __init__(self, phi_resolution: float = 1.0):
        """
        Initialize the Kinematic class.
        
        Args:
            phi_resolution (float): Resolution for phi angle calculations in degrees
        """
        # Robot arm dimensions in cm
        self.a1 = 10.5  # Shoulder to elbow length
        self.a2 = 10.0  # Elbow to wrist length  
        self.a3 = 16.8  # Wrist to end effector length
        
        # Current target coordinates
        self.coord = [0.0, 0.0, 0.0]
        
        # Phi angle resolution for inverse kinematics
        self.phi_resolution = phi_resolution
        self.phi_deg_values = [0]  # Currently only using phi = 0
        
        # Denavit-Hartenberg parameters
        self.dh_params = [
            {'alpha': 0, 'a': 0, 'd': 0},                    # Base joint
            {'alpha': math.pi / 2, 'a': 0, 'd': 0},          # Shoulder joint
            {'alpha': math.pi, 'a': self.a1, 'd': 0},        # Elbow joint
            {'alpha': 0, 'a': self.a2, 'd': 0},              # Wrist joint
            {'alpha': 0, 'a': 0, 'd': 0},                    # End effector
        ]
        
        # Joint limits in degrees
        self.joint_limits = {
            0: (0, 180),    # Base rotation
            1: (0, 180),    # Shoulder pitch
            2: (0, 180),    # Elbow pitch
            3: (0, 150),    # Wrist pitch
            4: (0, 180),    # Wrist roll
            5: (0, 180),    # Gripper
        }

    def _generate_transformation_matrix(self, theta: float, joint_index: int) -> np.ndarray:
        """
        Generate a 4x4 transformation matrix using DH parameters.
        
        Args:
            theta (float): Joint angle in radians
            joint_index (int): Index of the joint (0-4)
            
        Returns:
            np.ndarray: 4x4 transformation matrix
        """
        if joint_index >= len(self.dh_params):
            raise ValueError(f"Joint index {joint_index} out of range")
            
        dh = self.dh_params[joint_index]
        
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(dh['alpha'])
        sin_alpha = np.sin(dh['alpha'])
        
        return np.array([
            [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, dh['a'] * cos_theta],
            [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, dh['a'] * sin_theta],
            [0, sin_alpha, cos_alpha, dh['d']],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joint_angles: List[float]) -> np.ndarray:
        """
        Calculate forward kinematics to get end-effector pose from joint angles.
        
        Args:
            joint_angles (List[float]): List of 6 joint angles in radians
                                       [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
            
        Returns:
            np.ndarray: 4x4 transformation matrix representing end-effector pose
            
        Raises:
            ValueError: If joint_angles doesn't have exactly 6 elements
        """
        if len(joint_angles) != 6:
            raise ValueError("joint_angles must contain exactly 6 elements")
            
        # Extract first 4 joint angles (base rotation handled separately)
        theta0, theta1, theta2, theta3, theta4, theta5 = joint_angles
        
        # Generate transformation matrices for each joint
        t0_1 = self._generate_transformation_matrix(theta1, 1)
        t1_2 = self._generate_transformation_matrix(theta2, 2)
        t2_3 = self._generate_transformation_matrix(theta3, 3)
        t3_4 = self._generate_transformation_matrix(theta4, 4)
        
        # End effector transformation (wrist to end effector)
        t_end = np.array([
            [1, 0, 0, 0],
            [0, np.cos(np.pi / 2), -np.sin(np.pi / 2), -self.a3],
            [0, np.sin(np.pi / 2), np.cos(np.pi / 2), 0],
            [0, 0, 0, 1]
        ])
        
        # Calculate total transformation from base to end-effector
        t_total = t0_1 @ t1_2 @ t2_3 @ t3_4 @ t_end
        
        return t_total

    def inverse_kinematics(self, target_position: List[float]) -> List[List[float]]:
        """
        Calculate inverse kinematics to get joint angles from end-effector position.
        
        Args:
            target_position (List[float]): Target [x, y, z] coordinates in cm
            
        Returns:
            List[List[float]]: List of possible joint angle solutions in radians
        """
        px, py, pz = target_position
        self.coord = target_position
        
        solutions = []
        
        # Calculate base rotation angle
        theta0 = math.atan2(py, px)
        
        # Project to 2D (remove base rotation effect)
        px_projected = px * math.cos(theta0) + py * math.sin(theta0)
        px = px_projected
        
        # Iterate through different phi values (end-effector orientation)
        for phi_deg in self.phi_deg_values:
            phi = math.radians(phi_deg)
            
            # Calculate wrist center position
            wx = px - (self.a3 * np.cos(phi))
            wz = pz - (self.a3 * np.sin(phi))
            
            # Calculate elbow angle (theta2)
            cos_theta2 = (wx ** 2 + wz ** 2 - self.a1 ** 2 - self.a2 ** 2) / (2 * self.a1 * self.a2)
            
            # Check if solution exists
            if abs(cos_theta2) <= 1:
                # Two possible elbow configurations
                sin_theta2_1 = np.sqrt(1 - cos_theta2 ** 2)
                sin_theta2_2 = -np.sqrt(1 - cos_theta2 ** 2)
                
                theta2_1 = np.arctan2(sin_theta2_1, cos_theta2)
                theta2_2 = np.arctan2(sin_theta2_2, cos_theta2)
                
                # Calculate shoulder angles for both elbow configurations
                for theta2 in [theta2_1, theta2_2]:
                    denom = self.a1 ** 2 + self.a2 ** 2 + 2 * self.a1 * self.a2 * np.cos(theta2)
                    
                    if denom > 0:
                        sin_theta1 = (wz * (self.a1 + self.a2 * np.cos(theta2)) - 
                                     self.a2 * np.sin(theta2) * wx) / denom
                        cos_theta1 = (wx * (self.a1 + self.a2 * np.cos(theta2)) + 
                                     self.a2 * np.sin(theta2) * wz) / denom
                        
                        theta1 = np.arctan2(sin_theta1, cos_theta1)
                        
                        # Calculate wrist angle
                        theta3 = phi - theta1 - theta2
                        
                        # Create solution [base, shoulder, elbow, wrist]
                        solution = [theta0, theta1, -theta2, -theta3 + (math.pi / 2)]
                        solutions.append(solution)
        
        return solutions

    def get_end_effector_position(self, joint_angles: List[float]) -> Tuple[float, float, float]:
        """
        Get the end-effector position from joint angles.
        
        Args:
            joint_angles (List[float]): Joint angles in radians
            
        Returns:
            Tuple[float, float, float]: End-effector position (x, y, z) in cm
        """
        transform = self.forward_kinematics(joint_angles)
        return transform[0, 3], transform[1, 3], transform[2, 3]

    def validate_joint_angles(self, joint_angles: List[float]) -> bool:
        """
        Validate that all joint angles are within their limits.
        
        Args:
            joint_angles (List[float]): Joint angles in degrees
            
        Returns:
            bool: True if all angles are valid, False otherwise
        """
        for i, angle in enumerate(joint_angles[:6]):  # Check first 6 joints
            if i in self.joint_limits:
                min_angle, max_angle = self.joint_limits[i]
                if angle < min_angle or angle > max_angle:
                    return False
        return True

    def find_best_solution(self, solutions: List[List[float]], 
                          current_angles: List[float]) -> Optional[List[float]]:
        """
        Find the best inverse kinematics solution based on proximity to current angles.
        
        Args:
            solutions (List[List[float]]): List of possible solutions
            current_angles (List[float]): Current joint angles in radians
            
        Returns:
            Optional[List[float]]: Best solution in degrees, or None if no valid solution
        """
        valid_solutions = []
        
        for solution in solutions:
            # Convert to degrees
            solution_degrees = [math.degrees(angle) for angle in solution]
            
            # Add gripper angles (set to 0 for now)
            full_solution = [0] + solution_degrees + [0]
            
            # Check if solution is within joint limits
            if self.validate_joint_angles(full_solution):
                valid_solutions.append(solution_degrees)
        
        if not valid_solutions:
            return None
        
        # Find solution closest to current angles
        if len(current_angles) >= 4:
            current_subset = current_angles[1:5]  # Compare only the main 4 joints
            min_distance = float('inf')
            best_solution = None
            
            for solution in valid_solutions:
                distance = sum(abs(a - b) for a, b in zip(solution, current_subset))
                if distance < min_distance:
                    min_distance = distance
                    best_solution = solution
            
            return best_solution
        else:
            return valid_solutions[0]

    def plot_robot_arm(self, solutions: List[List[float]], app) -> None:
        """
        Plot the robot arm configuration in 2D.
        
        Args:
            solutions (List[List[float]]): List of joint angle solutions
            app: GUI application object containing matplotlib canvas
        """
        if not solutions:
            print("No solutions to plot")
            return
            
        px, py, pz = self.coord
        theta0 = math.atan2(py, px)
        px_rotated = px * math.cos(theta0) + py * math.sin(theta0)
        
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111)
        ax.clear()
        
        # Plot each solution
        colors = ['red', 'blue', 'green', 'orange']
        for i, solution in enumerate(solutions[:4]):  # Limit to 4 solutions
            color = colors[i % len(colors)]
            theta0, theta1, theta2, theta3 = solution
            
            # Calculate link positions
            # Joint 1 position
            x1 = self.a1 * np.cos(-theta1)
            z1 = self.a1 * np.sin(-theta1)
            
            # Joint 2 position
            x2 = x1 + self.a2 * np.cos(-theta1 + theta2)
            z2 = z1 + self.a2 * np.sin(-theta1 + theta2)
            
            # End effector position
            x3 = px_rotated
            z3 = pz
            
            # Plot links
            ax.plot([0, x1], [0, z1], color=color, linewidth=2, label=f'Link 1 (Sol {i+1})')
            ax.plot([x1, x2], [z1, z2], color=color, linewidth=2, linestyle='--', label=f'Link 2 (Sol {i+1})')
            ax.plot([x2, x3], [z2, z3], color=color, linewidth=2, linestyle=':', label=f'Link 3 (Sol {i+1})')
            
            # Plot joints
            ax.plot([0, x1, x2, x3], [0, z1, z2, z3], 'o', color=color, markersize=6)
        
        # Plot workspace and axes
        ax.axhline(y=0, color='black', linewidth=0.5)
        ax.axvline(x=0, color='black', linewidth=0.5)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Z (cm)')
        ax.set_title('ROT3U Robot Arm Configuration')
        ax.legend()
        ax.set_aspect('equal')
        
        # Update GUI canvas
        if hasattr(app, 'matplotlib_canvas') and app.matplotlib_canvas:
            app.matplotlib_canvas.get_tk_widget().destroy()
        
        if hasattr(app, 'matplotlib_plot'):
            app.matplotlib_canvas = FigureCanvasTkAgg(fig, master=app.matplotlib_plot)
            app.matplotlib_canvas.draw()
            app.matplotlib_canvas.get_tk_widget().pack(fill='both', expand=True)

    def update_phi_resolution(self, resolution: float) -> None:
        """
        Update the phi angle resolution for inverse kinematics.
        
        Args:
            resolution (float): New resolution in degrees
        """
        self.phi_resolution = resolution
        self.phi_deg_values = [0]  # Currently only using phi = 0
        
    def get_workspace_limits(self) -> dict:
        """
        Calculate the workspace limits of the robot arm.
        
        Returns:
            dict: Dictionary containing workspace limits
        """
        max_reach = self.a1 + self.a2 + self.a3
        min_reach = abs(self.a1 - self.a2) if self.a1 > self.a2 else 0
        
        return {
            'max_reach': max_reach,
            'min_reach': min_reach,
            'max_height': max_reach,
            'min_height': -max_reach
        }

    def is_position_reachable(self, position: List[float]) -> bool:
        """
        Check if a position is within the robot's workspace.
        
        Args:
            position (List[float]): Target position [x, y, z]
            
        Returns:
            bool: True if position is reachable, False otherwise
        """
        x, y, z = position
        distance = math.sqrt(x**2 + y**2 + z**2)
        workspace = self.get_workspace_limits()
        
        return workspace['min_reach'] <= distance <= workspace['max_reach']
