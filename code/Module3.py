"""
ROT3U Robotic Arm - Trajectory Generation and Utility Functions

This module provides trajectory generation and utility functions for the ROT3U robotic arm.
It includes path planning, angle validation, and coordinate transformations.

Author: Jaysh Khan
"""

import json
import math
import numpy as np

# Optional import - fallback to simple interpolation if not available
try:
    import roboticstoolbox as rtb
    HAS_ROBOTICS_TOOLBOX = True
except ImportError:
    HAS_ROBOTICS_TOOLBOX = False
    print("Note: roboticstoolbox-python not found. Using simple linear interpolation for trajectories.")


# Servo angle ranges (degrees)
SERVO_RANGES = {
    1: (0, 180),
    2: (0, 180),
    3: (0, 150),
    4: (0, 180),
    5: (0, 180),
    6: (0, 180),
}

# Robot arm dimensions (cm)
ARM_DIMENSIONS = {
    'a1': 10.5,  # shoulder to elbow
    'a2': 10.0,  # elbow to wrist
    'a3': 16.8,  # wrist to end effector
}


def inverse_kinematics(coordinates):
    """
    Calculate inverse kinematics for given coordinates.
    
    Args:
        coordinates (tuple): (x, y, z) coordinates in cm
        
    Returns:
        tuple: Two solutions (elbow up, elbow down) as lists of joint angles
    """
    a1, a2, a3 = ARM_DIMENSIONS['a1'], ARM_DIMENSIONS['a2'], ARM_DIMENSIONS['a3']
    px, py, pz = coordinates

    # Calculate base rotation angle
    theta0 = math.atan2(py, px)
    px_projected = px * math.cos(theta0) + py * math.sin(theta0)
    px = px_projected
    phi = 0  # End effector orientation

    phi = phi * np.pi / 180
    wx = px - (a3 * np.cos(phi))
    wz = pz - (a3 * np.sin(phi))

    # Calculate joint 2 angle
    c2 = (wx ** 2 + wz ** 2 - a1 ** 2 - a2 ** 2) / (2 * a1 * a2)

    if c2 <= 1:
        s2_1 = np.sqrt(1 - c2 ** 2)
        s2_2 = -np.sqrt(1 - c2 ** 2)
        theta2_1 = np.arctan2(s2_1, c2)
        theta2_2 = np.arctan2(s2_2, c2)

        # Calculate joint 1 angles
        denom_1 = a1 ** 2 + a2 ** 2 + 2 * a1 * a2 * np.cos(theta2_1)
        denom_2 = a1 ** 2 + a2 ** 2 + 2 * a1 * a2 * np.cos(theta2_2)
        
        s1_1 = (wz * (a1 + a2 * np.cos(theta2_1)) - a2 * np.sin(theta2_1) * wx) / denom_1
        s1_2 = (wz * (a1 + a2 * np.cos(theta2_2)) - a2 * np.sin(theta2_2) * wx) / denom_2
        c1_1 = (wx * (a1 + a2 * np.cos(theta2_1)) + a2 * np.sin(theta2_1) * wz) / denom_1
        c1_2 = (wx * (a1 + a2 * np.cos(theta2_2)) + a2 * np.sin(theta2_2) * wz) / denom_2
        
        theta1_1 = np.arctan2(s1_1, c1_1)
        theta1_2 = np.arctan2(s1_2, c1_2)

        # Calculate joint 3 angles
        theta3_1 = phi - theta1_1 - theta2_1
        theta3_2 = phi - theta1_2 - theta2_2

        # Return solutions: elbow up and elbow down
        solution_1 = [theta0, theta1_1, -theta2_1, -theta3_1 + (math.pi / 2)]
        solution_2 = [theta0, theta1_2, -theta2_2, -theta3_2 + (math.pi / 2)]

        return solution_1, solution_2
    
    return None, None


def validate_joint_angles(angles, show_errors=True):
    """
    Validate that all joint angles are within their valid ranges.
    
    Args:
        angles (list): List of joint angles in degrees
        show_errors (bool): Whether to show error messages
        
    Returns:
        bool: True if all angles are valid, False otherwise
    """
    for i, angle in enumerate(angles):
        if i in SERVO_RANGES:
            min_angle, max_angle = SERVO_RANGES[i]
            if angle < min_angle or angle > max_angle:
                if show_errors:
                    print(f"Servo {i} angle {angle}° out of range [{min_angle}°, {max_angle}°]")
                return False
    return True


def normalize_small_values(value, threshold=1e-5):
    """
    Set very small values to zero to avoid numerical precision issues.
    
    Args:
        value (float): Input value
        threshold (float): Threshold below which values are set to zero
        
    Returns:
        float: Normalized value
    """
    return 0.0 if abs(value) < threshold else value


def find_best_solution(solutions, current_angles):
    """
    Find the best inverse kinematics solution based on proximity to current angles.
    
    Args:
        solutions (list): List of possible solutions
        current_angles (list): Current joint angles
        
    Returns:
        list: Valid solutions that are within servo ranges
    """
    valid_solutions = []
    
    for solution in solutions:
        if solution is None:
            continue
            
        # Convert to degrees
        solution_degrees = [math.degrees(angle) for angle in solution]
        solution_degrees = [normalize_small_values(angle) for angle in solution_degrees]
        
        # Check if angles are positive and within general range
        if all(0 <= angle <= 180 for angle in solution_degrees):
            # Create full 6-DOF angle array for validation
            full_angles = [0] + solution_degrees + [0]  # Base and gripper angles
            
            if validate_joint_angles(full_angles, show_errors=False):
                valid_solutions.append(solution_degrees)
    
    return valid_solutions


def generate_line_trajectory(start, end, num_points=100):
    """
    Generate a straight line trajectory between two points.
    
    Args:
        start (tuple): Starting point (x, y, z)
        end (tuple): Ending point (x, y, z)
        num_points (int): Number of points in trajectory
        
    Returns:
        list: List of (x, y, z) coordinates along the trajectory
    """
    x_start, y_start, z_start = start
    x_end, y_end, z_end = end
    
    x_points = np.linspace(x_start, x_end, num_points)
    y_points = np.linspace(y_start, y_end, num_points)
    z_points = np.linspace(z_start, z_end, num_points)
    
    return [(x, y, z) for x, y, z in zip(x_points, y_points, z_points)]


def generate_shape_trajectory(shape='square', num_points=3):
    """
    Generate a trajectory for drawing basic shapes.
    
    Args:
        shape (str): Shape to draw ('square', 'triangle', 'line')
        num_points (int): Number of points per line segment
        
    Returns:
        list: List of (x, y, z) coordinates for the shape
    """
    shapes = {
        'square': [(-10, 25, 0), (-10, 32, 0), (10, 32, 0), (10, 25, 0), (-10, 25, 0)],
        'triangle': [(-10, 25, 0), (0, 32, 0), (10, 25, 0), (-10, 25, 0)],
        'line': [(-10, 25, 0), (10, 32, 0)]
    }
    
    if shape not in shapes:
        raise ValueError(f"Unsupported shape: {shape}. Choose from {list(shapes.keys())}")
    
    waypoints = shapes[shape]
    trajectory = []
    
    for i in range(len(waypoints) - 1):
        trajectory.extend(generate_line_trajectory(waypoints[i], waypoints[i + 1], num_points))
    
    return trajectory


def trajectory_to_joint_angles(coordinates_trajectory):
    """
    Convert a trajectory of coordinates to joint angles using inverse kinematics.
    
    Args:
        coordinates_trajectory (list): List of (x, y, z) coordinates
        
    Returns:
        list: List of joint angle arrays
    """
    joint_angles = []
    
    for coordinates in coordinates_trajectory:
        solution_1, solution_2 = inverse_kinematics(coordinates)
        
        if solution_1 is not None:
            current_angles = joint_angles[-1] if joint_angles else [0, 0, 0, 0]
            valid_solutions = find_best_solution([solution_1, solution_2], current_angles)
            
            if valid_solutions:
                joint_angles.append(valid_solutions[0])
    
    return joint_angles


def generate_smooth_trajectory(joint_angles_sequence):
    """
    Generate smooth joint trajectories between waypoints.
    
    Uses robotics toolbox if available, otherwise falls back to linear interpolation.
    
    Args:
        joint_angles_sequence (list): List of joint angle arrays
        
    Returns:
        list: List of smooth trajectory arrays
    """
    smooth_trajectories = []
    
    for i in range(len(joint_angles_sequence) - 1):
        start_angles = joint_angles_sequence[i]
        end_angles = joint_angles_sequence[i + 1]
        
        if HAS_ROBOTICS_TOOLBOX:
            # Use robotics toolbox for smooth trajectory
            trajectory = rtb.jtraj(start_angles, end_angles, 50)
            smooth_trajectories.append(trajectory.q)
        else:
            # Fallback to simple linear interpolation
            trajectory = _linear_interpolation_trajectory(start_angles, end_angles, 50)
            smooth_trajectories.append(trajectory)
    
    return smooth_trajectories


def _linear_interpolation_trajectory(start_angles, end_angles, num_points):
    """
    Simple linear interpolation between joint angles.
    
    Args:
        start_angles (list): Starting joint angles
        end_angles (list): Ending joint angles
        num_points (int): Number of interpolation points
        
    Returns:
        np.ndarray: Interpolated trajectory
    """
    start_angles = np.array(start_angles)
    end_angles = np.array(end_angles)
    
    trajectory = []
    for i in range(num_points):
        t = i / (num_points - 1)  # Parameter from 0 to 1
        interpolated = start_angles + t * (end_angles - start_angles)
        trajectory.append(interpolated)
    
    return np.array(trajectory)


def save_trajectory_to_json(trajectory, filename='trajectory.json'):
    """
    Save trajectory data to a JSON file.
    
    Args:
        trajectory (list): Trajectory data
        filename (str): Output filename
    """
    try:
        # Convert numpy arrays to lists for JSON serialization
        if hasattr(trajectory, 'tolist'):
             trajectory = trajectory.tolist()
        elif isinstance(trajectory, list):
             # Check if elements are numpy arrays
             trajectory = [item.tolist() if hasattr(item, 'tolist') else item for item in trajectory]

        with open(filename, 'w') as file:
            json.dump(trajectory, file, indent=2)
        print(f"Trajectory saved to {filename}")
    except Exception as e:
        print(f"Error saving trajectory: {e}")


# Example usage functions
def create_square_trajectory():
    """Create a trajectory for drawing a square."""
    coordinates = generate_shape_trajectory('square', 5)
    joint_angles = trajectory_to_joint_angles(coordinates)
    smooth_trajectory = generate_smooth_trajectory(joint_angles)
    return smooth_trajectory


def create_triangle_trajectory():
    """Create a trajectory for drawing a triangle."""
    coordinates = generate_shape_trajectory('triangle', 5)
    joint_angles = trajectory_to_joint_angles(coordinates)
    smooth_trajectory = generate_smooth_trajectory(joint_angles)
    return smooth_trajectory


if __name__ == "__main__":
    # Example: Generate a square trajectory
    print("Generating square trajectory...")
    square_traj = create_square_trajectory()
    print(f"Generated {len(square_traj)} trajectory segments")
    
    # Save to file
    save_trajectory_to_json(square_traj, 'square_trajectory.json')



