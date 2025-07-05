#!/usr/bin/env python3
"""
ROT3U Robotic Arm - Example Usage

This script demonstrates the main features of the ROT3U kinematics library.
It shows how to use forward kinematics, inverse kinematics, trajectory planning,
and visualization features programmatically.

Author: Jaysh Khan
"""

import math
import numpy as np
import matplotlib.pyplot as plt

# Import the ROT3U kinematics modules
from kinematics.Kinematics import Kinematic

try:
    from Module3 import (
        generate_shape_trajectory,
        trajectory_to_joint_angles,
        generate_smooth_trajectory,
        save_trajectory_to_json
    )
    HAS_TRAJECTORY_PLANNING = True
except ImportError as e:
    print(f"Warning: Could not import trajectory planning functions: {e}")
    HAS_TRAJECTORY_PLANNING = False


def example_forward_kinematics():
    """Demonstrate forward kinematics calculations."""
    print("=" * 50)
    print("FORWARD KINEMATICS EXAMPLE")
    print("=" * 50)
    
    # Initialize the robot
    robot = Kinematic()
    
    # Define some example joint angles (in degrees)
    test_angles = [
        [0, 0, 0, 0, 0, 0],      # Home position
        [90, 45, 90, 45, 0, 0],  # Example pose 1
        [45, 90, 45, 90, 0, 0],  # Example pose 2
        [0, 30, 60, 90, 0, 0],   # Example pose 3
    ]
    
    print("Testing forward kinematics for different joint configurations:")
    print()
    
    for i, angles_deg in enumerate(test_angles):
        # Convert to radians
        angles_rad = [math.radians(angle) for angle in angles_deg]
        
        # Calculate forward kinematics
        transform = robot.forward_kinematics(angles_rad)
        
        # Extract position
        x, y, z = transform[0, 3], transform[1, 3], transform[2, 3]
        
        print(f"Configuration {i+1}:")
        print(f"  Joint angles: {angles_deg}")
        print(f"  End-effector position: X={x:.2f}, Y={y:.2f}, Z={z:.2f} cm")
        print(f"  Distance from origin: {math.sqrt(x**2 + y**2 + z**2):.2f} cm")
        print()


def example_inverse_kinematics():
    """Demonstrate inverse kinematics calculations."""
    print("=" * 50)
    print("INVERSE KINEMATICS EXAMPLE")
    print("=" * 50)
    
    # Initialize the robot
    robot = Kinematic()
    
    # Define target positions
    target_positions = [
        [20, 0, 15],    # Position 1
        [15, 15, 10],   # Position 2
        [10, -10, 20],  # Position 3
        [25, 5, 5],     # Position 4
    ]
    
    print("Testing inverse kinematics for different target positions:")
    print()
    
    for i, target in enumerate(target_positions):
        print(f"Target Position {i+1}: X={target[0]}, Y={target[1]}, Z={target[2]} cm")
        
        # Check if position is reachable
        if not robot.is_position_reachable(target):
            print("  WARNING: Position may be outside workspace!")
        
        # Calculate inverse kinematics
        solutions = robot.inverse_kinematics(target)
        
        if solutions:
            print(f"  Found {len(solutions)} solution(s):")
            
            # Find best solution
            current_angles = [0, 0, 0, 0]  # Assume starting from home
            best_solution = robot.find_best_solution(solutions, current_angles)
            
            if best_solution:
                print(f"  Best solution: {[f'{angle:.1f}°' for angle in best_solution]}")
                
                # Verify with forward kinematics
                verify_angles = [0] + [math.radians(a) for a in best_solution] + [0]
                verify_transform = robot.forward_kinematics(verify_angles)
                vx, vy, vz = verify_transform[0, 3], verify_transform[1, 3], verify_transform[2, 3]
                
                error = math.sqrt((target[0]-vx)**2 + (target[1]-vy)**2 + (target[2]-vz)**2)
                print(f"  Verification: X={vx:.2f}, Y={vy:.2f}, Z={vz:.2f} cm")
                print(f"  Position error: {error:.3f} cm")
            else:
                print("  No valid solution within joint limits!")
        else:
            print("  No solution found!")
        print()


def example_workspace_analysis():
    """Demonstrate workspace analysis."""
    print("=" * 50)
    print("WORKSPACE ANALYSIS EXAMPLE")
    print("=" * 50)
    
    # Initialize the robot
    robot = Kinematic()
    
    # Get workspace limits
    workspace = robot.get_workspace_limits()
    
    print("Robot workspace characteristics:")
    print(f"  Maximum reach: {workspace['max_reach']:.1f} cm")
    print(f"  Minimum reach: {workspace['min_reach']:.1f} cm")
    print(f"  Maximum height: {workspace['max_height']:.1f} cm")
    print(f"  Minimum height: {workspace['min_height']:.1f} cm")
    print()
    
    # Test various positions
    test_positions = [
        [10, 0, 10],   # Should be reachable
        [30, 0, 0],    # Should be reachable
        [50, 0, 0],    # Should be unreachable
        [0, 0, 40],    # Should be unreachable
    ]
    
    print("Testing position reachability:")
    for pos in test_positions:
        reachable = robot.is_position_reachable(pos)
        status = "✓ Reachable" if reachable else "✗ Unreachable"
        print(f"  Position {pos}: {status}")
    print()


def example_trajectory_planning():
    """Demonstrate trajectory planning."""
    print("=" * 50)
    print("TRAJECTORY PLANNING EXAMPLE")
    print("=" * 50)
    
    if not HAS_TRAJECTORY_PLANNING:
        print("Trajectory planning functions not available.")
        print("This is optional functionality for advanced path planning.")
        print()
        return
    
    print("Generating trajectory for drawing a square...")
    
    # Generate square trajectory
    square_coords = generate_shape_trajectory('square', num_points=5)
    print(f"Generated {len(square_coords)} waypoints for square")
    
    # Convert to joint angles
    joint_angles_sequence = trajectory_to_joint_angles(square_coords)
    print(f"Converted to {len(joint_angles_sequence)} joint angle sets")
    
    if joint_angles_sequence:
        # Generate smooth trajectory
        smooth_trajectory = generate_smooth_trajectory(joint_angles_sequence)
        print(f"Generated {len(smooth_trajectory)} smooth trajectory segments")
        
        # Save trajectory
        save_trajectory_to_json(smooth_trajectory, 'example_square_trajectory.json')
        print("Trajectory saved to 'example_square_trajectory.json'")
    else:
        print("No valid joint angle solutions found for trajectory!")
    print()


def example_visualization():
    """Demonstrate robot arm visualization."""
    print("=" * 50)
    print("VISUALIZATION EXAMPLE")
    print("=" * 50)
    
    # Initialize the robot
    robot = Kinematic()
    
    # Define a target position
    target_pos = [20, 10, 15]
    print(f"Calculating solutions for position: {target_pos}")
    
    # Calculate inverse kinematics
    solutions = robot.inverse_kinematics(target_pos)
    
    if solutions:
        print(f"Found {len(solutions)} solution(s)")
        
        # Create a simple matplotlib figure for visualization
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Plot each solution
        colors = ['red', 'blue', 'green', 'orange']
        
        for i, solution in enumerate(solutions[:4]):  # Limit to 4 solutions
            color = colors[i % len(colors)]
            theta0, theta1, theta2, theta3 = solution
            
            # Calculate link positions (simplified 2D projection)
            x1 = robot.a1 * np.cos(-theta1)
            z1 = robot.a1 * np.sin(-theta1)
            
            x2 = x1 + robot.a2 * np.cos(-theta1 + theta2)
            z2 = z1 + robot.a2 * np.sin(-theta1 + theta2)
            
            # Project target position to 2D
            px_projected = target_pos[0] * math.cos(theta0) + target_pos[1] * math.sin(theta0)
            x3, z3 = px_projected, target_pos[2]
            
            # Plot links
            ax.plot([0, x1], [0, z1], color=color, linewidth=3, label=f'Solution {i+1}')
            ax.plot([x1, x2], [z1, z2], color=color, linewidth=3, linestyle='--')
            ax.plot([x2, x3], [z2, z3], color=color, linewidth=3, linestyle=':')
            
            # Plot joints
            ax.plot([0, x1, x2, x3], [0, z1, z2, z3], 'o', color=color, markersize=8)
        
        # Formatting
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Z (cm)')
        ax.set_title('ROT3U Robot Arm - Multiple Solutions')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        # Save the plot
        plt.savefig('robot_arm_example.png', dpi=300, bbox_inches='tight')
        print("Visualization saved as 'robot_arm_example.png'")
        
        # Show the plot (comment out if running headless)
        # plt.show()
        
    else:
        print("No solutions found for visualization!")
    print()


def main():
    """Run all examples."""
    print("ROT3U Robotic Arm - Example Usage")
    print("This script demonstrates the main features of the kinematics library.")
    print()
    
    try:
        # Run all examples
        example_forward_kinematics()
        example_inverse_kinematics()
        example_workspace_analysis()
        example_trajectory_planning()
        example_visualization()
        
        print("=" * 50)
        print("ALL EXAMPLES COMPLETED SUCCESSFULLY!")
        print("=" * 50)
        print()
        print("Generated files:")
        print("  - example_square_trajectory.json")
        print("  - robot_arm_example.png")
        print()
        print("To start the GUI application, run:")
        print("  python main.py")
        
    except Exception as e:
        print(f"Error running examples: {e}")
        print("Make sure all dependencies are installed:")
        print("  pip install -r requirements.txt")


if __name__ == "__main__":
    main() 