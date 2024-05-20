"""
This script demonstrates how to calculate the forward kinematics of a 6DOF robotic arm
using Denavit-Hartenberg parameters and homogeneous transformation matrices.

The End Effector is taken as a point and its transformation matrix is calculated
using translation and rotation matrices.

then the transformation matrix is multiplied with
the transformation matrix of the end effector

@Author: Jaysh Khan

"""
import math

import numpy as np


def dh_matrix(theta, alpha, a, d):
    """
    This function creates a homogeneous transformation matrix according to DH parameters.

    Args:
        theta (float): Rotational joint angle in radians.
        alpha (float): Link twist angle in radians.
        a (float): Link length in the same unit as d.
        d (float): Link offset along the z-axis in the same unit as a.

    Returns:
        A 4x4 homogeneous transformation matrix.
    """
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    return [[ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]]


def forward_kinematics(thetas, dhs):
    theta_ig, theta1, theta2, theta3, theta4, theta_ig2 = thetas
    print(f'getting angles for theta1={theta1}, theta2={theta2}, theta3={theta3}, theta4={theta4}')
    """Calculates forward kinematics with the provided DH table.

        Args:
            theta1: Angle of joint 1 (radians).
            theta2: Angle of joint 2 (radians).
            theta3: Angle of joint 3 (radians).

        Returns:
            A numpy array [x, y, z] representing the end-effector position.
        """
    T0_1 = np.array([
        [np.cos(theta1), -np.sin(theta1) * np.cos(dhs[1]['alpha']), np.sin(theta1) * np.sin(dhs[1]['alpha']),
         dhs[1]['a'] * np.cos(theta1)],
        [np.sin(theta1), np.cos(theta1) * np.cos(dhs[1]['alpha']), -np.cos(theta1) * np.sin(dhs[1]['alpha']),
         dhs[1]['a'] * np.sin(theta1)],
        [0, np.sin(dhs[1]['alpha']), np.cos(dhs[1]['alpha']), dhs[1]['d']],
        [0, 0, 0, 1]
    ])
    T1_2 = np.array([
        [np.cos(theta2), -np.sin(theta2) * np.cos(dhs[2]['alpha']), np.sin(theta2) * np.sin(dhs[2]['alpha']),
         dhs[2]['a'] * np.cos(theta2)],
        [np.sin(theta2), np.cos(theta2) * np.cos(dhs[2]['alpha']), -np.cos(theta2) * np.sin(dhs[2]['alpha']),
         dhs[2]['a'] * np.sin(theta2)],
        [0, np.sin(dhs[2]['alpha']), np.cos(dhs[2]['alpha']), dhs[2]['d']],
        [0, 0, 0, 1]
    ])
    T2_3 = np.array([
        [np.cos(theta3), -np.sin(theta3) * np.cos(dhs[3]['alpha']), np.sin(theta3) * np.sin(dhs[3]['alpha']),
         dhs[3]['a'] * np.cos(theta3)],
        [np.sin(theta3), np.cos(theta3) * np.cos(dhs[3]['alpha']), -np.cos(theta3) * np.sin(dhs[3]['alpha']),
         dhs[3]['a'] * np.sin(theta3)],
        [0, np.sin(dhs[3]['alpha']), np.cos(dhs[3]['alpha']), dhs[3]['d']],
        [0, 0, 0, 1]
    ])
    T3_4 = np.array([
        [np.cos(theta4), -np.sin(theta4) * np.cos(dhs[4]['alpha']), np.sin(theta4) * np.sin(dhs[4]['alpha']),
         dhs[4]['a'] * np.cos(theta4)],
        [np.sin(theta4), np.cos(theta4) * np.cos(dhs[4]['alpha']), -np.cos(theta4) * np.sin(dhs[4]['alpha']),
         dhs[4]['a'] * np.sin(theta4)],
        [0, np.sin(dhs[4]['alpha']), np.cos(dhs[4]['alpha']), dhs[4]['d']],
        [0, 0, 0, 1]
    ])

    T_EOF = np.array([
        [1, 0, 0, 0],
        [0, np.cos(np.pi / 2), -np.sin(np.pi / 2), -16.8],
        [0, np.sin(np.pi / 2), np.cos(np.pi / 2), 0],
        [0, 0, 0, 1]
    ])
    # Calculate the total transformation from base to end-effector
    T0_4 = T0_1 @ T1_2 @ T2_3 @ T3_4 @ T_EOF

    # Extract the end-effector position
    x = T0_4[0, 3]
    y = T0_4[1, 3]
    z = T0_4[2, 3]

    return T0_4


'''
Below Code is For manual Testing the Thetas Recieved from Inverse Kinematic
'''
angles = [0.0, 1.5707963267948966, 1.281950912444666, 0.8007330786079955, 3.1166650043481083, 0.0]
dhs = [
    {'alpha': 0, 'a': 0, 'd': 0},
    {'alpha': math.pi / 2, 'a': 0, 'd': 0},
    {'alpha': math.pi, 'a': 10.5, 'd': 0},
    {'alpha': 0, 'a': 10, 'd': 0},
    {'alpha': 0, 'a': 0, 'd': 0},
]

print(np.round(forward_kinematics(angles, dhs), 2))
