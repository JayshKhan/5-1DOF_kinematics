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
    """
    This function calculates the end-effector pose (homogeneous transformation matrix)
    of a 6DOF robotic arm given joint angles and DH parameters.

    Args:
        thetas (list): A list of joint angles (radians) for each joint.
        dhs (list): A list of DH parameter dictionaries for each link.
            Each dictionary should have keys 'alpha', 'a', and 'd'.

    Returns:
        A 4x4 homogeneous transformation matrix representing the end-effector pose.
    """
    T_0_4 = [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]]
    # Calculate the transformation matrix for each joint and multiply them together
    # to get the transformation matrix from the base frame to the last joint.

    for i, (theta, dh) in enumerate(zip(thetas, dhs)):
        T_0_4 = np.matmul(T_0_4, dh_matrix(theta, dh['alpha'], dh['a'], dh['d']))

    T_4_5 = np.array([[1, 0, 0, -2.5],
                      [0, math.cos(math.pi / 2), -math.sin(math.pi / 2), -16.8],
                      [0, math.sin(math.pi / 2), math.cos(math.pi / 2), 0],
                      [0, 0, 0, 1]])  # end effector transformation matrix
    T_0_5 = np.matmul(T_0_4, T_4_5)
    T_0_5[2][3] = T_0_5[2][3]# + 11.5  # Adding the height from ground to the baseframe
    return T_0_5
