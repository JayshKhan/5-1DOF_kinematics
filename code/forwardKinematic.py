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
    T = [[1, 0, 0, 0],
         [0, 1, 0, 0],
         [0, 0, 1, 0],
         [0, 0, 0, 1]]
    for i, (theta, dh) in enumerate(zip(thetas, dhs)):
        T = np.matmul(T, dh_matrix(theta, dh['alpha'], dh['a'], dh['d']))
    # add base length in z direction
    T[2][3] = T[2][3] + 12
    return T

# angles = [0, 90, 90, 0, 90, 0, 0]
#
# thetas = [math.radians(angle) for angle in angles]
# # thetas = [math.pi/4, math.pi/3, 0, -math.pi/2, math.pi/6]
#
# # Calculate end-effector pose
# end_effector_pose = np.round(forward_kinematics(thetas, dhs), 2)
#
# # Print the end-effector pose (homogeneous transformation matrix)
# end_effector_pose[2][3] = end_effector_pose[2][3] + 14
# print(end_effector_pose)
