# forward Kinematics of 6dof manipulator using Denavit-Hartenberg parameters

import numpy as np
import math

"""
   Function to generate the Denavit-Hartenberg matrix for a given set of parameters
   :param a:
   :param alpha:
   :param d:
   :param thetha:
   :return: np array A Transformation matrix of 4x4
   """


def dh_matrix(a, alpha, d, thetha):
    ct = math.cos(thetha)
    st = math.sin(thetha)
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d],
                     [0, 0, 0, 1]])


# DH parameters for testing
thetha = [0, 0, 0, 0, 0, 0, 0]
a = [0, 0, 10.5, 10, 0, 0, 0]
alpha = [0, 90, 0, 0, 90, 0, 0]
d = [0, 0, 0, 0, 0, 0, 14]

"""
Function to compute the forward kinematics of a 6-DOF manipulator
:param angles: list of joint angles
:return: np array of the end effector position
"""


def forward_kinematics(angles, a, d, alpha):
    # convert angles to radians

    theta = [math.radians(angle) for angle in angles]
    alpha = [math.radians(angle) for angle in alpha]

    # theta =[math.radians(angle) for angle in theta]

    # print(theta)

    T_0_1 = dh_matrix(a[0], alpha[0], d[1], theta[1])  # ignore the first joint

    T_1_2 = dh_matrix(a[1], alpha[1], d[2], theta[2])
    T_2_3 = dh_matrix(a[2], alpha[2], d[3], theta[3])
    T_3_4 = dh_matrix(a[3], alpha[3], d[4], theta[4])
    T_4_5 = dh_matrix(a[4], alpha[4], d[5], theta[5])
    # T_5_6 = dh_matrix(a[5], alpha[5], d[6], thetha[6])

    # T_0_2 = np.matmul(T_0_1, T_1_2) # ignore the first joint
    T_0_3 = np.matmul(T_1_2, T_2_3)
    T_0_4 = np.matmul(T_0_3, T_3_4)
    T_0_5 = np.matmul(T_0_4, T_4_5)

    return T_0_5

    # add base height in z direction
    # matrix[2, 3] += 12
    # return matrix

# For testing / Debug code
# print("End effector position")
# last = forward_kinematics(thetha, a, d, alpha)
# # round off the values to 2 decimal places print the values of x y z
# last = np.round(last, 2)
# print(f"X: {last[0, 3]} Y: {last[1, 3]} Z: {last[2, 3] + 12}")
#
# # print(forward_kinematics(thetha,a,d,alpha))
