import math

import numpy as np

from forwardKinematic import forward_kinematics

BASE_HGT = 115
BC = 10
CD = 16.5
AB = 10.5

def set_arm(x, y, z):
    # Adjust z coordinate for base height
    z -= BASE_HGT

    # Calculate distance from shoulder to end-effector
    dist_y_z = math.sqrt(z**2 + x**2 + y**2)

    # Calculate angles for servo 1 and servo 2
    alfa = 180.0 - (math.degrees(math.acos(
        (-BC*(AB+CD) + math.sqrt((-BC*(AB+CD))**2 - 4*AB * (14225 - dist_y_z**2))) / 71400)))
    gamma = math.degrees(math.acos((27875 - dist_y_z**2 - (34000 * math.cos(math.radians(alfa)))) / (-210 * dist_y_z)))
    omega = math.degrees(math.acos(math.sqrt(x**2 + y**2) / dist_y_z))

    theta_1 = 90 - math.degrees(math.atan2(-x, abs(y)))
    if z > 0.0:
        theta_2 = 180.0 - (gamma + omega)
    else:
        theta_2 = 180.0 - (gamma - omega)

    # Calculate angles for servo 3 and servo 4
    if y < 0.0:
        theta_3 = theta_4 = (alfa - 90.0)
    else:
        theta_3 = theta_4 = (270.0 - alfa)
        theta_2 = 180.0 - theta_2
        theta_1 = 180.0 - theta_1

    return theta_1, theta_2, theta_3, theta_4

# Example usage
while(True):
    x_input = 18.0#float(input("Enter x coordinate: "))
    y_input = 0#float(input("Enter y coordinate: "))
    z_input = -5.3#float(input("Enter z coordinate: "))
    angles=[0,0,0,0,0,0,0]
    angles[0],angles[1],angles[2],angles[4] = set_arm(x_input, y_input, z_input)

    dhs = [
        {'alpha': 0, 'a': 0, 'd': 0},
        {'alpha': math.pi / 2, 'a': 0, 'd': 0},
        {'alpha': 0, 'a': 10.5, 'd': 0},
        {'alpha': 0, 'a': 10, 'd': 0},
        {'alpha': 0, 'a': 0, 'd': 0},
    ]
    # 10,40,150,180,0,0,0
    # angles = [math.radians(angle) for angle in angles]
    print(angles)
    print(np.round(forward_kinematics(angles, dhs), 2))
    input("Press enter to continue")





