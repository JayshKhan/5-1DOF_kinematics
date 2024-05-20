import math

import matplotlib.pyplot as plt
import numpy as np

a1 = 10.5
a2 = 10
a3 = 16.8


def get_solutions(x, y, z, deg=1):
    print(f'getting angles for x={x}, y={y}, z={z}')
    px = x
    py = y
    pz = z
    angles = []

    # Define a range of phi values to explore
    phi_deg_values = np.arange(-180, 180, deg)
    theta0 = math.atan2(py, px)

    px_rotated = px * math.cos(theta0) + py * math.sin(theta0)
    px = px_rotated

    for phi_deg in phi_deg_values:
        phi = phi_deg * np.pi / 180

        wx = px - (a3 * np.cos(phi))
        wz = pz - (a3 * np.sin(phi))

        c2 = (wx ** 2 + wz ** 2 - a1 ** 2 - a2 ** 2) / (2 * a1 * a2)

        if c2 <= 1:
            s2_1 = np.sqrt(1 - c2 ** 2)
            s2_2 = -np.sqrt(1 - c2 ** 2)
            theta2_1 = np.arctan2(s2_1, c2)
            theta2_2 = np.arctan2(s2_2, c2)

            denom_1 = a1 ** 2 + a2 ** 2 + 2 * a1 * a2 * np.cos(theta2_1)
            denom_2 = a1 ** 2 + a2 ** 2 + 2 * a1 * a2 * np.cos(theta2_2)
            s1_1 = (wz * (a1 + a2 * np.cos(theta2_1)) - a2 * np.sin(theta2_1) * wx) / denom_1
            s1_2 = (wz * (a1 + a2 * np.cos(theta2_2)) - a2 * np.sin(theta2_2) * wx) / denom_2
            c1_1 = (wx * (a1 + a2 * np.cos(theta2_1)) + a2 * np.sin(theta2_1) * wz) / denom_1
            c1_2 = (wx * (a1 + a2 * np.cos(theta2_2)) + a2 * np.sin(theta2_2) * wz) / denom_2
            theta1_1 = np.arctan2(s1_1, c1_1)
            theta1_2 = np.arctan2(s1_2, c1_2)

            theta3_1 = phi - theta1_1 - theta2_1
            theta3_2 = phi - theta1_2 - theta2_2

            sol_1 = [theta0, theta1_1, -theta2_1, -theta3_1 + (math.pi / 2)]  # Elbow Up
            sol_2 = [theta0, theta1_2, -theta2_2, -theta3_2 + (math.pi / 2)]  # Elbow Down

            angles.append(sol_1)
            angles.append(sol_2)

            # visualizing
            ax_1 = a1 * np.cos(theta1_1)
            az_1 = a1 * np.sin(theta1_1)
            ax_2 = a1 * np.cos(theta1_2)
            az_2 = a1 * np.sin(theta1_2)

            bx = ax_1 + a2 * np.cos(theta1_1 + theta2_1)
            bz = az_1 + a2 * np.sin(theta1_1 + theta2_1)

            cx = px
            cz = pz

            xAxisArrayXCord = [-2, 2]
            xAxisArrayYCord = [0, 0]

            zAxisArrayXCord = [0, 0]
            zAxisArrayYCord = [-2, 2]

            link1XCoords_1 = [0, ax_1]
            link1ZCoords_1 = [0, az_1]
            link1XCoords_2 = [0, ax_2]
            link1ZCoords_2 = [0, az_2]

            link2XCoords_1 = [ax_1, bx]
            link2ZCoords_1 = [az_1, bz]
            link2XCoords_2 = [ax_2, bx]
            link2ZCoords_2 = [az_2, bz]

            link3XCoords_1 = [bx, cx]
            link3ZCoords_1 = [bz, cz]

            plt.plot(xAxisArrayXCord, xAxisArrayYCord, 'k')
            plt.plot(zAxisArrayXCord, zAxisArrayYCord, 'k')
            plt.plot(link1XCoords_1, link1ZCoords_1, 'y')
            plt.plot(link2XCoords_1, link2ZCoords_1, 'm')
            plt.plot(link3XCoords_1, link3ZCoords_1, 'black')

            plt.plot(xAxisArrayXCord, xAxisArrayYCord, 'k')
            plt.plot(zAxisArrayXCord, zAxisArrayYCord, 'k')
            plt.plot(link1XCoords_2, link1ZCoords_2, 'r')
            plt.plot(link2XCoords_2, link2ZCoords_2, 'g')
            plt.plot(link3XCoords_1, link3ZCoords_1, 'b')

    plt.title('Arm')
    plt.grid(True)
    plt.show()
    return angles
