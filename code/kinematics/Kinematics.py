'''
This file contains the Kinematic class which is responsible for the forward and inverse kinematics of the robot arm.
The forward kinematics function calculates the end-effector position given the joint angles.
The inverse kinematics function calculates the joint angles given the end-effector position.
@author: Jaysh Khan
'''

import math

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class Kinematic:
    def __init__(self, deg_change=1):
        self.a1 = 10.5
        self.a2 = 10
        self.a3 = 16.8
        self.coord = [0, 0, 0]
        self.deg = deg_change
        self.phi_deg_values = np.arange(-180, 180, self.deg)
        self.dhs = [
            {'alpha': 0, 'a': 0, 'd': 0},
            {'alpha': math.pi / 2, 'a': 0, 'd': 0},
            {'alpha': math.pi, 'a': self.a1, 'd': 0},
            {'alpha': 0, 'a': self.a2, 'd': 0},
            {'alpha': 0, 'a': 0, 'd': 0},
        ]

    def generate_matrix(self, theta, i):
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(self.dhs[i]['alpha']), np.sin(theta) * np.sin(self.dhs[i]['alpha']),
             self.dhs[i]['a'] * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(self.dhs[i]['alpha']), -np.cos(theta) * np.sin(self.dhs[i]['alpha']),
             self.dhs[i]['a'] * np.sin(theta)],
            [0, np.sin(self.dhs[i]['alpha']), np.cos(self.dhs[i]['alpha']), self.dhs[i]['d']],
            [0, 0, 0, 1]])

    def forward(self, angles):
        theta0, theta1, theta2, theta3, theta4, theta5 = angles
        t0_1 = self.generate_matrix(theta1, 1)
        t1_2 = self.generate_matrix(theta2, 2)
        t2_3 = self.generate_matrix(theta3, 3)
        t3_4 = self.generate_matrix(theta4, 4)

        t_end = np.array([
            [1, 0, 0, 0],
            [0, np.cos(np.pi / 2), -np.sin(np.pi / 2), -16.8],
            [0, np.sin(np.pi / 2), np.cos(np.pi / 2), 0],
            [0, 0, 0, 1]
        ])
        # Calculating the total transformation from base to end-effector
        t0_4 = t0_1 @ t1_2 @ t2_3 @ t3_4 @ t_end
        return t0_4

    def inverse(self, coordinates, app):
        px, py, pz = coordinates
        self.coord = coordinates
        angles = []
        theta0 = math.atan2(py, px)
        px_projected = px * math.cos(theta0) + py * math.sin(theta0)
        px = px_projected

        for phi in self.phi_deg_values:
            phi = phi * np.pi / 180
            wx = px - (self.a3 * np.cos(phi))
            wz = pz - (self.a3 * np.sin(phi))

            c2 = (wx ** 2 + wz ** 2 - self.a1 ** 2 - self.a2 ** 2) / (2 * self.a1 * self.a2)

            if c2 <= 1:
                s2_1 = np.sqrt(1 - c2 ** 2)
                s2_2 = -np.sqrt(1 - c2 ** 2)
                theta2_1 = np.arctan2(s2_1, c2)
                theta2_2 = np.arctan2(s2_2, c2)

                denom_1 = self.a1 ** 2 + self.a2 ** 2 + 2 * self.a1 * self.a2 * np.cos(theta2_1)
                denom_2 = self.a1 ** 2 + self.a2 ** 2 + 2 * self.a1 * self.a2 * np.cos(theta2_2)
                s1_1 = (wz * (self.a1 + self.a2 * np.cos(theta2_1)) - self.a2 * np.sin(theta2_1) * wx) / denom_1
                s1_2 = (wz * (self.a1 + self.a2 * np.cos(theta2_2)) - self.a2 * np.sin(theta2_2) * wx) / denom_2
                c1_1 = (wx * (self.a1 + self.a2 * np.cos(theta2_1)) + self.a2 * np.sin(theta2_1) * wz) / denom_1
                c1_2 = (wx * (self.a1 + self.a2 * np.cos(theta2_2)) + self.a2 * np.sin(theta2_2) * wz) / denom_2
                theta1_1 = np.arctan2(s1_1, c1_1)
                theta1_2 = np.arctan2(s1_2, c1_2)

                theta3_1 = phi - theta1_1 - theta2_1
                theta3_2 = phi - theta1_2 - theta2_2

                sol_1 = [theta0, theta1_1, -theta2_1, -theta3_1 + (math.pi / 2)]  # Elbow Up
                sol_2 = [theta0, theta1_2, -theta2_2, -theta3_2 + (math.pi / 2)]  # Elbow Down

                angles.append(sol_1)
                angles.append(sol_2)
        return angles

    def plot_3d_robot_arm(self, solutions, app):
        # TODO: Currently not ploting as expected, need to fix this

        print(f'Plotting {solutions} solutions')

        a1 = app.kinematic.a1
        a2 = app.kinematic.a2
        px, py, pz = app.kinematic.coord
        theta0 = math.atan2(py, px)
        px_rotated = px * math.cos(theta0) + py * math.sin(theta0)

        fig = plt.figure()
        ax = fig.add_subplot()
        # clear the canvas
        ax.clear()
        # Convert angles from degrees to radians
        for solution in solutions:
            print(f'Plotting solution {solution}')
            theta0, theta1_1, theta2_1, theta3_1 = solution

            # visualizing
            ax_1 = a1 * np.cos(-theta1_1)
            az_1 = a1 * np.sin(-theta1_1)

            bx = ax_1 + a2 * np.cos(-theta1_1 + theta2_1)
            bz = az_1 + a2 * np.sin(-theta1_1 + theta2_1)

            cx = px_rotated
            cz = pz

            xAxisArrayXCord = [-2, 2]
            xAxisArrayYCord = [0, 0]

            zAxisArrayXCord = [0, 0]
            zAxisArrayYCord = [-2, 2]

            link1XCoords_1 = [0, ax_1]
            link1ZCoords_1 = [0, az_1]

            link2XCoords_1 = [ax_1, bx]
            link2ZCoords_1 = [az_1, bz]

            link3XCoords_1 = [bx, cx]
            link3ZCoords_1 = [bz, cz]

            plt.plot(xAxisArrayXCord, xAxisArrayYCord, 'k')
            plt.plot(zAxisArrayXCord, zAxisArrayYCord, 'k')
            plt.plot(link1XCoords_1, link1ZCoords_1, 'y')
            plt.plot(link2XCoords_1, link2ZCoords_1, 'm')
            plt.plot(link3XCoords_1, link3ZCoords_1, 'black')

        # Legend to differentiate solutions
        ax.plot([], [], 'y', label='Link 1')
        ax.plot([], [], 'm', label='Link 2')
        ax.plot([], [], 'black', label='Link 3')
        # name x,z labels
        ax.set_xlabel('X')
        ax.set_ylabel('Z')

        ax.legend()

        # Create Tkinter canvas and embed Matplotlib plot
        # clear the canvas
        if app.matplotlib_canvas:
            app.matplotlib_canvas.get_tk_widget().destroy()
        app.matplotlib_canvas = FigureCanvasTkAgg(fig, master=app.matplotlib_plot)
        app.matplotlib_canvas.draw()
        app.matplotlib_canvas.get_tk_widget().pack(fill='both', expand=True)

    def update_phi(self, change):
        self.deg = change
        self.phi_deg_values = np.arange(-180, 180, change)
