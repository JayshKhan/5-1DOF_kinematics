# ''' File For Testing Multiple codes'''
# import math
#
# import numpy as np
# # trajectory Generation Using RoboticsToolBox
# import roboticstoolbox as rtb
# from roboticstoolbox import DHRobot, RevoluteDH
#
# '''
# self.dhs = [
#             {'alpha': 0, 'a': 0, 'd': 0},
#             {'alpha': math.pi / 2, 'a': 0, 'd': 0},
#             {'alpha': math.pi, 'a': self.a1, 'd': 0},
#             {'alpha': 0, 'a': self.a2, 'd': 0},
#             {'alpha': 0, 'a': 0, 'd': 0},
#         ]
# '''
# # Create a 4-DOF robot with standard DH parameters
# robot = DHRobot([
#     # turn Table Joint
#     RevoluteDH(a=0, alpha=0, d=0),
#     # Shoulder Joint
#     RevoluteDH(a=0, alpha=math.pi / 2, d=0),
#     # Elbow Joint
#     RevoluteDH(a=10.5, alpha=math.pi, d=0),
#     # Wrist Joint
#     RevoluteDH(a=10, alpha=0, d=0),
#     # Gripper Joint
#     RevoluteDH(a=0, alpha=0, d=0)
# ],name="Rot3u")
#
# print(robot)
# # plot = robot.plot(robot.q, backend="swift")
#
# # Define the trajectory and waypoints
#
# q0 = [0, 0, 0, 0]
# qf = [math.pi / 2, math.pi, math.radians(120), math.pi / 2]
# # quintic time scaling
# t = rtb.jtraj(q0, qf, 100)
# print(t.q)
#
#
# # print(np.round(arr,0) for arr in q.q)
# # print(np.rad2deg(q.q))
# import cv2
# import numpy as np
#
# # Initialize canvas
# canvas = np.ones((500, 500, 3), dtype=np.uint8) * 255  # White canvas
#
# # Drawing parameters
# drawing = False
# last_point = (0, 0)
# trajectory = []
#
# def draw_line(event, x, y, flags, param):
#     global drawing, last_point, trajectory
#
#     if event == cv2.EVENT_LBUTTONDOWN:
#         drawing = True
#         last_point = (x, y)
#         trajectory.append(last_point)  # Add initial point to trajectory
#
#     elif event == cv2.EVENT_MOUSEMOVE:
#         if drawing:
#             cv2.line(canvas, last_point, (x, y), (0, 0, 0), 2)
#             last_point = (x, y)
#             trajectory.append(last_point)  # Add points along the stroke
#
#     elif event == cv2.EVENT_LBUTTONUP:
#         drawing = False
#
# # Create window and set mouse callback
# cv2.namedWindow("Drawing Canvas")
# cv2.setMouseCallback("Drawing Canvas", draw_line)
#
# while True:
#     cv2.imshow("Drawing Canvas", canvas)
#     key = cv2.waitKey(1) & 0xFF
#
#     if key == ord("q"):  # Press 'q' to quit
#         break
#
#     if key == ord("c"):  # Press 'c' to clear the canvas
#         canvas = np.ones((500, 500, 3), dtype=np.uint8) * 255
#         trajectory = []
#
# cv2.destroyAllWindows()
#
# # --- Simulated Robot Control (Replace with your robot's API) ---
# print("Trajectory:", trajectory)
#
# # Example: Print coordinates scaled to a hypothetical robot workspace
# for x, y in trajectory:
#     scaled_x = x / 500 * 20  # Replace with your robot's scaling
#     scaled_y = y / 500 * 20
#     print(f"Move to: ({scaled_x:.2f}, {scaled_y:.2f})")

import cv2
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

# Robot workspace coordinates
workspace_points = np.array([(6, 10), (-6, 10), (6, 23), (-6, 23)])
workspace_width = workspace_points[:, 0].max() - workspace_points[:, 0].min()
workspace_height = workspace_points[:, 1].max() - workspace_points[:, 1].min()

# Canvas dimensions
canvas_width = 500
canvas_height = 500

# Initialize canvas
canvas = np.ones((canvas_height, canvas_width, 3), dtype=np.uint8) * 255
points = []
trajectory = []

def draw_line_segment(event, x, y, flags, param):
    global points, canvas, trajectory

    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        if len(points) > 1:
            cv2.line(canvas, points[-2], points[-1], (0, 0, 0), 2)
            cv2.imshow("Drawing Canvas", canvas)

def generate_line_trajectory(start, end, steps=10):
   # robotics Toolbox to generate trajectory  of x,y,z
    x_start, y_start = start
    x_end, y_end = end
    start = SE3([x_start, y_start, -8])
    end = SE3([x_end, y_end, -8])
    traj = rtb.tools.trajectory.ctraj(start, end, steps)
    return [(pose.t[0], pose.t[1]) for pose in traj]


def canvas_to_workspace(point):
    """Maps a canvas point to the robot's workspace."""
    x, y = point
    # Flip y-axis (canvas origin is top-left)
    y = canvas_height - y
    # Scale and translate
    workspace_x = (x / canvas_width) * workspace_width + workspace_points[:, 0].min()
    workspace_y = (y / canvas_height) * workspace_height + workspace_points[:, 1].min()
    return workspace_x, workspace_y

cv2.namedWindow("Drawing Canvas")
cv2.setMouseCallback("Drawing Canvas", draw_line_segment)

while True:
    cv2.imshow("Drawing Canvas", canvas)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        for i in range(len(points) - 1):
            x,y =canvas_to_workspace(points[i])
            x1,y1 = canvas_to_workspace(points[i+1])
            trajectory += generate_line_trajectory((x, y), (x1, y1))
            
        break

    if key == ord("c"):
        canvas = np.ones((canvas_height, canvas_width, 3), dtype=np.uint8) * 255
        points = []
        trajectory = []

# --- Transform and print trajectory for the robot ---
robot_trajectory = [canvas_to_workspace(point) for point in trajectory]
print("Robot Trajectory (Workspace Coordinates):", robot_trajectory)
# draw the trajectory  on plt
import matplotlib.pyplot as plt
plt.plot([x for x, y in robot_trajectory], [y for x, y in robot_trajectory], 'r-')
plt.show()



cv2.destroyAllWindows()