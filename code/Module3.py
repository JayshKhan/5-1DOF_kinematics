import json
import math
import subprocess
from time import sleep

import cv2
import numpy as np
import roboticstoolbox as rtb
import serial
from spatialmath import SE3

ranges = {
    1: (0, 180),
    2: (0, 180),
    3: (0, 150),
    4: (0, 180),
    5: (0, 180),
    6: (0, 180),
}
current_angles = [0, 0, 0, 0]

# Canvas dimensions
canvas_width = 30
canvas_height = 17

# Initialize canvas
canvas = np.ones((canvas_height, canvas_width, 3), dtype=np.uint8) * 255
points = []
trajectory = []


# def scale_coordinates_to_rectangle(coordinates, image_width, image_height):
#     """
#     Scales and translates a list of image coordinates to fit within
#     a specified rectangular region.
#
#     Args:
#         coordinates (list): List of (x, y) tuples representing image coordinates.
#         image_width (int): Width of the original image.
#         image_height (int): Height of the original image.
#
#     Returns:
#         list: A new list of (x, y) tuples scaled and translated to the rectangle.
#     """
#
#     # Define the target rectangle corners (in cm)
#     top_left = (-16, 23)
#     top_right = (16, 23)
#     bottom_right = (16, 23)
#     bottom_left = (-16, 23)
#
#     # Calculate rectangle dimensions
#     rect_width = top_right[0] - top_left[0]
#     rect_height = top_left[1] - bottom_left[1]
#
#     scaled_coordinates = []
#     for x, y in coordinates:
#         # Scale x from 0-image_width to 0-rect_width
#         scaled_x = (x / image_width) * rect_width
#
#         # Scale y from 0-image_height to 0-rect_height (and flip vertically)
#         scaled_y = (1 - (y / image_height)) * rect_height
#
#         # Translate to the rectangle's position
#         scaled_x += top_left[0]
#         scaled_y += bottom_left[1]
#
#         scaled_coordinates.append((scaled_x, scaled_y))
#
#     return scaled_coordinates
#
#
# # --- Function to detect edges and extract coordinates ---
# def extract_edge_coordinates(image_path, threshold1=100, threshold2=200):
#     """
#     Detects edges in an image and returns a list of (x, y) coordinates.
#
#     Args:
#         image_path (str): Path to the input image.
#         threshold1 (int): Lower threshold for Canny edge detection.
#         threshold2 (int): Upper threshold for Canny edge detection.
#
#     Returns:
#         list: A list of tuples, where each tuple represents the (x, y)
#               coordinates of an edge point.
#     """
#
#     # 1. Load the image
#     img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
#
#     # 2. Apply Gaussian blur to reduce noise
#     blurred_img = cv2.GaussianBlur(img, (5, 5), 0)
#
#     # 3. Use Canny edge detection
#     edges = cv2.Canny(blurred_img, threshold1, threshold2)
#
#     # show the image
#     cv2.imshow('edges', edges)
#     cv2.waitKey(0)
#     # 4. Find contours (connected components) of the edges
#     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#
#     # 5. Extract coordinates from the contours
#     coordinates = []
#     for contour in contours:
#         for point in contour:
#             x, y = point[0]
#             coordinates.append((x, y))
#
#     return scale_coordinates_to_rectangle(coordinates, img.shape[1], img.shape[0])


def inverse(coordinates):
    a1, a2, a3 = [10.5, 10, 16.8]
    px, py, pz = coordinates

    theta0 = math.atan2(py, px)
    px_projected = px * math.cos(theta0) + py * math.sin(theta0)
    px = px_projected
    phi = 0

    phi = phi * np.pi / 180
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

        angle = [theta0, theta1_1, -theta2_1, -theta3_1 + (math.pi / 2)]  # Elbow Up
        sol_2 = [theta0, theta1_2, -theta2_2, -theta3_2 + (math.pi / 2)]  # Elbow Down

        return angle, sol_2
    return None


def validate_all_angles(angles, errorShow=True):
    flag = True
    #     check the entries for the angles and see if they are within the range
    for i, entry in enumerate(angles):
        # from the ranges dict get the range for the servo
        low, high = ranges.get(i, (0, 180))
        value = entry
        if value < low or value > high:
            # if errorShow:
            #     # messagebox.showerror(f"Error Servo {i}", get_error_response(low, high))
            flag = False
    return flag


def adjust_to_zero(value, threshold=1e-5):
    if abs(value) < threshold:
        return 0
        # return round(value, 2)

    return value


def find_best_angle(solutions):
    # print(f'Solution Found {len(solutions)}')

    valid_solutions = []
    for solution in solutions:
        solution = [math.degrees(angle) for angle in solution]
        # adjust to zero
        solution = [adjust_to_zero(angle) for angle in solution]
        # print(solution)

        # if all the angles are above 0
        if all([angle >= 0 and angle <= 180 for angle in solution]):
            valid_check = [0, solution[0], solution[1], solution[2], solution[3], 0]
            if validate_all_angles(valid_check, False):
                valid_solutions.append(solution)
        print(f'valid solutions: {valid_solutions}')
    print(f'No Of Valid Solutions: {len(valid_solutions)}')
    # print(f'valid solutions: {valid_solutions}')

    # find the closest solution to the current angles
    closest_solution = None
    min_distance = float('inf')
    for solution in valid_solutions:
        distance = sum([abs(a - b) for a, b in zip(solution, current_angles)])
        if distance < min_distance:
            min_distance = distance
            closest_solution = solution
    print(f'closest solution: {closest_solution}')

    return valid_solutions


# # --- Example Usage ---
# image_path = "images/polygon.png"
# coordinates = extract_edge_coordinates(image_path)
# print(coordinates)
# # --- Pass coordinates to your inverse kinematics algorithm ---
# angles = []
# for x, y in coordinates:
#     solutions = inverse([x, y, 0])
#     current_angles = angles[-1] if angles else [0, 0, 0, 0]
#     solutions = find_best_angle(solutions)
#     for solution in solutions:
#         if solution is not None:
#             angles.append(solution)
#             break
#
# print(angles)
# for i in range(len(angles)):
#     angles[i].append(0)
#     angles[i].append(0)

arduino = None


def open_port():
    # giving permission to access the port
    try:
        cmd = ['chmod', '777', "/dev/ttyACM0"]
        subprocess.check_output(['sudo', '-S'] + cmd, input='52974\n', encoding='utf-8')
    except subprocess.CalledProcessError as e:
        print("An error occurred while trying to execute the command:", e)

    try:
        global arduino
        arduino = serial.Serial("/dev/ttyACM0", 9600)
        print("Access Granted")
    except serial.SerialException as e:
        print("Error opening port:", e)
        # messagebox.showerror("Error", "Error opening port. Please check the port and try again.")


def send_to_arduino(data):
    try:
        global arduino
        arduino = serial.Serial("/dev/ttyACM0", 9600)
        print(data)
        arduino.write(data.encode())
        print(arduino.readline())
        for i in range(10):
            print(arduino.readline())
        # arduino.close()
        print("Sent data to Arduino:", data)
    except serial.SerialException as e:
        print("Error connecting to Arduino:", e)


open_port()


def draw_line_segment(event, x, y, flags, param):
    global points, canvas, trajectory

    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        if len(points) > 1:
            cv2.line(canvas, points[-2], points[-1], (0, 0, 0), 2)
            cv2.imshow("Drawing Canvas", canvas)

def get_slope(x_start, y_start, x_end, y_end):
  """Calculates the slope of a line."""
  if x_start == x_end:
    raise ZeroDivisionError("Cannot calculate slope for vertical line")
  return (y_end - y_start) / (x_end - x_start)

def generate_line_trajectory(start ,end, num_points=10):
  """Calculates a list of points along a straight line trajectory."""
  x_start ,y_start = start
  x_end, y_end = end
  slope = get_slope(x_start, y_start, x_end, y_end)
  dx = (x_end - x_start) / (num_points - 1)
  points = []
  for i in range(num_points):
    x = x_start + i * dx
    y = slope * x + y_start  # Assuming no y-intercept needed
    points.append((x, y))
  return points




def canvas_to_workspace(point):

    top_left = (-15, 32)
    top_right = (15, 32)
    bottom_right = (15, 17)
    bottom_left = (-15, 17)
    #
    #     # Calculate rectangle dimensions
    workspace_width = top_right[0] - top_left[0]
    workspace_height = top_left[1] - bottom_left[1]
    x, y = point
    # Flip y-axis (canvas origin is top-left)
    # y = canvas_height - y
    if x > workspace_width / 2:
        w_x = x - workspace_width / 2
    elif x < workspace_width / 2:
        w_x = -workspace_width / 2 + x
    else:
        w_x = 0
    w_y = bottom_left[1] + y

    print(f"for points: {point} Got: ({w_x,w_y})")
    return w_x, w_y


cv2.namedWindow("Drawing Canvas")
cv2.setMouseCallback("Drawing Canvas", draw_line_segment)

# print(canvas_to_workspace((0,0)))

while True:
    cv2.imshow("Drawing Canvas", canvas)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        for i in range(len(points) - 1):
            x, y = points[i]#canvas_to_workspace(points[i])
            x1, y1 = points[i+1]#canvas_to_workspace(points[i + 1])
            trajectory += generate_line_trajectory((x, y), (x1, y1))

        break

    if key == ord("c"):
        canvas = np.ones((canvas_height, canvas_width, 3), dtype=np.uint8) * 255
        points = []
        trajectory = []

# --- Transform and print trajectory for the robot ---
robot_trajectory = [canvas_to_workspace(point) for point in trajectory]
print("Robot Trajectory (Workspace Coordinates):", trajectory)
# draw the trajectory  on plt
import matplotlib.pyplot as plt

plt.plot([x for x, y in robot_trajectory], [y for x, y in robot_trajectory], 'r-')
# plt.show()

cv2.destroyAllWindows()


# --- Inverse Kinematic ---
angles = []
for x, y in robot_trajectory:
    print(f" for point ({x},{y}) Solution is ", end=" ")
    solutions = inverse([x, y, 0])
    current_angles = angles[-1] if angles else [0, 0, 0, 0]
    solutions = find_best_angle(solutions)
    for solution in solutions:
        if solution is not None:
            print(solution)
            angles.append(solution)
            break
        print("None")
print(angles)


total_angles_traj =[]

# --- Send angles to Arduino ---
for i in range(len(angles)-1):
    # generate trajectory between all the angles
    angs = rtb.jtraj(angles[i], angles[i + 1], 10)

    for ang in angs.q:
        total_angles_traj.append(ang)
        data = f"{ang[0]},{ang[1]},{ang[2]},{ang[3]+90},0,0,0,c"
        # save data in a file like a cpp double 2d array

txt = f"double angles[{len(total_angles_traj)}][6] ="
txt +="{"
for angl in total_angles_traj:
    txt +="{"
    for ang in angl:
        txt += f"{ang},"
    txt = txt[:-1]
    txt +="},"
txt+="};"

print(txt)



# i = 0
# print("angles",angles)
# for angle in angles:
#     data = f"{angle[0]},{angle[1]},{angle[2]},{angle[3]+90},0,0,0,s"
#     send_to_arduino(data)
#     sleep(1)


# print(canvas_to_workspace((15,0)))
