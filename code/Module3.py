import math
import subprocess
from time import sleep

import serial

import cv2
import numpy as np

ranges = {
    1: (0, 180),
    2: (0, 180),
    3: (0, 150),
    4: (0, 180),
    5: (0, 180),
    6: (0, 180),
}
current_angles = [0, 0, 0, 0]


def scale_coordinates_to_rectangle(coordinates, image_width, image_height):
    """
    Scales and translates a list of image coordinates to fit within
    a specified rectangular region.

    Args:
        coordinates (list): List of (x, y) tuples representing image coordinates.
        image_width (int): Width of the original image.
        image_height (int): Height of the original image.

    Returns:
        list: A new list of (x, y) tuples scaled and translated to the rectangle.
    """

    # Define the target rectangle corners (in cm)
    top_left = (-16, 23)
    top_right = (16, 23)
    bottom_right = (16, 23)
    bottom_left = (-16, 23)

    # Calculate rectangle dimensions
    rect_width = top_right[0] - top_left[0]
    rect_height = top_left[1] - bottom_left[1]

    scaled_coordinates = []
    for x, y in coordinates:
        # Scale x from 0-image_width to 0-rect_width
        scaled_x = (x / image_width) * rect_width

        # Scale y from 0-image_height to 0-rect_height (and flip vertically)
        scaled_y = (1 - (y / image_height)) * rect_height

        # Translate to the rectangle's position
        scaled_x += top_left[0]
        scaled_y += bottom_left[1]

        scaled_coordinates.append((scaled_x, scaled_y))

    return scaled_coordinates


# --- Function to detect edges and extract coordinates ---
def extract_edge_coordinates(image_path, threshold1=100, threshold2=200):
    """
    Detects edges in an image and returns a list of (x, y) coordinates.

    Args:
        image_path (str): Path to the input image.
        threshold1 (int): Lower threshold for Canny edge detection.
        threshold2 (int): Upper threshold for Canny edge detection.

    Returns:
        list: A list of tuples, where each tuple represents the (x, y)
              coordinates of an edge point.
    """

    # 1. Load the image
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # 2. Apply Gaussian blur to reduce noise
    blurred_img = cv2.GaussianBlur(img, (5, 5), 0)

    # 3. Use Canny edge detection
    edges = cv2.Canny(blurred_img, threshold1, threshold2)

    # show the image
    # cv2.imshow('edges', edges)

    # 4. Find contours (connected components) of the edges
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # 5. Extract coordinates from the contours
    coordinates = []
    for contour in contours:
        for point in contour:
            x, y = point[0]
            coordinates.append((x, y))

    return scale_coordinates_to_rectangle(coordinates, img.shape[1], img.shape[0])


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
    print(f'Solution Found {len(solutions)}')

    valid_solutions = []
    for solution in solutions:
        solution = [math.degrees(angle) for angle in solution]
        # adjust to zero
        solution = [adjust_to_zero(angle) for angle in solution]
        print(solution)

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


# --- Example Usage ---
image_path = "images/polygon.png"
coordinates = extract_edge_coordinates(image_path)
print(coordinates)
# --- Pass coordinates to your inverse kinematics algorithm ---
angles = []
for x, y in coordinates:
    solutions = inverse([x, y, 0])
    current_angles = angles[-1] if angles else [0, 0, 0, 0]
    solutions = find_best_angle(solutions)
    for solution in solutions:
        if solution is not None:
            angles.append(solution)
            break

print(angles)
for i in range(len(angles)):
    angles[i].append(0)
    angles[i].append(0)

arduino =None
def open_port():
    # giving permission to access the port
    try:
        cmd = ['chmod', '777', "/dev/ttyACM0"]
        subprocess.check_output(['sudo', '-S'] + cmd, input='52974\n', encoding='utf-8')
    except subprocess.CalledProcessError as e:
        print("An error occurred while trying to execute the command:", e)

    try:
        global arduino
        arduino = serial.Serial("/dev/ttyACM0",9600)
        print("Access Granted")
    except serial.SerialException as e:
        print("Error opening port:", e)
        # messagebox.showerror("Error", "Error opening port. Please check the port and try again.")


def send_to_arduino(data):
    try:
        global arduino
        arduino = serial.Serial("/dev/ttyACM0", 9600)
        arduino.write(data.encode())
        print(arduino.readline())
        # arduino.close()
        print("Sent data to Arduino:", data)
    except serial.SerialException as e:
        print("Error connecting to Arduino:", e)


open_port()
# send the data to Arduino
for angle in angles:
    # convert the angles to string
    data = f"{angle[0]},{angle[1]},{angle[2]},{angle[3]},{angle[4]},{angle[5]},c"
    send_to_arduino(data)
    sleep(1)

