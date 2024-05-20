import math
import subprocess
# for reading from the arduino
from tkinter import messagebox

import numpy as np
import pyttsx3
import serial  # Import the serial library for Arduino communication

from code.GUI.responses import get_error_response, get_error_rresponses_for_singularity


def text_to_speech(text):
    engine = pyttsx3.init()
    engine.setProperty('rate', 150)  # Adjust speech rate (words per minute)

    engine.say(text)
    engine.runAndWait()


current_angles = [0, 0,  # hip
                  0,  # shoulder
                  0,  # elbow
                  0,  # wrist
                  0,  # wrist_rotate
                  0  # gripper
                  ]
# 0,89,0,89,0,0,0

# Servo Angle Ranges
ranges = {
    1: (0, 180),
    2: (0, 180),
    3: (0, 150),
    4: (0, 180),
    5: (0, 180),
    6: (0, 180),
}


def submit_angles(app):
    angles = []
    for entries in app.angle_entries:
        angles.append(float(entries.get()))
    update_display_and_send(angles, app)


def open_port(app):
    # giving permission to access the port
    try:
        cmd = ['chmod', '777', f'{app.COM_PORT}']
        subprocess.check_output(['sudo', '-S'] + cmd, input='52974\n', encoding='utf-8')
    except subprocess.CalledProcessError as e:
        print("An error occurred while trying to execute the command:", e)

    try:
        app.arduino = serial.Serial(app.COM_PORT, app.BAUD_RATE)
        print("Access Granted")
    except serial.SerialException as e:
        print("Error opening port:", e)
        messagebox.showerror("Error", "Error opening port. Please check the port and try again.")


# Function to validate angle input (ensures values are within servo range)
def validate_angle(text, *args):
    try:
        if text == "":
            return True
        value = int(text)
        if 0 <= value <= 180:
            return True
        else:
            return False
    except ValueError:
        return False


def validate_coords(text, *args):
    try:
        if text == "" or text == "-":
            return True
        value = float(text)
        return True
    except ValueError:
        return False


def validate_all_angles(angles, errorShow=True):
    flag = True
    #     check the entries for the angles and see if they are within the range
    for i, entry in enumerate(angles):
        # from the ranges dict get the range for the servo
        low, high = ranges.get(i, (0, 180))
        value = entry
        if value < low or value > high:
            if errorShow:
                messagebox.showerror(f"Error Servo {i}", get_error_response(low, high))
            flag = False
    return flag


def check_for_singularity(angles):
    return False
    flag = False
    s = angles

    if s[2] <= 90 and s[3] >= 30 and s[4] >= 0:
        flag = True

    if s[3] >= 90 and s[4] >= 90 and s[2] >= 90:
        flag = True
    if s[2] == 180 and s[3] > 90 > s[4]:
        flag = True

    if flag:
        messagebox.showerror("Singularity Error", get_error_rresponses_for_singularity())
        return True


# Function to generate the matrix transformation
def generate_last_transformation_matrix(angles, app):
    dhs = [
        {'alpha': 0, 'a': 0, 'd': 0},
        {'alpha': math.pi / 2, 'a': 0, 'd': 0},
        {'alpha': math.pi, 'a': 10.5, 'd': 0},
        {'alpha': 0, 'a': 10, 'd': 0},
        {'alpha': 0, 'a': 0, 'd': 0},
    ]
    angles = [math.radians(angle) for angle in angles]
    print(f'sending Angles {angles}')

    return np.round(app.kinematic.forward(angles), 2)


# Function to send data to Arduino via serial communication
def send_to_arduino(data, app):
    global arduino
    try:
        arduino = serial.Serial(app.COM_PORT, app.BAUD_RATE)
        arduino.write(data.encode())
        print(arduino.readline())
        # arduino.close()
        print("Sent data to Arduino:", data)
    except serial.SerialException as e:
        print("Error connecting to Arduino:", e)


def adjust_to_zero(value, threshold=1e-5):
    if abs(value) < threshold:
        return 0
        # return round(value, 2)

    return value


def find_best_angle(solutions,app):
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
    current_angles = [app.current_angles[1], app.current_angles[2], app.current_angles[3], app.current_angles[4]]
    closest_solution = None
    min_distance = float('inf')
    for solution in valid_solutions:
        distance = sum([abs(a - b) for a, b in zip(solution, current_angles)])
        if distance < min_distance:
            min_distance = distance
            closest_solution = solution
    print(f'closest solution: {closest_solution}')

    return closest_solution, valid_solutions


def create_cell(canvas, value, x, y, app, background="white", font=("Arial", 16)):
    x1 = x * (app.cell_size + app.padding)
    y1 = y * (app.cell_size + app.padding)
    x2 = x1 + app.cell_size
    y2 = y1 + app.cell_size
    canvas.create_rectangle(x1, y1, x2, y2, fill=background)
    canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2, text=value, font=("Arial", 10), fill="black")


def update_display(matrix, app):
    for y in range(4):
        for x in range(4):
            create_cell(app.canvas, matrix[y][x], x, y, app)


def update_display_and_send(angles, app):
    print(f'angles from {angles}')
    if validate_all_angles(angles, True) and not check_for_singularity(angles):
        # Generate the matrix transformation using the provided angles
        matrix = generate_last_transformation_matrix(angles, app)
        app.xyz_label.configure(text=f"X: {matrix[0][3]} \nY: {matrix[1][3]} \nZ: {matrix[2][3]}")
        app.xyz_label_verify.configure(text=f"From Forward: X: {matrix[0][3]} Y: {matrix[1][3]} Z: {matrix[2][3]}")
        print(matrix)

        # Update the display with the matrix
        update_display(matrix, app)

        """
        The following code is just to match the number of servos with the number of angles
        that arduino serial communication expects
        """
        # ignore the first servo frame 0 which is fixed frame
        angles = angles[1:]
        # add 0 to the end
        angles.append(0)
        angles.append(0)

        servo_mode = str(app.servo_mode_entry.get())
        mode = "s" if servo_mode.startswith("Seq") else "c"
        # Prepare data to send to Arduino (replace with your format)
        data_to_send = ",".join(str(round(angle,4)) for angle in angles) + "," + mode + "\n"
        print(data_to_send)

        # Send data to Arduino
        send_to_arduino(data_to_send, app)
        # updating the current angles
        app.current_angles = angles


def inverse_update_display_and_send(app):
    x = float(app.x_entry.get())
    y = float(app.y_entry.get())
    z = float(app.z_entry.get())
    if z < -10:
        messagebox.showerror("Error", "Table in The Way")
        return
    phi_change = int(app.phi_entry.get())
    app.update_phi(phi_change)
    print(f"X: {x}, Y: {y}, Z: {z}")
    solutions = app.kinematic.inverse([x, y, z], app.matplotlib_canvas)

    solution, valid_solutions = find_best_angle(solutions,app)

    if not solution:
        response = get_error_rresponses_for_singularity()
        messagebox.showerror("No Solution", response)
        # text_to_speech(response)
        return

    app.kinematic.plot_3d_robot_arm(valid_solutions, app)
    angles = [0, solution[0], solution[1], solution[2], solution[3], 0]
    app.angle_print.configure(
        text=f"Servo 1: {round(solution[0])}\nServo 2: {round(solution[1])}\nServo 3: {round(solution[2])}\nServo 4: {round(solution[3])}")
    update_display_and_send(angles, app)
