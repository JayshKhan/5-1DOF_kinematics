import math
import subprocess
import threading
import tkinter as tk
from tkinter import messagebox

import numpy as np
import serial

# Import forward kinematics functions
from forwardKinematic import forward_kinematics
from responses import get_error_response, get_error_rresponses_for_singularity

# Constants
COM_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# Define servo angle ranges
ranges = {
    1: (0, 180),
    2: (0, 180),
    3: (0, 150),
    4: (0, 180),
    5: (0, 180),
    6: (0, 180),
}

# Global variable for Arduino connection
arduino = None


def update_display_and_send():
    if validate_all_angles() and not check_for_singularity():
        angles = [int(entry.get()) for entry in angle_entries]
        matrix = generate_matrix(angles)
        update_display(matrix)
        send_to_arduino(",".join(str(angle) for angle in angles) + "\n")


# Function to open serial port
def open_port():
    global arduino
    try:
        cmd = ['chmod', '777', f'{COM_PORT}']
        subprocess.check_output(['sudo', '-S'] + cmd, input='52974\n', encoding='utf-8')
    except subprocess.CalledProcessError as e:
        print("An error occurred while trying to execute the command:", e)

    try:
        arduino = serial.Serial(COM_PORT, BAUD_RATE)
        print("Access Granted")
    except serial.SerialException as e:
        print("Error opening port:", e)
        messagebox.showerror("Error", "Error opening port. Please check the port and try again.")


# Function to validate angle input
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


# Function to validate all angles
def validate_all_angles():
    flag = True
    for i, entry in enumerate(angle_entries):
        low, high = ranges.get(i, (0, 180))
        value = int(entry.get())
        if value < low or value > high:
            messagebox.showerror(f"Error Servo {i}", get_error_response(low, high))
            flag = False
    return flag


# Function to check for singularity
def check_for_singularity():
    flag = False
    s = [int(entry.get()) for entry in angle_entries]
    if s[3] >= 90 and s[4] >= 90 and s[2] >= 90:
        flag = True
    if s[2] == 180 and s[3] > 90 > s[4]:
        flag = True
    if flag:
        messagebox.showerror("Singularity Error", get_error_rresponses_for_singularity())
        return True


# Function to generate the matrix transformation
def generate_matrix(angles):
    dhs = [
        {'alpha': 0, 'a': 0, 'd': 0},
        {'alpha': math.pi / 2, 'a': 0, 'd': 0},
        {'alpha': 0, 'a': 10.5, 'd': 0},
        {'alpha': 0, 'a': 10, 'd': 0},
        {'alpha': 0, 'a': 0, 'd': 0},
    ]
    angles = [math.radians(angle) for angle in angles]
    return np.round(forward_kinematics(angles, dhs), 2)


# Function to send data to Arduino
def send_to_arduino(data):
    global arduino
    try:
        arduino = serial.Serial(COM_PORT, BAUD_RATE)
        arduino.write(data.encode())
        print(arduino.readline())
        print("Sent data to Arduino:", data)
    except serial.SerialException as e:
        print("Error connecting to Arduino:", e)


# GUI creation
root = tk.Tk()
root.title("Robotic Arm Control")
root.geometry("600x600")

# Left Frame for Forward Kinematics
left_frame = tk.Frame(root)
left_frame.pack(side="left", padx=10, pady=10)

# Right Frame for Inverse Kinematics
right_frame = tk.Frame(root)
right_frame.pack(side="right", padx=10, pady=10)

# Left Frame - Forward Kinematics
# Create canvas for drawing matrix transformation
canvas = tk.Canvas(left_frame, width=150, height=150)
canvas.pack()

# Angle labels and entries
angle_labels = []
angle_entries = []
for i in range(6):
    angle_label = tk.Label(left_frame, text=f"Servo {i}:" if i > 0 else "ignore")
    angle_label.pack()
    angle_entry = tk.Entry(left_frame, validate="key", validatecommand=(root.register(validate_angle), '%P'))
    angle_entry.insert(0, "0")
    angle_entry.pack()
    angle_labels.append(angle_label)
    angle_entries.append(angle_entry)

# Submit button
submit_button = tk.Button(left_frame, text="Submit Angles", command=update_display_and_send)
submit_button.pack()

# Right Frame - Inverse Kinematics
# X, Y, Z entries
x_label = tk.Label(right_frame, text="X:")
x_label.grid(row=0, column=0)
x_entry = tk.Entry(right_frame, validate="key", validatecommand=(root.register(validate_angle), '%P'))
x_entry.insert(0, "0")
x_entry.grid(row=0, column=1)

y_label = tk.Label(right_frame, text="Y:")
y_label.grid(row=1, column=0)
y_entry = tk.Entry(right_frame, validate="key", validatecommand=(root.register(validate_angle), '%P'))
y_entry.insert(0, "0")
y_entry.grid(row=1, column=1)

z_label = tk.Label(right_frame, text="Z:")
z_label.grid(row=2, column=0)
z_entry = tk.Entry(right_frame, validate="key", validatecommand=(root.register(validate_angle), '%P'))
z_entry.insert(0, "0")
z_entry.grid(row=2, column=1)

# Grant access button
open_button = tk.Button(right_frame, text="Grant Access", command=open_port)
open_button.grid(row=3, column=0, columnspan=2)


# Function to update display and send data

# Function to update display
def update_display(matrix):
    for y in range(4):
        for x in range(4):
            create_cell(canvas, matrix[y][x], x, y)


# Function to create a cell for matrix display
def create_cell(canvas, value, x, y, background="white", font=("Arial", 16)):
    x1 = x * 40
    y1 = y * 40
    # Draw the cell
    x2 = x1 + 40
    y2 = y1 + 40
    canvas.create_rectangle(x1, y1, x2, y2, fill=background)
    canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2, text=value, font=font)


# Function to read data from Arduino
def read_from_arduino():
    try:
        arduino = serial.Serial(COM_PORT, BAUD_RATE)
        while True:
            data = arduino.readline().decode().strip()
            print("Received data from Arduino:", data)
    except serial.SerialException as e:
        print("Error connecting to Arduino:", e)


# Start thread for reading data from Arduino
read_thread = threading.Thread(target=read_from_arduino)
read_thread.start()

# Run the GUI
root.mainloop()
