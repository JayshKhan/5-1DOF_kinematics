"""
This code is a simple GUI for controlling a robotic arm using servos.
The GUI allows the user to input angles for each
servo and sends the data to an Arduino via serial communication.
The Arduino then controls the servos based on the input angles.
The GUI also displays the matrix transformation based on the input angles.
The code uses the Tkinter library for the GUI and the PySerial library for serial communication.

@Author: Jaysh Khan

"""

import tkinter as tk
from tkinter import ttk
import customtkinter as ctk
import numpy as np
from tkinter import messagebox

from forwardKinematic import forward_kinematics
from responses import get_error_response, get_error_rresponses_for_singularity
# from testingCodeVariations import Mforward_kinematics
import serial  # Import the serial library for Arduino communication
import math

import subprocess

# for reading from the arduino
import threading

# Constants
COM_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
arduino = None

# Servo Angle Ranges
ranges = {
    1: (0, 180),
    2: (0, 180),
    3: (0, 150),
    4: (0, 180),
    5: (0, 180),
    6: (0, 180),
}


def open_port():
    global arduino
    # giving permission to access the port
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


def validate_all_angles():
    flag = True
    #     check the entries for the angles and see if they are within the range
    for i, entry in enumerate(angle_entries):
        # from the ranges dict get the range for the servo
        low, high = ranges.get(i, (0, 180))
        value = int(entry.get())
        if value < low or value > high:
            messagebox.showerror(f"Error Servo {i}", get_error_response(low, high))
            flag = False
    return flag


def check_for_singularity():
    flag = False
    s = []
    for i, entry in enumerate(angle_entries):
        s.append(int(entry.get()))

    print(s[2], s[3], s[4])
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


# Function to send data to Arduino via serial communication
def send_to_arduino(data):
    global arduino
    try:
        arduino = serial.Serial(COM_PORT, BAUD_RATE)
        arduino.write(data.encode())
        print(arduino.readline())
        # arduino.close()
        print("Sent data to Arduino:", data)
    except serial.SerialException as e:
        print("Error connecting to Arduino:", e)


# Function to update the GUI display with the matrix transformation
def update_display(matrix):
    for y in range(4):
        for x in range(4):
            create_cell(canvas, matrix[y][x], x, y)


"""
||||||||||||||||||||||| The GUI |||||||||||||||||||||||
"""
# Main GUI creation and functionality
root = tk.Tk()
root.title("Servo Control GUI")
root.geometry("300x500")
root.configure(bg="black")
ctk.set_appearance_mode("Dark")

cell_size = 30
padding = 5


# Define function to create a single cell for the matrix display
def create_cell(canvas, value, x, y, background="white", font=("Arial", 16)):
    x1 = x * (cell_size + padding)
    y1 = y * (cell_size + padding)
    x2 = x1 + cell_size
    y2 = y1 + cell_size
    canvas.create_rectangle(x1, y1, x2, y2, fill=background)
    canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2, text=value, font=("Arial", 10), fill="black")


# Create a canvas for drawing the matrix transformation
canvas = tk.Canvas(root, width=4 * cell_size + 3 * padding, height=4 * cell_size + 3 * padding)
canvas.grid(row=7, column=0, pady=10)

# Create labels and entry field
angle_labels = []
angle_entries = []

# angles
for i in range(6):
    angle_label = ctk.CTkLabel(root, text=f"Servo {i}:" if i > 0 else "ignore", fg_color=("white", "black"))
    angle_label.grid(row=i, column=0)
    angle_entry = ctk.CTkEntry(root,
                               validate="key",
                               validatecommand=(root.register(validate_angle), '%P'))
    angle_entry.insert(0, "0")
    angle_entry.grid(row=i, column=1)
    angle_labels.append(angle_label)
    angle_entries.append(angle_entry)

# Create a button to submit angles and update display
submit_button = ctk.CTkButton(root, text="Submit Angles",
                              corner_radius=10,
                              width=300,

                              command=lambda: update_display_and_send())
submit_button.place(relx=0.5, rely=0.8, anchor=tk.CENTER)
submit_button.grid(row=6, column=0, columnspan=2, pady=10)

xyz = ctk.CTkLabel(root, text="X: 0 \nY: 0 \nZ: 0", fg_color=("white", "black"))
xyz.grid(row=7, column=1, columnspan=2)

# button to open port
open_button = ctk.CTkButton(root, text="Grant Access",
                            corner_radius=10,
                            width=300,

                            command=lambda: open_port())
open_button.place(relx=0.5, rely=0.8, anchor=tk.CENTER)
open_button.grid(row=8, column=0, columnspan=2, pady=10)


# Function called on button press to update display and send data to Arduino
def update_display_and_send():
    if validate_all_angles() and not check_for_singularity():
        angles = []

        for entry in angle_entries:
            angles.append(int(entry.get()))

        # Generate the matrix transformation using the provided angles
        matrix = generate_matrix(angles)
        xyz.configure(text=f"X: {matrix[0][3]} \nY: {matrix[1][3]} \nZ: {matrix[2][3]}")
        print(matrix)

        # Update the display with the matrix
        update_display(matrix)

        """
        The following code is just to match the number of servos with the number of angles
        that arduino serial communication expects
        """
        # ignore the first servo frame 0 which is fixed frame
        angles = angles[1:]
        # add 0 to the end
        angles.append(0)
        angles.append(0)

        # Prepare data to send to Arduino (replace with your format)
        data_to_send = ",".join(str(angle) for angle in angles) + "\n"
        print(data_to_send)

        # Send data to Arduino
        send_to_arduino(data_to_send)


def read_from_arduino():
    try:
        arduino = serial.Serial(COM_PORT, BAUD_RATE)
        while True:
            data = arduino.readline().decode().strip()
            print("Received data from Arduino:", data)
    except serial.SerialException as e:
        print("Error connecting to Arduino:", e)


# Create a thread for reading data from Arduino
read_thread = threading.Thread(target=read_from_arduino)
print("Starting read thread")
read_thread.start()
# End of thread creation
root.mainloop()
# End of GUI code

# add a thread to run the Serial communication read
