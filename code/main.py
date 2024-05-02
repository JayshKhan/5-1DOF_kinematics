import tkinter as tk
from tkinter import ttk
import customtkinter as ctk
import numpy as np

from forwardKinematic import forward_kinematics
import serial  # Import the serial library for Arduino communication
import math

# Constants
COM_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600


# Function to validate angle input (ensures values are within servo range)
def validate_angle(text):
    try:
        if text == "":
            return True
        value = int(text)
        if 0 <= value <= 180:  # Adjust the range based on your servo specifications
            return True
        else:
            return False
    except ValueError:
        return False


# Function to generate the matrix transformation (replace with your specific logic)
def generate_matrix(angles):
    dhs = [
        {'alpha': 0, 'a': 0, 'd': 0},
        {'alpha': math.pi / 2, 'a': 0, 'd': 0},
        {'alpha': 0, 'a': 10.5, 'd': 0},
        {'alpha': 0, 'a': 10, 'd': 0},
        {'alpha': math.pi / 2, 'a': 0, 'd': 0},
        {'alpha': 0, 'a': 0, 'd': 16.5},
        {'alpha': 0, 'a': 0, 'd': 0},
    ]
    return np.round(forward_kinematics(angles, dhs), 2)


# Function to send data to Arduino via serial communication
def send_to_arduino(data):
    try:
        # Replace with your Arduino's serial port and baud rate
        arduino = serial.Serial(COM_PORT, BAUD_RATE)  # Adjust port and baud rate accordingly
        arduino.write(data.encode())
        arduino.close()
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
root.geometry("300x400")
root.configure(bg="black")
ctk.set_appearance_mode("Dark")

cell_size = 30
padding = 5


# Define function to create a single cell
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


# Function called on button press to update display and send data to Arduino
def update_display_and_send():
    angles = []

    for entry in angle_entries:
        angles.append(int(entry.get()))

    # Generate the matrix transformation using the provided angles
    matrix = generate_matrix(angles)
    print(matrix)

    # Update the display with the matrix
    update_display(matrix)
    # ignore the first servo
    angles = angles[1:]
    # add 0 to the end
    angles.append(0)
    angles.append(0)

    # Prepare data to send to Arduino (replace with your format)
    data_to_send = ",".join(str(angle) for angle in angles) + "\n"
    print(data_to_send)

    # Send data to Arduino
    send_to_arduino(data_to_send)


import threading
import time


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
