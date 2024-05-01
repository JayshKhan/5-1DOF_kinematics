import tkinter as tk
from tkinter import ttk
import customtkinter as ctk
from testingCodeVariations import forward_kinematics
import serial  # Import the serial library for Arduino communication
import math


# Function to validate angle input (ensures values are within servo range)
def validate_angle(text):
    try:
        value = int(text)
        if 0 <= value <= 180:  # Adjust the range based on your servo specifications
            return True
        else:
            return False
    except ValueError:
        return False


# Function to generate the matrix transformation (replace with your specific logic)
def generate_matrix(angles, a, d, alpha):
    return forward_kinematics(angles, a, d, alpha)


# Function to send data to Arduino via serial communication
def send_to_arduino(data):
    try:
        # Replace with your Arduino's serial port and baud rate
        arduino = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust port and baud rate accordingly
        arduino.write(data.encode())
        arduino.close()
        print("Sent data to Arduino:", data)
    except serial.SerialException as e:
        print("Error connecting to Arduino:", e)


# Function to update the GUI display with the matrix transformation
def update_display(matrix):
    display_text.set("Matrix Transformation:\n")
    for row in matrix:
        # restric val to 2 decimal places

        display_text.set(display_text.get() + " || ".join([str(round(val, 2)) for val in row]) + "\n")


# Main GUI creation and functionality
root = tk.Tk()
root.title("Servo Control GUI")
root.geometry("1000x500")
root.configure(bg="black")
ctk.set_appearance_mode("Dark")

# Create labels and entry fields
a_labels = []
a_entries = []

alpha_labels = []
alpha_entries = []

d_labels = []
d_entries = []

angle_labels = []
angle_entries = []

# DH parameters
a = [0, 0, 10.5, 10, 0, 0]
alpha = [0, 90, 0, 0, 90, 0]
d = [0, 0, 0, 0, 0, 14]

# a
for i in range(6):
    a_label = ctk.CTkLabel(root, text=f"a {i + 1}:", fg_color=("white", "black"))
    a_label.grid(row=i, column=1)
    a_entry = ctk.CTkEntry(root)
    a_entry.insert(0, str(a[i]))
    a_entry.grid(row=i, column=2)
    a_labels.append(a_label)
    a_entries.append(a_entry)

# alpha
for i in range(6):
    alpha_label = ctk.CTkLabel(root, text=f"Alpha {i + 1}:", fg_color=("white", "black"))
    alpha_label.grid(row=i, column=3)
    alpha_entry = ctk.CTkEntry(root)
    alpha_entry.insert(0, str(alpha[i]))
    alpha_entry.grid(row=i, column=4)
    alpha_labels.append(alpha_label)
    alpha_entries.append(alpha_entry)

# d
for i in range(6):
    d_label = ctk.CTkLabel(root, text=f"d {i + 1}:", fg_color=("white", "black"))
    d_label.grid(row=i, column=5)
    d_entry = ctk.CTkEntry(root)
    d_entry.insert(0, str(d[i]))
    d_entry.grid(row=i, column=6)
    d_labels.append(d_label)
    d_entries.append(d_entry)

# angles
for i in range(6):
    angle_label = ctk.CTkLabel(root, text=f"Angle {i + 1}:", fg_color=("white", "black"))
    angle_label.grid(row=i, column=7)
    angle_entry = ctk.CTkEntry(root,
                               validate="key",
                               validatecommand=(root.register(validate_angle), '%P'))
    angle_entry.insert(0, "0")
    angle_entry.grid(row=i, column=8)
    angle_labels.append(angle_label)
    angle_entries.append(angle_entry)

# Create a button to submit angles and update display
submit_button = ctk.CTkButton(root, text="Submit Angles",
                              corner_radius=10,
                              command=lambda: update_display_and_send())
submit_button.place(relx=0.5, rely=0.8, anchor=tk.CENTER)
submit_button.grid(row=1, column=9, columnspan=2)

# Create a text area to display the matrix transformation
display_text = tk.StringVar()
display_label = ctk.CTkLabel(root, textvariable=display_text)
display_label.grid(row=7, column=0, columnspan=5)


# Function called on button press to update display and send data to Arduino
def update_display_and_send():
    angles = []
    a = []
    alpha = []
    d = []

    for entry in angle_entries:
        angles.append(int(entry.get()))
    for entry in a_entries:
        a.append(float(entry.get()))
    for entry in alpha_entries:
        alpha.append(float(entry.get()))
    for entry in d_entries:
        d.append(float(entry.get()))

    # Generate the matrix transformation using the provided angles
    matrix = generate_matrix(angles, a, d, alpha)
    print(matrix)

    # Update the display with the matrix
    update_display(matrix)

    # Prepare data to send to Arduino (replace with your format)
    data_to_send = ",".join(str(angle) for angle in angles) + "\n"
    print(data_to_send)

    # Send data to Arduino
    send_to_arduino(data_to_send)


root.mainloop()
