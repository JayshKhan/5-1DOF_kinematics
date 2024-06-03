'''
This file contains the GUI class for the servo control GUI.
The GUI has 3 tabs:
- Forward Kinematics
- Inverse Kinematics
- Settings

The Forward Kinematics tab contains:
- Entry fields for the angles of the servos
- A submit button to submit the angles to the robot arm
- Display of the end-effector position

The Inverse Kinematics tab contains:
- Entry fields for the X, Y, Z coordinates of the end-effector
- Entry field for the phi change
- A button to do the inverse kinematics
- Display of the angles of the servos
- Display of the end-effector position from the forward kinematics

The Settings tab contains:
- Entry fields for the COM port and baud rate for serial communication
- Option menu to select the servo mode (Sequential or Simultaneous)
- A save button to save the settings

@author: Jaysh Khan

'''

import tkinter as tk
from tkinter import ttk

import customtkinter as ctk
from kinematics.Kinematics import Kinematic  # ignore if shows error. import is working

# import functions from this module
from . import functions

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("green")
COLOR_GREEN = "#26d663"
COLOR_RED = "#dd0202"


class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.servo_mode_entry = None
        self.current_angles = [10, 10, 10, 10, 10, 10]
        self.port_entry = None
        self.baud_entry = None
        self.xyz_label_verify = None
        self.phi_change = 1
        self.kinematic = Kinematic(self.phi_change)
        # Variables
        self.COM_PORT = '/dev/ttyACM0'
        self.BAUD_RATE = 9600
        self.arduino = None
        self.servo_mode = "Sequential"

        self.cell_size = 30
        self.padding = 5
        self.angle_print = None
        self.phi_entry = None
        self.z_entry = None
        self.z_label = None
        self.phi_label = None
        self.y_entry = None
        self.y_label = None
        self.x_entry = None
        self.xyz_label = None
        self.canvas = None
        self.angle_entries = None
        self.angle_labels = None
        self.x_label = None
        self.Style = ttk.Style()
        self.Style.theme_create(themename="yummy", parent="alt", settings={
            "TNotebook": {
                "configure": {
                    "tabmargins": [2, 5, 2, 0],
                    "background": COLOR_GREEN,
                    "foreground": "black"
                }},
            "TNotebook.Tab": {
                "configure": {"padding": [5, 1]},
                "map": {"background": [("selected", COLOR_GREEN)],
                        "expand": [("selected", [1, 1, 1, 0])]}}})
        self.Style.theme_use(themename="yummy")
        self.Style.configure(
            style="primary.TNotebook",
            tabposition="nw",
            background="grey",
            foreground="black",
            borderwidth=2, darkcolortheme=True, lightcolortheme=False)

        self.title("Servo Control GUI")
        self.geometry("800x600")

        self.notebook = ttk.Notebook(self, style="primary.TNotebook")
        self.matplotlib_plot = None
        self.matplotlib_canvas = None
        self.create_fk_frame()
        self.create_ik_frame()
        self.create_draw_frame()
        self.create_setting_frame()

        self.create_port_access_button()

        self.notebook.pack(expand=1, fill="both")

    def create_fk_frame(self):
        fk_frame = tk.Frame(self.notebook)
        self.notebook.add(fk_frame, text="Forward Kinematics")

        l_fk_frame = tk.Frame(fk_frame)
        l_fk_frame.grid(row=0, column=0, sticky="nsew")

        r_fk_frame = tk.Frame(fk_frame)
        r_fk_frame.grid(row=0, column=1, sticky="ne")

        self.angle_labels = []
        self.angle_entries = []
        for i in range(6):
            angle_label = ctk.CTkLabel(l_fk_frame, text=f"Servo {i}:" if i > 0 else "ignore", font=("Arial", 12),
                                       text_color=("white", "black")
                                       )
            angle_label.grid(row=i, column=0, padx=5, pady=5)
            angle_entry = ctk.CTkEntry(l_fk_frame,
                                       validate="key",
                                       validatecommand=(self.register(functions.validate_angle), '%P'),
                                       width=100)
            angle_entry.insert(0, "0")
            angle_entry.grid(row=i, column=1, padx=5, pady=5)
            # hide the first angle
            if i == 0:
                angle_label.grid_remove()
                angle_entry.grid_remove()
            self.angle_labels.append(angle_label)
            self.angle_entries.append(angle_entry)

        submit_button = ctk.CTkButton(l_fk_frame, text="Submit Angles",
                                      corner_radius=10,
                                      width=300,
                                      command=self.submit_angles)
        submit_button.grid(row=6, column=0, columnspan=2, pady=10)

        self.canvas = tk.Canvas(r_fk_frame, width=150, height=150)
        self.canvas.grid(row=7, column=0, pady=10, columnspan=2)

        self.xyz_label = ctk.CTkLabel(r_fk_frame, text="X: 0 \nY: 0 \nZ: 0", text_color=("white", "black"))
        self.xyz_label.grid(row=8, column=0, columnspan=2)

    def create_ik_frame(self):
        ik_frame = tk.Frame(self.notebook)
        self.notebook.add(ik_frame, text="Inverse Kinematics")

        l_ik_frame = tk.Frame(ik_frame, width=300)
        self.matplotlib_plot = tk.Frame(ik_frame, width=300)
        l_ik_frame.grid(row=0, column=0, sticky="nsew")
        self.matplotlib_plot.grid(row=0, column=1, sticky="ne")

        self.x_label = ctk.CTkLabel(l_ik_frame, text="X:", text_color=("white", "black"))
        self.x_label.grid(row=9, column=0)
        self.x_entry = ctk.CTkEntry(l_ik_frame, validate="key",
                                    validatecommand=(self.register(functions.validate_coords), '%P'))
        self.x_entry.insert(0, "0")
        self.x_entry.grid(row=9, column=1)

        self.y_label = ctk.CTkLabel(l_ik_frame, text="Y:", text_color=("white", "black"))
        self.y_label.grid(row=10, column=0)
        self.y_entry = ctk.CTkEntry(l_ik_frame, validate="key",
                                    validatecommand=(self.register(functions.validate_coords), '%P'))
        self.y_entry.insert(0, "0")
        self.y_entry.grid(row=10, column=1)

        self.z_label = ctk.CTkLabel(l_ik_frame, text="Z:", text_color=("white", "black"))
        self.z_label.grid(row=11, column=0)
        self.z_entry = ctk.CTkEntry(l_ik_frame, validate="key",
                                    validatecommand=(self.register(functions.validate_coords), '%P'))
        self.z_entry.insert(0, "0")
        self.z_entry.grid(row=11, column=1)

        self.phi_label = ctk.CTkLabel(l_ik_frame, text="Phi Change:", text_color=("white", "black"))
        self.phi_label.grid(row=12, column=0)
        self.phi_entry = ctk.CTkEntry(l_ik_frame, validate="key",
                                      validatecommand=(self.register(functions.validate_angle), '%P'))
        self.phi_entry.insert(0, "10")
        self.phi_entry.grid(row=12, column=1)

        # button to do inverse Kinematic
        inverse_button = ctk.CTkButton(l_ik_frame, text="Do Inverse Kinematic",
                                       corner_radius=10,
                                       width=300,

                                       command=lambda: self.inverse_update_display_and_send())
        inverse_button.place(relx=0.5, rely=0.8, anchor=tk.CENTER)
        inverse_button.grid(row=13, column=0, columnspan=2, pady=10)

        self.angle_print = ctk.CTkLabel(l_ik_frame, text="Servo 1: 0\nServo 2: 0\nServo 3: 0\nServo 4: 0",
                                        text_color=("white", "black"))
        self.angle_print.grid(row=14, column=0, columnspan=2)

        self.xyz_label_verify = ctk.CTkLabel(l_ik_frame, text="From Forward: X: 0 Y: 0 Z: 0",
                                             text_color=("white", "black"))
        self.xyz_label_verify.grid(row=15, column=0, columnspan=2)

    def create_draw_frame(self):
        d_frame = tk.Frame(self.notebook)
        self.notebook.add(d_frame, text="Draw")


        open_cv2_button = ctk.CTkButton(d_frame, text="Open CV2 Canvas",
                                        corner_radius=10,
                                        width=300,
                                        command=lambda: functions.open_cv2_canvas())
        open_cv2_button.grid(row=1, column=0, columnspan=2, pady=10)






    def create_port_access_button(self):
        open_button = ctk.CTkButton(self, text="Grant Access",
                                    corner_radius=10,
                                    width=300,
                                    command=lambda: functions.open_port(self))
        open_button.pack(side=tk.BOTTOM, pady=10)

    def submit_angles(self):
        functions.submit_angles(self)

    def inverse_update_display_and_send(self):
        functions.inverse_update_display_and_send(self)

    def create_setting_frame(self):
        settings_frame = tk.Frame(self.notebook)
        self.notebook.add(settings_frame, text="Settings")

        port_label = ctk.CTkLabel(settings_frame, text="Port:", text_color=("white", "black"))
        port_label.grid(row=0, column=0, pady=10)
        self.port_entry = ctk.CTkEntry(settings_frame)
        self.port_entry.insert(0, self.COM_PORT)
        self.port_entry.grid(row=0, column=1, pady=10)

        baud_label = ctk.CTkLabel(settings_frame, text="Baud Rate:", text_color=("white", "black"))
        baud_label.grid(row=1, column=0, pady=10)
        self.baud_entry = ctk.CTkEntry(settings_frame)
        self.baud_entry.insert(0, self.BAUD_RATE)
        self.baud_entry.grid(row=1, column=1, pady=10)

        # move Servo Sequentially / Simultaneously
        servo_mode_label = ctk.CTkLabel(settings_frame, text="Servo Mode:", text_color=("white", "black"))
        servo_mode_label.grid(row=2, column=0, pady=10)
        self.servo_mode_entry = ctk.CTkOptionMenu(settings_frame, values=["Sequential", "Simultaneous"])
        self.servo_mode_entry.set("Sequential")
        self.servo_mode_entry.grid(row=2, column=1, pady=10)

        save_button = ctk.CTkButton(settings_frame, text="Save", corner_radius=10, width=300
                                    , command=lambda: self.save_settings())
        save_button.grid(row=4, column=0, columnspan=2, pady=10)

    def save_settings(self):
        self.BAUD_RATE = self.baud_entry.get()
        self.COM_PORT = self.port_entry.get()
        self.servo_mode = self.servo_mode_entry.get()

    def update_phi(self, phi):
        self.phi_change = phi
        self.kinematic.update_phi(phi)
