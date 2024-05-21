'''
@file main.py Entry Point
This file contains the main function which creates the GUI and reads data from the Arduino.

@author: Jaysh Khan
'''

import serial

from GUI.gui import App


def main():
    app = App()
    app.mainloop()
    return app


'''
This function reads data from the Arduino and prints it to the console.
@param app: The App GUI object which contains the COM port and baud rate.
'''


def read_from_arduino(app):
    try:
        arduino = serial.Serial(app.COM_PORT, app.BAUD_RATE)
        while True:
            data = arduino.readline().decode().strip()
            print("Received data from Arduino:", data)
    except serial.SerialException as e:
        print("Error connecting to Arduino:", e)


if __name__ == "__main__":
    app = main()
    read_from_arduino(app)
