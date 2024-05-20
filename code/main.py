import serial

from GUI.gui import App

def main():
    app = App()
    app.mainloop()
    return app

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