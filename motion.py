import serial
import time
from statistics import mean 

class IRobotMotion:
    def open(self):
        pass
    def close(self):
        pass
    def move(self, x_speed, y_speed, rot_speed):
        pass

class PrintingMotion(IRobotMotion):
    ## change polarity if direction is not correct
    def __init__(self, polarity=1):
        self.polarity = polarity
    
    def open(self):
        print("Starting up!")

    def close(self):
        print("Shutting down...")

    # simple logic to print what the mainboard would receive
    def move(self, x_speed, y_speed, rot_speed):
        direction = "left" if rot_speed * self.polarity > 0 else "right"
        print(f"Rotation direction: {direction};")


class OmniMotionRobot(IRobotMotion):
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None

    def open(self):
        """Opens the serial port connection."""
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            print(f"Serial connection opened on {self.port}")
        except Exception as e:
            print(f"Failed to open serial port: {e}")

    def close(self):
        """Closes the serial port connection."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print(f"Serial connection on {self.port} closed")

    def send_command(self, command):
        """Sends a command to the robot's mainboard via serial."""
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.write(command.encode())
                print(f"Command sent: {command}")
            except Exception as e:
                print(f"Failed to send command: {e}")
        else:
            print("Serial connection is not open")

    def move(self, x_speed, y_speed, rot_speed):
        """Command to rotate in place."""
        if rot_speed != 0:
            direction = "L" if rot_speed > 0 else "R"
            speed = abs(rot_speed)
            command = f"ROT {direction} {speed}"
            self.send_command(command)
            print(f"Moving: rotation speed = {rot_speed}")
        else:
            self.send_command("STOP")
            print("Stopping rotation")
