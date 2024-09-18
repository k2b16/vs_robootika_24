from statistics import mean 
import serial
import math
import struct

class IRobotMotion:
    def open(self):
        pass
    def close(self):
        pass
    def move(self, ratas1, ratas2, ratas3, rot_speed):
        pass

class PrintingMotion(IRobotMotion):
    ## change polarity if direction is not correct
    def __init__(self, polarity=1):
        self.polarity = polarity
    
    def open(self):
        print("Starting up!")

    def close(self):
        print("Shutting down...")


    def move(self, ratas1, ratas2, rot_speed):
        direction = "left" if rot_speed * self.polarity > 0 else "right"
        print(f"Rotation direction: {direction};")

class OmniMotionRobot(IRobotMotion):
    def __init__(self, port="/dev/ttyACM0", wheel_distance_from_center=0.2):
        self.port = port
        self.serial_connection = ""
        self.wheel_distance_from_center = wheel_distance_from_center

        self.wheel_angles = [0, 120, 240]


    def open(self):
        """Opens the serial port connection."""
        try:
            self.serial_connection = serial.Serial(
                port=self.port
            )
            print(f"Serial connection opened on {self.port}")
        except Exception as e:
            print(f"Failed to open serial port: {e}")

    def close(self):
        """Closes the serial port connection."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print(f"Serial connection on {self.port} closed")

    def send_command(self, speed1, speed2, speed3, thrower_speed, servo1, servo2, disable_failsafe=0):
        """Sends a command to the robot's mainboard via serial, following the firmware's struct."""
        delimiter = 0xAAAA  # fixed delimiter
        try:
            # Pack the data according to the firmware struct format
            packed_command = struct.pack(
                '<hhhHHHBH',  # format: 3 int16, 2 uint16, 1 uint8, 1 uint16
                speed1, speed2, speed3, thrower_speed, servo1, servo2, disable_failsafe, delimiter
            )
            self.serial_connection.write(packed_command)
            print(f"Command sent: speed1={speed1}, speed2={speed2}, speed3={speed3}, thrower_speed={thrower_speed}, servo1={servo1}, servo2={servo2}, disable_failsafe={disable_failsafe}")
        except Exception as e:
            print(f"Failed to send command: {e}")

    def calculate_wheel_velocities(self, robotSpeedX, robotSpeedY, robotAngularVelocity):
        """Calculates the linear velocity for each wheel based on robot movement."""
        robotSpeed = math.sqrt(robotSpeedX**2 + robotSpeedY**2)
        robotDirectionAngle = math.atan2(robotSpeedY, robotSpeedX)  # in radians

        wheel_velocities = []
        for wheel_angle_deg in self.wheel_angles:
            wheel_angle = math.radians(wheel_angle_deg)
            
            wheelLinearVelocity = (
                robotSpeed * math.cos(robotDirectionAngle - wheel_angle) + 
                self.wheel_distance_from_center * robotAngularVelocity
            )
            wheel_velocities.append(wheelLinearVelocity)

        return wheel_velocities


    def receive_feedback(self):
        """Receives feedback data from the robot's mainboard and unpacks it."""
        try:
            if self.serial_connection.in_waiting > 0:
                # Reading the expected size of the feedback struct (13 bytes: 6 int16, 1 uint8, 1 uint16)
                feedback_data = self.serial_connection.read(13)
                actual_speed1, actual_speed2, actual_speed3, motor1_position, motor2_position, motor3_position, sensors, feedback_delimiter = struct.unpack(
                    '<hhhhhhBH', feedback_data
                )
                print(f"Feedback received: speed1={actual_speed1}, speed2={actual_speed2}, speed3={actual_speed3}, "
                      f"motor1_position={motor1_position}, motor2_position={motor2_position}, motor3_position={motor3_position}, "
                      f"sensors={sensors}, delimiter={feedback_delimiter}")
                return actual_speed1, actual_speed2, actual_speed3, motor1_position, motor2_position, motor3_position, sensors, feedback_delimiter
            else:
                print("No feedback available")
        except Exception as e:
            print(f"Failed to receive feedback: {e}")
        return None

    def move(self, robotSpeedX, robotSpeedY, robotAngularVelocity):
        """Command to rotate in place."""
        wheel_velocities = self.calculate_wheel_velocities(robotSpeedX, robotSpeedY, robotAngularVelocity)

        speed1, speed2, speed3 = wheel_velocities

        
        """speed1 = 0 #Back wheel
        speed2 = 0  #Right wing
        speed3 = 0 #Left wing"""
        thrower = 1000
        servo1 = 0 #1500
        servo2 = 0 #1500
        disable_failsafe = 0

        self.send_command(int(speed1), int(speed2), int(speed3), thrower, servo1, servo2, disable_failsafe)

        #feedback = self.receive_feedback()
        #if feedback:
        #   actual_speed1, actual_speed2, actual_speed3, motor1_position, motor2_position, motor3_position, sensors, feedback_delimiter = feedback