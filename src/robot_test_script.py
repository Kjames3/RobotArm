# This is the code that controls the LX16A servo using a pid controller

# import the necessary libraries
from pylx16a.lx16a import *
import time
import math

# PID Controller Class
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


# Initialize the LX-16A servo controller
LX16A.initialize("COM4")  # Adjust the port as necessary (e.g., "/dev/ttyUSB0" on Linux)

# Create servo objects for IDs 1, 2, and 3
# servos = [LX16A(1), LX16A(2), LX16A(3)]
# servos = [LX16A(i) for i in range(1, 6)]

baseServo = LX16A(1)
armServo1 = LX16A(2)
armServo2 = LX16A(3)
wristServo = LX16A(4)
gripperServo = LX16A(5)

servos = [baseServo, armServo1, armServo2, wristServo, gripperServo]

baseServo.set_angle_limits(0, 240)
armServo1.set_angle_limits(0, 180)
armServo2.set_angle_limits(0, 180)
wristServo.set_angle_limits(0, 180)
gripperServo.set_angle_limits(0, 240)


# Set angle limits for each servo
# for servo in servos:
#     servo.set_angle_limits(0, 240)


def test_all_servos(servos):
    """
    Tests the range of motion for each servo by moving it from 0 to 240 degrees and back.
    Each movement takes 5 seconds, with a 6-second pause between movements.
    """
    for servo in servos:
        print(f"Testing servo {servo.id}")
        
        # Move to minimum position (0 degrees) over 5 seconds
        servo.move(0)  # Move to 0 degrees in 5 seconds
        time.sleep(6)  # Wait for movement to complete plus a bit
        
        # Move to maximum position (240 degrees) over 5 seconds
        servo.move(240)  # Move to 240 degrees in 5 seconds
        time.sleep(6)  # Wait for movement to complete plus a bit
        
        # Move back to minimum position (0 degrees) over 5 seconds
        servo.move(0)  # Move back to 0 degrees in 5 seconds
        time.sleep(6)  # Wait for movement to complete plus a bit
    
    print("Range of motion test completed for all servos.")

# Create PID controllers for each servo with example gains (adjust as needed)
# pids = [PID(1.0, 0.0, 0.1) for _ in range(3)]

# Define target configurations: each is a dictionary with 'angles' and 'time'
targets = [
    {'angles': [0, 0, 0], 'time': 2},    # Move to [0, 0, 0] in 2 seconds
    {'angles': [90, 0, 0], 'time': 2},   # Move to [90, 0, 0] in 2 seconds
    {'angles': [90, 90, 0], 'time': 2},  # Move to [90, 90, 0] in 2 seconds
    {'angles': [90, 90, 90], 'time': 2}, # Move to [90, 90, 90] in 2 seconds
    {'angles': [0, 0, 0], 'time': 2},    # Return to [0, 0, 0] in 2 seconds
]

try:
    test_all_servos(servos)
except KeyboardInterrupt:
    print("User interrupted the program.")
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
except KeyboardInterrupt:
    print("User interrupted the program.")
finally:
    # Stop all servos
    for servo in servos:
        #servo.stop()
        quit()