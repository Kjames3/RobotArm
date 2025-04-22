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
servos = [LX16A(1), LX16A(2), LX16A(3)]

# Set angle limits for each servo
for servo in servos:
    servo.set_angle_limits(0, 240)

# Create PID controllers for each servo with example gains (adjust as needed)
pids = [PID(1.0, 0.0, 0.1) for _ in range(3)]

# Define target configurations: each is a dictionary with 'angles' and 'time'
targets = [
    {'angles': [0, 0, 0], 'time': 2},    # Move to [0, 0, 0] in 2 seconds
    {'angles': [90, 0, 0], 'time': 2},   # Move to [90, 0, 0] in 2 seconds
    {'angles': [90, 90, 0], 'time': 2},  # Move to [90, 90, 0] in 2 seconds
    {'angles': [90, 90, 90], 'time': 2}, # Move to [90, 90, 90] in 2 seconds
    {'angles': [0, 0, 0], 'time': 2},    # Return to [0, 0, 0] in 2 seconds
]

try:
    for target in targets:
        # Get current angles
        start_angles = [servo.get_physical_angle() for servo in servos]
        target_angles = target['angles']
        T = target['time']
        start_time = time.time()
        last_time = start_time
        t = 0
        while t < T:
            current_time = time.time()
            dt = current_time - last_time
            if dt <= 0:
                continue
            last_time = current_time
            t = current_time - start_time
            if t > T:
                t = T
            for i in range(3):
                # Compute desired angle at time t
                theta_d = start_angles[i] + (target_angles[i] - start_angles[i]) * (t / T)
                # Read current angle
                theta = servos[i].get_physical_angle()
                # Compute error
                error = theta_d - theta
                # Compute PID output
                u = pids[i].update(error, dt)
                # Compute command angle
                theta_command = theta_d + u
                # Clamp to angle limits
                theta_command = max(0, min(240, theta_command))
                # Send command to servo
                servos[i].move(theta_command)
            # Sleep to maintain approximate loop frequency
            time.sleep(0.01)
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
except KeyboardInterrupt:
    print("User interrupted the program.")
finally:
    # Stop all servos
    for servo in servos:
        servo.stop()
    quit()