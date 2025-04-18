import time
from Rosmaster_Lib import Rosmaster
import serial.serialutil

class PidController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, setpoint, measured_value):
        curent_time = time.time()
        dt = curent_time - self.last_time
        # Prevent division by zero
        if dt == 0:
            return 0

        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        self.last_time = curent_time

        # Cap the output motor speed to the range [-100, 100]
        return max(min(output, 100), -100)

# Encoder settings
PULSES_PER_REVOLUTION = 748
DEGREES_PER_PULSE = 360 / PULSES_PER_REVOLUTION
PULSES_FOR_180_DEGREES = int(180 / DEGREES_PER_PULSE)

# PID parameters (I need to figure these out again)
KP = 0.1
KI = 0.01
KD = 0.05

try:
    # Initialize Rosmaster
    print("Initializing ROSMaster...")
    ros = Rosmaster()
    print("ROSMaster initialized successfully")

    # # Reset encoder to zero
    # ros.reset_motor_encoder()
    # print("Encoder reset to zero")

    # Read initial encoder count (for motor 1)
    initial_count = ros.get_motor_encoder()[0]
    print(f"Initial encoder count: {initial_count}")

    # Initialize PID controller
    pid = PIDController(KP, KI, KD)

    try:
        while True:
            # Move to 180 degrees clockwise
            print("Moving to 180 degrees clockwise")
            target = initial_count + PULSES_FOR_180_DEGREES
            while True:
                current = ros.get_motor_encoder()[0]
                if abs(target - current) < 5:  # Within 5 pulses (~3 degrees)
                    break
                speed = pid.compute(target, current)
                ros.set_motor(speed, 0, 0, 0)
                time.sleep(0.01)
            ros.set_motor(0, 0, 0, 0)  # Stop
            print("Reached 180 degrees, stopping for 2 seconds")
            time.sleep(2)

            # Move back to 0 degrees counter-clockwise
            print("Moving to 0 degrees counter-clockwise")
            target = initial_count
            while True:
                current = ros.get_motor_encoder()[0]
                if abs(target - current) < 5:
                    break
                speed = pid.compute(target, current)
                ros.set_motor(speed, 0, 0, 0)
                time.sleep(0.01)
            ros.set_motor(0, 0, 0, 0)  # Stop
            print("Reached 0 degrees, stopping for 2 seconds")
            time.sleep(2)

    except KeyboardInterrupt:
        print("Program interrupted")
        ros.set_motor(0, 0, 0, 0)  # Ensure motor stops
        print("Motor stopped")

except serial.serialutil.SerialException as e:
    print(f"Serial port error: {e}")
    print("Ensure the ROSMaster board is connected and /dev/myserial links to /dev/ttyUSB0.")
    print("Run: sudo ln -sf /dev/ttyUSB0 /dev/myserial")
except Exception as e:
    print(f"Unexpected error: {e}")
    print("Check ROSMaster board connection, firmware, or library installation.")
finally:
    # Ensure motor is stopped
    try:
        ros.set_motor(0, 0, 0, 0)
    except:
        pass