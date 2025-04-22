import time
from Rosmaster_Lib import Rosmaster
import serial.serialutil

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, setpoint, measured_value):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt == 0:  # Prevent division by zero
            return 0

        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        self.last_time = current_time

        # Cap output to motor speed range (-100 to 100)
        return max(min(output, 100), -100)

# Encoder settings (adjust based on motor specs)
PULSES_PER_REVOLUTION = 748  # Adjust this based on your motor's specifications
DEGREES_PER_PULSE = 360 / PULSES_PER_REVOLUTION
PULSES_FOR_180_DEGREES = int(180 / DEGREES_PER_PULSE)  # ~374 pulses

# PID parameters (tuned for responsiveness)
KP = 1.0  # Increased for stronger response
KI = 0.01  # Small to reduce steady-state error
KD = 0.05  # Small damping to prevent oscillations

# Timeout in seconds to prevent infinite spinning
TIMEOUT = 10

try:
    # Initialize Rosmaster
    print("Initializing ROSMaster...")
    ros = Rosmaster()
    print("ROSMaster initialized successfully")

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
            start_time = time.time()
            while True:
                current = ros.get_motor_encoder()
                error = target - current
                print(f"Speed: {speed}, Current: {current}, Target: {target}, Error: {error}")  # Debug print
                if abs(error) < 5:  # Within 5 pulses (~3 degrees)
                    break
                if time.time() - start_time > TIMEOUT:
                    print("Timeout reached, stopping motor")
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
            start_time = time.time()
            while True:
                current = ros.get_motor_encoder()[0]
                error = target - current
                print(f"Speed: {speed}, Current: {current}, Target: {target}, Error: {error}")  # Debug print
                if abs(error) < 5:
                    break
                if time.time() - start_time > TIMEOUT:
                    print("Timeout reached, stopping motor")
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