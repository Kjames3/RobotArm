# Requires: pip install pyserial
# Connect servo to the debug port of the controller
# Check the COM port: on Windows (e.g., COM4), on Linux (e.g., /dev/ttyUSB0)

import serial
from lewansoul_lx16a import LX16A  # Importing from LewanSoul library
import time

# Initialize the LX-16A servo controller
try:
    LX16A.initialize("COM4")  # Use "COM4" for Windows; for Linux, try "/dev/ttyUSB0"
except serial.SerialException as e:
    print(f"Failed to connect to servo controller: {e}")
    quit()

# Create a servo object with ID 1
servo1 = LX16A(1)

# Set angle limits (0 to 240 degrees)
servo1.set_angle_limits(0, 240)

try:
    while True:
        # Read current position and temperature
        current_pos = servo1.get_position()
        temp = servo1.get_temperature()
        print(f"Servo ID: {servo1.id}, Position: {current_pos:.1f} degrees, Temperature: {temp:.1f} °C")

        # Move to 0 degrees over 4 seconds
        servo1.moveTimeWrite(0, 4000)  # moveTimeWrite(angle, time_in_ms)
        print("Moving to 0 degrees over 4 seconds")
        time.sleep(4)  # Wait for movement to complete

        # Read position and temperature after movement
        current_pos = servo1.get_position()
        temp = servo1.get_temperature()
        print(f"Servo ID: {servo1.id}, Position: {current_pos:.1f} degrees, Temperature: {temp:.1f} °C")

        # Move to 180 degrees over 4 seconds
        servo1.moveTimeWrite(180, 4000)
        print("Moving to 180 degrees over 4 seconds")
        time.sleep(4)  # Wait for movement to complete

        # Read position and temperature after movement
        current_pos = servo1.get_position()
        temp = servo1.get_temperature()
        print(f"Servo ID: {servo1.id}, Position: {current_pos:.1f} degrees, Temperature: {temp:.1f} °C")

except serial.SerialException as e:
    print(f"Servo communication error: {e}")
    quit()

except KeyboardInterrupt:
    # Stop the servo on Ctrl+C
    print("User has stopped the servo")
    servo1.motorMode(0)  # Stop the servo motor
    quit()

finally:
    servo1.motorMode(0)  # Stop the servo
    quit()