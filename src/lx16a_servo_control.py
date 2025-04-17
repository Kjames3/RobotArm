# First : pip install pylx16a
# Second : Connect servos to the debug port of the controller

# check the COM3 port on windows, on linux it is /dev/ttyUSB0

from pylx16a.lx16a import lx16a
import time

# Initialise the LX-16A servo controller

LX16A.initialize("/dev/ttyUSB0")

# Create a servo object with the servo's ID (default is 1)

servo = LX16A(1)

# Move the servo through a range of positions
try:
    # Move to 0 degrees
    servo.move(0)
    print("Moving to 0 degrees")
    time.sleep(1)  # Wait for 1 second

    # Move to 90 degrees
    servo.move(90)
    print("Moving to 90 degrees")
    time.sleep(1)  # Wait for 1 second

    # Move to 180 degrees
    servo.move(180)
    print("Moving to 180 degrees")
    time.sleep(1)  # Wait for 1 second

except KeyboardInterrupt:
    # Stop the servo on Ctrl+C
    print("User has stopped the servo")
    servo.stop()

finally:
    LX16A.cleanup()