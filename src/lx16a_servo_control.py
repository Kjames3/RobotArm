# First : pip install pylx16a
# Second : Connect servos to the debug port of the controller

# check the COM3 port on windows, on linux it is /dev/ttyUSB0

# from pylx16a.lx16a import lx16a
from pylx16a.lx16a import *
import time

# Initialise the LX-16A servo controller

LX16A.initialize("COM4") # /dev/ttyUSB0 is most likely the connected port on Linux. 
# On Windows, try the COM ports instead, e.g. LX16A.initialize("COM3").


# Create a servo object with the servo's ID (default is 1)

servo1 = LX16A(1)
servo1.set_angle_limits(0, 240)

servo2 = LX16A(2)
servo2.set_angle_limits(0, 240)



# Move the servo through a range of positions
try:
    # Move to 0 degrees
    servo1.move(0)
    print("Moving to 0 degrees")
    time.sleep(1)  # Wait for 1 second

    # Move to 90 degrees
    servo1.move(90)
    print("Moving to 90 degrees")
    time.sleep(1)  # Wait for 1 second

    # Move to 180 degrees
    servo1.move(180)
    print("Moving to 180 degrees")
    time.sleep(1)  # Wait for 1 second

    # Move the second servo to 0 degrees
    servo2.move(0)
    print("Moving second servo to 0 degrees")
    time.sleep(1)  # Wait for 1 second

    # Move the second servo to 90 degrees
    servo2.move(90)
    print("Moving second servo to 90 degrees")

    # Move the second servo to 180 degrees
    servo2.move(180)
    print("Moving second servo to 180 degrees")
    time.sleep(1)  # Wait for 1 second

except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    quit()

except KeyboardInterrupt:
    # Stop the servo on Ctrl+C
    print("User has stopped the servo")
    servo1.stop()
    servo2.stop()

finally:
    servo1.stop()
    servo2.stop()
    # LX16A.cleanup()
    quit()