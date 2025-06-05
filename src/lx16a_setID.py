# Here is the code to change the ID of the Lx-16a servos

from pylx16a.lx16a import *


# Initialise the LX-16A servo with the correct port
LX16A.initialize("COM4") # Use COM for Windows, /dev/ttyUSB0 for Linux

try:
    # Create a servo object with the servo's ID (default is 1)
    servo = LX16A(1) # The default ID is 1
    

    # Change the ID of the servo
    new_id = 6 # The new ID of the servo
    servo.set_id(new_id) # Change the ID of the servo
    print(f"Servo ID changed to {new_id}") # Print the new ID of the servo

except Exception as e:
    print(f"Serial port error: {e}")
    print(f"Error: {e}")