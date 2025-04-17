from Rosmaster_Lib import Rosmaster
import time

# Initialize the ROSMaster controller
ros = Rosmaster()

try:
    while True:
        # Rotate forward for 2 seconds
        ros.set_motor(50, 0, 0, 0)  # Assuming motor is on port 1
        print("Moving forward at 50% speed")
        time.sleep(2)
        
        # Stop for 1 second
        ros.set_motor(0, 0, 0, 0)
        print("Stopping")
        time.sleep(1)
        
        # Rotate backward for 2 seconds
        ros.set_motor(-50, 0, 0, 0)
        print("Moving backward at 50% speed")
        time.sleep(2)
        
        # Stop for 1 second
        ros.set_motor(0, 0, 0, 0)
        print("Stopping")
        time.sleep(1)

except KeyboardInterrupt:
    print("Program interrupted")
    ros.set_motor(0, 0, 0, 0)  # Ensure motor stops
    print("Motor stopped")