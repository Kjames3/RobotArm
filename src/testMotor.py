import time
from Rosmaster_Lib import Rosmaster
import serial.serialutil

try:
    # Initialize Rosmaster (relies on /dev/myserial existing)
    print("Initializing ROSMaster...")
    ros = Rosmaster()
    print("ROSMaster initialized successfully")
    
    try:
        while True:
            # Rotate forward for 2 seconds
            ros.set_motor(50, 0, 0, 0)  # Motor on port 1, 50% speed
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