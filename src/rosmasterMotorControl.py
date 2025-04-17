import time
from Rosmaster_Lib import Rosmaster
import serial.tools.list_ports

def find_serial_port():
    """List available serial ports and return the most likely one."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        raise RuntimeError("No serial ports found. Ensure the ROSMaster board is connected.")
    for port in ports:
        print(f"Found port: {port.device} ({port.description})")
        # Return the first USB serial port (e.g., /dev/ttyUSB0 or /dev/ttyACM0)
        if "USB" in port.description or "ACM" in port.device:
            return port.device
    raise RuntimeError("No suitable serial port found. Check connections or try a different port.")

try:
    # Attempt to find the correct serial port
    serial_port = find_serial_port()
    print(f"Using serial port: {serial_port}")
    
    # Initialize Rosmaster with the specified port
    ros = Rosmaster(port=serial_port)
    
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

except RuntimeError as e:
    print(f"Error: {e}")
except serial.serialutil.SerialException as e:
    print(f"Serial port error: {e}")
    print("Ensure the ROSMaster board is connected and the port is correct.")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    # Cleanup is handled by Rosmaster's destructor, but ensure motor is stopped
    try:
        ros.set_motor(0, 0, 0, 0)
    except:
        pass