import sys, time, serial
import lewansoul_lx16a

SERIAL_PORT = "COM4"

controller = lewansoul_lx16a.ServoController(serial.Serial(SERIAL_PORT, 115200, timeout=0.1))

def servo_info(servo_id):
    print("\n-----------------------------------")
    print(f"Servo id: {servo_id}")
    print(f"Position: {controller.get_position(servo_id)}")
    print(f"Temperature: {controller.get_temperature(servo_id)}, limit: {controller.get_max_temperature_limit(servo_id)}")
    print(f"Voltage: {controller.get_voltage(servo_id)} mV")
    print(f"Led error: {controller.get_led_errors(servo_id)}")
    print("-----------------------------------")

def get_offset_info(servo_id, target_angle):
    print("\n-----------------------------------")
    offset = controller.get_position(servo_id)
    print(f"Offset: {offset}")
    print(f"Offset from 0: {offset - 0}")
    print(f"Offset from target: {offset - target_angle}")
    print("-----------------------------------")
    return offset

def return_position():
    print("************************************")
    print("Returning to position...")
    controller.move(1, 706, 1000)
    controller.move(2, 857, 1000)
    controller.move(3, 428, 1000)
    controller.move(4, 100, 1000)  # Adjusted from 1190 to be within range
    controller.move(5, 500, 1000)
    controller.move(6, 607, 1000)  # Open position
    for i in range(1, 7):
        time.sleep(1)
        servo_info(i)
    print("Returned to position")
    print("************************************")

if __name__ == "__main__":
    pos_ranges = [(190, 1180), (55, 915), (-190, 497), (-190, 394), (0, 1000), (288, 607)]
    try:
        for i in range(1, 7):
            mid_pos = (pos_ranges[i-1][0] + pos_ranges[i-1][1]) / 2
            controller.move(i, int(mid_pos), 1000)
            time.sleep(1)
            servo_info(i)

    except KeyboardInterrupt:
        print("Exiting...")
        for i in range(1, 7):
            controller.motor_mode(i, 0)  # Stop all servos
        sys.exit(0)

    except Exception as e:
        print(f"An error occurred: {e}")
        for i in range(1, 7):
            controller.motor_mode(i, 0)  # Stop all servos

    finally:
        return_position()