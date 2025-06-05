from pylx16a.lx16a import *
import time

# Define move over time function
def move_over_time(servo, target, duration):
    current = servo.get_position()
    if current is None:
        current = 0
    total_distance = target - current
    if total_distance == 0:
        return
    num_steps = 100  # Example: 100 steps for smoother movement
    step_size = total_distance / num_steps
    step_time = duration / num_steps
    for i in range(num_steps):
        new_pos = current + (i + 1) * step_size
        servo.move(int(new_pos))  # Assuming move takes integers
        time.sleep(step_time)


LX16A.initialize("COM4")
servo = LX16A(1)
servo.set_angle_limits(0, 180)

servo.move()
time.sleep(6)
servo.move_over_time(90, 2)
time.sleep(6)