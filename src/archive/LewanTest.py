import sys, time, serial
import lewansoul_lx16a
import numpy as np

SERIAL_PORT = "COM4"

controller = lewansoul_lx16a.ServoController(serial.Serial(SERIAL_PORT, 115200, timeout=0.1))

# Servo movement constraints
pos_ranges = [
    (190, 1180),  # Servo 1: Base rotation
    (55, 915),    # Servo 2: Shoulder
    (-190, 497),  # Servo 3: Elbow
    (-190, 394),  # Servo 4: Wrist tilt
    (0, 1000),    # Servo 5: Wrist rotation
    (288, 607)    # Servo 6: Gripper (closed to open)
]

# Link lengths (in mm, adjust based on SO100 specs)
L2 = 150.0  # Shoulder to elbow
L3 = 150.0  # Elbow to wrist
L4 = 90.0   # Wrist to gripper

# Define gripper positions based on user observation
GRIPPER_OPEN = 288    # Position for open gripper
GRIPPER_CLOSED = 607  # Position for closed gripper

def dh_transform(theta, d, a, alpha):
    """Compute the Denavit-Hartenberg transformation matrix."""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(theta_rad):
    """Compute the end effector pose given joint angles in radians."""
    T = np.eye(4)
    dh_params = [
        (theta_rad[0], 0, 0, 0),         # Joint 1: Base rotation
        (theta_rad[1], 0, L2, 0),        # Joint 2: Shoulder
        (theta_rad[2], 0, L3, 0),        # Joint 3: Elbow
        (theta_rad[3], 0, 0, np.pi/2),   # Joint 4: Wrist tilt
        (theta_rad[4], L4, 0, 0)         # Joint 5: Wrist rotation
    ]
    for theta, d, a, alpha in dh_params:
        T = T @ dh_transform(theta, d, a, alpha)
    return T

def get_theta_rad(positions):
    """Convert servo positions to joint angles in radians."""
    theta_rad = []
    for i, pos in enumerate(positions):
        pos_min, pos_max = pos_ranges[i]
        if i == 4:  # Servo 5: Wrist rotation
            theta_deg = -180 + (pos - pos_min) / (pos_max - pos_min) * 360
        else:
            theta_deg = -90 + (pos - pos_min) / (pos_max - pos_min) * 180
        theta_rad.append(np.radians(theta_deg))
    return theta_rad

def get_positions_from_theta_rad(theta_rad):
    """Convert joint angles in radians back to servo positions."""
    positions = []
    for i, theta in enumerate(theta_rad):
        theta_deg = np.degrees(theta)
        pos_min, pos_max = pos_ranges[i]
        if i == 4:  # Servo 5
            position = pos_min + (theta_deg + 180) / 360 * (pos_max - pos_min)
        else:
            position = pos_min + (theta_deg + 90) / 180 * (pos_max - pos_min)
        positions.append(position)
    return positions

def compute_jacobian(theta_rad, delta=1e-4):
    """Numerically compute the Jacobian matrix."""
    T = forward_kinematics(theta_rad)
    p = T[0:3, 3]
    z = T[0:3, 2]
    J = np.zeros((6, 5))
    for i in range(5):
        theta_pert = theta_rad.copy()
        theta_pert[i] += delta
        T_pert = forward_kinematics(theta_pert)
        p_pert = T_pert[0:3, 3]
        z_pert = T_pert[0:3, 2]
        J[0:3, i] = (p_pert - p) / delta
        J[3:6, i] = (z_pert - z) / delta
    return J

def inverse_kinematics(p_d, z_d=np.array([0, 0, -1]), k=0.1, max_iter=100, tol=1e-3, alpha=0.05, lambda_damping=0.1):
    """Solve inverse kinematics using damped least-squares to reach position p_d with orientation z_d."""
    current_positions = [controller.get_position(i + 1) for i in range(5)]
    theta_rad = get_theta_rad(current_positions)
    
    for _ in range(max_iter):
        T = forward_kinematics(theta_rad)
        p = T[0:3, 3]
        z = T[0:3, 2]
        e_p = p_d - p
        e_o = z_d - z
        e = np.concatenate((e_p, k * e_o))
        
        if np.linalg.norm(e) < tol:
            break
        
        J = compute_jacobian(theta_rad)
        # Damped least-squares: (J^T J + lambda^2 I)^(-1) J^T e
        JtJ = J.T @ J
        damping_matrix = lambda_damping**2 * np.eye(JtJ.shape[0])
        delta_theta = np.linalg.inv(JtJ + damping_matrix) @ J.T @ e
        theta_rad += alpha * delta_theta
        
        # Enforce joint limits in radian space
        for i in range(len(theta_rad)):
            pos = get_positions_from_theta_rad([theta_rad[i]])[0]
            pos_min, pos_max = pos_ranges[i]
            pos = max(pos_min, min(pos_max, pos))
            theta_rad[i] = get_theta_rad([pos])[0]
    
    else:
        print("Warning: IK did not converge within max iterations.")
        print(f"Final position error: {np.linalg.norm(p_d - p)} mm")
    
    positions = get_positions_from_theta_rad(theta_rad)
    positions = [max(min(pos, pos_ranges[i][1]), pos_ranges[i][0]) for i, pos in enumerate(positions)]
    return positions

def servo_info(servo_id):
    """Print servo information."""
    print("\n-----------------------------------")
    print(f"Servo id: {servo_id}")
    print(f"Position: {controller.get_position(servo_id)}")
    print(f"Temperature: {controller.get_temperature(servo_id)}, limit: {controller.get_max_temperature_limit(servo_id)}")
    print(f"Voltage: {controller.get_voltage(servo_id)} mV")
    print(f"Led error: {controller.get_led_errors(servo_id)}")
    print("-----------------------------------")

def return_position():
    print("************************************")
    print("Returning to position...")
    controller.move(1, 706, 1000)
    controller.move(2, 857, 1000)
    controller.move(3, 428, 1000)
    controller.move(4, 100, 1000)  # Adjusted from 1190 to be within range
    controller.move(5, 500, 1000)
    controller.move(6, GRIPPER_CLOSED, 1000)

    # Open position
    for i in range(1, 7):
        time.sleep(0.5)
        servo_info(i)
    print("Returned to position")
    print("************************************")

if __name__ == "__main__":
    print("Enter desired position (x y z) in mm and gripper state (open/closed), e.g., '100 50 200 open':")
    try:
        input_str = input().strip().split()

        x, y, z = map(float, input_str[:3])

        gripper_state = input_str[3].lower()
        
        p_d = np.array([x, y, z])
        z_d = np.array([0, 0, -1])  # End effector points downwards
        
        # Check if position is reachable
        max_reach = L2 + L3 + L4
        dist = np.linalg.norm(p_d)
        if dist > max_reach:
            print(f"Error: Position {p_d} is beyond max reach of {max_reach} mm")
            sys.exit(1)
        
        # Compute IK solution
        target_positions = inverse_kinematics(p_d, z_d, k=0)
        
        # Move servos 1-5 to target positions
        for i, pos in enumerate(target_positions):
            controller.move(i + 1, int(pos), 1000)
            servo_info(i + 1)
        
        # Control gripper (servo 6)
        if gripper_state == "open":
            controller.move(6, GRIPPER_OPEN, 1000)
            print("Gripper set to open")
        elif gripper_state == "closed":
            controller.move(6, GRIPPER_CLOSED, 1000)
            print("Gripper set to closed")
        else:
            print("Invalid gripper state; defaulting to closed")
            controller.move(6, GRIPPER_CLOSED, 1000)
        time.sleep(1.5)  # Wait for gripper move to complete
        servo_info(6)
        
        # Wait for movement to complete
        time.sleep(2)
        
    except ValueError as e:
        print(f"Input error: Please enter three numbers followed by 'open' or 'closed'. Error: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
        for i in range(1, 7):
            controller.set_motor_mode(i, 0)
    except KeyboardInterrupt:
        print("\nOperation interrupted by user.")
        return_position()
        print("Exiting...")
        for i in range(1, 7):
            controller.set_motor_mode(i, 0)
        sys.exit(0)

    finally:
        time.sleep(2)
        return_position()
        print("Operation completed.")
        