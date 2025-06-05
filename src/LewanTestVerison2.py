import sys
import time
import serial
import cProfile
import pstats
import numpy as np
from typing import List, Tuple, Optional
import lewansoul_lx16a

# Constants
SERIAL_PORT: str = "COM4"
SERVO_COUNT: int = 6
MOVE_TIME: int = 1000  # ms
POS_RANGES: List[Tuple[int, int]] = [
    (190, 1180),  # Servo 1: Base rotation
    (55, 915),    # Servo 2: Shoulder
    (-190, 497),  # Servo 3: Elbow
    (-190, 394),  # Servo 4: Wrist tilt
    (0, 1000),    # Servo 5: Wrist rotation
    (288, 607)    # Servo 6: Gripper (closed to open)
]
LINK_LENGTHS: Tuple[float, float, float] = (150.0, 150.0, 90.0)  # L2, L3, L4 in mm
GRIPPER_POS: Tuple[int, int] = (288, 607)  # Open, Closed
DH_PARAMS: List[Tuple[float, float, float, float]] = [
    (0, 0, 0, 0),           # Joint 1: Base rotation
    (0, 0, LINK_LENGTHS[0], 0),  # Joint 2: Shoulder
    (0, 0, LINK_LENGTHS[1], 0),  # Joint 3: Elbow
    (0, 0, 0, np.pi/2),     # Joint 4: Wrist tilt
    (0, LINK_LENGTHS[2], 0, 0)   # Joint 5: Wrist rotation
]

# Initialize controller
controller = lewansoul_lx16a.ServoController(serial.Serial(SERIAL_PORT, 115200, timeout=0.1))

def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Compute Denavit-Hartenberg transformation matrix using NumPy."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(theta_rad: np.ndarray) -> np.ndarray:
    """Compute end effector pose using tensor operations."""
    T = np.eye(4)
    for i, (theta, d, a, alpha) in enumerate(DH_PARAMS):
        T = T @ dh_transform(theta_rad[i] if i < 5 else 0, d, a, alpha)
    return T

def positions_to_radians(positions: List[float]) -> np.ndarray:
    """Convert servo positions to joint angles in radians."""
    theta_rad = np.zeros(len(positions))
    for i, pos in enumerate(positions):
        pos_min, pos_max = POS_RANGES[i]
        if i == 4:  # Wrist rotation
            theta_deg = -180 + (pos - pos_min) / (pos_max - pos_min) * 360
        else:
            theta_deg = -90 + (pos - pos_min) / (pos_max - pos_min) * 180
        theta_rad[i] = np.radians(theta_deg)
    return theta_rad

def radians_to_positions(theta_rad: np.ndarray) -> List[float]:
    """Convert joint angles in radians to servo positions."""
    positions = []
    for i, theta in enumerate(theta_rad):
        theta_deg = np.degrees(theta)
        pos_min, pos_max = POS_RANGES[i]
        if i == 4:  # Wrist rotation
            pos = pos_min + (theta_deg + 180) / 360 * (pos_max - pos_min)
        else:
            pos = pos_min + (theta_deg + 90) / 180 * (pos_max - pos_min)
        positions.append(np.clip(pos, pos_min, pos_max))
    return positions

def compute_jacobian(theta_rad: np.ndarray, delta: float = 1e-4) -> np.ndarray:
    """Compute Jacobian matrix numerically with optimized tensor operations."""
    T = forward_kinematics(theta_rad)
    p, z = T[:3, 3], T[:3, 2]
    J = np.zeros((6, 5))
    theta_pert = theta_rad.copy()
    
    for i in range(5):
        theta_pert[i] += delta
        T_pert = forward_kinematics(theta_pert)
        J[:3, i] = (T_pert[:3, 3] - p) / delta
        J[3:6, i] = (T_pert[:3, 2] - z) / delta
        theta_pert[i] = theta_rad[i]
    return J

def inverse_kinematics(
    target_pos: np.ndarray,
    target_orient: np.ndarray = np.array([0, 0, -1]),
    k: float = 0.1,
    max_iter: int = 100,
    tol: float = 1e-3,
    alpha: float = 0.05,
    lambda_damping: float = 0.1
) -> List[float]:
    """Solve inverse kinematics using damped least-squares."""
    current_positions = [controller.get_position(i + 1) for i in range(5)]
    theta_rad = positions_to_radians(current_positions)
    
    for _ in range(max_iter):
        T = forward_kinematics(theta_rad)
        p, z = T[:3, 3], T[:3, 2]
        error = np.concatenate((target_pos - p, k * (target_orient - z)))
        
        if np.linalg.norm(error) < tol:
            break
        
        J = compute_jacobian(theta_rad)
        JtJ = J.T @ J + lambda_damping**2 * np.eye(5)
        delta_theta = np.linalg.solve(JtJ, J.T @ error)
        theta_rad += alpha * delta_theta
        
        positions = radians_to_positions(theta_rad)
        theta_rad = positions_to_radians(positions)
    else:
        print(f"Warning: IK did not converge. Position error: {np.linalg.norm(target_pos - p):.2f} mm")
    
    return radians_to_positions(theta_rad)

def print_servo_info(servo_id: int) -> None:
    """Print servo status information."""
    print("\n-----------------------------------")
    print(f"Servo ID: {servo_id}")
    print(f"Position: {controller.get_position(servo_id)}")
    print(f"Temperature: {controller.get_temperature(servo_id)}°C, ")
    print(f"Limit: {controller.get_max_temperature_limit(servo_id)}°C")
    print(f"Voltage: {controller.get_voltage(servo_id)} mV")
    print(f"LED Errors: {controller.get_led_errors(servo_id)}")
    print("-----------------------------------")

def return_to_home() -> None:
    """Move all servos to home position."""
    print("************************************")
    print("Returning to home position...")
    home_positions = [706, 857, 428, 100, 500, GRIPPER_POS[1]]
    
    try:
        controller.group_move(list(range(1, 7)), [int(pos) for pos in home_positions], MOVE_TIME)
    except AttributeError:
        for i, pos in enumerate(home_positions, 1):
            controller.move(i, int(pos), MOVE_TIME)
            time.sleep(0.1)
    
    for i in range(1, 7):
        time.sleep(0.3)
        print_servo_info(i)
    print("Returned to home position")
    print("************************************")

def main() -> None:
    """Main function to handle user input and control the robotic arm with profiling."""
    def run_control_logic():
        print("Enter target position (x y z) in mm and gripper state (open/closed), e.g., '100 50 200 open':")
        x, y, z, gripper_state = input().strip().split()
        target_pos = np.array([float(x), float(y), float(z)])
        gripper_state = gripper_state.lower()
        
        max_reach = sum(LINK_LENGTHS)
        if np.linalg.norm(target_pos) > max_reach:
            print(f"Error: Position {target_pos} exceeds max reach of {max_reach:.1f} mm")
            return
        
        target_positions = inverse_kinematics(target_pos, k=0)
        
        try:
            controller.group_move(list(range(1, 6)), [int(pos) for pos in target_positions], MOVE_TIME)
        except AttributeError:
            for i, pos in enumerate(target_positions, 1):
                controller.move(i, int(pos), MOVE_TIME)
                print_servo_info(i)
                time.sleep(0.1)
        
        gripper_pos = GRIPPER_POS[0] if gripper_state == "open" else GRIPPER_POS[1]
        controller.move(6, gripper_pos, MOVE_TIME)
        print(f"Gripper set to {'open' if gripper_state == 'open' else 'closed'}")
        time.sleep(0.5)
        print_servo_info(6)
        
        time.sleep(1.5)
    
    # Run with profiling
    profiler = cProfile.Profile()
    try:
        profiler.enable()
        run_control_logic()
        profiler.disable()
        
        # Save profile data
        profiler.dump_stats('profile_stats.prof')
        
        # Print summary of profiling results
        stats = pstats.Stats(profiler).sort_stats('cumulative')
        print("\nProfiling Summary (Top 5 time-consuming functions):")
        stats.print_stats(5)
    
    except ValueError as e:
        print(f"Input error: Please enter three numbers followed by 'open' or 'closed'. Error: {e}")
    except Exception as e:
        print(f"Error: {e}")
        for i in range(1, 7):
            controller.set_motor_mode(i, 0)
    finally:
        time.sleep(1)
        return_to_home()
        print("Operation completed.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nOperation interrupted by user.")
        return_to_home()
        for i in range(1, 7):
            controller.set_motor_mode(i, 0)
        print("Exiting...")
        sys.exit(0)