import sys
import time
import serial
import numpy as np
from typing import List, Tuple
import tkinter as tk
from tkinter import messagebox
import lewansoul_lx16a
import logging

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('robot_arm_log.txt'),
        logging.StreamHandler()  # Also output to console
    ]
)
logger = logging.getLogger(__name__)

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
try:
    controller = lewansoul_lx16a.ServoController(serial.Serial(SERIAL_PORT, 115200, timeout=0.1))
    logger.info("Successfully initialized servo controller on %s", SERIAL_PORT)
except serial.SerialException as e:
    logger.error("Failed to connect to serial port: %s", e)
    sys.exit(1)

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
        if i == 0:  # Base rotation - FIXED: Full 360 degree range
            # Map position range to -180 to +180 degrees
            theta_deg = -180 + (pos - pos_min) / (pos_max - pos_min) * 360
        elif i == 4:  # Wrist rotation
            theta_deg = -180 + (pos - pos_min) / (pos_max - pos_min) * 360
        else:
            theta_deg = -90 + (pos - pos_min) / (pos_max - pos_min) * 180
        theta_rad[i] = np.radians(theta_deg)
    logger.debug("Converted positions %s to theta_rad %s", positions, theta_rad)
    return theta_rad

def radians_to_positions(theta_rad: np.ndarray) -> List[float]:
    """Convert joint angles in radians to servo positions."""
    positions = []
    for i, theta in enumerate(theta_rad):
        theta_deg = np.degrees(theta)
        pos_min, pos_max = POS_RANGES[i]
        
        if i == 0:  # Base rotation - FIXED: Full 360 degree range
            # Normalize angle to [-180, 180] range
            theta_deg = ((theta_deg + 180) % 360) - 180
            pos = pos_min + (theta_deg + 180) / 360 * (pos_max - pos_min)
        elif i == 4:  # Wrist rotation
            # Normalize angle to [-180, 180] range
            theta_deg = ((theta_deg + 180) % 360) - 180
            pos = pos_min + (theta_deg + 180) / 360 * (pos_max - pos_min)
        else:
            pos = pos_min + (theta_deg + 90) / 180 * (pos_max - pos_min)
        
        # Clamp to servo limits
        positions.append(np.clip(pos, pos_min, pos_max))
    logger.debug("Converted theta_rad %s to positions %s", theta_rad, positions)
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
    logger.debug("Computed Jacobian for theta_rad %s", theta_rad)
    return J

def check_reachability(target_pos: np.ndarray) -> bool:
    """Check if target position is within robot's reach."""
    # Calculate distance from base
    r = np.sqrt(target_pos[0]**2 + target_pos[1]**2)
    z = target_pos[2]
    
    # Maximum reach is sum of link lengths
    max_reach = LINK_LENGTHS[0] + LINK_LENGTHS[1] + LINK_LENGTHS[2]
    # Minimum reach (when arm is folded)
    min_reach = abs(LINK_LENGTHS[0] - LINK_LENGTHS[1] - LINK_LENGTHS[2])
    
    distance_from_base = np.sqrt(r**2 + z**2)
    
    is_reachable = min_reach <= distance_from_base <= max_reach
    logger.debug("Target distance: %.2f, Min reach: %.2f, Max reach: %.2f, Reachable: %s", 
                distance_from_base, min_reach, max_reach, is_reachable)
    return is_reachable

def inverse_kinematics(
    target_pos: np.ndarray,
    target_orient: np.ndarray = np.array([0, 0, -1]),
    k: float = 0.1,
    max_iter: int = 200,
    tol: float = 1e-3,
    alpha: float = 0.1,
    lambda_damping: float = 0.1
) -> List[float]:
    """Solve inverse kinematics with improved handling of full rotation range."""
    logger.info("Starting IK solver for target position %s", target_pos)
    
    # Check if target is reachable
    if not check_reachability(target_pos):
        logger.warning("Target position may be unreachable")
    
    # Better initial guess
    initial_theta1 = np.arctan2(target_pos[1], target_pos[0])
    
    # Try to estimate other joint angles based on target position
    r = np.sqrt(target_pos[0]**2 + target_pos[1]**2)
    z = target_pos[2]
    
    # Simple geometric approximation for initial guess
    # Assume end effector is pointing down
    target_wrist = target_pos + np.array([0, 0, LINK_LENGTHS[2]])  # Account for wrist offset
    r_wrist = np.sqrt(target_wrist[0]**2 + target_wrist[1]**2)
    z_wrist = target_wrist[2]
    
    # Two-link IK for shoulder and elbow
    L1, L2 = LINK_LENGTHS[0], LINK_LENGTHS[1]
    D = np.sqrt(r_wrist**2 + z_wrist**2)
    
    if D <= L1 + L2:  # Check if reachable by first two links
        # Elbow angle
        cos_theta3 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_theta3 = np.clip(cos_theta3, -1, 1)  # Ensure valid range
        theta3 = np.arccos(cos_theta3)
        
        # Shoulder angle
        alpha = np.arctan2(z_wrist, r_wrist)
        beta = np.arctan2(L2 * np.sin(theta3), L1 + L2 * np.cos(theta3))
        theta2 = alpha - beta
    else:
        theta2 = 0.0
        theta3 = 0.0
    
    theta_rad = np.array([initial_theta1, theta2, theta3, 0.0, 0.0])
    logger.debug("Initial theta_rad: %s", theta_rad)
    
    for iteration in range(max_iter):
        T = forward_kinematics(theta_rad)
        p, z_axis = T[:3, 3], T[:3, 2]
        error = np.concatenate((target_pos - p, k * (target_orient - z_axis)))
        
        error_norm = np.linalg.norm(target_pos - p)
        logger.debug("Iteration %d: Position error %.3f mm", iteration, error_norm)
        
        if error_norm < tol:
            logger.info("IK converged after %d iterations", iteration)
            break
        
        J = compute_jacobian(theta_rad)
        JtJ = J.T @ J + lambda_damping**2 * np.eye(5)
        
        try:
            delta_theta = np.linalg.solve(JtJ, J.T @ error)
        except np.linalg.LinAlgError:
            logger.warning("Singular matrix encountered in IK solver")
            delta_theta = np.linalg.pinv(JtJ) @ J.T @ error
        
        # Apply update with step size control
        theta_rad += alpha * delta_theta
        
        # Convert to positions and back to handle servo limits
        positions = radians_to_positions(theta_rad)
        theta_rad = positions_to_radians(positions)
        
        # Adaptive step size
        if iteration > 50 and error_norm > 10:
            alpha = max(0.01, alpha * 0.95)  # Reduce step size if not converging
    else:
        final_T = forward_kinematics(theta_rad)
        final_error = np.linalg.norm(target_pos - final_T[:3, 3])
        logger.warning("IK did not converge. Final error: %.3f mm", final_error)
    
    final_positions = radians_to_positions(theta_rad)
    logger.info("IK returning positions: %s", final_positions)
    return final_positions

def return_to_home() -> None:
    """Move all servos to home position."""
    home_positions = [705, 865, 430, 100, 500, GRIPPER_POS[1]]
    logger.info("Returning to home position: %s", home_positions)
    try:
        controller.group_move(list(range(1, 7)), [int(pos) for pos in home_positions], MOVE_TIME)
    except AttributeError as e:
        logger.error("Group move failed: %s", e)
        for i, pos in enumerate(home_positions, 1):
            try:
                controller.move(i, int(pos), MOVE_TIME)
                logger.debug("Moved servo %d to position %d", i, pos)
                time.sleep(0.1)
            except Exception as e:
                logger.error("Failed to move servo %d: %s", i, e)

class RobotArmGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Robot Arm Control")
        self.root.geometry("600x600")
        
        # State variables
        self.target_pos = np.array([0.0, 0.0, 0.0])
        self.is_moving = False
        self.manual_control = [False] * SERVO_COUNT  # Initialize manual control flags
        
        # Create GUI elements
        self.create_input_frame()
        self.create_servo_frame()
        self.create_position_frame()
        self.create_ik_equation_frame()
        
        logger.info("GUI initialized")
        # Start periodic updates
        self.update_gui()
    
    def create_input_frame(self):
        """Create input fields for target position and gripper state."""
        frame = tk.LabelFrame(self.root, text="Target Position", padx=5, pady=5)
        frame.pack(pady=10, fill="x")
        
        tk.Label(frame, text="X (mm):").grid(row=0, column=0)
        self.x_entry = tk.Entry(frame)
        self.x_entry.grid(row=0, column=1)
        
        tk.Label(frame, text="Y (mm):").grid(row=1, column=0)
        self.y_entry = tk.Entry(frame)
        self.y_entry.grid(row=1, column=1)
        
        tk.Label(frame, text="Z (mm):").grid(row=2, column=0)
        self.z_entry = tk.Entry(frame)
        self.z_entry.grid(row=2, column=1)
        
        tk.Button(frame, text="Move", command=self.move_to_position).grid(row=3, column=0, pady=5)
        tk.Button(frame, text="Home", command=self.return_to_home).grid(row=3, column=1, pady=5)
        tk.Button(frame, text="Random", command=self.random_movements).grid(row=3, column=2, pady=5)
        
        # Add test buttons for left/right positions
        tk.Button(frame, text="Test Left", command=self.test_left_position).grid(row=4, column=0, pady=5)
        tk.Button(frame, text="Test Right", command=self.test_right_position).grid(row=4, column=1, pady=5)
        
        tk.Label(frame, text="Gripper:").grid(row=5, column=0)
        self.gripper_var = tk.BooleanVar()
        tk.Checkbutton(frame, text="Open", variable=self.gripper_var).grid(row=5, column=1)
        logger.debug("Input frame created")
    
    def test_left_position(self):
        """Test movement to left side."""
        self.x_entry.delete(0, tk.END)
        self.x_entry.insert(0, "-150")
        self.y_entry.delete(0, tk.END)
        self.y_entry.insert(0, "150")
        self.z_entry.delete(0, tk.END)
        self.z_entry.insert(0, "200")
        logger.info("Set test position for left side")
    
    def test_right_position(self):
        """Test movement to right side."""
        self.x_entry.delete(0, tk.END)
        self.x_entry.insert(0, "150")
        self.y_entry.delete(0, tk.END)
        self.y_entry.insert(0, "150")
        self.z_entry.delete(0, tk.END)
        self.z_entry.insert(0, "200")
        logger.info("Set test position for right side")
    
    def create_servo_frame(self):
        """Create display for servo information and sliders."""
        frame = tk.LabelFrame(self.root, text="Servo Status", padx=5, pady=5)
        frame.pack(pady=10, fill="x")
        
        self.servo_labels = {}
        self.servo_sliders = {}
        for i in range(1, SERVO_COUNT + 1):
            tk.Label(frame, text=f"Servo {i}:").grid(row=i, column=0, sticky="w")
            self.servo_labels[i] = tk.Label(frame, text="", width=50)
            self.servo_labels[i].grid(row=i, column=1, sticky="w")
            
            pos_min, pos_max = POS_RANGES[i-1]
            slider = tk.Scale(frame, from_=pos_min, to=pos_max, orient=tk.HORIZONTAL,
                              length=200, command=lambda pos, servo=i: self.on_slider_move(servo, pos))
            slider.grid(row=i, column=2, padx=10)
            self.servo_sliders[i] = slider
        logger.debug("Servo frame created with %d sliders", SERVO_COUNT)
    
    def create_position_frame(self):
        """Create display for current and target positions."""
        frame = tk.LabelFrame(self.root, text="Position Info", padx=5, pady=5)
        frame.pack(pady=10, fill="x")
        
        self.current_pos_label = tk.Label(frame, text="Current Position: (0.0, 0.0, 0.0) mm")
        self.current_pos_label.pack()
        
        self.target_pos_label = tk.Label(frame, text="Target Position: (0.0, 0.0, 0.0) mm")
        self.target_pos_label.pack()
        
        self.reachability_label = tk.Label(frame, text="Reachability: Unknown")
        self.reachability_label.pack()
        logger.debug("Position frame created")
    
    def create_ik_equation_frame(self):
        """Create display for inverse kinematics equation."""
        frame = tk.LabelFrame(self.root, text="IK Equations", padx=5, pady=5)
        frame.pack(pady=10, fill="x")
        self.ik_label = tk.Label(frame, text="θ1 = 0, θ2 = 0, θ3 = 0, θ4 = 0, θ5 = 0")
        self.ik_label.pack()
        logger.debug("IK equation frame created")
    
    def update_gui(self):
        """Update GUI with real-time servo and position data."""
        try:
            for i in range(1, SERVO_COUNT + 1):
                try:
                    pos = controller.get_position(i)
                    temp = controller.get_temperature(i)
                    temp_limit = controller.get_max_temperature_limit(i)
                    voltage = controller.get_voltage(i)
                    errors = controller.get_led_errors(i)
                    self.servo_labels[i].config(
                        text=f"Pos: {pos}, Temp: {temp}°C/{temp_limit}°C, Voltage: {voltage}mV, Errors: {errors}"
                    )
                    if not self.manual_control[i-1]:
                        self.servo_sliders[i].set(pos)
                    logger.debug("Updated servo %d: position=%d, temp=%d°C, voltage=%dmV", i, pos, temp, voltage)
                except Exception as e:
                    self.servo_labels[i].config(text=f"Error: {e}")
                    logger.error("Error updating servo %d: %s", i, e)
            
            current_positions = [controller.get_position(i + 1) for i in range(5)]
            theta_rad = positions_to_radians(current_positions)
            T = forward_kinematics(theta_rad)
            current_pos = T[:3, 3]
            self.current_pos_label.config(
                text=f"Current Position: ({current_pos[0]:.1f}, {current_pos[1]:.1f}, {current_pos[2]:.1f}) mm"
            )
            
            # Update reachability status
            try:
                x = float(self.x_entry.get()) if self.x_entry.get() else 0
                y = float(self.y_entry.get()) if self.y_entry.get() else 0
                z = float(self.z_entry.get()) if self.z_entry.get() else 0
                test_pos = np.array([x, y, z])
                is_reachable = check_reachability(test_pos)
                self.reachability_label.config(
                    text=f"Target Reachable: {'Yes' if is_reachable else 'No'}",
                    fg='green' if is_reachable else 'red'
                )
            except ValueError:
                self.reachability_label.config(text="Reachability: Invalid input", fg='orange')
            
            self.target_pos_label.config(
                text=f"Target Position: ({self.target_pos[0]:.1f}, {self.target_pos[1]:.1f}, {self.target_pos[2]:.1f}) mm"
            )
            
            # Update IK display
            theta_degrees = np.degrees(theta_rad)
            self.ik_label.config(
                text=f"θ1={theta_degrees[0]:.1f}°, θ2={theta_degrees[1]:.1f}°, θ3={theta_degrees[2]:.1f}°, θ4={theta_degrees[3]:.1f}°, θ5={theta_degrees[4]:.1f}°"
            )
            
        except Exception as e:
            logger.error("Error updating GUI: %s", e)
        
        self.root.after(500, self.update_gui)
    
    def on_slider_move(self, servo_id: int, position: str):
        """Handle manual slider movement to control servo position."""
        try:
            pos = int(position)
            controller.move(servo_id, pos, MOVE_TIME)
            self.manual_control[servo_id-1] = True
            self.root.after(1000, lambda: self.reset_manual_control(servo_id))
            logger.info("Manually moved servo %d to position %d", servo_id, pos)
        except Exception as e:
            logger.error("Error moving servo %d: %s", servo_id, e)
    
    def reset_manual_control(self, servo_id: int):
        """Reset manual control flag after manual adjustment."""
        self.manual_control[servo_id-1] = False
        logger.debug("Reset manual control for servo %d", servo_id)
    
    def move_to_position(self):
        """Handle move to target position and gripper state."""
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            z = float(self.z_entry.get())
            self.target_pos = np.array([x, y, z])
            logger.info("Attempting to move to position: %s", self.target_pos)
            
            # Check reachability before attempting move
            if not check_reachability(self.target_pos):
                messagebox.showwarning("Warning", "Target position may be unreachable!")
            
            self.is_moving = True
            theta_rad = inverse_kinematics(self.target_pos)
            positions = radians_to_positions(theta_rad)
            
            for i, pos in enumerate(positions, 1):
                controller.move(i, int(pos), MOVE_TIME)
                logger.debug("Moving servo %d to position %d", i, int(pos))
            
            gripper_pos = GRIPPER_POS[0] if self.gripper_var.get() else GRIPPER_POS[1]
            controller.move(6, gripper_pos, MOVE_TIME)
            logger.debug("Set gripper to position %d", gripper_pos)
            
            time.sleep(MOVE_TIME / 1000)
            self.is_moving = False
            logger.info("Movement completed")
        except Exception as e:
            logger.error("Movement error: %s", e)
            messagebox.showerror("Error", f"Invalid input or movement error: {e}")
            self.is_moving = False
    
    def return_to_home(self):
        """Move to home position."""
        logger.info("Returning to home position")
        return_to_home()
        self.is_moving = False
        self.target_pos = np.array([0.0, 0.0, 0.0])
    
    def random_movements(self):
        """Generate a random reachable position and set it in the input boxes."""
        # Generate positions more likely to be reachable
        angle = np.random.uniform(0, 2*np.pi)
        radius = np.random.uniform(100, 300)  # Within reasonable reach
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        z = np.random.uniform(50, 250)
        
        self.x_entry.delete(0, tk.END)
        self.x_entry.insert(0, f"{x:.1f}")
        self.y_entry.delete(0, tk.END)
        self.y_entry.insert(0, f"{y:.1f}")
        self.z_entry.delete(0, tk.END)
        self.z_entry.insert(0, f"{z:.1f}")
        self.gripper_var.set(np.random.choice([True, False]))
        logger.info("Generated random position: (%.1f, %.1f, %.1f)", x, y, z)

def main():
    """Main function to launch GUI."""
    logger.info("Starting robot arm GUI application")
    root = tk.Tk()
    app = RobotArmGUI(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        logger.info("Application interrupted by user")
        return_to_home()
        for i in range(1, 7):
            try:
                controller.set_motor_mode(i, 0)
                logger.debug("Set servo %d to motor mode 0", i)
            except Exception as e:
                logger.error("Error setting motor mode for servo %d: %s", i, e)
        logger.info("Exiting application")
        sys.exit(0)
    except Exception as e:
        logger.error("GUI error: %s", e)
        return_to_home()
        for i in range(1, 7):
            try:
                controller.set_motor_mode(i, 0)
                logger.debug("Set servo %d to motor mode 0", i)
            except Exception as e:
                logger.error("Error setting motor mode for servo %d: %s", i, e)
        sys.exit(1)

if __name__ == "__main__":
    main()