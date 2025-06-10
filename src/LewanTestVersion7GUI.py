import sys
import time
import numpy as np
from typing import List, Tuple, Optional
import tkinter as tk
from tkinter import messagebox, ttk
import logging
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import threading
import queue

# Try to import serial components, but make them optional
try:
    import serial
    import lewansoul_lx16a
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("Hardware libraries not available. Running in simulation mode only.")

# Configure logging with different levels for file vs console
file_handler = logging.FileHandler('robot_arm_log.txt')
file_handler.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)  # Less verbose for console

formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logger.addHandler(file_handler)
logger.addHandler(console_handler)

# Prevent log messages from being propagated to the root logger
logger.propagate = False

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

# Add caching for IK solutions
IK_CACHE = {}
IK_CACHE_SIZE = 100  # Limit cache size to avoid memory issues

class MockServoController:
    """Mock servo controller for simulation mode."""
    
    def __init__(self):
        self.positions = [685, 485, 153, 102, 500, 447]  # Default positions
        self.temperatures = [35] * 6
        self.voltages = [7400] * 6
        self.errors = [0] * 6
        logger.info("Mock servo controller initialized")
    
    def move(self, servo_id: int, position: int, time_ms: int):
        """Simulate servo movement."""
        if 1 <= servo_id <= 6:
            self.positions[servo_id - 1] = position
            logger.debug("Mock: Moved servo %d to position %d", servo_id, position)
    
    def group_move(self, servo_ids: List[int], positions: List[int], time_ms: int):
        """Simulate group movement."""
        for servo_id, position in zip(servo_ids, positions):
            if 1 <= servo_id <= 6:
                self.positions[servo_id - 1] = position
        logger.debug("Mock: Group moved servos %s to positions %s", servo_ids, positions)
    
    def get_position(self, servo_id: int) -> int:
        """Get simulated servo position."""
        if 1 <= servo_id <= 6:
            return self.positions[servo_id - 1]
        return 0
    
    def get_temperature(self, servo_id: int) -> int:
        """Get simulated temperature."""
        return self.temperatures[servo_id - 1] if 1 <= servo_id <= 6 else 25
    
    def get_max_temperature_limit(self, servo_id: int) -> int:
        """Get max temperature limit."""
        return 85
    
    def get_voltage(self, servo_id: int) -> int:
        """Get simulated voltage."""
        return self.voltages[servo_id - 1] if 1 <= servo_id <= 6 else 7400
    
    def get_led_errors(self, servo_id: int) -> int:
        """Get simulated LED errors."""
        return self.errors[servo_id - 1] if 1 <= servo_id <= 6 else 0
    
    def set_motor_mode(self, servo_id: int, mode: int):
        """Set motor mode (simulation)."""
        logger.debug("Mock: Set servo %d to motor mode %d", servo_id, mode)

# Initialize controller
controller = None
if HARDWARE_AVAILABLE:
    try:
        import serial
        controller = lewansoul_lx16a.ServoController(serial.Serial(SERIAL_PORT, 115200, timeout=0.1))
        logger.info("Successfully initialized servo controller on %s", SERIAL_PORT)
    except (serial.SerialException, Exception) as e:
        logger.warning("Failed to connect to hardware: %s. Using mock controller.", e)
        controller = MockServoController()
else:
    controller = MockServoController()

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

def forward_kinematics(theta_rad: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray]]:
    """Compute end effector pose and all joint transforms."""
    T = np.eye(4)
    transforms = [T.copy()]  # Base transform
    
    for i, (theta, d, a, alpha) in enumerate(DH_PARAMS):
        T_joint = dh_transform(theta_rad[i] if i < 5 else 0, d, a, alpha)
        T = T @ T_joint
        transforms.append(T.copy())
    
    return T, transforms

def positions_to_radians_improved(positions: List[float]) -> np.ndarray:
    """Improved conversion from servo positions to joint angles in radians."""
    theta_rad = np.zeros(len(positions))
    
    for i, pos in enumerate(positions):
        pos_min, pos_max = POS_RANGES[i]
        
        if i == 0:  # Base rotation (Joint 1) - Full 360 degrees
            # Map servo range to -180 to +180 degrees
            theta_deg = -180 + (pos - pos_min) / (pos_max - pos_min) * 360
        elif i == 1:  # Shoulder (Joint 2) - Typical range -90 to +90
            # Map servo range to reasonable shoulder movement
            theta_deg = -90 + (pos - pos_min) / (pos_max - pos_min) * 180
        elif i == 2:  # Elbow (Joint 3) - Typical range -90 to +90  
            # Map servo range to reasonable elbow movement
            theta_deg = -90 + (pos - pos_min) / (pos_max - pos_min) * 180
        elif i == 3:  # Wrist tilt (Joint 4) - Pitch movement
            # Map servo range to wrist pitch
            theta_deg = -90 + (pos - pos_min) / (pos_max - pos_min) * 180
        elif i == 4:  # Wrist rotation (Joint 5) - Roll movement
            # Map servo range to full rotation
            theta_deg = -180 + (pos - pos_min) / (pos_max - pos_min) * 360
        else:
            theta_deg = 0
            
        theta_rad[i] = np.radians(theta_deg)
    
    return theta_rad


def radians_to_positions_improved(theta_rad: np.ndarray) -> List[float]:
    """Improved conversion from joint angles in radians to servo positions."""
    positions = []
    
    for i, theta in enumerate(theta_rad):
        theta_deg = np.degrees(theta)
        pos_min, pos_max = POS_RANGES[i]
        
        if i == 0:  # Base rotation
            # Normalize to -180 to +180 range
            theta_deg = ((theta_deg + 180) % 360) - 180
            pos = pos_min + (theta_deg + 180) / 360 * (pos_max - pos_min)
        elif i == 1:  # Shoulder
            pos = pos_min + (theta_deg + 90) / 180 * (pos_max - pos_min)
        elif i == 2:  # Elbow
            pos = pos_min + (theta_deg + 90) / 180 * (pos_max - pos_min)
        elif i == 3:  # Wrist tilt
            pos = pos_min + (theta_deg + 90) / 180 * (pos_max - pos_min)
        elif i == 4:  # Wrist rotation
            # Normalize to -180 to +180 range
            theta_deg = ((theta_deg + 180) % 360) - 180
            pos = pos_min + (theta_deg + 180) / 360 * (pos_max - pos_min)
        else:
            pos = (pos_min + pos_max) / 2
            
        positions.append(np.clip(pos, pos_min, pos_max))
    
    return positions

def compute_jacobian(theta_rad: np.ndarray, delta: float = 1e-4) -> np.ndarray:
    """Compute Jacobian matrix numerically."""
    T, _ = forward_kinematics(theta_rad)
    p, z = T[:3, 3], T[:3, 2]
    J = np.zeros((6, 5))
    theta_pert = theta_rad.copy()
    
    for i in range(5):
        theta_pert[i] += delta
        T_pert, _ = forward_kinematics(theta_pert)
        J[:3, i] = (T_pert[:3, 3] - p) / delta
        J[3:6, i] = (T_pert[:3, 2] - z) / delta
        theta_pert[i] = theta_rad[i]
    
    return J

def check_reachability(target_pos: np.ndarray) -> bool:
    """Check if target position is within robot's reach."""
    r = np.sqrt(target_pos[0]**2 + target_pos[1]**2)
    z = target_pos[2]
    
    max_reach = LINK_LENGTHS[0] + LINK_LENGTHS[1] + LINK_LENGTHS[2]
    min_reach = abs(LINK_LENGTHS[0] - LINK_LENGTHS[1] - LINK_LENGTHS[2])
    
    distance_from_base = np.sqrt(r**2 + z**2)
    return min_reach <= distance_from_base <= max_reach

def inverse_kinematics_improved(
    target_pos: np.ndarray,
    target_orient: np.ndarray = np.array([0, 0, -1]),
    max_iter: int = 500,
    tol: float = 1e-3,
    step_size: float = 0.05,
    use_cache: bool = True
) -> List[float]:
    """Improved inverse kinematics solver for 6-DOF robot arm."""

    # Round target position to reduce cache variations
    cache_key = tuple(np.round(target_pos, 2))

    # Check cache first
    if use_cache and cache_key in IK_CACHE:
        logger.info("Using cached IK solution for target position %s", target_pos)
        return IK_CACHE[cache_key]
    
    logger.info("Starting improved IK solver for target position %s", target_pos)
    
    if not check_reachability(target_pos):
        logger.warning("Target position may be unreachable")
    
    # Better initial guess using geometric approach
    x, y, z = target_pos
    
    # Joint 1: Base rotation - simple atan2
    theta1 = np.arctan2(y, x)
    
    # Calculate reach in XY plane and vertical component
    r = np.sqrt(x**2 + y**2)
    
    # Account for wrist offset - move target back by wrist length
    wrist_offset = LINK_LENGTHS[2]  # L4
    target_wrist = target_pos - wrist_offset * target_orient
    
    # Recalculate for wrist position
    x_w, y_w, z_w = target_wrist
    r_w = np.sqrt(x_w**2 + y_w**2)
    
    # 2-link inverse kinematics for shoulder and elbow
    L1, L2 = LINK_LENGTHS[0], LINK_LENGTHS[1]  # L2, L3
    
    # Distance from shoulder to wrist
    D = np.sqrt(r_w**2 + z_w**2)
    
    if D > L1 + L2:
        logger.warning("Target may be out of reach, clamping to workspace boundary")
        D = L1 + L2 - 10  # Small margin
    
    # Law of cosines for elbow angle
    cos_theta3 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta3 = np.clip(cos_theta3, -1, 1)
    
    # Two solutions for elbow - choose elbow up configuration
    theta3 = np.arccos(cos_theta3)  # Elbow up
    
    # Shoulder angle
    alpha = np.arctan2(z_w, r_w)
    beta = np.arctan2(L2 * np.sin(theta3), L1 + L2 * np.cos(theta3))
    theta2 = alpha - beta
    
    # Wrist angles - simplified approach
    # For now, keep wrist level (can be improved)
    theta4 = -(theta2 + theta3)  # Keep end effector pointing down
    theta5 = 0.0  # No wrist rotation initially
    
    # Initial joint angles
    theta_rad = np.array([theta1, theta2, theta3, theta4, theta5])
    
    # Iterative refinement using Jacobian
    for iteration in range(max_iter):
        # Current end effector pose
        T, _ = forward_kinematics(theta_rad)
        current_pos = T[:3, 3]
        current_orient = T[:3, 2]  # Z-axis of end effector
        
        # Position and orientation error
        pos_error = target_pos - current_pos
        orient_error = np.cross(current_orient, target_orient)
        
        # Combined error vector
        error = np.concatenate([pos_error, 0.1 * orient_error])  # Weight orientation less
        error_norm = np.linalg.norm(pos_error)
        
        logger.debug("IK iteration %d: position error = %.3f mm", iteration, error_norm)
        
        if error_norm < tol:
            logger.info("IK converged after %d iterations with error %.3f mm", iteration, error_norm)
            break
        
        # Compute Jacobian
        J = compute_jacobian(theta_rad)
        
        # Damped least squares solution
        lambda_damping = 0.01
        JtJ = J.T @ J + lambda_damping**2 * np.eye(5)
        
        try:
            delta_theta = np.linalg.solve(JtJ, J.T @ error)
        except np.linalg.LinAlgError:
            delta_theta = np.linalg.pinv(J) @ error
        
        # Adaptive step size
        if iteration > 100 and error_norm > 50:
            step_size = max(0.01, step_size * 0.9)
        
        # Update joint angles
        theta_rad += step_size * delta_theta
        
        # Convert to servo positions and back to ensure valid range
        positions = radians_to_positions_improved(theta_rad)
        theta_rad = positions_to_radians_improved(positions)
        
        # Early termination if not improving
        if iteration > 50 and error_norm > 100:
            logger.warning("IK not converging well, may need better initial guess")
            break
    
    else:
        logger.warning("IK did not converge within %d iterations. Final error: %.3f mm", max_iter, error_norm)
    
    # Convert final angles to servo positions
    final_positions = radians_to_positions_improved(theta_rad)
    
    logger.info("Final joint angles (deg): [%.1f, %.1f, %.1f, %.1f, %.1f]", 
                *np.degrees(theta_rad))
    logger.info("Final servo positions: %s", [int(p) for p in final_positions])
    
    # Store result in cache
    if use_cache:
        if len(IK_CACHE) >= IK_CACHE_SIZE:
            # Remove oldest entry
            IK_CACHE.pop(next(iter(IK_CACHE)))
        IK_CACHE[cache_key] = final_positions
    
    return final_positions

def return_to_home() -> None:
    """Move all servos to home position."""
    home_positions = [705, 865, 430, 100, 500, GRIPPER_POS[1]]
    logger.info("Returning to home position: %s", home_positions)
    try:
        if hasattr(controller, 'group_move'):
            controller.group_move(list(range(1, 7)), [int(pos) for pos in home_positions], MOVE_TIME)
        else:
            for i, pos in enumerate(home_positions, 1):
                controller.move(i, int(pos), MOVE_TIME)
                time.sleep(0.1)
    except Exception as e:
        logger.error("Error returning to home: %s", e)
        for i, pos in enumerate(home_positions, 1):
            try:
                controller.move(i, int(pos), MOVE_TIME)
                time.sleep(0.1)
            except Exception as e:
                logger.error("Failed to move servo %d: %s", i, e)

class RobotVisualizer:
    """3D visualization of the robot arm using matplotlib."""
    
    def __init__(self, parent_frame):
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, parent_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Initialize plot
        self.setup_plot()
        
    def setup_plot(self):
        """Setup the 3D plot."""
        self.ax.set_xlim([-400, 400])
        self.ax.set_ylim([-400, 400])
        self.ax.set_zlim([0, 400])
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('Robot Arm Visualization')
        
        # Set viewing angle
        self.ax.view_init(elev=20, azim=45)
        
        # Create persistent objects for faster updates
        colors = ['red', 'green', 'blue', 'orange', 'purple']
        self.link_lines = [self.ax.plot([], [], [], 'o-', lw=2, color=colors[i % len(colors)])[0] 
                          for i in range(5)]
        self.joint_points = [self.ax.plot([], [], [], 'o', ms=8, color=colors[i % len(colors)])[0] 
                            for i in range(6)]
        self.target_point = self.ax.plot([], [], [], 'r*', ms=15)[0]
        
        # Draw workspace boundary (only once)
        max_reach = sum(LINK_LENGTHS)
        theta = np.linspace(0, 2*np.pi, 50)
        x_circle = max_reach * np.cos(theta)
        y_circle = max_reach * np.sin(theta)
        z_circle = np.zeros_like(theta)
        self.ax.plot(x_circle, y_circle, z_circle, 'k--', alpha=0.3, label='Max Reach')
        
        # Draw base (only once)
        self.ax.scatter(0, 0, 0, color='gray', s=200, marker='o', alpha=1.0)
        
    def draw_cylinder(self, start, end, radius=10, color='blue', alpha=0.7):
        """Draw a cylinder between two points."""
        # Create cylinder mesh
        height = np.linalg.norm(np.array(end) - np.array(start))
        if height == 0:
            return
            
        # Create cylinder coordinates
        theta = np.linspace(0, 2*np.pi, 20)
        z_cyl = np.linspace(0, height, 10)
        theta_mesh, z_mesh = np.meshgrid(theta, z_cyl)
        x_mesh = radius * np.cos(theta_mesh)
        y_mesh = radius * np.sin(theta_mesh)
        
        # Transform cylinder to align with start-end vector
        direction = np.array(end) - np.array(start)
        direction = direction / np.linalg.norm(direction)
        
        # Simple alignment (could be improved)
        points = np.array([x_mesh.flatten(), y_mesh.flatten(), z_mesh.flatten()]).T
        
        # Rotate to align with direction (simplified)
        if not np.allclose(direction, [0, 0, 1]):
            # This is a simplified rotation - in practice you'd want proper rotation matrices
            pass
        
        # Translate to start position
        points += np.array(start)
        
        # Plot as scatter points (simplified visualization)
        self.ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                       c=color, alpha=alpha, s=1)
    
    def draw_robot(self, joint_positions: List[float], target_pos: Optional[np.ndarray] = None):
        """Draw the robot arm based on joint positions using optimized updates."""
        # Convert positions to angles
        theta_rad = positions_to_radians_improved(joint_positions[:5])
        
        # Get all transformation matrices
        _, transforms = forward_kinematics(theta_rad)
        
        # Extract joint positions
        joint_coords = [T[:3, 3] for T in transforms]
        
        # Update link lines and joint points
        for i, line in enumerate(self.link_lines):
            if i < len(joint_coords) - 1:
                start = joint_coords[i]
                end = joint_coords[i+1]
                x_data = [start[0], end[0]]
                y_data = [start[1], end[1]]
                z_data = [start[2], end[2]]
                line.set_data(x_data, y_data)
                line.set_3d_properties(z_data)
        
        # Update joint positions
        for i, point in enumerate(self.joint_points):
            if i < len(joint_coords):
                point.set_data([joint_coords[i][0]], [joint_coords[i][1]])
                point.set_3d_properties([joint_coords[i][2]])
            else:
                point.set_data([], [])
                point.set_3d_properties([])
        
        # Update target position
        if target_pos is not None:
            self.target_point.set_data([target_pos[0]], [target_pos[1]])
            self.target_point.set_3d_properties([target_pos[2]])
        else:
            self.target_point.set_data([], [])
            self.target_point.set_3d_properties([])
        
        # Only draw canvas once - much more efficient
        self.canvas.draw_idle()
    
    def update_target(self, target_pos: np.ndarray):
        """Update target position visualization."""
        if target_pos is not None:
            self.target_point.set_data([target_pos[0]], [target_pos[1]])
            self.target_point.set_3d_properties([target_pos[2]])
        else:
            self.target_point.set_data([], [])
            self.target_point.set_3d_properties([])
        self.canvas.draw_idle()

class RobotArmGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Robot Arm Control with 3D Visualization")
        self.root.geometry("1400x800")
        
        # State variables
        self.target_pos = np.array([200.0, 0.0, 200.0])
        self.is_moving = False
        self.manual_control = [False] * SERVO_COUNT
        self.simulation_mode = not HARDWARE_AVAILABLE or isinstance(controller, MockServoController)
        
        # Initialize movement queue and worker thread
        self.movement_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self.movement_worker, daemon=True)
        self.worker_thread.start()
        
        # Create main layout
        self.create_layout()
        self.create_control_frames()
        
        logger.info("GUI initialized in %s mode", "simulation" if self.simulation_mode else "hardware")
        
        # Start periodic updates
        self.update_gui()
        
    def movement_worker(self):
        """Background thread for processing movement commands."""
        while True:
            try:
                # Get next movement task
                task = self.movement_queue.get()
                if task is None:  # Shutdown signal
                    break
                    
                target_pos, gripper_open = task
                
                # Perform IK calculation
                positions = inverse_kinematics_improved(target_pos)
                
                # Move servos
                if hasattr(controller, 'group_move'):
                    controller.group_move(list(range(1, 6)), [int(pos) for pos in positions], MOVE_TIME)
                else:
                    for i, pos in enumerate(positions, 1):
                        controller.move(i, int(pos), MOVE_TIME)
                
                # Handle gripper
                gripper_pos = GRIPPER_POS[0] if gripper_open else GRIPPER_POS[1]
                controller.move(6, gripper_pos, MOVE_TIME)
                
                # Wait for movement to complete without blocking GUI
                time.sleep(MOVE_TIME / 1000 + 0.1)
                
                # Update GUI state from worker thread
                self.root.after(0, self._set_movement_complete)
                
            except Exception as e:
                logger.error("Movement worker error: %s", e)
                # Update GUI state from worker thread on error
                self.root.after(0, self._set_movement_complete)
            finally:
                self.movement_queue.task_done()
                
    def _set_movement_complete(self):
        """Helper method to safely update GUI state from worker thread."""
        self.is_moving = False
        logger.info("Movement completed")
    
    def create_layout(self):
        """Create the main layout with control panel and visualization."""
        # Create main panes
        main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Control panel (left side)
        self.control_frame = ttk.Frame(main_paned, width=600)
        main_paned.add(self.control_frame, weight=1)
        
        # Visualization panel (right side)
        viz_frame = ttk.LabelFrame(main_paned, text="3D Visualization", width=800)
        main_paned.add(viz_frame, weight=2)
        
        # Initialize visualizer
        self.visualizer = RobotVisualizer(viz_frame)
        
        # Mode indicator
        mode_label = ttk.Label(self.control_frame, 
                              text=f"Mode: {'Simulation' if self.simulation_mode else 'Hardware'}",
                              font=('Arial', 12, 'bold'),
                              foreground='blue' if self.simulation_mode else 'green')
        mode_label.pack(pady=5)
    
    def create_control_frames(self):
        """Create all control frames."""
        self.create_input_frame()
        self.create_servo_frame() 
        self.create_position_frame()
        self.create_ik_equation_frame()
    
    def create_input_frame(self):
        """Create input fields for target position and gripper state."""
        frame = ttk.LabelFrame(self.control_frame, text="Target Position", padding=10)
        frame.pack(pady=5, fill="x")
        
        # Position inputs
        ttk.Label(frame, text="X (mm):").grid(row=0, column=0, sticky="w")
        self.x_entry = ttk.Entry(frame, width=10)
        self.x_entry.grid(row=0, column=1, padx=5)
        self.x_entry.insert(0, "200")
        
        ttk.Label(frame, text="Y (mm):").grid(row=1, column=0, sticky="w")
        self.y_entry = ttk.Entry(frame, width=10)
        self.y_entry.grid(row=1, column=1, padx=5)
        self.y_entry.insert(0, "0")
        
        ttk.Label(frame, text="Z (mm):").grid(row=2, column=0, sticky="w")
        self.z_entry = ttk.Entry(frame, width=10)
        self.z_entry.grid(row=2, column=1, padx=5)
        self.z_entry.insert(0, "200")
        
        # Buttons
        button_frame = ttk.Frame(frame)
        button_frame.grid(row=3, column=0, columnspan=3, pady=10)
        
        ttk.Button(button_frame, text="Move", command=self.move_to_position).pack(side=tk.LEFT, padx=2)
        ttk.Button(button_frame, text="Home", command=self.return_to_home).pack(side=tk.LEFT, padx=2)
        ttk.Button(button_frame, text="Random", command=self.random_movements).pack(side=tk.LEFT, padx=2)
        
        # Test buttons
        test_frame = ttk.Frame(frame)
        test_frame.grid(row=4, column=0, columnspan=3, pady=5)
        
        ttk.Button(test_frame, text="Test Left", command=self.test_left_position).pack(side=tk.LEFT, padx=2)
        ttk.Button(test_frame, text="Test Right", command=self.test_right_position).pack(side=tk.LEFT, padx=2)
        ttk.Button(test_frame, text="Test Forward", command=self.test_forward_position).pack(side=tk.LEFT, padx=2)
        
        # Gripper control
        gripper_frame = ttk.Frame(frame)
        gripper_frame.grid(row=5, column=0, columnspan=3, pady=5)
        
        ttk.Label(gripper_frame, text="Gripper:").pack(side=tk.LEFT)
        self.gripper_var = tk.BooleanVar()
        ttk.Checkbutton(gripper_frame, text="Open", variable=self.gripper_var).pack(side=tk.LEFT, padx=5)
    
    def create_servo_frame(self):
        """Create display for servo information and sliders."""
        frame = ttk.LabelFrame(self.control_frame, text="Servo Status", padding=10)
        frame.pack(pady=5, fill="x")
        
        # Create scrollable frame for servos
        canvas = tk.Canvas(frame, height=200)
        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        self.servo_labels = {}
        self.servo_sliders = {}
        
        for i in range(1, SERVO_COUNT + 1):
            servo_frame = ttk.Frame(scrollable_frame)
            servo_frame.pack(fill="x", pady=2)
            
            ttk.Label(servo_frame, text=f"Servo {i}:", width=8).pack(side=tk.LEFT)
            
            self.servo_labels[i] = ttk.Label(servo_frame, text="", width=40)
            self.servo_labels[i].pack(side=tk.LEFT, padx=5)
            
            pos_min, pos_max = POS_RANGES[i-1]
            slider = tk.Scale(servo_frame, from_=pos_min, to=pos_max, orient=tk.HORIZONTAL,
                              length=150, command=lambda pos, servo=i: self.on_slider_move(servo, pos))
            slider.pack(side=tk.RIGHT, padx=5)
            self.servo_sliders[i] = slider
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
    
    def create_position_frame(self):
        """Create display for current and target positions."""
        frame = ttk.LabelFrame(self.control_frame, text="Position Info", padding=10)
        frame.pack(pady=5, fill="x")
        
        self.current_pos_label = ttk.Label(frame, text="Current Position: (0.0, 0.0, 0.0) mm")
        self.current_pos_label.pack(anchor="w")
        
        self.target_pos_label = ttk.Label(frame, text="Target Position: (200.0, 0.0, 200.0) mm")
        self.target_pos_label.pack(anchor="w")
        
        self.reachability_label = ttk.Label(frame, text="Reachability: Unknown")
        self.reachability_label.pack(anchor="w")
        
        self.error_label = ttk.Label(frame, text="Position Error: 0.0 mm")
        self.error_label.pack(anchor="w")
    
    def create_ik_equation_frame(self):
        """Create display for inverse kinematics equation."""
        frame = ttk.LabelFrame(self.control_frame, text="Joint Angles", padding=10)
        frame.pack(pady=5, fill="x")
        
        self.ik_label = ttk.Label(frame, text="θ1=0°, θ2=0°, θ3=0°, θ4=0°, θ5=0°", font=('Courier', 10))
        self.ik_label.pack(anchor="w")
    
    def test_left_position(self):
        """Test movement to left side."""
        self.set_target_entries(-150, 150, 200)
    
    def test_right_position(self):
        """Test movement to right side."""
        self.set_target_entries(150, 150, 200)
    
    def test_forward_position(self):
        """Test movement to forward position."""
        self.set_target_entries(0, 250, 150)
    
    def set_target_entries(self, x: float, y: float, z: float):
        """Set target position entries."""
        self.x_entry.delete(0, tk.END)
        self.x_entry.insert(0, str(x))
        self.y_entry.delete(0, tk.END)
        self.y_entry.insert(0, str(y))
        self.z_entry.delete(0, tk.END)
        self.z_entry.insert(0, str(z))
    
    def update_gui(self):
        """Update GUI with real-time servo and position data."""
        start_time = time.time()
        try:
            # Get current servo positions for visualization
            current_positions = []
            
            for i in range(1, SERVO_COUNT + 1):
                try:
                    pos = controller.get_position(i)
                    temp = controller.get_temperature(i)
                    temp_limit = controller.get_max_temperature_limit(i)
                    voltage = controller.get_voltage(i)
                    errors = controller.get_led_errors(i)
                    
                    # Store position for visualization
                    current_positions.append(pos)
                    
                    self.servo_labels[i].config(
                        text=f"Pos: {pos}, Temp: {temp}°C/{temp_limit}°C, Voltage: {voltage}mV, Errors: {errors}"
                    )
                    if not self.manual_control[i-1]:
                        self.servo_sliders[i].set(pos)
                    logger.debug("Updated servo %d: position=%d, temp=%d°C, voltage=%dmV", i, pos, temp, voltage)
                except Exception as e:
                    self.servo_labels[i].config(text=f"Error: {e}")
                    logger.error("Error updating servo %d: %s", i, e)
                    # Use default position if error occurs
                    if len(current_positions) < i:
                        current_positions.append(500)  # Default middle position
            
            # Calculate current end effector position using forward kinematics
            # Use only the first 5 servo positions for kinematics (excluding gripper)
            theta_rad = positions_to_radians_improved(current_positions[:5])
            T, _ = forward_kinematics(theta_rad)  # Fixed: properly unpack the tuple
            current_pos = T[:3, 3]  # Extract position from transformation matrix
            
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
                    text=f"Target Reachable: {'Yes' if is_reachable else 'No'}"
                )
                
                # Update target position for visualization
                self.target_pos = test_pos
                
            except ValueError:
                self.reachability_label.config(text="Reachability: Invalid input")
            
            self.target_pos_label.config(
                text=f"Target Position: ({self.target_pos[0]:.1f}, {self.target_pos[1]:.1f}, {self.target_pos[2]:.1f}) mm"
            )
            
            # Calculate position error
            position_error = np.linalg.norm(self.target_pos - current_pos)
            self.error_label.config(text=f"Position Error: {position_error:.1f} mm")
            
            # Update IK display
            theta_degrees = np.degrees(theta_rad)
            self.ik_label.config(
                text=f"θ1={theta_degrees[0]:.1f}°, θ2={theta_degrees[1]:.1f}°, θ3={theta_degrees[2]:.1f}°, θ4={theta_degrees[3]:.1f}°, θ5={theta_degrees[4]:.1f}°"
            )
            
            # **KEY FIX: Update the 3D visualization with current robot state**
            try:
                # Pass current positions to visualizer
                self.visualizer.draw_robot(current_positions, self.target_pos)
            except Exception as e:
                logger.error("Error updating visualization: %s", e)
            
        except Exception as e:
            logger.error("Error updating GUI: %s", e)
        
        # Adaptive update rate based on processing time
        elapsed = time.time() - start_time
        update_interval = max(100, min(500, int(100 / elapsed * 100))) if elapsed > 0 else 500
        logger.debug("GUI update took %.3f seconds, next update in %d ms", elapsed, update_interval)
        self.root.after(update_interval, self.update_gui)
    
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
            target_pos = np.array([x, y, z])
            logger.info("Attempting to move to position: %s", target_pos)
            
            # Check reachability before attempting move
            if not check_reachability(target_pos):
                messagebox.showwarning("Warning", "Target position may be unreachable!")
            
            self.is_moving = True
            self.target_pos = target_pos

            # Queue the movement task - this is all we need now
            self.movement_queue.put((self.target_pos, self.gripper_var.get()))
            
        except ValueError as e:
            logger.error("Invalid input values: %s", e)
            messagebox.showerror("Error", f"Invalid input values: {e}")
            self.is_moving = False
        except Exception as e:
            logger.error("Movement error: %s", e)
            messagebox.showerror("Error", f"Movement error: {e}")
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
        # Convert NumPy boolean to Python boolean to avoid Tkinter error
        self.gripper_var.set(bool(np.random.choice([True, False])))
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