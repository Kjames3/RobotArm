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
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Import subprocesses module to launch Rviz and IsaacSim from GUI
import subprocess

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
    (190, 1180),  # Servo 1: Base rotation (Rotation)
    (55, 915),    # Servo 2: Shoulder (Pitch)
    (-190, 497),  # Servo 3: Elbow (Elbow)
    (-190, 394),  # Servo 4: Wrist tilt (Wrist_Pitch)
    (0, 1000),    # Servo 5: Wrist rotation (Wrist_Roll)
    (288, 607)    # Servo 6: Gripper (Jaw)
]
LINK_LENGTHS: Tuple[float, float, float] = (150.0, 150.0, 90.0)  # L2, L3, L4 in mm
GRIPPER_POS: Tuple[int, int] = (288, 607)  # Open, Closed
DH_PARAMS: List[Tuple[float, float, float, float]] = [
    (0, 50, 0, np.pi/2),          # Joint 1: Base rotation (vertical axis) with base height
    (0, 0, 0, np.pi/2),           # Joint 2: Shoulder (vertical movement)
    (0, 0, LINK_LENGTHS[0], 0),   # Joint 3: Upper arm (horizontal when shoulder at 0)
    (0, 0, LINK_LENGTHS[1], 0),   # Joint 4: Forearm (horizontal movement)
    (0, 0, 0, np.pi/2),           # Joint 5: Wrist tilt (vertical axis)
    (0, LINK_LENGTHS[2], 0, 0)    # Joint 6: End effector offset
]

# Joint names from ROS 2 topic
JOINT_NAMES = ['', 'Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll', 'Jaw']

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
        # Use joint angle for first 5 joints, 0 for the end effector offset
        joint_angle = theta_rad[i] if i < len(theta_rad) else 0
        T_joint = dh_transform(joint_angle + theta, d, a, alpha)
        T = T @ T_joint
        transforms.append(T.copy())
    
    return T, transforms

def positions_to_radians_improved(positions: List[float]) -> np.ndarray:
    """Improved conversion from servo positions to joint angles in radians."""
    theta_rad = np.zeros(len(positions))
    
    # Home positions for reference (vertical arm configuration)
    home_positions = [705, 865, 430, 100, 500]
    
    for i, pos in enumerate(positions):
        pos_min, pos_max = POS_RANGES[i]
        home_pos = home_positions[i] if i < len(home_positions) else (pos_min + pos_max) / 2
        
        if i == 0:  # Base rotation (Joint 1) - Full 360 degrees
            # Map servo range to -180 to +180 degrees
            theta_deg = -180 + (pos - pos_min) / (pos_max - pos_min) * 360
        elif i == 1:  # Shoulder (Joint 2) - Vertical movement
            # Home position should be vertical (90 degrees), map around that
            normalized_pos = (pos - home_pos) / (pos_max - pos_min) * 180
            theta_deg = 90 + normalized_pos  # Start vertical, move from there
        elif i == 2:  # Upper arm (Joint 3) - Should be straight up when shoulder is vertical
            normalized_pos = (pos - home_pos) / (pos_max - pos_min) * 180
            theta_deg = 90 + normalized_pos  # 90 degrees = straight up, 0 = horizontal
        elif i == 3:  # Forearm (Joint 4) - Bend from straight
            normalized_pos = (pos - home_pos) / (pos_max - pos_min) * 180
            theta_deg = normalized_pos  # 0 degrees = straight, positive = bend
        elif i == 4:  # Wrist rotation (Joint 5) - Roll movement
            theta_deg = -180 + (pos - pos_min) / (pos_max - pos_min) * 360
        else:
            theta_deg = 0
            
        theta_rad[i] = np.radians(theta_deg)
    
    return theta_rad

def radians_to_positions_improved(theta_rad: np.ndarray) -> List[float]:
    """Improved conversion from joint angles in radians to servo positions."""
    positions = []
    
    # Home positions for reference (vertical arm configuration)
    home_positions = [705, 865, 430, 100, 500]
    
    for i, theta in enumerate(theta_rad):
        theta_deg = np.degrees(theta)
        pos_min, pos_max = POS_RANGES[i]
        home_pos = home_positions[i] if i < len(home_positions) else (pos_min + pos_max) / 2
        
        if i == 0:  # Base rotation
            theta_deg = ((theta_deg + 180) % 360) - 180
            pos = pos_min + (theta_deg + 180) / 360 * (pos_max - pos_min)
        elif i == 1:  # Shoulder
            normalized_pos = theta_deg - 90
            normalized_pos = np.clip(normalized_pos, -90, 90)
            pos = home_pos + normalized_pos / 180 * (pos_max - pos_min)
        elif i == 2:  # Elbow
            normalized_pos = theta_deg
            normalized_pos = np.clip(normalized_pos, -90, 90)
            pos = home_pos + normalized_pos / 180 * (pos_max - pos_min)
        elif i == 3:  # Wrist tilt
            pos = pos_min + (theta_deg + 90) / 180 * (pos_max - pos_min)
        elif i == 4:  # Wrist rotation
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
    cache_key = tuple(np.round(target_pos, 2))
    
    if use_cache and cache_key in IK_CACHE:
        logger.info("Using cached IK solution for target position %s", target_pos)
        return IK_CACHE[cache_key]
    
    logger.info("Starting improved IK solver for target position %s", target_pos)
    
    if not check_reachability(target_pos):
        logger.warning("Target position may be unreachable")
    
    x, y, z = target_pos
    theta1 = np.arctan2(y, x)
    r = np.sqrt(x**2 + y**2)
    wrist_offset = LINK_LENGTHS[2]
    target_wrist = target_pos - wrist_offset * target_orient
    x_w, y_w, z_w = target_wrist
    r_w = np.sqrt(x_w**2 + y_w**2)
    L1, L2 = LINK_LENGTHS[0], LINK_LENGTHS[1]
    D = np.sqrt(r_w**2 + z_w**2)
    
    if D > L1 + L2:
        logger.warning("Target may be out of reach, clamping to workspace boundary")
        D = L1 + L2 - 10
    
    cos_theta3 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta3 = np.clip(cos_theta3, -1, 1)
    theta3 = np.arccos(cos_theta3)
    alpha = np.arctan2(z_w, r_w)
    beta = np.arctan2(L2 * np.sin(theta3), L1 + L2 * np.cos(theta3))
    theta2 = alpha - beta
    theta4 = -(theta2 + theta3)
    theta5 = 0.0
    theta_rad = np.array([theta1, theta2, theta3, theta4, theta5])
    
    for iteration in range(max_iter):
        T, _ = forward_kinematics(theta_rad)
        current_pos = T[:3, 3]
        current_orient = T[:3, 2]
        pos_error = target_pos - current_pos
        orient_error = np.cross(current_orient, target_orient)
        error = np.concatenate([pos_error, 0.1 * orient_error])
        error_norm = np.linalg.norm(pos_error)
        
        logger.debug("IK iteration %d: position error = %.3f mm", iteration, error_norm)
        
        if error_norm < tol:
            logger.info("IK converged after %d iterations with error %.3f mm", iteration, error_norm)
            break
        
        J = compute_jacobian(theta_rad)
        lambda_damping = 0.01
        JtJ = J.T @ J + lambda_damping**2 * np.eye(5)
        
        try:
            delta_theta = np.linalg.solve(JtJ, J.T @ error)
        except np.linalg.LinAlgError:
            delta_theta = np.linalg.pinv(J) @ error
        
        if iteration > 100 and error_norm > 50:
            step_size = max(0.01, step_size * 0.9)
        
        theta_rad += step_size * delta_theta
        positions = radians_to_positions_improved(theta_rad)
        theta_rad = positions_to_radians_improved(positions)
    
    else:
        logger.warning("IK did not converge within %d iterations. Final error: %.3f mm", max_iter, error_norm)
    
    final_positions = radians_to_positions_improved(theta_rad)
    
    logger.info("Final joint angles (deg): [%.1f, %.1f, %.1f, %.1f, %.1f]", 
                *np.degrees(theta_rad))
    logger.info("Final servo positions: %s", [int(p) for p in final_positions])
    
    if use_cache:
        if len(IK_CACHE) >= IK_CACHE_SIZE:
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
        self.ax.view_init(elev=20, azim=45)
        
        colors = ['red', 'green', 'blue', 'orange', 'purple', 'cyan']
        self.link_lines = [self.ax.plot([], [], [], 'o-', lw=2, color=colors[i % len(colors)])[0] 
                          for i in range(6)]
        self.joint_points = [self.ax.plot([], [], [], 'o', ms=8, color=colors[i % len(colors)])[0] 
                            for i in range(7)]
        self.target_point = self.ax.plot([], [], [], 'r*', ms=15)[0]
        self.gripper_lines = [self.ax.plot([], [], [], '-', lw=4, color='darkred', label='Gripper')[0] for _ in range(2)]
        self.wrist_marker = self.ax.plot([], [], [], 's', ms=12, color='magenta', label='Wrist Joint')[0]
        self.end_effector_marker = self.ax.plot([], [], [], '^', ms=10, color='red', label='End Effector')[0]
        self.ax.legend(loc='upper right')
        
        max_reach = sum(LINK_LENGTHS)
        theta = np.linspace(0, 2*np.pi, 50)
        x_circle = max_reach * np.cos(theta)
        y_circle = max_reach * np.sin(theta)
        z_circle = np.zeros_like(theta)
        self.ax.plot(x_circle, y_circle, z_circle, 'k--', alpha=0.3, label='Max Reach')
        self.ax.scatter(0, 0, 0, color='gray', s=200, marker='o', alpha=1.0)
        
    def draw_robot(self, joint_positions: List[float], target_pos: Optional[np.ndarray] = None):
        """Draw the robot arm based on joint positions using optimized updates."""
        theta_rad = positions_to_radians_improved(joint_positions[:5])
        _, transforms = forward_kinematics(theta_rad)
        joint_coords = [T[:3, 3] for T in transforms]
        
        for i, line in enumerate(self.link_lines):
            if i < len(joint_coords) - 1:
                start = joint_coords[i]
                end = joint_coords[i+1]
                x_data = [start[0], end[0]]
                y_data = [start[1], end[1]]
                z_data = [start[2], end[2]]
                line.set_data(x_data, y_data)
                line.set_3d_properties(z_data)
            else:
                line.set_data([], [])
                line.set_3d_properties([])
        
        for i, point in enumerate(self.joint_points):
            if i < len(joint_coords):
                point.set_data([joint_coords[i][0]], [joint_coords[i][1]])
                point.set_3d_properties([joint_coords[i][2]])
            else:
                point.set_data([], [])
                point.set_3d_properties([])
        
        if len(joint_coords) >= 6:
            wrist_pos = joint_coords[5]
            self.wrist_marker.set_data([wrist_pos[0]], [wrist_pos[1]])
            self.wrist_marker.set_3d_properties([wrist_pos[2]])
            
            if len(joint_coords) >= 7:
                end_effector_pos = joint_coords[6]
                self.end_effector_marker.set_data([end_effector_pos[0]], [end_effector_pos[1]])
                self.end_effector_marker.set_3d_properties([end_effector_pos[2]])
            else:
                self.end_effector_marker.set_data([], [])
                self.end_effector_marker.set_3d_properties([])
        else:
            self.wrist_marker.set_data([], [])
            self.wrist_marker.set_3d_properties([])
            self.end_effector_marker.set_data([], [])
            self.end_effector_marker.set_3d_properties([])
        
        if len(joint_coords) > 0:
            end_effector_pos = joint_coords[-1]
            end_effector_transform = transforms[-1]
            x_axis = end_effector_transform[:3, 0]
            y_axis = end_effector_transform[:3, 1]
            z_axis = end_effector_transform[:3, 2]
            
            gripper_length = 20.0
            gripper_width = 15.0
            
            if len(joint_positions) > 5:
                gripper_servo_pos = joint_positions[5]
                gripper_min, gripper_max = GRIPPER_POS
                gripper_opening = (gripper_servo_pos - gripper_min) / (gripper_max - gripper_min)
                gripper_opening = np.clip(gripper_opening, 0, 1)
                actual_gripper_width = gripper_width * (0.3 + 0.7 * gripper_opening)
            else:
                actual_gripper_width = gripper_width * 0.5
            
            jaw_offset = y_axis * (actual_gripper_width / 2)
            jaw_tip_offset = z_axis * gripper_length
            
            jaw1_base = end_effector_pos + jaw_offset
            jaw1_tip = jaw1_base + jaw_tip_offset
            jaw2_base = end_effector_pos - jaw_offset
            jaw2_tip = jaw2_base + jaw_tip_offset
            
            self.gripper_lines[0].set_data([jaw1_base[0], jaw1_tip[0]], [jaw1_base[1], jaw1_tip[1]])
            self.gripper_lines[0].set_3d_properties([jaw1_base[2], jaw1_tip[2]])
            self.gripper_lines[1].set_data([jaw2_base[0], jaw2_tip[0]], [jaw2_base[1], jaw2_tip[1]])
            self.gripper_lines[1].set_3d_properties([jaw2_base[2], jaw2_tip[2]])
        else:
            for line in self.gripper_lines:
                line.set_data([], [])
                line.set_3d_properties([])
        
        if target_pos is not None:
            self.target_point.set_data([target_pos[0]], [target_pos[1]])
            self.target_point.set_3d_properties([target_pos[2]])
        else:
            self.target_point.set_data([], [])
            self.target_point.set_3d_properties([])
        
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

class RobotArmGUI(Node):
    def __init__(self, root: tk.Tk):
        # Initialize ROS 2 node
        super().__init__('robot_arm_gui')
        self.root = root
        self.root.title("Robot Arm Control with ROS 2 Humble")
        self.root.geometry("1400x800")
        
        # State variables
        self.target_pos = np.array([200.0, 0.0, 200.0])
        self.is_moving = False
        self.manual_control = [False] * SERVO_COUNT
        self.simulation_mode = not HARDWARE_AVAILABLE or isinstance(controller, MockServoController)
        self.current_positions = [685, 485, 153, 102, 500, 447]  # Default positions
        self.gripper_open = False
        
        # Initialize movement queue and worker thread
        self.movement_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self.movement_worker, daemon=True)
        self.worker_thread.start()
        
        # Create ROS 2 subscription
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Create main layout
        self.create_layout()
        self.create_control_frames()
        
        logger.info("GUI initialized in %s mode with ROS 2 node", "simulation" if self.simulation_mode else "hardware")
        
        # Start periodic updates
        self.update_gui()
    
    def joint_state_callback(self, msg: JointState):
        """Callback for /joint_states topic."""
        try:
            positions = [0.0] * SERVO_COUNT
            for i, name in enumerate(msg.name):
                if name in JOINT_NAMES[1:]:  # Skip empty name
                    idx = JOINT_NAMES.index(name) - 1  # Adjust for empty name
                    if idx < SERVO_COUNT:
                        # Convert joint angles (radians) to servo positions
                        pos_min, pos_max = POS_RANGES[idx]
                        if idx == 5:  # Gripper (Jaw)
                            # Map 0 to 1 (closed to open) to gripper range
                            gripper_opening = np.clip(msg.position[i], 0.0, 1.0)
                            positions[idx] = pos_min + gripper_opening * (pos_max - pos_min)
                            self.gripper_open = gripper_opening > 0.5
                        else:
                            # Map joint angles (-pi to pi) to servo positions
                            theta_deg = np.degrees(msg.position[i])
                            if idx == 0:  # Rotation
                                theta_deg = ((theta_deg + 180) % 360) - 180
                                positions[idx] = pos_min + (theta_deg + 180) / 360 * (pos_max - pos_min)
                            elif idx == 1:  # Pitch
                                home_pos = 865
                                normalized_pos = np.clip(theta_deg - 90, -90, 90)
                                positions[idx] = home_pos + normalized_pos / 180 * (pos_max - pos_min)
                            elif idx == 2:  # Elbow
                                home_pos = 430
                                normalized_pos = np.clip(theta_deg, -90, 90)
                                positions[idx] = home_pos + normalized_pos / 180 * (pos_max - pos_min)
                            elif idx == 3:  # Wrist_Pitch
                                positions[idx] = pos_min + (theta_deg + 90) / 180 * (pos_max - pos_min)
                            elif idx == 4:  # Wrist_Roll
                                theta_deg = ((theta_deg + 180) % 360) - 180
                                positions[idx] = pos_min + (theta_deg + 180) / 360 * (pos_max - pos_min)
                        positions[idx] = np.clip(positions[idx], pos_min, pos_max)
            
            self.current_positions = positions
            self.movement_queue.put((positions, self.gripper_open))
            logger.info("Received joint states: %s", [int(p) for p in positions])
        except Exception as e:
            logger.error("Error processing joint states: %s", e)
    
    def movement_worker(self):
        """Background thread for processing movement commands."""
        while True:
            try:
                task = self.movement_queue.get()
                if task is None:
                    break
                    
                positions, gripper_open = task
                
                if hasattr(controller, 'group_move'):
                    controller.group_move(list(range(1, 7)), [int(pos) for pos in positions], MOVE_TIME)
                else:
                    for i, pos in enumerate(positions, 1):
                        controller.move(i, int(pos), MOVE_TIME)
                
                time.sleep(MOVE_TIME / 1000 + 0.1)
                self.root.after(0, self._set_movement_complete)
                
            except Exception as e:
                logger.error("Movement worker error: %s", e)
                self.root.after(0, self._set_movement_complete)
            finally:
                self.movement_queue.task_done()
    
    def _set_movement_complete(self):
        """Helper method to safely update GUI state from worker thread."""
        self.is_moving = False
        logger.info("Movement completed")
    
    def create_layout(self):
        """Create the main layout with control panel and visualization."""
        main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.control_frame = ttk.Frame(main_paned, width=600)
        main_paned.add(self.control_frame, weight=1)
        
        viz_frame = ttk.LabelFrame(main_paned, text="3D Visualization", width=800)
        main_paned.add(viz_frame, weight=2)
        
        self.visualizer = RobotVisualizer(viz_frame)
        
        mode_label = ttk.Label(self.control_frame, 
                              text=f"Mode: {'Simulation' if self.simulation_mode else 'Hardware'} (ROS 2)",
                              font=('Arial', 12, 'bold'),
                              foreground='blue' if self.simulation_mode else 'green')
        mode_label.pack(pady=5)
    
    def create_control_frames(self):
        """Create all control frames."""
        self.create_input_frame()
        self.create_servo_frame() 
        self.create_position_frame()
        self.create_ik_equation_frame()
        self.create_launch_frame()
    
    def create_input_frame(self):
        """Create input fields for target position and gripper state."""
        frame = ttk.LabelFrame(self.control_frame, text="Target Position (Read-Only)", padding=10)
        frame.pack(pady=5, fill="x")
        
        ttk.Label(frame, text="X (mm):").grid(row=0, column=0, sticky="w")
        self.x_entry = ttk.Entry(frame, width=10, state='readonly')
        self.x_entry.grid(row=0, column=1, padx=5)
        self.x_entry.insert(0, "200")
        
        ttk.Label(frame, text="Y (mm):").grid(row=1, column=0, sticky="w")
        self.y_entry = ttk.Entry(frame, width=10, state='readonly')
        self.y_entry.grid(row=1, column=1, padx=5)
        self.y_entry.insert(0, "0")
        
        ttk.Label(frame, text="Z (mm):").grid(row=2, column=0, sticky="w")
        self.z_entry = ttk.Entry(frame, width=10, state='readonly')
        self.z_entry.grid(row=2, column=1, padx=5)
        self.z_entry.insert(0, "200")
        
        button_frame = ttk.Frame(frame)
        button_frame.grid(row=3, column=0, columnspan=3, pady=10)
        
        ttk.Button(button_frame, text="Home", command=self.return_to_home).pack(side=tk.LEFT, padx=2)
        
        gripper_frame = ttk.Frame(frame)
        gripper_frame.grid(row=5, column=0, columnspan=3, pady=5)
        
        ttk.Label(gripper_frame, text="Gripper:").pack(side=tk.LEFT)
        self.gripper_var = tk.BooleanVar()
        ttk.Checkbutton(gripper_frame, text="Open", variable=self.gripper_var, state='disabled').pack(side=tk.LEFT, padx=5)
    
    def create_servo_frame(self):
        """Create display for servo information and sliders."""
        frame = ttk.LabelFrame(self.control_frame, text="Servo Status", padding=10)
        frame.pack(pady=5, fill="x")
        
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
                              length=150, state='disabled')
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
    
    def create_launch_frame(self):
        """Create frame with buttons to launch external applications."""
        frame = ttk.LabelFrame(self.control_frame, text="External Applications", padding=10)
        frame.pack(pady=5, fill="x")
        
        button_frame = ttk.Frame(frame)
        button_frame.pack(fill="x")
        
        # Rviz launch button
        rviz_button = ttk.Button(button_frame, text="Launch Rviz", command=self.launch_rviz)
        rviz_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Isaac Sim launch button
        isaac_button = ttk.Button(button_frame, text="Launch Isaac Sim", command=self.launch_isaac_sim)
        isaac_button.pack(side=tk.LEFT, padx=5, pady=5)
    
    def launch_rviz(self):
        """Launch Rviz in a new terminal window."""
        try:
            # Try multiple terminal options to avoid snap conflicts
            terminal_commands = [
                # Try x-terminal-emulator first (system default)
                ['x-terminal-emulator', '-e', 'bash', '-c', 
                 'cd && cd SO-ARM101_MoveIt_IsaacSim && source install/setup.bash && echo "Starting Rviz with MoveIt..." && ros2 launch so_arm_moveit_config demo.launch.py; echo "Rviz closed. Press Enter to exit."; read'],
                
                # Try xterm as fallback
                ['xterm', '-e', 'bash', '-c', 
                 'cd && cd SO-ARM101_MoveIt_IsaacSim && source install/setup.bash && echo "Starting Rviz with MoveIt..." && ros2 launch so_arm_moveit_config demo.launch.py; echo "Rviz closed. Press Enter to exit."; read'],
                
                # Try konsole (KDE terminal)
                ['konsole', '-e', 'bash', '-c', 
                 'cd && cd SO-ARM101_MoveIt_IsaacSim && source install/setup.bash && echo "Starting Rviz with MoveIt..." && ros2 launch so_arm_moveit_config demo.launch.py; echo "Rviz closed. Press Enter to exit."; read'],
                
                # Try gnome-terminal with different approach (avoiding snap conflicts)
                ['/usr/bin/gnome-terminal', '--wait', '--', 'bash', '-c', 
                 'cd && cd SO-ARM101_MoveIt_IsaacSim && source install/setup.bash && echo "Starting Rviz with MoveIt..." && ros2 launch so_arm_moveit_config demo.launch.py; echo "Rviz closed. Press Enter to exit."; read']
            ]
            
            success = False
            for cmd in terminal_commands:
                try:
                    subprocess.Popen(cmd)
                    success = True
                    logger.info("Launched Rviz using: %s", cmd[0])
                    break
                except FileNotFoundError:
                    continue
                except Exception as e:
                    logger.debug("Failed to launch with %s: %s", cmd[0], e)
                    continue
            
            if success:
                messagebox.showinfo("Launch Success", "Rviz launched in new terminal window")
            else:
                raise Exception("No suitable terminal emulator found")
                
        except Exception as e:
            logger.error("Failed to launch Rviz: %s", e)
            messagebox.showerror("Launch Error", f"Failed to launch Rviz: {e}\n\nTry installing xterm: sudo apt install xterm")
    
    def launch_isaac_sim(self):
        """Launch Isaac Sim in a new terminal window."""
        try:
            # Try multiple terminal options to avoid snap conflicts
            terminal_commands = [
                # Try x-terminal-emulator first (system default)
                ['x-terminal-emulator', '-e', 'bash', '-c', 
                 'echo "Launching Isaac Sim..." && isaac-sim.sh; echo "Isaac Sim closed. Press Enter to exit."; read'],
                
                # Try xterm as fallback
                ['xterm', '-e', 'bash', '-c', 
                 'echo "Launching Isaac Sim..." && isaac-sim.sh; echo "Isaac Sim closed. Press Enter to exit."; read'],
                
                # Try konsole (KDE terminal)
                ['konsole', '-e', 'bash', '-c', 
                 'echo "Launching Isaac Sim..." && isaac-sim.sh; echo "Isaac Sim closed. Press Enter to exit."; read'],
                
                # Try gnome-terminal with different approach (avoiding snap conflicts)
                ['/usr/bin/gnome-terminal', '--wait', '--', 'bash', '-c', 
                 'echo "Launching Isaac Sim..." && isaac-sim.sh; echo "Isaac Sim closed. Press Enter to exit."; read']
            ]
            
            success = False
            for cmd in terminal_commands:
                try:
                    subprocess.Popen(cmd)
                    success = True
                    logger.info("Launched Isaac Sim using: %s", cmd[0])
                    break
                except FileNotFoundError:
                    continue
                except Exception as e:
                    logger.debug("Failed to launch with %s: %s", cmd[0], e)
                    continue
            
            if success:
                messagebox.showinfo("Launch Success", "Isaac Sim launched in new terminal window")
            else:
                raise Exception("No suitable terminal emulator found")
                
        except Exception as e:
            logger.error("Failed to launch Isaac Sim: %s", e)
            messagebox.showerror("Launch Error", f"Failed to launch Isaac Sim: {e}\n\nTry installing xterm: sudo apt install xterm")
    
    def update_gui(self):
        """Update GUI with real-time servo and position data."""
        start_time = time.time()
        try:
            current_positions = self.current_positions
            for i in range(1, SERVO_COUNT + 1):
                try:
                    pos = current_positions[i-1]
                    temp = controller.get_temperature(i)
                    temp_limit = controller.get_max_temperature_limit(i)
                    voltage = controller.get_voltage(i)
                    errors = controller.get_led_errors(i)
                    
                    self.servo_labels[i].config(
                        text=f"Pos: {pos:.0f}, Temp: {temp}°C/{temp_limit}°C, Voltage: {voltage}mV, Errors: {errors}"
                    )
                    self.servo_sliders[i].set(pos)
                    logger.debug("Updated servo %d: position=%.0f, temp=%d°C, voltage=%dmV", i, pos, temp, voltage)
                except Exception as e:
                    self.servo_labels[i].config(text=f"Error: {e}")
                    logger.error("Error updating servo %d: %s", i, e)
            
            theta_rad = positions_to_radians_improved(current_positions[:5])
            T, _ = forward_kinematics(theta_rad)
            current_pos = T[:3, 3]
            
            self.current_pos_label.config(
                text=f"Current Position: ({current_pos[0]:.1f}, {current_pos[1]:.1f}, {current_pos[2]:.1f}) mm"
            )
            
            try:
                self.target_pos_label.config(
                    text=f"Target Position: ({self.target_pos[0]:.1f}, {self.target_pos[1]:.1f}, {self.target_pos[2]:.1f}) mm"
                )
                is_reachable = check_reachability(self.target_pos)
                self.reachability_label.config(
                    text=f"Target Reachable: {'Yes' if is_reachable else 'No'}"
                )
            except ValueError:
                self.reachability_label.config(text="Reachability: Invalid input")
            
            position_error = np.linalg.norm(self.target_pos - current_pos)
            self.error_label.config(text=f"Position Error: {position_error:.1f} mm")
            
            theta_degrees = np.degrees(theta_rad)
            self.ik_label.config(
                text=f"θ1={theta_degrees[0]:.1f}°, θ2={theta_degrees[1]:.1f}°, θ3={theta_degrees[2]:.1f}°, θ4={theta_degrees[3]:.1f}°, θ5={theta_degrees[4]:.1f}°"
            )
            
            try:
                self.visualizer.draw_robot(current_positions, self.target_pos)
            except Exception as e:
                logger.error("Error updating visualization: %s", e)
            
        except Exception as e:
            logger.error("Error updating GUI: %s", e)
        
        elapsed = time.time() - start_time
        update_interval = max(100, min(500, int(100 / elapsed * 100))) if elapsed > 0 else 500
        logger.debug("GUI update took %.3f seconds, next update in %d ms", elapsed, update_interval)
        self.root.after(update_interval, self.update_gui)
    
    def return_to_home(self):
        """Move to home position."""
        logger.info("Returning to home position")
        return_to_home()
        self.is_moving = False
        self.target_pos = np.array([0.0, 0.0, 0.0])
        self.current_positions = [705, 865, 430, 100, 500, GRIPPER_POS[1]]
        self.gripper_open = False
    
    def destroy(self):
        """Clean up ROS 2 node and servos on shutdown."""
        super().destroy()
        return_to_home()
        for i in range(1, 7):
            try:
                controller.set_motor_mode(i, 0)
                logger.debug("Set servo %d to motor mode 0", i)
            except Exception as e:
                logger.error("Error setting motor mode for servo %d: %s", i, e)
        self.movement_queue.put(None)  # Signal worker thread to exit

def test_vertical_configuration():
    """Test function to verify the robot displays vertically in home position."""
    print("Testing vertical configuration...")
    
    home_positions = [705, 865, 430, 100, 500]
    theta_rad = positions_to_radians_improved(home_positions)
    print(f"Home joint angles (degrees): {np.degrees(theta_rad)}")
    
    T, transforms = forward_kinematics(theta_rad)
    
    print("Joint positions in home configuration:")
    for i, transform in enumerate(transforms):
        pos = transform[:3, 3]
        print(f"Joint {i}: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
    
    end_effector_pos = T[:3, 3]
    print(f"End effector position: ({end_effector_pos[0]:.1f}, {end_effector_pos[1]:.1f}, {end_effector_pos[2]:.1f})")
    
    if abs(end_effector_pos[0]) < 50 and abs(end_effector_pos[1]) < 50 and end_effector_pos[2] > 200:
        print("✓ Robot appears to be in vertical configuration")
    else:
        print("✗ Robot does not appear to be vertical")
    
    return theta_rad, transforms

def main():
    """Main function to launch GUI with ROS 2."""
    logger.info("Starting robot arm GUI application with ROS 2")
    rclpy.init()
    root = tk.Tk()
    app = RobotArmGUI(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        logger.info("Application interrupted by user")
        app.destroy()
        rclpy.shutdown()
        logger.info("Exiting application")
        sys.exit(0)
    except Exception as e:
        logger.error("GUI error: %s", e)
        app.destroy()
        rclpy.shutdown()
        sys.exit(1)

if __name__ == "__main__":
    test_vertical_configuration()
    main()