import sys
import time
import serial
import numpy as np
from typing import List, Tuple
import tkinter as tk
from tkinter import messagebox
import lewansoul_lx16a

# Constants
SERIAL_PORT: str = "COM4"
SERVO_COUNT: int = 6
MOVE_TIME: int = 1000  # ms
POS_RANGES: List[Tuple[int, int]] = [
    (200, 1180),  # Servo 1: Base rotation
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
except serial.SerialException as e:
    print(f"Error connecting to serial port: {e}")
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

def return_to_home() -> None:
    """Move all servos to home position."""
    home_positions = [706, 857, 428, 100, 500, GRIPPER_POS[1]]
    try:
        controller.group_move(list(range(1, 7)), [int(pos) for pos in home_positions], MOVE_TIME)
    except AttributeError:
        for i, pos in enumerate(home_positions, 1):
            controller.move(i, int(pos), MOVE_TIME)
            time.sleep(0.1)

class RobotArmGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Robot Arm Control")
        self.root.geometry("600x600")
        
        # State variables
        self.target_pos = np.array([0.0, 0.0, 0.0])
        self.is_moving = False
        self.free_move_mode = False
        
        # Create GUI elements
        self.create_input_frame()
        self.create_servo_frame()
        self.create_position_frame()
        self.create_ik_equation_frame()
        
        # Start periodic updates
        self.update_gui()
    
    def create_input_frame(self):
        """Create input fields for target position and gripper state."""
        frame = tk.Frame(self.root)
        frame.pack(pady=10)
        
        tk.Label(frame, text="Target Position (x y z in mm):").grid(row=0, column=0, columnspan=3)
        self.x_entry = tk.Entry(frame, width=10)
        self.y_entry = tk.Entry(frame, width=10)
        self.z_entry = tk.Entry(frame, width=10)
        self.x_entry.grid(row=1, column=0)
        self.y_entry.grid(row=1, column=1)
        self.z_entry.grid(row=1, column=2)
        
        tk.Label(frame, text="Gripper State:").grid(row=2, column=0)
        self.gripper_var = tk.StringVar(value="closed")
        tk.Radiobutton(frame, text="Open", variable=self.gripper_var, value="open").grid(row=2, column=1)
        tk.Radiobutton(frame, text="Closed", variable=self.gripper_var, value="closed").grid(row=2, column=2)
        
        tk.Button(frame, text="Move to Position", command=self.move_to_position).grid(row=3, column=0, columnspan=3, pady=5)
        tk.Button(frame, text="Return to Home", command=self.return_to_home).grid(row=4, column=0, columnspan=3, pady=5)
        tk.Button(frame, text="Toggle Free Move", command=self.toggle_free_move).grid(row=5, column=0, columnspan=3, pady=5)
    
    def create_servo_frame(self):
        """Create display for servo information."""
        frame = tk.LabelFrame(self.root, text="Servo Status", padx=5, pady=5)
        frame.pack(pady=10, fill="x")
        
        self.servo_labels = {}
        for i in range(1, SERVO_COUNT + 1):
            tk.Label(frame, text=f"Servo {i}:").grid(row=i, column=0, sticky="w")
            self.servo_labels[i] = tk.Label(frame, text="", width=50)
            self.servo_labels[i].grid(row=i, column=1, sticky="w")
    
    def create_position_frame(self):
        """Create display for current and target positions."""
        frame = tk.LabelFrame(self.root, text="Position", padx=5, pady=5)
        frame.pack(pady=10, fill="x")
        
        self.current_pos_label = tk.Label(frame, text="Current Position: (0.0, 0.0, 0.0)")
        self.current_pos_label.pack()
        self.target_pos_label = tk.Label(frame, text="Target Position: (0.0, 0.0, 0.0)")
        self.target_pos_label.pack()
    
    def create_ik_equation_frame(self):
        """Create display for inverse kinematics equation."""
        frame = tk.LabelFrame(self.root, text="Inverse Kinematics Equation", padx=5, pady=5)
        frame.pack(pady=10, fill="x")
        
        equation = "Δθ = (J^T J + λ²I)^(-1) J^T e, where e = [p_d - p; k(z_d - z)]"
        tk.Label(frame, text=equation, font=("Courier", 10)).pack()
    
    def update_gui(self):
        """Update GUI with real-time servo and position data."""
        try:
            # Update servo information
            for i in range(1, SERVO_COUNT + 1):
                pos = controller.get_position(i)
                temp = controller.get_temperature(i)
                temp_limit = controller.get_max_temperature_limit(i)
                voltage = controller.get_voltage(i)
                errors = controller.get_led_errors(i)
                self.servo_labels[i].config(
                    text=f"Pos: {pos}, Temp: {temp}°C/{temp_limit}°C, Voltage: {voltage}mV, Errors: {errors}"
                )
                
                # Warn if temperature or voltage is critical
                if temp > temp_limit * 0.9:
                    self.servo_labels[i].config(fg="red")
                elif voltage < 4500:  # Assuming 4.5V as critical threshold
                    self.servo_labels[i].config(fg="orange")
                else:
                    self.servo_labels[i].config(fg="black")
            
            # Update current position
            current_positions = [controller.get_position(i + 1) for i in range(5)]
            theta_rad = positions_to_radians(current_positions)
            T = forward_kinematics(theta_rad)
            current_pos = T[:3, 3]
            self.current_pos_label.config(
                text=f"Current Position: ({current_pos[0]:.1f}, {current_pos[1]:.1f}, {current_pos[2]:.1f}) mm"
            )
            
            # Update target position
            self.target_pos_label.config(
                text=f"Target Position: ({self.target_pos[0]:.1f}, {self.target_pos[1]:.1f}, {self.target_pos[2]:.1f}) mm"
            )
            
        except Exception as e:
            print(f"Error updating GUI: {e}")
        
        # Schedule next update
        self.root.after(500, self.update_gui)
    
    def move_to_position(self):
        """Handle move to target position and gripper state."""
        if self.is_moving or self.free_move_mode:
            messagebox.showwarning("Warning", "Movement in progress or free move mode active, please wait or exit free move.")
            return
        
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            z = float(self.z_entry.get())
            self.target_pos = np.array([x, y, z])
            gripper_state = self.gripper_var.get()
            
            max_reach = sum(LINK_LENGTHS)
            if np.linalg.norm(self.target_pos) > max_reach:
                messagebox.showerror("Error", f"Position {self.target_pos} exceeds max reach of {max_reach:.1f} mm")
                return
            
            self.is_moving = True
            # Ensure servos are in servo mode
            for i in range(1, SERVO_COUNT + 1):
                controller.set_motor_mode(i, 1)
            
            target_positions = inverse_kinematics(self.target_pos, k=0)
            
            try:
                controller.group_move(list(range(1, 6)), [int(pos) for pos in target_positions], MOVE_TIME)
            except AttributeError:
                for i, pos in enumerate(target_positions, 1):
                    controller.move(i, int(pos), MOVE_TIME)
                    time.sleep(0.1)
            
            gripper_pos = GRIPPER_POS[0] if gripper_state == "open" else GRIPPER_POS[1]
            controller.move(6, gripper_pos, MOVE_TIME)
            time.sleep(1.5)  # Wait for movement to complete
            self.is_moving = False
            
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid numbers for x, y, z.")
            self.is_moving = False
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred: {e}")
            self.is_moving = False
    
    def return_to_home(self):
        """Move to home position."""
        if self.is_moving or self.free_move_mode:
            messagebox.showwarning("Warning", "Movement in progress or free move mode active, please wait or exit free move.")
            return
        
        self.is_moving = True
        # Ensure servos are in servo mode
        for i in range(1, SERVO_COUNT + 1):
            controller.set_motor_mode(i, 1)
        return_to_home()
        time.sleep(1.5)
        self.is_moving = False
        self.target_pos = np.array([0.0, 0.0, 0.0])
    
    def toggle_free_move(self):
        """Toggle free move mode to allow manual servo movement."""
        if self.is_moving:
            messagebox.showwarning("Warning", "Movement in progress, please wait.")
            return
        
        self.free_move_mode = not self.free_move_mode
        if self.free_move_mode:
            try:
                for i in range(1, SERVO_COUNT + 1):
                    controller.set_motor_mode(i, 0)  # Disable torque
                messagebox.showinfo("Free Move", "Servos are now loose. Move them manually and observe positions in the GUI.")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to enable free move mode: {e}")
                self.free_move_mode = False
        else:
            try:
                for i in range(1, SERVO_COUNT + 1):
                    controller.set_motor_mode(i, 1)  # Re-enable servo mode
                messagebox.showinfo("Free Move", "Free move mode disabled. Servos are back in control mode.")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to disable free move mode: {e}")
                self.free_move_mode = True  # Revert to free move if re-enabling fails

def main():
    """Main function to launch GUI."""
    root = tk.Tk()
    app = RobotArmGUI(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\nOperation interrupted by user.")
        return_to_home()
        for i in range(1, 7):
            controller.set_motor_mode(i, 0)
        print("Exiting...")
        sys.exit(0)
    except Exception as e:
        print(f"GUI error: {e}")
        return_to_home()
        for i in range(1, 7):
            controller.set_motor_mode(i, 0)
        sys.exit(1)

if __name__ == "__main__":
    main()