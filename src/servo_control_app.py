import sys
import time
import serial
import threading
import json
import lewansoul_lx16a
from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_socketio import SocketIO
import serial.tools.list_ports

# Import the LewanSoul LX16A controller
# Fix the import path to point to the correct location
sys.path.append('c:\\Users\\besto\\OneDrive\\Documents\\Python Scripts\\RobotArm\\LewanSoul\\src')

# Configuration
SERIAL_PORT = "COM4"
SERVO_COUNT = 6

# Servo movement constraints (from LewanTest.py)
pos_ranges = [
    (190, 1180),  # Servo 1: Base rotation
    (55, 915),    # Servo 2: Shoulder
    (-190, 497),  # Servo 3: Elbow
    (-190, 394),  # Servo 4: Wrist tilt
    (0, 1000),    # Servo 5: Wrist rotation
    (288, 607)    # Servo 6: Gripper (closed to open)
]

# Initialize Flask app
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

# Initialize serial port and servo controller
controller = None
serial_port = None
serial_lock = threading.Lock()  # Lock for thread-safe serial access

def initialize_controller(max_retries=3, retry_delay=1):
    """Initialize the servo controller with retry logic."""
    global controller, serial_port
    for attempt in range(max_retries):
        try:
            # Check if port is available
            available_ports = [port.device for port in serial.tools.list_ports.comports()]
            if SERIAL_PORT not in available_ports:
                print(f"Error: {SERIAL_PORT} not found. Available ports: {available_ports}")
                return False
            
            # Attempt to open serial port
            serial_port = serial.Serial(SERIAL_PORT, 115200, timeout=0.1)
            controller = lewansoul_lx16a.ServoController(serial_port)
            print(f"Connected to servo controller on {SERIAL_PORT}")
            return True
        except serial.SerialException as e:
            print(f"Attempt {attempt + 1}/{max_retries} - Error connecting to {SERIAL_PORT}: {e}")
            if attempt < max_retries - 1:
                time.sleep(retry_delay)
        except Exception as e:
            print(f"Unexpected error: {e}")
            return False
    return False

# Store current servo positions
servo_positions = [0] * SERVO_COUNT

# Function to get servo information
def get_servo_info(servo_id):
    with serial_lock:  # Ensure thread-safe access
        if controller is None or serial_port is None or not serial_port.is_open:
            return {
                "id": servo_id,
                "position": 0,
                "temperature": 0,
                "voltage": 0,
                "error": "Controller not connected",
                "min": pos_ranges[servo_id-1][0],
                "max": pos_ranges[servo_id-1][1]
            }
        
        try:
            position = controller.get_position(servo_id)
            temperature = controller.get_temperature(servo_id)
            voltage = controller.get_voltage(servo_id)
            error = controller.get_led_errors(servo_id)
            
            # Calculate speed (approximation)
            speed = 0
            
            return {
                "id": servo_id,
                "position": position,
                "temperature": temperature,
                "voltage": voltage,
                "error": error,
                "speed": speed,
                "min": pos_ranges[servo_id-1][0],
                "max": pos_ranges[servo_id-1][1]
            }
        except Exception as e:
            return {
                "id": servo_id,
                "position": 0,
                "temperature": 0,
                "voltage": 0,
                "error": str(e),
                "min": pos_ranges[servo_id-1][0],
                "max": pos_ranges[servo_id-1][1]
            }

# Background thread for updating servo information
def background_thread():
    while True:
        if controller is not None and serial_port is not None and serial_port.is_open:
            all_servo_info = [get_servo_info(i+1) for i in range(SERVO_COUNT)]
            socketio.emit('servo_update', {'data': all_servo_info})
        time.sleep(0.5)  # Update every 500ms

# Routes
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/servo/<int:servo_id>', methods=['GET'])
def get_servo(servo_id):
    if 1 <= servo_id <= SERVO_COUNT:
        return jsonify(get_servo_info(servo_id))
    return jsonify({"error": "Invalid servo ID"}), 400

@app.route('/api/servo/<int:servo_id>', methods=['POST'])
def set_servo(servo_id):
    with serial_lock:  # Ensure thread-safe access
        if controller is None or serial_port is None or not serial_port.is_open:
            return jsonify({"error": "Controller not connected"}), 500
        
        if 1 <= servo_id <= SERVO_COUNT:
            data = request.json
            position = int(data.get('position', 0))
            
            # Ensure position is within valid range
            min_pos, max_pos = pos_ranges[servo_id-1]
            position = max(min_pos, min(max_pos, position))
            
            try:
                # Move servo with a time of 500ms for smooth movement
                controller.move(servo_id, position, 500)
                servo_positions[servo_id-1] = position
                return jsonify({"success": True, "position": position})
            except Exception as e:
                return jsonify({"error": str(e)}), 500
        
        return jsonify({"error": "Invalid servo ID"}), 400

@app.route('/api/servos', methods=['GET'])
def get_all_servos():
    all_servo_info = [get_servo_info(i+1) for i in range(SERVO_COUNT)]
    return jsonify(all_servo_info)

@app.route('/api/urdf', methods=['GET'])
def get_urdf():
    import os
    urdf_filename = 'so_100_arm.urdf'
    urdf_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'files'))
    return send_from_directory(urdf_dir, urdf_filename)

# SocketIO events
@socketio.on('connect')
def handle_connect():
    print('Client connected')
    # Start background thread only once
    if not hasattr(app, 'thread'):
        app.thread = socketio.start_background_task(background_thread)

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

# Cleanup on shutdown
def cleanup():
    global controller, serial_port
    if serial_port is not None and serial_port.is_open:
        try:
            serial_port.close()
            print(f"Closed serial port {SERIAL_PORT}")
        except Exception as e:
            print(f"Error closing serial port: {e}")
    controller = None
    serial_port = None

# Register cleanup on app shutdown
import atexit
atexit.register(cleanup)

# Main entry point
if __name__ == '__main__':
    # Create templates and static directories if they don't exist
    import os
    if not os.path.exists('templates'):
        os.makedirs('templates')
    if not os.path.exists('static'):
        os.makedirs('static')
    
    # Initialize controller
    if not initialize_controller():
        print("Failed to initialize servo controller. Exiting...")
        sys.exit(1)
    
    # Run the application
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=True)
    finally:
        cleanup()