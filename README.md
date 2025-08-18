# RobotArm

![Robot Arm Demo](https://via.placeholder.com/600x400?text=Robot+Arm+Demo+Image) (No video currently)

A robotic arm control system built with an Debug board, Python and ROS [Humble]. This project enables precise control of an multi-axis robotic arm for applications in education, research, and automation.

## 🚀 Features

- **Multi-Axis Control**: Supports 5-axis movement (e.g., base rotation, shoulder, elbow, wrist, gripper)
- **Multiple Control Modes**:
  - GUI manual control / Moveit pre-planned control using ROS
  - Programmable sequence execution
  - Inverse kinematics positioning
- **Real-time Feedback**: Position monitoring via [e.g., lx16a servo encoders]
- **Safety Features**: Collision detection and emergency stop
- **Extensible Design**: Modular architecture for adding new components

## 🛠️ Hardware Requirements

- Using the SO100 Robotic arm with 5 degrees of freedom
- Microcontroller: Hiwonder TTL / USB Debugging Board
- Servo Motors: LX16A Servo Motors[5]
- Control Interface: Custom GUI
- Power Supply: 12v 2.5amp 5 cell li-po battery

## 💻 Software Requirements

- **Firmware**: Can run project using Windows / Linux
- **Control Software**: [Python 3.10, ROS Humble]
- **Libraries**:
  - [Servo.h, PyGame, OpenCV, Pytorch, Numpy, ...]
  - [e.g., ROS control packages]

## 📦 Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/Kjames3/RobotArm.git
   cd RobotArm

2. **Navidate to the src folder**:
   
  
