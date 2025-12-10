# ROT3U Robotic Arm - 5-DOF + 1 Gripper Kinematics

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Python](https://img.shields.io/badge/python-3.8%2B-blue)
![Platform](https://img.shields.io/badge/platform-linux%7Cwindows-lightgrey)

<img src="docs/robot_arm_preview.png" align="right" width="300" alt="ROT3U Robot Arm">

A comprehensive Python application for controlling and simulating the ROT3U robotic arm, featuring forward/inverse kinematics, trajectory planning, and real-time visualization.

## 🚀 Features

- ✅ **Forward Kinematics**: Calculate end-effector position from joint angles
- ✅ **Inverse Kinematics**: Calculate joint angles from target positions
- ✅ **Real-time Visualization**: 2D plotting of robot configurations
- ✅ **Arduino Integration**: Hardware control and communication
- ✅ **Trajectory Planning**: Smooth path generation between waypoints
- ✅ **Modern GUI**: User-friendly interface built with CustomTkinter
- ✅ **Workspace Analysis**: Reachability and collision detection
- ✅ **Multiple Solutions**: Elbow-up and elbow-down configurations

## 📋 Requirements

- Python 3.8 or higher
- pip (Python package installer)

## 🛠️ Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/jaysh/5-1DOF_kinematics.git
   cd 5-1DOF_kinematics
   ```

2. **Install Python dependencies:**
   
   **Option A - Minimal installation (recommended):**
   ```bash
   pip install -r requirements_minimal.txt
   ```
   
   **Option B - Full installation with optional features:**
   ```bash
   pip install -r requirements.txt
   ```

3. **Run the application:**
   ```bash
   python code/main.py
   ```

## 🎯 Usage

### GUI Application

Start the graphical interface:
```bash
python code/main.py
```

The GUI provides three main tabs:
- **Forward Kinematics**: Input joint angles -> Get X,Y,Z position
- **Inverse Kinematics**: Input X,Y,Z position -> Get joint angles
- **Settings**: Configure Arduino connection

### Python API Example

You can use the kinematics library in your own scripts. See `code/example_usage.py` for a complete example.

```python
from kinematics.Kinematics import Kinematic
import math

# Initialize
robot = Kinematic()

# Forward Kinematics
joint_angles = [0, 45, 90, 45, 0, 0]  # degrees
transform = robot.forward_kinematics([math.radians(a) for a in joint_angles])
print(f"End Effector: {transform[:3, 3]}")

# Inverse Kinematics
target_pos = [15, 0, 20]  # cm
solutions = robot.inverse_kinematics(target_pos)
print(f"Found {len(solutions)} solutions")
```

To run the full example script:
```bash
python code/example_usage.py
```

## 🔌 Arduino Communication

The application communicates with an Arduino-controlled robot arm via serial.

**Packet Format:** `angle1,angle2,angle3,angle4,angle5,angle6,mode\n`

- **Angles**: 0-180 degrees
- **Mode**: `s` (sequential) or `c` (concurrent)

See `src/testCode.cpp` for the Arduino firmware implementation.

## 🤖 Robot Specifications

| Link | Length (cm) | Joint | Range |
|------|-------------|-------|-------|
| Shoulder | 10.5 | Base | 0-180° |
| Elbow | 10.0 | Shoulder | 0-180° |
| Wrist | 16.8 | Elbow | 0-180° |
| | | Wrist Pitch | 0-150° |
| | | Wrist Roll | 0-180° |
| | | Gripper | 0-180° |

## 📁 Project Structure

```
5-1DOF_kinematics/
├── code/
│   ├── GUI/              # Graphical User Interface
│   ├── kinematics/       # Core math & kinematics
│   ├── main.py           # Entry point
│   └── example_usage.py  # API examples
├── src/                  # Arduino firmware
├── docs/                 # Documentation images
└── requirements.txt      # Dependencies
```

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the project
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

**Happy Coding! 🚀**
