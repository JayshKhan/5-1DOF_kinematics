# Technical Documentation: ROT3U Robotic Arm

## 1. Executive Summary

This document provides a comprehensive technical overview of the ROT3U Robotic Arm project. It covers the system architecture, mathematical foundations of the kinematics engine, the serial communication protocol, and the embedded firmware logic. This report is intended for developers, engineers, and researchers wishing to understand or extend the system.

## 2. System Architecture

The system operates on a client-server architecture:

-   **Host (Client)**: A Python-based desktop application running on a PC (Linux/Windows). It handles user input (GUI), kinematic calculations (inverse/forward), trajectory planning, and visualization.
-   **Controller (Server)**: An Arduino Uno (or compatible) controlling an Adafruit PWM Servo Driver to drive 6 servo motors.

### Data Flow
1.  **User Input**: Target coordinates (X, Y, Z) are entered via the GUI.
2.  **Kinematics Engine**: The Python backend calculates the required joint angles using Inverse Kinematics (IK).
3.  **Validation**: Angles are checked against physical constraints.
4.  **Serialization**: Valid angles are formatted into a string packet.
5.  **Transmission**: The packet is sent over USB/Serial to the Arduino.
6.  **Actuation**: The Arduino parses the packet and updates the PWM signals to move the servos.

## 3. Kinematics Mathematical Model

The geometric solution for the ROT3U arm (5-DOF) is derived using the Denavit-Hartenberg (DH) parameters.

### 3.1 Robot Dimensions
-   `a1` (Shoulder to Elbow): 10.5 cm
-   `a2` (Elbow to Wrist): 10.0 cm
-   `a3` (Wrist to End-Effector): 16.8 cm

### 3.2 Inverse Kinematics (IK) Algorithm

The IK solver determines joint angles $(\theta_0, \theta_1, \theta_2, \theta_3)$ for a given target position $(P_x, P_y, P_z)$ and orientation ($\phi$).

1.  **Base Rotation ($\theta_0$)**:
    $$ \theta_0 = \text{atan2}(P_y, P_x) $$

2.  **Planar Projection**:
    The 3D problem is reduced to a 2D planar problem by projecting the target onto the plane defined by $\theta_0$:
    $$ P_{x\_proj} = \sqrt{P_x^2 + P_y^2} $$

3.  **Wrist Center Calculation**:
    $$ W_x = P_{x\_proj} - a_3 \cos(\phi) $$
    $$ W_z = P_z - a_3 \sin(\phi) $$

4.  **Elbow Angle ($\theta_2$)**:
    Using the Law of Cosines:
    $$ \cos(\theta_2) = \frac{W_x^2 + W_z^2 - a_1^2 - a_2^2}{2 a_1 a_2} $$
    $$ \theta_2 = \pm \text{acos}(\cos(\theta_2)) $$
    *Note: Two solutions exist (elbow-up and elbow-down).*

5.  **Shoulder Angle ($\theta_1$)**:
    $$ \theta_1 = \text{atan2}(W_z, W_x) - \text{atan2}(a_2 \sin(\theta_2), a_1 + a_2 \cos(\theta_2)) $$

6.  **Wrist Pitch ($\theta_3$)**:
    $$ \theta_3 = \phi - \theta_1 - \theta_2 $$

### 3.3 Forward Kinematics (FK)
FK is calculated by multiplying the homogeneous transformation matrices for each joint:
$$ T_{total} = T_0^1 \cdot T_1^2 \cdot T_2^3 \cdot T_3^4 \cdot T_{\text{end}} $$

Where each $T_i^{i+1}$ follows the standard DH matrix form.

## 4. Serial Communication Protocol

The communication is asynchronous, half-duplex, over UART (USB Serial).

-   **Baud Rate**: 9600 / 115200 (Configurable, Firmware uses 115200)
-   **Line Ending**: `\n` (Newline character)

### 4.1 Packet Structure

**Command**:
```csv
angle1,angle2,angle3,angle4,angle5,angle6,mode\n
```

-   `angle1` to `angle6` (float/int): Target angles in degrees (0-180).
-   `mode` (char): Movement mode.
    -   `'s'`: **Sequential**. Servos move one by one.
    -   `'c'`: **Concurrent**. Servos move simultaneously (default).

**Example**:
```text
90.0,45.5,135.0,90.0,0.0,0.0,c\n
```

### 4.2 Firmware Implementation (`testCode.cpp`)

The firmware logic breaks down as follows:

1.  **Setup**: Initializes Serial at 115200 baud and the PWM driver.
2.  **Parsing**: The code (though primarily a game demo in `testCode.cpp`) typically implemented `Serial.parseInt()` or similar logic to extract the comma-separated values.
    - *Note: The provided `testCode.cpp` seems to be a specialized "Tic-Tac-Toe" demo with hardcoded positions (`TrisPos`, `TakePos`) and game logic (`Check_Win`, `Computer_Move`). For general kinematic control, a dedicated parser firmware is usually required, but the structure implies `Set_Arm(x, y, z)` is the core actuation function.*

3.  **Actuation (`Set_Arm`)**:
    The functionality in `testCode.cpp` is surprisingly advanced; it contains an **embedded inverse kinematics solver** (`Set_Arm` function) running directly on the Arduino:
    ```cpp
    void Set_Arm(float x, float y, float z) {
        // Calculates x1, x2, x3, x4 based on geometry
        // Moves servos using MyServoWriteGradual or MyServoWriteAll
    }
    ```
    This means the system supports **both** host-side kinematics (sending angles) and on-board kinematics (sending coordinates), depending on the specific firmware routine active.

## 5. Python API Reference

### `Kinematic` Class (`kinematics/Kinematics.py`)

Primary interface for mathematical operations.

#### `forward_kinematics(joint_angles: List[float]) -> np.ndarray`
Returns the 4x4 transformation matrix for the given joint configuration.

#### `inverse_kinematics(target_position: List[float]) -> List[List[float]]`
Returns a list of all possible valid joint configurations (solutions) for a target [x, y, z] coordinate.

#### `find_best_solution(solutions, current_angles)`
Heuristic algorithm to select the optimal solution that minimizes total joint movement from the current state.

### `Module3` (`Module3.py`)

Utility module including:
-   `generate_shape_trajectory(shape, num_points)`: procedural generation of waypoints.
-   `generate_smooth_trajectory(joint_angles)`: Interpolation for smooth motion.

## 6. Recommendations & Future Work

1.  **Firmware Standardization**: The current `testCode.cpp` is a game demo. A clean, dedicated firmware that simply accepts joint angles (`angle1`...`angle6`) would make the system more robust for general-purpose use.
2.  **Error Handling**: Implementing a localized checksum (CRC) in the serial packet would prevent erratic movements due to command corruption.
3.  **Feedback Loop**: Adding current sensing or potentiometer feedback from the servos would allow for closed-loop control.

---
*Report generated by Antigravity Agent - 2025*
