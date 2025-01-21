# px4_pose

This package allows publishing the drone's transformation, position, path, etc., using ROS 2 standard messages to leverage tools like RViz and develop new applications such as teleoperation.

## Description
The `px4_pose` package contains three main nodes:

1. **`drone_pose`**: Publishes the `/tf` transformation between the drone frame (`base_link`) and the fixed map frame (`map`).
   - **Subscribed Topics:**
     - `/fmu/out/vehicle_odometry`: Receives the drone's odometry, including position, orientation, and velocity.
   - **Published Topics:**
     - `/vehicle_path`: Publishes the trajectory traveled by the drone as a sequence of poses.
     - `/vehicle_pose`: Publishes the drone's current pose (position and orientation) in the fixed map frame.
     - `/vehicle_velocity`: Publishes a visual marker to represent the drone's velocity as an arrow.
     - `/tf`: Publishes transformations between reference frames for navigation and visualization.

2. **`teleop_twist_keyboard`**: Allows generating velocity commands using the computer keyboard.
   - **Published Topics:**
     - `/offboard_velocity_cmd`: Publishes linear and angular velocity commands to control the drone.
     - `/arm_message`: Publishes commands to arm/disarm the drone using the space bar.
   - **Parameters:**
     - `speed` (default value: 0.5): Default linear speed for movements.
     - `turn` (default value: 0.2): Default angular speed for rotations.

3. **`drone_pose_control`**: Controls the drone's position using a finite state machine based on the received velocity commands.
   - **Subscribed Topics:**
     - `/fmu/out/vehicle_status`: Provides the current vehicle status (navigation state, landing required, etc.).
     - `/offboard_velocity_cmd`: Receives velocity commands to move the drone in offboard mode.
     - `/arm_message`: Receives a boolean message indicating whether the drone should be armed.
     - `/fmu/out/vehicle_attitude`: Provides vehicle attitude data (quaternion) used to calculate direction and rotation.
     - `/fmu/in/offboard_control_mode`: Indicates the control currently applied to the drone.
   - **Published Topics:**
     - `/fmu/in/trajectory_setpoint`: Sends position and velocity reference points for the drone's trajectory.
     - `/fmu/in/vehicle_command`: Sends specific commands such as arming/disarming the vehicle or taking off to an initial height.

> **Note**: To execute this package, you must have the `iris_description` package installed: [https://github.com/FredMSaico/iris_description](https://github.com/FredMSaico/iris_description).

---

## Installation

### Clone the Repository
In a ROS 2 workspace, clone the repository from GitHub:

```bash
cd px4_ws/src
git clone https://github.com/FredMSaico/px4_pose.git
```

### Build
Build the package using the following command:

```bash
cd px4_ws && colcon build --packages-select px4_pose
```

---

## Execution

### Verify `drone_pose` and `drone_pose_control` Nodes

Launch the nodes using the following command:

```bash
ros2 launch px4_pose drone_pose.launch.py
```

### Teleoperation

To manually control the drone using the keyboard, execute:

```bash
ros2 run px4_pose teleop_twist_keyboard
```
#### RC Controls - Mode 2

Using the arrow keys and WASD, you can control the drone with Mode 2 RC controls as follows:

- **W**: Move Up  
- **S**: Move Down  
- **A**: Yaw Left  
- **D**: Yaw Right  
- **Up Arrow**: Pitch Forward  
- **Down Arrow**: Pitch Backward  
- **Left Arrow**: Roll Left  
- **Right Arrow**: Roll Right  

##### Additional Command:
- **SPACE**: Arm/Disarm the drone

---

## License

This package is licensed under the [GPL-3.0 License](https://www.gnu.org/licenses/gpl-3.0.html).
