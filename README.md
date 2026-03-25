# motion_system workspace (`src`)

This workspace provides a ROS 2 based motion stack that separates low-level motor communication, shared interfaces, and high-level robot action scheduling.

## What Is In This Workspace

- `common/common_motor_interface`: shared C++ motor frame types used by the motor communication stack.
- `common/common_robot_interface`: shared Python action/state/joystick enums used by robot-side logic.
- `lib/motor_manager`: C++ motor runtime library for EtherCAT + drive handling.
- `lib/robot_manager`: Python robot action orchestration and scheduler packages.
- `ros2/motion_system_msgs`: ROS 2 message definitions for motor frames.
- `ros2/motion_system_pkg`: ROS 2 nodes and launch files that connect joystick, robot manager, and motor manager.

Legacy copies at repository root can be ignored when they include `COLCON_IGNORE`.

## Build And Run

From `lab_ws`:

```bash
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Start motor control:

```bash
ros2 launch motion_system_pkg motion_system.launch.py
```

Start joystick + robot manager:

```bash
ros2 launch motion_system_pkg robot_manager_node.launch.py
```

System dependencies include IgH EtherCAT Master (`libethercat`) and `yaml-cpp`. Teleoperation also requires ROS `joy`.

## Runtime Responsibilities

- `motor_manager_node` consumes `motor_command`, drives hardware, and publishes `motor_state`.
- `robot_manager_node.py` consumes `joy` and produces robot actions/motor commands through `RobotManager`.
- `motion_system_msgs` defines the motor frame messages used between these nodes.

## Main Runtime Parameters

- `motor_manager_node` uses a `config_file` parameter to load hardware and driver configuration.
- `robot_manager_node.py` uses `config_file` and `stride_length` to choose robot model, loop timing, and walk timing behavior.
