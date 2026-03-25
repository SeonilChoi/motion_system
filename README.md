# motion_system workspace

This workspace is a ROS 2 motion pipeline from joystick input to motor commands.

## Read This First

If you are new, follow this order:

1. Read `ros2/motion_system_pkg/README.md` to see what runs at runtime.
2. Read `lib/robot_manager/README.md` to understand action and gait logic.
3. Read `lib/motor_manager/README.md` to understand low-level motor control.
4. Read `common/common_robot_interface/README.md` and `common/common_motor_interface/README.md` for shared data contracts.

## How Data Moves Through The System

1. Joystick publishes `joy`.
2. `robot_manager_node.py` converts joystick state to high-level robot actions.
3. Robot logic creates motion intent and motor command frames.
4. `motor_manager_node` forwards commands into motor runtime and hardware.
5. Hardware feedback is published as `motor_state`.

## Workspace Layout

- `common/common_robot_interface`: shared Python action/state/joystick types.
- `common/common_motor_interface`: shared C++ motor frame types.
- `lib/robot_manager`: Python robot behavior stack (manager + schedulers + robots).
- `lib/motor_manager`: C++ motor runtime stack (interface + EtherCAT + driver + orchestrator).
- `ros2/motion_system_msgs`: message definitions for motor frame transport.
- `ros2/motion_system_pkg`: ROS nodes, launch files, runtime config.

## Build And Run

From `lab_ws`:

```bash
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run motor side first:

```bash
ros2 launch motion_system_pkg motion_system.launch.py
```

Run teleoperation side:

```bash
ros2 launch motion_system_pkg robot_manager_node.launch.py
```

## Dependencies

- ROS 2
- `yaml-cpp`
- IgH EtherCAT `libethercat` (motor stack)
- ROS `joy` package (teleoperation)
