# motion_system_pkg

Only **public** functions, methods, types, and **launch-declared arguments** (plus ROS parameters where they are part of the node contract).

---

## C++ — `MotorManagerNode` (`include/motion_system_pkg/motor_manager_node.hpp`)

| Kind | Symbol | Description |
|------|--------|-------------|
| Type alias | `MotorStatus` | `motion_system_msgs::msg::MotorStatus`. |
| (ctor) | `MotorManagerNode(const rclcpp::NodeOptions& options = {})` | Creates the node, reads `config_file`, constructs `MotorManager`, starts `run()` on a thread, subscribes `motor_command`, publishes `motor_state`, starts a 1 ms timer. |
| (dtor) | `~MotorManagerNode()` | `request_exit()` on `MotorManager` (stops RT loop), joins the run thread. |

| ROS parameter | Description |
|---------------|-------------|
| `config_file` | Path to motor YAML; empty string causes constructor to throw. |

**Entry point:** `main` in `src/motor_manager_node.cpp` — `rclcpp::init`, spin `MotorManagerNode`, `rclcpp::shutdown`.

---

## Python — `scripts/robot_manager_node.py`

### Public types (module level)

| Name | Description |
|------|-------------|
| `JoyAxes` | Indices into `sensor_msgs/Joy.axes` (`LEFT_HORIZONTAL` = 0 … `UP_DOWN_DIRECTION` = 7). |
| `JoyButton` | Indices into `Joy.buttons` (`A` = 0 … `RIGHT_AXES` = 10). |

### `RobotManagerNode` — public methods

| Method | Description |
|--------|-------------|
| `__init__` | Declares/reads `config_file`, constructs `RobotManager`, creates subscriptions (`joy`, `motor_state`), publisher (`motor_command`), timer (`dt`). |
| `joy_callback` | Validates joystick “mode”, then caches axis and button states. |
| `motor_state_callback` | Copies `MotorStatus` into internal `JointStatus` and calls `RobotManager.update_joint_status`. |
| `timer_callback` | Control loop: robot selection, actions, `set_action_frame`, publish `motor_command`. |

### Public function

| Function | Description |
|----------|-------------|
| `main` | `rclpy.init`, spin `RobotManagerNode`, destroy node and shutdown. |

| ROS parameter | Description |
|---------------|-------------|
| `config_file` | Path to robot YAML. |

---

## Launch — public API

### `launch/motor_manager_node.launch.py`

| Function | Description |
|----------|-------------|
| `generate_launch_description()` | Returns a `LaunchDescription` with `motor_manager_node`. |

| Argument | Default | Description |
|----------|---------|-------------|
| `config_file` | `…/share/motion_system_pkg/config/ethercat_integrated.yaml` | Motor stack YAML. |

### `launch/robot_manager_node.launch.py`

| Function | Description |
|----------|-------------|
| `generate_launch_description()` | Includes `joy.launch.py` and starts `robot_manager_node`. |

| Argument | Default | Description |
|----------|---------|-------------|
| `device_id` | `0` | Joystick index (`/dev/input/js{N}`). |
| `deadzone` | `0.05` | Axis deadzone for `joy_node`. |
| `autorepeat_rate` | `20.0` | Autorepeat rate (Hz) for `joy_node`. |
| `config_file` | `…/config/silver_lain.yaml` | Robot YAML for `RobotManager`. |
| `stride_length` | `0.1` | Passed as a node parameter (may be unused by the script). |

### `launch/joy.launch.py`

| Function | Description |
|----------|-------------|
| `generate_launch_description()` | Starts `joy` package `joy_node`. |

| Argument | Default | Description |
|----------|---------|-------------|
| `device_id` | `0` | Joystick index. |
| `deadzone` | `0.05` | Axis deadzone. |
| `autorepeat_rate` | `20.0` | Autorepeat (Hz); `0.0` disables. |
