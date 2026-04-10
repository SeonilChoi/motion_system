# motion_system_pkg

---

## `scripts/robot_manager_node.py`

### Variables (module)

None (only imports and definitions below).

### Classes

#### `JoyAxes` (`Enum`)

Joystick axis indices into `sensor_msgs/Joy.axes`.

| Member | Value |
|--------|-------|
| `LEFT_HORIZONTAL` | 0 |
| `LEFT_VERTICAL` | 1 |
| `LT` | 2 |
| `RIGHT_HORIZONTAL` | 3 |
| `RIGHT_VERTICAL` | 4 |
| `RT` | 5 |
| `LEFT_RIGHT_DIRECTION` | 6 |
| `UP_DOWN_DIRECTION` | 7 |

#### `JoyButton` (`Enum`)

Joystick button indices into `sensor_msgs/Joy.buttons`.

| Member | Value |
|--------|-------|
| `A` … `RIGHT_AXES` | 0 … 10 |

#### `RobotManagerNode` (`rclpy.node.Node`)

| Instance attribute | Description |
|--------------------|-------------|
| `_config_file` | ROS parameter `config_file` (robot YAML path). |
| `_robot_manager` | `RobotManager` instance. |
| `_selected_robot_id` | Index of robot receiving button/walk commands. |
| `_number_of_motors`, `_number_of_robots` | From `RobotManager`. |
| `_is_valid_joy_stick` | Set true when `LEFT_RIGHT_DIRECTION` axis reads `1` once. |
| `_joy_axes`, `_joy_buttons` | Latest `Joy` state. |
| `_prev_joy_axes`, `_prev_joy_buttons` | Previous cycle (edge detection). |
| `_joy_button_action` | Maps `A→HOME`, `B→MOVE`, `X→WALK`, `Y→STOP`. |
| `_joint_status` | Global motor feedback buffer (`motor_id` indices, p/v/t zeros initially). |
| `_robot_status` | Cached `RobotStatus` for selected robot (updated each timer tick). |
| `_curr_action` | One `ActionFrame` per robot; starts as `STOP`. |
| `_joy_sub`, `_motor_state_sub`, `_motor_command_pub`, `_timer` | ROS entities (`joy`, `motor_state`, `motor_command`, period `dt`). |

| Method | Description |
|--------|-------------|
| `__init__` | Declares `config_file`, builds `RobotManager`, state, subscribers/publisher, timer. |
| `_check_joy_stick_mode(mode)` | If `mode == 1`, enables joystick and logs. |
| `_select_robot(up_down)` | Changes `_selected_robot_id` by `int(up_down)` modulo robot count. |
| `_round_direction(direction)` | Snaps 2D unit direction to one of 8 compass sectors (π/8 bins). |
| `_normalize_joy_command(joy_axes)` | Reads left stick + right horizontal as walk command; uses `stride_length` / `duration` from `RobotManager` for selected robot; returns `ActionFrame(WALK, goal=[dx,dy,wz], duration=...)`. |
| `_publish_joint_command(joint_command)` | Builds `MotorStatus`: `number_of_target_interfaces` all `1`, `target_interface_id[i]` as `Int8MultiArray([interface_id[i]])`, `controller_index` from `motor_id`, copies p/v/t; publishes `motor_command`. |
| `motor_state_callback` | Copies `motor_state` arrays into `_joint_status`, calls `update_joint_status`. |
| `joy_callback` | Until valid mode, only checks `LEFT_RIGHT_DIRECTION`; then caches all axes/buttons. |
| `timer_callback` | Updates `_robot_status`; if joystick invalid, returns; on up/down axis edge calls `_select_robot`; forces `STOP` `ActionFrame` for robots in `State.STOPPED`; on button rising edge sets selected robot’s action; if WALK, updates counters when scheduler `progress == 1.0` and sets action from `_normalize_joy_command(self._for_data_collection_joy_axes())`; calls `set_action_frame` and `_publish_joint_command`; deep-copies joy state to previous. |

### Functions

| Function | Description |
|----------|-------------|
| `_joy_qos()` | QoS depth 1, best effort, keep last (for `joy` subscription). |
| `_motor_status_qos()` | Same profile (motor topics). |
| `_pose_qos()` | Same profile (defined but unused in this script). |
| `main()` | `rclpy.init`, spin `RobotManagerNode`, shutdown. |

---

## `include/motion_system_pkg/motor_manager_node.hpp`

### Classes

#### `MotorManagerNode` (`rclcpp::Node`)

| Member | Description |
|--------|-------------|
| `MotorStatus` | Type alias `motion_system_msgs::msg::MotorStatus`. |
| `motor_command_subscriber_` | Subscription to `motor_command`. |
| `motor_status_publisher_` | Publisher to `motor_state`. |
| `motor_status_timer_` | Wall timer driving feedback publish. |
| `config_file_` | Path from parameter `config_file`. |
| `motor_manager_` | `motor_manager::MotorManager` instance. |
| `manager_run_thread_` | Thread running `MotorManager::run()`. |

Public: constructor, destructor. Private: `motor_command_callback`, `timer_callback`.

### Functions

None at namespace scope (implementations in `.cpp`).

---

## `src/motor_manager_node.cpp`

### Variables (module)

None.

### Classes

(Same `MotorManagerNode`; behavior below.)

| Method | Description |
|--------|-------------|
| Constructor | Node name `motor_manager_node`; subscribes `motor_command` (QoS 1 best effort); advertises `motor_state`; 1 ms wall timer → `timer_callback`; requires non-empty `config_file` param or throws; constructs `MotorManager`, starts `run()` in `manager_run_thread_`. |
| Destructor | `request_stop()`, joins thread. |
| `motor_command_callback` | Fills `motor_frame_t[]` from message (target interfaces, controller index, controlword, statusword, errorcode, p/v/t); `write` to `MotorManager` with `controller_index.size()` as count. |
| `timer_callback` | `read` frames from `MotorManager`; resizes and fills `MotorStatus` arrays for `n = number_of_controllers()` (controller index, controlword, statusword, errorcode, position, velocity, torque); publishes. Does not populate `number_of_target_interfaces` / `target_interface_id` on the outbound message. |

### Functions

| Function | Description |
|----------|-------------|
| `main` | `rclcpp::init`, spin `MotorManagerNode`, shutdown. |

---

## `launch/motor_manager_node.launch.py`

### Variables

| Name | Description |
|------|-------------|
| `pkg_share` | `get_package_share_directory('motion_system_pkg')`. |
| `default_config` | `config/ethercat_integrated.yaml` under share. |

### Classes

None.

### Functions

| Function | Description |
|----------|-------------|
| `generate_launch_description()` | Declares launch arg `config_file` (default motor YAML); returns `LaunchDescription` with `motor_manager_node` and `parameters.config_file` from `LaunchConfiguration`. |

---

## `launch/robot_manager_node.launch.py`

### Variables

| Name | Description |
|------|-------------|
| `pkg_share` | Package share path. |
| `joy_launch` | Included `joy.launch.py` with device/deadzone/autorepeat forwarded. |

### Classes

None.

### Functions

| Function | Description |
|----------|-------------|
| `generate_launch_description()` | Declares `device_id`, `deadzone`, `autorepeat_rate`, `config_file` (default `silver_lain.yaml`), `stride_length`; includes joy launch; starts `robot_manager_node.py` with `config_file` and `stride_length` parameters. |

---

## `launch/joy.launch.py`

### Variables

None.

### Classes

None.

### Functions

| Function | Description |
|----------|-------------|
| `generate_launch_description()` | Declares `device_id`, `deadzone`, `autorepeat_rate`; runs `joy` package `joy_node` with those parameters. |

---

## `config/*.yaml` / `param/*.yaml`

Configuration data only (no Python/C++ classes). Used as `config_file` / driver parameter paths for `MotorManager` and `RobotManager` as referenced in launch files and motor YAML `param_file` entries.
