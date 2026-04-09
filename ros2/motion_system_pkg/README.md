# motion_system_pkg

ROS 2 package: launch files, the C++ `motor_manager_node`, and the Python `robot_manager_node` bridging `joy` / `RobotManager` ↔ `MotorManager` over ROS topics.

---

## C++ node — `motor_manager_node`

**Files:** `include/motion_system_pkg/motor_manager_node.hpp`, `src/motor_manager_node.cpp`

### `class MotorManagerNode : rclcpp::Node`

**Type alias:** `MotorStatus` → `motion_system_msgs::msg::MotorStatus`

**Constructor**

- Subscribes to `motor_command` (`MotorStatus`, QoS depth 1, best effort) → `motor_command_callback`.
- Advertises `motor_state` (`MotorStatus`, QoS depth 1, best effort).
- Wall timer **1 ms** → `timer_callback` (publishes `motor_state`).
- Declares parameter `config_file` (`string`, default `""`). If still empty after load, throws `std::runtime_error` (the error text may mention a launch file name that is not shipped; use `motor_manager_node.launch.py` and pass `config_file`).
- Constructs `std::unique_ptr<motor_manager::MotorManager>` and starts a **detached worker thread** running `motor_manager_->run()`.

**Destructor**

Calls `motor_manager_->request_stop()` and joins `manager_run_thread_`.

**Private methods**

| Method | Role |
|--------|------|
| `motor_command_callback` | For each index `i`, copies `MotorStatus` arrays into `motor_interface::motor_frame_t`, including `target_interface_id[i]` → fixed-size C array; calls `motor_manager_->write(frames, size)` with `size = controller_index.size()`. |
| `timer_callback` | Reads `number_of_controllers()` frames via `motor_manager_->read`, fills parallel arrays on `MotorStatus`, publishes (does not populate command-side-only fields such as `target_interface_id` on the feedback path). |

**Private members**

| Member | Role |
|--------|------|
| `motor_command_subscriber_` | Incoming commands. |
| `motor_status_publisher_` | Outgoing feedback. |
| `motor_status_timer_` | Periodic publish. |
| `config_file_` | Motor YAML path. |
| `motor_manager_` | Low-level runtime. |
| `manager_run_thread_` | Thread running `MotorManager::run()`. |

**`main`**

`rclcpp::init` → `spin(std::make_shared<MotorManagerNode>())` → `shutdown`.

---

## Python — `scripts/robot_manager_node.py`

### QoS helpers

| Function | Role |
|----------|------|
| `_joy_qos()` | Depth 1, best effort, keep last — `sensor_msgs/Joy`. |
| `_motor_status_qos()` | Same pattern — `MotorStatus` topics. |
| `_pose_qos()` | Same pattern (reserved / unused in the current script). |

### `JoyAxes` / `JoyButton`

Enums mapping `sensor_msgs/Joy` indices for axes and buttons (not part of `common_robot_interface`).

### `class RobotManagerNode(Node)`

**Parameters**

| Declared | Default | Usage |
|----------|---------|--------|
| `config_file` | `''` | Path to robot YAML for `RobotManager`. |

**Main attributes**

| Attribute | Role |
|-----------|------|
| `_robot_manager` | `RobotManager(config_file)`. |
| `_selected_robot_id` | Robot receiving button-driven actions (wrapped with `UP_DOWN_DIRECTION`). |
| `_number_of_motors` / `_number_of_robots` | From `RobotManager`. |
| `_joint_status` | Global `JointStatus` buffer (motor feedback). |
| `_robot_status` | `RobotStatus` for the selected robot. |
| `_curr_action` | `list[ActionFrame]`, one per robot; fed to `set_action_frame` each tick. |
| `_joy_sub` | `joy`. |
| `_motor_state_sub` | `motor_state` → `motor_state_callback`. |
| `_motor_command_pub` | `motor_command` → `_publish_joint_command`. |
| `_timer` | Period `RobotManager.dt` → `timer_callback`. |

**Joystick gating**

`_check_joy_stick_mode`: when `LEFT_RIGHT_DIRECTION` axis equals `1`, sets `_is_valid_joy_stick` (see `joy_callback`).

**Key methods**

| Method | Role |
|--------|------|
| `_select_robot` | Updates `_selected_robot_id` from `UP_DOWN_DIRECTION` edge. |
| `_normalize_joy_command` | Builds `ActionFrame` for `WALK` from stick axes using `stride_length` / `duration` from `RobotManager`. |
| `_publish_joint_command` | Packs `JointStatus` into `MotorStatus` (`Int8MultiArray` per motor for interface IDs) and publishes `motor_command`. |
| `motor_state_callback` | Copies parallel arrays from `MotorStatus` into `_joint_status`, calls `RobotManager.update_joint_status`. |
| `joy_callback` | Mode check until valid; then caches axes/buttons. |
| `timer_callback` | Updates `_robot_status`, handles robot selection and STOP on `State.STOPPED`, button edges → actions, optional walk command update, `set_action_frame` → `_publish_joint_command`, then saves previous joystick state. |

**`main`**

Standard `rclpy.init` → `spin` → `destroy_node` → `shutdown`.

---

## Launch files

### `launch/motor_manager_node.launch.py`

| Argument | Default | Passed as |
|----------|---------|-----------|
| `config_file` | `share/motion_system_pkg/config/ethercat_integrated.yaml` | node parameter `config_file` |

Starts executable `motor_manager_node`.

### `launch/joy.launch.py`

| Argument | Default | `joy_node` parameter |
|----------|---------|----------------------|
| `device_id` | `0` | `device_id` |
| `deadzone` | `0.05` | `deadzone` |
| `autorepeat_rate` | `20.0` | `autorepeat_rate` |

### `launch/robot_manager_node.launch.py`

| Argument | Default | Notes |
|----------|---------|--------|
| `device_id` | `0` | Joystick index for bundled `joy.launch.py`. |
| `deadzone` | `0.05` | Axis deadzone. |
| `autorepeat_rate` | `20.0` | Autorepeat (Hz). |
| `config_file` | `share/.../config/silver_lain.yaml` | Robot YAML for `RobotManager`. |
| `stride_length` | `0.1` | Declared in launch as a node parameter; **`robot_manager_node.py` does not declare or read it** — stride comes from each robot row in YAML (`RobotManager.stride_length(robot_id)`). |

Includes `joy.launch.py` and starts `robot_manager_node.py`.

---

## Other files

- `motion_system_pkg/__init__.py` is an empty ament Python package marker (no public API).
- Shell helpers mentioned in older docs are **not** present in this tree; use `utils_pkg` or ad-hoc `ros2 topic pub` for quick tests.
