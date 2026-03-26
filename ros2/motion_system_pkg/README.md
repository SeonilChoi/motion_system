# motion_system_pkg

ROS 2 package: nodes, launch files, and scripts bridging joystick → robot logic and ROS ↔ `MotorManager`.

---

## C++ node — `MotorManagerNode`

**Files:** `include/motion_system_pkg/motor_manager_node.hpp`, `src/motor_manager_node.cpp`

### `class MotorManagerNode : rclcpp::Node`

#### Type alias

| Name | Definition |
|------|------------|
| `MotorFrameMultiArray` | `motion_system_msgs::msg::MotorFrameMultiArray` |

#### Constructor

`explicit MotorManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())`

- Subscribes `motor_command` (`MotorFrameMultiArray`, QoS depth 1 best effort) → `motor_command_callback`.
- Publishes `motor_state` (QoS depth 10 best effort).
- Wall timer 1 ms → `motor_state_callback`.
- Declares parameter `config_file` (`string`, default `""`). If empty after load, throws `std::runtime_error`.
- Constructs `std::unique_ptr<motor_manager::MotorManager>` and starts a `std::thread` running `motor_manager_->run()`.

#### Destructor

Calls `motor_manager_->request_stop()` and joins `manager_run_thread_`.

#### Private methods

| Method | Meaning |
|--------|---------|
| `motor_command_callback(msg)` | Converts `msg->data[]` into `motor_interface::motor_frame_t[]` (clamps counts and array copies), calls `motor_manager_->write(frames, size)`. |
| `motor_state_callback` | Reads `number_of_controllers()` frames via `motor_manager_->read`, publishes `MotorFrameMultiArray` with resized `data`. |

#### Private members

| Member | Type | Meaning |
|--------|------|---------|
| `motor_command_subscriber_` | `rclcpp::Subscription<MotorFrameMultiArray>::SharedPtr` | Incoming commands. |
| `motor_state_publisher_` | `rclcpp::Publisher<MotorFrameMultiArray>::SharedPtr` | Outgoing feedback. |
| `motor_state_timer_` | `rclcpp::TimerBase::SharedPtr` | Periodic publish. |
| `config_file_` | `std::string` | Path to motor YAML. |
| `motor_manager_` | `std::unique_ptr<motor_manager::MotorManager>` | Low-level runtime. |
| `manager_run_thread_` | `std::thread` | Runs cyclic `MotorManager::run()`. |

#### `main(int argc, char* argv[])`

`rclcpp::init` → `spin(make_shared<MotorManagerNode>())` → `shutdown`.

---

## Python script — `scripts/robot_manager_node.py`

### `def _motor_qos() -> QoSProfile`

QoS: depth 1, best effort, keep last — motor topics.

### `def _joy_qos() -> QoSProfile`

Same pattern for `joy` subscription.

### Module-local enums

The script defines **`JoyAxes`** and **`JoyButton`** enums for `sensor_msgs/Joy` indices (not part of `common_robot_interface`). Types from `common_robot_interface`: **`Action`** (enum), **`ActionFrame`**, **`State`** (enum).

### `class RobotManagerNode(Node)`

ROS node `robot_manager_node`.

#### Constructor `__init__(self) -> None`

| Parameter (declare) | Default | Stored attribute | Meaning |
|---------------------|---------|------------------|---------|
| `config_file` | `''` | `_config_file` | Path to robot YAML for `RobotManager`. |

| Attribute | Meaning |
|-----------|---------|
| `_robot_manager` | `RobotManager(_config_file)`. |
| `_selected_robot_id` | Index of the robot receiving button-driven `ActionFrame` updates (0 … `number_of_robots - 1`). |
| `_number_of_robots` | `self._robot_manager.number_of_robots`. |
| `_number_of_motors` | `self._robot_manager.number_of_motors`. |
| `_joint_states` | Global `JointState` buffers sized by `_number_of_motors`. |
| `_curr_action` | `list[ActionFrame]`, one per robot; passed to `set_action` each timer tick. |
| `_joy_sub` | Subscription to `joy`. |
| `_motor_state_sub` | Subscription to `motor_state` (`motor_state_callback` currently no-op). |
| `_motor_cmd_pub` | Publisher `motor_command` *(declared; unused in current script body)*. |
| `_timer` | Period `self._robot_manager.dt` → `timer_callback`. |
| `_is_valid_joy_stick` | `False` until mode check passes. |
| `_joy_axes`, `_prev_joy_axes` | `dict[JoyAxes, float]`. |
| `_joy_buttons`, `_prev_joy_buttons` | `dict[JoyButton, bool]` for edge detection. |
| `_joy_button_action` | `JoyButton` → **`Action`** enum: A→HOME, B→MOVE, X→WALK, Y→STOP. |

#### Methods

| Method | Meaning |
|--------|---------|
| `_check_joy_stick_mode(mode: float)` | If `mode == 1`, sets `_is_valid_joy_stick` and logs. |
| `_select_robot(up_down: float)` | Changes `_selected_robot_id` from `UP_DOWN_DIRECTION` axis edge (wraps modulo `number_of_robots`). |
| `motor_state_callback(msg)` | Copies `msg.data[i]` into `_joint_states` arrays and forwards via `RobotManager.set_joint_states`. |
| `joy_callback(msg)` | Until valid, uses `LEFT_RIGHT_DIRECTION` axis for `_check_joy_stick_mode` then returns. Then copies all axes/buttons into dicts. |
| `timer_callback` | If joystick invalid, return. On `UP_DOWN_DIRECTION` edge, `_select_robot`. Logs selected `StateFrame` and each robot's `get_robot_states(robot_id).pose.points`. If selected robot `state == State.STOPPED`, sets selected action to STOP. Handles button rising-edge action latch, computes walk duration/goal, dispatches `set_action(_curr_action)`, then snapshots previous joystick states. |

### `def main() -> None`

`rclpy.init()` → `spin(RobotManagerNode())` → destroy node → `shutdown`.

---

## Launch files

### `launch/motor_manager_node.launch.py`

| Launch argument | Default | Passed to node as |
|-----------------|---------|-------------------|
| `config_file` | `share/motion_system_pkg/config/ethercat.yaml` | parameter `config_file` |

`generate_launch_description()` returns `LaunchDescription` with one `Node` executable `motor_manager_node`.

### `launch/joy.launch.py`

| Launch argument | Default | Node parameter |
|-----------------|---------|----------------|
| `device_id` | `'0'` | `device_id` |
| `deadzone` | `'0.05'` | `deadzone` |
| `autorepeat_rate` | `'20.0'` | `autorepeat_rate` |

Starts `joy` package `joy_node`.

### `launch/robot_manager_node.launch.py`

| Launch argument | Default | Meaning |
|-----------------|---------|---------|
| `device_id` | `'0'` | Joystick index for included `joy.launch.py`. |
| `deadzone` | `'0.05'` | Axis deadzone. |
| `autorepeat_rate` | `'20.0'` | Autorepeat Hz. |
| `config_file` | `share/.../config/silver_lain.yaml` | Robot YAML (`robots`, `dt`, per-row `stride_length`, …). |
| `stride_length` | `'0.1'` | Passed as a **node parameter** in launch; the current `robot_manager_node.py` **does not declare or read** it—effective stride comes from each robot entry in YAML (`Robot.stride_length` / `RobotManager.stride_length(robot_id)`). |

Includes `joy.launch.py` and starts `robot_manager_node.py` with `parameters` for `config_file` and `stride_length` (the latter reserved for future use unless wired in the node).

## Helper scripts

- `scripts/pub_motor_state_once.sh`: one-shot `/motor_state` publisher for quick tests.
- `publish_motor_state_once.sh` (workspace root): continuous `/motor_state` publisher (~1 kHz) with position ramp `-pi/2 -> +pi/2`.

---

## Package `motion_system_pkg/__init__.py`

Empty package marker for ament Python layout (no public API).
