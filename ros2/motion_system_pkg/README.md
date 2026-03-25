# motion_system_pkg

ROS 2 package: nodes, launch files, and scripts bridging joystick → robot logic and ROS ↔ `MotorManager`.

---

## C++ node — `MotorManagerNode`

**Files:** `include/motion_system_pkg/motor_manager_node.hpp`, `src/motor_manager_node.cpp`

### `class MotorManagerNode : rclcpp::Node`

#### Type alias

| Name | Definition |
|------|--------------|
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

Returns QoS: depth 1, best effort, keep last — used for motor topics.

### `def _joy_qos() -> QoSProfile`

Same pattern for `joy` subscription.

### `class RobotManagerNode(Node)`

ROS node `robot_manager_node`.

#### Constructor `__init__(self) -> None`

| Parameter (declare) | Default | Stored attribute | Meaning |
|---------------------|---------|------------------|---------|
| `config_file` | `''` | `_config_file` | Path to robot YAML for `RobotManager`. |
| `stride_length` | `0.5` | `_stride_length` | Overrides walk stride if set. |

| Attribute | Meaning |
|-----------|---------|
| `_robot_manager` | `RobotManager(config_file, stride_length=...)`. |
| `_joy_sub` | Subscription to `joy`. |
| `_motor_state_sub` | Subscription to `motor_state` (callback is no-op). |
| `_motor_cmd_pub` | Publisher `motor_command` *(declared; not used in current script body)*. |
| `_timer` | Periodic timer with period `self._robot_manager.dt` → `_timer_callback`. |
| `_is_valid_joy_stick` | `False` until mode check passes. |
| `_joy_axes` | `dict[JoyAxes, float]` zero-initialized. |
| `_joy_buttons`, `_prev_joy_buttons` | `dict[JoyButton, bool]` for edge detection. |

#### Methods

| Method | Meaning |
|--------|---------|
| `_check_joy_stick_mode(mode: float)` | If `mode == 1`, sets `_is_valid_joy_stick` True and logs. |
| `_motor_state_callback(msg)` | Placeholder (`pass`). |
| `_joy_callback(msg)` | Until valid, checks `msg.axes[JoyAxes.LEFT_RIGHT_DIRECTION.value]` via `_check_joy_stick_mode` and returns. Then copies all axes/buttons into dicts. |
| `_timer_callback` | If joystick invalid, return. Logs state; calls `joy_stick_command`; deep-copies buttons to `_prev_joy_buttons`. |

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
| `config_file` | `share/.../config/silver_lain.yaml` | Robot manager YAML. |
| `stride_length` | `'0.1'` | Walk stride parameter. |

Includes `joy.launch.py` and starts `robot_manager_node.py` with `config_file` and `stride_length`.

---

## Package `motion_system_pkg/__init__.py`

Empty package marker for ament Python layout (no public API).
