# utils_pkg

ROS 2 Python utilities (`ament_python`). Executable nodes are registered in `setup.py` under `console_scripts`.

---

## `utils_pkg/sine_motor_command_publisher.py`

### Node: `sine_motor_command_publisher`

| Item | Description |
|------|-------------|
| **Class** | `SineMotorCommandPublisher` (`rclpy.node.Node`) |
| **Purpose** | Publishes synthetic `motion_system_msgs/MotorStatus` on **`motor_command`** to exercise the motor stack (e.g. with `motor_manager_node` subscribed). |
| **Timer** | `0.01` s (100 Hz): builds one message per tick and increments an internal time `t`. |
| **Motors** | Fixed count **`N_MOTORS = 5`**. |
| **Message fill** | For each axis `i`: `number_of_target_interfaces = 2`; `target_interface_id[i] = Int8MultiArray([0, 1])`; `controller_index = 0..n-1`; `controlword` = `CW_NEW_SET_POINT_MINAS` (0x003F) for `i < 2`, else `CW_NEW_SET_POINT_ZEROERR` (0x103F); `statusword` / `errorcode` zero; `velocity` / `torque` zero; **`position[i] = sin(2π·0.1·t)`** (same phase for all axes in the current code). |
| **QoS** | Publisher depth 1, best effort, keep last (`_motor_command_qos`). |
| **Run** | `ros2 run utils_pkg sine_motor_command_publisher` (entry point `utils_pkg.sine_motor_command_publisher:main`). |

### Other symbols in this file

| Name | Role |
|------|------|
| `N_MOTORS` | Number of parallel controllers in each published message. |
| `CW_NEW_SET_POINT_MINAS` / `CW_NEW_SET_POINT_ZEROERR` | Controlword constants for MINAS vs ZeroErr-style set-point. |
| `_motor_command_qos()` | Builds the publisher QoS profile. |
| `main()` | `rclpy.init`, spin node, shutdown on interrupt. |

---

## `utils_pkg/__init__.py`

Package marker only; **no ROS nodes**.
