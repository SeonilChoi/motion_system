# motion_system (ROS 2 workspace `src`)

Colcon packages under this tree:

| Path | Package |
|------|---------|
| `common/common_motor_interface` | Shared `motor_frame_t` / `MAX_INTERFACE_SIZE` |
| `lib/motor_manager` | EtherCAT + Minas `MotorManager` library (ament) |
| `ros2/motion_system_msgs` | Message definitions |
| `ros2/motion_system_pkg` | `motor_manager_node`, `robot_manager_node.py`, launches, config |

Legacy copies at repo root (`motion_system_msgs/`, `motion_system_pkg/`) have `COLCON_IGNORE` so colcon does not see duplicate package names; remove those folders or the ignore files if you want only one layout.

Build (from `lab_ws`):

```bash
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch motion_system_pkg motion_system.launch.py
```

Requires IgH EtherCAT Master (`libethercat`) and `yaml-cpp` on the system.

## `motion_system_pkg`

### Nodes

| Executable | Role |
|------------|------|
| `motor_manager_node` | Subscribes to `motor_command`, publishes `motor_state` at 1 kHz; drives hardware from YAML `config_file`. |
| `robot_manager_node.py` | Subscribes to `joy` and `motor_state`, publishes `motor_command` (stick axes → velocities; see script for mapping). |

### Topics

| Name | Type | Direction (motor manager) |
|------|------|-------------------------|
| `motor_command` | `motion_system_msgs/msg/MotorFrameMultiArray` | In |
| `motor_state` | `motion_system_msgs/msg/MotorFrameMultiArray` | Out |
| `joy` | `sensor_msgs/msg/Joy` | Used by `robot_manager_node.py` only |

### Launches

| File | Purpose |
|------|---------|
| `motion_system.launch.py` | `motor_manager_node` with `config_file` (default: share-installed `config/example.yaml`). |
| `joy.launch.py` | `joy_node` only (`device_id`, `deadzone`, `autorepeat_rate` launch args). |
| `teleop.launch.py` | Includes `joy.launch.py` plus `robot_manager_node.py`. Run `motion_system.launch.py` in another terminal first so `motor_state` exists. |

Teleop depends on the ROS `joy` package (`sudo apt install ros-<distro>-joy` if missing).

### `robot_manager_node.py` parameters

- `config_file` (string): YAML for robot type, `dt`, `stride_length`, etc.
- `stride_length` (float): nominal stride length for walk timing.
