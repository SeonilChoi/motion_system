# motion_system (ROS 2 workspace `src`)

Colcon packages under this tree:

| Path | Package |
|------|---------|
| `common/common_motor_interface` | Shared `motor_frame_t` / `MAX_INTERFACE_SIZE` |
| `lib/motor_manager` | EtherCAT + Minas `MotorManager` library (ament) |
| `ros2/motion_system_msgs` | Message definitions |
| `ros2/motion_system_pkg` | `motor_manager_node`, launch, example config |

Legacy copies at repo root (`motion_system_msgs/`, `motion_system_pkg/`) have `COLCON_IGNORE` so colcon does not see duplicate package names; remove those folders or the ignore files if you want only one layout.

Build (from `lab_ws`):

```bash
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch motion_system_pkg motion_system.launch.py
```

Requires IgH EtherCAT Master (`libethercat`) and `yaml-cpp` on the system.
