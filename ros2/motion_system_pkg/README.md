# motion_system_pkg

ROS 2 bridge package (`ament_cmake` + Python scripts): **`motor_manager_node`** (C++) runs `MotorManager` and talks **`MotorStatus`** on `motor_command` / `motor_state`; **`robot_manager_node.py`** ties `joy`, `RobotManager`, and those topics.

**Launches:** `motor_manager_node.launch.py` (default motor YAML: `config/ethercat_integrated.yaml`), `robot_manager_node.launch.py` (includes `joy.launch.py`, default robot YAML: `config/silver_lain.yaml`), `joy.launch.py`.

Installs `config/`, `param/`, and `launch/` under `share/motion_system_pkg`.
