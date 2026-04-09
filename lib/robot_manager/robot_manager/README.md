# robot_manager (package)

Loads robot YAML, builds one **`Robot`** per `robots[]` row, and exposes **`RobotManager`**: `set_action_frame`, `update_joint_status`, `get_robot_status`, `get_state_frame`, `dt`, `number_of_motors`, `number_of_robots`. YAML schema matches `ros2/motion_system_pkg/config/*.yaml` examples.
