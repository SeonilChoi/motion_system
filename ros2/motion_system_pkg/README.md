# motion_system_pkg

`motion_system_pkg` is the ROS 2 integration package that connects joystick input, robot action logic, and motor control runtime.

## Role

- Provides executable nodes for runtime operation.
- Declares launch files for motor control and teleoperation workflows.
- Hosts default YAML configuration used by nodes.

## Main Executables

- `motor_manager_node`: bridges ROS topics to C++ motor manager runtime
- `robot_manager_node.py`: reads `joy`, updates robot actions, and publishes motor commands

## Topics

- `motor_command`: command frames sent to motor manager side
- `motor_state`: state frames published from motor manager side
- `joy`: joystick input for teleoperation

## Launch Flow

- Launch motor runtime first with motor configuration.
- Launch joystick + robot manager teleoperation path after motor runtime is available.

## Important Parameters

- `config_file`: node-specific YAML configuration path
- `stride_length`: walk timing scale for robot-side action generation
