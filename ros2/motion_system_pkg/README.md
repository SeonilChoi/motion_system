# motion_system_pkg

`motion_system_pkg` is the runtime entry point of this workspace. If you want to run or debug the system, start here.

## What This Package Does

- Runs `motor_manager_node` for motor command execution and state publishing.
- Runs `robot_manager_node.py` for joystick-to-action processing.
- Provides launch files that connect both flows.

## Runtime Model

- Input: `joy` (from `joy_node`)
- Command channel: `motor_command`
- Feedback channel: `motor_state`

The typical direction is `joy -> robot_manager_node.py -> motor_command -> motor_manager_node -> motor_state`.

## Read Code In This Order

1. `scripts/robot_manager_node.py`
2. `src/motor_manager_node.cpp`
3. `launch/robot_manager_node.launch.py`
4. `launch/motion_system.launch.py`
5. `config/*.yaml`

## Launch Sequence

1. Start motor runtime with proper hardware config.
2. Start robot manager and joystick.

## Important Parameters

- `config_file`: selects YAML configuration file.
- `stride_length`: affects walk duration in robot action generation.

## Debug Tips

- If no movement occurs, check whether `joy` is arriving and joystick mode is accepted.
- If commands publish but hardware does not respond, verify `motor_manager_node` config and EtherCAT availability.
