# robot_manager package

This package is the integration point between input intent and concrete robot behavior.

## Main Responsibility

- Load robot configuration from YAML.
- Instantiate the proper robot implementation.
- Convert joystick state into `Action` and forward it to the robot.

## Key Class

- `RobotManager` in `src/robot_manager/robot_manager.py`

## How To Read The Code

1. `_loadConfigurations`: config loading and defaults.
2. `_robot_class_from_config`: robot selection logic.
3. `joy_stick_command`: button-edge action switching and walk command generation.
4. `get_state`: state passthrough from robot implementation.

## Action Mapping Summary

- Buttons switch action mode: HOME, MOVE, WALK, STOP.
- WALK uses left/right stick axes and computes duration from `stride_length`.
- Non-WALK actions are forwarded immediately as state-machine actions.

## Configuration Keys

- `robot`: robot type key such as `little_reader` or `silver_lain`
- `dt`: scheduler tick period
- `stride_length`: nominal stride distance used for walk duration
