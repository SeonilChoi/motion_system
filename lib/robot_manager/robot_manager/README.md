# robot_manager

`robot_manager` is the application-facing Python package that turns joystick input and configuration into robot actions.

## Role

- Loads robot configuration from YAML.
- Selects a concrete robot class from a registry.
- Converts joystick button/axis input into `Action` objects.

## Main Class

- `RobotManager`

## Key Behavior

- Supports robot selection by config key (`little_reader`, `silver_lain`).
- Uses button edges to switch between HOME/MOVE/WALK/STOP action modes.
- In WALK mode, computes duration from stick magnitude and configured `stride_length`.
- Exposes current robot state through `get_state()`.

## Main Parameters

- `config_file`: YAML path for robot, `dt`, and defaults
- `stride_length`: optional override for walk duration scaling
