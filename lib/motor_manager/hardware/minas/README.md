# minas

`minas` provides a drive-specific implementation for Panasonic MINAS style servo control on top of the shared motor interface.

## Role

- Loads drive parameter definitions and interface maps from YAML.
- Implements controlword/statusword state transitions for enable/disable and command acceptance.
- Converts between raw drive units and physical units used in `motor_frame_t`.

## Main Component

- `MinasDriver`: implementation of `MotorDriver`

## Core Behavior

- Supplies PDO/SDO items used by communication layers
- Handles CiA402-like state checks through controlword logic
- Performs position/velocity/torque conversion using configured scale factors

## Inputs

- Driver configuration from system YAML
- Optional driver parameter YAML file referenced by `param_file`
