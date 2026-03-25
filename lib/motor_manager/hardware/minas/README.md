# minas

`minas` is the drive-specific layer for Panasonic MINAS style control behavior.

## Main Component

- `MinasDriver` implements the `MotorDriver` contract.

## What This Driver Handles

- Loads drive item/interface definitions from YAML.
- Applies controlword/statusword-based enable and state checks.
- Converts raw drive units into physical units used by `motor_frame_t`.

## Read This Code When

- You need to tune unit conversion behavior.
- You want to change PDO/SDO item definitions.
- Enable or command acceptance logic behaves unexpectedly.

## Inputs

- Base driver config from system YAML
- Optional `param_file` with detailed drive mapping/parameters
