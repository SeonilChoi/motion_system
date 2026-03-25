# common_robot_interface

`common_robot_interface` is a shared Python interface package that defines action/state records and joystick enums for robot behavior control.

## Role

- Defines common data contracts between `robot_manager`, schedulers, and robot implementations.
- Keeps high-level robot logic independent from ROS message classes.

## Main Contents

- `action.py`: `ActionKind`, `Action`
- `state.py`: `StateKind`, `State`
- `joy.py`: `JoyAxes`, `JoyButton`
- `__init__.py`: public re-exports for package-level imports

## Usage

Import from `common_robot_interface` in robot-side Python packages to exchange actions and states with consistent semantics.
