# common_robot_interface

This package defines shared Python types for robot actions, robot state, and joystick mapping.

## Why It Exists

- Makes all robot-side modules use the same enums and dataclasses.
- Prevents type mismatch between manager, scheduler, and robot implementations.

## Core Files

- `action.py`: action kind and action payload
- `state.py`: state kind and state payload
- `joy.py`: joystick axis/button enums
- `__init__.py`: package-level exports

## When To Read This Package

Read this first when debugging action/state interpretation differences across robot modules.
