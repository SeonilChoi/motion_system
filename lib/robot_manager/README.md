# robot_manager (Python stack)

This tree implements high-level robot behavior, from action requests to scheduler-driven state transitions.

## Start Here

If you are trying to understand robot behavior quickly:

1. Read `robot_manager/README.md`.
2. Read `packages/robot_control/README.md`.
3. Read `core/robot_interface/README.md`.

## Architecture

- `robot_manager`: top-level orchestrator that maps joystick input to `Action`.
- `robot_control`: concrete schedulers and robot implementations.
- `robot_interface`: abstract base contracts for robots and schedulers.

## Core Flow

1. `RobotManager` receives joystick/button state.
2. It decides the current `ActionKind` and builds an `Action`.
3. Selected robot class forwards action to scheduler.
4. Scheduler updates and exposes `State`.

## Where To Debug

- Wrong state transitions: check `packages/robot_control/scheduler/*`.
- Wrong robot class selected: check robot key handling in `robot_manager`.
- Walk timing issues: check duration computation from `stride_length`.
