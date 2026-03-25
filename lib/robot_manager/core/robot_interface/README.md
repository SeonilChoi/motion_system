# robot_interface

`robot_interface` defines minimal abstract contracts for robot implementations and scheduler implementations.

## Role

- Separates high-level orchestration from robot-specific logic.
- Provides stable method signatures used by `robot_control` and `robot_manager`.

## Main Interfaces

- `Robot`: requires `get_state()` and `set_action(action)`
- `Scheduler`: owns time state, reset behavior, and abstract `tick(action)`

## Shared Dependency

- Uses `common_robot_interface` types (`Action`, `State`, enums).

## Scope

No concrete gait/FSM policy is implemented here; this package is interface-only.
