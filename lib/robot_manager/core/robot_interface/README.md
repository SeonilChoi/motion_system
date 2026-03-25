# robot_interface

`robot_interface` is the abstract foundation for robot-side Python code.

## Why It Matters

- Defines contracts that all robot implementations must follow.
- Lets `robot_manager` call robot code without knowing robot-specific details.

## Interfaces

- `Robot`: base class with `get_state()` and `set_action(action)`.
- `Scheduler`: base class with time/state ownership and `tick(action)`.

## What Is Not Here

- No concrete transitions.
- No gait policy.
- No robot-specific implementation.

Concrete behavior lives in `packages/robot_control`.
