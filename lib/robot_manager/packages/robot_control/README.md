# robot_control

`robot_control` contains concrete robot behavior implementations built on top of `robot_interface`.

## Role

- Implements finite-state and gait-oriented schedulers.
- Provides concrete robot classes that delegate action handling to schedulers.

## Main Components

- `scheduler/fsm_scheduler.py`: state transition scheduler for HOME/MOVE/STOP flows
- `scheduler/gait_scheduler.py`: gait scheduler with progress tracking and per-leg events
- `robots/little_reader.py`: robot using `FsmScheduler`
- `robots/silver_lain.py`: robot using `GaitScheduler` with predefined leg groups

## Behavior Highlights

- Scheduler transitions are driven by `ActionKind`.
- Gait scheduler tracks progress and emits lift-off/touch-down events.
- Robot classes expose state and accept actions through a uniform API.
