# robot_control

This package contains concrete scheduler logic and robot model classes.

## What It Contains

- `scheduler/fsm_scheduler.py`: finite state transitions for HOME/MOVE/STOP style actions.
- `scheduler/gait_scheduler.py`: gait progress tracking with leg phase events.
- `robots/little_reader.py`: robot wrapper using FSM scheduler.
- `robots/silver_lain.py`: robot wrapper using gait scheduler and leg grouping.

## Recommended Reading Order

1. `scheduler/fsm_scheduler.py`
2. `scheduler/gait_scheduler.py`
3. `robots/little_reader.py`
4. `robots/silver_lain.py`

## Behavior Model

- Robot wrappers are thin adapters around scheduler instances.
- Schedulers consume `Action` and update `State`.
- Gait scheduler additionally tracks per-leg phase changes and event generation.

## Debug Entry Points

- Unexpected transition: verify transition tables in scheduler files.
- Walk phase behavior: inspect progress, offsets, and event creation in gait scheduler.
