# robot_manager (library tree)

This tree contains the Python-side robot behavior stack for action scheduling and joystick-driven command generation.

## Role

- Defines abstract robot/scheduler interfaces.
- Implements concrete schedulers and robot models.
- Provides `RobotManager`, which maps joystick input to robot actions based on configuration.

## Submodules

- `core/robot_interface`: abstract `Robot` and `Scheduler`
- `packages/robot_control`: concrete schedulers and robot classes
- `robot_manager`: integration layer that selects and drives robot instances

## Data Flow

- Joystick axes/buttons are converted into `Action`.
- A selected robot implementation updates state through its scheduler.
- Current `State` is exposed for external node code.
