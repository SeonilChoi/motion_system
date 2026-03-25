# motor_manager library

This module is the top-level C++ orchestrator for low-level motor runtime execution.

## Main Class

- `motor_manager::MotorManager`

## What It Does

- Reads system YAML and constructs masters, controllers, and drivers.
- Runs the periodic motor cycle with deterministic timing.
- Provides thread-safe `write` and `read` interfaces for command/state frames.

## Runtime Sequence

1. Load configurations.
2. Initialize all runtime objects.
3. Start periodic loop.
4. Perform receive, state check/enable, update, and transmit each tick.
5. Stop cleanly and release resources.

## Debug Focus

- Startup issues: config parsing and object creation.
- Runtime stability: loop timing and thread scheduling.
- Command path issues: `write` update path and controller write logic.
