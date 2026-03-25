# motion_system_msgs

This package defines the ROS message schema for motor command and feedback transport.

## Why It Exists

- Keeps all motor message definitions in one place.
- Prevents schema drift between publishers and subscribers.

## Message Files

- `msg/MotorFrame.msg`: single motor frame payload.
- `msg/MotorFrameMultiArray.msg`: grouped frames for multi-axis systems.

## How To Use

- Add this package as a dependency in any ROS node package that publishes or subscribes to motor topics.
- Build this package before runtime packages that import its message types.

## When To Read This Package

Read this package when topic serialization, field meaning, or message compatibility is in question.
