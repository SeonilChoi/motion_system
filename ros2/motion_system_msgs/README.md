# motion_system_msgs

`motion_system_msgs` defines ROS 2 message types used to exchange motor command/state frames between runtime nodes.

## Role

- Provides interface-only ROS message package for the motion system.
- Keeps message definitions centralized so producers and consumers share one schema.

## Main Messages

- `MotorFrame.msg`
- `MotorFrameMultiArray.msg`

## Usage

This package is a dependency of node packages such as `motion_system_pkg` and should be built before nodes that publish or subscribe to motor frame topics.
