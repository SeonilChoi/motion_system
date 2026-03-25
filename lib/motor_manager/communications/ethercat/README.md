# ethercat

`ethercat` is the communication backend that connects the motor runtime to IgH EtherCAT Master.

## Main Components

- `EthercatMaster`: owns master/domain lifecycle and cycle-level send/receive.
- `EthercatController`: handles per-slave process data mapping and read/write logic.

## Cycle-Level Behavior

1. Receive and process domain data.
2. Read status values into `motor_frame_t`.
3. Write command values from `motor_frame_t`.
4. Queue/send domain data and perform clock sync.

## Where To Debug

- Communication init errors: master/domain setup path.
- Wrong value mapping: controller offset/entry registration.
- In-cycle sync issues: application time and clock sync calls.

## Dependencies

- `motor_interface`
- system `libethercat`
