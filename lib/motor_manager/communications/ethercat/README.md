# ethercat

`ethercat` implements the EtherCAT communication backend using the IgH EtherCAT Master API.

## Role

- Creates and manages EtherCAT master/domain objects.
- Registers PDO/SDO entries from driver-provided maps.
- Moves cyclic process data between EtherCAT memory and `motor_frame_t`.

## Main Components

- `EthercatMaster`: lifecycle, domain processing, synchronization, transmit/receive
- `EthercatController`: per-slave offsets, command write, state read, enable/disable checks

## Runtime Flow

- Initialize master and slave config
- Receive/process domain data every cycle
- Read status values into motor frames
- Write command values from motor frames
- Queue/send domain data and sync clocks

## Dependencies

- `motor_interface`
- system `libethercat`
