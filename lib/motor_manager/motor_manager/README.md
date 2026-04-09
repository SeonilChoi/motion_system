# motor_manager (library)

**`MotorManager`**: loads motor YAML (`period`, `masters`, `drivers`), owns cyclic **`write` / `read`** on `motor_frame_t[]`, and runs **`run()`** in a realtime loop (EtherCAT + MINAS / ZeroErr drivers). Constants include `MAX_CONTROLLER_SIZE` (16). See header `include/motor_manager/motor_manager.hpp`.
