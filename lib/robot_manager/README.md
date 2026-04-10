# robot_manager (stack)

## Overview

ROS-agnostic Python robot stack as several `ament_python` packages: **`core/robot_interface`** (`robot_interface`) defines abstract `Robot` / `Scheduler` / planner / kinematics hooks; **`kinematics`**, **`planner`**, and **`scheduler`** implement solvers and FSM/gait logic; **`robots`** wires concrete robots; **`robot_manager`** loads YAML and ties the stack together. Shared types come from **`common_robot_interface`**.

## Repository layout

```text
lib/robot_manager/
├── README.md
├── core/
│   └── robot_interface/
│       ├── package.xml
│       ├── README.md
│       ├── setup.cfg
│       ├── setup.py
│       └── src/robot_interface/
├── kinematics/
│   ├── package.xml
│   ├── README.md
│   ├── setup.cfg
│   ├── setup.py
│   └── src/kinematics/
├── planner/
│   ├── package.xml
│   ├── README.md
│   ├── setup.cfg
│   ├── setup.py
│   └── src/planner/
├── scheduler/
│   ├── package.xml
│   ├── README.md
│   ├── setup.cfg
│   ├── setup.py
│   └── src/scheduler/
├── robots/
│   ├── package.xml
│   ├── README.md
│   ├── setup.cfg
│   ├── setup.py
│   └── src/robots/
└── robot_manager/
    ├── package.xml
    ├── README.md
    ├── setup.cfg
    ├── setup.py
    └── src/robot_manager/
```

## Package READMEs

- [core/robot_interface](core/robot_interface/README.md)
- [kinematics](kinematics/README.md)
- [planner](planner/README.md)
- [scheduler](scheduler/README.md)
- [robots](robots/README.md)
- [robot_manager](robot_manager/README.md)
