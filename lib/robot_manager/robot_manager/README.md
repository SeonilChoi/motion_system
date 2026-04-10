# robot_manager (package)

**`RobotManager`** connects an **external caller** to **`Robot`** instances (`src/robot_manager/robot_manager.py`).

```mermaid
flowchart TB
  EXT[External caller]

  RM[RobotManager]

  R[Robots\nSilverLain / LittleReader / …]

  EXT -->|"① set_action_frame\nList[ActionFrame]"| RM
  RM -->|"calls each\nrobots[i].set_action_frame"| R
  R -->|"merged p,v,t"| RM
  RM -->|"① returns\nJointStatus"| EXT

  EXT -->|"② update_joint_status\nJointStatus"| RM
  RM -->|"slices by\ncontroller_indexes"| R

  EXT -->|"③ get_state_frame(robot_id)"| RM
  RM --> R
  R --> RM
  RM -->|"StateFrame"| EXT

  EXT -->|"④ get_robot_status(robot_id)"| RM
  RM --> R
  R --> RM
  RM -->|"RobotStatus"| EXT
```

① **Commands:** `ActionFrame` list → `RobotManager` → each `Robot` → merged **`JointStatus`**.  
② **Feedback:** global **`JointStatus`** → sliced → each **`Robot`**.  
③ **`StateFrame`** / ④ **`RobotStatus`:** `robot_id` query via `RobotManager` → chosen **`Robot`**.
