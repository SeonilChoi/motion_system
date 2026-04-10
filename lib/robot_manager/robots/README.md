# robots

`set_action_frame` only — concrete **`Robot`** implementations in `src/robots/`.

---

## `SilverLain.set_action_frame`

```mermaid
flowchart TB
  AF[ActionFrame] --> A["(n×3) zeros; column 0 ← _curr_joint_status.position"]
  A --> B{HOME?}
  B -->|yes| B2[set_first_step True]
  B -->|no| C
  B2 --> C[tick(frame)]
  C --> D{WALK ∧ event ∧\ngait _events?}
  D -->|yes| D2[FK → planner set_initial_state ×6]
  D -->|no| E
  D2 --> E{WALK ∧\n_events set?}
  E -->|yes| E2["compute_next_target + inverse_with_pose\n→ column 0"]
  E -->|no| F
  E2 --> F{WALK ∧ goal\nnot all 0?}
  F -->|yes| F2[scheduler.step]
  F -->|no| G
  F2 --> G[return (n_motors, 3)]
```

---

## `LittleReader.set_action_frame`

```mermaid
flowchart TB
  AF[ActionFrame] --> T[tick(frame)]
  T --> C{_curr_joint_status\nis None?}
  C -->|yes| Z[return (zeros, zeros, zeros)\neach length n_motors]
  C -->|no| E[return (p copy, v copy, t copy)\nzeros if field is None]
```

Returns a **3-tuple** of 1D arrays, not `(n_motors, 3)` — **`RobotManager.set_action_frame`** currently assumes a single matrix per robot.
