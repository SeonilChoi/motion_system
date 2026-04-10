# kinematics

`ament_python` kinematic solvers for legged robots. Imported by packages under `robots/`. Depends on **`robot_interface`** (`KinematicsSolver`).

---

## `src/kinematics/silver_lain_solver.py`

### Variables (module)

| Name | Value | Meaning |
|------|-------|---------|
| `PI` | `np.pi` | Shared π constant. |
| `R30` | `π/6` | 30° in radians; leg mounting / DH offsets. |
| `R60` | `π/3` | 60° in radians; spread between leg frames. |
| `R90` | `π/2` | 90° in radians; knee / link twist conventions. |

### Classes

#### `SilverLainSolver` (`KinematicsSolver`)

Six-leg Silver Lain–style model: `link_list` is `(l0, l1, l2, l3)` per leg chain. Legs `0–2` use one mirror convention; legs `3–5` use the opposite (sign flips in DH and IK).

| Method | Description |
|--------|-------------|
| `__init__(link_list)` | Passes `link_list` to `KinematicsSolver`. |
| `_update_dh_params(index, positions)` | Builds a `6×4` DH table \([a, α, d, θ]\) for leg `index` (0–5) from that leg’s three joints `positions[index*3:index*3+3]` and fixed offsets (`R30`, `R60`, `R90`, `PI`). Left vs right half of the robot uses different θ offsets and α signs. |
| `_to_leg_frame(index, point)` | Maps a world-space foot point to the leg’s root frame using the fixed first DH row at zero joint angles (transform `T10` applied to homogeneous `point`). |
| `_inverse_kinematics(index, point)` | 3-DOF analytic IK in the leg frame: subtracts `l0` along x, solves hip yaw `th1`, planar 2-link elbow (`l2`, `l3`) with `th2`, `th3`, applies leg-index mirroring (`th3`, `th2` signs / `R90` offset differ for `index < 3` vs `≥ 3`). Returns `[th1, th2, -th3]`. |
| `_forward(positions)` | For each leg `i`, runs `_forward_kinematics` on `_update_dh_params(i, positions)`; returns `points` with shape `(6, 3)` (one foot position per leg in body frame). |
| `_inverse(points)` | For each leg, transforms `points[i]` with `_to_leg_frame`, solves `_inverse_kinematics`, writes three joints into `positions[i*3:i*3+3]`; full vector length **18**. |
| `forward_with_pose(pose, positions)` | Body-frame FK `_forward`, then applies `_get_pose_transformation_matrix(pose)` to lift all six points to world frame. Returns `(6, 3)`. |
| `inverse_with_pose(pose, points)` | World-frame targets `points` transformed to body frame via `_invert_pose_transformation_matrix`, then `_inverse`. Returns 18-vector of joint positions. |

### Functions

Module-level free functions: none (only the class and module constants above).

---

## `src/kinematics/__init__.py`

### Variables / classes / functions

Re-exports **`SilverLainSolver`** as the public API (`__all__ = ['SilverLainSolver']`).
