#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

ROOT_DIR = Path(__file__).resolve().parent
PI = np.pi
R30 = PI / 6
R60 = PI / 3
R90 = PI / 2
LINK_LIST = [0.32, 0.0, 0.615, 1.25]


class SilverLainSolver:
    def __init__(self, link_list: list[float]) -> None:
        self._link_list = link_list

    @staticmethod
    def _get_pose_transformation_matrix(pose: np.ndarray) -> np.ndarray:
        x, y, z, roll, pitch, yaw = pose

        rz = np.array(
            [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]],
            dtype=np.float64,
        )
        ry = np.array(
            [[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]],
            dtype=np.float64,
        )
        rx = np.array(
            [[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]],
            dtype=np.float64,
        )

        rotation = rz @ ry @ rx
        position = np.array([x, y, z], dtype=np.float64).reshape(3, 1)
        return np.r_[np.c_[rotation, position], [[0, 0, 0, 1]]]

    @staticmethod
    def _get_transformation_matrix(param: np.ndarray) -> np.ndarray:
        a, alpha, d, theta = param
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array(
            [[ct, -st, 0, a], [st * ca, ct * ca, -sa, -sa * d], [st * sa, ct * sa, ca, ca * d], [0, 0, 0, 1]],
            dtype=np.float64,
        )

    @staticmethod
    def _forward_kinematics(dh_params: np.ndarray) -> np.ndarray:
        transform = np.eye(4, dtype=np.float64)
        for param in dh_params:
            transform = transform @ SilverLainSolver._get_transformation_matrix(param)
        return transform[:3, -1].reshape(3)

    def _update_dh_params(self, index: int, positions: np.ndarray) -> np.ndarray:
        l0, l1, l2, l3 = self._link_list
        q1, q2, q3 = positions[index * 3 : index * 3 + 3]

        if index < 3:
            return np.array(
                [
                    [0, 0, 0, -R30 - R60 * index],
                    [l0, 0, 0, q1],
                    [0, 0, l1, 0],
                    [0, R90, 0, R90 + q2],
                    [l2, PI, 0, q3],
                    [l3, 0, 0, 0],
                ],
                dtype=np.float64,
            )

        return np.array(
            [
                [0, 0, 0, R30 + R60 * (5 - index)],
                [l0, 0, 0, q1],
                [0, 0, l1, 0],
                [0, -R90, 0, -R90 + q2],
                [l2, -PI, 0, q3],
                [l3, 0, 0, 0],
            ],
            dtype=np.float64,
        )

    def forward_with_pose(self, pose: np.ndarray, positions: np.ndarray) -> np.ndarray:
        points_body = np.zeros((6, 3), dtype=np.float64)
        for i in range(6):
            points_body[i] = self._forward_kinematics(self._update_dh_params(i, positions))

        transform = self._get_pose_transformation_matrix(pose)
        points_h = np.c_[points_body, np.ones((6, 1))]
        points_world = (transform @ points_h.T).T
        return points_world[:, :3]


def _default_data_dir() -> Path:
    return ROOT_DIR / "ros2" / "test_pkg" / "data"


def _stem_sort_key(stem: str) -> tuple:
    """Order: numeric tags 1,2,… then non-numeric lexicographically."""
    if stem.isdigit():
        return (0, int(stem), "")
    return (1, stem, "")


def _paired_action_path(data_dir: Path, obs_path: Path) -> Path:
    stem = obs_path.name[: -len("_obs.npy")]
    return data_dir / f"{stem}_action.npy"


def _list_ordered_dataset_pairs(data_dir: Path) -> list[tuple[Path, Path, str]]:
    """All valid (obs, action, stem) pairs: plain obs.npy first, then *_obs.npy by stem order."""
    data_dir = data_dir.expanduser().resolve()
    if not data_dir.is_dir():
        raise FileNotFoundError(f"Data directory does not exist: {data_dir}")

    out: list[tuple[Path, Path, str]] = []
    plain_obs = data_dir / "obs.npy"
    plain_act = data_dir / "action.npy"
    if plain_obs.is_file() and plain_act.is_file():
        out.append((plain_obs, plain_act, ""))

    tagged_obs = [p for p in data_dir.glob("*_obs.npy") if p.name != "obs.npy"]
    tagged_obs.sort(key=lambda p: _stem_sort_key(p.name[: -len("_obs.npy")]))

    for obs_p in tagged_obs:
        stem = obs_p.name[: -len("_obs.npy")]
        act_p = _paired_action_path(data_dir, obs_p)
        if act_p.is_file():
            out.append((obs_p, act_p, stem))

    return out


def _resolve_obs_action_paths(data_dir: Path, tag: str | None) -> tuple[Path, Path, str | None]:
    """Return (obs_path, action_path, stem_prefix). stem_prefix is '' for obs.npy or '1' for 1_obs.npy."""
    data_dir = data_dir.expanduser().resolve()
    if not data_dir.is_dir():
        raise FileNotFoundError(f"Data directory does not exist: {data_dir}")

    if tag is not None:
        prefix = tag.strip()
        obs_p = data_dir / f"{prefix}_obs.npy"
        act_p = data_dir / f"{prefix}_action.npy"
        if obs_p.is_file() and act_p.is_file():
            return obs_p, act_p, prefix
        raise FileNotFoundError(
            f"With --tag {prefix!r}, expected {obs_p.name} and {act_p.name} under {data_dir}"
        )

    plain_obs = data_dir / "obs.npy"
    plain_act = data_dir / "action.npy"
    if plain_obs.is_file() and plain_act.is_file():
        return plain_obs, plain_act, ""

    candidates = sorted(data_dir.glob("*_obs.npy"))
    if not candidates:
        raise FileNotFoundError(
            f"No obs.npy / action.npy or *_obs.npy in {data_dir}. "
            f"Collect data or pass --tag (e.g. --tag 1 for 1_obs.npy)."
        )

    valid = [(o, _paired_action_path(data_dir, o)) for o in candidates if _paired_action_path(data_dir, o).is_file()]
    if not valid:
        raise FileNotFoundError(f"No matching *_action.npy for files in {data_dir}")

    obs_path, action_path = max(valid, key=lambda pair: pair[0].stat().st_mtime)
    stem = obs_path.name[: -len("_obs.npy")]
    return obs_path, action_path, stem


def _load_one_pair(
    obs_path: Path, action_path: Path, stem: str
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    obs = np.load(obs_path)
    action = np.load(action_path)
    if obs.ndim != 2 or action.ndim != 2:
        raise ValueError(f"{obs_path.name} and {action_path.name} must be 2D arrays")
    if obs.shape[0] != action.shape[0]:
        raise ValueError("obs and action must have the same number of samples")

    cond_name = f"{stem}_condition.npy" if stem else "condition.npy"
    cond_path = obs_path.parent / cond_name
    condition: np.ndarray | None = None
    if cond_path.is_file():
        c = np.load(cond_path)
        if c.ndim == 2 and c.shape[0] == obs.shape[0]:
            condition = c
        else:
            print(
                f"Skipping {cond_path.name}: expected 2D array with {obs.shape[0]} rows, got {getattr(c, 'shape', None)}"
            )
    return obs, action, condition


def _load_dataset(
    data_dir: Path | None = None,
    tag: str | None = None,
    *,
    latest_only: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None, list[tuple[int, int, str]]]:
    """
    Load obs/action (+ optional condition). Returns segment list (start, end_exclusive, label)
    for HUD when multiple files are concatenated.
    """
    base = data_dir if data_dir is not None else _default_data_dir()

    if tag is not None:
        obs_path, action_path, used_tag = _resolve_obs_action_paths(base, tag)
        obs, action, condition = _load_one_pair(obs_path, action_path, used_tag)
        label = f"{used_tag}_" if used_tag else ""
        print(f"Loaded {label}obs / {label}action from {obs_path.parent} ({obs.shape[0]} samples)")
        if condition is not None:
            print(f"Loaded {label}condition {condition.shape}")
        seg_label = used_tag if used_tag else "obs"
        return obs, action, condition, [(0, obs.shape[0], seg_label)]

    pairs = _list_ordered_dataset_pairs(base)
    if not pairs:
        raise FileNotFoundError(
            f"No obs.npy / action.npy or paired *_obs.npy in {base}. "
            f"Collect data or pass --tag (e.g. --tag 1 for 1_obs.npy)."
        )

    if latest_only and len(pairs) > 1:
        obs_path, action_path, stem = max(pairs, key=lambda x: x[0].stat().st_mtime)
        obs, action, condition = _load_one_pair(obs_path, action_path, stem)
        label = f"{stem}_" if stem else ""
        print(f"Loaded (--latest-only) {label}obs / {label}action ({obs.shape[0]} samples)")
        if condition is not None:
            print(f"Loaded {label}condition {condition.shape}")
        return obs, action, condition, [(0, obs.shape[0], stem if stem else "obs")]

    obs_blocks: list[np.ndarray] = []
    action_blocks: list[np.ndarray] = []
    segments: list[tuple[int, int, str]] = []
    cond_blocks: list[tuple[int, np.ndarray | None]] = []

    pos = 0
    for obs_path, action_path, stem in pairs:
        o, a, c = _load_one_pair(obs_path, action_path, stem)
        if obs_blocks:
            if o.shape[1] != obs_blocks[0].shape[1] or a.shape[1] != action_blocks[0].shape[1]:
                raise ValueError(
                    f"Feature mismatch: {obs_path.name} obs{o.shape} vs first {obs_blocks[0].shape}; "
                    "all runs must share the same obs/action widths."
                )
        obs_blocks.append(o)
        action_blocks.append(a)
        n = o.shape[0]
        seg_label = stem if stem else "obs"
        segments.append((pos, pos + n, seg_label))
        pos += n
        cond_blocks.append((n, c))

    obs = np.vstack(obs_blocks)
    action = np.vstack(action_blocks)
    print(f"Loaded {len(pairs)} run(s) from {base} → {obs.shape[0]} samples total (order: {[s[2] for s in segments]})")

    condition = None
    cond_widths = {c.shape[1] for _, c in cond_blocks if c is not None}
    if len(cond_widths) == 1:
        cd = cond_widths.pop()
        parts: list[np.ndarray] = []
        for n, c in cond_blocks:
            if c is not None and c.shape[1] == cd:
                parts.append(c)
            else:
                parts.append(np.full((n, cd), np.nan, dtype=np.float64))
        condition = np.vstack(parts)
        print(f"Merged condition {condition.shape} (NaN rows where a run has no condition file)")
    elif len(cond_widths) > 1:
        print("Condition column counts differ between runs; skipping merged condition.")

    return obs, action, condition, segments


def _segment_info(
    segments: list[tuple[int, int, str]], frame_idx: int
) -> tuple[str, int, int]:
    """Return (label, local_index, segment_length)."""
    for s, e, label in segments:
        if s <= frame_idx < e:
            return label, frame_idx - s, e - s
    return "?", frame_idx, 0


def _split_obs(obs: np.ndarray, action_dim: int) -> dict[str, np.ndarray | None]:
    """Parse obs layout: legacy pose(6)+prev_motor(M), or full pose+pose_vel+prev_motor+prev_motor_vel."""
    n = obs.shape[1]
    legacy = 6 + action_dim
    with_vel = 12 + 2 * action_dim
    if n == legacy:
        return {
            "pose": obs[:, :6],
            "pose_vel": None,
            "prev_motor": obs[:, 6 : 6 + action_dim],
            "prev_motor_vel": None,
        }
    if n >= with_vel:
        return {
            "pose": obs[:, :6],
            "pose_vel": obs[:, 6:12],
            "prev_motor": obs[:, 12 : 12 + action_dim],
            "prev_motor_vel": obs[:, 12 + action_dim : 12 + 2 * action_dim],
        }
    raise ValueError(
        f"obs feature dim {n} does not match action_dim={action_dim}: "
        f"expected legacy {legacy} or new >= {with_vel}"
    )


def _obs_position_slices(obs: np.ndarray, action_dim: int) -> tuple[np.ndarray, np.ndarray]:
    """Extract pose (6) and prev motor positions (M) only; ignore velocity blocks in obs."""
    s = _split_obs(obs, action_dim)
    return s["pose"], s["prev_motor"]


def _plot_base_trajectory_2d(pose: np.ndarray, segments: list[tuple[int, int, str]]) -> None:
    """Plan-view path: body X on vertical (bottom→up), body Y on horizontal (right→left positive)."""
    bx = pose[:, 0]
    by = pose[:, 1]
    n = pose.shape[0]
    frames = np.arange(n)

    # Matplotlib: horizontal = by, vertical = bx → X reads upward, Y grows toward the left.
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(by, bx, "k-", linewidth=0.5, alpha=0.35, zorder=1)
    sc = ax.scatter(by, bx, c=frames, cmap="viridis", s=14, alpha=0.88, zorder=2, edgecolors="none")
    ax.scatter([by[0]], [bx[0]], c="lime", s=130, zorder=5, edgecolors="k", linewidths=0.8, label="start")
    ax.scatter([by[-1]], [bx[-1]], c="red", s=130, zorder=5, edgecolors="k", linewidths=0.8, label="end")

    boundary_labeled = False
    for s, _e, lab in segments:
        if s == 0:
            continue
        lbl = "run boundary" if not boundary_labeled else None
        boundary_labeled = True
        ax.scatter(
            [by[s]],
            [bx[s]],
            c="orange",
            s=100,
            marker="^",
            zorder=4,
            edgecolors="k",
            linewidths=0.6,
            label=lbl,
        )
        ax.annotate(
            str(lab),
            (float(by[s]), float(bx[s])),
            textcoords="offset points",
            xytext=(6, 6),
            fontsize=8,
            alpha=0.9,
        )

    # Square limits in (horizontal=by, vertical=bx) space
    ym, yM = float(np.min(by)), float(np.max(by))
    xm, xM = float(np.min(bx)), float(np.max(bx))
    ch = 0.5 * (ym + yM)
    cv = 0.5 * (xm + xM)
    half_h = max(ch - ym, yM - ch)
    half_v = max(cv - xm, xM - cv)
    half = max(max(half_h, half_v) * 1.08, 0.5)

    ax.set_title("Base trajectory (X ↑ vertical, Y ← horizontal)")
    ax.set_xlabel("Y (body)")
    ax.set_ylabel("X (body)")
    ax.set_xlim(ch - half, ch + half)
    ax.set_ylim(cv - half, cv + half)
    ax.invert_xaxis()
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04).set_label("frame index")
    fig.tight_layout()


def _plot_data_dashboard(
    obs: np.ndarray,
    action: np.ndarray,
    condition: np.ndarray | None,
) -> None:
    """Time-series and heatmap views of obs / action / optional condition."""
    action_dim = action.shape[1]
    parts = _split_obs(obs, action_dim)
    pose = parts["pose"]
    pose_vel = parts["pose_vel"]
    prev_m = parts["prev_motor"]
    prev_mv = parts["prev_motor_vel"]
    t = np.arange(obs.shape[0])
    pose_labels = ["x", "y", "z", "roll", "pitch", "yaw"]

    # --- Figure 1: line plots (pose, optional vels, prev motor, action) ---
    n_rows = 3 + (1 if pose_vel is not None else 0) + (1 if prev_mv is not None else 0) + (1 if condition is not None else 0)
    fig1, axes1 = plt.subplots(n_rows, 1, sharex=True, figsize=(12, 2.2 * n_rows + 0.5))
    if n_rows == 1:
        axes1 = [axes1]
    row = 0

    for i in range(6):
        axes1[row].plot(t, pose[:, i], label=pose_labels[i], linewidth=0.8)
    axes1[row].set_ylabel("pose")
    axes1[row].legend(loc="upper right", ncol=3, fontsize=7)
    axes1[row].set_title("Dataset time series")
    axes1[row].grid(True, alpha=0.3)
    row += 1

    if pose_vel is not None:
        for i in range(6):
            axes1[row].plot(t, pose_vel[:, i], label=f"d{pose_labels[i]}", linewidth=0.8)
        axes1[row].set_ylabel("pose Δ / step")
        axes1[row].legend(loc="upper right", ncol=3, fontsize=7)
        axes1[row].grid(True, alpha=0.3)
        row += 1

    for j in range(action_dim):
        axes1[row].plot(t, prev_m[:, j], linewidth=0.5, alpha=0.7)
    axes1[row].set_ylabel("prev motor (obs)")
    axes1[row].grid(True, alpha=0.3)
    row += 1

    if prev_mv is not None:
        for j in range(action_dim):
            axes1[row].plot(t, prev_mv[:, j], linewidth=0.5, alpha=0.7)
        axes1[row].set_ylabel("prev motor vel")
        axes1[row].grid(True, alpha=0.3)
        row += 1

    if condition is not None:
        cd = condition.shape[1]
        c_labels = ["joy x (goal₀)", "joy y (goal₁)", "joy z (progress)"][:cd]
        for i in range(cd):
            axes1[row].plot(t, condition[:, i], label=c_labels[i] if i < len(c_labels) else f"c{i}", linewidth=0.9)
        axes1[row].set_ylabel("condition")
        axes1[row].legend(loc="upper right", fontsize=8)
        axes1[row].grid(True, alpha=0.3)
        row += 1

    for j in range(action_dim):
        axes1[row].plot(t, action[:, j], linewidth=0.5, alpha=0.75)
    axes1[row].set_ylabel("action (cmd)")
    axes1[row].set_xlabel("sample index")
    axes1[row].grid(True, alpha=0.3)

    fig1.tight_layout()

    # --- Figure 2: heatmaps ---
    fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5))
    vmax_obs = np.percentile(np.abs(obs), 99)
    vmax_obs = float(max(vmax_obs, 1e-6))
    im0 = axes2[0].imshow(
        obs.T,
        aspect="auto",
        origin="lower",
        interpolation="nearest",
        cmap="coolwarm",
        vmin=-vmax_obs,
        vmax=vmax_obs,
        extent=(0, obs.shape[0] - 1, 0, obs.shape[1]),
    )
    axes2[0].set_xlabel("sample")
    axes2[0].set_ylabel("obs feature index")
    axes2[0].set_title("obs heatmap")
    fig2.colorbar(im0, ax=axes2[0], fraction=0.046, pad=0.04)

    vmax_a = np.percentile(np.abs(action), 99)
    vmax_a = float(max(vmax_a, 1e-6))
    im1 = axes2[1].imshow(
        action.T,
        aspect="auto",
        origin="lower",
        interpolation="nearest",
        cmap="coolwarm",
        vmin=-vmax_a,
        vmax=vmax_a,
        extent=(0, action.shape[0] - 1, 0, action.shape[1]),
    )
    axes2[1].set_xlabel("sample")
    axes2[1].set_ylabel("action dim")
    axes2[1].set_title("action heatmap")
    fig2.colorbar(im1, ax=axes2[1], fraction=0.046, pad=0.04)

    fig2.tight_layout()

    if condition is not None and condition.shape[1] > 0:
        fig3, ax3 = plt.subplots(figsize=(12, 2.5))
        vmax_c = np.percentile(np.abs(condition), 99)
        vmax_c = float(max(vmax_c, 1e-6))
        imc = ax3.imshow(
            condition.T,
            aspect="auto",
            origin="lower",
            interpolation="nearest",
            cmap="viridis",
            vmin=-vmax_c,
            vmax=vmax_c,
            extent=(0, condition.shape[0] - 1, 0, condition.shape[1]),
        )
        ax3.set_xlabel("sample")
        ax3.set_ylabel("condition dim")
        ax3.set_title("condition heatmap (e.g. joy_cmd)")
        fig3.colorbar(imc, ax=ax3, fraction=0.035, pad=0.04)
        fig3.tight_layout()


def _transform_points(T_world_body: np.ndarray, points_body: np.ndarray) -> np.ndarray:
    points_h = np.c_[points_body, np.ones((points_body.shape[0], 1))]
    return (T_world_body @ points_h.T).T[:, :3]


def _leg_chain_world(solver: SilverLainSolver, pose: np.ndarray, q: np.ndarray, leg_idx: int) -> np.ndarray:
    dh_params = solver._update_dh_params(leg_idx, q)
    T_world_body = solver._get_pose_transformation_matrix(pose)
    T = T_world_body.copy()

    points = [T[:3, -1].copy()]
    for param in dh_params:
        T = T @ solver._get_transformation_matrix(param)
        points.append(T[:3, -1].copy())

    # Use meaningful chain nodes: body center -> coxa root -> femur joint -> tibia joint -> foot.
    return np.vstack([points[0], points[2], points[4], points[5], points[6]])


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize SilverLain dataset (obs/action .npy).")
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=None,
        help=f"Directory containing npy files (default: {_default_data_dir()})",
    )
    parser.add_argument(
        "--tag",
        type=str,
        default=None,
        help="Load only {tag}_obs.npy / {tag}_action.npy (disables multi-run merge)",
    )
    parser.add_argument(
        "--latest-only",
        action="store_true",
        help="When several runs exist and --tag is not set, load only the newest *_obs.npy (old behavior)",
    )
    parser.add_argument(
        "--robot-only",
        action="store_true",
        help="Only open the 3D robot animation (skip base XY trajectory, obs/action/condition plots)",
    )
    args = parser.parse_args()
    obs, action, condition, segments = _load_dataset(
        data_dir=args.data_dir,
        tag=args.tag,
        latest_only=args.latest_only,
    )
    action_dim = action.shape[1]
    pose, prev_motor_cmd = _obs_position_slices(obs, action_dim)

    solver = SilverLainSolver(LINK_LIST)
    sample_count = obs.shape[0]

    # Precompute limits for stable animation.
    all_feet = np.zeros((sample_count, 6, 3), dtype=np.float64)
    all_hips = np.zeros((sample_count, 6, 3), dtype=np.float64)
    for i in range(sample_count):
        q = prev_motor_cmd[i]
        p = pose[i]
        all_feet[i] = solver.forward_with_pose(p, q)

        T_world_body = solver._get_pose_transformation_matrix(p)
        hip_body = np.array(
            [
                [LINK_LIST[0] * np.cos(-np.pi / 6 - np.pi / 3 * k), LINK_LIST[0] * np.sin(-np.pi / 6 - np.pi / 3 * k), 0.0]
                for k in range(3)
            ]
            + [
                [LINK_LIST[0] * np.cos(np.pi / 6 + np.pi / 3 * (5 - k)), LINK_LIST[0] * np.sin(np.pi / 6 + np.pi / 3 * (5 - k)), 0.0]
                for k in range(3, 6)
            ],
            dtype=np.float64,
        )
        all_hips[i] = _transform_points(T_world_body, hip_body)

    all_points = np.vstack([all_feet.reshape(-1, 3), all_hips.reshape(-1, 3), pose[:, :3]])
    mins = all_points.min(axis=0)
    maxs = all_points.max(axis=0)
    center = (mins + maxs) * 0.5
    # Same numeric range on X, Y, Z (cube limits) + isometric box aspect.
    half_ranges = np.maximum(center - mins, maxs - center)
    half = float(max(half_ranges.max() * 1.08, 0.5))

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    if len(segments) > 1:
        ax.set_title(f"SilverLain — {len(segments)} runs, {sample_count} frames (concatenated)")
    else:
        ax.set_title("SilverLain Hexapod Animation")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)
    ax.set_box_aspect((1, 1, 1))
    ax.grid(True)

    body_line, = ax.plot([], [], [], "k-", linewidth=2, label="body")
    leg_lines = [ax.plot([], [], [], "-", linewidth=2)[0] for _ in range(6)]
    feet_scatter = ax.scatter([], [], [], c="red", s=24, label="feet")
    text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)
    ax.legend(loc="upper right")

    def _update(frame_idx: int):
        p = pose[frame_idx]
        q = prev_motor_cmd[frame_idx]
        T_world_body = solver._get_pose_transformation_matrix(p)

        body_body = np.array(
            [
                [LINK_LIST[0] * np.cos(-np.pi / 6 - np.pi / 3 * k), LINK_LIST[0] * np.sin(-np.pi / 6 - np.pi / 3 * k), 0.0]
                for k in range(3)
            ]
            + [
                [LINK_LIST[0] * np.cos(np.pi / 6 + np.pi / 3 * (5 - k)), LINK_LIST[0] * np.sin(np.pi / 6 + np.pi / 3 * (5 - k)), 0.0]
                for k in range(3, 6)
            ],
            dtype=np.float64,
        )
        body_world = _transform_points(T_world_body, body_body)
        body_world = np.vstack([body_world, body_world[0]])
        body_line.set_data(body_world[:, 0], body_world[:, 1])
        body_line.set_3d_properties(body_world[:, 2])

        foot_points = np.zeros((6, 3), dtype=np.float64)
        for leg_idx, line in enumerate(leg_lines):
            chain = _leg_chain_world(solver, p, q, leg_idx)
            line.set_data(chain[:, 0], chain[:, 1])
            line.set_3d_properties(chain[:, 2])
            foot_points[leg_idx] = chain[-1]

        feet_scatter._offsets3d = (foot_points[:, 0], foot_points[:, 1], foot_points[:, 2])
        seg_label, local_i, seg_len = _segment_info(segments, frame_idx)
        text.set_text(
            f"run {seg_label!r}  {local_i + 1}/{seg_len}  |  total {frame_idx + 1}/{sample_count}"
        )
        return [body_line, *leg_lines, feet_scatter, text]

    if not args.robot_only:
        _plot_base_trajectory_2d(pose, segments)
        _plot_data_dashboard(obs, action, condition)
        plt.show(block=False)

    anim = FuncAnimation(fig, _update, frames=sample_count, interval=40, blit=False, repeat=True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
