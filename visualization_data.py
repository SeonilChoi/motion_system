#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path
from typing import NamedTuple

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# This file lives at motion_system/visualization_data.py
_MOTION_SYSTEM_DIR = Path(__file__).resolve().parent
# Same default as data_collector_node ``output_dir``: motion_system/ros2/test_pkg/data
_DEFAULT_SPLIT_DATA_DIR = _MOTION_SYSTEM_DIR.joinpath("ros2", "test_pkg", "data")

# Split field order matches ``ros2/test_pkg/test_pkg/data_collector_node.py`` (_save_obs_split / hstack)
_SPLIT_STEMS: tuple[str, ...] = (
    "position",
    "orientation",
    "direction",
    "progress",
    "target_contact",
    "target_position",
    "target_orientation",
    "target_linear_velocity",
    "target_angular_velocity",
    "target_joint_position",
    "target_joint_velocity",
)

# Must match data_collector_node. On disk: position / target_position z += POSE_Z_OFFSET;
# joint position = offset + reorder + direction; joint velocity = reorder + direction only.
# Inverses below used for FK / animation where solver expects incoming obs frame.
_POSE_Z_OFFSET = np.float32(-0.7788)
_JOINT_POS_OFFSET = np.array(
    [
        0.0,
        0.6981317007977318,
        -2.443460952792061,
        0.0,
        0.6981317007977318,
        -2.443460952792061,
        0.0,
        0.6981317007977318,
        -2.443460952792061,
        0.0,
        -0.6981317007977318,
        2.443460952792061,
        0.0,
        -0.6981317007977318,
        2.443460952792061,
        0.0,
        -0.6981317007977318,
        2.443460952792061,
    ],
    dtype=np.float64,
)
_JOINT_REORDER = np.array(
    [15, 12, 9, 6, 3, 0, 16, 13, 10, 7, 4, 1, 17, 14, 11, 8, 5, 2],
    dtype=np.intp,
)
_JOINT_DIRECTION = np.array(
    [-1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, 1],
    dtype=np.float64,
)
_INV_JOINT_REORDER = np.empty(18, dtype=np.intp)
_INV_JOINT_REORDER[_JOINT_REORDER] = np.arange(18, dtype=np.intp)


def _translation_from_storage(t: np.ndarray) -> np.ndarray:
    """Remove z offset saved on disk (obs frame for SilverLain FK)."""
    x = np.asarray(t, dtype=np.float64).copy()
    x[:, 2] = x[:, 2] - float(_POSE_Z_OFFSET)
    return x


def _joint_positions_from_storage(stored: np.ndarray) -> np.ndarray:
    """Invert offset → reorder → direction to get SilverLain solver joint order."""
    b = np.asarray(stored, dtype=np.float64) / _JOINT_DIRECTION
    a = b[:, _INV_JOINT_REORDER]
    return a - _JOINT_POS_OFFSET


def _joint_velocities_from_storage(stored: np.ndarray) -> np.ndarray:
    """Invert reorder → direction (no offset) to match solver-frame joint rates, rad/s."""
    x = np.asarray(stored, dtype=np.float64)
    if x.ndim != 2 or x.shape[1] != _JOINT_DIRECTION.shape[0]:
        return x
    b = x / _JOINT_DIRECTION
    return b[:, _INV_JOINT_REORDER]


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
    return _DEFAULT_SPLIT_DATA_DIR


def _stem_sort_key(stem: str) -> tuple:
    if stem.isdigit():
        return (0, int(stem), "")
    return (1, stem, "")


def _split_bundle_path(data_dir: Path, prefix: str, stem: str) -> Path:
    """``data_dir / prefix / stem.npy`` (per-run folder) or flat ``prefix_stem.npy``."""
    if prefix:
        sub = data_dir / prefix
        if sub.is_dir():
            return sub / f"{stem}.npy"
        return data_dir / f"{prefix}_{stem}.npy"
    return data_dir / f"{stem}.npy"


def _try_load_split_bundle(data_dir: Path, prefix: str) -> dict[str, np.ndarray] | None:
    paths = {s: _split_bundle_path(data_dir, prefix, s) for s in _SPLIT_STEMS}
    if not paths["position"].is_file():
        return None
    if not all(p.is_file() for p in paths.values()):
        return None
    arrays: dict[str, np.ndarray] = {}
    for k, p in paths.items():
        arrays[k] = np.load(p)
    n = int(arrays["position"].shape[0])
    for k in _SPLIT_STEMS:
        a = arrays[k]
        if a.ndim == 1:
            arrays[k] = a.reshape(n, -1)
        elif a.ndim != 2:
            raise ValueError(f"{k}: expected 1D or 2D, got {a.shape}")
        if arrays[k].shape[0] != n:
            raise ValueError(f"{k}: expected {n} rows, got {arrays[k].shape[0]}")
    return arrays


def _split_bundle_max_mtime(data_dir: Path, prefix: str) -> float:
    return max(_split_bundle_path(data_dir, prefix, s).stat().st_mtime for s in _SPLIT_STEMS)


def _list_split_prefixes(data_dir: Path) -> list[str]:
    data_dir = data_dir.expanduser().resolve()
    out: set[str] = set()
    if _try_load_split_bundle(data_dir, "") is not None:
        out.add("")
    for sub in data_dir.iterdir():
        if sub.is_dir() and not sub.name.startswith("."):
            if _try_load_split_bundle(data_dir, sub.name) is not None:
                out.add(sub.name)
    for p in data_dir.glob("*_position.npy"):
        prefix = p.name[: -len("_position.npy")]
        if prefix and _try_load_split_bundle(data_dir, prefix) is not None:
            out.add(prefix)
    return sorted(out, key=_stem_sort_key)


def _hstack_from_split(parts: dict[str, np.ndarray]) -> np.ndarray:
    return np.hstack([parts[s] for s in _SPLIT_STEMS])


def _merge_split_prefixes(data_dir: Path, prefixes: list[str]) -> tuple[dict[str, np.ndarray], list[tuple[int, int, str]]]:
    blocks: list[dict[str, np.ndarray]] = []
    segments: list[tuple[int, int, str]] = []
    pos = 0
    for prefix in prefixes:
        d = _try_load_split_bundle(data_dir, prefix)
        if d is None:
            raise RuntimeError(f"incomplete split bundle for prefix {prefix!r}")
        n = d["position"].shape[0]
        blocks.append(d)
        label = prefix if prefix else "split"
        segments.append((pos, pos + n, label))
        pos += n
    merged: dict[str, np.ndarray] = {}
    for k in _SPLIT_STEMS:
        merged[k] = np.vstack([b[k] for b in blocks])
    return merged, segments


def _load_condition_file(data_dir: Path, stem: str, n_rows: int) -> np.ndarray | None:
    candidates: list[Path] = []
    if stem:
        candidates.append(data_dir / stem / "condition.npy")
    candidates.append(data_dir / (f"{stem}_condition.npy" if stem else "condition.npy"))
    for cond_path in candidates:
        if not cond_path.is_file():
            continue
        c = np.load(cond_path)
        if c.ndim == 2 and c.shape[0] == n_rows:
            return c
        print(
            f"Skipping {cond_path}: expected 2D with {n_rows} rows, got {getattr(c, 'shape', None)}"
        )
    return None


class LoadedDataset(NamedTuple):
    """Hstacked features + per-field arrays from data_collector split .npy bundles."""

    obs: np.ndarray
    condition: np.ndarray | None
    segments: list[tuple[int, int, str]]
    split_parts: dict[str, np.ndarray]


class JointJumpAnalysis(NamedTuple):
    """Per-transition stats on ``target_joint_position`` (storage frame, same as on disk)."""

    dq: np.ndarray
    max_abs_per_transition: np.ndarray
    l2_per_transition: np.ndarray
    transition_valid: np.ndarray
    threshold: float
    spike_indices: np.ndarray


def _inter_run_transition_mask(n: int, segments: list[tuple[int, int, str]]) -> np.ndarray:
    """Length ``n - 1``. False where diff crosses a stitched run boundary (merged datasets)."""
    valid = np.ones(max(0, n - 1), dtype=bool)
    for s, _, _ in segments:
        if s > 0:
            valid[s - 1] = False
    return valid


def analyze_joint_jumps(
    joint_storage: np.ndarray,
    segments: list[tuple[int, int, str]],
    *,
    percentile: float = 99.5,
    mult: float = 2.5,
) -> JointJumpAnalysis:
    """Flag abrupt joint steps: threshold = ``percentile(valid max-abs steps) * mult``."""
    j = np.asarray(joint_storage, dtype=np.float64)
    n = j.shape[0]
    n_j = int(j.shape[1]) if j.ndim == 2 else 0
    if n < 2:
        z = np.zeros((0, n_j), dtype=np.float64)
        return JointJumpAnalysis(
            dq=z,
            max_abs_per_transition=np.zeros(0, dtype=np.float64),
            l2_per_transition=np.zeros(0, dtype=np.float64),
            transition_valid=np.zeros(0, dtype=bool),
            threshold=float("nan"),
            spike_indices=np.array([], dtype=np.intp),
        )

    dq = np.diff(j, axis=0)
    max_abs = np.max(np.abs(dq), axis=1)
    l2 = np.linalg.norm(dq, axis=1)
    valid = _inter_run_transition_mask(n, segments)
    if not np.any(valid):
        return JointJumpAnalysis(
            dq=dq,
            max_abs_per_transition=max_abs,
            l2_per_transition=l2,
            transition_valid=valid,
            threshold=float("nan"),
            spike_indices=np.array([], dtype=np.intp),
        )

    ref = float(np.percentile(max_abs[valid], percentile))
    thr = max(ref * mult, 1e-12)
    spikes = np.where(valid & (max_abs > thr))[0].astype(np.intp, copy=False)
    return JointJumpAnalysis(
        dq=dq,
        max_abs_per_transition=max_abs,
        l2_per_transition=l2,
        transition_valid=valid,
        threshold=thr,
        spike_indices=spikes,
    )


def _print_joint_jump_report(
    analysis: JointJumpAnalysis,
    segments: list[tuple[int, int, str]],
    *,
    percentile: float,
    mult: float,
    top_k: int = 35,
) -> None:
    n_t = analysis.dq.shape[0]
    if n_t == 0:
        print("Joint jump analysis: not enough samples.")
        return

    valid_count = int(np.count_nonzero(analysis.transition_valid))
    print(
        "\n--- Joint step jumps (target_joint_position on disk, rad) ---\n"
        f"Transitions: {n_t}, valid (no run stitch): {valid_count}, "
        f"threshold = {analysis.threshold:.6g} ({percentile}pct × {mult} on valid steps only)\n"
        f"Spikes (abrupt): {analysis.spike_indices.size}"
    )
    if analysis.spike_indices.size == 0:
        print("(none above threshold)\n")
        return

    order = np.argsort(-analysis.max_abs_per_transition[analysis.spike_indices])
    show = order[: min(top_k, order.size)]
    print(f"Top {show.size} spike(s) by max |Δq_j|:\n")
    for rank, oi in enumerate(show, start=1):
        ti = int(analysis.spike_indices[int(oi)])
        step = float(analysis.max_abs_per_transition[ti])
        row = analysis.dq[ti]
        ji = int(np.argmax(np.abs(row)))
        dq_j = float(row[ji])
        lab, loc_i, seg_n = _segment_info(segments, ti + 1)
        print(
            f"  {rank:2d}. transition {ti}→{ti + 1}  global frame after: {ti + 1}  "
            f"run {lab!r} local {loc_i + 1}/{seg_n}  "
            f"max|Δq|={step:.6g} (joint {ji}, Δ={dq_j:+.6g})"
        )
    print()


def _plot_joint_jump_figures(analysis: JointJumpAnalysis) -> None:
    n_t = analysis.dq.shape[0]
    if n_t == 0:
        return

    n = n_t + 1
    x_end = np.arange(1, n, dtype=np.float64)
    mx = analysis.max_abs_per_transition
    l2 = analysis.l2_per_transition
    thr = analysis.threshold
    spikes = analysis.spike_indices
    inv = ~analysis.transition_valid
    y_top = float(np.max(mx)) if mx.size else 1.0
    if np.isfinite(thr):
        y_top = max(y_top, thr * 1.05)

    fig_m, axes_m = plt.subplots(2, 1, sharex=True, figsize=(12, 5.5))
    ax0, ax1 = axes_m[0], axes_m[1]
    ax0.plot(x_end, mx, color="0.35", linewidth=0.7, alpha=0.9)
    ax1.plot(x_end, l2, color="0.35", linewidth=0.7, alpha=0.9)
    leg_handles: list = []
    if np.isfinite(thr):
        leg_handles.append(
            ax0.axhline(
                thr,
                color="crimson",
                linestyle="--",
                linewidth=1.0,
                label=f"threshold ({thr:.4g})",
            )
        )
    if np.any(inv):
        for ax in (ax0, ax1):
            ax.fill_between(
                x_end,
                0.0,
                y_top,
                where=inv,
                color="0.85",
                alpha=0.35,
                step="pre",
                linewidth=0,
            )
        leg_handles.append(
            mpatches.Patch(facecolor="0.85", alpha=0.35, edgecolor="none", label="stitched run boundary")
        )
    if spikes.size:
        for ax in (ax0, ax1):
            for si in spikes:
                ax.axvline(si + 1, color="orange", alpha=0.25, linewidth=0.8)
    ax0.set_ylabel("max |Δq| (rad)")
    ax1.set_ylabel("‖Δq‖₂ (rad)")
    ax0.grid(True, alpha=0.3)
    ax1.grid(True, alpha=0.3)
    ax0.set_title("Joint angle step size (storage-frame target_joint_position; k→k+1)")
    ax1.set_xlabel("sample index after step (transition k→k+1 ends at row k+1)")
    if leg_handles:
        ax0.legend(handles=leg_handles, loc="upper right", fontsize=8)
    fig_m.tight_layout()

    abs_dq = np.abs(analysis.dq)
    vmax = float(max(np.percentile(abs_dq, 99.5), 1e-9))
    fig_h, ax_h = plt.subplots(figsize=(13, 4.2))
    im = ax_h.imshow(
        abs_dq.T,
        aspect="auto",
        origin="lower",
        interpolation="nearest",
        cmap="magma",
        vmin=0.0,
        vmax=vmax,
        extent=(0.5, n - 0.5, -0.5, abs_dq.shape[1] - 0.5),
    )
    ax_h.set_xlabel("transition index (step into sample column)")
    ax_h.set_ylabel("joint column (storage order)")
    ax_h.set_title("|Δq| heatmap (abrupt rows align with bright horizontal streaks)")
    if spikes.size:
        for si in spikes:
            ax_h.axvline(si + 1, color="cyan", alpha=0.35, linewidth=0.7)
    fig_h.colorbar(im, ax=ax_h, fraction=0.035, pad=0.02).set_label("|Δq| (rad)")
    fig_h.tight_layout()


def _load_dataset(
    data_dir: Path | None = None,
    tag: str | None = None,
    *,
    latest_only: bool = False,
) -> LoadedDataset:
    base = data_dir if data_dir is not None else _default_data_dir()

    if tag is not None:
        prefix = tag.strip()
        d = _try_load_split_bundle(base, prefix)
        if d is None:
            loc = f"{base.name}/{prefix}/" if prefix else f"{base.name}/"
            raise FileNotFoundError(
                f"No split bundle for --tag {prefix!r} (expected 11 arrays under {loc} or flat {prefix}_*.npy)."
            )
        n = d["position"].shape[0]
        obs = _hstack_from_split(d)
        cond = _load_condition_file(base, prefix, n)
        loc = f"{base.name}/{prefix}/" if prefix else f"{base.name}/"
        print(f"Loaded split bundle {loc}… ({n} samples)")
        if cond is not None:
            print(f"Loaded condition {cond.shape}")
        return LoadedDataset(obs, cond, [(0, n, prefix or "split")], d)

    split_prefixes = _list_split_prefixes(base)
    if split_prefixes:
        if latest_only and len(split_prefixes) > 1:
            prefix = max(split_prefixes, key=lambda pr: _split_bundle_max_mtime(base, pr))
            d = _try_load_split_bundle(base, prefix)
            assert d is not None
            n = d["position"].shape[0]
            obs = _hstack_from_split(d)
            cond = _load_condition_file(base, prefix, n)
            loc = f"{base.name}/{prefix}/" if prefix else f"{base.name}/"
            print(f"Loaded (--latest-only) split {loc}… ({n} samples)")
            return LoadedDataset(obs, cond, [(0, n, prefix or "split")], d)

        merged, segments = _merge_split_prefixes(base, split_prefixes)
        obs = _hstack_from_split(merged)
        n = obs.shape[0]
        cond_blocks: list[tuple[int, np.ndarray | None]] = []
        pos = 0
        for s, e, lab in segments:
            nn = e - s
            c = _load_condition_file(base, lab if lab != "split" else "", nn)
            cond_blocks.append((nn, c))
            pos += nn

        condition = None
        cond_widths = {c.shape[1] for _, c in cond_blocks if c is not None}
        if len(cond_widths) == 1:
            cd = cond_widths.pop()
            parts_c: list[np.ndarray] = []
            for nn, c in cond_blocks:
                if c is not None and c.shape[1] == cd:
                    parts_c.append(c)
                else:
                    parts_c.append(np.full((nn, cd), np.nan, dtype=np.float64))
            condition = np.vstack(parts_c)
        print(
            f"Loaded {len(split_prefixes)} split run(s) from {base} → {n} samples "
            f"(order: {[s[2] for s in segments]})"
        )
        if condition is not None:
            print(f"Merged condition {condition.shape}")
        return LoadedDataset(obs, condition, segments, merged)

    raise FileNotFoundError(
        f"No split dataset in {base}: need ``position.npy`` (and 10 siblings) in a subfolder or flat, "
        f"e.g. ``data/1/position.npy`` or ``data/1_position.npy``. See data_collector_node."
    )


def _segment_info(
    segments: list[tuple[int, int, str]], frame_idx: int
) -> tuple[str, int, int]:
    for s, e, label in segments:
        if s <= frame_idx < e:
            return label, frame_idx - s, e - s
    return "?", frame_idx, 0


def _pose_and_joints_from_dataset(ds: LoadedDataset) -> tuple[np.ndarray, np.ndarray]:
    p = ds.split_parts
    pose = np.hstack([_translation_from_storage(p["position"]), p["orientation"]])
    q = _joint_positions_from_storage(p["target_joint_position"])
    return pose, q


def _plot_base_trajectory_2d(pose: np.ndarray, segments: list[tuple[int, int, str]]) -> None:
    bx = pose[:, 0]
    by = pose[:, 1]
    n = pose.shape[0]
    frames = np.arange(n)

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


def _plot_joint_angles_figure(
    parts: dict[str, np.ndarray],
    segments: list[tuple[int, int, str]],
) -> None:
    """Separate figure: joint angles + joint velocities in solver frame (rad, rad/s)."""
    jm = np.asarray(parts["target_joint_position"], dtype=np.float64)
    n, n_j = jm.shape[0], jm.shape[1]
    if n_j < 1:
        return
    q = _joint_positions_from_storage(jm)
    t = np.arange(n, dtype=np.float64)

    jv = parts.get("target_joint_velocity")
    has_vel = (
        jv is not None
        and isinstance(jv, np.ndarray)
        and jv.shape == jm.shape
    )
    qd = _joint_velocities_from_storage(np.asarray(jv, dtype=np.float64)) if has_vel else None

    n_legs = 6

    def _mark_run_boundaries(axes_list: list) -> None:
        for s, _e, _lab in segments:
            if s <= 0:
                continue
            for ax in axes_list:
                ax.axvline(s, color="0.65", linestyle=":", linewidth=0.9, alpha=0.85)

    if n_j % n_legs != 0:
        nrows = 2 if has_vel else 1
        fig, axs = plt.subplots(nrows, 1, figsize=(14, 4.8 * nrows), sharex=True)
        if nrows == 1:
            axs = [axs]
        cmap = plt.cm.tab20(np.linspace(0, 1, min(n_j, 20)))
        for j in range(n_j):
            axs[0].plot(t, q[:, j], color=cmap[j % len(cmap)], label=f"j{j}", linewidth=0.85, alpha=0.9)
        axs[0].set_ylabel("angle (rad)")
        axs[0].set_title("Joint angles (solver frame) — non-6× layout")
        axs[0].grid(True, alpha=0.3)
        axs[0].legend(loc="upper right", ncol=min(6, n_j), fontsize=7)
        _mark_run_boundaries(list(axs))
        if has_vel and qd is not None:
            for j in range(n_j):
                axs[1].plot(t, qd[:, j], color=cmap[j % len(cmap)], label=f"j{j}", linewidth=0.85, alpha=0.9)
            axs[1].set_xlabel("sample index")
            axs[1].set_ylabel("rate (rad/s)")
            axs[1].set_title("Joint velocities (solver frame)")
            axs[1].grid(True, alpha=0.3)
            axs[1].legend(loc="upper right", ncol=min(6, n_j), fontsize=7)
        else:
            axs[0].set_xlabel("sample index")
        fig.suptitle(
            "Joint angles & velocities (solver frame) — storage inverted per data_collector",
            fontsize=11,
        )
        fig.tight_layout(rect=[0, 0, 1, 0.97])
        return

    joints_per_leg = n_j // n_legs
    n_row = 4 if has_vel else 2
    fig, axes = plt.subplots(n_row, 3, sharex=True, figsize=(14, 3.3 * n_row))
    jnames = ("j0", "j1", "j2")
    all_flat: list = []
    for leg in range(n_legs):
        r, c = divmod(leg, 3)
        ax_a = axes[r, c]
        all_flat.append(ax_a)
        base = leg * joints_per_leg
        for k in range(joints_per_leg):
            ax_a.plot(
                t,
                q[:, base + k],
                label=jnames[k] if k < len(jnames) else f"j{k}",
                linewidth=0.9,
                alpha=0.92,
            )
        ax_a.set_ylabel(f"L{leg} θ (rad)")
        ax_a.grid(True, alpha=0.3)
        ax_a.legend(loc="upper right", fontsize=7)
        if has_vel and qd is not None:
            ax_v = axes[r + 2, c]
            all_flat.append(ax_v)
            for k in range(joints_per_leg):
                ax_v.plot(
                    t,
                    qd[:, base + k],
                    label=jnames[k] if k < len(jnames) else f"j{k}",
                    linewidth=0.9,
                    alpha=0.92,
                )
            ax_v.set_ylabel(f"L{leg} dq/dt (rad/s)")
            ax_v.grid(True, alpha=0.3)
            ax_v.legend(loc="upper right", fontsize=7)

    _mark_run_boundaries(all_flat)
    for c in range(3):
        axes[n_row - 1, c].set_xlabel("sample index")
    fig.suptitle(
        "Joint angles (top) & joint velocities (bottom), solver frame — offset/reorder/sign inverted",
        fontsize=11,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.97])


def _plot_split_dashboard(parts: dict[str, np.ndarray], condition: np.ndarray | None) -> None:
    n = parts["position"].shape[0]
    t = np.arange(n)
    rows_spec: list[tuple[str, np.ndarray, list[str] | None]] = [
        ("position (xyz)", parts["position"], ["x", "y", "z"]),
        ("orientation (rpy?)", parts["orientation"], ["a", "b", "c"]),
        ("direction", parts["direction"], ["u", "v"]),
        ("progress", parts["progress"], ["p"]),
        ("target_contact", parts["target_contact"], [f"L{i}" for i in range(parts["target_contact"].shape[1])]),
        ("target_position", parts["target_position"], ["x", "y", "z"]),
        ("target_orientation", parts["target_orientation"], ["a", "b", "c"]),
        ("target_linear_velocity", parts["target_linear_velocity"], ["vx", "vy", "vz"]),
        ("target_angular_velocity", parts["target_angular_velocity"], ["wx", "wy", "wz"]),
        ("target_joint_position", parts["target_joint_position"], None),
        ("target_joint_velocity", parts["target_joint_velocity"], None),
    ]
    n_rows = len(rows_spec) + (1 if condition is not None else 0)
    fig1, axes1 = plt.subplots(n_rows, 1, sharex=True, figsize=(12, 1.9 * n_rows + 0.5))
    if n_rows == 1:
        axes1 = [axes1]
    row = 0
    for ylabel, mat, labels in rows_spec:
        ax = axes1[row]
        nc = mat.shape[1]
        if labels is not None and len(labels) >= nc:
            for j in range(nc):
                ax.plot(t, mat[:, j], label=labels[j], linewidth=0.65)
            ax.legend(loc="upper right", ncol=min(4, nc), fontsize=7)
        else:
            for j in range(nc):
                ax.plot(t, mat[:, j], linewidth=0.45, alpha=0.75)
        ax.set_ylabel(ylabel[:24])
        ax.grid(True, alpha=0.3)
        if row == 0:
            ax.set_title("Split dataset (data_collector fields)")
        row += 1

    if condition is not None:
        cd = condition.shape[1]
        c_labels = ["joy x", "joy y", "joy z (progress)"][:cd]
        for i in range(cd):
            axes1[row].plot(t, condition[:, i], label=c_labels[i] if i < len(c_labels) else f"c{i}", linewidth=0.9)
        axes1[row].set_ylabel("condition")
        axes1[row].legend(loc="upper right", fontsize=8)
        axes1[row].grid(True, alpha=0.3)
        row += 1

    axes1[row - 1].set_xlabel("sample index")
    fig1.tight_layout()

    fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5))
    obs = _hstack_from_split(parts)
    vmax = float(max(np.percentile(np.abs(obs), 99), 1e-6))
    im0 = axes2[0].imshow(
        obs.T,
        aspect="auto",
        origin="lower",
        interpolation="nearest",
        cmap="coolwarm",
        vmin=-vmax,
        vmax=vmax,
        extent=(0, n - 1, 0, obs.shape[1]),
    )
    axes2[0].set_xlabel("sample")
    axes2[0].set_ylabel("concat feature index")
    axes2[0].set_title("obs (hstacked split fields)")
    fig2.colorbar(im0, ax=axes2[0], fraction=0.046, pad=0.04)

    jm = parts["target_joint_position"]
    vmax_j = float(max(np.percentile(np.abs(jm), 99), 1e-6))
    im1 = axes2[1].imshow(
        jm.T,
        aspect="auto",
        origin="lower",
        interpolation="nearest",
        cmap="coolwarm",
        vmin=-vmax_j,
        vmax=vmax_j,
        extent=(0, n - 1, 0, jm.shape[1]),
    )
    axes2[1].set_xlabel("sample")
    axes2[1].set_ylabel("joint index")
    axes2[1].set_title("target_joint_position")
    fig2.colorbar(im1, ax=axes2[1], fraction=0.046, pad=0.04)
    fig2.tight_layout()

    if condition is not None and condition.shape[1] > 0:
        fig3, ax3 = plt.subplots(figsize=(12, 2.5))
        vmax_c = float(max(np.percentile(np.abs(condition), 99), 1e-6))
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
        ax3.set_title("condition heatmap")
        fig3.colorbar(imc, ax=ax3, fraction=0.035, pad=0.04)
        fig3.tight_layout()


def _plot_data_dashboard(ds: LoadedDataset, joint_jump: JointJumpAnalysis | None = None) -> None:
    _plot_split_dashboard(ds.split_parts, ds.condition)
    _plot_joint_angles_figure(ds.split_parts, ds.segments)
    if joint_jump is not None:
        _plot_joint_jump_figures(joint_jump)


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

    return np.vstack([points[0], points[2], points[4], points[5], points[6]])


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize SilverLain data_collector split .npy bundles (per-run folder or flat names)."
    )
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
        help="Load only this run prefix, e.g. data/1/ → --tag 1 (or flat 1_position.npy, …)",
    )
    parser.add_argument(
        "--latest-only",
        action="store_true",
        help="If multiple runs exist, load only the newest (by mtime)",
    )
    parser.add_argument(
        "--robot-only",
        action="store_true",
        help="Only 3D animation (skip trajectory + dashboard)",
    )
    parser.add_argument(
        "--no-joint-jump",
        action="store_true",
        help="Skip joint abrupt-change console report and jump figures",
    )
    parser.add_argument(
        "--joint-percentile",
        type=float,
        default=99.5,
        help="Percentile of valid per-step max|Δq| for threshold (default: 99.5)",
    )
    parser.add_argument(
        "--joint-mult",
        type=float,
        default=2.5,
        help="Threshold = percentile value × this factor (default: 2.5)",
    )
    args = parser.parse_args()
    ds = _load_dataset(
        data_dir=args.data_dir,
        tag=args.tag,
        latest_only=args.latest_only,
    )

    # 잠시: 베이스 포즈(x,y,z + rpy) 궤적 2D만 표시. 아래 ``if False`` 블록에서 나머지 복구.
    _parts = ds.split_parts
    pose = np.hstack([_translation_from_storage(_parts["position"]), _parts["orientation"]])
    _plot_base_trajectory_2d(pose, ds.segments)
    plt.show()

    if False:  # noqa: SIM102 — 관절 점프·대시보드·3D 애니 (필요 시 False→코드 밖으로 꺼내기)
        joint_analysis: JointJumpAnalysis | None = None
        if not args.no_joint_jump:
            joint_analysis = analyze_joint_jumps(
                ds.split_parts["target_joint_position"],
                ds.segments,
                percentile=float(args.joint_percentile),
                mult=float(args.joint_mult),
            )
            _print_joint_jump_report(
                joint_analysis,
                ds.segments,
                percentile=float(args.joint_percentile),
                mult=float(args.joint_mult),
            )

        pose, prev_motor_cmd = _pose_and_joints_from_dataset(ds)

        solver = SilverLainSolver(LINK_LIST)
        sample_count = pose.shape[0]

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
        half_ranges = np.maximum(center - mins, maxs - center)
        half = float(max(half_ranges.max() * 1.08, 0.5))

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")
        if len(ds.segments) > 1:
            ax.set_title(f"SilverLain — {len(ds.segments)} runs, {sample_count} frames (concatenated)")
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
            seg_label, local_i, seg_len = _segment_info(ds.segments, frame_idx)
            text.set_text(
                f"run {seg_label!r}  {local_i + 1}/{seg_len}  |  total {frame_idx + 1}/{sample_count}"
            )
            return [body_line, *leg_lines, feet_scatter, text]

        if not args.robot_only:
            _plot_base_trajectory_2d(pose, ds.segments)
            _plot_data_dashboard(ds, joint_analysis)
            plt.show(block=False)

        anim = FuncAnimation(fig, _update, frames=sample_count, interval=40, blit=False, repeat=True)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
