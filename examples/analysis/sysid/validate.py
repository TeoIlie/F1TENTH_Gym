"""Single-call validation/debugging entry point for the STD sysid pipeline.

Given a 100 Hz Vicon NPZ bag and a parameter set, produces:

  - dataset_overview.png            — windowed bag signals
  - dataset_overview_mirror.png     — same, mirrored windows (omitted with --no-mirror)
  - rollout_overlay.png             — sim vs real, one segment per window
  - metrics.txt                     — window counts, drop counts, total + per-channel loss

To compare two parameter sets, run twice with different `--params` /
`--out-dir` and diff `metrics.txt`. Replaces the prior split between
`compare_loss.py` and the `__main__` blocks of `dataset.py` / `rollout.py`.

Example:
  python -m examples.analysis.sysid.validate --bag bag.npz [--params tuned.yaml] [--mirror] [--out-dir DIR]
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
import yaml
from scipy.signal import savgol_filter

from examples.analysis.sysid.dataset import CHANNELS, Dataset, Window, load_dataset
from examples.analysis.sysid.env import SYSID_PARAMS
from examples.analysis.sysid.loss import CHANNEL_COEFFS, window_loss
from examples.analysis.sysid.rollout import Rollout


def _plot_dataset_overview(dataset: Dataset, npz_path: str, out_path: str, mirror: bool = False) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = np.load(npz_path)
    t = data["t"]
    vicon_x = data["vicon_x"]
    vicon_y = data["vicon_y"]
    vicon_body_vx = data["vicon_body_vx"]
    vicon_body_vy = data["vicon_body_vy"]
    vicon_r = data["vicon_r"]
    cmd_speed = data["cmd_speed"]
    cmd_steer = data["cmd_steer"]
    rs_core_speed = data["rs_core_speed"]
    omega_full = rs_core_speed / SYSID_PARAMS["R_w"]

    if mirror:
        # Reflect raw bag signals so they match the mirrored windows we're about to overlay.
        vicon_y = -vicon_y
        vicon_body_vy = -vicon_body_vy
        vicon_r = -vicon_r
        cmd_steer = -cmd_steer
        # omega is sign-symmetric under L/R mirror — no flip.

    real_a_x = np.gradient(savgol_filter(vicon_body_vx, 21, 2), dataset.dt)
    real_speed = np.hypot(vicon_body_vx, vicon_body_vy)
    real_beta = np.where(real_speed > 0.2, np.arctan2(vicon_body_vy, vicon_body_vx), np.nan)

    selected = [w for w in dataset.windows if w.is_mirrored == mirror]
    n_steps = len(selected[0].cmd_steer) if selected else 0

    suffix = " [MIRRORED]" if mirror else ""
    fig, grid = plt.subplots(2, 5, figsize=(40, 14))
    fig.suptitle(
        f"Dataset overview{suffix} — {os.path.basename(npz_path)} ({len(selected)} windows)",
        fontsize=14,
    )
    axes = [
        grid[0, 0],  # XY
        grid[0, 1],  # vx
        grid[0, 2],  # vy
        grid[1, 0],  # steer
        grid[1, 1],  # yaw_rate
        grid[1, 2],  # a_x
        grid[1, 3],  # beta
        grid[0, 3],  # omega
        grid[0, 4],  # heading (yaw)
    ]

    def shade_windows(ax):
        for w in selected:
            ax.axvspan(t[w.t0_idx], t[w.t0_idx + n_steps], color="orange", alpha=0.15, linewidth=0)

    ax = axes[0]
    ax.plot(vicon_x, vicon_y, label="Real (Vicon)", linewidth=1.0)
    for w in selected:
        ax.plot(w.init_state[0], w.init_state[1], "o", color="orange", markersize=4)
    ax.plot(vicon_x[0], vicon_y[0], "ko", markersize=8, label="Bag start")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("XY Trajectory (orange dots = window init_state)")
    ax.set_aspect("equal")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    shade_windows(ax)
    ax.plot(t, cmd_speed, label="cmd_speed", linewidth=1, alpha=0.6)
    ax.plot(t, vicon_body_vx, label="Real vx", linewidth=1)
    for w in selected:
        ax.plot(t[w.t0_idx], w.init_state[3], "o", color="orange", markersize=4)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("vx (orange dots = init speed)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    shade_windows(ax)
    ax.plot(t, vicon_body_vy, label="Real vy", linewidth=1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("vy")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[3]
    shade_windows(ax)
    ax.plot(t, cmd_steer, label="cmd_steer", linewidth=1)
    for w in selected:
        ax.plot(t[w.t0_idx], w.init_state[2], "o", color="orange", markersize=4)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Steering (rad)")
    ax.set_title("Steering cmd (orange dots = init delta)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[4]
    shade_windows(ax)
    ax.plot(t, vicon_r, label="Real yaw rate", linewidth=1)
    for w in selected:
        ax.plot(t[w.t0_idx], w.init_state[5], "o", color="orange", markersize=4)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Yaw rate (rad/s)")
    ax.set_title("Yaw rate (orange dots = init yaw_rate)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[5]
    shade_windows(ax)
    ax.plot(t, real_a_x, label="Real a_x (smoothed)", linewidth=1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.set_title("Longitudinal acceleration")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[6]
    shade_windows(ax)
    ax.plot(t, real_beta, label="Real β", linewidth=1)
    for w in selected:
        ax.plot(t[w.t0_idx], w.init_state[6], "o", color="orange", markersize=4)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Slip angle β (rad)")
    ax.set_title("Slip angle (orange dots = init beta)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[7]
    shade_windows(ax)
    ax.plot(t, omega_full, label=f"VESC ω (rs_core_speed/R_w, R_w={SYSID_PARAMS['R_w']:.4f})", linewidth=1)
    for w in selected:
        ax.plot(t[w.t0_idx], w.init_state[7], "o", color="orange", markersize=4)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular velocity (rad/s)")
    ax.set_title("Wheel ω (orange dots = init ω_f = init ω_r, AWD)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[8]
    shade_windows(ax)
    real_yaw_full = -data["vicon_yaw"] if mirror else data["vicon_yaw"]
    ax.plot(t, real_yaw_full, label="Real yaw (unwrapped)", linewidth=1)
    for w in selected:
        ax.plot(t[w.t0_idx], w.init_state[4], "o", color="orange", markersize=4)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Yaw (rad)")
    ax.set_title("Heading (orange dots = init yaw)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    plt.close(fig)


def _debug_to_loss_sim(traces: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Convert `Rollout.run_debug` output to the channel dict `window_loss` consumes."""
    return {
        "yaw_rate": traces["ang_vel_z"],
        "v_y": traces["linear_vel_y"],
        "a_x": traces["a_x"],
        "v_x": traces["linear_vel_x"],
        "omega": 0.5 * (traces["omega_front"] + traces["omega_rear"]),
        "pose": np.stack([traces["pose_x"], traces["pose_y"]], axis=-1),
        "yaw": traces["pose_theta"],
        "beta": traces["beta"],
    }


def _check_yaw_continuity(
    originals: list[Window],
    sim_traces: list[dict[str, np.ndarray]],
    *,
    step_thr: float = np.pi / 2,
) -> list[tuple[int, float, float]]:
    """Per-window max |Δyaw| must stay below step_thr in both sim and real;
    otherwise an upstream wrap was missed and the overlay plot would mislead
    (loss math is still correct via the wrapped residual).
    """
    failures: list[tuple[int, float, float]] = []
    for w, tr in zip(originals, sim_traces):
        max_sim = float(np.max(np.abs(np.diff(tr["pose_theta"]))))
        max_real = float(np.max(np.abs(np.diff(w.real_yaw))))
        if max_sim > step_thr or max_real > step_thr:
            failures.append((int(w.t0_idx), max_sim, max_real))
    return failures


def _format_continuity_lines(failures: list[tuple[int, float, float]]) -> list[str]:
    if not failures:
        return ["yaw_continuity: PASS"]
    lines = [
        f"WARN: yaw continuity check failed for {len(failures)} window(s) "
        "(per-step Δyaw > π/2; overlay plot may mislead, loss math still correct):"
    ]
    for t0_idx, max_sim, max_real in failures:
        lines.append(f"  t0_idx={t0_idx} max_sim_step={max_sim:.4f} max_real_step={max_real:.4f}")
    return lines


def _plot_rollout_overlay(
    dataset: Dataset,
    originals: list[Window],
    sim_traces: list[dict],
    npz_path: str,
    out_path: str,
    warmup_s: float = 0.2,
) -> None:
    """Overlay precomputed sim rollouts (one per non-mirrored window) onto the real bag signals."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    R_w = SYSID_PARAMS["R_w"]

    data = np.load(npz_path)
    t = data["t"]
    vicon_x = data["vicon_x"]
    vicon_y = data["vicon_y"]
    vicon_body_vx = data["vicon_body_vx"]
    vicon_body_vy = data["vicon_body_vy"]
    vicon_r = data["vicon_r"]
    cmd_steer = data["cmd_steer"]
    rs_core_speed = data["rs_core_speed"]
    real_omega = rs_core_speed / R_w
    real_a_x = np.gradient(savgol_filter(vicon_body_vx, 21, 2), dataset.dt)

    n_steps = len(originals[0].cmd_steer) if originals else 0

    fig, grid = plt.subplots(2, 5, figsize=(40, 14))
    fig.suptitle(
        f"Rollout overlay — {os.path.basename(npz_path)} ({len(originals)} windows)",
        fontsize=14,
    )
    axes = [
        grid[0, 0],  # XY
        grid[0, 1],  # v_x
        grid[0, 2],  # v_y
        grid[1, 0],  # steering
        grid[1, 1],  # yaw_rate
        grid[1, 2],  # a_x
        grid[1, 3],  # beta
        grid[0, 3],  # omega
        grid[0, 4],  # heading overlay (sim − init vs real − init)
        grid[1, 4],  # heading wrapped residual
    ]

    def shade_warmup(ax):
        for w in originals:
            t0 = t[w.t0_idx]
            ax.axvspan(t0, t0 + warmup_s, color="red", alpha=0.10, linewidth=0)

    def window_t(w):
        return t[w.t0_idx] + np.arange(n_steps + 1) * dataset.dt

    sim_label_added = False
    ax = axes[0]
    ax.plot(vicon_x, vicon_y, label="Real (Vicon)", linewidth=1.0, color="C0")
    for w, tr in zip(originals, sim_traces):
        lbl = "Sim" if not sim_label_added else None
        ax.plot(tr["pose_x"], tr["pose_y"], "--", color="C1", linewidth=1.2, label=lbl)
        ax.plot(w.init_state[0], w.init_state[1], "o", color="C1", markersize=4)
        sim_label_added = True
    ax.plot(vicon_x[0], vicon_y[0], "ko", markersize=6, label="Bag start")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("XY (sim segments dashed; orange dot = window init)")
    ax.set_aspect("equal")
    ax.legend()
    ax.grid(True, alpha=0.3)

    def overlay_channel(ax, real_full, sim_key, title, ylabel):
        ax.plot(t, real_full, label="Real", linewidth=1.0, color="C0")
        first = True
        for w, tr in zip(originals, sim_traces):
            tt = window_t(w)
            ax.plot(tt, tr[sim_key], "--", color="C1", linewidth=1.2, label="Sim" if first else None)
            ax.plot(tt[0], tr[sim_key][0], "o", color="C1", markersize=4)
            first = False
        shade_warmup(ax)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)

    overlay_channel(axes[1], vicon_body_vx, "linear_vel_x", "v_x — body frame", "m/s")
    overlay_channel(axes[2], vicon_body_vy, "linear_vel_y", "v_y — body frame", "m/s")
    overlay_channel(axes[4], vicon_r, "ang_vel_z", "Yaw rate", "rad/s")
    overlay_channel(axes[5], real_a_x, "a_x", "Longitudinal accel (a_x)", "m/s²")

    ax = axes[3]
    ax.plot(t, cmd_steer, label="cmd_steer", linewidth=1.0, color="C0", alpha=0.7)
    first = True
    for w, tr in zip(originals, sim_traces):
        tt = window_t(w)
        ax.plot(tt, tr["delta"], "--", color="C1", linewidth=1.2, label="Sim δ" if first else None)
        first = False
    shade_warmup(ax)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Steering (rad)")
    ax.set_title("Steering — cmd vs sim δ (red = warmup discard)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[6]
    real_speed = np.hypot(vicon_body_vx, vicon_body_vy)
    real_beta = np.where(real_speed > 0.2, np.arctan2(vicon_body_vy, vicon_body_vx), np.nan)
    ax.plot(t, real_beta, label="Real β", linewidth=1.0, color="C0")
    first = True
    for w, tr in zip(originals, sim_traces):
        tt = window_t(w)
        ax.plot(tt, tr["beta"], "--", color="C1", linewidth=1.2, label="Sim β" if first else None)
        first = False
    shade_warmup(ax)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Slip angle β (rad)")
    ax.set_title("Slip angle β")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[7]
    ax.plot(t, real_omega, label=f"VESC ω (rs_core_speed/R_w, R_w={R_w:.4f})", linewidth=1.0, color="C0")
    first = True
    for w, tr in zip(originals, sim_traces):
        tt = window_t(w)
        ax.plot(tt, tr["omega_front"], "--", color="C1", linewidth=1.2, label="Sim ω_front" if first else None)
        ax.plot(tt, tr["omega_rear"], "--", color="C2", linewidth=1.2, label="Sim ω_rear" if first else None)
        ax.plot(tt[0], tr["omega_front"][0], "o", color="C1", markersize=4)
        first = False
    shade_warmup(ax)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular velocity (rad/s)")
    ax.set_title("Wheel ω — VESC vs Sim (init ω_f = ω_r seeded from VESC, AWD)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Heading overlay — subtract init_state[4] so all windows share one y-axis.
    ax = axes[8]
    first = True
    for w, tr in zip(originals, sim_traces):
        tt = window_t(w)
        init_yaw = w.init_state[4]
        ax.plot(
            tt,
            w.real_yaw - init_yaw,
            color="C0",
            linewidth=1.0,
            label="Real Δyaw" if first else None,
        )
        ax.plot(
            tt,
            tr["pose_theta"] - init_yaw,
            "--",
            color="C1",
            linewidth=1.2,
            label="Sim Δyaw" if first else None,
        )
        first = False
    shade_warmup(ax)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Δyaw vs init (rad)")
    ax.set_title("Heading — sim vs real (each window centered at init yaw)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Wrapped heading residual — exactly what the loss squares.
    ax = axes[9]
    first = True
    for w, tr in zip(originals, sim_traces):
        tt = window_t(w)
        diff = tr["pose_theta"] - w.real_yaw
        wrapped = np.arctan2(np.sin(diff), np.cos(diff))
        ax.plot(
            tt,
            wrapped,
            color="C3",
            linewidth=1.0,
            label="wrap(sim − real)" if first else None,
        )
        first = False
    ax.axhline(0.0, color="gray", linewidth=0.5, alpha=0.5)
    shade_warmup(ax)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Wrapped residual (rad)")
    ax.set_title("Heading wrapped residual (loss target; jumps = wrap bug)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    plt.close(fig)


def _format_channel_table(per_channel: dict[str, float]) -> list[str]:
    """Per-channel breakdown as a 3-column table: contribution | raw MSE | coeff.

    `contribution = coeff · raw`, so raw is recovered by division. When
    `coeff == 0` the channel is plumbed but disabled — raw is not knowable
    from contribution alone, so it's reported as `n/a`.
    """
    header = f"{'channel':<10} {'contribution':>14} {'raw_MSE':>14} {'coeff':>10}"
    rule = "-" * len(header)
    lines = [header, rule]
    for ch in CHANNELS:
        contrib = per_channel.get(ch, float("nan"))
        coeff = CHANNEL_COEFFS[ch]
        raw_str = f"{contrib / coeff:>14.6f}" if coeff != 0.0 else f"{'n/a':>14}"
        lines.append(f"{ch:<10} {contrib:>14.6f} {raw_str} {coeff:>10.4f}")
    return lines


def _write_metrics(
    out_path: str,
    *,
    bag: Path,
    params_source: str,
    args: argparse.Namespace,
    dataset: Dataset,
    total_loss: float,
    per_channel: dict[str, float],
    continuity_failures: list[tuple[int, float, float]],
) -> None:
    n_originals = sum(not w.is_mirrored for w in dataset.windows)
    n_mirrored = len(dataset.windows) - n_originals
    lines = [
        f"bag: {bag}",
        f"params: {params_source}",
        f"window_length_s: {args.window_length_s}",
        f"stride_s: {args.stride_s}",
        f"min_speed: {args.min_speed}",
        f"warmup_s: {args.warmup_s}",
        f"mirror: {str(args.mirror).lower()}",
        "",
        f"candidate_windows: {dataset.n_candidates}",
        f"windows_total: {len(dataset.windows)}",
        f"windows_originals: {n_originals}",
        f"windows_mirrored: {n_mirrored}",
        f"dropped_low_speed: {dataset.n_dropped_low_speed}",
        f"dropped_nonfinite: {dataset.n_dropped_nonfinite}",
        "",
        f"total_loss: {total_loss:.6f}",
        "",
        *_format_channel_table(per_channel),
        "",
        *_format_continuity_lines(continuity_failures),
    ]
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")


def _load_params(path: str | None) -> tuple[dict, str]:
    if path is None:
        return dict(SYSID_PARAMS), "SYSID_PARAMS (env.py)"
    with open(path, "r") as f:
        return yaml.safe_load(f), str(Path(path).resolve())


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="python -m examples.analysis.sysid.validate")
    p.add_argument("--bag", required=True, help="Path to 100Hz NPZ bag")
    p.add_argument("--params", default=None, help="YAML param file (default: SYSID_PARAMS)")
    p.add_argument("--window-length-s", type=float, default=1.5)
    p.add_argument("--stride-s", type=float, default=0.5)
    p.add_argument("--min-speed", type=float, default=0.3)
    p.add_argument("--warmup-s", type=float, default=0.2)
    p.add_argument(
        "--mirror",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Include mirrored windows (default: False, matches study.py)",
    )
    p.add_argument("--out-dir", default=None, help="Default: figures/analysis/sysid/<bag_stem>/")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    bag = Path(args.bag).resolve()
    if not bag.exists():
        raise SystemExit(f"Bag not found: {bag}")

    params, params_source = _load_params(args.params)
    out_dir = Path(args.out_dir).resolve() if args.out_dir else Path("figures") / "analysis" / "sysid" / bag.stem
    out_dir.mkdir(parents=True, exist_ok=True)

    dataset = load_dataset(
        str(bag),
        window_length_s=args.window_length_s,
        stride_s=args.stride_s,
        min_speed=args.min_speed,
        mirror=args.mirror,
    )
    n_originals = sum(not w.is_mirrored for w in dataset.windows)
    n_mirrored = len(dataset.windows) - n_originals
    print(
        f"loaded {len(dataset.windows)} windows from {bag} "
        f"(originals={n_originals}, mirrored={n_mirrored}, "
        f"dropped_low_speed={dataset.n_dropped_low_speed}, "
        f"dropped_nonfinite={dataset.n_dropped_nonfinite}, "
        f"candidates={dataset.n_candidates})"
    )

    _plot_dataset_overview(dataset, str(bag), str(out_dir / "dataset_overview.png"), mirror=False)
    print(f"saved {out_dir / 'dataset_overview.png'}")
    if args.mirror:
        _plot_dataset_overview(dataset, str(bag), str(out_dir / "dataset_overview_mirror.png"), mirror=True)
        print(f"saved {out_dir / 'dataset_overview_mirror.png'}")

    originals = [w for w in dataset.windows if not w.is_mirrored]
    warmup_steps = int(round(args.warmup_s / dataset.dt))
    min_window_len = min(len(w.real_v_x) for w in dataset.windows)
    if warmup_steps >= min_window_len:
        raise SystemExit(
            f"--warmup-s={args.warmup_s} ({warmup_steps} steps at dt={dataset.dt}) "
            f"leaves no scored samples (shortest window has {min_window_len} samples)"
        )
    totals: list[float] = []
    accum: dict[str, list[float]] = {ch: [] for ch in CHANNELS}

    with Rollout(params=params) as rollout:
        # One sim per original window — traces feed both the overlay and the loss.
        # Mirrored windows still need their own pass through `rollout.run`.
        sim_traces = [rollout.run_debug(w) for w in originals]
        _plot_rollout_overlay(
            dataset, originals, sim_traces, str(bag), str(out_dir / "rollout_overlay.png"), warmup_s=args.warmup_s
        )
        print(f"saved {out_dir / 'rollout_overlay.png'}")

        continuity_failures = _check_yaw_continuity(originals, sim_traces)
        for line in _format_continuity_lines(continuity_failures):
            print(line)

        for w, tr in zip(originals, sim_traces):
            total, per_ch = window_loss(_debug_to_loss_sim(tr), w, CHANNEL_COEFFS, warmup_steps)
            totals.append(total)
            for ch in CHANNELS:
                accum[ch].append(per_ch[ch])
        for w in dataset.windows:
            if not w.is_mirrored:
                continue
            total, per_ch = window_loss(rollout.run(w), w, CHANNEL_COEFFS, warmup_steps)
            totals.append(total)
            for ch in CHANNELS:
                accum[ch].append(per_ch[ch])

    total_loss = float(np.mean(totals))
    per_channel = {ch: float(np.mean(accum[ch])) for ch in CHANNELS}

    print(f"\ntotal_loss = {total_loss:.6f}")
    print()
    for line in _format_channel_table(per_channel):
        print(line)

    metrics_path = out_dir / "metrics.txt"
    _write_metrics(
        str(metrics_path),
        bag=bag,
        params_source=params_source,
        args=args,
        dataset=dataset,
        total_loss=total_loss,
        per_channel=per_channel,
        continuity_failures=continuity_failures,
    )
    print(f"saved {metrics_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
