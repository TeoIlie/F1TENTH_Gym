"""STD system identification rollout helper.

Owns a single GKEnv instance configured for sim-vs-real replay and exposes a
`run(window) -> dict[str, np.ndarray]` interface matching the loss-module
contract. Build once per worker; call `set_params(params)` between Optuna
trials to hot-swap PAC2002 coefficients without rebuilding the env.

See docs/plan/OPTUNA_SYS_ID.md for the locked design decisions.
"""

from __future__ import annotations

from typing import Callable

import numpy as np

from examples.analysis.sysid.dataset import Window
from examples.analysis.sysid.env import SYSID_PARAMS
from gymkhana.envs.gymkhana_env import GKEnv
from train.config.env_config import get_drift_train_config

_SYSID_OVERRIDES: dict = {
    "num_agents": 1,
    "model": "std",
    "control_input": ["speed", "steering_angle"],
    "observation_config": {"type": "dynamic_state"},
    "normalize_obs": False,
    "normalize_act": False,
    "prevent_instability": False,
    "track_direction": "normal",
    "map": "Spielberg_blank",
    "render_track_lines": False,
    "render_arc_length_annotations": False,
    "render_lookahead_curvatures": False,
    "debug_frenet_projection": False,
    "record_obs_min_max": False,
}


def _build_sysid_config(params: dict | None) -> dict:
    config = get_drift_train_config()
    config.update(_SYSID_OVERRIDES)
    # Default to SYSID_PARAMS so the env's R_w matches what dataset.py
    # divided VESC rs_core_speed by when seeding init_state[7:9].
    config["params"] = params if params is not None else SYSID_PARAMS
    return config


class Rollout:
    """Replays a `Window` through a single, reusable GKEnv instance.

    Construct once per worker process; call `set_params(params)` between
    trials to hot-swap PAC2002 coefficients without rebuilding the env.
    Supports use as a context manager to guarantee `close()`.
    """

    def __init__(self, params: dict | None = None):
        config = _build_sysid_config(params)
        self._env = GKEnv(config=config)
        self._dt = float(self._env.timestep)
        self._agent_id = self._env.agent_ids[0]

    @property
    def dt(self) -> float:
        return self._dt

    def set_params(self, params: dict) -> None:
        self._env.configure({"params": params})

    def close(self) -> None:
        self._env.close()

    def __enter__(self) -> Rollout:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def run(self, window: Window) -> dict[str, np.ndarray]:
        assert window.init_state.shape == (9,), (
            f"Expected 9-wide init_state (STD with VESC-seeded omegas); got {window.init_state.shape}"
        )
        n = len(window.cmd_steer)

        v_x = np.empty(n + 1, dtype=np.float64)
        v_y = np.empty(n + 1, dtype=np.float64)
        yaw_rate = np.empty(n + 1, dtype=np.float64)

        # `dynamic_state` obs surfaces body-frame v_x, v_y, yaw_rate from
        # agent.standard_state — same quantities the bag was built from.
        obs, _ = self._env.reset(options={"states": window.init_state.reshape(1, 9)})
        agent_obs = obs[self._agent_id]
        v_x[0] = agent_obs["linear_vel_x"]
        v_y[0] = agent_obs["linear_vel_y"]
        yaw_rate[0] = agent_obs["ang_vel_z"]

        for k in range(n):
            action = np.array(
                [[window.cmd_steer[k], window.cmd_speed[k]]],
                dtype=np.float32,
            )
            obs, _, _, _, _ = self._env.step(action)
            agent_obs = obs[self._agent_id]
            v_x[k + 1] = agent_obs["linear_vel_x"]
            v_y[k + 1] = agent_obs["linear_vel_y"]
            yaw_rate[k + 1] = agent_obs["ang_vel_z"]

        a_x = np.gradient(v_x, self._dt)

        sim = {"yaw_rate": yaw_rate, "v_y": v_y, "a_x": a_x, "v_x": v_x}

        # Trial params can drive the integrator to NaN/inf. Surface this loudly
        # so the Optuna objective can map it to a prunable trial loss instead of
        # silently propagating non-finite values into the NMSE.
        for ch, arr in sim.items():
            if not np.all(np.isfinite(arr)):
                raise FloatingPointError(
                    f"Non-finite sim signal in channel {ch!r} on window t0_idx={window.t0_idx}; "
                    "trial params likely caused integrator divergence."
                )

        return sim

    def run_debug(self, window: Window) -> dict[str, np.ndarray]:
        """Reset + step like `run`, but return the full obs trace.

        For visual validation only. Not on the Optuna hot path. Returns
        body-frame velocities, world-frame pose, steering, slip, wheel
        angular velocities (omega_front, omega_rear, read from raw STD
        state[7:9]), plus the same finite-diff `a_x` the loss uses.
        """
        assert window.init_state.shape == (9,), (
            f"Expected 9-wide init_state (STD with VESC-seeded omegas); got {window.init_state.shape}"
        )
        n = len(window.cmd_steer)
        keys = ("pose_x", "pose_y", "pose_theta", "linear_vel_x", "linear_vel_y", "ang_vel_z", "delta", "beta")
        traces: dict[str, np.ndarray] = {k: np.empty(n + 1, dtype=np.float64) for k in keys}
        traces["omega_front"] = np.empty(n + 1, dtype=np.float64)
        traces["omega_rear"] = np.empty(n + 1, dtype=np.float64)
        agent = self._env.sim.agents[0]

        def record(idx: int, agent_obs: dict) -> None:
            for k in keys:
                traces[k][idx] = float(agent_obs[k])
            traces["omega_front"][idx] = float(agent.state[7])
            traces["omega_rear"][idx] = float(agent.state[8])

        obs, _ = self._env.reset(options={"states": window.init_state.reshape(1, 9)})
        record(0, obs[self._agent_id])

        for k in range(n):
            action = np.array([[window.cmd_steer[k], window.cmd_speed[k]]], dtype=np.float32)
            obs, _, _, _, _ = self._env.step(action)
            record(k + 1, obs[self._agent_id])

        traces["a_x"] = np.gradient(traces["linear_vel_x"], self._dt)
        return traces


def make_rollout_fn(params: dict) -> Callable[[Window], dict[str, np.ndarray]]:
    """Convenience: build a `Rollout` and return its `run` bound method.

    For Phase 1 tests and one-off baselines. Optuna study code should
    construct `Rollout` directly and call `set_params` between trials so
    a single env is reused per worker.
    """
    return Rollout(params=params).run


def _plot_rollout_overlay(
    dataset,  # examples.analysis.sysid.dataset.Dataset
    npz_path: str,
    out_path: str,
    warmup_s: float = 0.2,
) -> None:
    """Overlay sim rollouts (one per non-mirrored window) onto the real bag signals.

    Run with default STD params (no Optuna). Useful for confirming
    init_state reset, command replay, and signal extraction all work
    before launching any study.
    """
    import os

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from scipy.signal import savgol_filter

    from examples.analysis.sysid.env import SYSID_PARAMS

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

    originals = [w for w in dataset.windows if not w.is_mirrored]
    n_steps = len(originals[0].cmd_steer) if originals else 0

    # Run sim once per window with default params (Rollout's GKEnv is built
    # from get_drift_train_config()'s params).
    sim_traces: list[dict] = []
    with Rollout() as rollout:
        for w in originals:
            sim_traces.append(rollout.run_debug(w))

    fig, grid = plt.subplots(2, 4, figsize=(32, 14))
    fig.suptitle(
        f"Rollout overlay (default STD params) — {os.path.basename(npz_path)} ({len(originals)} windows)",
        fontsize=14,
    )
    axes = [
        grid[0, 0],
        grid[0, 1],
        grid[0, 2],
        grid[1, 0],
        grid[1, 1],
        grid[1, 2],
        grid[1, 3],
        grid[0, 3],  # wheel angular velocity overlay
    ]

    def shade_warmup(ax):
        for w in originals:
            t0 = t[w.t0_idx]
            ax.axvspan(t0, t0 + warmup_s, color="red", alpha=0.10, linewidth=0)

    def window_t(w):
        # Sim arrays span N+1 samples, real signals same span; aligned to t[w.t0_idx].
        return t[w.t0_idx] + np.arange(n_steps + 1) * dataset.dt

    sim_label_added = False

    # --- XY ---
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

    # --- Steering: cmd vs sim's actual delta ---
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

    # --- Slip β ---
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

    # --- Wheel angular velocity: VESC vs sim ω_f / ω_r ---
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

    fig.tight_layout()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    import argparse
    import os
    from pathlib import Path

    from examples.analysis.sysid.dataset import load_dataset

    parser = argparse.ArgumentParser(description="Visual validation of sysid rollout vs real bag")
    parser.add_argument("--path", required=True, help="Path to 100Hz NPZ bag")
    parser.add_argument("--window-length-s", type=float, default=1.5)
    parser.add_argument("--stride-s", type=float, default=0.5)
    parser.add_argument("--min-speed", type=float, default=0.3)
    args = parser.parse_args()

    ds = load_dataset(
        args.path,
        window_length_s=args.window_length_s,
        stride_s=args.stride_s,
        min_speed=args.min_speed,
        mirror=False,
    )
    print(f"windows: {len(ds.windows)}")

    stem = Path(args.path).stem
    out_dir = os.path.join("figures", "analysis", "sysid", stem)
    out_path = os.path.join(out_dir, "rollout_overlay.png")
    _plot_rollout_overlay(ds, args.path, out_path)
    print(f"plot saved to {out_path}")
