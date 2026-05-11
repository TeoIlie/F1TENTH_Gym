"""STD system identification loss function.

Pure functions for scoring sim-vs-real trajectories. The Optuna objective
will call `dataset_loss(rollout_fn, dataset)` with a closure capturing the
trial's parameter dict, keeping this module independent of Optuna and the env.

Sim signals are passed as `dict[str, np.ndarray]` keyed by CHANNELS (from
dataset.py). Each array has shape (N+1,) matching Window.real_*.

The per-channel loss is fixed-scale weighted MSE:

    L = sum_c  CHANNEL_COEFFS[c] * mean((sim_c - real_c)**2)

Each coefficient bundles "how much do I care about this channel" with a
unit-scaling factor, so contributions land on comparable, unitless magnitudes
without depending on per-dataset variance (which collapses on steady-state bags).

Most channels are 1-D arrays of shape (N+1,). The `pose` channel is the
exception: sim["pose"] and Window.real_pose are both (N+1, 2) (column 0 = x,
column 1 = y). `np.mean((sim - real)**2)` then averages over all 2(N+1)
elements, i.e. yields ½ · mean(Δx² + Δy²) — half the textbook squared
Euclidean distance per timestep. The factor of ½ is silently absorbed into
CHANNEL_COEFFS["pose"] (which is hand-tuned anyway), so no special-case
branch is needed in window_loss.

See docs/plan/OPTUNA_SYS_ID.md for the locked design decisions.
"""

from __future__ import annotations

from typing import Callable

import numpy as np

from examples.analysis.sysid.dataset import CHANNELS, Dataset, Window

# Per-channel coefficients combining importance and unit-scaling. Tuned so each
# contribution lands on a comparable, unitless magnitude given typical drift-regime
# signal scales (yaw_rate ~1 rad/s, v_y ~0.5 m/s, a_x ~5 m/s², v_x ~5 m/s).
# `pose` is a 2-D channel: contribution is coeff * ½ · mean(Δx² + Δy²); the
# factor of ½ is absorbed into the coefficient. Tune via validate.py before
# launching a study.
CHANNEL_COEFFS: dict[str, float] = {
    "yaw_rate": 0.3,
    "v_y": 0.5,
    "a_x": 0.04,
    "v_x": 2.0,
    "omega": 0.005,
    "pose": 5.0,
    "yaw": 1.0,  # TODO_TUNE via validate.py
    "beta": 1.0,  # TODO_TUNE via validate.py
}

# Channels whose residuals must be wrapped to (−π, π] before squaring,
# otherwise (s − r)² is wrong near the ±π wrap.
_ANGULAR_CHANNELS: frozenset[str] = frozenset({"yaw", "beta"})


def channel_loss(sim: np.ndarray, real: np.ndarray, coeff: float) -> float:
    # `np.mean` with no axis is intentional: collapses over all dims so this
    # function works uniformly for 1-D channels (yaw_rate, v_y, ...) and the
    # 2-D `pose` channel (shape (N+1, 2)). Don't add `axis=0` here.
    return coeff * float(np.mean((sim - real) ** 2))


def window_loss(
    sim: dict[str, np.ndarray],
    window: Window,
    coeffs: dict[str, float],
    warmup_steps: int,
) -> tuple[float, dict[str, float]]:
    per_channel: dict[str, float] = {}
    weighted_total = 0.0
    for ch in CHANNELS:
        real_full = getattr(window, f"real_{ch}")
        sim_full = sim[ch]
        assert sim_full.shape == real_full.shape, (
            f"sim[{ch!r}] shape {sim_full.shape} does not match real shape {real_full.shape}"
        )
        sim_w = sim_full[warmup_steps:]
        real_w = real_full[warmup_steps:]
        if ch in _ANGULAR_CHANNELS:
            diff = np.arctan2(np.sin(sim_w - real_w), np.cos(sim_w - real_w))
            contrib = coeffs[ch] * float(np.mean(diff**2))
        else:
            contrib = channel_loss(sim_w, real_w, coeffs[ch])
        per_channel[ch] = contrib
        weighted_total += contrib
    return weighted_total, per_channel


def dataset_loss(
    rollout_fn: Callable[[Window], dict[str, np.ndarray]],
    dataset: Dataset,
    coeffs: dict[str, float] = CHANNEL_COEFFS,
    warmup_s: float = 0.2,
) -> tuple[float, dict[str, float]]:
    warmup_steps = int(round(warmup_s / dataset.dt))
    signal_len = len(dataset.windows[0].real_v_x)
    if warmup_steps >= signal_len:
        raise ValueError(
            f"warmup_s={warmup_s} (warmup_steps={warmup_steps}) leaves no samples to score "
            f"in windows of length {signal_len}"
        )

    totals: list[float] = []
    per_channel_accum: dict[str, list[float]] = {ch: [] for ch in CHANNELS}

    for window in dataset.windows:
        sim = rollout_fn(window)
        total, per_channel = window_loss(sim, window, coeffs, warmup_steps)
        totals.append(total)
        for ch in CHANNELS:
            per_channel_accum[ch].append(per_channel[ch])

    mean_total = float(np.mean(totals))
    mean_per_channel = {ch: float(np.mean(per_channel_accum[ch])) for ch in CHANNELS}
    return mean_total, mean_per_channel
