"""STD system identification loss function.

Pure functions for scoring sim-vs-real trajectories. The Optuna objective
will call `dataset_loss(rollout_fn, dataset)` with a closure capturing the
trial's parameter dict, keeping this module independent of Optuna and the env.

Sim signals are passed as `dict[str, np.ndarray]` keyed by CHANNELS (from
dataset.py). Each array has shape (N+1,) matching Window.real_*.

See docs/plan/OPTUNA_SYS_ID.md for the locked design decisions.
"""

from __future__ import annotations

from typing import Callable

import numpy as np

from examples.analysis.sysid.dataset import CHANNELS, Dataset, Window

DEFAULT_WEIGHTS: dict[str, float] = {"yaw_rate": 3.0, "v_y": 2.0, "a_x": 1.0, "v_x": 0.5}

_VAR_EPS = 1e-9


def channel_nmse(sim: np.ndarray, real: np.ndarray, variance: float) -> float:
    return float(np.mean((sim - real) ** 2) / (variance + _VAR_EPS))


def window_loss(
    sim: dict[str, np.ndarray],
    window: Window,
    variances: dict[str, float],
    weights: dict[str, float],
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
        nmse = channel_nmse(sim_full[warmup_steps:], real_full[warmup_steps:], variances[ch])
        per_channel[ch] = nmse
        weighted_total += weights[ch] * nmse
    return weighted_total, per_channel


def dataset_loss(
    rollout_fn: Callable[[Window], dict[str, np.ndarray]],
    dataset: Dataset,
    weights: dict[str, float] = DEFAULT_WEIGHTS,
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
        total, per_channel = window_loss(sim, window, dataset.variances, weights, warmup_steps)
        totals.append(total)
        for ch in CHANNELS:
            per_channel_accum[ch].append(per_channel[ch])

    mean_total = float(np.mean(totals))
    mean_per_channel = {ch: float(np.mean(per_channel_accum[ch])) for ch in CHANNELS}
    return mean_total, mean_per_channel
