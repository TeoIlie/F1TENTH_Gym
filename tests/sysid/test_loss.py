"""Unit tests for examples.analysis.sysid.loss.

Tests are pure: synthetic Window/Dataset objects and synthetic sim dicts —
no env, no rollout. Rollout integration is verified separately.

Coverage:
  - channel_loss: identity, known-value correctness.
  - window_loss: identity, warmup slicing, weighted-sum aggregation,
    per-channel breakdown completeness.
  - dataset_loss: identity, warmup_s→steps conversion, mean across windows,
    mirror invariance under sign-symmetric sim signals.
"""

from __future__ import annotations

import numpy as np
import pytest

from examples.analysis.sysid.dataset import CHANNELS, Dataset, Window, mirror_window
from examples.analysis.sysid.loss import (
    CHANNEL_COEFFS,
    channel_loss,
    dataset_loss,
    window_loss,
)

DT = 0.01
N = 150  # cmd length; real signals are N+1 = 151
WARMUP_S = 0.2
WARMUP_STEPS = int(round(WARMUP_S / DT))  # 20


def _make_window(rng: np.random.Generator, t0_idx: int = 0) -> Window:
    return Window(
        t0_idx=t0_idx,
        init_state=np.array(
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 20.0, 20.0]
        ),  # omega values arbitrary; loss tests don't read init_state
        cmd_steer=rng.normal(0, 0.1, N),
        cmd_speed=rng.normal(2.0, 0.2, N),
        real_v_x=rng.normal(2.0, 0.5, N + 1),
        real_v_y=rng.normal(0.0, 0.3, N + 1),
        real_yaw_rate=rng.normal(0.0, 0.5, N + 1),
        real_a_x=rng.normal(0.0, 1.0, N + 1),
        real_omega=rng.normal(20.0, 2.0, N + 1),
        is_mirrored=False,
    )


def _sim_from_real(window: Window) -> dict[str, np.ndarray]:
    return {
        "yaw_rate": window.real_yaw_rate.copy(),
        "v_y": window.real_v_y.copy(),
        "a_x": window.real_a_x.copy(),
        "v_x": window.real_v_x.copy(),
        "omega": window.real_omega.copy(),
    }


# ---------- channel_loss ----------


def test_channel_loss_identity_is_zero():
    rng = np.random.default_rng(0)
    real = rng.normal(0, 1, 100)
    assert channel_loss(real, real, coeff=2.5) == 0.0


def test_channel_loss_known_value():
    sim = np.array([1.0, 2.0, 3.0])
    real = np.array([1.0, 2.0, 4.0])  # diff = [0, 0, 1] → MSE = 1/3
    coeff = 0.6
    expected = coeff * (1.0 / 3.0)
    assert channel_loss(sim, real, coeff) == pytest.approx(expected)


def test_channel_loss_zero_coeff_returns_zero():
    sim = np.array([1.0, 2.0, 3.0])
    real = np.array([4.0, 5.0, 6.0])
    assert channel_loss(sim, real, coeff=0.0) == 0.0


# ---------- window_loss ----------


def test_window_loss_identity():
    rng = np.random.default_rng(1)
    w = _make_window(rng)
    sim = _sim_from_real(w)
    total, per_channel = window_loss(sim, w, CHANNEL_COEFFS, WARMUP_STEPS)
    assert total == pytest.approx(0.0, abs=1e-12)
    for ch in CHANNELS:
        assert per_channel[ch] == pytest.approx(0.0, abs=1e-12)


def test_window_loss_warmup_region_is_ignored():
    # Inject huge errors inside the warmup region only — loss must be unchanged.
    rng = np.random.default_rng(2)
    w = _make_window(rng)

    sim_clean = _sim_from_real(w)
    total_clean, _ = window_loss(sim_clean, w, CHANNEL_COEFFS, WARMUP_STEPS)

    sim_dirty = _sim_from_real(w)
    for ch in CHANNELS:
        sim_dirty[ch][:WARMUP_STEPS] += 1e6  # huge perturbation inside warmup
    total_dirty, per_channel_dirty = window_loss(sim_dirty, w, CHANNEL_COEFFS, WARMUP_STEPS)

    assert total_dirty == pytest.approx(total_clean)
    for ch in CHANNELS:
        assert per_channel_dirty[ch] == pytest.approx(0.0, abs=1e-12)


def test_window_loss_post_warmup_perturbation_counted():
    # Inject a constant offset just past the warmup boundary on yaw_rate only.
    rng = np.random.default_rng(3)
    w = _make_window(rng)
    sim = _sim_from_real(w)
    sim["yaw_rate"][WARMUP_STEPS:] += 0.5  # constant offset → MSE = 0.25

    total, per_channel = window_loss(sim, w, CHANNEL_COEFFS, WARMUP_STEPS)

    expected_yaw_contrib = CHANNEL_COEFFS["yaw_rate"] * (0.5**2)
    assert per_channel["yaw_rate"] == pytest.approx(expected_yaw_contrib, rel=1e-9)
    for ch in ("v_y", "a_x", "v_x", "omega"):
        assert per_channel[ch] == pytest.approx(0.0, abs=1e-12)
    assert total == pytest.approx(expected_yaw_contrib, rel=1e-9)


def test_window_loss_total_is_sum_of_per_channel_contribs():
    # Hand-craft per-channel offsets and verify total = sum of contribs.
    # (Note: with the new API, per_channel already contains coeff * MSE,
    # so total == sum(per_channel.values()) — no extra coeff multiplication.)
    rng = np.random.default_rng(4)
    w = _make_window(rng)
    sim = _sim_from_real(w)
    offsets = {"yaw_rate": 0.1, "v_y": 0.2, "a_x": 0.3, "v_x": 0.4, "omega": 1.0}
    for ch, off in offsets.items():
        sim[ch][WARMUP_STEPS:] += off

    total, per_channel = window_loss(sim, w, CHANNEL_COEFFS, WARMUP_STEPS)
    expected_total = sum(per_channel[ch] for ch in CHANNELS)
    assert total == pytest.approx(expected_total, rel=1e-12)
    # And each contrib should match coeff * offset^2.
    for ch, off in offsets.items():
        assert per_channel[ch] == pytest.approx(CHANNEL_COEFFS[ch] * (off**2), rel=1e-9)


def test_window_loss_per_channel_keys_complete():
    rng = np.random.default_rng(5)
    w = _make_window(rng)
    _, per_channel = window_loss(_sim_from_real(w), w, CHANNEL_COEFFS, WARMUP_STEPS)
    assert set(per_channel.keys()) == set(CHANNELS)


# ---------- dataset_loss ----------


def _make_dataset(windows: list[Window]) -> Dataset:
    return Dataset(
        windows=windows,
        dt=DT,
        n_candidates=len(windows),
        n_dropped_low_speed=0,
        n_dropped_nonfinite=0,
    )


def test_dataset_loss_identity():
    rng = np.random.default_rng(6)
    windows = [_make_window(rng, t0_idx=i * 10) for i in range(3)]
    ds = _make_dataset(windows)
    total, per_channel = dataset_loss(_sim_from_real, ds, CHANNEL_COEFFS, warmup_s=WARMUP_S)
    assert total == pytest.approx(0.0, abs=1e-12)
    for ch in CHANNELS:
        assert per_channel[ch] == pytest.approx(0.0, abs=1e-12)


def test_dataset_loss_warmup_seconds_to_steps_conversion():
    # Set warmup_s so all-but-one sample is dropped; perturbing the dropped region
    # must be invisible. dt=0.01, N+1=151 → warmup_s=1.50 drops 150 samples, scoring 1.
    rng = np.random.default_rng(7)
    w = _make_window(rng)
    ds = _make_dataset([w])

    def rollout_perturb_early(window: Window) -> dict[str, np.ndarray]:
        sim = _sim_from_real(window)
        for ch in CHANNELS:
            sim[ch][:150] += 1e6  # only the last sample is scored
        return sim

    total, per_channel = dataset_loss(rollout_perturb_early, ds, CHANNEL_COEFFS, warmup_s=1.50)
    assert total == pytest.approx(0.0, abs=1e-12)
    for ch in CHANNELS:
        assert per_channel[ch] == pytest.approx(0.0, abs=1e-12)


def test_dataset_loss_aggregates_as_arithmetic_mean():
    # Two windows with deliberately different per-window losses; verify mean.
    rng = np.random.default_rng(8)
    w0 = _make_window(rng, t0_idx=0)
    w1 = _make_window(rng, t0_idx=10)
    ds = _make_dataset([w0, w1])

    # Window-specific perturbation: w0 gets +0.1 on yaw_rate, w1 gets +0.3.
    def rollout(window: Window) -> dict[str, np.ndarray]:
        sim = _sim_from_real(window)
        offset = 0.1 if window.t0_idx == 0 else 0.3
        sim["yaw_rate"][WARMUP_STEPS:] += offset
        return sim

    total, per_channel = dataset_loss(rollout, ds, CHANNEL_COEFFS, warmup_s=WARMUP_S)

    contrib0 = CHANNEL_COEFFS["yaw_rate"] * (0.1**2)
    contrib1 = CHANNEL_COEFFS["yaw_rate"] * (0.3**2)
    expected_mean = 0.5 * (contrib0 + contrib1)
    assert per_channel["yaw_rate"] == pytest.approx(expected_mean, rel=1e-9)
    for ch in ("v_y", "a_x", "v_x", "omega"):
        assert per_channel[ch] == pytest.approx(0.0, abs=1e-12)
    assert total == pytest.approx(expected_mean, rel=1e-9)


def test_dataset_loss_mirror_invariance_under_symmetric_sim():
    """Mirroring the real signals and the sim consistently must produce identical loss.

    Catches sign-handling bugs in any future per-channel sign logic. With the
    current implementation this is a near-tautology since (sim - real)^2 is
    sign-invariant — but it pins the contract.
    """
    rng = np.random.default_rng(9)
    w = _make_window(rng)
    w_mirr = mirror_window(w)
    ds_orig = _make_dataset([w])
    ds_mirr = _make_dataset([w_mirr])

    def rollout(window: Window) -> dict[str, np.ndarray]:
        return _sim_from_real(window)

    total_orig, per_orig = dataset_loss(rollout, ds_orig, CHANNEL_COEFFS, warmup_s=WARMUP_S)
    total_mirr, per_mirr = dataset_loss(rollout, ds_mirr, CHANNEL_COEFFS, warmup_s=WARMUP_S)

    assert total_orig == pytest.approx(total_mirr, abs=1e-12)
    for ch in CHANNELS:
        assert per_orig[ch] == pytest.approx(per_mirr[ch], abs=1e-12)
