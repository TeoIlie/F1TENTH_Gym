"""Unit tests for examples.analysis.sysid.dataset.

Covers window construction, filtering (low-speed mask, NaN guard), variance
computation, and the mirror transform. Uses synthetic NPZ bags so tests run
fast and have known ground truth.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest
from scipy.signal import savgol_filter

from examples.analysis.sysid.dataset import (
    load_dataset,
    mirror_window,
)
from examples.analysis.sysid.env import SYSID_PARAMS

R_W = SYSID_PARAMS["R_w"]

DT = 0.01
WINDOW_S = 1.5
STRIDE_S = 0.5
N_STEPS = int(round(WINDOW_S / DT))  # 150
STRIDE = int(round(STRIDE_S / DT))  # 50


def _make_npz(
    path,
    n: int = 500,
    speed=1.0,
    vy=0.0,
    yaw_rate=0.0,
    steer=0.1,
    yaw=0.0,
    rs_core_speed=None,
    include_rs_core_speed: bool = True,
):
    """Write a synthetic 100Hz NPZ. Scalar args are broadcast to length-n arrays.

    `rs_core_speed` defaults to `vx_arr` (no-slip), keeping legacy tests
    semantically equivalent to the prior no-slip-derived omega seeding.
    Set `include_rs_core_speed=False` to omit the field entirely.
    """
    t = np.arange(n) * DT
    vx_arr = np.broadcast_to(np.asarray(speed, dtype=float), (n,)).copy()
    vy_arr = np.broadcast_to(np.asarray(vy, dtype=float), (n,)).copy()
    yaw_rate_arr = np.broadcast_to(np.asarray(yaw_rate, dtype=float), (n,)).copy()
    steer_arr = np.broadcast_to(np.asarray(steer, dtype=float), (n,)).copy()
    yaw_arr = np.broadcast_to(np.asarray(yaw, dtype=float), (n,)).copy()
    x = np.cumsum(vx_arr) * DT
    y = np.cumsum(vy_arr) * DT
    fields = dict(
        t=t,
        cmd_speed=vx_arr.copy(),
        cmd_steer=steer_arr,
        vicon_x=x,
        vicon_y=y,
        vicon_yaw=yaw_arr,
        vicon_body_vx=vx_arr,
        vicon_body_vy=vy_arr,
        vicon_r=yaw_rate_arr,
    )
    if include_rs_core_speed:
        if rs_core_speed is None:
            rs_arr = vx_arr.copy()
        else:
            rs_arr = np.broadcast_to(np.asarray(rs_core_speed, dtype=float), (n,)).copy()
        fields["rs_core_speed"] = rs_arr
    np.savez(path, **fields)


@pytest.fixture
def straight_npz(tmp_path):
    p = tmp_path / "straight.npz"
    _make_npz(str(p))
    return str(p)


@pytest.fixture
def asymmetric_npz(tmp_path):
    """Bag with all signals non-zero so mirror sign flips and init values are testable."""
    p = tmp_path / "asym.npz"
    _make_npz(str(p), speed=2.0, vy=0.3, yaw_rate=1.2, steer=0.4, yaw=0.1)
    return str(p)


# -- Window construction --


def test_window_shapes(straight_npz):
    ds = load_dataset(straight_npz, mirror=False)
    assert len(ds.windows) > 0
    for w in ds.windows:
        assert w.cmd_steer.shape == (N_STEPS,)
        assert w.cmd_speed.shape == (N_STEPS,)
        assert w.real_v_x.shape == (N_STEPS + 1,)
        assert w.real_v_y.shape == (N_STEPS + 1,)
        assert w.real_yaw_rate.shape == (N_STEPS + 1,)
        assert w.real_a_x.shape == (N_STEPS + 1,)
        assert w.real_omega.shape == (N_STEPS + 1,)
        assert w.real_pose.shape == (N_STEPS + 1, 2)
        assert w.init_state.shape == (9,)


def test_window_count_and_stride(straight_npz):
    ds = load_dataset(straight_npz, mirror=False)
    # n=500, n_steps=150, t0_max=349, stride=50 → t0 in {0,50,100,150,200,250,300}
    expected = list(range(0, 500 - N_STEPS, STRIDE))
    assert [w.t0_idx for w in ds.windows] == expected


def test_init_state_matches_bag_at_t0(asymmetric_npz):
    ds = load_dataset(asymmetric_npz, mirror=False)
    data = np.load(asymmetric_npz)
    for w in ds.windows:
        i = w.t0_idx
        assert w.init_state[0] == pytest.approx(data["vicon_x"][i])
        assert w.init_state[1] == pytest.approx(data["vicon_y"][i])
        assert w.init_state[2] == pytest.approx(data["cmd_steer"][i])
        assert w.init_state[3] == pytest.approx(np.hypot(data["vicon_body_vx"][i], data["vicon_body_vy"][i]))
        assert w.init_state[4] == pytest.approx(data["vicon_yaw"][i])
        assert w.init_state[5] == pytest.approx(data["vicon_r"][i])
        expected_beta = np.arctan2(data["vicon_body_vy"][i], data["vicon_body_vx"][i])
        assert w.init_state[6] == pytest.approx(expected_beta)


def test_real_signals_match_bag_slice(asymmetric_npz):
    ds = load_dataset(asymmetric_npz, mirror=False)
    data = np.load(asymmetric_npz)
    for w in ds.windows:
        i = w.t0_idx
        np.testing.assert_allclose(w.real_v_x, data["vicon_body_vx"][i : i + N_STEPS + 1])
        np.testing.assert_allclose(w.real_v_y, data["vicon_body_vy"][i : i + N_STEPS + 1])
        np.testing.assert_allclose(w.real_yaw_rate, data["vicon_r"][i : i + N_STEPS + 1])
        np.testing.assert_allclose(w.real_omega, data["rs_core_speed"][i : i + N_STEPS + 1] / R_W)
        np.testing.assert_allclose(w.real_pose[:, 0], data["vicon_x"][i : i + N_STEPS + 1])
        np.testing.assert_allclose(w.real_pose[:, 1], data["vicon_y"][i : i + N_STEPS + 1])
        np.testing.assert_allclose(w.cmd_steer, data["cmd_steer"][i : i + N_STEPS])
        np.testing.assert_allclose(w.cmd_speed, data["cmd_speed"][i : i + N_STEPS])


def test_real_pose_first_sample_matches_init_state(asymmetric_npz):
    """real_pose[0] must equal init_state[:2] — both come from `vicon_x[t0]` /
    `vicon_y[t0]` but through different code paths (scalar index vs slice). If
    they ever drift apart, the pose loss would silently score the sim's t=0
    against a different point than where it was seeded.
    """
    ds = load_dataset(asymmetric_npz, mirror=False)
    for w in ds.windows:
        np.testing.assert_allclose(w.real_pose[0], w.init_state[:2])


def test_real_a_x_is_smoothed_gradient(asymmetric_npz):
    ds = load_dataset(asymmetric_npz, mirror=False)
    data = np.load(asymmetric_npz)
    expected_full = np.gradient(savgol_filter(data["vicon_body_vx"], 21, 2), DT)
    for w in ds.windows:
        i = w.t0_idx
        np.testing.assert_allclose(w.real_a_x, expected_full[i : i + N_STEPS + 1])


# -- Filtering --


def test_low_speed_mask_drops_stationary(tmp_path):
    p = tmp_path / "halt.npz"
    speed = np.zeros(500)
    speed[250:] = 1.0
    _make_npz(str(p), speed=speed)
    ds = load_dataset(str(p), mirror=False, min_speed=0.3)
    # Any retained window must have mean speed over its [t0, t0+N_STEPS+1) ≥ 0.3.
    for w in ds.windows:
        n_moving = max(0, (w.t0_idx + N_STEPS + 1) - 250)
        mean_speed = n_moving / (N_STEPS + 1)
        assert mean_speed >= 0.3


def test_nan_guard_drops_windows(tmp_path):
    p = tmp_path / "nan.npz"
    n = 500
    steer = 0.1 * np.ones(n)
    steer[200] = np.nan  # cmd_steer doesn't propagate via cumsum, so NaN stays local
    _make_npz(str(p), n=n, steer=steer)
    ds = load_dataset(str(p), mirror=False)
    for w in ds.windows:
        assert np.all(np.isfinite(w.init_state))
        for sig in (w.real_v_x, w.real_v_y, w.real_yaw_rate, w.real_a_x, w.real_omega, w.cmd_steer, w.cmd_speed):
            assert np.all(np.isfinite(sig))
    expected_no_nan = (n - N_STEPS) // STRIDE + 1
    assert len(ds.windows) < expected_no_nan


def test_empty_result_raises(tmp_path):
    p = tmp_path / "stationary.npz"
    _make_npz(str(p), speed=0.0)
    with pytest.raises(ValueError, match="No windows"):
        load_dataset(str(p), mirror=False)


def test_too_short_bag_raises(tmp_path):
    p = tmp_path / "short.npz"
    _make_npz(str(p), n=50)  # 0.5 s, well below 1.5 s window
    with pytest.raises(ValueError, match="too short"):
        load_dataset(str(p), mirror=False)


def test_init_beta_masked_below_min_speed(tmp_path):
    """init β = 0 when instantaneous speed at t0 ≤ min_speed; otherwise atan2(vy, vx)."""
    p = tmp_path / "ramp.npz"
    n = 500
    vx = np.linspace(0.0, 1.0, n)
    vy_const = 0.1
    _make_npz(str(p), n=n, speed=vx, vy=vy_const * np.ones(n))
    ds = load_dataset(str(p), mirror=False, min_speed=0.3)
    saw_masked = False
    saw_unmasked = False
    for w in ds.windows:
        i = w.t0_idx
        speed_at_t0 = float(np.hypot(vx[i], vy_const))
        if speed_at_t0 > 0.3:
            assert w.init_state[6] == pytest.approx(np.arctan2(vy_const, vx[i]))
            saw_unmasked = True
        else:
            assert w.init_state[6] == 0.0
            saw_masked = True
    # Both branches must have been exercised, otherwise the test asserts nothing meaningful.
    assert saw_masked and saw_unmasked


# -- Mirror transform --


def test_mirror_involution(asymmetric_npz):
    ds = load_dataset(asymmetric_npz, mirror=False)
    w = ds.windows[0]
    w_mm = mirror_window(mirror_window(w))
    np.testing.assert_allclose(w_mm.init_state, w.init_state)
    np.testing.assert_allclose(w_mm.cmd_steer, w.cmd_steer)
    np.testing.assert_allclose(w_mm.cmd_speed, w.cmd_speed)
    np.testing.assert_allclose(w_mm.real_v_x, w.real_v_x)
    np.testing.assert_allclose(w_mm.real_v_y, w.real_v_y)
    np.testing.assert_allclose(w_mm.real_yaw_rate, w.real_yaw_rate)
    np.testing.assert_allclose(w_mm.real_a_x, w.real_a_x)
    np.testing.assert_allclose(w_mm.real_omega, w.real_omega)
    np.testing.assert_allclose(w_mm.real_pose, w.real_pose)
    assert w_mm.is_mirrored == w.is_mirrored


def test_mirror_sign_flips(asymmetric_npz):
    ds = load_dataset(asymmetric_npz, mirror=False)
    w = ds.windows[0]
    m = mirror_window(w)
    # Flipped: y, delta, yaw, yaw_rate, beta on init_state; cmd_steer; v_y, yaw_rate signals.
    assert m.init_state[1] == pytest.approx(-w.init_state[1])
    assert m.init_state[2] == pytest.approx(-w.init_state[2])
    assert m.init_state[4] == pytest.approx(-w.init_state[4])
    assert m.init_state[5] == pytest.approx(-w.init_state[5])
    assert m.init_state[6] == pytest.approx(-w.init_state[6])
    np.testing.assert_allclose(m.cmd_steer, -w.cmd_steer)
    np.testing.assert_allclose(m.real_v_y, -w.real_v_y)
    np.testing.assert_allclose(m.real_yaw_rate, -w.real_yaw_rate)
    np.testing.assert_allclose(m.real_pose[:, 1], -w.real_pose[:, 1])
    # Preserved: x, v on init_state; cmd_speed; v_x, a_x signals.
    assert m.init_state[0] == pytest.approx(w.init_state[0])
    assert m.init_state[3] == pytest.approx(w.init_state[3])
    np.testing.assert_allclose(m.cmd_speed, w.cmd_speed)
    np.testing.assert_allclose(m.real_v_x, w.real_v_x)
    np.testing.assert_allclose(m.real_a_x, w.real_a_x)
    np.testing.assert_allclose(m.real_omega, w.real_omega)
    np.testing.assert_allclose(m.real_pose[:, 0], w.real_pose[:, 0])
    # Wheel angular speeds are positive scalars, sign-symmetric under L/R mirror.
    assert m.init_state[7] == pytest.approx(w.init_state[7])
    assert m.init_state[8] == pytest.approx(w.init_state[8])
    assert m.is_mirrored is True


def test_mirror_flag_toggles(asymmetric_npz):
    ds = load_dataset(asymmetric_npz, mirror=False)
    w = ds.windows[0]
    assert w.is_mirrored is False
    assert mirror_window(w).is_mirrored is True
    assert mirror_window(mirror_window(w)).is_mirrored is False


def test_mirror_doubles_dataset(straight_npz):
    ds_no = load_dataset(straight_npz, mirror=False)
    ds_yes = load_dataset(straight_npz, mirror=True)
    assert len(ds_yes.windows) == 2 * len(ds_no.windows)
    n_mir = sum(w.is_mirrored for w in ds_yes.windows)
    assert n_mir == len(ds_no.windows)


# -- VESC-seeded wheel angular velocity (omega_f, omega_r) --


def test_init_omega_seeded_from_vesc(tmp_path):
    """omega_f = omega_r = rs_core_speed[t0] / R_w (AWD assumption)."""
    p = tmp_path / "vesc.npz"
    n = 500
    # Make rs_core_speed deliberately distinct from vx so we can tell which
    # one fed the omega seed (a no-slip seed would equal vx/R_w, not this).
    rs = np.linspace(0.5, 2.5, n)
    _make_npz(str(p), n=n, speed=1.0, rs_core_speed=rs)
    ds = load_dataset(str(p), mirror=False)
    for w in ds.windows:
        expected = rs[w.t0_idx] / R_W
        assert w.init_state[7] == pytest.approx(expected)
        assert w.init_state[8] == pytest.approx(expected)


def test_missing_rs_core_speed_raises(tmp_path):
    p = tmp_path / "no_vesc.npz"
    _make_npz(str(p), include_rs_core_speed=False)
    with pytest.raises(KeyError, match="rs_core_speed"):
        load_dataset(str(p), mirror=False)


def test_nan_in_rs_core_speed_drops_window(tmp_path):
    """Non-finite VESC sample at t0 must drop the window, like other channels."""
    p = tmp_path / "vesc_nan.npz"
    n = 500
    rs = np.ones(n)
    rs[200] = np.nan
    _make_npz(str(p), n=n, speed=1.0, rs_core_speed=rs)
    ds = load_dataset(str(p), mirror=False)
    for w in ds.windows:
        assert np.all(np.isfinite(w.init_state))
    expected_no_nan = (n - N_STEPS) // STRIDE + 1
    assert len(ds.windows) < expected_no_nan


# -- Misc --


def test_window_is_frozen(straight_npz):
    ds = load_dataset(straight_npz, mirror=False)
    with pytest.raises(dataclasses.FrozenInstanceError):
        ds.windows[0].t0_idx = 999  # type: ignore[misc]


def test_dataset_dt_set(straight_npz):
    ds = load_dataset(straight_npz, mirror=False, dt=0.01)
    assert ds.dt == 0.01
