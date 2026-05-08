"""Integration tests for examples.analysis.sysid.rollout.

These tests construct a real GKEnv per module (slow) and replay windows
loaded from a real bag. Coverage:
  - shape/key contract returned by `Rollout.run`
  - dt property matches env timestep
  - first sample of sim signals matches Window.init_state (reset works)
  - same window replayed twice produces bit-identical output (sampler-determinism invariant)
  - `set_params` actually changes rollout output (hot-swap works)
  - L/R mirror invariant: under default symmetric STD params, mirrored window's
    sim signals are sign-flipped on antisymmetric channels (yaw_rate, v_y) and
    identical on symmetric channels (v_x, a_x)
  - non-finite sim signal raises FloatingPointError (Optuna-prunable trial)
  - context manager closes the env
  - dataset_loss runs end-to-end and produces a finite, non-negative number
  - sim a_x is the finite-difference of sim v_x (no smoothing applied)
"""

from __future__ import annotations

import dataclasses
import os
from copy import deepcopy
from unittest.mock import MagicMock

import numpy as np
import pytest

from examples.analysis.sysid.dataset import CHANNELS, Window, load_dataset, mirror_window
from examples.analysis.sysid.env import SYSID_PARAMS
from examples.analysis.sysid.loss import dataset_loss
from examples.analysis.sysid.rollout import Rollout
from gymkhana.envs.gymkhana_env import GKEnv

BAG_PATH = "examples/analysis/bags/circle_Apr6_100Hz.npz"
DT = 0.01

pytestmark = pytest.mark.skipif(
    not os.path.exists(BAG_PATH),
    reason=f"requires test bag {BAG_PATH}",
)


@pytest.fixture(scope="module")
def dataset():
    return load_dataset(BAG_PATH, mirror=False)


@pytest.fixture(scope="module")
def rollout():
    r = Rollout()
    yield r
    r.close()


# ---------- shape / dt / init-state correctness ----------


def test_run_returns_correct_shape_and_keys(rollout, dataset):
    sim = rollout.run(dataset.windows[0])
    assert set(sim.keys()) == set(CHANNELS)
    expected_len = len(dataset.windows[0].real_v_x)
    for ch in CHANNELS:
        assert sim[ch].shape == (expected_len,), f"{ch} has wrong shape"
        assert np.all(np.isfinite(sim[ch])), f"{ch} contains non-finite values"


def test_dt_matches_env_timestep(rollout):
    assert rollout.dt == pytest.approx(DT)


def test_initial_sample_reflects_init_state(rollout, dataset):
    """First sample of sim signals must match Window.init_state — confirms
    `env.reset(options={'states': ...})` actually applies the requested state.
    standard_state for STD sets v_x = v*cos(β), v_y = v*sin(β), yaw_rate = state[5].
    """
    w = dataset.windows[0]
    sim = rollout.run(w)
    v, beta = w.init_state[3], w.init_state[6]
    assert sim["yaw_rate"][0] == pytest.approx(w.init_state[5], rel=1e-4, abs=1e-5)
    assert sim["v_x"][0] == pytest.approx(v * np.cos(beta), rel=1e-4, abs=1e-5)
    assert sim["v_y"][0] == pytest.approx(v * np.sin(beta), rel=1e-4, abs=1e-5)


def test_reset_seeds_wheel_omegas_from_init_state(rollout, dataset):
    """End-to-end contract: a 9-wide init_state with extreme omegas must land
    verbatim in agent.state[7:8] after reset, with no recomputation from v.

    Pins the chain dataset.py → rollout.reshape(1,9) → GKEnv.reset →
    init_std(compute_wheel_speeds=False). Without this test, init_std could
    silently revert to the no-slip formula and every sysid test would still
    pass (downstream sim signals would just be wrong, not asserted-against).
    """
    w = dataset.windows[0]
    # Use a value that the no-slip formula provably could not produce —
    # 5x the reasonable steady-state omega for any speed in the bag.
    extreme_omega = 100.0
    seeded = w.init_state.copy()
    seeded[7] = extreme_omega
    seeded[8] = extreme_omega + 1.0  # different value to also pin index ordering
    w_seeded = dataclasses.replace(w, init_state=seeded)

    rollout._env.reset(options={"states": w_seeded.init_state.reshape(1, 9)})
    agent_state = rollout._env.sim.agents[0].state
    assert agent_state[7] == pytest.approx(extreme_omega, rel=1e-9)
    assert agent_state[8] == pytest.approx(extreme_omega + 1.0, rel=1e-9)


# ---------- determinism ----------


def test_run_is_deterministic(rollout, dataset):
    """Sampler-determinism invariant from OVERVIEW.md — same input must yield
    bit-identical output, otherwise trial-to-trial noise masquerades as a bad
    parameter.
    """
    w = dataset.windows[0]
    sim_a = rollout.run(w)
    sim_b = rollout.run(w)
    for ch in CHANNELS:
        np.testing.assert_array_equal(sim_a[ch], sim_b[ch], err_msg=f"non-deterministic on {ch}")


# ---------- a_x derivation ----------


def test_a_x_is_finite_diff_of_v_x(rollout, dataset):
    sim = rollout.run(dataset.windows[0])
    expected_a_x = np.gradient(sim["v_x"], DT)
    np.testing.assert_array_equal(sim["a_x"], expected_a_x)


# ---------- set_params ----------


def test_set_params_changes_output(rollout, dataset):
    """Hot-swap must actually rebuild the simulator's params; otherwise every
    Optuna trial would silently score the same baseline rollout.
    """
    w = dataset.windows[0]
    base_params = GKEnv.f1tenth_std_vehicle_params()
    sim_base = rollout.run(w)

    # Halve the lateral peak factor — should perceptibly change v_y / yaw_rate.
    perturbed = deepcopy(base_params)
    perturbed["tire_p_dy1"] = base_params["tire_p_dy1"] * 0.5
    rollout.set_params(perturbed)
    try:
        sim_perturbed = rollout.run(w)
    finally:
        rollout.set_params(base_params)  # restore for downstream tests

    diff_yaw = float(np.max(np.abs(sim_perturbed["yaw_rate"] - sim_base["yaw_rate"])))
    diff_vy = float(np.max(np.abs(sim_perturbed["v_y"] - sim_base["v_y"])))
    assert diff_yaw > 1e-3, f"yaw_rate unchanged after set_params (max |Δ|={diff_yaw:.2e})"
    assert diff_vy > 1e-3, f"v_y unchanged after set_params (max |Δ|={diff_vy:.2e})"


# ---------- mirror invariant ----------


def test_mirror_invariant_under_default_params(rollout):
    """OVERVIEW.md mirror invariant — under STD's structural L/R symmetry,
    a mirrored window must produce sim signals that are sign-flipped on
    antisymmetric channels (yaw_rate, v_y) and identical on symmetric ones
    (v_x, a_x). Catches sign bugs in mirror_window or any future per-channel
    handling in run().

    Uses a synthetic constant-steer / constant-speed window at moderate
    speed so the test exercises the dynamic regime cleanly. Real-bag windows
    at <1 m/s sit in STD's kinematic-dynamic blend where the invariant breaks
    down numerically and is not informative.
    """
    n = 150
    w = Window(
        t0_idx=0,
        init_state=np.array([0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0 / SYSID_PARAMS["R_w"], 3.0 / SYSID_PARAMS["R_w"]]),
        cmd_steer=np.full(n, 0.15),
        cmd_speed=np.full(n, 3.0),
        real_v_x=np.zeros(n + 1),  # unused for this test
        real_v_y=np.zeros(n + 1),
        real_yaw_rate=np.zeros(n + 1),
        real_a_x=np.zeros(n + 1),
        real_omega=np.zeros(n + 1),
        is_mirrored=False,
    )
    w_mirr = mirror_window(w)

    sim = rollout.run(w)
    sim_m = rollout.run(w_mirr)

    # Float32 obs precision + float64 integrator picking up small L/R numerical
    # asymmetries (~0.1% of signal magnitude) limit how tight we can go. A real
    # sign bug in mirror_window or the run loop would produce O(signal) errors —
    # ~3 orders of magnitude above this floor — so the test still catches them.
    # a_x = gradient(v_x)/dt amplifies v_x asymmetry by 1/dt = 100× → looser.
    tol = dict(rtol=1e-3, atol=5e-4)
    tol_ax = dict(rtol=5e-2, atol=5e-2)
    np.testing.assert_allclose(sim_m["v_x"], sim["v_x"], **tol)
    np.testing.assert_allclose(sim_m["v_y"], -sim["v_y"], **tol)
    np.testing.assert_allclose(sim_m["yaw_rate"], -sim["yaw_rate"], **tol)
    np.testing.assert_allclose(sim_m["a_x"], sim["a_x"], **tol_ax)
    np.testing.assert_allclose(sim_m["omega"], sim["omega"], **tol)


# ---------- NaN/inf guard ----------


def test_run_raises_on_non_finite_sim_signal(rollout, dataset, monkeypatch):
    """Trial params can drive the integrator non-finite. The guard must raise
    so the Optuna objective can map this to a prunable trial loss instead of
    silently returning NaN.
    """
    w = dataset.windows[0]
    agent_id = rollout._agent_id
    nan_obs = {agent_id: {"linear_vel_x": float("nan"), "linear_vel_y": 0.0, "ang_vel_z": 0.0}}
    fake_step = MagicMock(return_value=(nan_obs, 0.0, False, False, {}))
    monkeypatch.setattr(rollout._env, "step", fake_step)

    with pytest.raises(FloatingPointError, match="Non-finite sim signal"):
        rollout.run(w)


# ---------- context manager ----------


def test_context_manager_calls_close():
    r = Rollout()
    r.close = MagicMock(wraps=r.close)
    with r as ctx:
        assert ctx is r
    r.close.assert_called_once()


# ---------- end-to-end loss integration ----------


def test_dataset_loss_end_to_end_is_finite_nonnegative(rollout, dataset):
    total, per_channel = dataset_loss(rollout.run, dataset)
    assert np.isfinite(total) and total >= 0
    for ch in CHANNELS:
        assert np.isfinite(per_channel[ch]) and per_channel[ch] >= 0
