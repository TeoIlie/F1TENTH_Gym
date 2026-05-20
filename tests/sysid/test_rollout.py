"""Integration tests for examples.analysis.sysid.rollout.

These tests construct a real GKEnv per module (slow) and replay synthetic
windows — a constant-steer / constant-speed input at moderate speed that
exercises the full STD dynamic regime without depending on a recorded bag
(which is unavailable in CI). Coverage:
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
from copy import deepcopy
from unittest.mock import MagicMock

import numpy as np
import pytest

from examples.analysis.sysid.dataset import CHANNELS, Dataset, Window, mirror_window
from examples.analysis.sysid.env import SYSID_PARAMS
from examples.analysis.sysid.loss import dataset_loss
from examples.analysis.sysid.rollout import Rollout
from gymkhana.envs.gymkhana_env import GKEnv

DT = 0.01


def _make_synthetic_window(n: int = 150, v: float = 3.0, steer: float = 0.15) -> Window:
    """Constant-steer / constant-speed window at moderate speed.

    Sized to clear the dataset_loss warmup (warmup_s=0.2 → 20 steps at dt=0.01).
    Sits above STD's kinematic-dynamic blend so set_params perturbations and
    the L/R mirror invariant exercise dynamic-regime tire forces. The real_*
    arrays are zeros: every assertion in this module is on sim shapes, sim
    init-state propagation, sim determinism, sim a_x = ∂v_x, hot-swap effect
    on sim, the NaN guard (with a mocked step), or sim-vs-zero finiteness in
    dataset_loss — none depend on real-trajectory values.
    """
    omega0 = v / SYSID_PARAMS["R_w"]
    return Window(
        t0_idx=0,
        init_state=np.array([0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, omega0, omega0]),
        cmd_steer=np.full(n, steer),
        cmd_speed=np.full(n, v),
        real_v_x=np.zeros(n + 1),
        real_v_y=np.zeros(n + 1),
        real_yaw_rate=np.zeros(n + 1),
        real_a_x=np.zeros(n + 1),
        real_omega=np.zeros(n + 1),
        real_pose=np.zeros((n + 1, 2)),
        real_yaw=np.zeros(n + 1),
        real_beta=np.zeros(n + 1),
        is_mirrored=False,
    )


@pytest.fixture(scope="module")
def window() -> Window:
    return _make_synthetic_window()


@pytest.fixture(scope="module")
def dataset(window) -> Dataset:
    return Dataset(
        windows=[window],
        dt=DT,
        n_candidates=1,
        n_dropped_low_speed=0,
        n_dropped_nonfinite=0,
    )


@pytest.fixture(scope="module")
def rollout():
    r = Rollout()
    yield r
    r.close()


# ---------- shape / dt / init-state correctness ----------


def test_run_returns_correct_shape_and_keys(rollout, window):
    sim = rollout.run(window)
    assert set(sim.keys()) == set(CHANNELS)
    expected_len = len(window.real_v_x)
    for ch in CHANNELS:
        expected_shape = (expected_len, 2) if ch == "pose" else (expected_len,)
        assert sim[ch].shape == expected_shape, f"{ch} has wrong shape"
        assert np.all(np.isfinite(sim[ch])), f"{ch} contains non-finite values"


def test_run_pose_init_matches_init_state(rollout, window):
    """First pose sample must equal the seeded x,y from init_state — pins that
    `env.reset(options={'states': ...})` propagates world-frame pose verbatim
    (otherwise the XY tracking channel would score against a sim that started
    from somewhere other than the window-start pose).
    """
    sim = rollout.run(window)
    np.testing.assert_allclose(sim["pose"][0], window.init_state[:2], rtol=1e-6, atol=1e-8)


def test_dt_matches_env_timestep(rollout):
    assert rollout.dt == pytest.approx(DT)


def test_initial_sample_reflects_init_state(rollout, window):
    """First sample of sim signals must match Window.init_state — confirms
    `env.reset(options={'states': ...})` actually applies the requested state.
    standard_state for STD sets v_x = v*cos(β), v_y = v*sin(β), yaw_rate = state[5].
    """
    sim = rollout.run(window)
    v, beta = window.init_state[3], window.init_state[6]
    assert sim["yaw_rate"][0] == pytest.approx(window.init_state[5], rel=1e-4, abs=1e-5)
    assert sim["v_x"][0] == pytest.approx(v * np.cos(beta), rel=1e-4, abs=1e-5)
    assert sim["v_y"][0] == pytest.approx(v * np.sin(beta), rel=1e-4, abs=1e-5)


def test_reset_seeds_wheel_omegas_from_init_state(rollout, window):
    """End-to-end contract: a 9-wide init_state with extreme omegas must land
    verbatim in agent.state[7:8] after reset, with no recomputation from v.

    Pins the chain dataset.py → rollout.reshape(1,9) → GKEnv.reset →
    init_std(compute_wheel_speeds=False). Without this test, init_std could
    silently revert to the no-slip formula and every sysid test would still
    pass (downstream sim signals would just be wrong, not asserted-against).
    """
    # Use a value that the no-slip formula provably could not produce —
    # ~30x the steady-state omega for the synthetic window's 3 m/s.
    extreme_omega = 100.0
    seeded = window.init_state.copy()
    seeded[7] = extreme_omega
    seeded[8] = extreme_omega + 1.0  # different value to also pin index ordering
    w_seeded = dataclasses.replace(window, init_state=seeded)

    rollout._env.reset(options={"states": w_seeded.init_state.reshape(1, 9)})
    agent_state = rollout._env.sim.agents[0].state
    assert agent_state[7] == pytest.approx(extreme_omega, rel=1e-9)
    assert agent_state[8] == pytest.approx(extreme_omega + 1.0, rel=1e-9)


# ---------- determinism ----------


def test_run_is_deterministic(rollout, window):
    """Sampler-determinism invariant from OVERVIEW.md — same input must yield
    bit-identical output, otherwise trial-to-trial noise masquerades as a bad
    parameter.
    """
    sim_a = rollout.run(window)
    sim_b = rollout.run(window)
    for ch in CHANNELS:
        np.testing.assert_array_equal(sim_a[ch], sim_b[ch], err_msg=f"non-deterministic on {ch}")


# ---------- a_x derivation ----------


def test_a_x_is_finite_diff_of_v_x(rollout, window):
    sim = rollout.run(window)
    expected_a_x = np.gradient(sim["v_x"], DT)
    np.testing.assert_array_equal(sim["a_x"], expected_a_x)


# ---------- set_params ----------


def test_set_params_changes_output(rollout, window):
    """Hot-swap must actually rebuild the simulator's params; otherwise every
    Optuna trial would silently score the same baseline rollout.
    """
    base_params = GKEnv.f1tenth_std_vehicle_params()
    sim_base = rollout.run(window)

    # Halve the lateral peak factor — should perceptibly change v_y / yaw_rate.
    perturbed = deepcopy(base_params)
    perturbed["tire_p_dy1"] = base_params["tire_p_dy1"] * 0.5
    rollout.set_params(perturbed)
    try:
        sim_perturbed = rollout.run(window)
    finally:
        rollout.set_params(base_params)  # restore for downstream tests

    diff_yaw = float(np.max(np.abs(sim_perturbed["yaw_rate"] - sim_base["yaw_rate"])))
    diff_vy = float(np.max(np.abs(sim_perturbed["v_y"] - sim_base["v_y"])))
    assert diff_yaw > 1e-3, f"yaw_rate unchanged after set_params (max |Δ|={diff_yaw:.2e})"
    assert diff_vy > 1e-3, f"v_y unchanged after set_params (max |Δ|={diff_vy:.2e})"


# ---------- mirror invariant ----------


def test_mirror_invariant_under_default_params(rollout, window):
    """OVERVIEW.md mirror invariant — under STD's structural L/R symmetry,
    a mirrored window must produce sim signals that are sign-flipped on
    antisymmetric channels (yaw_rate, v_y) and identical on symmetric ones
    (v_x, a_x). Catches sign bugs in mirror_window or any future per-channel
    handling in run().

    The synthetic constant-steer / constant-speed window at 3 m/s exercises
    the dynamic regime cleanly — well above STD's kinematic-dynamic blend
    (<1 m/s) where the invariant breaks down numerically.
    """
    w_mirr = mirror_window(window)

    sim = rollout.run(window)
    sim_m = rollout.run(w_mirr)

    # Float32 obs precision + float64 integrator picking up small L/R numerical
    # asymmetries limit how tight we can go. Symmetric channels (v_x, omega,
    # pose_x, pose_y) stay near float32 precision (~0.1%); antisymmetric ones
    # (v_y, yaw_rate, yaw, beta) accumulate noticeably more drift (up to ~5%
    # rel on small-magnitude v_y / beta) because the integrator processes
    # "turn left" vs "turn right" with non-bitwise-symmetric floating point
    # ops. a_x = gradient(v_x)/dt amplifies v_x asymmetry by 1/dt = 100×.
    # A real sign bug in mirror_window or the run loop would produce O(signal)
    # errors — ~20× above this floor — so the test still catches them.
    tol_sym = dict(rtol=2e-3, atol=5e-4)
    tol_anti = dict(rtol=5e-2, atol=2e-2)
    tol_ax = dict(rtol=1e-1, atol=2e-2)
    np.testing.assert_allclose(sim_m["v_x"], sim["v_x"], **tol_sym)
    np.testing.assert_allclose(sim_m["v_y"], -sim["v_y"], **tol_anti)
    np.testing.assert_allclose(sim_m["yaw_rate"], -sim["yaw_rate"], **tol_anti)
    np.testing.assert_allclose(sim_m["a_x"], sim["a_x"], **tol_ax)
    np.testing.assert_allclose(sim_m["omega"], sim["omega"], **tol_sym)
    # World-frame pose: x stays, y flips under L/R mirror.
    np.testing.assert_allclose(sim_m["pose"][:, 0], sim["pose"][:, 0], **tol_sym)
    np.testing.assert_allclose(sim_m["pose"][:, 1], -sim["pose"][:, 1], **tol_sym)
    # Heading and slip both flip sign under L/R mirror.
    np.testing.assert_allclose(sim_m["yaw"], -sim["yaw"], **tol_anti)
    np.testing.assert_allclose(sim_m["beta"], -sim["beta"], **tol_anti)


# ---------- NaN/inf guard ----------


def test_run_raises_on_non_finite_sim_signal(rollout, window, monkeypatch):
    """Trial params can drive the integrator non-finite. The guard must raise
    so the Optuna objective can map this to a prunable trial loss instead of
    silently returning NaN.
    """
    agent_id = rollout._agent_id
    nan_obs = {
        agent_id: {
            "linear_vel_x": float("nan"),
            "linear_vel_y": 0.0,
            "ang_vel_z": 0.0,
            "pose_x": 0.0,
            "pose_y": 0.0,
            "pose_theta": 0.0,
            "beta": 0.0,
        }
    }
    fake_step = MagicMock(return_value=(nan_obs, 0.0, False, False, {}))
    monkeypatch.setattr(rollout._env, "step", fake_step)

    with pytest.raises(FloatingPointError, match="Non-finite sim signal"):
        rollout.run(window)


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
