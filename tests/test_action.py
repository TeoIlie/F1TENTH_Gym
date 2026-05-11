"""
Tests for CarAction __init__ parsing and order-independent control_input handling.
"""

import warnings

import gymnasium as gym
import numpy as np
import pytest

from gymkhana.envs.action import (
    AcclAction,
    CarAction,
    SpeedAction,
    SteeringAngleAction,
    SteeringSpeedAction,
)
from gymkhana.envs.dynamic_models import bang_bang_steer
from gymkhana.envs.gymkhana_env import GKEnv


@pytest.fixture
def params():
    """Minimal parameter dict for constructing CarAction."""
    return {
        "a_max": 9.51,
        "v_max": 20.0,
        "v_min": -5.0,
        "s_max": 0.4189,
        "s_min": -0.4189,
        "sv_max": 3.2,
        "sv_min": -3.2,
    }


# ============================================================================
# List input: order-independent parsing
# ============================================================================


class TestListOrderIndependence:
    """control_input as a list of two strings should work in any order."""

    def test_accl_steering_angle(self, params):
        ca = CarAction(["accl", "steering_angle"], params=params, normalize=False)
        assert isinstance(ca._longitudinal_action, AcclAction)
        assert isinstance(ca._steer_action, SteeringAngleAction)

    def test_steering_angle_accl(self, params):
        ca = CarAction(["steering_angle", "accl"], params=params, normalize=False)
        assert isinstance(ca._longitudinal_action, AcclAction)
        assert isinstance(ca._steer_action, SteeringAngleAction)

    def test_speed_steering_angle(self, params):
        ca = CarAction(["speed", "steering_angle"], params=params, normalize=False)
        assert isinstance(ca._longitudinal_action, SpeedAction)
        assert isinstance(ca._steer_action, SteeringAngleAction)

    def test_steering_angle_speed(self, params):
        ca = CarAction(["steering_angle", "speed"], params=params, normalize=False)
        assert isinstance(ca._longitudinal_action, SpeedAction)
        assert isinstance(ca._steer_action, SteeringAngleAction)

    def test_accl_steering_speed(self, params):
        ca = CarAction(["accl", "steering_speed"], params=params, normalize=False)
        assert isinstance(ca._longitudinal_action, AcclAction)
        assert isinstance(ca._steer_action, SteeringSpeedAction)

    def test_steering_speed_accl(self, params):
        ca = CarAction(["steering_speed", "accl"], params=params, normalize=False)
        assert isinstance(ca._longitudinal_action, AcclAction)
        assert isinstance(ca._steer_action, SteeringSpeedAction)

    def test_speed_steering_speed(self, params):
        ca = CarAction(["speed", "steering_speed"], params=params, normalize=False)
        assert isinstance(ca._longitudinal_action, SpeedAction)
        assert isinstance(ca._steer_action, SteeringSpeedAction)

    def test_steering_speed_speed(self, params):
        ca = CarAction(["steering_speed", "speed"], params=params, normalize=False)
        assert isinstance(ca._longitudinal_action, SpeedAction)
        assert isinstance(ca._steer_action, SteeringSpeedAction)

    def test_reversed_order_same_space(self, params):
        """Both orderings should produce identical action spaces."""
        ca1 = CarAction(["accl", "steering_angle"], params=params, normalize=False)
        ca2 = CarAction(["steering_angle", "accl"], params=params, normalize=False)
        np.testing.assert_array_equal(ca1.space.low, ca2.space.low)
        np.testing.assert_array_equal(ca1.space.high, ca2.space.high)

    def test_reversed_order_same_type(self, params):
        """Both orderings should report the same type tuple."""
        ca1 = CarAction(["speed", "steering_angle"], params=params, normalize=False)
        ca2 = CarAction(["steering_angle", "speed"], params=params, normalize=False)
        assert ca1.type == ca2.type


# ============================================================================
# List input: error cases
# ============================================================================


class TestListErrors:
    """Invalid list inputs should raise clear ValueErrors."""

    def test_two_longitudinal_types(self, params):
        with pytest.raises(ValueError, match="two longitudinal types"):
            CarAction(["accl", "speed"], params=params, normalize=False)

    def test_two_steering_types(self, params):
        with pytest.raises(ValueError, match="two steering types"):
            CarAction(["steering_angle", "steering_speed"], params=params, normalize=False)

    def test_unknown_mode(self, params):
        with pytest.raises(ValueError, match="Unknown control mode 'unknown'"):
            CarAction(["accl", "unknown"], params=params, normalize=False)

    def test_both_unknown(self, params):
        with pytest.raises(ValueError, match="Unknown control mode"):
            CarAction(["foo", "bar"], params=params, normalize=False)

    def test_empty_list(self, params):
        with pytest.raises(ValueError, match="exactly 2 elements"):
            CarAction([], params=params, normalize=False)

    def test_single_element_list(self, params):
        with pytest.raises(ValueError, match="exactly 2 elements"):
            CarAction(["accl"], params=params, normalize=False)

    def test_three_element_list(self, params):
        with pytest.raises(ValueError, match="exactly 2 elements"):
            CarAction(["accl", "steering_angle", "speed"], params=params, normalize=False)

    def test_non_string_non_list(self, params):
        with pytest.raises(ValueError, match="Unknown control mode"):
            CarAction(123, params=params, normalize=False)


# ============================================================================
# String input: single control mode with defaults
# ============================================================================


class TestSingleStringInput:
    """A single string should default the missing control type with a warning."""

    def test_accl_defaults_to_steering_speed(self, params):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            ca = CarAction("accl", params=params, normalize=False)
            assert len(w) == 1
            assert "defaulting to steering speed" in str(w[0].message)
        assert isinstance(ca._longitudinal_action, AcclAction)
        assert isinstance(ca._steer_action, SteeringSpeedAction)

    def test_speed_defaults_to_steering_angle(self, params):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            ca = CarAction("speed", params=params, normalize=False)
            assert len(w) == 1
            assert "defaulting to steering angle" in str(w[0].message)
        assert isinstance(ca._longitudinal_action, SpeedAction)
        assert isinstance(ca._steer_action, SteeringAngleAction)

    def test_steering_angle_defaults_to_speed(self, params):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            ca = CarAction("steering_angle", params=params, normalize=False)
            assert len(w) == 1
            assert "defaulting to speed" in str(w[0].message)
        assert isinstance(ca._longitudinal_action, SpeedAction)
        assert isinstance(ca._steer_action, SteeringAngleAction)

    def test_steering_speed_defaults_to_accl(self, params):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            ca = CarAction("steering_speed", params=params, normalize=False)
            assert len(w) == 1
            assert "defaulting to acceleration" in str(w[0].message)
        assert isinstance(ca._longitudinal_action, AcclAction)
        assert isinstance(ca._steer_action, SteeringSpeedAction)

    def test_unknown_string_raises(self, params):
        with pytest.raises(ValueError, match="Unknown control mode"):
            CarAction("unknown", params=params, normalize=False)


# ============================================================================
# Action space and type properties
# ============================================================================


class TestSpaceAndType:
    """Verify action space bounds and type tuple are consistent."""

    def test_space_shape(self, params):
        ca = CarAction(["accl", "steering_angle"], params=params, normalize=False)
        assert ca.space.shape == (2,)

    def test_space_ordering_steer_first(self, params):
        """Action space index 0 should be steer bounds, index 1 should be longitudinal bounds."""
        ca = CarAction(["accl", "steering_angle"], params=params, normalize=False)
        # index 0: steering angle bounds
        assert ca.space.low[0] == pytest.approx(params["s_min"])
        assert ca.space.high[0] == pytest.approx(params["s_max"])
        # index 1: acceleration bounds
        assert ca.space.low[1] == pytest.approx(-params["a_max"])
        assert ca.space.high[1] == pytest.approx(params["a_max"])

    def test_space_ordering_speed(self, params):
        ca = CarAction(["speed", "steering_angle"], params=params, normalize=False)
        # index 0: steering angle bounds
        assert ca.space.low[0] == pytest.approx(params["s_min"])
        assert ca.space.high[0] == pytest.approx(params["s_max"])
        # index 1: speed bounds
        assert ca.space.low[1] == pytest.approx(params["v_min"])
        assert ca.space.high[1] == pytest.approx(params["v_max"])

    def test_type_tuple_is_steer_then_long(self, params):
        """type property should return (steer_type, longitudinal_type)."""
        ca = CarAction(["accl", "steering_angle"], params=params, normalize=False)
        assert ca.type == ("steering_angle", "accl")

    def test_normalized_space(self, params):
        """When normalized, both dimensions should be [-1, 1]."""
        ca = CarAction(["accl", "steering_angle"], params=params, normalize=True)
        np.testing.assert_array_almost_equal(ca.space.low, [-1.0, -1.0])
        np.testing.assert_array_almost_equal(ca.space.high, [1.0, 1.0])


# ============================================================================
# Regression: act() index-to-controller mapping
# ============================================================================


class TestActIndexMapping:
    """Verify act() feeds action[0] to steer controller and action[1] to longitudinal controller.

    Regression test for index-swap bugs between the action array and the
    underlying steer / longitudinal controllers.
    """

    # A dummy state vector (length 7, matching single-track models).
    # state[2] = steering angle, state[3] = velocity — used by controllers.
    _state = np.zeros(7, dtype=np.float64)

    @pytest.mark.parametrize(
        "control_input",
        [
            ["accl", "steering_angle"],
            ["steering_angle", "accl"],
        ],
    )
    def test_accl_steering_angle_non_normalized(self, params, control_input):
        """With normalize=False, accl passes through and steering_angle feeds bang-bang."""
        ca = CarAction(control_input, params=params, normalize=False)

        steer_idx = ca.type.index("steering_angle")  # should be 0
        long_idx = ca.type.index("accl")  # should be 1
        assert steer_idx == 0 and long_idx == 1, "type tuple must be (steer, long)"

        steer_val = 0.2
        accl_val = 3.5
        action = np.array([steer_val, accl_val], dtype=np.float64)

        long_out, steer_out = ca.act(action, state=self._state, params=params)

        # Longitudinal (accl) pass-through: should equal accl_val, not steer_val
        assert long_out == pytest.approx(accl_val), (
            f"Longitudinal output {long_out} should come from action[1]={accl_val}"
        )
        # Steer output comes from bang_bang_steer on steer_val — just verify it
        # was NOT fed the accl value (which would produce a very different result)
        assert steer_out != pytest.approx(accl_val)

    @pytest.mark.parametrize(
        "control_input",
        [
            ["accl", "steering_angle"],
            ["steering_angle", "accl"],
        ],
    )
    def test_accl_steering_angle_normalized(self, params, control_input):
        """With normalize=True, act() must denormalize each axis correctly."""
        ca = CarAction(control_input, params=params, normalize=True)

        steer_val_norm = 0.5
        accl_val_norm = -0.8
        action = np.array([steer_val_norm, accl_val_norm], dtype=np.float64)

        long_out, steer_out = ca.act(action, state=self._state, params=params)

        # Longitudinal should be accl_val_norm * a_max
        expected_accl = accl_val_norm * params["a_max"]
        assert long_out == pytest.approx(expected_accl), f"Normalized accl output {long_out} should be {expected_accl}"

    @pytest.mark.parametrize(
        "control_input",
        [
            ["speed", "steering_angle"],
            ["steering_angle", "speed"],
        ],
    )
    def test_speed_steering_angle_non_normalized(self, params, control_input):
        """Speed action feeds p_accl controller; verify it receives the longitudinal element."""
        ca = CarAction(control_input, params=params, normalize=False)

        steer_idx = ca.type.index("steering_angle")
        long_idx = ca.type.index("speed")
        assert steer_idx == 0 and long_idx == 1

        steer_val = 0.1
        speed_val = 10.0
        action = np.array([steer_val, speed_val], dtype=np.float64)

        long_out, steer_out = ca.act(action, state=self._state, params=params)

        # With state velocity=0 and desired_speed=10, p_accl should return a_max
        assert long_out == pytest.approx(params["a_max"]), "p_accl(10, 0, ...) should saturate at a_max"

    @pytest.mark.parametrize(
        "control_input",
        [
            ["accl", "steering_speed"],
            ["steering_speed", "accl"],
        ],
    )
    def test_accl_steering_speed_non_normalized(self, params, control_input):
        """Both accl and steering_speed are pure pass-through — easiest to verify exactly."""
        ca = CarAction(control_input, params=params, normalize=False)

        steer_val = 1.5
        accl_val = -2.0
        action = np.array([steer_val, accl_val], dtype=np.float64)

        long_out, steer_out = ca.act(action, state=self._state, params=params)

        assert long_out == pytest.approx(accl_val)
        assert steer_out == pytest.approx(steer_val)


# ============================================================================
# First-order steering actuator lag in SteeringAngleAction
# ============================================================================


@pytest.fixture(scope="module")
def lagged_env():
    """Shared GKEnv with first-order steering lag enabled (T_steer=0.025)."""
    p = GKEnv.f1tenth_std_vehicle_params()
    p["T_steer"] = 0.025
    env = gym.make(
        "gymkhana:gymkhana-v0",
        config={
            "map": "Spielberg",
            "num_agents": 1,
            "model": "std",
            "observation_config": {"type": "drift"},
            "params": p,
            "normalize_act": True,
            "normalize_obs": True,
            "timestep": 0.01,
        },
    )
    yield env
    env.close()


class TestSteeringAngleFirstOrderLag:
    """Tests for the exact ZOH first-order lag in SteeringAngleAction.

    Covers: bang-bang back-compat when T_steer is absent/zero, exact exponential
    step response, time-to-63% identity, rate saturation cascading correctly into
    the exponential tail, dt-independence in real time, and an end-to-end env
    smoke test confirming monotone non-overshooting approach to s_max.
    """

    # --- Back-compat: bang-bang preserved -----------------------------------

    @pytest.mark.parametrize(
        "T_steer, timestep",
        [
            (None, None),  # T_steer absent, no timestep
            (0.0, 0.01),  # T_steer explicitly zero
            (0.025, None),  # T_steer set but no timestep supplied
        ],
    )
    def test_falls_back_to_bang_bang(self, params, T_steer, timestep):
        p = dict(params) if T_steer is None else dict(params, T_steer=T_steer)
        sa = SteeringAngleAction(params=p, normalize=False, timestep=timestep)

        # When T_steer is absent / zero / has no timestep, act() must match
        # bang_bang_steer exactly. Cover the three sign branches (+, -, 0).
        state = np.zeros(7)
        for desired, current in [(0.4, 0.1), (-0.4, 0.1), (0.1, 0.1)]:
            state[2] = current
            assert sa.act(desired, state=state, params=p) == bang_bang_steer(desired, current, p["sv_max"])

    # --- Exact exponential step response ------------------------------------

    def test_step_response_matches_closed_form(self, params):
        T, dt, delta_ref, N = 0.05, 0.01, 0.4, 50
        p = dict(params, T_steer=T, sv_max=1e6)
        sa = SteeringAngleAction(params=p, normalize=False, timestep=dt)

        # Verify the lag-gain identity through act(): at δ=0 with sv_max
        # effectively unbounded, sv = k·(δ_ref − δ) = k·δ_ref.
        expected_k = (1.0 - np.exp(-dt / T)) / dt
        np.testing.assert_allclose(sa.act(delta_ref, state=np.zeros(7), params=p), expected_k * delta_ref)

        delta, traj = 0.0, [0.0]
        state = np.zeros(7)
        for _ in range(N):
            state[2] = delta
            delta += dt * sa.act(delta_ref, state=state, params=p)
            traj.append(delta)

        analytic = delta_ref * (1.0 - np.exp(-np.arange(N + 1) * dt / T))
        np.testing.assert_allclose(traj, analytic, atol=1e-12, rtol=1e-12)

    # --- Rate saturation cascading into exponential tail --------------------

    def test_rate_clipped_then_exponential_tail(self, params):
        T, dt, delta_ref, sv_max = 0.025, 0.01, 0.5, 3.2
        p = dict(params, T_steer=T, sv_max=sv_max)
        sa = SteeringAngleAction(params=p, normalize=False, timestep=dt)
        alpha = 1.0 - np.exp(-dt / T)

        # Step 0: large error → demand exceeds sv_max, clip binds.
        state = np.zeros(7)
        sv0 = sa.act(delta_ref, state=state, params=p)
        assert sv0 > sv_max

        # Roll forward with the rate clip until the demand drops below sv_max,
        # then verify the next sample matches the exact exponential.
        delta = 0.0
        for _ in range(60):
            state[2] = delta
            sv_raw = sa.act(delta_ref, state=state, params=p)
            if abs(sv_raw) < sv_max:
                np.testing.assert_allclose(sv_raw, alpha * (delta_ref - delta) / dt, rtol=1e-12, atol=1e-12)
                return
            delta += dt * sv_max
        pytest.fail("Rate clip never released within 60 steps")

    # --- Dt-independence in real time ---------------------------------------

    def test_dt_independent_real_time_trajectory(self, params):
        T, delta_ref, total_t = 0.05, 0.4, 0.2
        p = dict(params, T_steer=T, sv_max=1e6)
        analytic = delta_ref * (1.0 - np.exp(-total_t / T))

        for dt in (0.01, 0.005):
            sa = SteeringAngleAction(params=p, normalize=False, timestep=dt)
            delta = 0.0
            state = np.zeros(7)
            for _ in range(int(round(total_t / dt))):
                state[2] = delta
                delta += dt * sa.act(delta_ref, state=state, params=p)
            np.testing.assert_allclose(delta, analytic, atol=1e-12, rtol=1e-12)

    # --- End-to-end env step ------------------------------------------------

    def test_monotonic_approach_to_s_max(self, lagged_env):
        env = lagged_env
        env.reset(seed=0)

        s_max = env.unwrapped.params["s_max"]
        # constant max-left steer, modest throttle
        action = np.array([[1.0, 0.0]], dtype=np.float32)

        deltas = []
        N = 20  # 0.2 s
        for _ in range(N):
            _, _, term, trunc, _ = env.step(action)
            delta = env.unwrapped.sim.agents[0].state[2]
            deltas.append(delta)
            if term or trunc:
                break

        deltas = np.asarray(deltas)
        # Monotone non-decreasing (modulo tiny numerical noise)
        diffs = np.diff(deltas)
        assert np.all(diffs >= -1e-6)
        # Approaching s_max, no overshoot
        assert deltas[-1] <= s_max + 1e-6
        assert deltas[-1] > 0.0


# ============================================================================
# Mid-run param swap via env.configure() — must propagate to act() output
# ============================================================================


class TestConfigureUpdatesActOutput:
    """Verify GKEnv.configure({"params": ...}) updates flow through to each
    action class's act() output via the per-agent action_type reference.

    Regression net for the pre-refactor bug where each subclass cached
    scaling factors (a_max, sv_max, s_max, T_steer-derived k) at __init__
    and silently ignored mid-run param swaps. These tests exercise the exact
    path env.step() takes — agent.action_type.act(...) reading from
    agent.params, both updated via configure() → Simulator.update_params().
    """

    @staticmethod
    def _make_env(control_input, normalize_act, params):
        return gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "model": "std",
                "observation_config": {"type": "drift"},
                "control_input": control_input,
                "params": params,
                "normalize_act": normalize_act,
                "normalize_obs": True,
                "timestep": 0.01,
            },
        )

    def test_accl_action_a_max_update(self):
        p = GKEnv.f1tenth_std_vehicle_params()
        p["a_max"] = 6.0
        env = self._make_env(["accl", "steering_angle"], normalize_act=True, params=p)
        try:
            env.reset(seed=0)
            agent = env.unwrapped.sim.agents[0]
            long_act = agent.action_type._longitudinal_action  # AcclAction (normalized)

            # action=0.5 with a_max=6 → 3.0
            assert long_act.act(0.5, state=agent.state, params=agent.params) == pytest.approx(3.0)

            env.unwrapped.configure({"params": dict(agent.params, a_max=12.0)})

            # action=0.5 with a_max=12 → 6.0
            assert long_act.act(0.5, state=agent.state, params=agent.params) == pytest.approx(6.0)
        finally:
            env.close()

    def test_speed_action_a_max_update(self):
        p = GKEnv.f1tenth_std_vehicle_params()
        p["a_max"] = 6.0
        env = self._make_env(["speed", "steering_angle"], normalize_act=True, params=p)
        try:
            env.reset(seed=0)
            agent = env.unwrapped.sim.agents[0]
            long_act = agent.action_type._longitudinal_action  # SpeedAction (normalized)

            # At state[3]=0 with action=1 → desired=v_max=20, p_accl uses
            # kp = 2·a_max/v_max = 0.6, so accl = kp·20 = 12.0 with a_max=6.
            agent.state[3] = 0.0
            out_pre = long_act.act(1.0, state=agent.state, params=agent.params)
            assert out_pre == pytest.approx(12.0)

            env.unwrapped.configure({"params": dict(agent.params, a_max=12.0)})

            # After doubling a_max, kp doubles → output doubles to 24.0.
            out_post = long_act.act(1.0, state=agent.state, params=agent.params)
            assert out_post == pytest.approx(24.0)
            assert out_post == pytest.approx(2.0 * out_pre)
        finally:
            env.close()

    def test_steering_angle_action_T_steer_update(self):
        p = GKEnv.f1tenth_std_vehicle_params()
        p["T_steer"] = 0.001
        env = self._make_env(["accl", "steering_angle"], normalize_act=False, params=p)
        try:
            env.reset(seed=0)
            agent = env.unwrapped.sim.agents[0]
            steer_act = agent.action_type._steer_action  # SteeringAngleAction

            delta_ref = 0.4
            state = agent.state.copy()
            state[2] = 0.0
            dt = 0.01

            # T_steer=0.001, dt=0.01 → k = (1 - exp(-10))/0.01 ≈ 99.995
            k_pre = (1.0 - np.exp(-dt / 0.001)) / dt
            out_pre = steer_act.act(delta_ref, state=state, params=agent.params)
            assert out_pre == pytest.approx(k_pre * delta_ref)

            env.unwrapped.configure({"params": dict(agent.params, T_steer=0.5)})

            # T_steer=0.5, dt=0.01 → k ≈ 1.98 → ≈50× smaller than pre.
            k_post = (1.0 - np.exp(-dt / 0.5)) / dt
            out_post = steer_act.act(delta_ref, state=state, params=agent.params)
            assert out_post == pytest.approx(k_post * delta_ref)
            assert abs(out_pre) > 10 * abs(out_post), "T_steer change should produce a clearly distinct sv"
        finally:
            env.close()

    def test_steering_speed_action_sv_max_update(self):
        p = GKEnv.f1tenth_std_vehicle_params()
        p["sv_max"] = 3.2
        p["sv_min"] = -3.2  # env asserts sv_min == -sv_max
        env = self._make_env(["accl", "steering_speed"], normalize_act=True, params=p)
        try:
            env.reset(seed=0)
            agent = env.unwrapped.sim.agents[0]
            steer_act = agent.action_type._steer_action  # SteeringSpeedAction (normalized)

            # action=0.5 with sv_max=3.2 → 1.6
            assert steer_act.act(0.5, state=agent.state, params=agent.params) == pytest.approx(1.6)

            env.unwrapped.configure({"params": dict(agent.params, sv_max=8.0, sv_min=-8.0)})

            # action=0.5 with sv_max=8.0 → 4.0
            assert steer_act.act(0.5, state=agent.state, params=agent.params) == pytest.approx(4.0)
        finally:
            env.close()
