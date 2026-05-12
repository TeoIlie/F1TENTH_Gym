"""
Unit tests for recovery training mode in GKEnv.

Tests cover: config initialization, config validation, perturbed reset,
recovery success checking, reward components, and done conditions.
"""

import unittest
from unittest.mock import patch

import gymnasium as gym
import numpy as np

from gymkhana.envs import GKEnv


def _make_recovery_env(**overrides):
    """Create a recovery-mode environment with sensible test defaults."""
    config = {
        "model": "std",
        "params": GKEnv.f1tenth_std_vehicle_params(),
        "num_agents": 1,
        "timestep": 0.01,
        "training_mode": "recover",
        "predictive_collision": False,
        "observation_config": {"type": None},
    }
    config.update(overrides)
    env = gym.make("gymkhana:gymkhana-v0", config=config, render_mode=None)
    return env


class TestRecoveryConfig(unittest.TestCase):
    """Test that recovery config keys are correctly extracted in __init__."""

    def setUp(self):
        self.env = _make_recovery_env()
        self.env.reset(seed=0)  # deterministic recovery perturbations
        self.uw = self.env.unwrapped

    def tearDown(self):
        self.env.close()

    def test_recovery_config_initialization(self):
        """All recovery-specific attributes are set on the unwrapped env."""
        # training mode
        self.assertEqual(self.uw.training_mode, "recover")

        # map and episode length from recovery-specific keys
        self.assertEqual(self.uw.map, "IMS")
        self.assertEqual(self.uw.max_episode_steps, 2048)

        # arc-length bounds
        self.assertEqual(self.uw.recovery_s_init, 96)
        self.assertEqual(self.uw.recovery_s_max, 140)

        # perturbation ranges (lists from default_config)
        self.assertEqual(self.uw.recovery_v_range, [2, 12])
        self.assertIsInstance(self.uw.recovery_beta_range, list)
        self.assertIsInstance(self.uw.recovery_r_range, list)
        self.assertIsInstance(self.uw.recovery_yaw_range, list)

        # reward gains
        self.assertEqual(self.uw.recovery_timestep_penalty, 1.0)
        self.assertEqual(self.uw.recovery_success_reward, 100)
        self.assertEqual(self.uw.recovery_collision_penalty, -50)

        # threshold values exist
        for attr in [
            "recovery_delta_thresh",
            "recovery_beta_thresh",
            "recovery_r_thresh",
            "recovery_d_beta_thresh",
            "recovery_d_r_thresh",
            "recovery_frenet_u_thresh",
        ]:
            self.assertTrue(hasattr(self.uw, attr), f"Missing attribute: {attr}")

        # derivative tracking initialized
        self.assertIsNotNone(self.uw.prev_beta)
        self.assertIsNotNone(self.uw.prev_r)

        # recovery_succeeded flag initialized
        self.assertFalse(self.uw.recovery_succeeded)

    def test_recovery_config_validation(self):
        """num_agents>1 and predictive_collision=True are rejected."""
        with self.assertRaises(ValueError, msg="Should reject num_agents > 1"):
            _make_recovery_env(num_agents=2)

        with self.assertRaises(ValueError, msg="Should reject predictive_collision=True"):
            _make_recovery_env(predictive_collision=True)


class TestRecoveryReset(unittest.TestCase):
    """Test that reset() generates correct perturbed initial states."""

    def setUp(self):
        self.env = _make_recovery_env()
        self.env.reset(seed=0)  # deterministic recovery perturbations
        self.uw = self.env.unwrapped

    def tearDown(self):
        self.env.close()

    def test_recovery_reset_generates_perturbed_state(self):
        """Auto-reset samples random state within configured ranges; explicit state overrides."""
        agent = self.uw.sim.agents[0]
        std = agent.standard_state

        # Position should be near recovery_s_init on the track
        s, _, _ = self.uw.track.cartesian_to_frenet(std["x"], std["y"], std["yaw"], use_raceline=False)
        self.assertAlmostEqual(s, self.uw.recovery_s_init, delta=1.0)

        # Velocity within range (state[3] is v in STD model)
        v = agent.state[3]
        self.assertGreaterEqual(v, self.uw.recovery_v_range[0])
        self.assertLessEqual(v, self.uw.recovery_v_range[1])

        # Slip (beta) within range
        beta = agent.state[6]
        self.assertGreaterEqual(beta, self.uw.recovery_beta_range[0])
        self.assertLessEqual(beta, self.uw.recovery_beta_range[1])

        # Yaw rate within range
        r = agent.state[5]
        self.assertGreaterEqual(r, self.uw.recovery_r_range[0])
        self.assertLessEqual(r, self.uw.recovery_r_range[1])

        # Second reset should produce different state (randomness)
        first_state = agent.state.copy()
        self.env.reset()
        second_state = self.uw.sim.agents[0].state
        self.assertFalse(
            np.allclose(first_state, second_state),
            "Two consecutive resets should produce different states",
        )

        # Explicit state override bypasses random generation
        x, y, base_yaw = self.uw.track.frenet_to_cartesian(self.uw.recovery_s_init, ey=0, ephi=0)
        explicit_state = np.array([[x, y, 0.0, 5.0, base_yaw, 0.0, 0.0]])
        self.env.reset(options={"states": explicit_state})
        state_after = self.uw.sim.agents[0].state
        self.assertAlmostEqual(state_after[3], 5.0, places=4)  # v
        self.assertAlmostEqual(state_after[5], 0.0, places=4)  # r
        self.assertAlmostEqual(state_after[6], 0.0, places=4)  # beta


class TestRecoverySuccess(unittest.TestCase):
    """Test _check_recovery_success() threshold logic."""

    def setUp(self):
        self.env = _make_recovery_env()
        self.env.reset(seed=0)  # deterministic recovery perturbations
        self.uw = self.env.unwrapped

    def tearDown(self):
        self.env.close()

    def test_recovery_success_near_zero_state(self):
        """Near-equilibrium state should satisfy all 6 recovery conditions."""
        # Reset with near-zero perturbation via explicit state
        x, y, base_yaw = self.uw.track.frenet_to_cartesian(self.uw.recovery_s_init, ey=0, ephi=0)
        tiny_beta = 0.001
        tiny_r = 0.001
        states = np.array([[x, y, 0.0, 5.0, base_yaw, tiny_r, tiny_beta]])
        self.env.reset(options={"states": states})

        # After reset, prev_beta and prev_r are set to current values,
        # so derivatives are 0. All conditions should be met.
        self.assertTrue(self.uw._check_recovery_success())

    def test_recovery_success_fails_with_large_perturbation(self):
        """Large beta/r should fail recovery check."""
        x, y, base_yaw = self.uw.track.frenet_to_cartesian(self.uw.recovery_s_init, ey=0, ephi=0)
        large_beta = 0.5  # well above beta_thresh
        large_r = 1.0  # well above r_thresh
        states = np.array([[x, y, 0.0, 5.0, base_yaw, large_r, large_beta]])
        self.env.reset(options={"states": states})

        self.assertFalse(self.uw._check_recovery_success())


class TestRecoveryReward(unittest.TestCase):
    """Test recovery reward components and done conditions."""

    def setUp(self):
        self.env = _make_recovery_env()
        self.env.reset(seed=0)  # deterministic recovery perturbations
        self.uw = self.env.unwrapped

    def tearDown(self):
        self.env.close()

    def test_recovery_reward_components(self):
        """Verify collision, success, and timestep penalty components."""
        dt = self.uw.timestep

        # Case 1: normal step (no collision, no success)
        self.uw.boundary_exceeded[0] = False
        self.uw.recovery_succeeded = False
        reward = self.uw._get_reward()

        expected = -self.uw.recovery_timestep_penalty * dt
        self.assertAlmostEqual(reward, expected, places=6, msg="Normal step reward mismatch")

        # Case 2: boundary crash
        self.uw.boundary_exceeded[0] = True
        self.uw.recovery_succeeded = False
        reward_crash = self.uw._get_reward()

        expected_crash = expected + self.uw.recovery_collision_penalty
        self.assertAlmostEqual(reward_crash, expected_crash, places=6, msg="Crash reward mismatch")

        # Case 3: recovery success (no collision)
        self.uw.boundary_exceeded[0] = False
        self.uw.recovery_succeeded = True
        reward_success = self.uw._get_reward()

        expected_success = expected + self.uw.recovery_success_reward
        self.assertAlmostEqual(reward_success, expected_success, places=6, msg="Success reward mismatch")

    def test_recovery_done_conditions(self):
        """Verify terminated/truncated for boundary, success, and arc-length."""
        # Case 1: boundary exceeded -> terminated, not truncated
        self.uw.boundary_exceeded[0] = True
        with patch.object(self.uw, "_check_recovery_success", return_value=False):
            terminated, truncated, _ = self.uw._check_done()
        self.assertTrue(terminated, "Boundary exceeded should terminate")
        self.assertFalse(truncated, "Boundary exceeded alone should not truncate")
        self.uw.boundary_exceeded[0] = False

        # Case 2: recovery success -> terminated, not truncated
        with patch.object(self.uw, "_check_recovery_success", return_value=True):
            terminated, truncated, _ = self.uw._check_done()
        self.assertTrue(terminated, "Recovery success should terminate")
        self.assertFalse(truncated, "Recovery success alone should not truncate")

        # Case 3: arc-length exceeded -> truncated
        with patch.object(self.uw, "_check_recovery_success", return_value=False):
            # Mock position far past recovery_s_max
            self.uw.poses_x = [0.0]
            self.uw.poses_y = [0.0]
            original_calc = self.uw.track.centerline.spline.calc_arclength_inaccurate
            self.uw.track.centerline.spline.calc_arclength_inaccurate = lambda x, y: (
                self.uw.recovery_s_max + 10,
                0,
            )
            terminated, truncated, _ = self.uw._check_done()
            self.uw.track.centerline.spline.calc_arclength_inaccurate = original_calc
        self.assertFalse(terminated, "Arc-length exceed should not terminate")
        self.assertTrue(truncated, "Arc-length exceed should truncate")

    def test_derivative_tracking_updated_in_step(self):
        """After step(), prev_beta and prev_r should match the agent's post-step state."""
        # Reset with known state so the env is in a clean recovery episode
        x, y, base_yaw = self.uw.track.frenet_to_cartesian(self.uw.recovery_s_init, ey=0, ephi=0)
        states = np.array([[x, y, 0.0, 5.0, base_yaw, 0.1, 0.05]])
        self.env.reset(options={"states": states})

        # Take one step
        action = np.array([[0.0, 0.0]])
        self.env.step(action)

        # prev_beta/prev_r should now reflect the post-step agent state
        agent = self.uw.sim.agents[0]
        self.assertAlmostEqual(self.uw.prev_beta, agent.standard_state["slip"], places=8)
        self.assertAlmostEqual(self.uw.prev_r, agent.standard_state["yaw_rate"], places=8)
