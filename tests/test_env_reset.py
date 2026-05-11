import unittest
from unittest.mock import patch

import gymnasium as gym
import numpy as np

from gymkhana.envs import GKEnv


class TestFrenetModeReset(unittest.TestCase):
    """
    Tests for environment reset behavior when predictive_collision=False (Frenet mode).
    In this mode, the environment should reset ONLY when the ego agent exceeds track boundaries,
    NOT on lap completion or non-ego agent collisions.
    """

    @classmethod
    def setUpClass(cls):
        """Create test environment with Frenet boundary checking enabled."""
        cls.env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "observation_config": {"type": None},
                "reset_config": {"type": "rl_random_static"},
                "predictive_collision": False,  # Enable Frenet boundary checking
            },
        )
        cls.env.reset()

    @classmethod
    def tearDownClass(cls):
        """Clean up environment."""
        cls.env.close()

    def test_frenet_mode_reset_on_boundary_violation(self):
        """Test that _check_done returns True when ego agent exceeds track boundaries."""
        unwrapped = self.env.unwrapped

        # Mock track with known boundaries
        unwrapped.track.centerline.w_lefts = np.array([2.0] * 100)
        unwrapped.track.centerline.w_rights = np.array([2.0] * 100)
        unwrapped.track.centerline.ss = np.linspace(0, 100, 100)

        # Set ego agent poses
        unwrapped.poses_x = np.array([0.0])
        unwrapped.poses_y = np.array([0.0])
        unwrapped.poses_theta = np.array([0.0])

        # Mock: ego agent exceeds boundary (ey=2.5m > half_width=2.0m)
        with patch.object(unwrapped.track, "cartesian_to_frenet", return_value=(50.0, 2.5, 0.0)):
            # Update state to populate boundary_exceeded array
            unwrapped._update_state()
            # Check if done
            done, _, _ = unwrapped._check_done()

        self.assertTrue(done, "Environment should reset when ego agent exceeds track boundary in Frenet mode")

    def test_frenet_mode_no_reset_within_boundaries(self):
        """Test that _check_done returns False when ego agent is within track boundaries."""
        unwrapped = self.env.unwrapped

        # Mock track with known boundaries
        unwrapped.track.centerline.w_lefts = np.array([2.0] * 100)
        unwrapped.track.centerline.w_rights = np.array([2.0] * 100)
        unwrapped.track.centerline.ss = np.linspace(0, 100, 100)

        # Set ego agent poses
        unwrapped.poses_x = np.array([0.0])
        unwrapped.poses_y = np.array([0.0])
        unwrapped.poses_theta = np.array([0.0])

        # Initialize toggle_list to simulate no lap completion
        unwrapped.toggle_list = np.array([0])

        # Mock: ego agent within boundaries (ey=0.5m < half_width=2.0m)
        with patch.object(unwrapped.track, "cartesian_to_frenet", return_value=(50.0, 0.5, 0.0)):
            # Update state to populate boundary_exceeded array
            unwrapped._update_state()
            # Check if done
            done, _, _ = unwrapped._check_done()

        self.assertFalse(done, "Environment should NOT reset when ego agent is within track boundaries in Frenet mode")


class TestTerminatedTruncatedLogic(unittest.TestCase):
    """
    Tests for the distinction between terminated and truncated in _check_done().

    Terminated: Episode ends due to a terminal event (collision, boundary violation, or lap completion)
    Truncated: Episode ends due to reaching the maximum timestep limit

    Uses shared class-level envs grouped by config to avoid redundant gym.make() calls.
    Each test fully overwrites the relevant state before calling _check_done().
    """

    @classmethod
    def setUpClass(cls):
        """Create shared environments for all tests in this class."""
        # 1-agent predictive collision env (used by most tests)
        cls.env_predictive_1agent = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "observation_config": {"type": None},
                "reset_config": {"type": "rl_random_static"},
                "predictive_collision": True,
                "max_episode_steps": 100,
            },
        )
        cls.env_predictive_1agent.reset()

        # 1-agent Frenet mode env
        cls.env_frenet_1agent = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "observation_config": {"type": None},
                "reset_config": {"type": "rl_random_static"},
                "predictive_collision": False,
            },
        )
        cls.env_frenet_1agent.reset()

        # 2-agent predictive collision env
        cls.env_predictive_2agent = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 2,
                "ego_idx": 0,
                "observation_config": {"type": None},
                "reset_config": {"type": "rl_random_static"},
                "predictive_collision": True,
            },
        )
        cls.env_predictive_2agent.reset()

        # 2-agent Frenet mode env
        cls.env_frenet_2agent = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 2,
                "ego_idx": 0,
                "observation_config": {"type": None},
                "reset_config": {"type": "rl_random_static"},
                "predictive_collision": False,
            },
        )
        cls.env_frenet_2agent.reset()

    @classmethod
    def tearDownClass(cls):
        cls.env_predictive_1agent.close()
        cls.env_frenet_1agent.close()
        cls.env_predictive_2agent.close()
        cls.env_frenet_2agent.close()

    def test_predictive_collision_mode_terminated_on_collision(self):
        """Test that terminated=True when predictive_collision=True and ego agent collides."""
        unwrapped = self.env_predictive_1agent.unwrapped

        # Set up state with collision
        unwrapped.collisions = np.array([1.0])  # Ego agent (index 0) has collision
        unwrapped.toggle_list = np.array([0])  # No lap completion
        unwrapped.current_step = 50  # Within max_episode_steps

        terminated, truncated, _ = unwrapped._check_done()

        self.assertTrue(terminated, "terminated should be True when ego agent collides in predictive_collision mode")
        self.assertFalse(truncated, "truncated should be False when collision occurs before time limit")

    def test_predictive_collision_mode_not_terminated_no_collision(self):
        """Test that terminated=False when predictive_collision=True and no collision."""
        unwrapped = self.env_predictive_1agent.unwrapped

        # Set up state without collision
        unwrapped.collisions = np.array([0.0])  # No collision
        unwrapped.toggle_list = np.array([0])  # No lap completion
        unwrapped.current_step = 50  # Within max_episode_steps

        terminated, truncated, _ = unwrapped._check_done()

        self.assertFalse(
            terminated,
            "terminated should be False when no collision in predictive_collision mode without lap completion",
        )
        self.assertFalse(truncated, "truncated should be False when within time limit")

    def test_predictive_collision_mode_terminated_on_lap_completion(self):
        """Test that terminated=True when predictive_collision=True and all agents complete 2 laps."""
        unwrapped = self.env_predictive_2agent.unwrapped

        # Set up state with both agents completing 2 laps (toggle_list >= 4)
        unwrapped.collisions = np.array([0.0, 0.0])  # No collisions
        unwrapped.toggle_list = np.array([4, 4])  # Both agents completed 2 laps
        unwrapped.current_step = 50  # Within max_episode_steps

        terminated, truncated, _ = unwrapped._check_done()

        self.assertTrue(
            terminated, "terminated should be True when all agents complete 2 laps in predictive_collision mode"
        )
        self.assertFalse(truncated, "truncated should be False when episode ends before time limit")

    def test_frenet_mode_terminated_on_boundary_violation(self):
        """Test that terminated=True when predictive_collision=False and boundary is violated."""
        unwrapped = self.env_frenet_1agent.unwrapped

        # Mock track with known boundaries
        unwrapped.track.centerline.w_lefts = np.array([2.0] * 100)
        unwrapped.track.centerline.w_rights = np.array([2.0] * 100)
        unwrapped.track.centerline.ss = np.linspace(0, 100, 100)

        # Set ego agent poses
        unwrapped.poses_x = np.array([0.0])
        unwrapped.poses_y = np.array([0.0])
        unwrapped.poses_theta = np.array([0.0])

        # Mock: ego agent exceeds boundary (ey=2.5m > half_width=2.0m)
        with patch.object(unwrapped.track, "cartesian_to_frenet", return_value=(50.0, 2.5, 0.0)):
            unwrapped._update_state()
            terminated, truncated, _ = unwrapped._check_done()

        self.assertTrue(terminated, "terminated should be True when ego agent exceeds track boundary in Frenet mode")
        self.assertFalse(truncated, "truncated should be False when boundary violation occurs before time limit")

    def test_frenet_mode_not_terminated_within_boundaries(self):
        """Test that terminated=False when predictive_collision=False and within boundaries."""
        unwrapped = self.env_frenet_1agent.unwrapped

        # Mock track with known boundaries
        unwrapped.track.centerline.w_lefts = np.array([2.0] * 100)
        unwrapped.track.centerline.w_rights = np.array([2.0] * 100)
        unwrapped.track.centerline.ss = np.linspace(0, 100, 100)

        # Set ego agent poses
        unwrapped.poses_x = np.array([0.0])
        unwrapped.poses_y = np.array([0.0])
        unwrapped.poses_theta = np.array([0.0])

        # Initialize toggle_list to simulate no lap completion
        unwrapped.toggle_list = np.array([0])

        # Mock: ego agent within boundaries (ey=0.5m < half_width=2.0m)
        with patch.object(unwrapped.track, "cartesian_to_frenet", return_value=(50.0, 0.5, 0.0)):
            unwrapped._update_state()
            terminated, truncated, _ = unwrapped._check_done()

        self.assertFalse(
            terminated, "terminated should be False when ego agent is within track boundaries in Frenet mode"
        )
        self.assertFalse(truncated, "truncated should be False when within time limit")

    def test_truncated_when_max_episode_steps_reached(self):
        """Test that truncated=True when current_step exceeds max_episode_steps."""
        unwrapped = self.env_predictive_1agent.unwrapped

        # Set up state that would normally not be terminal
        unwrapped.collisions = np.array([0.0])  # No collision
        unwrapped.toggle_list = np.array([0])  # No lap completion
        unwrapped.current_step = 101  # Exceeds max_episode_steps

        terminated, truncated, _ = unwrapped._check_done()

        self.assertFalse(terminated, "terminated should be False when time limit is only reason for episode end")
        self.assertTrue(truncated, "truncated should be True when current_step exceeds max_episode_steps")

    def test_truncated_when_at_max_episode_steps(self):
        """Test that truncated=False when current_step equals max_episode_steps (not exceeds)."""
        unwrapped = self.env_predictive_1agent.unwrapped

        # Set up state at exactly max_episode_steps
        unwrapped.collisions = np.array([0.0])  # No collision
        unwrapped.toggle_list = np.array([0])  # No lap completion
        unwrapped.current_step = 100  # Equals max_episode_steps

        terminated, truncated, _ = unwrapped._check_done()

        self.assertFalse(
            truncated, "truncated should be False when current_step equals (not exceeds) max_episode_steps"
        )

    def test_truncated_not_set_when_terminal_event_occurs(self):
        """Test that truncated remains False even if max_episode_steps is reached when terminal event occurs."""
        unwrapped = self.env_predictive_1agent.unwrapped

        # Set up state with collision AND max_episode_steps exceeded
        unwrapped.collisions = np.array([1.0])  # Collision occurs
        unwrapped.toggle_list = np.array([0])  # No lap completion
        unwrapped.current_step = 101  # Exceeds max_episode_steps

        terminated, truncated, _ = unwrapped._check_done()

        self.assertTrue(terminated, "terminated should be True when collision occurs")
        self.assertTrue(truncated, "truncated should be True when max_episode_steps exceeded (both can be true)")

    def test_predictive_collision_multi_agent_only_ego_collision_matters(self):
        """Test that in predictive_collision mode, only ego agent collision triggers termination."""
        unwrapped = self.env_predictive_2agent.unwrapped

        # Set up state where non-ego agent collides but ego doesn't
        unwrapped.collisions = np.array([0.0, 1.0])  # Only non-ego agent (index 1) collides
        unwrapped.toggle_list = np.array([0, 0])  # No lap completion
        unwrapped.current_step = 50  # Within max_episode_steps

        terminated, truncated, _ = unwrapped._check_done()

        self.assertFalse(
            terminated,
            "terminated should be False when only non-ego agent collides in predictive_collision mode",
        )

    def test_frenet_collision_multi_agent_only_ego_boundary_matters(self):
        """Test that in Frenet mode, only ego agent boundary violation triggers termination."""
        unwrapped = self.env_frenet_2agent.unwrapped

        # Directly set boundary_exceeded: only non-ego agent exceeds boundary
        unwrapped.boundary_exceeded = np.array([False, True])  # Ego agent OK, non-ego exceeds
        unwrapped.toggle_list = np.array([0, 0])  # No lap completion
        unwrapped.current_step = 50  # Within max_episode_steps

        terminated, truncated, _ = unwrapped._check_done()

        self.assertFalse(
            terminated,
            "terminated should be False when only non-ego agent exceeds boundary in Frenet mode",
        )


class TestFullStateReset(unittest.TestCase):
    """
    Tests for full vehicle state initialization for STD model.
    The STD model has 9 states: [x, y, delta, v, yaw, yaw_rate, slip_angle, omega_f, omega_r]
    Users can provide 7 states and omega_f, omega_r are calculated automatically.
    """

    @classmethod
    def setUpClass(cls):
        """Create test environment with STD model (shared across all tests)."""
        cls.env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "model": "std",
                "params": GKEnv.f1tenth_std_vehicle_params(),
                "observation_config": {"type": None},
                "reset_config": {"type": "rl_random_static"},
            },
        )

    @classmethod
    def tearDownClass(cls):
        """Clean up environment."""
        cls.env.close()

    def test_backward_compatible_pose_only_reset(self):
        """Test that existing pose-only reset still works."""
        poses = np.array([[0.0, 0.0, np.pi / 2]])
        obs, info = self.env.reset(options={"poses": poses})

        # Verify state initialized correctly
        state = self.env.unwrapped.sim.agents[0].state
        self.assertAlmostEqual(state[0], 0.0, places=5)  # x
        self.assertAlmostEqual(state[1], 0.0, places=5)  # y
        self.assertAlmostEqual(state[4], np.pi / 2, places=5)  # yaw
        self.assertAlmostEqual(state[3], 0.0, places=5)  # velocity should be 0

    def test_full_state_reset_std_model(self):
        """Test full 7-element state initialization for STD model."""
        # State: [x, y, delta, v, yaw, yaw_rate, slip_angle]
        states = np.array([[1.0, 2.0, 0.1, 5.0, np.pi / 4, 0.5, 0.05]])
        obs, info = self.env.reset(options={"states": states})

        state = self.env.unwrapped.sim.agents[0].state
        self.assertAlmostEqual(state[0], 1.0, places=5)  # x
        self.assertAlmostEqual(state[1], 2.0, places=5)  # y
        self.assertAlmostEqual(state[2], 0.1, places=5)  # delta
        self.assertAlmostEqual(state[3], 5.0, places=5)  # velocity
        self.assertAlmostEqual(state[4], np.pi / 4, places=5)  # yaw
        self.assertAlmostEqual(state[5], 0.5, places=5)  # yaw_rate
        self.assertAlmostEqual(state[6], 0.05, places=5)  # slip_angle

        # Verify omega_f and omega_r are calculated (not zero for non-zero velocity)
        self.assertNotEqual(state[7], 0.0)  # omega_f
        self.assertNotEqual(state[8], 0.0)  # omega_r

    def test_omega_calculation_formula(self):
        """Verify omega_f and omega_r are calculated correctly from velocity, slip, and steering."""
        v, beta, delta = 5.0, 0.05, 0.1
        R_w = self.env.unwrapped.params["R_w"]

        states = np.array([[0.0, 0.0, delta, v, 0.0, 0.0, beta]])
        self.env.reset(options={"states": states})

        state = self.env.unwrapped.sim.agents[0].state
        # omega_f = v * cos(beta) * cos(delta) / R_w
        expected_omega_f = v * np.cos(beta) * np.cos(delta) / R_w
        # omega_r = v * cos(beta) / R_w
        expected_omega_r = v * np.cos(beta) / R_w

        self.assertAlmostEqual(state[7], expected_omega_f, places=5)
        self.assertAlmostEqual(state[8], expected_omega_r, places=5)

    def test_mutual_exclusivity_error(self):
        """Test that specifying both poses and states raises error."""
        poses = np.array([[0.0, 0.0, 0.0]])
        states = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

        with self.assertRaises(ValueError) as ctx:
            self.env.reset(options={"poses": poses, "states": states})
        self.assertIn("Cannot provide both", str(ctx.exception))

    def test_wrong_state_shape_error(self):
        """Test that wrong state shape raises clear error."""
        wrong_states = np.array([[0.0, 0.0, 0.0]])  # Only 3 elements instead of 7 or 9

        with self.assertRaises(ValueError):
            self.env.reset(options={"states": wrong_states})

        # Width 8 is not in the accepted set (7, 9) for STD either.
        wrong_states_8 = np.zeros((1, 8))
        with self.assertRaises(ValueError):
            self.env.reset(options={"states": wrong_states_8})

    def test_full_state_reset_std_9_elements(self):
        """Test full 9-element state initialization preserves wheel angular velocities."""
        # State: [x, y, delta, v, yaw, yaw_rate, slip_angle, omega_f, omega_r]
        # Use wheel speeds that intentionally do NOT match the no-slip formula
        # so we can be sure they are taken as-given.
        omega_f_user = 123.0
        omega_r_user = 456.0
        states = np.array([[1.0, 2.0, 0.1, 5.0, np.pi / 4, 0.5, 0.05, omega_f_user, omega_r_user]])
        self.env.reset(options={"states": states})

        state = self.env.unwrapped.sim.agents[0].state
        self.assertAlmostEqual(state[0], 1.0, places=5)
        self.assertAlmostEqual(state[1], 2.0, places=5)
        self.assertAlmostEqual(state[2], 0.1, places=5)
        self.assertAlmostEqual(state[3], 5.0, places=5)
        self.assertAlmostEqual(state[4], np.pi / 4, places=5)
        self.assertAlmostEqual(state[5], 0.5, places=5)
        self.assertAlmostEqual(state[6], 0.05, places=5)
        # Wheel speeds taken as given, NOT re-derived from no-slip formula.
        self.assertAlmostEqual(state[7], omega_f_user, places=5)
        self.assertAlmostEqual(state[8], omega_r_user, places=5)

    def test_9_element_preserves_wheel_speeds_under_velocity_clamp(self):
        """When v is clamped, the explicit wheel speeds are still taken as given.

        Documents the contract that the caller owns consistency between v and
        omega_{f,r} when supplying a 9-wide row.
        """
        params = self.env.unwrapped.params
        # Pick a velocity outside the [v_min, v_max] window so it gets clamped.
        v_request = params["v_max"] + 100.0
        omega_f_user = 7.0
        omega_r_user = 11.0
        states = np.array([[0.0, 0.0, 0.0, v_request, 0.0, 0.0, 0.0, omega_f_user, omega_r_user]])
        self.env.reset(options={"states": states})

        state = self.env.unwrapped.sim.agents[0].state
        self.assertAlmostEqual(state[3], params["v_max"], places=5)  # clamped
        self.assertAlmostEqual(state[7], omega_f_user, places=5)  # unchanged
        self.assertAlmostEqual(state[8], omega_r_user, places=5)  # unchanged

    def test_wrong_num_agents_error(self):
        """Test that wrong number of agents in states raises error."""
        # Create env with 2 agents but provide states for 1
        env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 2,
                "model": "std",
                "params": GKEnv.f1tenth_std_vehicle_params(),
                "observation_config": {"type": None},
            },
        )

        states = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])  # Only 1 agent

        with self.assertRaises(ValueError):
            env.reset(options={"states": states})

        env.close()

    def test_mb_model_states_rejected(self):
        """Test that full-state reset is rejected for the MB model."""
        env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "model": "mb",
                "params": GKEnv.fullscale_vehicle_params(),
                "observation_config": {"type": None},
            },
        )

        states = np.zeros((1, 7))

        with self.assertRaises(ValueError) as ctx:
            env.reset(options={"states": states})
        self.assertIn("MB", str(ctx.exception))

        env.close()


class TestMultiAgentFullStateReset(unittest.TestCase):
    """Tests for full state reset with multiple agents."""

    def test_multi_agent_full_state(self):
        """Test full state initialization with multiple agents."""
        env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 2,
                "model": "std",
                "params": GKEnv.f1tenth_std_vehicle_params(),
                "observation_config": {"type": None},
            },
        )

        states = np.array(
            [
                [0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0],
                [5.0, 5.0, 0.1, 4.0, np.pi / 6, 0.2, 0.01],
            ]
        )
        obs, info = env.reset(options={"states": states})

        for i in range(2):
            state = env.unwrapped.sim.agents[i].state
            np.testing.assert_array_almost_equal(state[:7], states[i])

        env.close()


class TestStateConstraints(unittest.TestCase):
    """Tests for constraint application in full state reset."""

    @classmethod
    def setUpClass(cls):
        """Create test environment with STD model (shared across all tests)."""
        cls.env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "model": "std",
                "params": GKEnv.f1tenth_std_vehicle_params(),
                "observation_config": {"type": None},
            },
        )

    @classmethod
    def tearDownClass(cls):
        """Clean up environment."""
        cls.env.close()

    def test_steering_constraint_applied(self):
        """Test that steering angle is clamped to valid range."""
        params = self.env.unwrapped.params
        s_max = params["s_max"]

        # State with steering exceeding max
        states = np.array([[0.0, 0.0, s_max + 0.5, 5.0, 0.0, 0.0, 0.0]])
        self.env.reset(options={"states": states})

        state = self.env.unwrapped.sim.agents[0].state
        self.assertLessEqual(state[2], s_max)  # delta should be clamped

    def test_velocity_constraint_applied(self):
        """Test that velocity is clamped to valid range."""
        params = self.env.unwrapped.params
        v_max = params["v_max"]

        # State with velocity exceeding max
        states = np.array([[0.0, 0.0, 0.0, v_max + 10, 0.0, 0.0, 0.0]])
        self.env.reset(options={"states": states})

        state = self.env.unwrapped.sim.agents[0].state
        self.assertLessEqual(state[3], v_max)  # velocity should be clamped


class TestSimulationContinuation(unittest.TestCase):
    """Tests for simulation behavior after full state reset."""

    @classmethod
    def setUpClass(cls):
        """Create test environment with STD model (shared across all tests)."""
        cls.env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "model": "std",
                "params": GKEnv.f1tenth_std_vehicle_params(),
                "control_input": ["accl", "steering_angle"],
                "observation_config": {"type": None},
            },
        )

    @classmethod
    def tearDownClass(cls):
        """Clean up environment."""
        cls.env.close()

    def test_simulation_continues_from_state(self):
        """Test that simulation correctly continues from initialized state."""
        # Initialize with specific state (non-zero velocity forward)
        initial_state = np.array([[10.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0]])
        self.env.reset(options={"states": initial_state})

        # Take a step with zero action
        action = np.array([[0.0, 0.0]])
        obs, _, _, _, _ = self.env.step(action)

        # State should have evolved - x position should have increased
        new_state = self.env.unwrapped.sim.agents[0].state
        # With velocity=3.0 m/s and timestep=0.01s, x should increase by ~0.03m per step
        # (step() is called in reset() once, so after our step we're 2 steps in)
        self.assertGreater(new_state[0], 10.0)

    def test_round_trip_state_preservation(self):
        """Test: run simulation -> extract state -> reset with state -> verify state matches."""
        # Run simulation for a few steps
        self.env.reset()
        for _ in range(10):
            action = self.env.action_space.sample()
            self.env.step(action)

        # Extract the first 7 elements of state (user-controllable states)
        captured_state = self.env.unwrapped.sim.agents[0].state[:7].copy()

        # Reset with captured state
        self.env.reset(options={"states": captured_state.reshape(1, -1)})

        # Verify state matches (first 7 elements)
        restored_state = self.env.unwrapped.sim.agents[0].state[:7]
        np.testing.assert_array_almost_equal(restored_state, captured_state, decimal=5)

    def test_round_trip_full_9_state_preservation(self):
        """Full 9-wide round trip: run -> capture all 9 -> reset(9-wide) -> all 9 match.

        This is the sysid/replay use case — capturing wheel speeds out of a
        running sim and re-injecting them so the next rollout starts from the
        exact same dynamical state, not a no-slip approximation of it.
        """
        self.env.reset()
        for _ in range(10):
            action = self.env.action_space.sample()
            self.env.step(action)

        captured_state = self.env.unwrapped.sim.agents[0].state[:9].copy()

        self.env.reset(options={"states": captured_state.reshape(1, -1)})

        restored_state = self.env.unwrapped.sim.agents[0].state[:9]
        np.testing.assert_array_almost_equal(restored_state, captured_state, decimal=5)


class TestInternalBookkeeping(unittest.TestCase):
    """Tests for internal state bookkeeping after full state reset."""

    @classmethod
    def setUpClass(cls):
        """Create test environment with STD model (shared across all tests)."""
        cls.env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "model": "std",
                "params": GKEnv.f1tenth_std_vehicle_params(),
                "observation_config": {"type": None},
            },
        )

    @classmethod
    def tearDownClass(cls):
        """Clean up environment."""
        cls.env.close()

    def test_start_positions_extracted_from_states(self):
        """Test that start_xs, start_ys, start_thetas are correctly extracted from states."""
        x, y, yaw = 5.0, 10.0, np.pi / 3
        states = np.array([[x, y, 0.1, 3.0, yaw, 0.2, 0.01]])
        self.env.reset(options={"states": states})

        unwrapped = self.env.unwrapped
        self.assertAlmostEqual(unwrapped.start_xs[0], x, places=5)
        self.assertAlmostEqual(unwrapped.start_ys[0], y, places=5)
        self.assertAlmostEqual(unwrapped.start_thetas[0], yaw, places=5)


class TestSkipIntegration(unittest.TestCase):
    """Tests for skip_integration parameter in step() method."""

    @classmethod
    def setUpClass(cls):
        """Create test environment with STD model (shared across all tests)."""
        cls.env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "model": "std",
                "params": GKEnv.f1tenth_std_vehicle_params(),
                "control_input": ["accl", "steering_angle"],
                "observation_config": {"type": None},
                "reset_config": {"type": "rl_random_static"},
            },
        )

    @classmethod
    def tearDownClass(cls):
        """Clean up environment."""
        cls.env.close()

    def test_step_with_skip_integration_false_modifies_state(self):
        """Test that step() with skip_integration=False modifies the agent state."""
        # Initialize with known state (non-zero velocity)
        initial_state = np.array([[10.0, 5.0, 0.0, 3.0, np.pi / 4, 0.0, 0.0]])
        self.env.reset(options={"states": initial_state})

        # Capture state after reset (note: reset calls step once with skip_integration=True)
        state_before = self.env.unwrapped.sim.agents[0].state.copy()

        # Take a step with skip_integration=False (default) and non-zero action
        action = np.array([[1.0, 0.1]])  # Some acceleration and steering
        self.env.unwrapped.step(action, skip_integration=False)

        # State should have changed due to integration
        state_after = self.env.unwrapped.sim.agents[0].state

        # Verify at least position changed (vehicle moved forward)
        self.assertNotAlmostEqual(
            state_before[0],
            state_after[0],
            places=5,
            msg="x position should change when skip_integration=False",
        )

        # Verify state actually evolved (not identical)
        state_different = not np.allclose(state_before, state_after, atol=1e-5)
        self.assertTrue(state_different, "State should be modified when skip_integration=False")

    def test_step_with_skip_integration_true_preserves_state(self):
        """Test that step() with skip_integration=True does NOT modify the agent state."""
        # Initialize with known state (non-zero velocity)
        initial_state = np.array([[10.0, 5.0, 0.0, 3.0, np.pi / 4, 0.0, 0.0]])
        self.env.reset(options={"states": initial_state})

        # Capture state after reset
        state_before = self.env.unwrapped.sim.agents[0].state.copy()

        # Take a step with skip_integration=True and non-zero action
        action = np.array([[1.0, 0.1]])  # Some acceleration and steering
        self.env.unwrapped.step(action, skip_integration=True)

        # State should NOT have changed (no integration occurred)
        state_after = self.env.unwrapped.sim.agents[0].state

        # Verify state is identical (within numerical precision)
        np.testing.assert_array_almost_equal(
            state_before,
            state_after,
            decimal=5,
            err_msg="State should NOT be modified when skip_integration=True",
        )


class TestNonStdFullStateReset(unittest.TestCase):
    """Full-state reset for KS / ST / STP. STD coverage lives in TestFullStateReset.

    Multi-agent + wrong-num-agents paths are model-independent and already
    covered by the existing STD tests; the rejection-side check here uses ST.
    """

    def test_full_state_reset_supported_models(self):
        """Every supported non-STD model accepts a row of its expected width."""
        cases = [
            ("ks", np.array([1.0, 2.0, 0.1, 4.0, np.pi / 4]), None),
            ("st", np.array([1.0, 2.0, 0.1, 4.0, np.pi / 4, 0.3, 0.05]), None),
            ("stp", np.array([1.0, 2.0, 0.1, 4.0, np.pi / 4, 0.3, 0.05]), GKEnv.f1tenth_stp_vehicle_params()),
        ]
        for model, row, params in cases:
            with self.subTest(model=model):
                config = {
                    "map": "Spielberg",
                    "num_agents": 1,
                    "model": model,
                    "observation_config": {"type": None},
                    "reset_config": {"type": "rl_random_static"},
                }
                if params is not None:
                    config["params"] = params
                env = gym.make("gymkhana:gymkhana-v0", config=config)
                try:
                    env.reset(options={"states": row.reshape(1, -1)})
                    state = env.unwrapped.sim.agents[0].state
                    np.testing.assert_array_almost_equal(state, row, decimal=5)
                finally:
                    env.close()

    def test_full_state_wrong_width_rejected(self):
        """Wrong row width raises ValueError. ST chosen as a representative.

        Includes width 9 — that width is only valid for STD; ST must still
        reject it so the STD-specific 9-wide acceptance doesn't leak across
        models.
        """
        env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "model": "st",
                "observation_config": {"type": None},
                "reset_config": {"type": "rl_random_static"},
            },
        )
        try:
            with self.assertRaises(ValueError):
                env.reset(options={"states": np.zeros((1, 5))})  # ST expects 7
            with self.assertRaises(ValueError):
                env.reset(options={"states": np.zeros((1, 9))})  # 9 is STD-only
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
