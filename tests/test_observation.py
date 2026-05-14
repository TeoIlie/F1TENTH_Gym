import unittest

import gymnasium as gym
import numpy as np
from gymnasium.spaces import Box

from gymkhana.envs import GKEnv
from gymkhana.envs.observation import (
    observation_factory,
    sample_lookahead_curvatures_fast,
    sample_lookahead_widths_fast,
)
from gymkhana.envs.utils import deep_update
from train.config.env_config import get_drift_train_config, get_env_id


class TestObservationInterface(unittest.TestCase):
    @staticmethod
    def _make_env(config={}) -> GKEnv:
        conf = {
            "map": "Spielberg",
            "num_agents": 1,
            "timestep": 0.01,
            "integrator": "rk4",
            "control_input": ["speed", "steering_angle"],
            "params": {"mu": 1.0},
        }
        conf = deep_update(conf, config)

        env = gym.make("gymkhana:gymkhana-v0", config=conf)
        return env

    def test_original_obs_space(self):
        """
        Check backward compatibility with the original observation space.
        """
        env = self._make_env(config={"observation_config": {"type": "original"}})

        obs, _ = env.reset()

        obs_keys = [
            "ego_idx",
            "scans",
            "poses_x",
            "poses_y",
            "poses_theta",
            "linear_vels_x",
            "linear_vels_y",
            "ang_vels_z",
            "collisions",
            "lap_times",
            "lap_counts",
        ]

        # check that the observation space has the correct types
        self.assertTrue(all([isinstance(env.observation_space.spaces[k], Box) for k in obs_keys if k != "ego_idx"]))
        self.assertTrue(all([env.observation_space.spaces[k].dtype == np.float32 for k in obs_keys if k != "ego_idx"]))

        # check the observation space is a dict
        self.assertTrue(isinstance(obs, dict))

        # check that the observation has the correct keys
        self.assertTrue(all([k in obs for k in obs_keys]))
        self.assertTrue(all([k in obs_keys for k in obs]))
        self.assertTrue(env.observation_space.contains(obs))

    def test_features_observation(self):
        """
        Check the FeatureObservation allows to select an arbitrary subset of features.
        """
        features = ["pose_x", "pose_y", "pose_theta"]

        env = self._make_env(config={"observation_config": {"type": "features", "features": features}})

        # check the observation space is a dict
        self.assertTrue(isinstance(env.observation_space, gym.spaces.Dict))

        # check that the observation space has the correct keys
        for agent_id in env.unwrapped.agent_ids:
            space = env.observation_space.spaces[agent_id].spaces
            self.assertTrue(all([k in space for k in features]))
            self.assertTrue(all([k in features for k in space]))

        # check that the observation space has the correct types
        for agent_id in env.unwrapped.agent_ids:
            space = env.observation_space.spaces[agent_id].spaces
            self.assertTrue(all([isinstance(space[k], Box) for k in features]))
            self.assertTrue(all([space[k].dtype == np.float32 for k in features]))

        # check the actual observation
        obs, _ = env.reset()
        obs, _, _, _, _ = env.step(env.action_space.sample())

        for i, agent_id in enumerate(env.unwrapped.agent_ids):
            pose_x, pose_y, pose_theta = env.unwrapped.sim.agent_poses[i]
            obs_x, obs_y, obs_theta = (
                obs[agent_id]["pose_x"],
                obs[agent_id]["pose_y"],
                obs[agent_id]["pose_theta"],
            )

            for ground_truth, observation in zip([pose_x, pose_y, pose_theta], [obs_x, obs_y, obs_theta]):
                self.assertTrue(np.allclose(ground_truth, observation))

    def test_unexisting_obs_space(self):
        """
        Check that an error is raised when an unexisting observation type is requested.
        """
        env = self._make_env()
        with self.assertRaises(ValueError):
            observation_factory(env, vehicle_id=0, type="unexisting_obs_type")

    def test_kinematic_obs_space(self):
        """
        Check the kinematic state observation space contains the correct features [x, y, theta, v].
        """
        env = self._make_env(config={"observation_config": {"type": "kinematic_state"}})

        kinematic_features = ["pose_x", "pose_y", "pose_theta", "linear_vel_x", "delta"]

        # check kinematic features are in the observation space
        for agent_id in env.unwrapped.agent_ids:
            space = env.observation_space.spaces[agent_id].spaces
            self.assertTrue(all([k in space for k in kinematic_features]))
            self.assertTrue(all([k in kinematic_features for k in space]))

        # check the actual observation
        obs, _ = env.reset()
        obs, _, _, _, _ = env.step(env.action_space.sample())

        for i, agent_id in enumerate(env.unwrapped.agent_ids):
            pose_x, pose_y, _, velx, pose_theta, _, _ = env.unwrapped.sim.agents[i].state
            obs_x, obs_y, obs_theta = (
                obs[agent_id]["pose_x"],
                obs[agent_id]["pose_y"],
                obs[agent_id]["pose_theta"],
            )
            obs_velx = obs[agent_id]["linear_vel_x"]

            for ground_truth, observed in zip([pose_x, pose_y, pose_theta, velx], [obs_x, obs_y, obs_theta, obs_velx]):
                self.assertTrue(np.allclose(ground_truth, observed))

    def test_dynamic_obs_space(self):
        """
        Check the dynamic state observation space contains the correct features.
        """
        env = self._make_env(config={"observation_config": {"type": "dynamic_state"}})

        kinematic_features = [
            "pose_x",
            "pose_y",
            "pose_theta",
            "linear_vel_x",
            "linear_vel_y",
            "ang_vel_z",
            "delta",
            "beta",
        ]

        # check kinematic features are in the observation space
        for agent_id in env.unwrapped.agent_ids:
            space = env.observation_space.spaces[agent_id].spaces
            self.assertTrue(all([k in space for k in kinematic_features]))
            self.assertTrue(all([k in kinematic_features for k in space]))

        # check the actual observation
        obs, _ = env.reset()
        obs, _, _, _, _ = env.step(env.action_space.sample())

        for i, agent_id in enumerate(env.unwrapped.agent_ids):
            std_state = env.unwrapped.sim.agents[i].standard_state
            pose_x = std_state["x"]
            pose_y = std_state["y"]
            pose_theta = std_state["yaw"]
            velx = std_state["v_x"]
            vely = std_state["v_y"]
            delta = std_state["delta"]
            ang_vel_z = std_state["yaw_rate"]
            beta = std_state["slip"]

            agent_obs = obs[agent_id]
            obs_x, obs_y, obs_theta = (
                agent_obs["pose_x"],
                agent_obs["pose_y"],
                agent_obs["pose_theta"],
            )
            obs_velx, obs_vely, obs_delta, obs_beta, obs_ang_vel_z = (
                agent_obs["linear_vel_x"],
                agent_obs["linear_vel_y"],
                agent_obs["delta"],
                agent_obs["beta"],
                agent_obs["ang_vel_z"],
            )

            for ground_truth, observed in zip(
                [pose_x, pose_y, pose_theta, velx, vely, delta, beta, ang_vel_z],
                [obs_x, obs_y, obs_theta, obs_velx, obs_vely, obs_delta, obs_beta, obs_ang_vel_z],
            ):
                self.assertTrue(np.allclose(ground_truth, observed))

    def test_consistency_observe_space(self):
        obs_type_ids = ["kinematic_state", "dynamic_state", "original"]

        env = self._make_env()
        env.reset()

        for obs_type_id in obs_type_ids:
            obs_type = observation_factory(env, type=obs_type_id)
            space = obs_type.space()
            observation = obs_type.observe()

            self.assertTrue(
                space.contains(observation),
                f"Observation {obs_type_id} is not contained in its space",
            )

    def test_gymnasium_api(self):
        from gymnasium.utils.env_checker import check_env

        obs_type_ids = ["kinematic_state", "dynamic_state", "original"]

        for obs_type_id in obs_type_ids:
            env = self._make_env(config={"observation_config": {"type": obs_type_id}})
            check_env(
                env.unwrapped,
                skip_render_check=True,
            )


class TestDriftObservation(unittest.TestCase):
    """Test suite for drift observation type"""

    @classmethod
    def setUpClass(cls):
        """Create environment once for all tests"""
        # Use the exact config from drift_debug.py
        cls.config = get_drift_train_config()
        # Ensure normalization is off for easier testing
        cls.config["normalize_obs"] = False
        cls.config["sparse_width_obs"] = False
        # Pin to "drift"; yaml default may differ (e.g. drift_real) and break feature indexing.
        cls.config["observation_config"] = {"type": "drift"}
        cls.lookahead_n_points = cls.config["lookahead_n_points"]
        cls.lookahead_ds = cls.config["lookahead_ds"]

    def setUp(self):
        """Create fresh environment for each test"""
        self.env = gym.make(get_env_id(), config=self.config)
        self.env.reset()

    def tearDown(self):
        """Clean up environment after each test"""
        self.env.close()

    def test_observation_space_shape(self):
        """Test that drift observation has correct shape"""
        obs, _ = self.env.reset()

        # Calculate expected size based on drift features
        expected_size = (
            1  # linear_vel_x
            + 1  # linear_vel_y
            + 1  # frenet_u
            + 1  # frenet_n
            + 1  # ang_vel_z
            + 1  # beta
            + 1  # delta
            + 1  # prev_steering_cmd
            + 1  # prev_throttle_cmd
            + 1  # prev_avg_wheel_omega
            + 1  # integrated_vel_cmd
            + self.lookahead_n_points  # lookahead_curvatures
            + self.lookahead_n_points  # lookahead_widths
        )

        self.assertEqual(obs.shape[0], expected_size, f"Expected obs size {expected_size}, got {obs.shape[0]}")
        self.assertEqual(obs.dtype, np.float32, "Observation should be float32")

    def test_linear_vel_x(self):
        """Test that linear_vel_x holds current longitudinal velocity from standardized state"""
        obs, _ = self.env.reset()

        # Step once to get meaningful velocities
        action = np.array([[0.0, 0.2]])  # [steering, acceleration]
        obs, _, _, _, _ = self.env.step(action)

        # Get ground truth from agent's standardized state
        agent = self.env.unwrapped.sim.agents[0]
        std_state = agent.standard_state
        expected_vx = std_state["v_x"]

        # Extract from observation (first element)
        observed_vx = obs[0]

        self.assertAlmostEqual(
            observed_vx, expected_vx, places=5, msg=f"linear_vel_x mismatch: expected {expected_vx}, got {observed_vx}"
        )

    def test_linear_vel_y(self):
        """Test that linear_vel_y holds current lateral velocity from standardized state"""
        obs, _ = self.env.reset()

        # Step once to get meaningful velocities
        action = np.array([[0.0, 0.2]])
        obs, _, _, _, _ = self.env.step(action)

        # Get ground truth from agent's standardized state
        agent = self.env.unwrapped.sim.agents[0]
        std_state = agent.standard_state
        expected_vy = std_state["v_y"]

        # Extract from observation (second element)
        observed_vy = obs[1]

        self.assertAlmostEqual(
            observed_vy, expected_vy, places=5, msg=f"linear_vel_y mismatch: expected {expected_vy}, got {observed_vy}"
        )

    def test_frenet_u(self):
        """Test that frenet_u holds heading error (ephi) from track.cartesian_to_frenet"""
        obs, _ = self.env.reset()

        # Step to get non-zero state
        action = np.array([[0.1, 0.2]])
        obs, _, _, _, _ = self.env.step(action)

        # Get ground truth from track projection
        agent = self.env.unwrapped.sim.agents[0]
        std_state = agent.standard_state
        x, y, theta = std_state["x"], std_state["y"], std_state["yaw"]

        track = self.env.unwrapped.track
        s, ey, ephi = track.cartesian_to_frenet(x, y, theta, use_raceline=False)

        # Extract from observation (third element)
        observed_u = obs[2]

        self.assertAlmostEqual(
            observed_u, ephi, places=5, msg=f"frenet_u (ephi) mismatch: expected {ephi}, got {observed_u}"
        )

    def test_frenet_n(self):
        """Test that frenet_n holds lateral distance (ey) from track.cartesian_to_frenet"""
        obs, _ = self.env.reset()

        # Step to get non-zero state
        action = np.array([[0.1, 0.2]])
        obs, _, _, _, _ = self.env.step(action)

        # Get ground truth from track projection
        agent = self.env.unwrapped.sim.agents[0]
        std_state = agent.standard_state
        x, y, theta = std_state["x"], std_state["y"], std_state["yaw"]

        track = self.env.unwrapped.track
        s, ey, ephi = track.cartesian_to_frenet(x, y, theta, use_raceline=False)

        # Extract from observation (fourth element)
        observed_n = obs[3]

        self.assertAlmostEqual(observed_n, ey, places=5, msg=f"frenet_n (ey) mismatch: expected {ey}, got {observed_n}")

    def test_ang_vel_z(self):
        """Test that ang_vel_z holds current yaw rate from standardized state"""
        obs, _ = self.env.reset()

        # Step with steering to induce yaw rate
        action = np.array([[0.3, 0.5]])
        for _ in range(5):  # Multiple steps to build up yaw rate
            obs, _, _, _, _ = self.env.step(action)

        # Get ground truth from agent's standardized state
        agent = self.env.unwrapped.sim.agents[0]
        std_state = agent.standard_state
        expected_yaw_rate = std_state["yaw_rate"]

        # Extract from observation (fifth element)
        observed_yaw_rate = obs[4]

        self.assertAlmostEqual(
            observed_yaw_rate,
            expected_yaw_rate,
            places=5,
            msg=f"ang_vel_z mismatch: expected {expected_yaw_rate}, got {observed_yaw_rate}",
        )

    def test_delta(self):
        """Test that delta holds current steering angle from standardized state"""
        obs, _ = self.env.reset()

        # Step with specific steering command
        steering_cmd = 0.25
        action = np.array([[steering_cmd, 0.2]])
        for _ in range(3):  # Multiple steps for steering to reach commanded value
            obs, _, _, _, _ = self.env.step(action)

        # Get ground truth from agent's standardized state
        agent = self.env.unwrapped.sim.agents[0]
        std_state = agent.standard_state
        expected_delta = std_state["delta"]

        # Extract from observation (sixth element)
        observed_delta = obs[5]

        self.assertAlmostEqual(
            observed_delta,
            expected_delta,
            places=5,
            msg=f"delta mismatch: expected {expected_delta}, got {observed_delta}",
        )

    def test_prev_steering_cmd(self):
        """Test that prev_steering_cmd holds steering command from previous time step"""
        obs, _ = self.env.reset()

        # First action
        first_steering = 0.15
        action1 = np.array([[first_steering, 0.2]])
        obs1, _, _, _, _ = self.env.step(action1)

        # Second action (different steering)
        second_steering = 0.30
        action2 = np.array([[second_steering, 0.2]])
        obs2, _, _, _, _ = self.env.step(action2)

        # After second step, prev_steering_cmd should equal first_steering
        # Extract from observation
        observed_prev_steer = obs2[7]

        self.assertAlmostEqual(
            observed_prev_steer,
            first_steering,
            places=5,
            msg=f"prev_steering_cmd should be {first_steering}, got {observed_prev_steer}",
        )

    def test_prev_throttle_cmd(self):
        """Test that prev_throttle_cmd holds the raw throttle command from the previous step."""
        obs, _ = self.env.reset()

        # First action with specific throttle value
        first_throttle = 0.3
        action1 = np.array([[0.0, first_throttle]])
        obs1, _, _, _, _ = self.env.step(action1)

        # Second action with different throttle
        second_throttle = 0.6
        action2 = np.array([[0.0, second_throttle]])
        obs2, _, _, _, _ = self.env.step(action2)

        # prev_throttle_cmd is the raw value, no conversion. Index 8 in the drift feature list.
        observed_prev_throttle = obs2[8]

        self.assertAlmostEqual(
            observed_prev_throttle,
            first_throttle,
            places=5,
            msg=f"prev_throttle_cmd should be {first_throttle}, got {observed_prev_throttle}",
        )

    def test_prev_avg_wheel_omega(self):
        """Test that prev_avg_wheel_omega holds average wheel speed from previous time step"""
        obs, _ = self.env.reset()

        # Step multiple times to build up wheel angular velocities
        action = np.array([[0.0, 0.5]])
        for _ in range(3):
            obs, _, _, _, _ = self.env.step(action)

        # Get current wheel omegas to verify they're non-zero
        agent = self.env.unwrapped.sim.agents[0]
        curr_avg = agent.curr_avg_wheel_omega

        # Step again
        obs_next, _, _, _, _ = self.env.step(action)

        # After next step, prev_avg_wheel_omega should equal curr from previous step
        # Extract from observation
        observed_prev_omega = obs_next[9]

        # The prev value in next step should equal curr value from this step
        self.assertAlmostEqual(
            observed_prev_omega,
            curr_avg,
            places=5,
            msg=f"prev_avg_wheel_omega should be {curr_avg}, got {observed_prev_omega}",
        )

    def _accl_from_raw_throttle(self, raw_throttle):
        """Mirror AcclAction.act() to recompute the physical acceleration applied this step."""
        params = self.env.unwrapped.sim.agents[0].params
        if self.env.unwrapped.normalize_act:
            return raw_throttle * params["a_max"]
        return raw_throttle

    def test_integrated_vel_cmd(self):
        """Test that integrated_vel_cmd holds integrated velocity command"""
        obs, _ = self.env.reset()

        # Get initial velocity command
        agent = self.env.unwrapped.sim.agents[0]
        initial_vel_cmd = agent.integrated_vel_cmd

        # Step with constant acceleration
        accl = 0.4
        action = np.array([[0.0, accl]])
        obs, _, _, _, _ = self.env.step(action)

        # Recompute the actual acceleration applied from the raw throttle (mirrors AcclAction.act)
        actual_accl = self._accl_from_raw_throttle(accl)
        timestep = self.env.unwrapped.timestep

        # Expected velocity command after integration
        expected_vel_cmd = initial_vel_cmd + actual_accl * timestep
        # Apply clipping as done in base_classes.py
        v_min = agent.params["v_min"]
        v_max = agent.params["v_max"]
        expected_vel_cmd = np.clip(expected_vel_cmd, v_min, v_max)

        # Extract from observation
        observed_vel_cmd = obs[10]

        self.assertAlmostEqual(
            observed_vel_cmd,
            expected_vel_cmd,
            places=4,
            msg=f"integrated_vel_cmd mismatch: expected {expected_vel_cmd}, got {observed_vel_cmd}",
        )

    def test_integrated_vel_cmd_multi_step_integration(self):
        """Test that integrated_vel_cmd correctly integrates over multiple time steps"""
        obs, _ = self.env.reset()

        # Get initial velocity command
        agent = self.env.unwrapped.sim.agents[0]
        initial_vel_cmd = agent.integrated_vel_cmd

        # Step twice with known acceleration
        accl = 0.3
        action = np.array([[0.0, accl]])
        timestep = self.env.unwrapped.timestep
        v_min = agent.params["v_min"]
        v_max = agent.params["v_max"]

        # First step
        obs1, _, _, _, _ = self.env.step(action)
        actual_accl_1 = self._accl_from_raw_throttle(accl)
        expected_vel_cmd_1 = np.clip(initial_vel_cmd + actual_accl_1 * timestep, v_min, v_max)

        # Second step
        obs2, _, _, _, _ = self.env.step(action)
        actual_accl_2 = self._accl_from_raw_throttle(accl)
        expected_vel_cmd_2 = np.clip(expected_vel_cmd_1 + actual_accl_2 * timestep, v_min, v_max)

        # Extract from observation
        observed_vel_cmd = obs2[10]

        self.assertAlmostEqual(
            observed_vel_cmd,
            expected_vel_cmd_2,
            places=4,
            msg=f"integrated_vel_cmd after 2 steps: expected {expected_vel_cmd_2}, got {observed_vel_cmd}",
        )

    def test_lookahead_curvatures(self):
        """Test that lookahead_curvatures matches sample_lookahead_curvatures_fast result"""
        obs, _ = self.env.reset()

        # Step to get non-initial position
        action = np.array([[0.0, 0.3]])
        obs, _, _, _, _ = self.env.step(action)

        # Get current position in Frenet coordinates
        agent = self.env.unwrapped.sim.agents[0]
        std_state = agent.standard_state
        x, y, theta = std_state["x"], std_state["y"], std_state["yaw"]

        track = self.env.unwrapped.track
        s, ey, ephi = track.cartesian_to_frenet(x, y, theta, use_raceline=False)

        # Get expected curvatures using the same function
        expected_curvatures = sample_lookahead_curvatures_fast(
            track, s, n_points=self.lookahead_n_points, ds=self.lookahead_ds
        )

        # Extract from observation
        observed_curvatures = obs[11 : 11 + self.lookahead_n_points]

        np.testing.assert_array_almost_equal(
            observed_curvatures,
            expected_curvatures,
            decimal=5,
            err_msg="lookahead_curvatures do not match expected values",
        )

    def test_lookahead_widths(self):
        """Test that lookahead_widths matches sample_lookahead_widths_fast result"""
        obs, _ = self.env.reset()

        # Step to get non-initial position
        action = np.array([[0.0, 0.3]])
        obs, _, _, _, _ = self.env.step(action)

        # Get current position in Frenet coordinates
        agent = self.env.unwrapped.sim.agents[0]
        std_state = agent.standard_state
        x, y, theta = std_state["x"], std_state["y"], std_state["yaw"]

        track = self.env.unwrapped.track
        s, ey, ephi = track.cartesian_to_frenet(x, y, theta, use_raceline=False)

        # Get expected widths using the same function
        expected_widths = sample_lookahead_widths_fast(track, s, n_points=self.lookahead_n_points, ds=self.lookahead_ds)

        # Extract from observation
        start_idx = 11 + self.lookahead_n_points
        end_idx = 11 + 2 * self.lookahead_n_points
        observed_widths = obs[start_idx:end_idx]

        np.testing.assert_array_almost_equal(
            observed_widths, expected_widths, decimal=5, err_msg="lookahead_widths do not match expected values"
        )

    def test_curr_avg_wheel_omega(self):
        """Test that curr_avg_wheel_omega is correctly computed and stored for STD model"""
        obs, _ = self.env.reset()

        # Step multiple times to build up wheel speeds
        action = np.array([[0.0, 0.5]])
        for _ in range(5):
            obs, _, _, _, _ = self.env.step(action)

        # Get current wheel omegas from agent (computed at start of last update_pose)
        agent = self.env.unwrapped.sim.agents[0]
        curr_omega_at_step5 = agent.curr_avg_wheel_omega

        # Verify it's non-zero (vehicle is moving)
        self.assertNotAlmostEqual(curr_omega_at_step5, 0.0, places=2, msg="curr_avg_wheel_omega should be non-zero")

        # Verify it's a finite number
        self.assertTrue(np.isfinite(curr_omega_at_step5), "curr_avg_wheel_omega should be finite")

        # Step once more - prev_avg_wheel_omega in next observation should match curr from this step
        obs_next, _, _, _, _ = self.env.step(action)

        # Extract prev_avg_wheel_omega
        observed_prev_omega = obs_next[9]

        # The prev value at step 6 should equal curr value from step 5
        self.assertAlmostEqual(
            observed_prev_omega,
            curr_omega_at_step5,
            places=5,
            msg=f"prev_avg_wheel_omega should be {curr_omega_at_step5}, got {observed_prev_omega}",
        )

    def test_observation_contains_no_nan(self):
        """Test that observation contains no NaN values"""
        obs, _ = self.env.reset()

        # Step multiple times
        action = np.array([[0.1, 0.3]])
        for _ in range(10):
            obs, _, _, _, _ = self.env.step(action)

        # Check for NaN
        self.assertFalse(np.any(np.isnan(obs)), "Observation contains NaN values")

    def test_observation_contains_no_inf(self):
        """Test that observation contains no infinite values"""
        obs, _ = self.env.reset()

        # Step multiple times
        action = np.array([[0.1, 0.3]])
        for _ in range(10):
            obs, _, _, _, _ = self.env.step(action)

        # Check for inf
        self.assertFalse(np.any(np.isinf(obs)), "Observation contains infinite values")

    def test_temporal_consistency(self):
        """Test that prev values correctly shift from curr values across time steps"""
        obs, _ = self.env.reset()

        # Step 1
        action1 = np.array([[0.2, 0.4]])
        obs1, _, _, _, _ = self.env.step(action1)
        agent = self.env.unwrapped.sim.agents[0]
        curr_steer_1 = agent.curr_steering_cmd
        curr_omega_1 = agent.curr_avg_wheel_omega

        # Step 2
        action2 = np.array([[0.3, 0.5]])
        obs2, _, _, _, _ = self.env.step(action2)
        prev_steer_2 = obs2[7]
        prev_omega_2 = obs2[9]

        # Verify temporal shift
        self.assertAlmostEqual(prev_steer_2, curr_steer_1, places=5, msg="Steering command not shifted correctly")
        self.assertAlmostEqual(prev_omega_2, curr_omega_1, places=5, msg="Wheel omega not shifted correctly")


class TestIntegratedVelCmdGuard(unittest.TestCase):
    """integrated_vel_cmd must raise under speed control and construct under accl."""

    def _make_env(self, control_input):
        config = get_drift_train_config()
        config["normalize_obs"] = False
        config["sparse_width_obs"] = False
        config["control_input"] = control_input
        config["observation_config"] = {"type": "drift"}
        return gym.make(get_env_id(), config=config)

    def test_raises_under_speed_mode(self):
        with self.assertRaises(ValueError) as ctx:
            self._make_env(["speed", "steering_angle"])
        msg = str(ctx.exception)
        self.assertIn("integrated_vel_cmd", msg)
        self.assertIn("speed", msg)

    def test_constructs_under_accl_mode(self):
        env = self._make_env(["accl", "steering_angle"])
        try:
            env.reset()
        finally:
            env.close()


class TestCurrCmdOptInFeatures(unittest.TestCase):
    """curr_steering_cmd / curr_throttle_cmd are opt-in obs features (no default obs type uses
    them). Verify the obs_size_dict + agent_obs wiring works end-to-end via direct
    VectorObservation construction."""

    def _make_env(self):
        config = get_drift_train_config()
        config["normalize_obs"] = False
        config["sparse_width_obs"] = False
        config["control_input"] = ["accl", "steering_angle"]
        config["observation_config"] = {"type": "frenet"}  # build env without drift constraints
        return gym.make(get_env_id(), config=config)

    def test_curr_cmd_features_reflect_current_step_action(self):
        from gymkhana.envs.observation import VectorObservation

        env = self._make_env()
        try:
            env.reset()
            vec = VectorObservation(env.unwrapped, features=["curr_steering_cmd", "curr_throttle_cmd"])

            steer_cmd, throttle_cmd = 0.4, 0.7
            env.step(np.array([[steer_cmd, throttle_cmd]], dtype=np.float32))
            obs = vec.observe()
            self.assertEqual(obs.shape, (2,), f"Expected shape (2,), got {obs.shape}")
            self.assertAlmostEqual(obs[0], steer_cmd, places=5, msg="curr_steering_cmd mismatch")
            self.assertAlmostEqual(obs[1], throttle_cmd, places=5, msg="curr_throttle_cmd mismatch")
        finally:
            env.close()
