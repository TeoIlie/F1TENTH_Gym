import unittest

import gymnasium as gym
import numpy as np
from gymnasium.spaces import Box

from gymkhana.envs import GKEnv
from gymkhana.envs.observation import (
    observation_factory,
    sample_lookahead_curvatures_fast,
    sample_lookahead_widths_fast,
    sample_raceline_velocities_fast,
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
        # "drift" obs requires "accl" longitudinal control (see observation.py:680).
        cls.config["observation_config"] = {"type": "drift"}
        cls.config["control_input"] = ["accl", "steering_angle"]
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


class TestFrenetReference(unittest.TestCase):
    """Test suite for the frenet_reference observation option (centerline vs raceline).

    v1 scope: only frenet_u and frenet_n follow the reference line. Lookahead curvatures
    and widths stay centerline-based under both references — see test_lookahead_features_
    unaffected_by_reference, which must be updated deliberately if that ever changes.
    """

    # drift_real layout: [vx, vy, frenet_u, frenet_n, r, beta, omega, curvatures..., widths...]
    FRENET_U_I = 2
    FRENET_N_I = 3

    @classmethod
    def setUpClass(cls):
        cls.config = get_drift_train_config()
        cls.config["normalize_obs"] = False
        cls.config["sparse_width_obs"] = False
        cls.config["observation_config"] = {"type": "drift_real"}
        cls.config["control_input"] = ["accl", "steering_angle"]

    def _make_env(self, frenet_reference=None, map=None):
        """Build an env, omitting frenet_reference entirely when None (default-path coverage)."""
        # Overrides go through deep_update so nested dicts are rebuilt; editing
        # config["observation_config"] in place would leak into cls.config and later tests.
        overrides = {}
        if frenet_reference is not None:
            overrides["observation_config"] = {"frenet_reference": frenet_reference}
        if map is not None:
            overrides["map"] = map
        return gym.make(get_env_id(), config=deep_update(self.config, overrides))

    @staticmethod
    def _step(env):
        """Step once off the reset pose so the Frenet errors are non-trivial."""
        obs, _, _, _, _ = env.step(np.array([[0.1, 0.2]]))
        return obs

    def test_default_is_centerline(self):
        """Omitting frenet_reference keeps the pre-existing centerline behaviour."""
        env = self._make_env()
        try:
            env.reset(seed=3)
            self.assertEqual(env.unwrapped.observation_type.frenet_reference, "centerline")
            self.assertFalse(env.unwrapped._use_raceline_frenet, "Raceline projection must stay off by default")

            obs = self._step(env)
            std_state = env.unwrapped.sim.agents[0].standard_state
            _, ey, ephi = env.unwrapped.track.cartesian_to_frenet(
                std_state["x"], std_state["y"], std_state["yaw"], use_raceline=False
            )
            self.assertAlmostEqual(obs[self.FRENET_U_I], ephi, places=5, msg="frenet_u should be centerline ephi")
            self.assertAlmostEqual(obs[self.FRENET_N_I], ey, places=5, msg="frenet_n should be centerline ey")
        finally:
            env.close()

    def test_raceline_reference_matches_raceline_projection(self):
        """frenet_u/frenet_n come from the raceline projection, and differ from the centerline."""
        env = self._make_env("raceline")
        try:
            env.reset(seed=3)
            self.assertTrue(env.unwrapped._use_raceline_frenet)

            obs = self._step(env)
            std_state = env.unwrapped.sim.agents[0].standard_state
            x, y, theta = std_state["x"], std_state["y"], std_state["yaw"]
            track = env.unwrapped.track

            _, ey_race, ephi_race = track.cartesian_to_frenet(x, y, theta, use_raceline=True)
            self.assertAlmostEqual(obs[self.FRENET_U_I], ephi_race, places=5, msg="frenet_u should be raceline ephi")
            self.assertAlmostEqual(obs[self.FRENET_N_I], ey_race, places=5, msg="frenet_n should be raceline ey")

            # The two references must actually disagree, else the test proves nothing.
            _, ey_center, _ = track.cartesian_to_frenet(x, y, theta, use_raceline=False)
            self.assertNotAlmostEqual(
                ey_race, ey_center, places=3, msg="Raceline and centerline ey coincide; test is vacuous"
            )
        finally:
            env.close()

    def test_raceline_cache_untouched_when_disabled(self):
        """The raceline projection is skipped entirely under the centerline reference."""
        env = self._make_env("centerline")
        try:
            env.reset(seed=3)
            self._step(env)
            cache = env.unwrapped._frenet_cache_raceline
            self.assertTrue(
                np.array_equal(cache, np.zeros_like(cache)),
                "_frenet_cache_raceline must stay zeros when frenet_reference='centerline'",
            )
        finally:
            env.close()

    def test_lookahead_features_unaffected_by_reference(self):
        """v1 invariant: lookahead curvatures and widths remain centerline-sampled."""
        observations = {}
        for reference in ("centerline", "raceline"):
            env = self._make_env(reference)
            try:
                env.reset(seed=5)
                observations[reference] = self._step(env)
            finally:
                env.close()

        # Assert on everything outside frenet_u/frenet_n rather than a hardcoded lookahead
        # offset, so the invariant survives changes to the drift_real feature list.
        frenet_slice = slice(self.FRENET_U_I, self.FRENET_N_I + 1)
        others = np.ones(observations["centerline"].shape, dtype=bool)
        others[frenet_slice] = False
        np.testing.assert_array_equal(
            observations["centerline"][others],
            observations["raceline"][others],
            err_msg="Only frenet_u/frenet_n may differ across references in v1",
        )
        self.assertFalse(
            np.allclose(observations["centerline"][frenet_slice], observations["raceline"][frenet_slice]),
            "frenet_u/frenet_n should differ across references",
        )

    def test_observation_length_unchanged(self):
        """The reference line does not change the observation vector layout."""
        lengths = {}
        for reference in ("centerline", "raceline"):
            env = self._make_env(reference)
            try:
                obs, _ = env.reset(seed=5)
                lengths[reference] = (obs.shape[0], env.observation_space.shape[0])
            finally:
                env.close()
        self.assertEqual(lengths["centerline"], lengths["raceline"])

    def test_invalid_reference_raises(self):
        """An unrecognised reference fails loudly at construction."""
        with self.assertRaises(ValueError) as ctx:
            self._make_env("middle")
        self.assertIn("frenet_reference", str(ctx.exception))

    def test_missing_raceline_raises(self):
        """Requesting the raceline on a track that has none fails instead of silently aliasing."""
        from gymkhana.envs.track import Track

        source = Track.from_track_name(self.config["map"])
        # Track.__init__ aliases raceline -> centerline when raceline is None, which would make
        # frenet_reference="raceline" a silent no-op.
        no_raceline = Track(
            spec=source.spec,
            occupancy_map=source.occupancy_map,
            filepath=source.filepath,
            ext=source.ext,
            centerline=source.centerline_regular,
            raceline=None,
        )
        with self.assertRaises(ValueError) as ctx:
            self._make_env("raceline", map=no_raceline)
        self.assertIn("raceline", str(ctx.exception))

    def test_frenet_n_norm_bounds_widen_for_raceline(self):
        """The raceline reference widens the frenet_n bound to the full track width."""
        from gymkhana.envs.utils import GLOBAL_MAX_WIDTH, calculate_norm_bounds

        env = self._make_env()
        try:
            env.reset(seed=3)
            unwrapped = env.unwrapped
            centerline_bounds = calculate_norm_bounds(unwrapped, ["frenet_n"], frenet_reference="centerline")
            raceline_bounds = calculate_norm_bounds(unwrapped, ["frenet_n"], frenet_reference="raceline")
            default_bounds = calculate_norm_bounds(unwrapped, ["frenet_n"])

            self.assertEqual(centerline_bounds["frenet_n"], (-0.5 * GLOBAL_MAX_WIDTH, 0.5 * GLOBAL_MAX_WIDTH))
            self.assertEqual(raceline_bounds["frenet_n"], (-GLOBAL_MAX_WIDTH, GLOBAL_MAX_WIDTH))
            self.assertEqual(default_bounds["frenet_n"], centerline_bounds["frenet_n"])
        finally:
            env.close()


class TestRacelineVxsObservation(unittest.TestCase):
    """Test suite for the raceline_vxs feature and the drift_real_vref observation type.

    raceline_vxs reports the raceline's precomputed velocity profile (the vx_mps column of
    <map>_raceline.csv) sampled ahead of the car. Unlike the curvature/width lookaheads it is
    indexed by raceline arc length, so these tests pin that distinction.
    """

    # drift_real_vref layout: drift_real followed by raceline_vxs, so the leading indices
    # are identical to drift_real and the velocity window is always the tail.
    N_DRIFT_REAL_SCALARS = 7

    @classmethod
    def setUpClass(cls):
        cls.config = get_drift_train_config()
        cls.config["normalize_obs"] = False
        cls.config["sparse_width_obs"] = False
        cls.config["observation_config"] = {"type": "drift_real_vref", "frenet_reference": "raceline"}
        cls.config["control_input"] = ["accl", "steering_angle"]

    def _make_env(self, overrides=None, **obs_config_overrides):
        overrides = dict(overrides or {})
        if obs_config_overrides:
            overrides["observation_config"] = obs_config_overrides
        return gym.make(get_env_id(), config=deep_update(self.config, overrides))

    @staticmethod
    def _step(env):
        """Step once off the reset pose so the sampled window is not the reset one."""
        obs, _, _, _, _ = env.step(np.array([[0.1, 0.2]]))
        return obs

    def test_observation_length_is_drift_real_plus_window(self):
        """drift_real_vref appends exactly raceline_vx_n_points elements to drift_real."""
        vref_env = self._make_env()
        base_env = self._make_env(type="drift_real")
        try:
            vref_obs, _ = vref_env.reset(seed=3)
            base_obs, _ = base_env.reset(seed=3)
            n = vref_env.unwrapped.raceline_vx_n_points

            self.assertEqual(vref_obs.shape[0], base_obs.shape[0] + n)
            self.assertEqual(vref_env.observation_space.shape[0], vref_obs.shape[0])
        finally:
            vref_env.close()
            base_env.close()

    def test_leading_features_match_drift_real(self):
        """Appending the window must not shift any pre-existing drift_real index."""
        vref_env = self._make_env()
        base_env = self._make_env(type="drift_real")
        try:
            vref_env.reset(seed=7)
            base_env.reset(seed=7)
            vref_obs = self._step(vref_env)
            base_obs = self._step(base_env)

            np.testing.assert_allclose(
                vref_obs[: base_obs.shape[0]],
                base_obs,
                rtol=1e-6,
                atol=1e-6,
                err_msg="drift_real_vref must be drift_real with the window appended, not interleaved",
            )
        finally:
            vref_env.close()
            base_env.close()

    def test_window_matches_direct_sampler(self):
        """The observation tail equals the sampler called with the raceline arc length."""
        env = self._make_env()
        try:
            env.reset(seed=3)
            obs = self._step(env)
            unwrapped = env.unwrapped
            n = unwrapped.raceline_vx_n_points

            std_state = unwrapped.sim.agents[0].standard_state
            s_raceline, _, _ = unwrapped.track.cartesian_to_frenet(
                std_state["x"], std_state["y"], std_state["yaw"], use_raceline=True
            )
            expected = sample_raceline_velocities_fast(
                unwrapped.track, s_raceline, n_points=n, ds=unwrapped.raceline_vx_ds
            )
            np.testing.assert_array_almost_equal(obs[-n:], expected, decimal=5)
        finally:
            env.close()

    def test_window_is_fresh_at_reset(self):
        """The raceline cache is populated before the first observe(), not one step behind."""
        env = self._make_env()
        try:
            unwrapped = env.unwrapped
            n = unwrapped.raceline_vx_n_points
            for seed in (1, 2, 3):
                obs, _ = env.reset(seed=seed)
                std_state = unwrapped.sim.agents[0].standard_state
                s_pose, _, _ = unwrapped.track.cartesian_to_frenet(
                    std_state["x"], std_state["y"], std_state["yaw"], use_raceline=True
                )
                expected = sample_raceline_velocities_fast(
                    unwrapped.track, s_pose, n_points=n, ds=unwrapped.raceline_vx_ds
                )
                np.testing.assert_array_almost_equal(obs[-n:], expected, decimal=5)
        finally:
            env.close()

    def test_normalized_arc_length_raceline_rejected(self):
        """ss in a [0,1] parameterization would break the wrap-around, so it must fail loudly."""
        env = self._make_env()
        try:
            env.reset(seed=3)
            track = env.unwrapped.track
            raceline = track.raceline
            original_ss = raceline.ss
            try:
                raceline.ss = np.linspace(0.0, 1.0, len(original_ss), dtype=np.float32)
                with self.assertRaises(ValueError) as ctx:
                    sample_raceline_velocities_fast(track, 0.0, n_points=3, ds=0.4)
                self.assertIn("normalized", str(ctx.exception))
            finally:
                raceline.ss = original_ss
        finally:
            env.close()

    def test_window_is_indexed_by_raceline_not_centerline_s(self):
        """Sampling at the centerline s must give a different window (the classic wiring bug).

        Drift's centerline is ~24.6 m and its raceline ~22.8 m, so the same numeric s lands on
        different parts of the two lines.
        """
        env = self._make_env()
        try:
            env.reset(seed=3)
            obs = self._step(env)
            unwrapped = env.unwrapped
            n = unwrapped.raceline_vx_n_points
            ds = unwrapped.raceline_vx_ds

            s_centerline = float(unwrapped._frenet_cache[0, 0])
            s_raceline = float(unwrapped._frenet_cache_raceline[0, 0])
            self.assertNotAlmostEqual(
                s_centerline, s_raceline, places=3, msg="Arc lengths coincide here; test is vacuous"
            )

            wrong = sample_raceline_velocities_fast(unwrapped.track, s_centerline, n_points=n, ds=ds)
            self.assertFalse(
                np.allclose(obs[-n:], wrong),
                "raceline_vxs appears to be sampled at the centerline arc length",
            )
        finally:
            env.close()

    def test_window_values_lie_on_the_raceline_profile(self):
        """Every sampled value is drawn from the map's actual velocity profile."""
        env = self._make_env()
        try:
            env.reset(seed=3)
            obs = self._step(env)
            unwrapped = env.unwrapped
            n = unwrapped.raceline_vx_n_points
            vxs = np.asarray(unwrapped.track.raceline.vxs, dtype=np.float64)
            window = obs[-n:]

            self.assertTrue(np.all(window >= vxs.min() - 1e-5))
            self.assertTrue(np.all(window <= vxs.max() + 1e-5))
            # Nearest-neighbour sampling, so each value must be an actual waypoint velocity.
            for v in window:
                self.assertTrue(np.any(np.isclose(vxs, v, atol=1e-4)), f"{v} is not a raceline waypoint velocity")
            # A constant window would mean the profile is not really being read.
            self.assertGreater(np.ptp(window), 0.0, "Velocity window is constant; profile likely not loaded")
        finally:
            env.close()

    def test_first_element_is_velocity_at_current_position(self):
        """Offset 0 (not ds) so the controller gets the reference at its own arc length."""
        env = self._make_env()
        try:
            env.reset(seed=3)
            self._step(env)
            unwrapped = env.unwrapped
            raceline = unwrapped.track.raceline
            s = float(unwrapped._frenet_cache_raceline[0, 0])

            window = sample_raceline_velocities_fast(unwrapped.track, s, n_points=3, ds=unwrapped.raceline_vx_ds)
            nearest = int(np.argmin(np.abs(np.asarray(raceline.ss, dtype=np.float64) - s)))
            self.assertAlmostEqual(float(window[0]), float(raceline.vxs[nearest]), places=5)
        finally:
            env.close()

    def test_sampler_wraps_around_end_of_line(self):
        """A window straddling the end of a closed raceline continues from the start."""
        env = self._make_env()
        try:
            env.reset(seed=3)
            track = env.unwrapped.track
            raceline = track.raceline
            length = float(raceline.ss[-1])
            ds = 0.4
            n = 5

            # s == length wraps to 0, so the whole window must equal the window sampled from 0.
            at_end = sample_raceline_velocities_fast(track, length, n_points=n, ds=ds)
            from_start = sample_raceline_velocities_fast(track, 0.0, n_points=n, ds=ds)
            np.testing.assert_allclose(at_end, from_start, rtol=0, atol=1e-4)

            # A window straddling the end stays in range and picks up early-line values.
            straddling = sample_raceline_velocities_fast(track, length - ds / 2, n_points=n, ds=ds)
            self.assertEqual(straddling.shape, (n,))
            self.assertTrue(np.all(np.isfinite(straddling)))
            vxs = np.asarray(raceline.vxs, dtype=np.float64)
            for v in straddling:
                self.assertTrue(np.any(np.isclose(vxs, v, atol=1e-4)))
        finally:
            env.close()

    def test_requires_raceline_frenet_reference(self):
        """The feature fails loudly rather than silently reading a stale zero cache."""
        for reference in ("centerline", None):
            with self.subTest(frenet_reference=reference):
                obs_config = {"type": "drift_real_vref"}
                if reference is not None:
                    obs_config["frenet_reference"] = reference
                # Replace observation_config wholesale: deep_update *merges* nested dicts, so it
                # cannot express "frenet_reference absent" against a base that sets it.
                config = deep_update(self.config, {})
                config["observation_config"] = obs_config
                with self.assertRaises(ValueError) as ctx:
                    gym.make(get_env_id(), config=config)
                self.assertIn("raceline_vxs", str(ctx.exception))

    def test_missing_raceline_raises(self):
        """A map with no raceline must fail, not fall back to the centerline's constant vxs."""
        from gymkhana.envs.track import Track

        source = Track.from_track_name(self.config["map"])
        no_raceline = Track(
            spec=source.spec,
            occupancy_map=source.occupancy_map,
            filepath=source.filepath,
            ext=source.ext,
            centerline=source.centerline_regular,
            raceline=None,
        )
        with self.assertRaises(ValueError) as ctx:
            self._make_env(overrides={"map": no_raceline})
        self.assertIn("raceline", str(ctx.exception))

    def test_window_respects_config_keys(self):
        """raceline_vx_n_points / raceline_vx_ds size and space the window."""
        env = self._make_env(overrides={"raceline_vx_n_points": 4, "raceline_vx_ds": 1.0})
        try:
            obs, _ = env.reset(seed=3)
            unwrapped = env.unwrapped
            self.assertEqual(unwrapped.raceline_vx_n_points, 4)
            self.assertEqual(obs[-4:].shape, (4,))

            expected = sample_raceline_velocities_fast(
                unwrapped.track, float(unwrapped._frenet_cache_raceline[0, 0]), n_points=4, ds=1.0
            )
            np.testing.assert_array_almost_equal(obs[-4:], expected, decimal=5)
        finally:
            env.close()

    def test_invalid_config_keys_raise(self):
        """Degenerate window settings fail at construction."""
        with self.assertRaises(ValueError):
            self._make_env(overrides={"raceline_vx_n_points": 0})
        with self.assertRaises(ValueError):
            self._make_env(overrides={"raceline_vx_ds": 0.0})

    def test_normalization_uses_velocity_bounds(self):
        """raceline_vxs shares linear_vel_x's bound so errors between them are meaningful."""
        from gymkhana.envs.utils import calculate_norm_bounds

        env = self._make_env(overrides={"normalize_obs": True})
        try:
            obs, _ = env.reset(seed=3)
            unwrapped = env.unwrapped
            self.assertTrue(unwrapped.normalize_obs, "drift_real_vref must support normalization")

            bounds = calculate_norm_bounds(unwrapped, ["linear_vel_x", "raceline_vxs"], frenet_reference="raceline")
            self.assertEqual(bounds["raceline_vxs"], bounds["linear_vel_x"])
            self.assertEqual(bounds["raceline_vxs"], (unwrapped.params["v_min"], unwrapped.params["v_max"]))

            n = unwrapped.raceline_vx_n_points
            self.assertTrue(np.all(np.abs(obs[-n:]) <= 1.0), "Normalized window must lie within [-1, 1]")
            self.assertTrue(env.observation_space.contains(obs))
        finally:
            env.close()
