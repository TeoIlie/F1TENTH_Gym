# MIT License
import time
import unittest
from unittest.mock import patch

import gymnasium as gym
import numpy as np

from gymkhana.envs.collision_models import collision, get_vertices

# Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


class CollisionTests(unittest.TestCase):
    def setUp(self):
        # test params
        np.random.seed(1234)

        # Collision check body
        self.vertices1 = np.asarray([[4, 11.0], [5, 5], [9, 9], [10, 10]])

        # car size
        self.length = 0.32
        self.width = 0.22

    def test_get_vert(self, debug=False):
        test_pose = np.array([2.3, 6.7, 0.8])
        vertices = get_vertices(test_pose, self.length, self.width)
        rect = np.vstack((vertices, vertices[0, :]))
        if debug:
            import matplotlib.pyplot as plt

            plt.scatter(test_pose[0], test_pose[1], c="red")
            plt.plot(rect[:, 0], rect[:, 1])
            plt.xlim([1, 4])
            plt.ylim([5, 8])
            plt.axes().set_aspect("equal")
            plt.show()
        self.assertTrue(vertices.shape == (4, 2))

    def test_get_vert_fps(self):
        test_pose = np.array([2.3, 6.7, 0.8])
        start = time.time()
        for _ in range(1000):
            get_vertices(test_pose, self.length, self.width)
        elapsed = time.time() - start
        fps = 1000 / elapsed
        print("get vertices fps:", fps)
        self.assertGreater(fps, 500)

    def test_random_collision(self):
        # perturb the body by a small amount and make sure it all collides with the original body
        for _ in range(1000):
            a = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
            b = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
            self.assertTrue(collision(a, b))

    def test_fps(self):
        # also perturb the body but mainly want to test GJK speed
        start = time.time()
        for _ in range(1000):
            a = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
            b = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
            collision(a, b)
        elapsed = time.time() - start
        fps = 1000 / elapsed
        print("gjk fps:", fps)
        # self.assertGreater(fps, 500)  This is a platform dependent test, not ideal.


class TestFrenetBoundaryChecking(unittest.TestCase):
    """
    Tests for Frenet-based boundary checking (predictive_collision=False mode).
    This mode uses explicit Frenet coordinate boundary detection instead of
    predictive TTC-based collision detection.
    """

    @classmethod
    def setUpClass(cls):
        """Create shared test environment with Frenet boundary checking enabled."""
        cls.env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "observation_config": {"type": None},
                "reset_config": {"type": "rl_random_static"},
                "predictive_collision": False,  # Enable Frenet boundary checking
                "progress_gain": 1.0,
            },
        )
        cls.env.reset()

    @classmethod
    def tearDownClass(cls):
        """Clean up environment."""
        cls.env.close()

    def test_frenet_boundary_agent_within_bounds(self):
        """Test that agent within track boundaries is not detected as collision."""
        unwrapped = self.env.unwrapped

        # Mock track with known boundaries
        unwrapped.track.centerline.w_lefts = np.array([2.0] * 100)
        unwrapped.track.centerline.w_rights = np.array([2.0] * 100)
        unwrapped.track.centerline.ss = np.linspace(0, 100, 100)

        # Agent well within bounds (ey=0.5m, half_width=2.0m)
        unwrapped._frenet_cache[0] = [50.0, 0.5, 0.0]
        result = unwrapped._check_boundary_frenet(0)

        self.assertFalse(result, "Agent within bounds should not trigger collision")

    def test_frenet_boundary_agent_exceeds_left_bound(self):
        """Test that agent exceeding left boundary is detected as collision."""
        unwrapped = self.env.unwrapped

        # Mock track with known boundaries
        unwrapped.track.centerline.w_lefts = np.array([2.0] * 100)
        unwrapped.track.centerline.w_rights = np.array([2.0] * 100)
        unwrapped.track.centerline.ss = np.linspace(0, 100, 100)

        # Agent exceeds left bound (ey=2.5m > half_width=2.0m)
        unwrapped._frenet_cache[0] = [50.0, 2.5, 0.0]
        result = unwrapped._check_boundary_frenet(0)

        self.assertTrue(result, "Agent exceeding left boundary should trigger collision")

    def test_frenet_boundary_agent_exceeds_right_bound(self):
        """Test that agent exceeding right boundary is detected as collision."""
        unwrapped = self.env.unwrapped

        # Mock track with known boundaries
        unwrapped.track.centerline.w_lefts = np.array([2.0] * 100)
        unwrapped.track.centerline.w_rights = np.array([2.0] * 100)
        unwrapped.track.centerline.ss = np.linspace(0, 100, 100)

        # Agent exceeds right bound (ey=-2.5m < -half_width=-2.0m)
        unwrapped._frenet_cache[0] = [50.0, -2.5, 0.0]
        result = unwrapped._check_boundary_frenet(0)

        self.assertTrue(result, "Agent exceeding right boundary should trigger collision")

    def test_frenet_reward_in_bounds_positive_progress(self):
        """Test that agent in bounds receives progress reward."""
        unwrapped = self.env.unwrapped

        # Mock track
        unwrapped.track.centerline.w_lefts = np.array([2.0] * 100)
        unwrapped.track.centerline.w_rights = np.array([2.0] * 100)
        unwrapped.track.centerline.ss = np.linspace(0, 100, 100)

        # Initialize last_s
        unwrapped.last_s = [10.9]
        unwrapped.poses_x = [0.0]
        unwrapped.poses_y = [0.0]
        unwrapped.poses_theta = [0.0]

        # In bounds (ey=0.5m), s=11.0 → 0.1m progress from last_s=10.9
        unwrapped._frenet_cache[0] = [11.0, 0.5, 0.0]
        unwrapped._update_state()
        reward = unwrapped._get_reward()

        # Should get progress reward: 11.0 - 10.9 = 0.1
        self.assertAlmostEqual(reward, 0.1, places=5, msg="In-bounds agent should receive progress reward")

    def test_frenet_reward_out_of_bounds_exclusive_penalty(self):
        """Test that agent out of bounds receives -1 penalty (exclusive, not additive)."""
        unwrapped = self.env.unwrapped

        # Mock track
        unwrapped.track.centerline.w_lefts = np.array([2.0] * 100)
        unwrapped.track.centerline.w_rights = np.array([2.0] * 100)
        unwrapped.track.centerline.ss = np.linspace(0, 100, 100)

        # Initialize last_s
        unwrapped.last_s = [10.0]
        unwrapped.poses_x = [0.0]
        unwrapped.poses_y = [0.0]
        unwrapped.poses_theta = [0.0]

        # Out of bounds (ey=2.5m > 2.0m), s=11.0
        unwrapped._frenet_cache[0] = [11.0, 2.5, 0.0]
        unwrapped._update_state()
        reward = unwrapped._get_reward()

        # Should get exclusive penalty
        self.assertAlmostEqual(reward, -50.0, places=5, msg="Out-of-bounds agent should receive exclusive -50 penalty")

    def test_frenet_error_handling_missing_track_boundaries(self):
        """Test that missing track boundary data raises clear ValueError."""
        unwrapped = self.env.unwrapped

        # Mock track with missing boundary data
        unwrapped.track.centerline.w_lefts = None
        unwrapped.track.centerline.w_rights = None
        unwrapped.track.centerline.ss = np.linspace(0, 100, 100)

        # Mock Frenet conversion to succeed
        with patch.object(unwrapped.track, "cartesian_to_frenet", return_value=(50.0, 0.5, 0.0)):
            with self.assertRaises(ValueError) as context:
                unwrapped._check_boundary_frenet(0)

        error_msg = str(context.exception)
        self.assertIn("boundary data", error_msg.lower())
        self.assertIn("w_lefts", error_msg.lower())

    def test_frenet_vs_predictive_reward_structure(self):
        """Test that Frenet mode has exclusive penalty vs predictive's additive penalty."""
        # Create two environments: one with Frenet, one with predictive
        env_frenet = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "observation_config": {"type": None},
                "reset_config": {"type": "rl_random_static"},
                "predictive_collision": False,  # Frenet mode
                "progress_gain": 1.0,
            },
        )

        env_predictive = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "observation_config": {"type": None},
                "reset_config": {"type": "rl_random_static"},
                "predictive_collision": True,  # Predictive mode
                "progress_gain": 1.0,
            },
        )

        try:
            env_frenet.reset()
            env_predictive.reset()

            # Setup: agent with 1.0m progress and collision
            unwrapped_frenet = env_frenet.unwrapped
            unwrapped_predictive = env_predictive.unwrapped

            # Mock track for both
            for unwrapped in [unwrapped_frenet, unwrapped_predictive]:
                unwrapped.track.centerline.w_lefts = np.array([2.0] * 100)
                unwrapped.track.centerline.w_rights = np.array([2.0] * 100)
                unwrapped.track.centerline.ss = np.linspace(0, 100, 100)
                unwrapped.last_s = [10.0]
                unwrapped.poses_x = [0.0]
                unwrapped.poses_y = [0.0]
                unwrapped.poses_theta = [0.0]

            # Frenet: out of bounds (ey=2.5m), s=10.1 (0.1m progress)
            unwrapped_frenet._frenet_cache[0] = [10.1, 2.5, 0.0]
            unwrapped_frenet._update_state()
            reward_frenet = unwrapped_frenet._get_reward()

            # Predictive: collision, s=10.1 (0.1m progress)
            unwrapped_predictive.collisions[0] = 1
            unwrapped_predictive._frenet_cache[0] = [10.1, 0.0, 0.0]
            reward_predictive = unwrapped_predictive._get_reward()

            # Frenet: exclusive penalty = -50
            self.assertAlmostEqual(reward_frenet, -50.0, places=5, msg="Frenet mode should have exclusive -1 penalty")
            self.assertAlmostEqual(
                reward_predictive,
                -49.9,
                places=5,
                msg="Predictive mode should have additive penalty (0.1 - 1.0 = -0.9)",
            )

        finally:
            env_frenet.close()
            env_predictive.close()


class TestWallDeflectionBehavior(unittest.TestCase):
    """
    Tests for wall_deflection configuration parameter.
    """

    @patch("gymkhana.envs.base_classes.check_ttc_jit")
    def test_wall_deflection_false_no_velocity_change(self, mock_check_ttc_jit):
        """Test that wall_deflection=False does not change velocity on collision."""
        # Mock check_ttc_jit to return True (collision detected)
        mock_check_ttc_jit.return_value = True

        env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "observation_config": {"type": "original"},
                "wall_deflection": False,
            },
        )
        env.reset()
        agent = env.unwrapped.sim.agents[0]

        # Set known velocity and control values
        agent.state[3] = 5.0  # velocity
        agent.accel = 2.0
        agent.steer_angle_vel = 0.5

        # Call check_ttc with a dummy scan (mock will handle the return)
        dummy_scan = np.zeros(agent.scan_simulator.num_beams)
        agent.check_ttc(dummy_scan)

        # With wall_deflection=False, velocity should NOT be zeroed
        self.assertAlmostEqual(
            agent.state[3], 5.0, places=5, msg="Velocity should not change with wall_deflection=False"
        )
        self.assertAlmostEqual(agent.accel, 2.0, places=5, msg="Acceleration should not change")
        self.assertAlmostEqual(agent.steer_angle_vel, 0.5, places=5, msg="Steering velocity should not change")
        self.assertTrue(agent.in_collision, msg="Collision flag should be set")

        env.close()

    @patch("gymkhana.envs.base_classes.check_ttc_jit")
    def test_wall_deflection_true_zeros_velocity(self, mock_check_ttc_jit):
        """Test that wall_deflection=True zeros velocity on collision."""
        # Mock check_ttc_jit to return True (collision detected)
        mock_check_ttc_jit.return_value = True

        env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "observation_config": {"type": "original"},
                "wall_deflection": True,
            },
        )
        env.reset()
        agent = env.unwrapped.sim.agents[0]

        # Set known velocity and control values
        agent.state[3] = 5.0  # velocity
        agent.accel = 2.0
        agent.steer_angle_vel = 0.5

        # Call check_ttc with a dummy scan (mock will handle the return)
        dummy_scan = np.zeros(agent.scan_simulator.num_beams)
        agent.check_ttc(dummy_scan)

        # With wall_deflection=True, velocity SHOULD be zeroed
        self.assertAlmostEqual(agent.state[3], 0.0, places=5, msg="Velocity should be zero with wall_deflection=True")
        self.assertAlmostEqual(agent.accel, 0.0, places=5, msg="Acceleration should be zero")
        self.assertAlmostEqual(agent.steer_angle_vel, 0.0, places=5, msg="Steering velocity should be zero")
        self.assertTrue(agent.in_collision, msg="Collision flag should be set")

        env.close()
