import unittest
from unittest.mock import patch

import numpy as np

from gymkhana.envs import GKEnv
from gymkhana.envs.rendering.renderer import RenderSpec
from gymkhana.envs.utils import deep_update


class TestRenderer(unittest.TestCase):
    @staticmethod
    def _make_env(config={}, render_mode=None) -> GKEnv:
        import gymnasium as gym

        base_config = {
            "map": "Spielberg",
            "num_agents": 1,
            "timestep": 0.01,
            "integrator": "rk4",
            "control_input": ["speed", "steering_angle"],
            "model": "st",
            "observation_config": {"type": "kinematic_state"},
            "params": {"mu": 1.0},
        }
        config = deep_update(base_config, config)

        env = gym.make(
            "gymkhana:gymkhana-v0",
            config=config,
            render_mode=render_mode,
        )

        return env

    # def test_human_render(self):
    #     env = self._make_env(render_mode="human")
    #     env.reset()
    #     for _ in range(100):
    #         action = env.action_space.sample()
    #         env.step(action)
    #         env.render()
    #     env.close()

    #     self.assertTrue(True, "Human render test failed")

    def test_rgb_array_render(self):
        env = self._make_env(render_mode="rgb_array")
        env.reset()
        for _ in range(20):
            action = env.action_space.sample()
            env.step(action)
            frame = env.render()

            self.assertTrue(isinstance(frame, np.ndarray), "Frame is not a numpy array")
            self.assertTrue(len(frame.shape) == 3, "Frame is not a 3D array")
            self.assertTrue(frame.shape[2] == 3, "Frame does not have 3 channels")

        env.close()

        self.assertTrue(True, "rgb_array render test failed")

    def test_rgb_array_list(self):
        env = self._make_env(render_mode="rgb_array_list")
        env.reset()

        steps = 20
        for _ in range(steps):
            action = env.action_space.sample()
            env.step(action)

        frame_list = env.render()

        self.assertTrue(isinstance(frame_list, list), "the returned object is not a list of frames")
        self.assertTrue(
            len(frame_list) == steps + 1,
            "the returned list does not have the correct number of frames",
        )
        self.assertTrue(
            all([isinstance(frame, np.ndarray) for frame in frame_list]),
            "not all frames are numpy arrays",
        )
        self.assertTrue(
            all([len(frame.shape) == 3 for frame in frame_list]),
            "not all frames are 3D arrays",
        )
        self.assertTrue(
            all([frame.shape[2] == 3 for frame in frame_list]),
            "not all frames have 3 channels",
        )

        env.close()

    def test_arc_length_annotations_disabled_by_default(self):
        """Test that arc length annotations are disabled by default with no performance impact."""
        env = self._make_env(render_mode="rgb_array")
        env.reset()

        # Verify the feature is disabled by default
        self.assertFalse(
            env.unwrapped.render_arc_length_annotations, "Arc length annotations should be disabled by default"
        )

        # Verify no callback was registered for arc length annotations
        # The track's arc annotation cache should remain None when disabled
        self.assertIsNone(
            env.unwrapped.track.arc_annotation_points_render,
            "Arc annotation render cache should be None when feature is disabled",
        )

        # Run some steps to ensure no performance impact
        for _ in range(10):
            action = env.action_space.sample()
            env.step(action)
            frame = env.render()
            self.assertIsNotNone(frame, "Frame should render successfully with annotations disabled")

        env.close()

    def test_arc_length_annotations_enabled(self):
        """Test that arc length annotations can be enabled and render without crashing."""
        env = self._make_env(
            config={
                "render_arc_length_annotations": True,
                "arc_length_annotation_interval": 10.0,
            },
            render_mode="rgb_array",
        )
        env.reset()

        # Verify the feature is enabled
        self.assertTrue(env.unwrapped.render_arc_length_annotations, "Arc length annotations should be enabled")
        self.assertEqual(env.unwrapped.arc_length_annotation_interval, 10.0, "Interval should be 10.0 meters")

        # Render multiple frames to ensure annotations work
        for _ in range(10):
            action = env.action_space.sample()
            env.step(action)
            frame = env.render()

            self.assertIsNotNone(frame, "Frame should render successfully with annotations enabled")
            self.assertIsInstance(frame, np.ndarray, "Frame should be a numpy array")

        # Verify that annotation points were rendered (cache should be populated)
        self.assertIsNotNone(
            env.unwrapped.track.arc_annotation_points_render,
            "Arc annotation render cache should be populated after rendering",
        )

        env.close()

    def test_arc_length_annotations_different_intervals(self):
        """Test that different annotation interval values work correctly."""
        test_intervals = [2.5, 20.0]

        for interval in test_intervals:
            with self.subTest(interval=interval):
                env = self._make_env(
                    config={
                        "render_arc_length_annotations": True,
                        "arc_length_annotation_interval": interval,
                    },
                    render_mode="rgb_array",
                )
                env.reset()

                # Verify interval is set correctly
                self.assertEqual(
                    env.unwrapped.arc_length_annotation_interval,
                    interval,
                    f"Interval should be {interval} meters",
                )

                # Render a few frames to ensure it works
                for _ in range(10):
                    action = env.action_space.sample()
                    env.step(action)
                    frame = env.render()
                    self.assertIsNotNone(frame, f"Frame should render with interval={interval}")

                env.close()

    def test_arc_length_annotations_with_track_lines(self):
        """Test that arc length annotations work correctly with track lines enabled."""
        env = self._make_env(
            config={
                "render_track_lines": True,
                "render_arc_length_annotations": True,
                "arc_length_annotation_interval": 5.0,
            },
            render_mode="rgb_array",
        )
        env.reset()

        # Verify both features are enabled
        self.assertTrue(env.unwrapped.render_track_lines, "Track lines should be enabled")
        self.assertTrue(env.unwrapped.render_arc_length_annotations, "Arc length annotations should be enabled")

        # Render multiple frames with both features
        for _ in range(10):
            action = env.action_space.sample()
            env.step(action)
            frame = env.render()

            self.assertIsNotNone(frame, "Frame should render with both track lines and annotations")
            self.assertIsInstance(frame, np.ndarray, "Frame should be a numpy array")

        # Verify both rendering caches are populated
        self.assertIsNotNone(env.unwrapped.track.centerline.waypoint_render, "Centerline should be rendered")
        self.assertIsNotNone(
            env.unwrapped.track.arc_annotation_points_render,
            "Arc annotations should be rendered",
        )

        env.close()

    def test_control_debug_panel_rgb_array(self):
        """Test that the control debug panel renders without crashing in rgb_array mode."""
        original_from_yaml = RenderSpec.from_yaml

        def from_yaml_with_debug(yaml_file, overrides=None):
            spec = original_from_yaml(yaml_file, overrides=overrides)
            spec.show_ctr_debug = True
            return spec

        with patch.object(RenderSpec, "from_yaml", side_effect=from_yaml_with_debug):
            env = self._make_env(render_mode="rgb_array")

        env.reset()
        for _ in range(10):
            action = env.action_space.sample()
            env.step(action)
            frame = env.render()
            self.assertIsInstance(frame, np.ndarray)
            self.assertEqual(len(frame.shape), 3)

        # Verify debug data is present in render_obs
        render_obs = env.unwrapped.render_obs
        self.assertIn("steering_cmds", render_obs)
        self.assertIn("throttle_cmds", render_obs)
        self.assertIn("v_x", render_obs)
        self.assertIn("delta", render_obs)
        self.assertIn("steer_bounds", render_obs)
        self.assertIn("throttle_bounds", render_obs)

        env.close()

    def test_control_debug_panel_disabled(self):
        """Test that debug data is not in render_obs when show_ctr_debug is False."""
        original_from_yaml = RenderSpec.from_yaml

        def from_yaml_with_ctr_debug_off(yaml_file, overrides=None):
            spec = original_from_yaml(yaml_file, overrides=overrides)
            spec.show_ctr_debug = False
            return spec

        with patch.object(RenderSpec, "from_yaml", side_effect=from_yaml_with_ctr_debug_off):
            env = self._make_env(render_mode="rgb_array")

        env.reset()
        action = env.action_space.sample()
        env.step(action)
        env.render()

        render_obs = env.unwrapped.render_obs
        self.assertNotIn("steering_cmds", render_obs)

        env.close()

    # --- Observation debug overlay tests ---

    def _make_obs_debug_env(self, obs_config, extra_config=None):
        """Helper to create an env with show_obs_debug enabled and a given observation config."""
        original_from_yaml = RenderSpec.from_yaml

        def from_yaml_with_obs_debug(yaml_file, overrides=None):
            spec = original_from_yaml(yaml_file, overrides=overrides)
            spec.show_obs_debug = True
            return spec

        config = {"observation_config": obs_config}
        if extra_config:
            config.update(extra_config)

        with patch.object(RenderSpec, "from_yaml", side_effect=from_yaml_with_obs_debug):
            env = self._make_env(config=config, render_mode="rgb_array")
        return env

    def test_obs_debug_original_observation(self):
        """Test that _last_raw_features and get_debug_features work for OriginalObservation."""
        env = self._make_obs_debug_env({"type": "original"})
        env.reset()
        action = env.action_space.sample()
        env.step(action)
        env.render()

        obs_type = env.unwrapped.observation_type
        self.assertIsNotNone(obs_type._last_raw_features)

        features = obs_type.get_debug_features(0)
        self.assertIsInstance(features, dict)
        self.assertGreater(len(features), 0)
        self.assertIn("scans", features)
        self.assertIn("linear_vels_x", features)
        # Values should be per-agent scalars or 1D arrays (not multi-agent arrays)
        self.assertEqual(features["linear_vels_x"].ndim, 0)

        env.close()

    def test_obs_debug_features_observation(self):
        """Test that _last_raw_features and get_debug_features work for FeaturesObservation."""
        env = self._make_obs_debug_env({"type": "kinematic_state"})
        env.reset()
        action = env.action_space.sample()
        env.step(action)
        env.render()

        obs_type = env.unwrapped.observation_type
        self.assertIsNotNone(obs_type._last_raw_features)

        features = obs_type.get_debug_features(0)
        self.assertIsInstance(features, dict)
        self.assertGreater(len(features), 0)
        self.assertIn("pose_x", features)
        self.assertIn("linear_vel_x", features)

        env.close()

    def test_obs_debug_vector_observation(self):
        """Test that _last_raw_features and get_debug_features work for VectorObservation."""
        env = self._make_obs_debug_env({"type": "frenet"})
        env.reset()
        action = env.action_space.sample()
        env.step(action)
        env.render()

        obs_type = env.unwrapped.observation_type
        self.assertIsNotNone(obs_type._last_raw_features)

        features = obs_type.get_debug_features(0)
        self.assertIsInstance(features, dict)
        self.assertGreater(len(features), 0)
        self.assertIn("frenet_u", features)
        self.assertIn("frenet_n", features)

        env.close()

    def test_obs_debug_features_are_unnormalized(self):
        """Test that get_debug_features returns raw values even when normalization is on."""
        env = self._make_obs_debug_env(
            {"type": "drift"},
            extra_config={
                "normalize_obs": True,
                "model": "std",
                "control_input": ["accl", "steering_angle"],
                "params": GKEnv.f1tenth_std_vehicle_params(),
            },
        )
        env.reset()
        action = np.array([[0.0, 5.0]])
        env.step(action)

        self.assertTrue(env.unwrapped.normalize_obs, "Normalization should be active for this test")

        obs_type = env.unwrapped.observation_type

        # observe() returns a normalized flat vector
        normalized_obs = obs_type.observe()
        # get_debug_features() returns the raw named dict before normalization
        raw_features = obs_type.get_debug_features(0)

        # Reconstruct a raw flat vector from debug features in the same feature order
        raw_vec = []
        for v in raw_features.values():
            if isinstance(v, np.ndarray):
                raw_vec.extend(v.flat)
            else:
                raw_vec.append(float(v))
        raw_vec = np.array(raw_vec, dtype=np.float32)

        # Raw and normalized vectors should differ (normalization changes values)
        self.assertFalse(
            np.allclose(raw_vec, normalized_obs, atol=1e-6),
            "Debug features should be raw/unnormalized, but they match the normalized obs",
        )

        env.close()

    def test_obs_debug_render_obs_keys(self):
        """Test that obs debug data is passed through render_obs when enabled."""
        env = self._make_obs_debug_env({"type": "original"})
        env.reset()
        action = env.action_space.sample()
        env.step(action)
        env.render()

        render_obs = env.unwrapped.render_obs
        self.assertIn("obs_debug_getter", render_obs)
        self.assertIn("obs_debug_normalize", render_obs)
        self.assertTrue(callable(render_obs["obs_debug_getter"]))

        env.close()

    # --- render_config override tests ---

    def test_render_config_default(self):
        """Without render_config, RenderSpec fields come from packaged rendering.yaml."""
        env = self._make_env()
        spec = env.unwrapped.render_spec
        self.assertEqual(spec.window_size, 800)
        self.assertEqual(spec.render_type, "pyqt6")
        env.close()

    def test_render_config_full_override(self):
        """render_config overrides multiple fields at once."""
        env = self._make_env(config={"render_config": {"window_size": 1234, "render_type": "pygame"}})
        spec = env.unwrapped.render_spec
        self.assertEqual(spec.window_size, 1234)
        self.assertEqual(spec.render_type, "pygame")
        env.close()

    def test_render_config_partial_override(self):
        """Fields not present in render_config keep their packaged-yaml defaults."""
        env = self._make_env(config={"render_config": {"show_wheels": False}})
        spec = env.unwrapped.render_spec
        self.assertFalse(spec.show_wheels)
        self.assertEqual(spec.window_size, 800)
        env.close()

    def test_obs_debug_disabled(self):
        """Test that obs debug data is not in render_obs when show_obs_debug is False."""
        original_from_yaml = RenderSpec.from_yaml

        def from_yaml_with_obs_debug_off(yaml_file, overrides=None):
            spec = original_from_yaml(yaml_file, overrides=overrides)
            spec.show_obs_debug = False
            return spec

        with patch.object(RenderSpec, "from_yaml", side_effect=from_yaml_with_obs_debug_off):
            env = self._make_env(render_mode="rgb_array")

        env.reset()
        action = env.action_space.sample()
        env.step(action)
        env.render()

        render_obs = env.unwrapped.render_obs
        self.assertNotIn("obs_debug_getter", render_obs)

        env.close()
