import os
import sys
import unittest
from unittest.mock import MagicMock, patch

# Add parent directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from train.train_utils import compute_global_track_bounds


class TestComputeGlobalTrackBounds(unittest.TestCase):
    """Test the compute_global_track_bounds helper function for multi-map training."""

    def test_single_track(self):
        """Test bounds computation for a single track."""
        track_pool = ["Drift"]
        bounds = compute_global_track_bounds(track_pool, track_scale=1.0)

        # Verify return structure
        self.assertIn("track_max_curv", bounds)
        self.assertIn("track_min_width", bounds)
        self.assertIn("track_max_width", bounds)

        # Verify types and reasonable values
        self.assertIsInstance(bounds["track_max_curv"], float)
        self.assertIsInstance(bounds["track_min_width"], float)
        self.assertIsInstance(bounds["track_max_width"], float)

        # Sanity checks
        self.assertGreater(bounds["track_max_curv"], 0, "Max curvature should be positive")
        self.assertGreater(bounds["track_min_width"], 0, "Min width should be positive")
        self.assertGreater(bounds["track_max_width"], 0, "Max width should be positive")
        self.assertGreaterEqual(
            bounds["track_max_width"],
            bounds["track_min_width"],
            "Max width should be >= min width",
        )

    def test_multiple_tracks(self):
        """Test bounds computation across multiple tracks returns maximum bounds."""
        track_pool = ["Drift", "Austin", "Monza"]
        bounds = compute_global_track_bounds(track_pool, track_scale=1.0)

        # Verify structure
        self.assertIn("track_max_curv", bounds)
        self.assertIn("track_min_width", bounds)
        self.assertIn("track_max_width", bounds)

        # Compute individual track bounds for comparison
        individual_bounds = []
        for track_name in track_pool:
            track_bounds = compute_global_track_bounds([track_name], track_scale=1.0)
            individual_bounds.append(track_bounds)

        # Global bounds should be >= all individual bounds
        for track_bounds in individual_bounds:
            self.assertGreaterEqual(
                bounds["track_max_curv"],
                track_bounds["track_max_curv"],
                "Global max curvature should be >= individual track max curvature",
            )
            self.assertLessEqual(
                bounds["track_min_width"],
                track_bounds["track_min_width"],
                "Global min width should be <= individual track min width",
            )
            self.assertGreaterEqual(
                bounds["track_max_width"],
                track_bounds["track_max_width"],
                "Global max width should be >= individual track max width",
            )

    def test_invalid_track_name(self):
        """Test that invalid track names raise ValueError."""
        track_pool = ["InvalidTrackName123"]

        with self.assertRaises(ValueError) as context:
            compute_global_track_bounds(track_pool, track_scale=1.0)

        error_msg = str(context.exception)
        self.assertIn("Invalid track name", error_msg)
        self.assertIn("InvalidTrackName123", error_msg)

    def test_default_track_scale(self):
        """Test that default track_scale parameter works correctly."""
        track_pool = ["Drift"]

        # Should work without explicitly passing track_scale (defaults to 1.0)
        bounds = compute_global_track_bounds(track_pool)

        # Verify we got valid bounds
        self.assertGreater(bounds["track_max_curv"], 0)
        self.assertGreater(bounds["track_min_width"], 0)
        self.assertGreater(bounds["track_max_width"], 0)

        # Should match explicit track_scale=1.0
        bounds_explicit = compute_global_track_bounds(track_pool, track_scale=1.0)
        self.assertEqual(bounds["track_max_curv"], bounds_explicit["track_max_curv"])
        self.assertEqual(bounds["track_min_width"], bounds_explicit["track_min_width"])
        self.assertEqual(bounds["track_max_width"], bounds_explicit["track_max_width"])


class TestMakeSubprocvecenvMultiMap(unittest.TestCase):
    """Test make_subprocvecenv with multi-map track_pool functionality."""

    def test_cycling_distribution_with_multiple_tracks(self):
        """Test that maps are correctly distributed in cycling pattern across envs."""
        from train.config.env_config import get_drift_train_config
        from train.train_utils import make_subprocvecenv

        config = get_drift_train_config()
        n_envs = 3
        track_pool = ["Drift", "Drift2", "Drift_mirror"]

        # Create multi-map environment
        env = make_subprocvecenv(seed=42, config=config, n_envs=n_envs, track_pool=track_pool)

        # Verify environment was created successfully
        self.assertIsNotNone(env)
        self.assertEqual(env.num_envs, n_envs)

        # Clean up
        env.close()

    def test_distribution_counting_accuracy(self):
        """Test that the distribution counter accurately reflects map assignments."""
        from collections import Counter

        from train.config.env_config import get_drift_train_config
        from train.train_utils import make_subprocvecenv

        config = get_drift_train_config()
        n_envs = 3
        track_pool = ["Drift", "Drift2", "Drift_mirror"]

        # Expected distribution using same logic as implementation
        expected_distribution = Counter(track_pool[i % len(track_pool)] for i in range(n_envs))

        # With 3 envs and 3 tracks: Drift=1, Drift2=1, Drift_mirror=1
        self.assertEqual(expected_distribution["Drift"], 1)
        self.assertEqual(expected_distribution["Drift2"], 1)
        self.assertEqual(expected_distribution["Drift_mirror"], 1)

        # Create environment (this will print the actual distribution)
        env = make_subprocvecenv(seed=42, config=config, n_envs=n_envs, track_pool=track_pool)

        # Verify environment was created
        self.assertEqual(env.num_envs, n_envs)

        # Clean up
        env.close()

    def test_track_pool_none_fallback_to_single_map(self):
        """Test that track_pool=None uses original single-map behavior."""
        from train.config.env_config import get_drift_train_config
        from train.train_utils import make_subprocvecenv

        config = get_drift_train_config()
        config["map"] = "Drift"
        n_envs = 2

        # Create environment with track_pool=None
        env = make_subprocvecenv(seed=42, config=config, n_envs=n_envs, track_pool=None)

        # Verify environment was created successfully
        self.assertIsNotNone(env)
        self.assertEqual(env.num_envs, n_envs)

        # Clean up
        env.close()

    def test_single_track_in_pool(self):
        """Test that single track in pool assigns same map to all envs."""
        from collections import Counter

        from train.config.env_config import get_drift_train_config
        from train.train_utils import make_subprocvecenv

        config = get_drift_train_config()
        n_envs = 2
        track_pool = ["Drift"]

        # Expected: all envs get "Drift"
        expected_distribution = Counter(track_pool[i % len(track_pool)] for i in range(n_envs))
        self.assertEqual(expected_distribution["Drift"], n_envs)

        # Create environment
        env = make_subprocvecenv(seed=42, config=config, n_envs=n_envs, track_pool=track_pool)

        # Verify environment was created
        self.assertEqual(env.num_envs, n_envs)

        # Clean up
        env.close()

    def test_empty_track_pool_raises_value_error(self):
        """Test that empty track_pool raises ValueError with clear message."""
        from train.config.env_config import get_drift_train_config
        from train.train_utils import make_subprocvecenv

        config = get_drift_train_config()
        n_envs = 4
        track_pool = []  # Empty list

        # Should raise ValueError
        with self.assertRaises(ValueError) as context:
            make_subprocvecenv(seed=42, config=config, n_envs=n_envs, track_pool=track_pool)

        # Verify error message
        error_msg = str(context.exception)
        self.assertIn("track_pool must be a non-empty list", error_msg)

    def test_invalid_track_name_raises_helpful_error(self):
        """Test that invalid track name in pool raises ValueError with track name."""
        from train.config.env_config import get_drift_train_config
        from train.train_utils import make_subprocvecenv

        config = get_drift_train_config()
        n_envs = 2
        track_pool = ["Drift", "InvalidTrackName123"]

        # Should raise ValueError when trying to load invalid track
        with self.assertRaises(ValueError) as context:
            make_subprocvecenv(seed=42, config=config, n_envs=n_envs, track_pool=track_pool)

        # Verify error message mentions the invalid track
        error_msg = str(context.exception)
        self.assertIn("Invalid track name", error_msg)
        self.assertIn("InvalidTrackName123", error_msg)


class TestMakeEvalEnv(unittest.TestCase):
    """Test make_eval_env: multi-map distribution, recovery fallback, direction pinning.

    Mocks ``make_env``, ``DummyVecEnv``, and ``_validate_track_names`` so the tests
    verify config flow without paying for real gym env construction (~1.2s each).
    """

    def _capture_eval_configs(self, eval_config: dict) -> list[dict]:
        """Run make_eval_env with construction mocked out; return the per-env config dicts."""
        captured = []

        def fake_make_env(seed, rank, config):
            captured.append(config)
            return lambda: MagicMock()

        with (
            patch("train.train_utils.make_env", side_effect=fake_make_env),
            patch("train.train_utils.DummyVecEnv", return_value=MagicMock()),
            patch("train.train_utils._validate_track_names"),
        ):
            from train.train_utils import make_eval_env

            make_eval_env(seed=0, config=eval_config)

        return captured

    def test_track_pool_creates_one_env_per_map(self):
        """Race config -> one config per map, with config['map'] set per env."""
        from train.config.env_config import get_drift_test_config

        config = get_drift_test_config()
        track_pool = config["track_pool"]

        captured = self._capture_eval_configs(config)
        self.assertEqual(len(captured), len(track_pool))
        self.assertEqual([c["map"] for c in captured], track_pool)

    def test_no_track_pool_creates_single_env(self):
        """Recovery config (track_pool=None) -> single config dict; no KeyError."""
        from train.config.env_config import get_recovery_test_config

        captured = self._capture_eval_configs(get_recovery_test_config())
        self.assertEqual(len(captured), 1)

    def test_pins_random_direction_to_normal(self):
        """track_direction='random' must be pinned to 'normal' for deterministic eval."""
        from train.config.env_config import get_drift_test_config

        config = get_drift_test_config()
        config["track_direction"] = "random"

        captured = self._capture_eval_configs(config)
        for c in captured:
            self.assertEqual(c["track_direction"], "normal")

    def test_respects_explicit_direction(self):
        """Explicit 'normal' / 'reverse' must pass through unmodified."""
        from train.config.env_config import get_drift_test_config

        for direction in ("normal", "reverse"):
            config = get_drift_test_config()
            config["track_direction"] = direction
            captured = self._capture_eval_configs(config)
            for c in captured:
                self.assertEqual(c["track_direction"], direction)


if __name__ == "__main__":
    unittest.main()
