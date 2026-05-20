import pathlib
import tarfile
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from gymkhana.envs.track import Raceline, Track, find_track_dir
from gymkhana.envs.track.track_utils import get_min_max_curvature, get_min_max_track_width


class TestTrack(unittest.TestCase):
    def test_error_handling(self):
        wrong_track_name = "i_dont_exists"
        self.assertRaises(FileNotFoundError, Track.from_track_name, wrong_track_name)

    def test_raceline(self):
        track_name = "Spielberg"
        track = Track.from_track_name(track_name)

        # check raceline is not None
        self.assertNotEqual(track.raceline, None)

        # check loaded raceline match the one in the csv file
        track_dir = find_track_dir(track_name)
        assert track_dir is not None and track_dir.exists(), "track_dir does not exist"

        raceline = np.loadtxt(track_dir / f"{track_name}_raceline.csv", delimiter=";")
        s_idx, x_idx, y_idx, psi_idx, kappa_idx, vx_idx, ax_idx = range(7)

        self.assertTrue(np.isclose(track.raceline.ss, raceline[:, s_idx]).all())
        self.assertTrue(np.isclose(track.raceline.xs, raceline[:, x_idx]).all())
        self.assertTrue(np.isclose(track.raceline.ys, raceline[:, y_idx]).all())
        self.assertTrue(np.isclose(track.raceline.yaws, raceline[:, psi_idx]).all())
        self.assertTrue(np.isclose(track.raceline.ks, raceline[:, kappa_idx]).all())
        self.assertTrue(np.isclose(track.raceline.vxs, raceline[:, vx_idx]).all())
        self.assertTrue(np.isclose(track.raceline.axs, raceline[:, ax_idx]).all())

    def test_map_dir_structure(self):
        """
        Check that the map dir structure is correct:
        - maps/
            - Trackname/
                - Trackname_map.*               # map image
                - Trackname_map.yaml            # map specification
                - [Trackname_raceline.csv]      # raceline (optional)
                - [Trackname_centerline.csv]    # centerline (optional)
        """
        mapdir = pathlib.Path(__file__).parent.parent / "maps"
        non_track_dirs = {"global_racetrajectory_optimization", "raceline_config"}
        for trackdir in mapdir.iterdir():
            if trackdir.is_file() or trackdir.name.startswith("."):
                continue
            if trackdir.name in non_track_dirs:
                continue

            # check subdir is capitalized (at least first letter is capitalized)
            trackdirname = trackdir.stem
            self.assertTrue(trackdirname[0].isupper(), f"trackdir {trackdirname} is not capitalized")

            # check map spec file exists
            file_spec = trackdir / f"{trackdirname}_map.yaml"
            self.assertTrue(
                file_spec.exists(),
                f"map spec file {file_spec} does not exist in {trackdir}",
            )

            # read map image file from spec
            map_spec = Track.load_spec(track=str(trackdir), filespec=str(file_spec))
            file_image = trackdir / map_spec.image

            # check map image file exists
            self.assertTrue(
                file_image.exists(),
                f"map image file {file_image} does not exist in {trackdir}",
            )

            # check raceline and centerline files
            file_raceline = trackdir / f"{trackdir.stem}_raceline.csv"
            file_centerline = trackdir / f"{trackdir.stem}_centerline.csv"

            if file_raceline.exists():
                # try to load raceline files
                # it will raise an assertion error if the file format are not valid
                Raceline.from_raceline_file(file_raceline)

            if file_centerline.exists():
                # try to load raceline files
                # it will raise an assertion error if the file format are not valid
                Raceline.from_centerline_file(file_centerline)

    def test_find_track_dir_downloads_when_not_cached(self):
        """Test that find_track_dir downloads, extracts, and caches a track not found locally."""
        fake_track = "FakeTestTrack"

        # Build a minimal tarball with the expected directory structure
        with tempfile.TemporaryDirectory() as build_dir:
            track_dir = pathlib.Path(build_dir) / fake_track
            track_dir.mkdir()
            (track_dir / f"{fake_track}_map.yaml").write_text("image: map.png\nresolution: 0.05\norigin: [0,0,0]")
            (track_dir / "map.png").write_bytes(b"\x89PNG fake")

            tar_path = pathlib.Path(build_dir) / f"{fake_track}.tar.xz"
            with tarfile.open(tar_path, "w:xz") as tar:
                tar.add(track_dir, arcname=fake_track)
            tar_bytes = tar_path.read_bytes()

        mock_resp = MagicMock(status_code=200, content=tar_bytes)

        with tempfile.TemporaryDirectory() as fake_home, tempfile.TemporaryDirectory() as fake_tmp:
            with (
                patch.object(pathlib.Path, "home", return_value=pathlib.Path(fake_home)),
                patch("gymkhana.envs.track.track_utils.tempfile.gettempdir", return_value=fake_tmp),
                patch("gymkhana.envs.track.track_utils.requests.get", return_value=mock_resp) as mock_get,
            ):
                # First call: should download and extract
                result = find_track_dir(fake_track)

                from gymkhana.envs.track.track_utils import MAPS_URL

                expected_url = f"{MAPS_URL}/{fake_track}.tar.xz"
                mock_get.assert_called_once_with(url=expected_url, allow_redirects=True)
                self.assertTrue(result.exists())
                self.assertEqual(result.stem, fake_track)
                self.assertTrue((result / f"{fake_track}_map.yaml").exists())

                # Second call: should use cache, no additional download
                mock_get.reset_mock()
                result2 = find_track_dir(fake_track)

                mock_get.assert_not_called()
                self.assertEqual(result, result2)

    def test_find_track_dir_raises_on_404(self):
        """Test that a 404 response raises FileNotFoundError."""
        mock_resp = MagicMock(status_code=404)

        with tempfile.TemporaryDirectory() as fake_home:
            with patch.object(pathlib.Path, "home", return_value=pathlib.Path(fake_home)):
                with patch("gymkhana.envs.track.track_utils.requests.get", return_value=mock_resp):
                    with self.assertRaises(FileNotFoundError):
                        find_track_dir("NonExistentTrack")

    def test_find_track_dir_uses_cache(self):
        """Test that a cached track is returned without downloading."""
        fake_track = "CachedTestTrack"

        with tempfile.TemporaryDirectory() as fake_home:
            # Pre-populate the cache
            cache_dir = pathlib.Path(fake_home) / ".gymkhana" / "maps" / fake_track
            cache_dir.mkdir(parents=True)
            (cache_dir / "dummy.txt").write_text("cached")

            with patch.object(pathlib.Path, "home", return_value=pathlib.Path(fake_home)):
                with patch("gymkhana.envs.track.track_utils.requests.get") as mock_get:
                    result = find_track_dir(fake_track)

                    mock_get.assert_not_called()
                    self.assertEqual(result, cache_dir)

    def test_frenet_to_cartesian(self):
        track_name = "Spielberg"
        track = Track.from_track_name(track_name)

        # Check frenet to cartesian conversion
        # using the track's xs, ys
        for s, x, y in zip(track.centerline.ss, track.centerline.xs, track.centerline.ys):
            x_, y_, _ = track.frenet_to_cartesian(s, 0, 0)
            self.assertAlmostEqual(x, x_, places=4)
            self.assertAlmostEqual(y, y_, places=4)

    def test_frenet_to_cartesian_to_frenet(self):
        track_name = "Spielberg"
        track = Track.from_track_name(track_name)

        # check frenet to cartesian conversion
        s_ = 0
        for s in np.linspace(0, 1, 10):
            x, y, psi = track.frenet_to_cartesian(s, 0, 0)
            s_, d, _ = track.cartesian_to_frenet(x, y, psi, s_guess=s_)
            self.assertAlmostEqual(s, s_, places=4)
            self.assertAlmostEqual(d, 0, places=4)

        # check frenet to cartesian conversion
        # with non-zero lateral offset
        s_ = 0
        for s in np.linspace(0, 1, 10):
            d = np.random.uniform(-1.0, 1.0)
            x, y, psi = track.frenet_to_cartesian(s, d, 0)
            s_, d_, _ = track.cartesian_to_frenet(x, y, psi, s_guess=s_)
            # Handle edge case where we are checking for s=0 but s_ is the last s (same point, but different s)
            self.assertTrue(
                np.isclose(s, s_, atol=1e-4) or np.isclose(s + track.centerline.spline.s[-1], s_, atol=1e-4)
            )
            self.assertAlmostEqual(d, d_, places=4)

    def test_get_min_max_track_width(self):
        """Test extraction of min/max track width from centerline."""
        # Create mock track with known width values
        w_lefts = np.array([1.0, 2.0, 3.0, 1.5], dtype=np.float32)
        w_rights = np.array([1.5, 2.5, 3.5, 2.0], dtype=np.float32)
        # Total widths: [2.5, 4.5, 6.5, 3.5]
        # Expected min = 2.5, max = 6.5

        centerline = Raceline(
            xs=np.array([0.0, 1.0, 2.0, 3.0], dtype=np.float32),
            ys=np.array([0.0, 1.0, 2.0, 3.0], dtype=np.float32),
            velxs=np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32),
            w_lefts=w_lefts,
            w_rights=w_rights,
        )

        track = Track(
            spec=None,
            occupancy_map=np.zeros((10, 10)),
            centerline=centerline,
            raceline=None,
        )

        # Call the function
        min_width, max_width = get_min_max_track_width(track)

        # Verify exact values
        self.assertEqual(min_width, 2.5, "min_width should be exactly 2.5")
        self.assertEqual(max_width, 6.5, "max_width should be exactly 6.5")

        # Verify return types
        self.assertIsInstance(min_width, float)
        self.assertIsInstance(max_width, float)

    def test_get_min_max_track_width_error_cases(self):
        """Test error handling for get_min_max_track_width."""
        # Create a mock track with no centerline
        track_no_centerline = Track(
            spec=None,
            occupancy_map=np.zeros((10, 10)),
            centerline=None,
            raceline=None,
        )

        # Should raise ValueError when centerline is None
        with self.assertRaises(ValueError) as context:
            get_min_max_track_width(track_no_centerline)
        self.assertIn("centerline not available", str(context.exception).lower())

        # Create a mock track with centerline but no width data
        raceline_no_widths = Raceline(
            xs=np.array([0.0, 1.0, 2.0]),
            ys=np.array([0.0, 1.0, 2.0]),
            velxs=np.array([1.0, 1.0, 1.0]),
            w_lefts=None,  # Missing width data
            w_rights=None,
        )

        track_no_widths = Track(
            spec=None,
            occupancy_map=np.zeros((10, 10)),
            centerline=raceline_no_widths,
            raceline=None,
        )

        # Should raise ValueError when width data is missing
        with self.assertRaises(ValueError) as context:
            get_min_max_track_width(track_no_widths)
        self.assertIn("width data not available", str(context.exception).lower())

    def test_get_min_max_curvature(self):
        """Test extraction of min and max curvature with symmetric bounds based on max absolute value."""
        # Test case 1: Asymmetric curvatures with max positive value larger
        # kappas = [-1.0, 4.0] → max_abs = 4.0 → bounds should be (-4.0, 4.0)
        kappas_1 = np.array([-1.0, 4.0], dtype=np.float32)

        centerline_1 = Raceline(
            xs=np.array([0.0, 1.0], dtype=np.float32),
            ys=np.array([0.0, 1.0], dtype=np.float32),
            velxs=np.array([1.0, 1.0], dtype=np.float32),
            kappas=kappas_1,
        )

        track_1 = Track(
            spec=None,
            occupancy_map=np.zeros((10, 10)),
            centerline=centerline_1,
            raceline=None,
        )

        min_curv_1, max_curv_1 = get_min_max_curvature(track_1)

        # Verify symmetric bounds based on max absolute value (4.0)
        self.assertAlmostEqual(min_curv_1, -4.0, places=5, msg="min curvature should be -4.0 (symmetric)")
        self.assertAlmostEqual(max_curv_1, 4.0, places=5, msg="max curvature should be 4.0")
        self.assertIsInstance(min_curv_1, float)
        self.assertIsInstance(max_curv_1, float)

        # Test case 2: Asymmetric curvatures with max negative value larger in magnitude
        # kappas = [-4.0, 1.0] → max_abs = 4.0 → bounds should be (-4.0, 4.0)
        kappas_2 = np.array([-4.0, 1.0], dtype=np.float32)

        centerline_2 = Raceline(
            xs=np.array([0.0, 1.0], dtype=np.float32),
            ys=np.array([0.0, 1.0], dtype=np.float32),
            velxs=np.array([1.0, 1.0], dtype=np.float32),
            kappas=kappas_2,
        )

        track_2 = Track(
            spec=None,
            occupancy_map=np.zeros((10, 10)),
            centerline=centerline_2,
            raceline=None,
        )

        min_curv_2, max_curv_2 = get_min_max_curvature(track_2)

        # Verify symmetric bounds based on max absolute value (4.0)
        self.assertAlmostEqual(min_curv_2, -4.0, places=5, msg="min curvature should be -4.0")
        self.assertAlmostEqual(max_curv_2, 4.0, places=5, msg="max curvature should be 4.0 (symmetric)")
        self.assertIsInstance(min_curv_2, float)
        self.assertIsInstance(max_curv_2, float)

        # Test case 3: Already symmetric curvatures
        # kappas = [-3.5, 0.0, 3.5] → max_abs = 3.5 → bounds should be (-3.5, 3.5)
        kappas_3 = np.array([-3.5, 0.0, 3.5], dtype=np.float32)

        centerline_3 = Raceline(
            xs=np.array([0.0, 1.0, 2.0], dtype=np.float32),
            ys=np.array([0.0, 1.0, 2.0], dtype=np.float32),
            velxs=np.array([1.0, 1.0, 1.0], dtype=np.float32),
            kappas=kappas_3,
        )

        track_3 = Track(
            spec=None,
            occupancy_map=np.zeros((10, 10)),
            centerline=centerline_3,
            raceline=None,
        )

        min_curv_3, max_curv_3 = get_min_max_curvature(track_3)

        # Verify symmetric bounds
        self.assertAlmostEqual(min_curv_3, -3.5, places=5, msg="min curvature should be -3.5")
        self.assertAlmostEqual(max_curv_3, 3.5, places=5, msg="max curvature should be 3.5")
        self.assertIsInstance(min_curv_3, float)
        self.assertIsInstance(max_curv_3, float)

    def test_get_min_max_curvature_error_cases(self):
        """Test error handling for get_min_max_curvature."""
        # Create a mock track with no centerline
        track_no_centerline = Track(
            spec=None,
            occupancy_map=np.zeros((10, 10)),
            centerline=None,
            raceline=None,
        )

        # Should raise ValueError when centerline is None
        with self.assertRaises(ValueError) as context:
            get_min_max_curvature(track_no_centerline)
        self.assertIn("centerline not available", str(context.exception).lower())

        # Create a mock track with centerline but no curvature data
        raceline_no_curvature = Raceline(
            xs=np.array([0.0, 1.0, 2.0]),
            ys=np.array([0.0, 1.0, 2.0]),
            velxs=np.array([1.0, 1.0, 1.0]),
            kappas=None,  # Missing curvature data
        )

        track_no_curvature = Track(
            spec=None,
            occupancy_map=np.zeros((10, 10)),
            centerline=raceline_no_curvature,
            raceline=None,
        )

        # Should raise ValueError when curvature data is missing
        with self.assertRaises(ValueError) as context:
            get_min_max_curvature(track_no_curvature)
        self.assertIn("curvature data", str(context.exception).lower())

    def test_centerline_validation_left_width_exceeds_tolerance(self):
        """Test that centerline with w_left > w_right by >10% raises ValueError."""
        # Create temporary centerline file
        # Format: [x, y, w_right, w_left]
        # w_right = 2.0, w_left = 3.0 (50% difference, exceeds 10% tolerance)
        waypoints = np.array(
            [
                [0.0, 0.0, 2.0, 3.0],
                [1.0, 0.0, 2.0, 3.0],
                [2.0, 0.0, 2.0, 3.0],
                [3.0, 0.0, 2.0, 3.0],
            ]
        )

        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
            np.savetxt(f, waypoints, delimiter=",")
            filepath = pathlib.Path(f.name)

        try:
            # Should raise ValueError due to asymmetry
            with self.assertRaises(ValueError) as context:
                Raceline.from_centerline_file(filepath)

            error_msg = str(context.exception)
            self.assertIn("Centerline validation failed", error_msg)
            self.assertIn("w_right=2.000m, w_left=3.000m", error_msg)
            self.assertIn("exceeds 10% tolerance", error_msg)
        finally:
            filepath.unlink()

    def test_centerline_validation_right_width_exceeds_tolerance(self):
        """Test that centerline with w_right > w_left by >10% raises ValueError."""
        # Create temporary centerline file
        # Format: [x, y, w_right, w_left]
        # w_right = 3.5, w_left = 2.0 (43% difference, exceeds 10% tolerance)
        waypoints = np.array(
            [
                [0.0, 0.0, 3.5, 2.0],
                [1.0, 0.0, 3.5, 2.0],
                [2.0, 0.0, 3.5, 2.0],
                [3.0, 0.0, 3.5, 2.0],
            ]
        )

        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
            np.savetxt(f, waypoints, delimiter=",")
            filepath = pathlib.Path(f.name)

        try:
            # Should raise ValueError due to asymmetry
            with self.assertRaises(ValueError) as context:
                Raceline.from_centerline_file(filepath)

            error_msg = str(context.exception)
            self.assertIn("Centerline validation failed", error_msg)
            self.assertIn("w_right=3.500m, w_left=2.000m", error_msg)
            self.assertIn("exceeds 10% tolerance", error_msg)
        finally:
            filepath.unlink()

    def test_centerline_validation_left_width_within_tolerance(self):
        """Test that centerline with w_left slightly > w_right (<10%) loads successfully."""
        # Create temporary centerline file
        # Format: [x, y, w_right, w_left]
        # w_right = 2.0, w_left = 2.18 (9% difference, within 10% tolerance)
        waypoints = np.array(
            [
                [0.0, 0.0, 2.0, 2.18],
                [1.0, 0.0, 2.0, 2.18],
                [2.0, 1.0, 2.0, 2.18],
                [3.0, 1.0, 2.0, 2.18],
            ]
        )

        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
            np.savetxt(f, waypoints, delimiter=",")
            filepath = pathlib.Path(f.name)

        try:
            # Should load without error (within 10% tolerance)
            raceline = Raceline.from_centerline_file(filepath)
            self.assertIsNotNone(raceline)
            self.assertIsNotNone(raceline.xs)
            self.assertIsNotNone(raceline.ys)
        finally:
            filepath.unlink()

    def test_centerline_validation_right_width_within_tolerance(self):
        """Test that centerline with w_right slightly > w_left (<10%) loads successfully."""
        # Create temporary centerline file
        # Format: [x, y, w_right, w_left]
        # w_right = 2.2, w_left = 2.0 (9% difference, within 10% tolerance)
        waypoints = np.array(
            [
                [0.0, 0.0, 2.2, 2.0],
                [1.0, 0.0, 2.2, 2.0],
                [2.0, 1.0, 2.2, 2.0],
                [3.0, 1.0, 2.2, 2.0],
            ]
        )

        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
            np.savetxt(f, waypoints, delimiter=",")
            filepath = pathlib.Path(f.name)

        try:
            # Should load without error (within 10% tolerance)
            raceline = Raceline.from_centerline_file(filepath)
            self.assertIsNotNone(raceline)
            self.assertIsNotNone(raceline.xs)
            self.assertIsNotNone(raceline.ys)
        finally:
            filepath.unlink()

    def test_track_creates_reversed_versions_on_init(self):
        """Test that Track.__init__ creates both regular and reversed versions of centerline/raceline."""
        track_name = "Spielberg"
        track = Track.from_track_name(track_name)

        # Check regular versions exist
        self.assertIsNotNone(track.centerline_regular)
        self.assertIsNotNone(track.raceline_regular)

        # Check reversed versions exist
        self.assertIsNotNone(track.centerline_reversed)
        self.assertIsNotNone(track.raceline_reversed)

        # Check reversed versions are different objects
        self.assertIsNot(track.centerline_reversed, track.centerline_regular)
        self.assertIsNot(track.raceline_reversed, track.raceline_regular)

        # Check active references default to regular
        self.assertIs(track.centerline, track.centerline_regular)
        self.assertIs(track.raceline, track.raceline_regular)

    def test_track_set_direction_reversed_true(self):
        """Test that set_direction(reversed=True) activates reversed references."""
        track_name = "Spielberg"
        track = Track.from_track_name(track_name)

        # Set to reversed direction
        track.set_direction(reversed=True)

        # Check active references point to reversed versions
        self.assertIs(track.centerline, track.centerline_reversed)
        self.assertIs(track.raceline, track.raceline_reversed)

    def test_track_set_direction_reversed_false(self):
        """Test that set_direction(reversed=False) activates regular references."""
        track_name = "Spielberg"
        track = Track.from_track_name(track_name)

        # First set to reversed
        track.set_direction(reversed=True)
        self.assertIs(track.centerline, track.centerline_reversed)

        # Now set back to regular
        track.set_direction(reversed=False)

        # Check active references point to regular versions
        self.assertIs(track.centerline, track.centerline_regular)
        self.assertIs(track.raceline, track.raceline_regular)

    def test_track_raceline_defaults_to_centerline(self):
        """
        Test that when raceline is not provided, both regular and reversed raceline
        reference the centerline.
        """
        # Create track with centerline only (no raceline)
        centerline = Raceline(
            xs=np.array([0.0, 1.0, 2.0], dtype=np.float32),
            ys=np.array([0.0, 1.0, 0.0], dtype=np.float32),
            velxs=np.array([1.0, 1.0, 1.0], dtype=np.float32),
            w_lefts=np.array([1.0, 1.0, 1.0], dtype=np.float32),
            w_rights=np.array([1.0, 1.0, 1.0], dtype=np.float32),
        )

        track = Track(
            spec=None,
            occupancy_map=np.zeros((10, 10)),
            centerline=centerline,
            raceline=None,  # No raceline provided
        )

        # Check raceline_regular defaults to centerline_regular
        self.assertIs(track.raceline_regular, track.centerline_regular)

        # Check raceline_reversed references centerline_reversed
        self.assertIs(track.raceline_reversed, track.centerline_reversed)

    def test_track_raceline_reversed_shares_reference_when_defaulted(self):
        """
        Test that when raceline defaults to centerline, the reversed versions
        share the same reversed object reference.
        """
        # Create track with centerline only
        centerline = Raceline(
            xs=np.array([0.0, 1.0, 2.0], dtype=np.float32),
            ys=np.array([0.0, 1.0, 0.0], dtype=np.float32),
            velxs=np.array([1.0, 1.0, 1.0], dtype=np.float32),
            w_lefts=np.array([1.0, 1.0, 1.0], dtype=np.float32),
            w_rights=np.array([1.0, 1.0, 1.0], dtype=np.float32),
        )

        track = Track(
            spec=None,
            occupancy_map=np.zeros((10, 10)),
            centerline=centerline,
            raceline=None,
        )

        # Check that raceline_reversed and centerline_reversed are the same object
        self.assertIs(track.raceline_reversed, track.centerline_reversed)

        # This avoids unnecessary duplication when raceline defaults to centerline
