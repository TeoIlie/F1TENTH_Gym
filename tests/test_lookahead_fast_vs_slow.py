"""
Test to validate that sample_lookahead_curvatures_fast produces identical
results to sample_lookahead_curvatures while being significantly faster.
"""

import os
import time

import matplotlib.pyplot as plt
import numpy as np

from gymkhana.envs.observation import sample_lookahead_curvatures, sample_lookahead_curvatures_fast
from gymkhana.envs.track.cubic_spline import CubicSpline2D


class MockRaceline:
    """Mock raceline object (simulates Raceline with spline attribute)."""

    def __init__(self, spline):
        self.spline = spline


class MockTrack:
    """Mock track object for testing curvature sampling."""

    def __init__(self, spline):
        # Track has centerline, which has spline (CubicSpline2D)
        self.centerline = MockRaceline(spline)


def create_test_tracks():
    """Create various test tracks with different geometries."""
    tracks = {}

    # Track 1: Circle
    radius = 10.0
    theta = np.linspace(0, 2 * np.pi, 100)[:-1]
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    spline = CubicSpline2D(x, y)
    tracks["circle"] = MockTrack(spline)

    # Track 2: Figure-8 (lemniscate)
    t = np.linspace(0, 2 * np.pi, 200)[:-1]
    a = 5.0
    x = a * np.cos(t) / (1 + np.sin(t) ** 2)
    y = a * np.sin(t) * np.cos(t) / (1 + np.sin(t) ** 2)
    spline = CubicSpline2D(x, y)
    tracks["figure8"] = MockTrack(spline)

    # Track 3: Oval (straights + curves)
    points = []
    # Straight section
    points.extend([[x, 0] for x in np.linspace(0, 20, 50)])
    # Curved section 1
    theta = np.linspace(0, np.pi, 30)
    r = 8.0
    points.extend([[20 + r * np.cos(t), r + r * np.sin(t)] for t in theta])
    # Straight section back
    points.extend([[x, 2 * r] for x in np.linspace(20, 0, 50)])
    # Curved section 2
    theta = np.linspace(np.pi, 2 * np.pi, 30)
    points.extend([[r * np.cos(t), r + r * np.sin(t)] for t in theta])

    points = np.array(points)
    spline = CubicSpline2D(points[:, 0], points[:, 1])
    tracks["oval"] = MockTrack(spline)

    return tracks


def test_numerical_equivalence():
    """Test that both implementations produce numerically equivalent results."""
    print("=" * 70)
    print("Testing Numerical Equivalence")
    print("=" * 70)

    tracks = create_test_tracks()

    # Test parameters
    test_configs = [
        {"n_points": 5, "ds": 0.2},
        {"n_points": 10, "ds": 0.3},
        {"n_points": 20, "ds": 0.5},
        {"n_points": 15, "ds": 0.1},
    ]

    failures = []

    for track_name, track in tracks.items():
        print(f"\n{track_name.upper()} Track:")
        print("-" * 70)

        track_length = track.centerline.spline.s[-1]

        for config in test_configs:
            n_points = config["n_points"]
            ds = config["ds"]

            # Test at multiple positions along the track
            test_positions = [0.0, track_length * 0.25, track_length * 0.5, track_length * 0.75]

            max_abs_diff = 0.0
            max_rel_diff = 0.0

            for current_s in test_positions:
                # Call both implementations
                curv_slow = sample_lookahead_curvatures(track, current_s, n_points, ds)
                curv_fast = sample_lookahead_curvatures_fast(track, current_s, n_points, ds)

                # Compute differences
                abs_diff = np.abs(curv_slow - curv_fast)
                max_abs_diff = max(max_abs_diff, np.max(abs_diff))

                # Relative difference (avoid division by zero)
                with np.errstate(divide="ignore", invalid="ignore"):
                    rel_diff = np.abs((curv_slow - curv_fast) / (curv_slow + 1e-10))
                    rel_diff = np.where(np.isfinite(rel_diff), rel_diff, 0.0)
                    max_rel_diff = max(max_rel_diff, np.max(rel_diff))

            # Check if differences are within acceptable tolerance
            tolerance_abs = 1e-6  # Absolute tolerance
            tolerance_rel = 1e-4  # Relative tolerance (0.01%)

            passed = (max_abs_diff < tolerance_abs) and (max_rel_diff < tolerance_rel)
            status = "✓" if passed else "✗"

            print(
                f"  {status} n={n_points:2d}, ds={ds:.1f}m  →  "
                f"Max abs diff: {max_abs_diff:.2e}, Max rel diff: {max_rel_diff:.2%}"
            )

            if not passed:
                failures.append(
                    f"{track_name} track (n={n_points}, ds={ds}): "
                    f"abs_diff={max_abs_diff:.2e}, rel_diff={max_rel_diff:.2%}"
                )

    print("=" * 70)

    # Use pytest assertion
    assert len(failures) == 0, f"Numerical equivalence failed for {len(failures)} configuration(s):\n" + "\n".join(
        f"  - {f}" for f in failures
    )


def test_performance_comparison():
    """Benchmark the performance difference between implementations."""
    print("\n" + "=" * 70)
    print("Performance Benchmarking")
    print("=" * 70)

    tracks = create_test_tracks()

    # Configuration for performance test
    n_points = 10
    ds = 0.3
    n_iterations = 1000

    speedups = []

    for track_name, track in tracks.items():
        track_length = track.centerline.spline.s[-1]

        # Generate random positions for testing
        np.random.seed(42)  # For reproducibility
        test_positions = np.random.uniform(0, track_length, n_iterations)

        # Benchmark slow version
        start = time.perf_counter()
        for s in test_positions:
            sample_lookahead_curvatures(track, s, n_points, ds)
        time_slow = time.perf_counter() - start

        # Benchmark fast version (including JIT compilation on first call)
        start = time.perf_counter()
        for s in test_positions:
            sample_lookahead_curvatures_fast(track, s, n_points, ds)
        time_fast = time.perf_counter() - start

        # Compute speedup
        speedup = time_slow / time_fast
        speedups.append(speedup)

        print(f"\n{track_name.upper()} Track ({n_iterations} iterations):")
        print(f"  Regular implementation: {time_slow * 1000:.2f} ms  ({time_slow * 1e6 / n_iterations:.2f} µs/call)")
        print(f"  Fast implementation:    {time_fast * 1000:.2f} ms  ({time_fast * 1e6 / n_iterations:.2f} µs/call)")
        print(f"  Speedup: {speedup:.1f}x")

    avg_speedup = np.mean(speedups)
    print("\n" + "=" * 70)
    print(f"Average speedup: {avg_speedup:.1f}x")
    print("=" * 70)

    # Assert that fast implementation is actually faster
    assert avg_speedup > 1.0, f"Fast implementation should be faster (got {avg_speedup:.1f}x speedup)"


def test_edge_cases():
    """Test edge cases and boundary conditions."""
    print("\n" + "=" * 70)
    print("Testing Edge Cases")
    print("=" * 70)

    # Create a simple circular track
    radius = 10.0
    theta = np.linspace(0, 2 * np.pi, 100)[:-1]
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    spline = CubicSpline2D(x, y)
    track = MockTrack(spline)
    track_length = track.centerline.spline.s[-1]

    test_cases = [
        ("Start of track", 0.0, 10, 0.3),
        ("Near end with wrap-around", track_length - 1.0, 10, 0.3),
        ("Small ds", track_length * 0.5, 10, 0.05),
        ("Large ds", track_length * 0.5, 10, 2.0),
        ("Single point", track_length * 0.25, 1, 0.3),
        ("Many points", track_length * 0.25, 50, 0.1),
    ]

    failures = []
    tolerance = 1e-5

    for name, current_s, n_points, ds in test_cases:
        try:
            curv_slow = sample_lookahead_curvatures(track, current_s, n_points, ds)
            curv_fast = sample_lookahead_curvatures_fast(track, current_s, n_points, ds)

            # Check shapes match
            assert curv_slow.shape == curv_fast.shape, f"Shape mismatch: {curv_slow.shape} vs {curv_fast.shape}"

            # Check numerical agreement
            max_diff = np.max(np.abs(curv_slow - curv_fast))

            passed = max_diff < tolerance
            status = "✓" if passed else "✗"

            print(f"  {status} {name:25s}  max_diff={max_diff:.2e}")

            if not passed:
                failures.append(f"{name}: max_diff={max_diff:.2e} (tolerance={tolerance:.2e})")
                print(f"      Slow: {curv_slow}")
                print(f"      Fast: {curv_fast}")

        except Exception as e:
            print(f"  ✗ {name:25s}  ERROR: {e}")
            failures.append(f"{name}: Exception - {e}")

    print("=" * 70)

    # Use pytest assertion
    assert len(failures) == 0, f"Edge case tests failed for {len(failures)} case(s):\n" + "\n".join(
        f"  - {f}" for f in failures
    )


def test_visualization():
    """Create visualization comparing outputs of both implementations."""
    print("\n" + "=" * 70)
    print("Generating Comparison Visualization")
    print("=" * 70)

    # Create a figure-8 track
    t = np.linspace(0, 2 * np.pi, 200)[:-1]
    a = 5.0
    x = a * np.cos(t) / (1 + np.sin(t) ** 2)
    y = a * np.sin(t) * np.cos(t) / (1 + np.sin(t) ** 2)
    spline = CubicSpline2D(x, y)
    track = MockTrack(spline)

    # Sample along the entire track
    track_length = track.centerline.spline.s[-1]
    s_positions = np.linspace(0, track_length, 50)

    n_points = 10
    ds = 0.3

    # Collect results
    slow_results = []
    fast_results = []
    differences = []

    for s in s_positions:
        curv_slow = sample_lookahead_curvatures(track, s, n_points, ds)
        curv_fast = sample_lookahead_curvatures_fast(track, s, n_points, ds)

        slow_results.append(curv_slow)
        fast_results.append(curv_fast)
        differences.append(np.abs(curv_slow - curv_fast))

    slow_results = np.array(slow_results)
    fast_results = np.array(fast_results)
    differences = np.array(differences)

    # Create visualization
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # Plot 1: Regular implementation
    im1 = axes[0].imshow(slow_results.T, aspect="auto", cmap="RdBu_r", extent=[0, track_length, n_points, 0])
    axes[0].set_xlabel("Arc length position s [m]")
    axes[0].set_ylabel("Lookahead point index")
    axes[0].set_title("sample_lookahead_curvatures (Regular)")
    plt.colorbar(im1, ax=axes[0], label="Curvature [1/m]")

    # Plot 2: Fast implementation
    im2 = axes[1].imshow(fast_results.T, aspect="auto", cmap="RdBu_r", extent=[0, track_length, n_points, 0])
    axes[1].set_xlabel("Arc length position s [m]")
    axes[1].set_ylabel("Lookahead point index")
    axes[1].set_title("sample_lookahead_curvatures_fast (Numba-optimized)")
    plt.colorbar(im2, ax=axes[1], label="Curvature [1/m]")

    # Plot 3: Absolute difference
    im3 = axes[2].imshow(differences.T, aspect="auto", cmap="RdBu_r", extent=[0, track_length, n_points, 0])
    axes[2].set_xlabel("Arc length position s [m]")
    axes[2].set_ylabel("Lookahead point index")
    axes[2].set_title("Absolute Difference |slow - fast|")
    plt.colorbar(im3, ax=axes[2], label="Absolute difference [1/m]")

    plt.tight_layout()
    output_dir = os.path.join(os.path.dirname(__file__), "..", "figures", "tests")
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(os.path.join(output_dir, "curvature_fast_slow_validation.png"), dpi=300, bbox_inches="tight")

    max_diff = np.max(differences)
    mean_diff = np.mean(differences)

    print(f"  Maximum difference: {max_diff:.2e}")
    print(f"  Mean difference:    {mean_diff:.2e}")
    print("=" * 70)

    # Use pytest assertion
    tolerance = 1e-5
    assert max_diff < tolerance, (
        f"Maximum difference {max_diff:.2e} exceeds tolerance {tolerance:.2e}. Mean difference: {mean_diff:.2e}"
    )

    return plt


if __name__ == "__main__":
    """
    Run tests manually for debugging purposes.
    For proper test execution, use: pytest test_lookahead_fast_vs_slow.py
    """
    print("\n" + "=" * 70)
    print("LOOKAHEAD CURVATURE SAMPLING: FAST vs SLOW COMPARISON")
    print("=" * 70)
    print("Running tests manually (use pytest for proper test execution)")
    print("=" * 70)

    # Run all tests - they will raise AssertionError if they fail
    try:
        test_numerical_equivalence()
        print("\n✓ Numerical equivalence test passed")
    except AssertionError as e:
        print(f"\n✗ Numerical equivalence test failed:\n{e}")

    try:
        test_edge_cases()
        print("\n✓ Edge cases test passed")
    except AssertionError as e:
        print(f"\n✗ Edge cases test failed:\n{e}")

    try:
        test_performance_comparison()
        print("\n✓ Performance comparison test passed")
    except AssertionError as e:
        print(f"\n✗ Performance comparison test failed:\n{e}")

    try:
        plt = test_visualization()
        plt.show()
        print("\n✓ Visualization test passed")
    except AssertionError as e:
        print(f"\n✗ Visualization test failed:\n{e}")

    print("\n" + "=" * 70)
    print("Manual test run complete. Use 'pytest test_lookahead_fast_vs_slow.py' for proper test execution.")
    print("=" * 70)
