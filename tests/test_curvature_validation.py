"""
Test script to validate curvature calculations against known geometric shapes.

This test suite validates that the cubic spline curvature calculations are
mathematically correct by comparing against known analytical solutions.
"""

import os

import matplotlib.pyplot as plt
import numpy as np

from gymkhana.envs.track.cubic_spline import CubicSpline2D


def test_straight_line():
    """Straight line should have zero curvature everywhere."""
    x = np.linspace(0, 10, 100)
    y = np.linspace(0, 10, 100)  # 45-degree line

    spline = CubicSpline2D(x, y)

    # Sample curvatures at multiple points
    curvatures = [spline.calc_curvature(s) for s in np.linspace(0, spline.s[-1], 20)]
    max_curvature = np.max(np.abs(curvatures))

    print(f"Straight line - Max absolute curvature: {max_curvature:.6f}")

    # A straight line should have zero curvature (within numerical precision)
    assert max_curvature < 1e-3, f"Straight line should have ~zero curvature, got {max_curvature}"


def test_circle():
    """Circle with radius R should have curvature = 1/R."""
    radius = 5.0
    expected_curvature = 1.0 / radius

    # Create circle (exclude the last point to avoid duplication)
    theta = np.linspace(0, 2 * np.pi, 100)[:-1]
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)

    spline = CubicSpline2D(x, y)

    # Sample curvatures
    curvatures = [spline.calc_curvature(s) for s in np.linspace(0, spline.s[-1], 20)]
    mean_curvature = np.mean(np.abs(curvatures))
    std_curvature = np.std(curvatures)
    error = abs(mean_curvature - expected_curvature)

    print(
        f"Circle (R={radius}m) - Expected: {expected_curvature:.4f} m⁻¹, "
        f"Got: {mean_curvature:.4f} m⁻¹, Error: {error:.6f}, Std: {std_curvature:.6f}"
    )

    # Allow 5% error due to discretization and cubic spline approximation
    tolerance = 0.05 * expected_curvature
    assert error < tolerance, (
        f"Circle curvature should be {expected_curvature:.4f} ± {tolerance:.4f} m⁻¹, got {mean_curvature:.4f} m⁻¹"
    )


def test_sine_wave():
    """Test curvature on a sine wave at inflection point (known analytical formula)."""
    # y = A*sin(ω*x), curvature κ = -A*ω²*sin(ω*x) / (1 + A²*ω²*cos²(ω*x))^(3/2)
    A = 2.0  # amplitude
    omega = 2 * np.pi / 10  # wavelength = 10m

    x = np.linspace(0, 20, 200)
    y = A * np.sin(omega * x)

    spline = CubicSpline2D(x, y)

    # Check curvature at x=10 (interior inflection point, avoiding boundary artifacts)
    # At x=10: ω*x = 2π, so sin(2π)=0, cos(2π)=1, therefore κ = 0
    # Note: We test at an interior point because CubicSpline2D uses periodic boundary
    # conditions for closed racing tracks, which can create artifacts at boundaries
    # for non-closed curves like this sine wave
    x_test = 10.0
    y_test = A * np.sin(omega * x_test)
    s_test, _ = spline.calc_arclength(x_test, y_test, s_guess=spline.s[-1] / 2)
    k_test = spline.calc_curvature(s_test)

    print(f"Sine wave - Curvature at interior inflection (x={x_test}): {k_test:.6f} (expected ~0)")

    # At inflection point, curvature should be near zero
    assert abs(k_test) < 0.01, f"Sine wave at x={x_test} should have ~zero curvature, got {k_test}"


def test_lookahead_sampling():
    """Test the lookahead sampling logic (replicates sample_lookahead_curvatures)."""
    # Create a circular track
    radius = 10.0
    theta = np.linspace(0, 2 * np.pi, 100)[:-1]
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)

    spline = CubicSpline2D(x, y)

    # Manually sample curvatures using the same logic as sample_lookahead_curvatures
    current_s = 5.0
    n_points = 10
    ds = 0.3
    track_length = spline.s[-1]

    manual_curvatures = []
    for i in range(n_points):
        # This is the exact formula from sample_lookahead_curvatures (line 58)
        s_ahead = (current_s + (i + 1) * ds) % track_length
        manual_curvatures.append(spline.calc_curvature(s_ahead))

    mean_curvature = np.mean(np.abs(manual_curvatures))
    expected_curvature = 1.0 / radius

    print(
        f"Lookahead sampling - Sampled {n_points} points at ds={ds}m intervals: "
        f"Mean κ={mean_curvature:.4f} m⁻¹ (expected {expected_curvature:.4f} m⁻¹)"
    )

    # Verify lookahead sampling produces reasonable curvature values
    # Allow 10% error since we're sampling discrete points on a circle
    tolerance = 0.1 * expected_curvature
    assert abs(mean_curvature - expected_curvature) < tolerance, (
        f"Lookahead sampling should produce mean curvature ~{expected_curvature:.4f} m⁻¹, got {mean_curvature:.4f} m⁻¹"
    )


def test_curvature_visualization():
    """Generate visualization of curvature along a figure-8 track.

    This test creates a diagnostic plot showing how curvature varies along
    a lemniscate (figure-8) track, demonstrating both the track geometry
    and the corresponding curvature profile.
    """
    # Create a lemniscate curve (figure-8 shape) - clear varying curvature
    t = np.linspace(0, 2 * np.pi, 200)[:-1]
    a = 5.0
    x = a * np.cos(t) / (1 + np.sin(t) ** 2)
    y = a * np.sin(t) * np.cos(t) / (1 + np.sin(t) ** 2)

    spline = CubicSpline2D(x, y)

    # Sample curvature along entire track
    s_samples = np.linspace(0, spline.s[-1], 200)
    curvatures = [spline.calc_curvature(s) for s in s_samples]

    # Create diagnostic plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    # Top panel: Track layout in XY plane
    ax1.plot(x, y, "b-", linewidth=2, label="Track")
    ax1.set_xlabel("X [m]")
    ax1.set_ylabel("Y [m]")
    ax1.set_title("Track Layout")
    ax1.axis("equal")
    ax1.grid(True)
    ax1.legend()

    # Bottom panel: Curvature vs arc length
    ax2.plot(s_samples, curvatures, "r-", linewidth=2, label="Curvature")
    ax2.axhline(y=0, color="k", linestyle="--", alpha=0.3, label="Zero curvature")
    ax2.set_xlabel("Arc length s [m]")
    ax2.set_ylabel("Curvature κ [1/m]")
    ax2.set_title("Curvature Profile Along Track")
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    output_dir = os.path.join(os.path.dirname(__file__), "..", "figures", "tests")
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(os.path.join(output_dir, "curvature_validation.png"), dpi=300, bbox_inches="tight")
    # Verify we got reasonable curvature values
    assert len(curvatures) == len(s_samples), "Curvature samples mismatch"
    assert all(np.isfinite(curvatures)), "All curvatures must be finite"

    return plt


if __name__ == "__main__":
    """Run all curvature validation tests."""
    print("=" * 70)
    print("Curvature Validation Test Suite")
    print("=" * 70)

    # Run tests
    try:
        test_straight_line()
        print("✓ test_straight_line passed")
    except AssertionError as e:
        print(f"✗ test_straight_line FAILED: {e}")

    try:
        test_circle()
        print("✓ test_circle passed")
    except AssertionError as e:
        print(f"✗ test_circle FAILED: {e}")

    try:
        test_sine_wave()
        print("✓ test_sine_wave passed")
    except AssertionError as e:
        print(f"✗ test_sine_wave FAILED: {e}")

    try:
        test_lookahead_sampling()
        print("✓ test_lookahead_sampling passed")
    except AssertionError as e:
        print(f"✗ test_lookahead_sampling FAILED: {e}")

    try:
        plt = test_curvature_visualization()
        print("✓ test_curvature_visualization passed")
        plt.show()
    except AssertionError as e:
        print(f"✗ test_curvature_visualization FAILED: {e}")

    print("=" * 70)
