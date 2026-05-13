"""
Unit tests for _get_reward() function in GKEnv.

These tests expose the wraparound bug where completing a lap
gives a large negative reward instead of a small positive reward.
"""

import gymnasium as gym
import numpy as np
import pytest


class TestGetRewardWraparound:
    """Test suite for reward calculation wraparound handling"""

    @pytest.fixture(scope="class")
    def env(self):
        """Shared env for reward wraparound tests. Each test fully overwrites state."""
        env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "timestep": 0.01,
                "progress_gain": 1.0,
                "predictive_collision": True,
            },
            render_mode=None,
        )
        env.reset()
        yield env.unwrapped
        env.close()

    def test_normal_forward_progress(self, env):
        """Test that normal forward progress gives positive reward"""
        track_length = env.track.centerline.spline.s[-1]

        # Set up normal forward motion scenario
        last_s = 50.0
        env.last_s = [last_s]
        env.poses_x = [env.track.centerline.xs[0]]
        env.poses_y = [env.track.centerline.ys[0]]
        env.collisions = np.zeros(1)

        # 0.1m forward progress from last_s=50
        env._frenet_cache[0, 0] = 50.1

        reward = env._get_reward()

        print("\nNormal forward progress test:")
        print(f"  Track length: {track_length:.2f}m")
        print(f"  Last s: {last_s:.2f}m")
        print("  Current s: 50.1m")
        print("  Expected reward: ~0.1m")
        print(f"  Actual reward: {reward:.2f}m")

        assert np.isclose(reward, 0.1), f"Forward progress should give ~0.1m reward, got {reward}"

    def test_backward_wraparound_lap_completion(self, env):
        """
        Test that completing a lap gives positive reward (capped by max_progress).

        Scenario:
        - Agent at s=338m (5m before finish on ~343m track)
        - Moves to s=5m (crossed finish line)
        - Raw progress: 5 - 338 = -333m (large negative due to wraparound)
        - Wraparound correction: -333 + 343 = 10m ✓
        - Clipping with margin=10: np.clip(10, -2, 2) = 2m (max_progress)

        Note: The actual progress is 10m, but it gets clipped to max_progress (2m)
        because margin=10 limits progress to v_max * timestep * 10 = 20 * 0.01 * 10 = 2m.
        This is expected behavior - the clipping acts as a safety net.
        """
        track_length = env.track.centerline.spline.s[-1]

        # Set up lap completion scenario
        # Agent is 5m before finish line
        last_s = track_length - 5.0
        env.last_s = [last_s]
        env.poses_x = [env.track.centerline.xs[0]]
        env.poses_y = [env.track.centerline.ys[0]]
        env.collisions = np.zeros(1)

        # 5m after start (crossed finish)
        env._frenet_cache[0, 0] = 5.0

        reward = env._get_reward()

        # Calculate expected reward (clipped to max_progress)
        max_progress = env.params["v_max"] * env.timestep * 10.0  # margin=10
        expected_reward = max_progress  # 2.0m

        print("\n✓ Lap completion with clipping:")
        print(f"  Track length: {track_length:.2f}m")
        print(f"  Last s: {last_s:.2f}m (near finish)")
        print("  Current s: 5.00m (crossed finish)")
        print(f"  Raw progress: {5.0 - last_s:.2f}m")
        print("  Corrected progress (before clip): ~10.00m")
        print(f"  Max progress (margin=10): {max_progress:.2f}m")
        print(f"  Expected reward (after clip): {expected_reward:.2f}m")
        print(f"  Actual reward: {reward:.2f}m")

        # Verify wraparound was corrected and reward is positive (not large negative)
        assert reward > 0, (
            f"Lap completion should give positive reward! "
            f"Got {reward:.2f}m. This indicates wraparound correction failed."
        )

        # Verify reward matches clipped max_progress
        assert np.isclose(reward, expected_reward, atol=0.01), (
            f"Expected clipped reward of {expected_reward:.2f}m, got {reward:.2f}m"
        )

    def test_backward_wraparound_mid_track_to_start(self, env):
        """
        Test wraparound from middle of track back to start (capped by max_progress).

        Scenario:
        - Agent at s=333m (10m before finish on ~343m track)
        - Moves to s=10m (crossed finish line)
        - Raw progress: 10 - 333 = -323m (large negative due to wraparound)
        - Wraparound correction: -323 + 343 = 20m ✓
        - Clipping with margin=10: np.clip(20, -2, 2) = 2m (max_progress)

        Note: Similar to lap completion test, the 20m progress gets clipped to 2m.
        """
        track_length = env.track.centerline.spline.s[-1]

        # Set up scenario: from 10m before finish to 10m after start
        last_s = track_length - 10.0
        env.last_s = [last_s]
        env.collisions = np.zeros(1)

        # 10m from start (after wraparound)
        env._frenet_cache[0, 0] = 10.0

        reward = env._get_reward()

        # Calculate expected reward (clipped to max_progress)
        max_progress = env.params["v_max"] * env.timestep * 10.0  # margin=10
        expected_reward = max_progress  # 2.0m (not 20m due to clipping)

        print("\n✓ Backward wraparound with clipping (10m before finish to 10m after):")
        print(f"  Track length: {track_length:.2f}m")
        print(f"  Last s: {last_s:.2f}m")
        print("  Current s: 10.00m")
        print("  Corrected progress (before clip): ~20.00m")
        print(f"  Max progress (margin=10): {max_progress:.2f}m")
        print(f"  Expected reward (after clip): {expected_reward:.2f}m")
        print(f"  Actual reward: {reward:.2f}m")

        # Verify reward is positive (wraparound corrected)
        assert reward > 0, f"Wraparound should give positive reward, got {reward}"

        # Verify reward matches clipped max_progress
        assert np.isclose(reward, expected_reward, atol=0.01), (
            f"Expected clipped reward of {expected_reward:.2f}m, got {reward:.2f}m"
        )

    def test_no_wraparound_(self, env):
        """Test that movement near finish line (without crossing) works correctly"""
        track_length = env.track.centerline.spline.s[-1]

        # Both positions before finish line
        last_s = track_length - 10.0
        env.last_s = [last_s]
        env.collisions = np.zeros(1)

        # Move forward 5m, still before finish
        env._frenet_cache[0, 0] = track_length - 9.9

        reward = env._get_reward()

        print("\nNo wraparound (near finish):")
        print(f"  Last s: {last_s:.2f}m")
        print(f"  Current s: {track_length - 9.9:.2f}m")
        print("  Expected reward: ~0.1m")
        print(f"  Actual reward: {reward:.2f}m")

        assert np.isclose(reward, 0.1), f"Expected reward 0.1, got {reward}"

    def test_collision_penalty(self, env):
        """Test that collision penalty is applied correctly"""
        # Set up forward progress with collision
        last_s = 50.0
        env.last_s = [last_s]
        env.collisions = np.array([1])  # Collision detected

        # 0.1m forward progress
        env._frenet_cache[0, 0] = 50.1

        reward = env._get_reward()

        expected_reward = 50.1 - last_s - 50.0  # progress - collision penalty

        print("\nCollision penalty test:")
        print(f"  Last s: {last_s:.2f}m")
        print("  Progress: 5.00m")
        print("  Collision penalty: 1.00")
        print(f"  Expected reward: {expected_reward:.2f}m")
        print(f"  Actual reward: {reward:.2f}m")

        assert np.isclose(reward, expected_reward), f"Expected reward {expected_reward}, got {reward}"

    def test_multi_agent_collaborative(self):
        """Test that multi-agent rewards are summed correctly"""
        # Create 2-agent environment (separate from shared single-agent env)
        wrapped_env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 2,
                "progress_gain": 1.0,
            },
            render_mode=None,
        )
        wrapped_env.reset()
        env = wrapped_env.unwrapped

        # Set up both agents with forward progress
        last_s_agent0 = 50.0
        last_s_agent1 = 100.0
        env.last_s = [last_s_agent0, last_s_agent1]
        env.collisions = np.zeros(2)

        # Both agents moved forward 0.1m
        env._frenet_cache[0, 0] = 50.1
        env._frenet_cache[1, 0] = 100.1

        reward = env._get_reward()

        expected_reward = 0.2  # Both agents' progress

        print("\nMulti-agent collaborative reward:")
        print(f"  Expected total: {expected_reward:.2f}m")
        print(f"  Actual reward: {reward:.2f}m")

        assert np.isclose(reward, expected_reward), f"Expected reward {expected_reward}, got {reward}"

        wrapped_env.close()


class TestCorrectWraparoundProg:
    """Test suite for _correct_wraparound_prog() method"""

    @pytest.fixture(scope="class")
    def env(self):
        """Shared env for wraparound prog tests. Tests only call _correct_wraparound_prog (read-only)."""
        env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "timestep": 0.01,
            },
            render_mode=None,
        )
        env.reset()
        yield env.unwrapped
        env.close()

    def test_forward_wraparound_correction(self, env):
        """
        Test forward wraparound: car crosses finish line going forward.

        Scenario: last_s=99.9m, current_s=0.1m on 100m track
        Raw progress: 0.1 - 99.9 = -99.8m (large negative)
        Should correct to: -99.8 + 100 = 0.2m (actual forward progress)
        """
        track_length = env.track.centerline.spline.s[-1]

        # Simulate crossing finish line forward
        last_s = track_length - 0.1  # 0.1m before finish
        current_s = 0.1  # 0.1m after start
        prog = current_s - last_s  # Large negative value

        corrected = env._correct_wraparound_prog(prog=prog, track_length=track_length)
        expected = 0.2  # Actual distance traveled

        print("\n✓ Forward wraparound correction test:")
        print(f"  Track length: {track_length:.2f}m")
        print(f"  Last s: {last_s:.2f}m (near finish)")
        print(f"  Current s: {current_s:.2f}m (crossed to start)")
        print(f"  Raw progress: {prog:.2f}m")
        print(f"  Corrected progress: {corrected:.2f}m")
        print(f"  Expected: {expected:.2f}m")

        assert np.isclose(corrected, expected, atol=0.01), (
            f"Forward wraparound correction failed: expected {expected:.2f}m, got {corrected:.2f}m"
        )

    def test_backward_wraparound_correction(self, env):
        """
        Test backward wraparound: car crosses finish line going backward.

        Scenario: last_s=0.1m, current_s=99.9m on 100m track
        Raw progress: 99.9 - 0.1 = 99.8m (large positive)
        Should correct to: 99.8 - 100 = -0.2m (actual backward progress)
        """
        track_length = env.track.centerline.spline.s[-1]

        # Simulate crossing finish line backward
        last_s = 0.1  # 0.1m after start
        current_s = track_length - 0.1  # 0.1m before finish (reversed across line)
        prog = current_s - last_s  # Large positive value

        corrected = env._correct_wraparound_prog(prog=prog, track_length=track_length)
        expected = -0.2  # Actual distance traveled backward

        print("\n✓ Backward wraparound correction test:")
        print(f"  Track length: {track_length:.2f}m")
        print(f"  Last s: {last_s:.2f}m (just after start)")
        print(f"  Current s: {current_s:.2f}m (reversed to near finish)")
        print(f"  Raw progress: {prog:.2f}m")
        print(f"  Corrected progress: {corrected:.2f}m")
        print(f"  Expected: {expected:.2f}m")

        assert np.isclose(corrected, expected, atol=0.01), (
            f"Backward wraparound correction failed: expected {expected:.2f}m, got {corrected:.2f}m"
        )

    def test_normal_forward_progress_no_correction(self, env):
        """
        Test that normal forward progress is not modified.

        Progress within physics bounds should pass through unchanged.
        """
        track_length = env.track.centerline.spline.s[-1]

        # Normal forward progress: 0.15m at v=15m/s with dt=0.01s
        prog_normal = 0.15
        corrected = env._correct_wraparound_prog(prog=prog_normal, track_length=track_length)

        print("\n✓ Normal forward progress (no correction):")
        print(f"  Raw progress: {prog_normal:.2f}m")
        print(f"  Corrected progress: {corrected:.2f}m")
        print(f"  Max allowed: {env.params['v_max'] * env.timestep * 1.05:.4f}m")

        assert corrected == prog_normal, (
            f"Normal progress should not be modified: expected {prog_normal}m, got {corrected}m"
        )

    def test_normal_backward_progress_no_correction(self, env):
        """
        Test that normal backward progress is not modified.

        Small backward motion within physics bounds should pass through unchanged.
        """
        track_length = env.track.centerline.spline.s[-1]

        # Normal backward progress: -0.04m at v=-4m/s with dt=0.01s
        prog_backward = -0.04
        corrected = env._correct_wraparound_prog(prog=prog_backward, track_length=track_length)

        print("\n✓ Normal backward progress (no correction):")
        print(f"  Raw progress: {prog_backward:.2f}m")
        print(f"  Corrected progress: {corrected:.2f}m")
        print(f"  Min allowed: {-abs(env.params['v_min']) * env.timestep * 1.05:.4f}m")

        assert corrected == prog_backward, (
            f"Normal backward progress should not be modified: expected {prog_backward}m, got {corrected}m"
        )

    def test_zero_progress_no_correction(self, env):
        """Test that zero progress (stationary vehicle) is not modified."""
        track_length = env.track.centerline.spline.s[-1]

        prog_zero = 0.0
        corrected = env._correct_wraparound_prog(prog=prog_zero, track_length=track_length)

        print("\n✓ Zero progress (stationary):")
        print(f"  Raw progress: {prog_zero:.2f}m")
        print(f"  Corrected progress: {corrected:.2f}m")

        assert corrected == prog_zero, "Zero progress should not be modified"

    def test_boundary_threshold_forward(self, env):
        """
        Test progress exactly at the forward wraparound threshold.

        Progress just below the threshold should NOT trigger correction.
        """
        track_length = env.track.centerline.spline.s[-1]

        # Exactly at threshold (should not trigger correction)
        max_backward = abs(env.params["v_min"]) * env.timestep * 1.05
        prog_at_threshold = -max_backward

        corrected = env._correct_wraparound_prog(prog=prog_at_threshold, track_length=track_length)

        print("\n✓ Boundary test (at forward threshold):")
        print(f"  Threshold: {-max_backward:.4f}m")
        print(f"  Progress: {prog_at_threshold:.4f}m")
        print(f"  Corrected: {corrected:.4f}m")

        # At exact threshold, should NOT trigger wraparound (< not <=)
        assert corrected == prog_at_threshold, "Progress at threshold should not be corrected"

    def test_boundary_threshold_backward(self, env):
        """
        Test progress exactly at the backward wraparound threshold.

        Progress just below the threshold should NOT trigger correction.
        """
        track_length = env.track.centerline.spline.s[-1]

        # Exactly at threshold (should not trigger correction)
        max_forward = env.params["v_max"] * env.timestep * 1.05
        prog_at_threshold = max_forward

        corrected = env._correct_wraparound_prog(prog=prog_at_threshold, track_length=track_length)

        print("\n✓ Boundary test (at backward threshold):")
        print(f"  Threshold: {max_forward:.4f}m")
        print(f"  Progress: {prog_at_threshold:.4f}m")
        print(f"  Corrected: {corrected:.4f}m")

        # At exact threshold, should NOT trigger wraparound (> not >=)
        assert corrected == prog_at_threshold, "Progress at threshold should not be corrected"


class TestGetRewardEdgeCases:
    """Test edge cases in reward calculation"""

    @pytest.fixture(scope="class")
    def env(self):
        """Shared env for edge case tests. Each test fully overwrites state."""
        env = gym.make(
            "gymkhana:gymkhana-v0",
            config={"map": "Spielberg", "num_agents": 1, "progress_gain": 1.0},
            render_mode=None,
        )
        env.reset()
        yield env.unwrapped
        env.close()

    def test_zero_progress(self, env):
        """Test that standing still gives zero reward"""
        last_s = 50.0
        env.last_s = [last_s]
        env.collisions = np.zeros(1)

        # No movement
        env._frenet_cache[0, 0] = 50.0

        reward = env._get_reward()

        print("\nZero progress test:")
        print(f"  Last s: {last_s:.2f}m")
        print("  Current s: 50.00m")
        print("  Expected reward: 0.00m")
        print(f"  Actual reward: {reward:.2f}m")

        assert np.isclose(reward, 0.0), f"Zero progress should give 0 reward, got {reward}"

    def test_backward_motion_no_wraparound(self, env):
        """Test that moving backward (without wraparound) gives negative reward"""
        last_s = 100.0
        env.last_s = [last_s]
        env.collisions = np.zeros(1)

        # Moved backward 0.1m
        env._frenet_cache[0, 0] = 99.9

        reward = env._get_reward()

        expected_reward = -0.1

        print("\nBackward motion (no wraparound):")
        print(f"  Last s: {last_s:.2f}m")
        print(f"  Expected reward: {expected_reward:.2f}m")
        print(f"  Actual reward: {reward:.2f}m")

        assert np.isclose(reward, expected_reward), f"Expected reward {expected_reward}, got {reward}"


class TestRewardCalculation:
    """Test suite validating core reward calculation components in Frenet mode"""

    @pytest.fixture(scope="class")
    def env(self):
        """Shared Frenet mode env. Each test fully overwrites state before calling _get_reward."""
        env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "timestep": 0.01,
                "progress_gain": 1.0,
                "predictive_collision": False,
                "out_of_bounds_penalty": -1.0,
                "negative_vel_penalty": -0.5,
            },
            render_mode=None,
        )
        env.reset()
        yield env.unwrapped
        env.close()

    def test_out_of_bounds_penalty_applied_frenet_boundary(self, env):
        """Test that out_of_bounds_penalty is applied when boundary_exceeded is True"""
        # Set up Frenet mode with boundary exceeded
        last_s = 50.0
        env.last_s = [last_s]
        env.boundary_exceeded = np.array([True])  # Boundary violation
        env.poses_x = [env.track.centerline.xs[0]]
        env.poses_y = [env.track.centerline.ys[0]]

        # Same location (zero progress)
        env._frenet_cache[0, 0] = 50.0

        # Set agent velocity (state[3] = v_x for ST model)
        env.sim.agents[0].state[3] = 5.0

        reward = env._get_reward()

        print("\n✓ Out-of-bounds penalty (Frenet mode):")
        print("  Boundary exceeded: True")
        print("  Progress: 0.0m")
        print(f"  Out-of-bounds penalty: {env.out_of_bounds_penalty}")
        print(f"  Expected reward: {env.out_of_bounds_penalty}")
        print(f"  Actual reward: {reward:.2f}")

        # In Frenet mode with boundary_exceeded=True, only penalty is applied
        assert np.isclose(reward, env.out_of_bounds_penalty), (
            f"Expected out-of-bounds penalty {env.out_of_bounds_penalty}, got {reward}"
        )

    def test_progress_gain_multiplier_application(self, env):
        """Test that forward progress is multiplied by progress_gain before reward calculation"""
        original_progress_gain = env.progress_gain
        try:
            env.progress_gain = 2.0  # Custom multiplier
            last_s = 50.0
            env.last_s = [last_s]
            env.boundary_exceeded = np.array([False])  # No boundary violation
            env.poses_x = [env.track.centerline.xs[0]]
            env.poses_y = [env.track.centerline.ys[0]]

            # 1.0m ahead
            env._frenet_cache[0, 0] = 51.0

            # Set agent velocity (state[3] = v_x for ST model)
            env.sim.agents[0].state[3] = 5.0

            reward = env._get_reward()

            # Expected: progress (1.0m) * progress_gain (2.0) = 2.0
            expected_reward = 1.0 * env.progress_gain

            print("\n✓ Progress gain multiplier:")
            print("  Raw progress: 1.0m")
            print(f"  Progress gain: {env.progress_gain}")
            print(f"  Expected reward: {expected_reward:.2f}")
            print(f"  Actual reward: {reward:.2f}")

            assert np.isclose(reward, expected_reward, atol=0.01), (
                f"Expected progress gain reward {expected_reward:.2f}, got {reward:.2f}"
            )
        finally:
            env.progress_gain = original_progress_gain

    def test_negative_velocity_penalty_application(self, env):
        """Test that negative_vel_penalty is applied when longitudinal velocity is negative"""
        last_s = 50.0
        env.last_s = [last_s]
        env.boundary_exceeded = np.array([False])  # No boundary violation
        env.poses_x = [env.track.centerline.xs[0]]
        env.poses_y = [env.track.centerline.ys[0]]

        # 0.1m ahead (small forward progress)
        env._frenet_cache[0, 0] = 50.1

        # Set agent with NEGATIVE velocity (state[3] = v_x for ST model)
        env.sim.agents[0].state[3] = -2.0

        reward = env._get_reward()

        # Expected: progress (0.1m) * progress_gain (1.0) + negative_vel_penalty (-0.5)
        expected_reward = (0.1 * 1.0) + (-0.5)

        print("\n✓ Negative velocity penalty:")
        print("  Agent velocity: -2.0 m/s")
        print("  Raw progress: 0.1m")
        print(f"  Negative velocity penalty: {env.negative_vel_penalty}")
        print(f"  Expected reward: {expected_reward:.2f}")
        print(f"  Actual reward: {reward:.2f}")

        assert np.isclose(reward, expected_reward, atol=0.01), (
            f"Expected reward with velocity penalty {expected_reward:.2f}, got {reward:.2f}"
        )


if __name__ == "__main__":
    # Run with pytest
    pytest.main([__file__, "-v", "-s"])
