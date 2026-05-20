"""Tests for the configurable instability-prevention feature.

Covers parity-when-off, trigger-and-truncate-when-on, the revert-on-blow-up
safety, and the SubprocVecEnv aggregator's self-gating.
"""

import gymnasium as gym
import numpy as np
import pytest

from gymkhana.envs.gymkhana_env import GKEnv
from train.train_utils import aggregate_and_print_instability_count


def _make_env(prevent_instability: bool, yaw_rate_bound: float | None = None, slip_bound: float | None = None):
    cfg = {
        "map": "Spielberg",
        "num_agents": 1,
        "num_beams": 2,  # cut scan-simulator setup time
        "model": "st",
        "params": GKEnv.f1tenth_vehicle_params(),
        "observation_config": {"type": "original"},
        "normalize_obs": False,
        "prevent_instability": prevent_instability,
    }
    if yaw_rate_bound is not None:
        cfg["instability_yaw_rate_bound"] = yaw_rate_bound
    if slip_bound is not None:
        cfg["instability_slip_bound"] = slip_bound
    return gym.make("gymkhana:gymkhana-v0", config=cfg)


def _hard_steer_action(env):
    """Return an action that produces a non-trivial yaw rate (full-lock steer + speed)."""
    space = env.action_space
    return np.tile(np.array([space.high[0, 0], space.high[0, 1]], dtype=np.float32), (1, 1))


def test_prevent_instability_off_is_noop():
    """prevent_instability=False ⇒ check never runs, no truncation, count stays 0."""
    env = _make_env(prevent_instability=False)
    try:
        unwrapped = env.unwrapped
        assert unwrapped.prevent_instability is False
        # Per-RaceCar: flag off ⇒ check is skipped in update_pose regardless of bounds.
        for agent in unwrapped.sim.agents:
            assert agent.prevent_instability is False

        env.reset(seed=0)
        action = _hard_steer_action(env)
        for _ in range(20):
            _, _, terminated, truncated, info = env.step(action)
            assert info["instability_truncation"] is False
            if terminated or truncated:
                env.reset()

        assert unwrapped._instability_count == 0
        assert all(a.unstable is False for a in unwrapped.sim.agents)
    finally:
        env.close()


def test_prevent_instability_triggers_and_reverts():
    """Tight bounds flow through to RaceCar, trip truncation, and revert agent.state.

    Combines the trigger-path and revert-path checks in one env to halve setup cost.
    """
    env = _make_env(prevent_instability=True, yaw_rate_bound=1e-9, slip_bound=1e-9)
    try:
        unwrapped = env.unwrapped
        assert unwrapped.prevent_instability is True
        assert unwrapped.instability_bounds == {"yaw_rate": 1e-9, "slip": 1e-9}
        # Bound dict reaches the RaceCar.
        assert unwrapped.sim.agents[0].instability_bounds == {"yaw_rate": 1e-9, "slip": 1e-9}

        env.reset(seed=0)
        agent = unwrapped.sim.agents[0]
        action = _hard_steer_action(env)

        for _ in range(20):
            pre_state = agent.state.copy()
            _, _, _, truncated, info = env.step(action)
            if info["instability_truncation"]:
                # Trigger-path assertions
                assert truncated is True
                assert info["unstable_agents"] == [0]
                assert "violations" in info["unstable_info"][0]
                assert unwrapped._instability_count >= 1
                # Revert-path assertion: _check_state_sanity restored prev_state.
                np.testing.assert_array_equal(agent.state, pre_state)
                return

        pytest.fail("Expected instability truncation with bound=1e-9 within 20 steps")
    finally:
        env.close()


class _FakeVecEnv:
    """Minimal SubprocVecEnv stand-in — only get_attr is used by the aggregator."""

    def __init__(self, per_env: list[dict]):
        self._envs = per_env

    def get_attr(self, name, indices=None):
        envs = self._envs if indices is None else [self._envs[i] for i in indices]
        return [e[name] for e in envs]


def test_aggregate_print_self_gates_when_off(capsys):
    """All envs have prevent_instability=False ⇒ aggregator prints nothing."""
    vec = _FakeVecEnv([{"prevent_instability": False, "_instability_count": 0} for _ in range(3)])
    aggregate_and_print_instability_count(vec)
    assert capsys.readouterr().out == ""


def test_aggregate_print_sums_and_breaks_down(capsys):
    """Mixed counts: total is summed, only non-zero envs appear in the breakdown."""
    vec = _FakeVecEnv(
        [
            {"prevent_instability": True, "_instability_count": 3},
            {"prevent_instability": True, "_instability_count": 0},
            {"prevent_instability": True, "_instability_count": 1},
        ]
    )
    aggregate_and_print_instability_count(vec)
    out = capsys.readouterr().out
    assert "4 total across 3 envs" in out
    assert "env 0: 3" in out
    assert "env 2: 1" in out
    assert "env 1:" not in out  # zero-count envs suppressed
