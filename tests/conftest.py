"""
Shared pytest fixtures for the test suite.

Session-scoped fixtures are created once per test run. Tests using these
MUST NOT mutate the env in ways that leak to other tests.
"""

import gymnasium as gym
import pytest

from gymkhana.envs import GKEnv


@pytest.fixture(scope="session")
def spielberg_base_env():
    """Shared base Spielberg env (no observation, rl_random_static reset).

    Used by tests that only need an unwrapped env to mock internal state
    (e.g. _check_done, _get_reward). Do NOT mutate in ways that affect other tests.
    """
    env = gym.make(
        "gymkhana:gymkhana-v0",
        config={
            "map": "Spielberg",
            "num_agents": 1,
            "observation_config": {"type": None},
            "reset_config": {"type": "rl_random_static"},
            "predictive_collision": True,
        },
    )
    env.reset()
    yield env
    env.close()


@pytest.fixture(scope="session")
def spielberg_std_env():
    """Shared Spielberg STD model env with drift observations and normalization."""
    env = gym.make(
        "gymkhana:gymkhana-v0",
        config={
            "map": "Spielberg",
            "num_agents": 1,
            "model": "std",
            "observation_config": {"type": "drift"},
            "params": GKEnv.f1tenth_std_vehicle_params(),
            "normalize_obs": True,
            "normalize_act": True,
            "control_input": ["accl", "steering_angle"],
        },
    )
    env.reset()
    yield env
    env.close()


@pytest.fixture(scope="session")
def spielberg_std_env_unnormalized():
    """Shared Spielberg STD model env with drift observations, no normalization."""
    env = gym.make(
        "gymkhana:gymkhana-v0",
        config={
            "map": "Spielberg",
            "num_agents": 1,
            "model": "std",
            "observation_config": {"type": "drift"},
            "params": GKEnv.f1tenth_std_vehicle_params(),
            "normalize_obs": False,
            "normalize_act": False,
            "control_input": ["accl", "steering_angle"],
        },
    )
    env.reset()
    yield env
    env.close()
