import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np

from gymkhana.envs.gymkhana_env import GKEnv
from train.config.env_config import get_drift_test_config, get_env_id


def make_env(long_act_type):
    config = get_drift_test_config()
    config["params"] = GKEnv.f1tenth_vehicle_params()
    config["map"] = "Spielberg_blank"
    config["model"] = "st"
    config["normalize_act"] = False
    config["normalize_obs"] = False
    config["debug_frenet_projection"] = False
    config["control_input"] = [long_act_type, "steering_angle"]
    config["observation_config"] = {"type": "kinematic_state"}

    env = gym.make(
        get_env_id(),
        config=config,
        render_mode="human",
    )
    obs, info = env.reset()
    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}")
    return env


def run_lag_test(long_act_type, action, obs_key, target, extra_steps):
    env = make_env(long_act_type)

    curr = 0.0
    traj = [curr]

    while curr < target:
        obs, reward, done, truncated, info = env.step(action)
        curr = float(obs["agent_0"][obs_key])
        traj.append(curr)
        env.render()

    for _ in range(extra_steps):
        obs, reward, done, truncated, info = env.step(action)
        curr = float(obs["agent_0"][obs_key])
        traj.append(curr)
        env.render()

    env.close()

    plt.plot(traj)
    plt.show()


def steer_lag():
    target = 0.4189
    run_lag_test(
        long_act_type="accl",
        action=np.array([[target, 0.0]]),
        obs_key="delta",
        target=target,
        extra_steps=20,
    )


def accl_lag():
    target = 19.0
    run_lag_test(
        long_act_type="accl",
        action=np.array([[0.0, 9.51]]),
        obs_key="linear_vel_x",
        target=target,
        extra_steps=100,
    )


def vel_lag():
    target = 19.0
    run_lag_test(
        long_act_type="speed",
        action=np.array([[0.0, target]]),
        obs_key="linear_vel_x",
        target=target,
        extra_steps=100,
    )


steer_lag()

# accl_lag()

# vel_lag()
