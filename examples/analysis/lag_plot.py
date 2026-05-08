import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np

from gymkhana.envs.gymkhana_env import GKEnv
from train.config.env_config import get_drift_test_config, get_env_id


def make_env(long_act_type, t_steer: float = 0.025):
    config = get_drift_test_config()
    params = GKEnv.f1tenth_vehicle_params()
    # f1tenth_st.yaml has T_steer commented out (defaults to 0 → bang-bang),
    # so inject it explicitly to exercise the first-order lag path.
    params["T_steer"] = t_steer
    config["params"] = params
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


def run_lag_test(env, action, obs_key, target, extra_steps):
    env.reset()

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

    return np.asarray(traj)


def steer_lag(env):
    target = 0.4189
    return run_lag_test(
        env=env,
        action=np.array([[target, 0.0]]),
        obs_key="delta",
        target=target,
        extra_steps=20,
    )


def accl_lag(env):
    target = 19.0
    return run_lag_test(
        env=env,
        action=np.array([[0.0, 9.51]]),
        obs_key="linear_vel_x",
        target=target,
        extra_steps=100,
    )


def vel_lag(env):
    target = 19.0
    return run_lag_test(
        env=env,
        action=np.array([[0.0, target]]),
        obs_key="linear_vel_x",
        target=target,
        extra_steps=100,
    )


def t_steer_bug_demo():
    """Reuse one env across a configure() call that changes T_steer.

    With the bug present (cached self.k in SteeringAngleAction), both runs
    produce identical trajectories. After the fix, the second run should
    rise noticeably slower (larger T_steer → slower first-order response).
    """
    t_steer_initial = 0.025
    t_steer_changed = 0.2

    env = make_env("accl", t_steer=t_steer_initial)

    traj1 = steer_lag(env)
    env.unwrapped.configure(config={"params": {"T_steer": t_steer_changed}})
    traj2 = steer_lag(env)

    plt.figure()
    plt.plot(traj1, label=f"T_steer={t_steer_initial}")
    plt.plot(traj2, label=f"T_steer={t_steer_changed} (after configure)")
    plt.axhline(0.4189, color="k", linestyle="--", linewidth=0.5, label="target")
    plt.xlabel("step")
    plt.ylabel("delta [rad]")
    plt.title("T_steer bug: identical curves = bug present, divergent = fixed")
    plt.legend()
    plt.show()

    return env


t_steer_bug_demo()
