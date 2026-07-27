import os

import gymnasium as gym
import numpy as np
import wandb
from stable_baselines3 import PPO
from wandb.integration.sb3 import WandbCallback

from train.config.env_config import PROJECT_NAME, SEED
from train.train_utils import get_ckpt_callback, get_output_dirs, make_output_dirs

# toggle this to train or evaluate
train = False
map = "Spielberg"


def main():
    if train:
        proj_root, output_root = get_output_dirs()

        run = wandb.init(project=PROJECT_NAME, sync_tensorboard=True, monitor_gym=True, dir=proj_root, save_code=True)
        run_id = run.id
        tensorboard_dir, models_dir, _ = make_output_dirs(run.id, output_root)

        env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": map,
                "num_agents": 1,
                "timestep": 0.01,
                "num_beams": 36,
                "integrator": "rk4",
                "control_input": ["speed", "steering_angle"],
                "observation_config": {"type": "rl"},
                "reset_config": {"type": "rl_random_static"},
                "normalize_act": False,
                "normalize_obs": False,
                "predictive_collision": True,
                "wall_deflection": True,
            },
        )

        model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=tensorboard_dir, device="auto", seed=SEED)
        model.learn(
            total_timesteps=1_000_000,
            callback=[
                WandbCallback(gradient_save_freq=0, model_save_path=models_dir, verbose=2),
                get_ckpt_callback(models_dir=models_dir, save_freq=200000),
            ],
            progress_bar=True,
        )

        final_model_path = f"{models_dir}/ppo_drift_final_{run_id}"
        model.save(final_model_path)

        # If debugging observation normalization, explicitly close the environment to
        # trigger observation statistics printing -> env is wrapped in DummyVecEnv, so we need to close the underlying env
        # if debug_obs_norm:
        #     if hasattr(env, "envs"):
        #         # VecEnv wrapper - close the underlying environments
        #         for underlying_env in env.envs:
        #             underlying_env.close()
        #     else:
        #         # Regular env
        #         env.close()
        env.close()
        run.finish()

    else:
        proj_root, _ = get_output_dirs()
        run_id = "q81h6jga"  # replace with your run ID
        model_path = os.path.join(proj_root, "outputs", "models", "5ybfzkyr", "checkpoints", "ckpt_1000000_steps.zip")
        model = PPO.load(model_path, print_system_info=True, device="cpu")
        eval_env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": map,
                "num_agents": 1,
                "timestep": 0.01,
                "num_beams": 36,
                "integrator": "rk4",
                "control_input": ["speed", "steering_angle"],
                "observation_config": {"type": "rl"},
                "reset_config": {"type": "rl_random_static"},
                "normalize_act": False,
                "normalize_obs": False,
                "predictive_collision": True,
                "wall_deflection": True,
                "debug_frenet_projection": True,
                "render_track_lines": True,
            },
            render_mode="human",
        )
        np.random.seed()
        obs, info = eval_env.reset()
        done = False
        reward = 0.0
        total_reward = 0.0
        i = 0
        while not done:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, trunc, info = eval_env.step(action)
            total_reward += reward

            print(f"\nStep: {i}")
            print(f"Reward: {reward}")
            print(f"Total Reward: {total_reward}")

            eval_env.render()

            i += 1

            # VecEnv resets automatically
            # if done:
            #   obs = env.reset()
        eval_env.close()


if __name__ == "__main__":
    main()
