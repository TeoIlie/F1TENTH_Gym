"""
Orchestration methods for model training, downloading, and evaluation
"""

import argparse
import os
from dataclasses import dataclass
from functools import partial

import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.policies import BasePolicy
from wandb.integration.sb3 import WandbCallback

import wandb
from train.callbacks import (
    make_curriculum_callback,
    make_instability_callback,
    make_log_std_schedule_callback,
    make_obs_min_max_callback,
)
from train.config.env_config import (
    ACTOR_LAYER_SIZE,
    ADDITIONAL_TIMESTEPS,
    BEST_MODEL,
    CKPT_SAVE_FREQ,
    CRITIC_LAYER_SIZE,
    END_LEARNING_RATE,
    EVAL_SEED,
    LOG_STD_END,
    LOG_STD_INIT,
    N_ENVS,
    N_STEPS,
    SEED,
    START_LEARNING_RATE,
    TOTAL_TIMESTEPS,
    TRANSFER_RESET_CRITIC,
    TRANSFER_RESET_LOG_STD,
    USE_CUSTOM_RELU,
    get_curriculum_config,
    get_env_id,
)
from train.train_utils import (
    CustomLeakyReLU,
    aggregate_and_print_instability_count,
    aggregate_and_print_obs_min_max,
    download_model_from_wandb,
    extract_norm_bounds,
    extract_rl_config,
    generate_run_id,
    get_ckpt_callback,
    get_eval_callback,
    get_output_dirs,
    linear_schedule,
    log_best_eval_timestep,
    make_eval_env,
    make_output_dirs,
    make_subprocvecenv,
    print_header,
    save_config,
    save_full_gym_config,
)


@dataclass
class TrainingProfile:
    project_name: str  # wandb project name
    track_pool: list[str] | None  # track pool for multi-map training
    train_config: dict  # from get_*_train_config()
    test_config: dict  # from get_*_test_config()
    display_name: str  # "PPO Race" or "PPO Recover" (for print headers)
    model_prefix: str  # "ppo_race" or "ppo_recover" (for save paths)


GYM_YAML = "gym_config.yaml"
GYM_OVERRIDES_YAML = "gym_overrides_config.yaml"
CURRICULUM_YAML = "curriculum_config.yaml"
RL_YAML = "rl_config.yaml"
TRANSFER_YAML = "transfer_config.yaml"
NORM_BOUNDS_YAML = "norm_bounds.yaml"
OBS_MIN_MAX_YAML = "obs_min_max.yaml"


def train(profile: TrainingProfile):
    print_header(profile.display_name + " Training")

    proj_root, output_root = get_output_dirs()

    run_id = generate_run_id()
    run = wandb.init(
        project=profile.project_name,
        id=run_id,
        name=run_id,
        sync_tensorboard=True,
        monitor_gym=True,
        dir=proj_root,
        save_code=True,
    )

    tensorboard_dir, models_dir, config_dir = make_output_dirs(run.id, output_root)
    save_config(profile.train_config, config_dir, GYM_OVERRIDES_YAML)
    save_full_gym_config(profile.train_config, config_dir, GYM_YAML)

    env = make_subprocvecenv(SEED, profile.train_config, N_ENVS, profile.track_pool)
    eval_env = make_eval_env(EVAL_SEED, profile.test_config)

    learning_rate = linear_schedule(START_LEARNING_RATE, END_LEARNING_RATE)

    policy_kwargs = dict(
        net_arch=dict(pi=[ACTOR_LAYER_SIZE, ACTOR_LAYER_SIZE], vf=[CRITIC_LAYER_SIZE, CRITIC_LAYER_SIZE]),
        log_std_init=LOG_STD_INIT,
    )
    if USE_CUSTOM_RELU:
        policy_kwargs["activation_fn"] = CustomLeakyReLU

    model = PPO(
        policy="MlpPolicy",
        env=env,
        n_steps=N_STEPS,
        verbose=1,
        tensorboard_log=tensorboard_dir,
        device="auto",
        seed=SEED,
        learning_rate=learning_rate,
        policy_kwargs=policy_kwargs,
    )

    rl_config = extract_rl_config(model, TOTAL_TIMESTEPS, N_ENVS)
    save_config(rl_config, config_dir, RL_YAML)

    curriculum_config = get_curriculum_config()
    save_config(curriculum_config, config_dir, CURRICULUM_YAML)

    norm_bounds = extract_norm_bounds(eval_env)
    if norm_bounds is not None:
        save_config(norm_bounds, config_dir, NORM_BOUNDS_YAML)

    callbacks = [
        WandbCallback(gradient_save_freq=0, verbose=2),
        get_ckpt_callback(models_dir=models_dir, save_freq=CKPT_SAVE_FREQ),
        get_eval_callback(eval_env=eval_env, models_dir=models_dir),
    ]
    curriculum_cb = make_curriculum_callback(
        curriculum_config, training_mode=profile.train_config.get("training_mode", "")
    )
    if curriculum_cb is not None:
        callbacks.append(curriculum_cb)
    obs_min_max_cb = make_obs_min_max_callback(profile.train_config, config_dir, OBS_MIN_MAX_YAML)
    if obs_min_max_cb is not None:
        callbacks.append(obs_min_max_cb)
    instability_cb = make_instability_callback(profile.train_config)
    if instability_cb is not None:
        callbacks.append(instability_cb)
    callbacks.append(make_log_std_schedule_callback(LOG_STD_INIT, LOG_STD_END, TOTAL_TIMESTEPS))

    model.learn(
        total_timesteps=TOTAL_TIMESTEPS,
        callback=callbacks,
        progress_bar=True,
    )

    final_model_path = f"{models_dir}/{profile.model_prefix}_{run_id}"
    model.save(final_model_path)

    # Save best model
    best_model_path = f"{models_dir}/{BEST_MODEL}/{BEST_MODEL}"
    run.save(f"{best_model_path}.zip", base_path=models_dir)

    aggregate_and_print_obs_min_max(env)
    aggregate_and_print_instability_count(env)
    env.close()
    eval_env.close()

    log_best_eval_timestep(models_dir)

    run.finish()


def evaluate(profile: TrainingProfile, model_path: str = ""):
    print_header(profile.display_name + " Evaluation")

    proj_root, _ = get_output_dirs()

    if model_path == "":
        model_path = os.path.join(proj_root, "wandb", "latest-run", "files", "model.zip")

    model = PPO.load(model_path, print_system_info=True, device="cpu")
    print(f"Loaded model from {model_path}")

    eval_env = gym.make(
        get_env_id(),
        config=profile.test_config,
        render_mode="human",
    )
    np.random.seed()
    obs, info = eval_env.reset()
    done, trunc = False, False
    total_reward = 0.0

    while not (done or trunc):
        action, _states = model.predict(obs, deterministic=True)

        obs, reward, done, trunc, info = eval_env.step(action)
        total_reward += reward
        eval_env.render()

    eval_env.close()
    print(f"Total reward: {total_reward}")


def continue_training(profile: TrainingProfile, model_path: str, additional_timesteps: int = ADDITIONAL_TIMESTEPS):
    """
    Continue training from a saved model checkpoint with a new wandb run.

    Args:
        model_path: Path to saved model.zip file
        additional_timesteps: additional steps to train
    """
    print_header(profile.display_name + " - Continue Training")

    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model file not found: {model_path}")

    if not model_path.endswith(".zip"):
        raise ValueError(f"Model path must be a .zip file, got: {model_path}")

    if additional_timesteps <= 0:
        raise ValueError(f"additional_timesteps must be positive, got: {additional_timesteps}")

    print(f"Loading model from: {model_path}")
    print(f"Additional timesteps: {additional_timesteps:,}")

    proj_root, output_root = get_output_dirs()

    new_run_id = generate_run_id()
    run = wandb.init(
        project=profile.project_name,
        id=new_run_id,
        name=new_run_id,
        sync_tensorboard=True,
        monitor_gym=True,
        dir=proj_root,
        save_code=True,
    )

    print(f"New run ID: {new_run_id}")

    # Create output directories for this continuation run
    tensorboard_dir, models_dir, config_dir = make_output_dirs(new_run_id, output_root)

    # Uses current env_config.py
    env = make_subprocvecenv(SEED, profile.train_config, N_ENVS, profile.track_pool)
    eval_env = make_eval_env(EVAL_SEED, profile.test_config)

    model = PPO.load(model_path, env=env, device="auto")

    model.tensorboard_log = tensorboard_dir

    print("Model loaded successfully")
    print("Continuing from checkpoint's timestep count")

    save_config(profile.train_config, config_dir, GYM_OVERRIDES_YAML)
    save_full_gym_config(profile.train_config, config_dir, GYM_YAML)

    rl_config = extract_rl_config(model, additional_timesteps, N_ENVS)
    save_config(rl_config, config_dir, RL_YAML)

    curriculum_config = get_curriculum_config()
    save_config(curriculum_config, config_dir, CURRICULUM_YAML)

    norm_bounds = extract_norm_bounds(eval_env)
    if norm_bounds is not None:
        save_config(norm_bounds, config_dir, NORM_BOUNDS_YAML)

    callbacks = [
        WandbCallback(gradient_save_freq=0, verbose=2),
        get_ckpt_callback(models_dir=models_dir, save_freq=CKPT_SAVE_FREQ),
        get_eval_callback(eval_env=eval_env, models_dir=models_dir),
    ]
    curriculum_cb = make_curriculum_callback(
        curriculum_config, training_mode=profile.train_config.get("training_mode", "")
    )
    if curriculum_cb is not None:
        callbacks.append(curriculum_cb)
    obs_min_max_cb = make_obs_min_max_callback(profile.train_config, config_dir, OBS_MIN_MAX_YAML)
    if obs_min_max_cb is not None:
        callbacks.append(obs_min_max_cb)
    instability_cb = make_instability_callback(profile.train_config)
    if instability_cb is not None:
        callbacks.append(instability_cb)

    print("\nContinuing training...")

    model.learn(
        total_timesteps=additional_timesteps,
        callback=callbacks,
        progress_bar=True,
        reset_num_timesteps=False,  # False to continue from checkpoint
    )

    final_model_path = f"{models_dir}/{profile.model_prefix}_{new_run_id}"
    model.save(final_model_path)

    # Save best model
    best_model_path = f"{models_dir}/{BEST_MODEL}/{BEST_MODEL}"
    run.save(f"{best_model_path}.zip", base_path=models_dir)

    print("\nContinued training completed!")
    print(f"Final model saved: {final_model_path}.zip")
    print(f"New run ID: {new_run_id}")

    aggregate_and_print_obs_min_max(env)
    aggregate_and_print_instability_count(env)
    env.close()
    eval_env.close()

    log_best_eval_timestep(models_dir)

    run.finish()


def transfer_train(
    profile: TrainingProfile,
    model_path: str,
    additional_timesteps: int = ADDITIONAL_TIMESTEPS,
    reset_log_std: float | None = TRANSFER_RESET_LOG_STD,
    reset_critic: bool = TRANSFER_RESET_CRITIC,
):
    """
    Transfer a trained model to a new task with fresh optimizer, LR schedule, and optional log_std reset.

    Loads pretrained weights (preserving learned dynamics knowledge), but resets training state
    so the model adapts to the new task's reward function from a clean optimization starting point.

    Args:
        profile: Target task's training profile (may differ from source model's task)
        model_path: Path to source model .zip file
        additional_timesteps: Total timesteps for the transfer training run
        reset_log_std: Value to reset log_std to (None to keep source model's value)
    """
    print_header(profile.display_name + " - Transfer Training")

    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model file not found: {model_path}")

    if not model_path.endswith(".zip"):
        raise ValueError(f"Model path must be a .zip file, got: {model_path}")

    if additional_timesteps <= 0:
        raise ValueError(f"additional_timesteps must be positive, got: {additional_timesteps}")

    print(f"Loading model from: {model_path}")
    print(f"Target task: {profile.display_name}")
    print(f"Additional timesteps: {additional_timesteps:,}")
    print(f"Reset log_std: {reset_log_std}")
    print(f"Reset critic: {reset_critic}")

    proj_root, output_root = get_output_dirs()

    new_run_id = generate_run_id()
    run = wandb.init(
        project=profile.project_name,
        id=new_run_id,
        name=new_run_id,
        sync_tensorboard=True,
        monitor_gym=True,
        dir=proj_root,
        save_code=True,
    )

    print(f"New run ID: {new_run_id}")

    # Create output directories for this transfer run
    tensorboard_dir, models_dir, config_dir = make_output_dirs(new_run_id, output_root)

    # Uses current env_config.py
    env = make_subprocvecenv(SEED, profile.train_config, N_ENVS, profile.track_pool)
    eval_env = make_eval_env(EVAL_SEED, profile.test_config)

    model = PPO.load(model_path, env=env, device="auto")
    model.tensorboard_log = tensorboard_dir

    # Fresh LR schedule - must set both learning_rate (source of truth) and lr_schedule (cached callable used at each update)
    model.learning_rate = linear_schedule(START_LEARNING_RATE, END_LEARNING_RATE)
    model.lr_schedule = model.learning_rate

    print(f"\nReset LR schedule: {model.lr_schedule(1.0)} -> {model.lr_schedule(0.0)}")

    # Fresh Adam optimizer - use optimizer_class/optimizer_kwargs to match SB3's original construction
    model.policy.optimizer = model.policy.optimizer_class(
        model.policy.parameters(),
        lr=model.learning_rate(1.0),
        **model.policy.optimizer_kwargs,
    )

    pg = model.policy.optimizer.param_groups[0]
    print(f"Reset optimizer: {model.policy.optimizer.__class__.__name__}(lr={pg['lr']}, eps={pg['eps']})")

    # Reset update counter for clean logging (cosmetic — not used in any schedule or logic)
    model._n_updates = 0

    # Reset log_std for fresh exploration in the new task, if set in config file
    if reset_log_std is not None:
        if not hasattr(model.policy, "log_std"):
            raise AttributeError("Policy has no log_std parameter (not a continuous action distribution)")
        model.policy.log_std.data.fill_(reset_log_std)
        print(f"Reset log_std to {reset_log_std}")

    # Reset critic network to random weights (orthogonal init), if set in config file
    if reset_critic:
        model.policy.mlp_extractor.value_net.apply(partial(BasePolicy.init_weights, gain=np.sqrt(2)))
        model.policy.value_net.apply(partial(BasePolicy.init_weights, gain=1.0))
        print("Reset critic network to random weights (orthogonal init)")

    print("\nModel loaded successfully")

    save_config(profile.train_config, config_dir, GYM_OVERRIDES_YAML)
    save_full_gym_config(profile.train_config, config_dir, GYM_YAML)

    rl_config = extract_rl_config(model, additional_timesteps, N_ENVS)
    save_config(rl_config, config_dir, RL_YAML)

    transfer_config = {
        "original_model_path": model_path,
        "additional_timesteps": additional_timesteps,
        "reset_log_std": reset_log_std,
        "reset_critic": reset_critic,
    }
    save_config(transfer_config, config_dir, TRANSFER_YAML)

    curriculum_config = get_curriculum_config()
    save_config(curriculum_config, config_dir, CURRICULUM_YAML)

    norm_bounds = extract_norm_bounds(eval_env)
    if norm_bounds is not None:
        save_config(norm_bounds, config_dir, NORM_BOUNDS_YAML)

    callbacks = [
        WandbCallback(gradient_save_freq=0, verbose=2),
        get_ckpt_callback(models_dir=models_dir, save_freq=CKPT_SAVE_FREQ),
        get_eval_callback(eval_env=eval_env, models_dir=models_dir),
    ]
    curriculum_cb = make_curriculum_callback(
        curriculum_config, training_mode=profile.train_config.get("training_mode", "")
    )
    if curriculum_cb is not None:
        callbacks.append(curriculum_cb)
    obs_min_max_cb = make_obs_min_max_callback(profile.train_config, config_dir, OBS_MIN_MAX_YAML)
    if obs_min_max_cb is not None:
        callbacks.append(obs_min_max_cb)
    instability_cb = make_instability_callback(profile.train_config)
    if instability_cb is not None:
        callbacks.append(instability_cb)

    print("\nStarting transfer training...")

    model.learn(
        total_timesteps=additional_timesteps,
        callback=callbacks,
        progress_bar=True,
        reset_num_timesteps=True,  # New task = fresh step counter and LR schedule
    )

    final_model_path = f"{models_dir}/{profile.model_prefix}_{new_run_id}"
    model.save(final_model_path)

    best_model_path = f"{models_dir}/{BEST_MODEL}/{BEST_MODEL}"
    run.save(f"{best_model_path}.zip", base_path=models_dir)

    print("\nTransfer training completed!")
    print(f"Final model saved: {final_model_path}.zip")
    print(f"New run ID: {new_run_id}")

    aggregate_and_print_obs_min_max(env)
    aggregate_and_print_instability_count(env)
    env.close()
    eval_env.close()

    log_best_eval_timestep(models_dir)

    run.finish()


def evaluate_onnx(profile: TrainingProfile, onnx_path: str):
    """Evaluate an ONNX-exported policy in the simulation environment."""
    from gymkhana.inference import OnnxPolicyRunner

    print_header(profile.display_name + " ONNX Evaluation")

    runner = OnnxPolicyRunner(onnx_path)
    print(f"Loaded ONNX model from {onnx_path}")

    eval_env = gym.make(
        get_env_id(),
        config=profile.test_config,
        render_mode="human",
    )
    np.random.seed()
    obs, info = eval_env.reset()
    done, trunc = False, False
    total_reward = 0.0

    while not (done or trunc):
        action = runner.predict(obs)
        obs, reward, done, trunc, info = eval_env.step(np.array([action]))
        total_reward += reward
        eval_env.render()

    eval_env.close()
    print(f"Total reward: {total_reward}")


def download_and_evaluate(profile: TrainingProfile, run_id: str):
    """Download model from wandb and evaluate it."""
    print_header("Downloading and Evaluating Model from WandB")

    _, output_root = get_output_dirs()
    download_dir = os.path.join(output_root, "downloads", run_id)
    model_cache_path = os.path.join(download_dir, "model.zip")

    # Use cached model if available, otherwise download
    if os.path.exists(model_cache_path):
        print(f"Using cached model from {download_dir}")
    else:
        print(f"Downloading model from wandb run: {run_id}")
        model_cache_path = download_model_from_wandb(run_id, download_dir, profile.model_prefix, profile.project_name)
        print(f"Model cached to {download_dir}")

    evaluate(profile=profile, model_path=model_cache_path)


def main(profile: TrainingProfile):
    """Parse user argument and reroute to correct method"""
    parser = argparse.ArgumentParser(description="Train or evaluate a model")
    parser.add_argument(
        "--m",
        choices=["t", "e", "d", "c", "f", "x"],
        default="t",
        help="Run mode: 't' train, 'e' evaluate, 'd' download+evaluate, 'c' continue, 'f' transfer, 'x' evaluate ONNX",
    )
    parser.add_argument(
        "--path",
        type=str,
        default="",
        help="Path to trained model for evaluation or continue training (uses latest if not specified for mode 'e')",
    )
    parser.add_argument(
        "--run_id",
        type=str,
        default="",
        help="Wandb run ID to download model from (required for mode 'd')",
    )
    args = parser.parse_args()

    if args.m == "t":
        train(profile=profile)
    elif args.m == "e":
        evaluate(profile=profile, model_path=args.path)
    elif args.m == "d":
        if not args.run_id:
            parser.error("--run_id is required when using mode 'd' (download)")
        download_and_evaluate(profile=profile, run_id=args.run_id)
    elif args.m == "c":
        if not args.path:
            parser.error("--path is required when using mode 'c' (continue training)")
        continue_training(profile=profile, model_path=args.path)
    elif args.m == "f":
        if not args.path:
            parser.error("--path is required when using mode 'f' (transfer/fine-tune)")
        transfer_train(profile=profile, model_path=args.path)
    elif args.m == "x":
        if not args.path:
            parser.error("--path is required when using mode 'x' (evaluate ONNX)")
        evaluate_onnx(profile=profile, onnx_path=args.path)
