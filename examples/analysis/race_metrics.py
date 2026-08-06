"""Racing Lap-Time Metrics

Runs a controller for N laps and reports average and best lap time.

Laps are detected from the Frenet arc length ``s`` rather than the env's built-in
``lap_counts``: the built-in counter anchors its start line to the (randomly sampled)
reset pose, uses a 0.32 m proximity blob that a drifting car can miss entirely, and
caps out at 2 laps. Arc length wraps to 0 at the track's own start/finish line, so a
lap is simply an ``s`` wrap — track-anchored, lateral-position independent, and
unlimited in lap count.

Usage — learned (PPO) controller:
    python examples/analysis/race_metrics.py --controller_type learned --run_id <wandb_run_id>

Usage — classic baselines:
    python examples/analysis/race_metrics.py --controller_type stanley
    python examples/analysis/race_metrics.py --controller_type stmpc

The learned controller expects the model at outputs/downloads/<run_id>/model.zip; download
it first with `python train/ppo_race.py --m d --run_id <run_id>`.

Output saved to figures/analysis/race_metrics/<controller_type or run_id>/
"""

import argparse
import os

import gymnasium as gym
import numpy as np

from examples.controllers import TARGET_SPEED, create_controller
from train.config.env_config import get_env_id
from train.train_utils import get_output_dirs, print_header

CONTROLLER_TYPE = "learned"
RUN_ID = "178a1a5l"
DESC = "drift model - CW & CCW on Drift_large, with `sparse_width_obs` = True"

# CONTROLLER_TYPE = "stanley"
# DESC = "stanley"
# RUN_ID = ""

MAP = "Drift"
N_LAPS = 10
SEED = 42

# Speed the car is launched at on every reset. A standing start leaves the tire model at near-zero
# slip velocity, where steering produces almost no yaw, so the car ploughs straight off the track.
# Launching rolling avoids that. Only affects the out-lap, which is never timed.
INIT_SPEED = TARGET_SPEED

# Give up rather than loop forever if the controller cannot string laps together.
MAX_RESETS = 50

# 11 laps of a ~25 m track at ~5 m/s is ~5.5k steps; the env default of 4096 (41 s) is far too low.
MAX_EPISODE_STEPS = 100_000

AGENT_IDX = 0


def build_env(controller, map_name, render=False):
    """Create the eval env for a controller, with racing-specific config overrides.

    ``render_mode`` must be set here, at construction: with ``render_mode=None`` a later
    ``env.render()`` call is a silent no-op.
    """
    config = controller.get_env_config()
    config["map"] = map_name
    config["max_episode_steps"] = MAX_EPISODE_STEPS
    config["track_direction"] = "normal"  # test config defaults to "random", re-rolled every reset

    env = gym.make(get_env_id(), config=config, render_mode="human" if render else None)
    controller.initialize(env)
    return env


def reset_at_start_line(env):
    """Reset the car at the track's start/finish line (s=0), on the centerline, rolling.

    Resets via ``states`` rather than ``poses``: a ``poses`` reset zeroes the whole state
    vector apart from x/y/yaw, so the car would start from rest. The 7-wide STD user state
    is ``[x, y, delta, v, yaw, r, beta]``.

    The s=0 placement only affects reproducibility — lap timing is anchored to ``s``, so it
    would be correct from a random spawn too.
    """
    x, y, yaw = env.unwrapped.track.frenet_to_cartesian(s=0, ey=0, ephi=0)
    obs, _ = env.reset(options={"states": np.array([[x, y, 0.0, INIT_SPEED, yaw, 0.0, 0.0]])})
    return obs


def collect_lap_times(env, controller, n_laps, render=False):
    """Drive until ``n_laps`` complete laps have been timed.

    Returns:
        lap_times: list of lap durations in seconds.
        resets: number of episode terminations (off-track) or truncations along the way.
    """
    dt = env.unwrapped.timestep
    track_length = env.unwrapped.track.centerline.spline.s[-1]
    print(f"Track length: {track_length:.2f} m")

    obs = reset_at_start_line(env)
    controller.on_reset(obs)

    lap_times = []
    resets = 0
    step = 0
    s_prev = env.unwrapped._frenet_cache[AGENT_IDX, 0]
    armed = False
    lap_start_step = None  # stays None until the first wrap arms the timer

    while len(lap_times) < n_laps:
        action = controller.get_action(obs)
        obs, _, done, trunc, _ = env.step(action)
        if render:
            env.render()
        step += 1

        s = env.unwrapped._frenet_cache[AGENT_IDX, 0]

        # A lap is an `s` wrap: high near the end of the lap, low just after. `armed` forces the
        # car to reach mid-track first, so one oscillating across the start line cannot bank
        # spurious ~0 s laps. The fractions are loose — a step covers <0.2 m, the bands are metres.
        if not armed and s > 0.5 * track_length:
            armed = True
        elif armed and s_prev > 0.75 * track_length and s < 0.25 * track_length:
            if lap_start_step is not None:
                lap_times.append((step - lap_start_step) * dt)
                print(f"  Lap {len(lap_times)}/{n_laps}: {lap_times[-1]:.3f} s")
            lap_start_step, armed = step, False
        s_prev = s

        if done or trunc:
            resets += 1
            if resets > MAX_RESETS:
                raise RuntimeError(
                    f"Exceeded {MAX_RESETS} resets after banking only {len(lap_times)}/{n_laps} laps. "
                    "The controller cannot complete laps on this track."
                )
            print(f"  Episode ended ({'terminated' if done else 'truncated'}) — discarding partial lap, resetting")
            obs = reset_at_start_line(env)
            controller.on_reset(obs)
            s_prev = env.unwrapped._frenet_cache[AGENT_IDX, 0]
            lap_start_step, armed = None, False

    return lap_times, resets


def save_metrics(lap_times, resets, output_path, controller_label="", map_name="", desc=""):
    """Format, print, and save lap-time metrics to a file."""
    lap_times = np.asarray(lap_times)
    bar = "=" * 50

    lines = [
        "",
        bar,
        "RACE METRICS",
        bar,
        f"Controller: {controller_label}",
        f"Map: {map_name}",
        f"Laps timed: {len(lap_times)}",
        f"Resets (off-track / truncation): {resets}",
        "",
        *(f"  Lap {i + 1:2d}: {t:.3f} s" for i, t in enumerate(lap_times)),
        "",
        f"Average lap time: {np.mean(lap_times):.3f} s (std: {np.std(lap_times):.3f} s)",
        f"Best lap time:    {np.min(lap_times):.3f} s",
        bar,
    ]

    text = "\n".join(lines)
    print(text)

    with open(output_path, "w") as f:
        f.write(text + "\n")
        if desc:
            f.write(f"\nNote: {desc}\n")
    print(f"Metrics saved to: {output_path}")


def parse_args():
    parser = argparse.ArgumentParser(description="Racing Lap-Time Metrics")
    parser.add_argument(
        "--controller_type", default=CONTROLLER_TYPE, help="Controller type: 'learned', 'stanley', 'steer', 'stmpc'"
    )
    parser.add_argument("--run_id", default=RUN_ID, help="Wandb run ID for the learned model")
    parser.add_argument("--desc", default=None, help="Short description of the model")
    parser.add_argument("--laps", type=int, default=N_LAPS, help="Number of laps to time")
    parser.add_argument("--map", default=MAP, help="Map name")
    parser.add_argument("--render", action="store_true", help="Open a render window to watch the run")
    return parser.parse_args()


def main():
    args = parse_args()
    controller_type = args.controller_type
    run_id = args.run_id
    map_name = args.map

    if args.laps < 1:
        raise SystemExit("--laps must be at least 1")
    if controller_type == "learned" and not run_id:
        raise SystemExit("--run_id is required for the learned controller")

    desc = args.desc if args.desc is not None else (DESC if controller_type == CONTROLLER_TYPE else controller_type)

    print_header("Racing Lap-Time Metrics")
    print(f"Controller: {controller_type}, Run ID: {run_id}, Map: {map_name}, Laps: {args.laps}")
    print(f"Description: {desc}")

    proj_root, _ = get_output_dirs()

    label = run_id if controller_type == "learned" else controller_type
    subfolder = f"{proj_root}/figures/analysis/race_metrics/{label}"
    os.makedirs(subfolder, exist_ok=True)

    model_path = f"{proj_root}/outputs/downloads/{run_id}/model.zip" if controller_type == "learned" else None
    controller = create_controller(controller_type, model_path=model_path, map=map_name)

    env = build_env(controller, map_name, render=args.render)
    np.random.seed(SEED)

    print(f"\nRunning {args.laps} laps...")
    lap_times, resets = collect_lap_times(env, controller, args.laps, render=args.render)
    env.close()

    save_metrics(
        lap_times,
        resets,
        os.path.join(subfolder, "metrics.txt"),
        controller_label=controller_type + (f" ({run_id})" if controller_type == "learned" else ""),
        map_name=map_name,
        desc=desc,
    )

    print("\nDone!")


if __name__ == "__main__":
    main()
