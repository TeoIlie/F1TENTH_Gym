import gymnasium as gym
import numpy as np

from examples.controllers import (
    FRENET_N_GAIN,
    FRENET_U_GAIN,
    TARGET_SPEED,
    create_controller,
)
from examples.controllers.steer_controller import (
    LOOKAHEAD_N_POINTS,
)
from examples.examples_utils import display_drift_obs
from train.config.env_config import get_env_id

NUM_STEPS = 20_000


def main():
    # Create controller using factory pattern
    controller = create_controller("stanley", target_speed=TARGET_SPEED, map="Drift")

    env = gym.make(
        get_env_id(),
        config=controller.get_env_config(),
        render_mode="human",
    )

    x, y, yaw = env.unwrapped.track.frenet_to_cartesian(s=0, ey=0, ephi=0)

    init_state = np.array(
        [
            [
                x,  # x position (Cartesian coordinates)
                y,  # y position (Cartesian coordinates)
                0.0,  # delta (steering angle)
                TARGET_SPEED,  # v (velocity)
                yaw,  # yaw angle (Cartesian orientation)
                0.0,  # yaw_rate
                0.0,  # slip_angle
            ]
        ]
    )
    obs, _ = env.reset(options={"states": init_state})

    print("Centerline Tracking Controller")
    print("==============================")
    print(f"Controller gains: Kn={FRENET_N_GAIN}, Ku={FRENET_U_GAIN}")
    print(f"Target speed: {TARGET_SPEED} m/s")
    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}")
    print()

    # Run simulation
    done = False
    step_count = 0
    total_lateral_error = 0.0
    total_heading_error = 0.0
    total_reward = 0.0

    for step in range(NUM_STEPS):
        # Get action
        action = controller.get_action(obs)

        # Step environment
        obs, reward, done, truncated, info = env.step(action)

        # Extract obs for stats
        frenet_u = obs[2]  # heading error
        frenet_n = obs[3]  # lateral deviation

        total_lateral_error += abs(frenet_n)
        total_heading_error += abs(frenet_u)
        total_reward += reward

        display_drift_obs(step, obs, reward, LOOKAHEAD_N_POINTS, total_reward)

        env.render()

        step_count += 1

    # Print final statistics
    avg_lateral = total_lateral_error / step_count
    avg_heading = total_heading_error / step_count
    avg_reward = total_reward / step_count
    print("\nFinal Statistics:")
    print(f"  Total steps: {step_count}")
    print(f"  Average lateral error: {avg_lateral:.4f} m")
    print(f"  Average heading error: {avg_heading:.4f} rad ({np.degrees(avg_heading):.2f} deg)")
    print(f"  Total reward: {total_reward:.2f}")
    print(f"  Average reward per step: {avg_reward:.4f}")

    env.close()


if __name__ == "__main__":
    main()
