"""STD system identification rollout helper.

Owns a single GKEnv instance configured for sim-vs-real replay and exposes a
`run(window) -> dict[str, np.ndarray]` interface matching the loss-module
contract. Build once per worker; call `set_params(params)` between Optuna
trials to hot-swap PAC2002 coefficients without rebuilding the env.

See docs/plan/OPTUNA_SYS_ID.md for the locked design decisions.
"""

from __future__ import annotations

from typing import Callable

import numpy as np

from examples.analysis.sysid.dataset import Window
from examples.analysis.sysid.env import SYSID_PARAMS
from gymkhana.envs.gymkhana_env import GKEnv
from train.config.env_config import get_drift_train_config

_SYSID_OVERRIDES: dict = {
    "num_agents": 1,
    "model": "std",
    "control_input": ["speed", "steering_angle"],
    "observation_config": {"type": "dynamic_state"},
    "normalize_obs": False,
    "normalize_act": False,
    "prevent_instability": False,
    "track_direction": "normal",
    "map": "Spielberg_blank",
    "render_track_lines": False,
    "render_arc_length_annotations": False,
    "render_lookahead_curvatures": False,
    "debug_frenet_projection": False,
    "record_obs_min_max": False,
}


def _build_sysid_config(params: dict | None) -> dict:
    config = get_drift_train_config()
    config.update(_SYSID_OVERRIDES)
    # Default to SYSID_PARAMS so the env's R_w matches what dataset.py
    # divided VESC rs_core_speed by when seeding init_state[7:9].
    config["params"] = params if params is not None else SYSID_PARAMS
    return config


class Rollout:
    """Replays a `Window` through a single, reusable GKEnv instance.

    Construct once per worker process; call `set_params(params)` between
    trials to hot-swap PAC2002 coefficients without rebuilding the env.
    Supports use as a context manager to guarantee `close()`.
    """

    def __init__(self, params: dict | None = None):
        config = _build_sysid_config(params)
        self._env = GKEnv(config=config)
        self._dt = float(self._env.timestep)
        self._agent_id = self._env.agent_ids[0]

    @property
    def dt(self) -> float:
        return self._dt

    def set_params(self, params: dict) -> None:
        self._env.configure({"params": params})

    def close(self) -> None:
        self._env.close()

    def __enter__(self) -> Rollout:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def run(self, window: Window) -> dict[str, np.ndarray]:
        assert window.init_state.shape == (9,), (
            f"Expected 9-wide init_state (STD with VESC-seeded omegas); got {window.init_state.shape}"
        )
        n = len(window.cmd_steer)

        v_x = np.empty(n + 1, dtype=np.float64)
        v_y = np.empty(n + 1, dtype=np.float64)
        yaw_rate = np.empty(n + 1, dtype=np.float64)
        omega = np.empty(n + 1, dtype=np.float64)
        pose = np.empty((n + 1, 2), dtype=np.float64)
        yaw = np.empty(n + 1, dtype=np.float64)
        beta = np.empty(n + 1, dtype=np.float64)

        # `dynamic_state` obs surfaces body-frame v_x, v_y, yaw_rate from
        # agent.standard_state — same quantities the bag was built from.
        # ω_front / ω_rear live on raw STD state[7:9]; mean matches the AWD
        # reset assumption (real ω is a single VESC scalar seeding both).
        # World-frame pose_x / pose_y feed the integrated XY tracking channel.
        agent = self._env.sim.agents[0]
        obs, _ = self._env.reset(options={"states": window.init_state.reshape(1, 9)})
        agent_obs = obs[self._agent_id]
        v_x[0] = agent_obs["linear_vel_x"]
        v_y[0] = agent_obs["linear_vel_y"]
        yaw_rate[0] = agent_obs["ang_vel_z"]
        omega[0] = 0.5 * (agent.state[7] + agent.state[8])
        pose[0, 0] = agent_obs["pose_x"]
        pose[0, 1] = agent_obs["pose_y"]
        yaw[0] = agent_obs["pose_theta"]
        beta[0] = agent_obs["beta"]

        for k in range(n):
            action = np.array(
                [[window.cmd_steer[k], window.cmd_speed[k]]],
                dtype=np.float32,
            )
            obs, _, _, _, _ = self._env.step(action)
            agent_obs = obs[self._agent_id]
            v_x[k + 1] = agent_obs["linear_vel_x"]
            v_y[k + 1] = agent_obs["linear_vel_y"]
            yaw_rate[k + 1] = agent_obs["ang_vel_z"]
            omega[k + 1] = 0.5 * (agent.state[7] + agent.state[8])
            pose[k + 1, 0] = agent_obs["pose_x"]
            pose[k + 1, 1] = agent_obs["pose_y"]
            yaw[k + 1] = agent_obs["pose_theta"]
            beta[k + 1] = agent_obs["beta"]

        a_x = np.gradient(v_x, self._dt)
        # Env wraps pose_theta to [0, 2π); unwrap to match bag's vicon_yaw.
        yaw = np.unwrap(yaw)

        sim = {
            "yaw_rate": yaw_rate,
            "v_y": v_y,
            "a_x": a_x,
            "v_x": v_x,
            "omega": omega,
            "pose": pose,
            "yaw": yaw,
            "beta": beta,
        }

        # Trial params can drive the integrator to NaN/inf. Surface this loudly
        # so the Optuna objective can map it to a prunable trial loss instead of
        # silently propagating non-finite values into the loss.
        for ch, arr in sim.items():
            if not np.all(np.isfinite(arr)):
                raise FloatingPointError(
                    f"Non-finite sim signal in channel {ch!r} on window t0_idx={window.t0_idx}; "
                    "trial params likely caused integrator divergence."
                )

        return sim

    def run_debug(self, window: Window) -> dict[str, np.ndarray]:
        """Reset + step like `run`, but return the full obs trace.

        For visual validation only. Not on the Optuna hot path. Returns
        body-frame velocities, world-frame pose, steering, slip, wheel
        angular velocities (omega_front, omega_rear, read from raw STD
        state[7:9]), plus the same finite-diff `a_x` the loss uses.
        """
        assert window.init_state.shape == (9,), (
            f"Expected 9-wide init_state (STD with VESC-seeded omegas); got {window.init_state.shape}"
        )
        n = len(window.cmd_steer)
        keys = ("pose_x", "pose_y", "pose_theta", "linear_vel_x", "linear_vel_y", "ang_vel_z", "delta", "beta")
        traces: dict[str, np.ndarray] = {k: np.empty(n + 1, dtype=np.float64) for k in keys}
        traces["omega_front"] = np.empty(n + 1, dtype=np.float64)
        traces["omega_rear"] = np.empty(n + 1, dtype=np.float64)
        agent = self._env.sim.agents[0]

        def record(idx: int, agent_obs: dict) -> None:
            for k in keys:
                traces[k][idx] = float(agent_obs[k])
            traces["omega_front"][idx] = float(agent.state[7])
            traces["omega_rear"][idx] = float(agent.state[8])

        obs, _ = self._env.reset(options={"states": window.init_state.reshape(1, 9)})
        record(0, obs[self._agent_id])

        for k in range(n):
            action = np.array([[window.cmd_steer[k], window.cmd_speed[k]]], dtype=np.float32)
            obs, _, _, _, _ = self._env.step(action)
            record(k + 1, obs[self._agent_id])

        traces["a_x"] = np.gradient(traces["linear_vel_x"], self._dt)
        traces["pose_theta"] = np.unwrap(traces["pose_theta"])
        return traces


def make_rollout_fn(params: dict) -> Callable[[Window], dict[str, np.ndarray]]:
    """Convenience: build a `Rollout` and return its `run` bound method.

    For Phase 1 tests and one-off baselines. Optuna study code should
    construct `Rollout` directly and call `set_params` between trials so
    a single env is reused per worker.
    """
    return Rollout(params=params).run
