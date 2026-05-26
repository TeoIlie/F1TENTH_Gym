"""STD system identification dataset loader.

Loads a 100 Hz Vicon-recorded NPZ bag, slices it into fixed-length windows
suitable for sim-vs-real loss evaluation, and (optionally) appends mirrored
copies to neutralize left/right asymmetry in the data.

Each Window holds:
  - the 9-element STD user state at t0 (`[x, y, delta, v, yaw, yaw_rate,
    beta, omega_front, omega_rear]`), with wheel angular velocities seeded
    from VESC `rs_core_speed/R_w` (AWD: same scalar to both wheels),
  - the command sequence to replay (cmd_steer, cmd_speed),
  - the real signals to score against: body-frame (v_x, v_y, yaw_rate, a_x),
    drivetrain anchor (omega = rs_core_speed / R_w), and world-frame pose
    (x, y) stacked as `(N+1, 2)` for the integrated XY tracking channel.

See docs/plan/OPTUNA_SYS_ID.md for the locked design decisions.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter

from examples.analysis.sysid.env import SYSID_PARAMS

CHANNELS = ("yaw_rate", "v_y", "a_x", "v_x", "omega", "pose", "yaw", "beta")

# init_state layout (9-wide, matches STD's `user_state_lens()` 9-branch):
#   [x, y, delta, v, yaw, yaw_rate, beta, omega_front, omega_rear]
# Under left/right mirror (reflection across the longitudinal body axis):
#   x, v stay; y, delta, yaw, yaw_rate, beta flip sign; wheel speeds are
#   positive scalars and stay (a left-mirrored car still spins its wheels
#   forward at the same rate).
_MIRROR_INIT_SIGNS = np.array([1.0, -1.0, -1.0, 1.0, -1.0, -1.0, -1.0, 1.0, 1.0])

# World-frame pose under L/R mirror: x stays, y flips. Broadcasts over the
# (N+1, 2) `real_pose` array (column 0 = x, column 1 = y).
_MIRROR_POSE_SIGNS = np.array([1.0, -1.0])


@dataclass(frozen=True)
class Window:
    t0_idx: int
    init_state: np.ndarray  # shape (9,) — see _MIRROR_INIT_SIGNS for layout
    cmd_steer: np.ndarray  # shape (N,)
    cmd_speed: np.ndarray  # shape (N,)
    real_v_x: np.ndarray  # shape (N+1,)
    real_v_y: np.ndarray
    real_yaw_rate: np.ndarray
    real_a_x: np.ndarray
    real_omega: np.ndarray  # VESC rs_core_speed/R_w over the window (rad/s)
    real_pose: np.ndarray  # shape (N+1, 2) — world-frame [x, y] per timestep
    real_yaw: np.ndarray  # shape (N+1,) — unwrapped vicon_yaw
    real_beta: np.ndarray  # shape (N+1,) — arctan2(vy, vx)
    is_mirrored: bool
    # Resolved absolute path of the source NPZ. Empty when the window was built
    # by load_dataset directly (back-compat); stamped by load_dataset_tagged /
    # load_datasets so multi-bag callers can group windows by origin.
    source_bag: str = ""


@dataclass(frozen=True)
class Dataset:
    windows: list[Window]
    dt: float
    n_candidates: int
    n_dropped_low_speed: int
    n_dropped_nonfinite: int


def mirror_window(w: Window) -> Window:
    # Only the fields below change under L/R reflection — `replace` forwards
    # everything else (t0_idx, source_bag, future fields) automatically so adding
    # to Window can't silently break mirror semantics.
    # `real_omega` is copied (not flipped) because wheel speeds are sign-symmetric.
    return replace(
        w,
        init_state=w.init_state * _MIRROR_INIT_SIGNS,
        cmd_steer=-w.cmd_steer,
        cmd_speed=w.cmd_speed.copy(),
        real_v_x=w.real_v_x.copy(),
        real_v_y=-w.real_v_y,
        real_yaw_rate=-w.real_yaw_rate,
        real_a_x=w.real_a_x.copy(),
        real_omega=w.real_omega.copy(),
        real_pose=w.real_pose * _MIRROR_POSE_SIGNS,
        real_yaw=-w.real_yaw,
        real_beta=-w.real_beta,
        is_mirrored=not w.is_mirrored,
    )


def load_dataset(
    npz_path: str,
    window_length_s: float = 1.5,
    stride_s: float = 0.5,
    min_speed: float = 0.3,
    mirror: bool = False,
    sg_window: int = 21,
    sg_polyorder: int = 2,
    dt: float = 0.01,
) -> Dataset:
    data = np.load(npz_path)
    t = data["t"]
    cmd_speed = data["cmd_speed"]
    cmd_steer = data["cmd_steer"]
    vicon_x = data["vicon_x"]
    vicon_y = data["vicon_y"]
    vicon_yaw = data["vicon_yaw"]
    vicon_body_vx = data["vicon_body_vx"]
    vicon_body_vy = data["vicon_body_vy"]
    vicon_r = data["vicon_r"]
    if "rs_core_speed" not in data.files:
        raise KeyError(
            f"NPZ {npz_path!r} is missing 'rs_core_speed' (VESC wheel-speed feedback). "
            "Sysid requires VESC-seeded wheel angular velocity for the 9-wide STD reset; "
            "re-export the bag with rs_core_speed included."
        )
    # AWD assumption: a single VESC scalar seeds both omega_front and omega_rear.
    # R_w must match what `Rollout` passes to GKEnv; both pull from SYSID_PARAMS.
    omega_full = data["rs_core_speed"] / SYSID_PARAMS["R_w"]

    n_total = len(t)
    n_steps = int(round(window_length_s / dt))
    stride = int(round(stride_s / dt))
    if n_total < n_steps + 1:
        raise ValueError(f"Bag too short for window_length_s={window_length_s} (n_total={n_total})")

    # Smoothed real longitudinal accel: computed once on the full bag, then sliced.
    real_a_x_full = np.gradient(savgol_filter(vicon_body_vx, sg_window, sg_polyorder), dt)

    speed_full = np.hypot(vicon_body_vx, vicon_body_vy)
    beta_full = np.where(speed_full > min_speed, np.arctan2(vicon_body_vy, vicon_body_vx), 0.0)

    windows: list[Window] = []
    t0_max = n_total - n_steps - 1
    n_candidates = 0
    n_dropped_low_speed = 0
    n_dropped_nonfinite = 0
    for t0 in range(0, t0_max + 1, stride):
        n_candidates += 1
        end = t0 + n_steps  # exclusive for cmds; cmds drive N steps, real signals span N+1 samples
        cmd_s = cmd_steer[t0:end]
        cmd_v = cmd_speed[t0:end]
        rvx = vicon_body_vx[t0 : end + 1]
        rvy = vicon_body_vy[t0 : end + 1]
        rr = vicon_r[t0 : end + 1]
        rax = real_a_x_full[t0 : end + 1]
        romega = omega_full[t0 : end + 1]  # romega[0] seeds init_state[7:9] (AWD); full slice is scored.
        rpose = np.stack([vicon_x[t0 : end + 1], vicon_y[t0 : end + 1]], axis=-1)  # (N+1, 2)
        ryaw = vicon_yaw[t0 : end + 1]
        rbeta = np.arctan2(rvy, rvx)

        if np.mean(speed_full[t0 : end + 1]) < min_speed:
            n_dropped_low_speed += 1
            continue
        if not all(np.all(np.isfinite(a)) for a in (rvx, rvy, rr, rax, romega, rpose, ryaw, rbeta, cmd_s, cmd_v)):
            n_dropped_nonfinite += 1
            continue

        init_state = np.array(
            [
                vicon_x[t0],
                vicon_y[t0],
                cmd_steer[t0],
                speed_full[t0],
                vicon_yaw[t0],
                vicon_r[t0],
                beta_full[t0],
                romega[0],
                romega[0],
            ],
            dtype=float,
        )

        windows.append(
            Window(
                t0_idx=int(t0),
                init_state=init_state,
                cmd_steer=cmd_s.astype(float),
                cmd_speed=cmd_v.astype(float),
                real_v_x=rvx.astype(float),
                real_v_y=rvy.astype(float),
                real_yaw_rate=rr.astype(float),
                real_a_x=rax.astype(float),
                real_omega=romega.astype(float),
                real_pose=rpose.astype(float),
                real_yaw=ryaw.astype(float),
                real_beta=rbeta.astype(float),
                is_mirrored=False,
            )
        )

    if not windows:
        raise ValueError(f"No windows survived filtering for {npz_path}")

    if mirror:
        windows = windows + [mirror_window(w) for w in windows]

    return Dataset(
        windows=windows,
        dt=dt,
        n_candidates=n_candidates,
        n_dropped_low_speed=n_dropped_low_speed,
        n_dropped_nonfinite=n_dropped_nonfinite,
    )


def load_dataset_tagged(npz_path: str, **kwargs) -> Dataset:
    """``load_dataset`` then stamp the resolved NPZ path into each window's
    ``source_bag``. Use this (or ``load_datasets``) whenever downstream code
    needs to group windows by origin (multi-bag validate/study runs)."""
    ds = load_dataset(npz_path, **kwargs)
    resolved = str(Path(npz_path).resolve())
    tagged = [replace(w, source_bag=resolved) for w in ds.windows]
    return replace(ds, windows=tagged)


def load_datasets(npz_paths: list[str], **kwargs) -> Dataset:
    """Load N bags, tag each window with its source path, concatenate windows
    into one Dataset. Inputs are sorted by resolved path before loading so
    window order is deterministic regardless of CLI argument order.

    Weighting is equal-per-window: a 200-window bag contributes 10× more than a
    20-window bag. All bags share window_length_s / stride_s / min_speed /
    mirror / dt via **kwargs forwarded to ``load_dataset``.
    """
    if not npz_paths:
        raise ValueError("load_datasets requires at least one path")
    sorted_paths = sorted({str(Path(p).resolve()) for p in npz_paths})
    parts = [load_dataset_tagged(p, **kwargs) for p in sorted_paths]
    dts = {p.dt for p in parts}
    if len(dts) != 1:
        raise ValueError(f"Heterogeneous dt across bags: {dts}")
    return Dataset(
        windows=[w for p in parts for w in p.windows],
        dt=parts[0].dt,
        n_candidates=sum(p.n_candidates for p in parts),
        n_dropped_low_speed=sum(p.n_dropped_low_speed for p in parts),
        n_dropped_nonfinite=sum(p.n_dropped_nonfinite for p in parts),
    )
