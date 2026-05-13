"""Shared fixtures for sysid tests.

Provides `make_npz` (synthetic bag writer mirroring the real bag schema) and
session-scoped `synthetic_bag_npz` / module-scoped `synthetic_dataset` fixtures
so tests that previously gated on a checked-out real bag can run in CI.
"""

from __future__ import annotations

import numpy as np
import pytest

DT = 0.01


def make_npz(
    path,
    n: int = 500,
    speed=1.0,
    vy=0.0,
    yaw_rate=0.0,
    steer=0.1,
    yaw=0.0,
    rs_core_speed=None,
    include_rs_core_speed: bool = True,
):
    """Write a synthetic 100 Hz NPZ matching the real bag schema.

    Scalar args broadcast to length-n arrays. `rs_core_speed` defaults to
    `vx_arr` (no-slip). Set `include_rs_core_speed=False` to omit the field.
    """
    t = np.arange(n) * DT
    vx_arr = np.broadcast_to(np.asarray(speed, dtype=float), (n,)).copy()
    vy_arr = np.broadcast_to(np.asarray(vy, dtype=float), (n,)).copy()
    yaw_rate_arr = np.broadcast_to(np.asarray(yaw_rate, dtype=float), (n,)).copy()
    steer_arr = np.broadcast_to(np.asarray(steer, dtype=float), (n,)).copy()
    yaw_arr = np.broadcast_to(np.asarray(yaw, dtype=float), (n,)).copy()
    x = np.cumsum(vx_arr) * DT
    y = np.cumsum(vy_arr) * DT
    fields = dict(
        t=t,
        cmd_speed=vx_arr.copy(),
        cmd_steer=steer_arr,
        vicon_x=x,
        vicon_y=y,
        vicon_yaw=yaw_arr,
        vicon_body_vx=vx_arr,
        vicon_body_vy=vy_arr,
        vicon_r=yaw_rate_arr,
    )
    if include_rs_core_speed:
        if rs_core_speed is None:
            rs_arr = vx_arr.copy()
        else:
            rs_arr = np.broadcast_to(np.asarray(rs_core_speed, dtype=float), (n,)).copy()
        fields["rs_core_speed"] = rs_arr
    np.savez(path, **fields)


@pytest.fixture(scope="session")
def synthetic_bag_npz(tmp_path_factory):
    """Session-scoped synthetic bag standing in for the real circle bag.

    Non-trivial steer + lateral velocity so the rollout doesn't trivially
    reproduce it (baseline loss > 0), but mild enough that midpoint-of-STAGE1
    params don't diverge. 10 s at 100 Hz → ~17 windows at default 1.5 s window
    / 0.5 s stride after the low-speed mask (speed=2.0 ≫ min_speed=0.3).
    """
    path = tmp_path_factory.mktemp("sysid_bag") / "synthetic_circle_100Hz.npz"
    make_npz(str(path), n=1000, speed=2.0, vy=0.1, yaw_rate=0.3, steer=0.15)
    return str(path)


@pytest.fixture(scope="module")
def synthetic_dataset(synthetic_bag_npz):
    """Module-scoped dataset loaded from the synthetic bag."""
    from examples.analysis.sysid.dataset import load_dataset

    return load_dataset(synthetic_bag_npz, mirror=False)
