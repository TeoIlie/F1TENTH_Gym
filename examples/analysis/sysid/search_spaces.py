"""Optuna search-space definitions for STD system identification (Phase 3).

Stage-1 / Stage-2 distributions and the param-overlay helper. Bounds are
locked by the Phase-2 sensitivity analysis — see
docs/plan/OPTUNA_SYS_ID_SENSITIVITY_REPORT.md §6 for justifications, and
docs/plan/OPTUNA_SYS_ID.md for the overall design.
"""

from __future__ import annotations

from copy import deepcopy

from optuna.distributions import BaseDistribution, FloatDistribution

from examples.analysis.sysid.env import SYSID_PARAMS
from examples.analysis.sysid.sensitivity import FROZEN_PARAMS

STAGE1_SPACE: dict[str, BaseDistribution] = {
    "I_z": FloatDistribution(low=0.05, high=0.20),
    "a_max": FloatDistribution(low=1.0, high=6.0),
    "I_y_w": FloatDistribution(low=0.0005, high=0.0025),
    "tire_p_cy1": FloatDistribution(low=0.34, high=2.36),
    "tire_p_cx1": FloatDistribution(low=1.03, high=2.26),
    "tire_p_ky1": FloatDistribution(low=-80.0, high=-15.0),
    # T_steer added here (single line) if the sensitivity follow-up promotes it.
}

STAGE2_SPACE: dict[str, BaseDistribution] = {
    "tire_p_dy1": FloatDistribution(low=0.7, high=1.5),
    "tire_p_dx1": FloatDistribution(low=0.7, high=1.5),
    "tire_p_ey1": FloatDistribution(low=-0.013, high=-0.0019),
    "tire_p_ex1": FloatDistribution(low=0.12, high=0.81),
}

STAGE_SPACES: dict[int, dict[str, BaseDistribution]] = {1: STAGE1_SPACE, 2: STAGE2_SPACE}


def apply_trial_params(base_params: dict, trial_values: dict[str, float]) -> dict:
    """Return ``deepcopy(base_params)`` with ``trial_values`` overlaid.

    Non-mutating: the input ``base_params`` is never modified.
    """
    out = deepcopy(base_params)
    for name, value in trial_values.items():
        out[name] = value
    return out


# --- Import-time invariants -------------------------------------------------
# Fail loudly if a search space drifts out of sync with SYSID_PARAMS / freeze
# decisions. Cheap (runs once on first import) and prevents silent bugs where
# Optuna would suggest a value for a key that doesn't exist in the params dict.
def _validate_spaces() -> None:
    all_search_keys: set[str] = set()
    for stage, space in STAGE_SPACES.items():
        for name, dist in space.items():
            if name not in SYSID_PARAMS:
                raise AssertionError(f"Stage {stage}: {name!r} not in SYSID_PARAMS")
            if name in FROZEN_PARAMS:
                raise AssertionError(f"Stage {stage}: {name!r} is in FROZEN_PARAMS — must not be in search space")
            if isinstance(dist, FloatDistribution):
                if dist.low >= dist.high:
                    raise AssertionError(
                        f"Stage {stage}: {name!r} bounds malformed (low={dist.low} >= high={dist.high})"
                    )
                if dist.log and (dist.low <= 0 or dist.high <= 0):
                    raise AssertionError(
                        f"Stage {stage}: {name!r} is log-distributed but has non-positive bound "
                        f"(low={dist.low}, high={dist.high})"
                    )
        if name in all_search_keys:
            raise AssertionError(f"Stage {stage}: {name!r} appears in multiple stages")
        all_search_keys |= set(space.keys())

    overlap = set(STAGE1_SPACE.keys()) & set(STAGE2_SPACE.keys())
    if overlap:
        raise AssertionError(f"STAGE1_SPACE and STAGE2_SPACE overlap: {sorted(overlap)}")


_validate_spaces()
