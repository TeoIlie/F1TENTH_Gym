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

STEER_LAG: dict[str, BaseDistribution] = {
    "sv_max": FloatDistribution(low=3.0, high=7.0),  # nominal 4.95 — sv_min mirrored to -sv_max via SYMMETRIC_PAIRS
    "T_steer": FloatDistribution(low=0.005, high=0.10, log=True),  # nominal 0.025
}

VEHICLE_DYN: dict[str, BaseDistribution] = {
    "I_z": FloatDistribution(low=0.01, high=0.20),  # nominal 0.04712
    "I_y_w": FloatDistribution(low=0.0001, high=0.005),  # nominal 0.0017
    "h_s": FloatDistribution(low=0.02, high=0.09),  # nominal 0.074
    "R_w": FloatDistribution(low=0.04, high=0.06),  # nominal 0.049
}

STAGE1: dict[str, BaseDistribution] = {
    "a_max": FloatDistribution(low=5.0, high=8.0),  # nominal 6.0
    "tire_p_dx1": FloatDistribution(low=0.1, high=1.5),  # nominal 1.1739
    "tire_p_kx1": FloatDistribution(low=5.0, high=30.0),  # nominal 22.303
    "tire_p_dy1": FloatDistribution(low=0.1, high=1.5),  # nominal 1.0489
    "tire_p_ky1": FloatDistribution(low=-30.0, high=-5.0),  # nominal -21.92
}

STAGE2: dict[str, BaseDistribution] = {
    "tire_p_cx1": FloatDistribution(low=1.0, high=1.8),  # nominal 1.6411
    "tire_p_ex1": FloatDistribution(low=-2.1, high=0.99),  # nominal 0.46403
    "tire_p_cy1": FloatDistribution(low=1.0, high=1.8),  # nominal 1.3507
    "tire_p_ey1": FloatDistribution(low=-2.1, high=0.99),  # nominal -0.0074722
}

STAGE3: dict[str, BaseDistribution] = {
    # longitudinal combined - Fx reduction due to lateral slip α
    "tire_r_bx1": FloatDistribution(low=5.0, high=25.0),  # nominal 13.276
    "tire_r_bx2": FloatDistribution(low=-25.0, high=-5.0),  # nominal -13.778
    # lateral combined - Fy reduction due to longitudinal slip κ
    "tire_r_by1": FloatDistribution(low=3.0, high=15.0),  # nominal 7.1433
    "tire_r_by2": FloatDistribution(low=3.0, high=18.0),  # nominal 9.1916
}

FINE_TUNE: dict[str, BaseDistribution] = {
    # combined-slip shape & curvature
    "tire_r_cx1": FloatDistribution(low=0.8, high=2.0),  # nominal 1.2568
    "tire_r_ex1": FloatDistribution(low=-1.0, high=0.99),  # nominal 0.65225
    "tire_r_cy1": FloatDistribution(low=0.8, high=1.5),  # nominal 1.0719
    "tire_r_ey1": FloatDistribution(low=-1.0, high=0.99),  # nominal -0.27572
    # additional lateral combined-slip terms
    "tire_r_by3": FloatDistribution(low=-0.5, high=0.5),  # nominal -0.027856
    # κ-induced side force Svyk
    "tire_r_vy1": FloatDistribution(low=-0.2, high=0.2),  # nominal -0.027825
    "tire_r_vy5": FloatDistribution(low=0.5, high=4.0),  # nominal 1.9
    "tire_r_vy4": FloatDistribution(low=5.0, high=20.0),  # nominal 12.12
    "tire_r_vy6": FloatDistribution(low=-20.0, high=-5.0),  # nominal -10.704
}


STAGE_SPACES: dict[str, dict[str, BaseDistribution]] = {
    "steer_lag": STEER_LAG,
    "vehicle_dyn": VEHICLE_DYN,
    "stage1": STAGE1,
    "stage2": STAGE2,
    "stage3": STAGE3,
    "fine_tune": FINE_TUNE,
}


# Keys whose sampled value is negated and written into a partner key, so CMA-ES
# can search a single dimension while a physical symmetry constraint
# (sv_min == -sv_max) is enforced exactly on every trial.
SYMMETRIC_PAIRS: dict[str, str] = {"sv_max": "sv_min"}


def apply_trial_params(base_params: dict, trial_values: dict[str, float]) -> dict:
    """Return ``deepcopy(base_params)`` with ``trial_values`` overlaid.

    Non-mutating: the input ``base_params`` is never modified.
    """
    out = deepcopy(base_params)
    for name, value in trial_values.items():
        out[name] = value
        partner = SYMMETRIC_PAIRS.get(name)
        if partner is not None:
            out[partner] = -value
    return out


# --- Import-time invariants -------------------------------------------------
# Fail loudly if a search space drifts out of sync with SYSID_PARAMS / freeze
# decisions. Cheap (runs once on first import) and prevents silent bugs where
# Optuna would suggest a value for a key that doesn't exist in the params dict.
def _validate_spaces() -> None:
    # Cross-stage key overlap is intentionally allowed: a param may appear in
    # multiple stages (e.g. coarse fit in one, refinement in another). Running
    # a later stage will re-search and overwrite the earlier stage's value —
    # caller is responsible for stage ordering.
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


_validate_spaces()
