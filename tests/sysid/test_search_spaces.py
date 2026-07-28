"""Tests for examples.analysis.sysid.search_spaces (Phase 3).

Pure-function invariants — fast, no env construction.
"""

from __future__ import annotations

from copy import deepcopy

from optuna.distributions import FloatDistribution

from examples.analysis.sysid.env import SYSID_PARAMS
from examples.analysis.sysid.search_spaces import (
    FINE_TUNE,
    STAGE_SPACES,
    STEER_LAG,
    SYMMETRIC_PAIRS,
    VEHICLE_DYN,
    apply_trial_params,
)
from examples.analysis.sysid.sensitivity import FROZEN_PARAMS


def test_search_keys_exist_in_sysid_params():
    for stage, space in STAGE_SPACES.items():
        for name in space:
            assert name in SYSID_PARAMS, f"Stage {stage}: {name!r} missing from SYSID_PARAMS"


def test_stages_disjoint():
    assert set(VEHICLE_DYN) & set(FINE_TUNE) == set()


def test_stages_disjoint_with_frozen():
    for stage, space in STAGE_SPACES.items():
        overlap = set(space) & set(FROZEN_PARAMS)
        assert overlap == set(), f"Stage {stage} overlaps frozen: {overlap}"


def test_bounds_well_formed():
    for stage, space in STAGE_SPACES.items():
        for name, dist in space.items():
            assert isinstance(dist, FloatDistribution), f"{name} unexpected dist type"
            assert dist.low < dist.high, f"Stage {stage} {name}: low={dist.low} high={dist.high}"
            if dist.log:
                assert dist.low > 0 and dist.high > 0, f"Stage {stage} {name}: log distribution with non-positive bound"


def test_apply_trial_params_non_mutating():
    base = {"a": 1.0, "b": 2.0, "nested": [1, 2, 3]}
    snapshot = deepcopy(base)
    out = apply_trial_params(base, {"a": 10.0})
    assert base == snapshot, "input dict was mutated"
    assert out["a"] == 10.0
    assert out["b"] == 2.0
    # Confirm deepcopy: mutating nested in output must not touch input.
    out["nested"].append(99)
    assert base["nested"] == [1, 2, 3]


def test_apply_trial_params_overlays_full_dict():
    base = dict(SYSID_PARAMS)
    trial = {name: 0.5 for name in (*VEHICLE_DYN, *STEER_LAG)}
    out = apply_trial_params(base, trial)
    # All non-search-space keys preserved, except symmetric partners which mirror their pair.
    for k, v in SYSID_PARAMS.items():
        if k in trial:
            assert out[k] == 0.5
        elif k in SYMMETRIC_PAIRS.values():
            assert out[k] == -0.5
        else:
            assert out[k] == v
