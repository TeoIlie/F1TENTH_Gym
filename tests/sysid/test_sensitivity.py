"""Tests for examples.analysis.sysid.sensitivity (sweep plumbing).

Three core invariants:
  1. Frozen params not in main candidate list (static guarantee).
  2. Frozen guard fires when allow_frozen=False (runtime guarantee).
  3. δ=0 row reproduces the unswept dataset_loss (catches param-mutation bugs).
"""

from __future__ import annotations

import pytest

from examples.analysis.sysid.env import SYSID_PARAMS
from examples.analysis.sysid.loss import dataset_loss
from examples.analysis.sysid.sensitivity import (
    FROZEN_PARAMS,
    STAGE1_CANDIDATES,
    VEHICLE_DYN_CANDIDATES,
    run_sweep,
)


def test_frozen_params_excluded_from_candidates():
    assert set(STAGE1_CANDIDATES) & set(FROZEN_PARAMS) == set()
    assert set(VEHICLE_DYN_CANDIDATES) & set(FROZEN_PARAMS) == set()
    assert set(STAGE1_CANDIDATES) & set(VEHICLE_DYN_CANDIDATES) == set()
    for name in (*STAGE1_CANDIDATES, *VEHICLE_DYN_CANDIDATES):
        assert name in SYSID_PARAMS, f"{name!r} not in SYSID_PARAMS"


def test_run_sweep_rejects_frozen_param_without_opt_in():
    with pytest.raises(ValueError, match="frozen"):
        run_sweep(
            rollout=None,  # never reached — guard fires first
            dataset=None,
            base_params=SYSID_PARAMS,
            candidates=("tire_p_dx3",),
            deltas=(0.0,),
        )


def test_delta_zero_reproduces_baseline(synthetic_bag_npz):
    """For every candidate, δ=0 yields the same loss as a clean dataset_loss
    call — catches accidental param mutation across iterations."""
    from examples.analysis.sysid.dataset import load_dataset
    from examples.analysis.sysid.rollout import Rollout

    ds = load_dataset(synthetic_bag_npz, mirror=False, window_length_s=1.0, stride_s=5.0)
    with Rollout() as r:
        r.set_params(SYSID_PARAMS)
        expected_total, _ = dataset_loss(r.run, ds)
        rows = run_sweep(r, ds, SYSID_PARAMS, candidates=STAGE1_CANDIDATES, deltas=(0.0,))

    assert len(rows) == len(STAGE1_CANDIDATES)
    for row in rows:
        assert row.delta == 0.0
        assert row.total == pytest.approx(expected_total, rel=1e-12, abs=1e-12)
