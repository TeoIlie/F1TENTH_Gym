"""Tests for examples.analysis.sysid.study (Phase 3).

Smoke tests:
  1. 2-trial study runs end-to-end on a synthetic bag; journal file persists.
  2. Diverged trial maps to PRUNED state (not COMPLETE-with-inf).
  3. dump_best_params YAML round-trip preserves SYSID_PARAMS shape.
  4. Enqueue equivalence: midpoint trial matches direct dataset_loss call.
"""

from __future__ import annotations

import math
from unittest.mock import patch

import optuna
import pytest
import yaml

from examples.analysis.sysid.env import SYSID_PARAMS
from examples.analysis.sysid.rollout import Rollout
from examples.analysis.sysid.search_spaces import SYMMETRIC_PAIRS, VEHICLE_DYN
from examples.analysis.sysid.study import Objective, build_study, dump_best_params


@pytest.fixture(scope="module")
def dataset(synthetic_dataset):
    return synthetic_dataset


@pytest.fixture(scope="module")
def rollout():
    with Rollout() as r:
        yield r


# ---------- smoke: 2-trial study runs and persists ----------


def test_study_smoke_runs_and_persists(tmp_path, dataset, rollout):
    journal_path = tmp_path / "smoke.journal"
    study = build_study("smoke_test", journal_path, seed=42)

    obj = Objective(dataset, rollout, dict(SYSID_PARAMS), VEHICLE_DYN)
    study.optimize(obj, n_trials=2)

    assert journal_path.exists() and journal_path.stat().st_size > 0
    assert len(study.trials) == 2
    assert math.isfinite(study.best_value)
    # Sanity: with this small a budget, best_value won't necessarily beat
    # the YAML default; we just need it to be a real number, not inf/nan.


# ---------- diverged trial → state PRUNED (avoids Optuna's "Should not reach" assert) ----------


def test_diverged_trial_is_pruned(tmp_path, dataset, rollout):
    """Returning inf from the objective trips Optuna's CMA-ES sampler with
    "AssertionError: Should not reach." The objective must instead raise
    optuna.TrialPruned on integrator divergence.
    """
    journal_path = tmp_path / "div.journal"
    study = build_study("div_test", journal_path, seed=0)

    obj = Objective(dataset, rollout, dict(SYSID_PARAMS), VEHICLE_DYN)

    # Patch Rollout.run to raise FloatingPointError every call.
    with patch.object(Rollout, "run", side_effect=FloatingPointError("synthetic divergence")):
        study.optimize(obj, n_trials=1)

    trial = study.trials[0]
    assert trial.state == optuna.trial.TrialState.PRUNED, (
        f"Expected PRUNED (objective raises TrialPruned on divergence), got {trial.state}"
    )
    assert trial.value is None


# ---------- dump_best_params round-trip ----------


def test_dump_best_params_round_trip(tmp_path, dataset, rollout):
    journal_path = tmp_path / "dump.journal"
    study = build_study("dump_test", journal_path, seed=7)

    obj = Objective(dataset, rollout, dict(SYSID_PARAMS), VEHICLE_DYN)
    study.optimize(obj, n_trials=2)

    out_yaml = tmp_path / "out.yaml"
    dump_best_params(study, dict(SYSID_PARAMS), out_yaml, header="# test\n\n")

    assert out_yaml.exists()
    with out_yaml.open("r") as f:
        loaded = yaml.safe_load(f)

    # All SYSID_PARAMS keys present.
    assert set(loaded.keys()) == set(SYSID_PARAMS.keys())
    # Search-space keys equal study.best_params.
    for k, v in study.best_params.items():
        assert loaded[k] == pytest.approx(v)
    # Symmetric-pair partners are derived (negated) from their search-space key,
    # not preserved from SYSID_PARAMS. Verify the symmetry constraint holds.
    for src, partner in SYMMETRIC_PAIRS.items():
        if src in study.best_params:
            assert loaded[partner] == pytest.approx(-study.best_params[src])
    # Remaining (truly untouched) keys equal SYSID_PARAMS.
    overlaid = set(VEHICLE_DYN) | {p for s, p in SYMMETRIC_PAIRS.items() if s in study.best_params}
    for k, v in SYSID_PARAMS.items():
        if k not in overlaid:
            assert loaded[k] == v


# ---------- enqueue equivalence (catches silent-noop bugs in apply_trial_params / set_params) ----------


def test_enqueue_trial_matches_direct_rollout(tmp_path, dataset, rollout):
    """Enqueue a trial at a known in-bounds point; the returned loss must
    match what a direct ``dataset_loss`` call gives for the same param dict.
    Catches bugs where apply_trial_params or set_params silently no-op
    (e.g. wrong key, value not propagated to env).

    NOTE: cannot use YAML defaults — Phase-2 deliberately set the vehicle_dyn
    lower bound for ``I_z`` to 0.05 (above the YAML default of 0.047)
    because the default is wrong. Pick a point inside every search bound.
    """
    from examples.analysis.sysid.loss import dataset_loss
    from examples.analysis.sysid.search_spaces import apply_trial_params

    # Midpoint of every vehicle_dyn bound — guaranteed in-bounds.
    point = {name: 0.5 * (dist.low + dist.high) for name, dist in VEHICLE_DYN.items()}

    # Direct path: overlay onto SYSID_PARAMS, set on env, run dataset_loss.
    direct_params = apply_trial_params(dict(SYSID_PARAMS), point)
    rollout.set_params(direct_params)
    direct_loss, _ = dataset_loss(rollout.run, dataset)

    # Optuna path: enqueue same point, run one trial through Objective.
    journal_path = tmp_path / "enqueue.journal"
    study = build_study("enqueue_test", journal_path, seed=0)
    study.enqueue_trial(point)
    obj = Objective(dataset, rollout, dict(SYSID_PARAMS), VEHICLE_DYN)
    study.optimize(obj, n_trials=1)

    assert study.trials[0].value == pytest.approx(direct_loss, rel=1e-6, abs=1e-9), (
        f"Enqueued trial value {study.trials[0].value} differs from direct "
        f"dataset_loss {direct_loss} — apply_trial_params or set_params likely no-opped."
    )


# ---------- multi-bag CLI contract: --study-name required when N>1 ----------


def test_main_requires_study_name_for_multi_bag(tmp_path, synthetic_bag_npz):
    """Multi-bag invocations must provide --study-name explicitly — auto-naming
    from a single bag stem is undefined for N>1. The error must surface BEFORE
    any heavy work (dataset load, wandb init, study build); we rely on this so
    misconfigured invocations fail fast.
    """
    from .conftest import make_npz

    second = tmp_path / "second.npz"
    make_npz(str(second), n=1000, speed=2.0, vy=0.1, yaw_rate=0.3, steer=0.15)

    from examples.analysis.sysid.study import main

    with pytest.raises(SystemExit, match="study-name is required"):
        main(
            [
                "--bag",
                synthetic_bag_npz,
                "--bag",
                str(second),
                "--stage",
                "vehicle_dyn",
                "--n-trials",
                "1",
            ]
        )
