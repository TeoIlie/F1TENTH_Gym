"""Optuna study driver for STD system identification.

CLI::

    python -m examples.analysis.sysid.study \\
        --bag <path> --stage {1,2} --n-trials 500 \\
        [--base-params <yaml>]   # required for stage 2

Re-running with the same study-name continues an existing study (Optuna's
``load_if_exists=True``). For parallelism, launch the same CLI N times in
separate shells with the same study-name + storage; SQLite handles the
inter-process locking. CMA-ES degrades past ~4 workers.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import logging
import math
import sys
from pathlib import Path

import optuna
import yaml
from optuna.distributions import BaseDistribution, FloatDistribution
from optuna.samplers import CmaEsSampler

import wandb
from examples.analysis.sysid.dataset import Dataset, load_dataset
from examples.analysis.sysid.env import SYSID_PARAMS
from examples.analysis.sysid.loss import dataset_loss
from examples.analysis.sysid.rollout import Rollout
from examples.analysis.sysid.search_spaces import STAGE_SPACES, apply_trial_params

_LOG = logging.getLogger("sysid.study")
REPO_ROOT = Path(__file__).resolve().parents[3]

# Fixed seed for reproducibility. Same seed across parallel workers is fine —
# Optuna's storage assigns distinct trial numbers atomically, so each worker
# steps the sampler from a different point.
SEED = 42

WANDB_PROJECT = "f1tenth-sysid"

# Phase-1 baseline values (YAML defaults, no mirror) for known bags. Logged to
# wandb config so the run is self-describing — gives a horizontal "what we're
# trying to beat" reference. Unknown bags log None.
PHASE1_BASELINES: dict[str, float] = {
    "circle_Apr6_100Hz": 5.02,
    "rosbag2_2026_05_04-17_54_17_100Hz": 27.4,
}


class Objective:
    """Optuna objective: suggest, hot-swap params, score with dataset_loss.

    On integrator divergence (``FloatingPointError`` from ``Rollout.run``) the
    trial is marked PRUNED via ``optuna.TrialPruned``. Returning ``inf`` from
    the objective trips Optuna's CMA-ES sampler (cannot update covariance from
    non-finite values) and was raising ``AssertionError: Should not reach.``
    in ``_run_trial`` — pruning is the correct signal for a degenerate trial.

    If ``wandb_run`` is provided, logs per-trial value + per-channel NMSE +
    suggested params keyed by trial number. Diverged trials log ``value=inf``
    so they show up in the wandb chart, but Optuna records them as PRUNED.
    """

    def __init__(
        self,
        dataset: Dataset,
        rollout: Rollout,
        base_params: dict,
        space: dict[str, BaseDistribution],
        wandb_run=None,
    ) -> None:
        self.dataset = dataset
        self.rollout = rollout
        self.base_params = base_params
        self.space = space
        self.wandb_run = wandb_run

    def __call__(self, trial: optuna.Trial) -> float:
        values: dict[str, float] = {}
        for name, dist in self.space.items():
            if not isinstance(dist, FloatDistribution):
                raise TypeError(f"Unsupported distribution {type(dist).__name__} for {name!r}")
            values[name] = trial.suggest_float(name, dist.low, dist.high, log=dist.log)

        self.rollout.set_params(apply_trial_params(self.base_params, values))
        diverged = False
        try:
            total, per_channel = dataset_loss(self.rollout.run, self.dataset)
        except FloatingPointError:
            diverged = True
            total = math.inf
            per_channel = {}

        if self.wandb_run is not None:
            log_dict: dict = {"trial": trial.number, "value": total, "diverged": int(diverged)}
            for ch, v in per_channel.items():
                log_dict[f"nmse/{ch}"] = v
            for k, v in values.items():
                log_dict[f"param/{k}"] = v
            self.wandb_run.log(log_dict, step=trial.number)

        if diverged:
            raise optuna.TrialPruned("Integrator diverged on this parameter set.")
        return float(total)


def build_study(study_name: str, storage_url: str, seed: int) -> optuna.Study:
    return optuna.create_study(
        study_name=study_name,
        storage=storage_url,
        sampler=CmaEsSampler(seed=seed),
        direction="minimize",
        load_if_exists=True,
    )


def dump_best_params(study: optuna.Study, base_params: dict, out_yaml_path: Path, header: str) -> None:
    """Write full SYSID_PARAMS dict with best_params overlaid + comment header."""
    merged = apply_trial_params(base_params, study.best_params)
    out_yaml_path.parent.mkdir(parents=True, exist_ok=True)
    with out_yaml_path.open("w") as f:
        f.write(header)
        yaml.safe_dump(merged, f, sort_keys=True)


# ----- CLI ------------------------------------------------------------------


def _load_base_params(stage: int, override: str | None) -> dict:
    if override is None:
        if stage == 2:
            raise SystemExit("--base-params is required for stage 2 (point at stage-1 best YAML)")
        return dict(SYSID_PARAMS)
    with Path(override).open("r") as f:
        return yaml.safe_load(f)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="python -m examples.analysis.sysid.study")
    p.add_argument("--bag", required=True)
    p.add_argument("--stage", type=int, required=True, choices=(1, 2))
    p.add_argument("--n-trials", type=int, default=500)
    p.add_argument("--base-params", default=None, help="YAML to load (required for stage 2)")
    p.add_argument("--study-name", default=None, help="Default: <bag_stem>_stage{N}")
    p.add_argument("--storage", default=None, help="Default: sqlite:///<repo>/studies/<study_name>.db")
    p.add_argument("--out-yaml", default=None, help="Default: gymkhana/envs/params/f1tenth_std_optuna_stage{N}.yaml")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s: %(message)s")
    args = parse_args(argv)

    bag = Path(args.bag).resolve()
    if not bag.exists():
        raise SystemExit(f"Bag not found: {bag}")

    name = args.study_name or f"{bag.stem}_stage{args.stage}"
    if args.storage is not None:
        storage = args.storage
    else:
        db = (REPO_ROOT / "studies" / f"{name}.db").resolve()
        db.parent.mkdir(parents=True, exist_ok=True)
        storage = f"sqlite:///{db}"
    out_yaml = (
        Path(args.out_yaml).resolve()
        if args.out_yaml
        else REPO_ROOT / "gymkhana" / "envs" / "params" / f"f1tenth_std_optuna_stage{args.stage}.yaml"
    )

    space = STAGE_SPACES[args.stage]
    base_params = _load_base_params(args.stage, args.base_params)

    _LOG.info("Loading dataset from %s", bag)
    dataset = load_dataset(str(bag))
    _LOG.info("Dataset: %d windows", len(dataset.windows))
    _LOG.info("Study: name=%s storage=%s", name, storage)
    study = build_study(name, storage, SEED)

    wandb_init_kwargs = dict(
        project=WANDB_PROJECT,
        group=name,  # workers in the same study share this group
        job_type=f"stage{args.stage}",
        config={
            "bag": bag.stem,
            "stage": args.stage,
            "n_trials": args.n_trials,
            "seed": SEED,
            "search_space": {k: [d.low, d.high] for k, d in space.items()},
            "baseline_phase1": PHASE1_BASELINES.get(bag.stem),
        },
    )
    with wandb.init(**wandb_init_kwargs) as wandb_run, Rollout(params=base_params) as rollout:
        objective = Objective(dataset, rollout, base_params, space, wandb_run=wandb_run)
        _LOG.info("Running %d trials (stage=%d, |space|=%d, seed=%d)", args.n_trials, args.stage, len(space), SEED)
        study.optimize(objective, n_trials=args.n_trials)
        wandb_run.summary["best_value"] = study.best_value
        wandb_run.summary["best_params"] = study.best_params
        wandb_run.summary["n_trials_total"] = len(study.trials)

    timestamp = _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds")
    header = (
        f"# Generated by examples/analysis/sysid/study.py @ {timestamp}\n"
        f"# bag: {bag}\n"
        f"# stage: {args.stage}\n"
        f"# study: {name} ({storage})\n"
        f"# seed: {SEED}\n"
        f"# n_trials: {len(study.trials)}\n"
        f"# best_value: {study.best_value}\n"
        f"# best_params: {study.best_params}\n\n"
    )
    dump_best_params(study, base_params, out_yaml, header)
    _LOG.info("Best value: %.6f", study.best_value)
    _LOG.info("Wrote %s", out_yaml)
    return 0


if __name__ == "__main__":
    sys.exit(main())
