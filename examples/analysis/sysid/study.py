"""Optuna study driver for STD system identification.

CLI::

    python -m examples.analysis.sysid.study \\
        --bag <path> [--bag <path2> ...] --stage <name> --n-trials 500 \\
        [--base-params <yaml>] [--study-name <name>]   # default base: SYSID_PARAMS

Stage names come from ``STAGE_SPACES`` in ``search_spaces.py``.

Multi-bag: ``--bag`` is repeatable. Bags are sorted by resolved path and their
windows concatenated into one Dataset (equal-per-window weighting). With N>1
bags, ``--study-name`` is required (single-bag still auto-derives from the bag
stem). Provenance is stamped in three places: the output YAML header (one
``# bag:`` line per bag), ``study.user_attrs["bags"]`` (persists in
JournalStorage), and the wandb run config (``bags``, ``n_bags``,
``bag_window_counts``).

Re-running with the same study-name continues an existing study (Optuna's
``load_if_exists=True``). For parallelism, launch the same CLI N times in
separate shells with the same study-name + journal path; ``JournalStorage``
is race-free for concurrent workers. CMA-ES degrades past ~4 workers.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import logging
import math
import sys
from collections import Counter
from pathlib import Path

import optuna
import wandb
import yaml
from optuna.distributions import BaseDistribution, FloatDistribution
from optuna.samplers import CmaEsSampler
from optuna.storages import JournalStorage
from optuna.storages.journal import JournalFileBackend

from examples.analysis.sysid.dataset import Dataset, load_datasets
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


class Objective:
    """Optuna objective: suggest, hot-swap params, score with dataset_loss.

    On integrator divergence (``FloatingPointError`` from ``Rollout.run``) the
    trial is marked PRUNED via ``optuna.TrialPruned``. Returning ``inf`` from
    the objective trips Optuna's CMA-ES sampler (cannot update covariance from
    non-finite values) and was raising ``AssertionError: Should not reach.``
    in ``_run_trial`` — pruning is the correct signal for a degenerate trial.

    If ``wandb_run`` is provided, logs per-trial value + per-channel loss
    contributions (``contrib/<ch>`` = ``CHANNEL_COEFFS[ch] * MSE_ch``) +
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
                log_dict[f"contrib/{ch}"] = v
            for k, v in values.items():
                log_dict[f"param/{k}"] = v
            self.wandb_run.log(log_dict, step=trial.number)

        if diverged:
            raise optuna.TrialPruned("Integrator diverged on this parameter set.")
        return float(total)


def build_study(study_name: str, journal_path: Path, seed: int) -> optuna.Study:
    """Open or create a CMA-ES study backed by a JournalStorage file.

    JournalStorage is Optuna's append-only log backend, race-free for parallel
    workers (no schema, no DDL, no alembic). Re-running with the same name +
    journal_path resumes via ``load_if_exists=True``.
    """
    journal_path.parent.mkdir(parents=True, exist_ok=True)
    storage = JournalStorage(JournalFileBackend(str(journal_path)))
    return optuna.create_study(
        study_name=study_name,
        storage=storage,
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


def _load_base_params(override: str | None) -> dict:
    if override is None:
        return dict(SYSID_PARAMS)
    with Path(override).open("r") as f:
        return yaml.safe_load(f)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="python -m examples.analysis.sysid.study")
    p.add_argument(
        "--bag",
        action="append",
        required=True,
        help="Path to a 100Hz NPZ bag. Repeat for multi-bag mode (e.g. --bag A.npz --bag B.npz).",
    )
    p.add_argument("--stage", required=True, choices=tuple(STAGE_SPACES.keys()))
    p.add_argument("--n-trials", type=int, default=500)
    p.add_argument("--base-params", default=None, help="YAML to load (default: SYSID_PARAMS)")
    p.add_argument(
        "--study-name",
        default=None,
        help="Default: <bag_stem>_<stage> (single-bag only); required when multiple --bag are given.",
    )
    p.add_argument(
        "--storage", default=None, help="Path to the Optuna journal file. Default: <repo>/studies/<study_name>.journal"
    )
    p.add_argument("--out-yaml", default=None, help="Default: gymkhana/envs/params/f1tenth_std_optuna_<stage>.yaml")
    p.add_argument(
        "--mirror",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="L/R-mirror windows to symmetrize a single-handed bag (default: off).",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s: %(message)s")
    args = parse_args(argv)

    # Sort + dedup so CLI argument order doesn't change behavior (CMA-ES is seed-driven;
    # floating-point reductions aren't associative, so window order matters in the low bits).
    bags = sorted({Path(b).resolve() for b in args.bag})
    for b in bags:
        if not b.exists():
            raise SystemExit(f"Bag not found: {b}")

    if len(bags) > 1 and args.study_name is None:
        raise SystemExit("--study-name is required when multiple --bag are given")
    name = args.study_name or f"{bags[0].stem}_{args.stage}"

    journal_path = (
        Path(args.storage).resolve()
        if args.storage is not None
        else (REPO_ROOT / "studies" / f"{name}.journal").resolve()
    )
    out_yaml = (
        Path(args.out_yaml).resolve()
        if args.out_yaml
        else REPO_ROOT / "gymkhana" / "envs" / "params" / f"f1tenth_std_optuna_{args.stage}.yaml"
    )

    space = STAGE_SPACES[args.stage]
    base_params = _load_base_params(args.base_params)

    _LOG.info("Loading %d bag(s) (mirror=%s)", len(bags), args.mirror)
    for b in bags:
        _LOG.info("  bag: %s", b)
    dataset = load_datasets([str(b) for b in bags], mirror=args.mirror)
    # Each window carries source_bag (stamped by load_datasets) — group for per-bag visibility.
    bag_window_counts = {b.stem: 0 for b in bags}
    for path, count in Counter(w.source_bag for w in dataset.windows).items():
        bag_window_counts[Path(path).stem] = count
    for stem, count in bag_window_counts.items():
        _LOG.info("  bag %s: %d windows", stem, count)
    _LOG.info("Dataset: %d windows total across %d bag(s)", len(dataset.windows), len(bags))
    _LOG.info("Study: name=%s journal=%s", name, journal_path)
    study = build_study(name, journal_path, SEED)
    # Stamp provenance into the journal so the study is self-describing even
    # without the wandb run or the output YAML alongside.
    study.set_user_attr("bags", [str(b) for b in bags])
    study.set_user_attr("stage", args.stage)

    wandb_init_kwargs = dict(
        project=WANDB_PROJECT,
        group=name,  # workers in the same study share this group
        job_type=f"stage{args.stage}",
        dir=str(REPO_ROOT),  # write wandb/ at repo root, not CWD
        config={
            "bags": [b.stem for b in bags],
            "n_bags": len(bags),
            "bag_window_counts": bag_window_counts,
            "stage": args.stage,
            "n_trials": args.n_trials,
            "seed": SEED,
            "mirror": args.mirror,
            "search_space": {k: [d.low, d.high] for k, d in space.items()},
        },
    )
    with wandb.init(**wandb_init_kwargs) as wandb_run, Rollout(params=base_params) as rollout:
        objective = Objective(dataset, rollout, base_params, space, wandb_run=wandb_run)
        _LOG.info("Running %d trials (stage=%s, |space|=%d, seed=%d)", args.n_trials, args.stage, len(space), SEED)
        study.optimize(objective, n_trials=args.n_trials)
        # Objective maps divergence to TrialPruned; if every trial pruned, best_value/best_params raise.
        has_completed = any(t.state == optuna.trial.TrialState.COMPLETE for t in study.trials)
        wandb_run.summary["best_value"] = study.best_value if has_completed else None
        wandb_run.summary["best_params"] = study.best_params if has_completed else None
        wandb_run.summary["n_trials_total"] = len(study.trials)
        wandb_run.summary["all_trials_pruned"] = not has_completed

    if not has_completed:
        _LOG.error(
            "All %d trials pruned; skipping dump_best_params. Inspect journal: %s", len(study.trials), journal_path
        )
        return 1

    timestamp = _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds")
    # One `# bag:` line per bag — six-month-from-now-you finds this YAML and
    # immediately knows what produced it without needing wandb or the journal.
    bag_header_lines = "".join(f"# bag: {b}\n" for b in bags)
    # Stage 2 typically loads Stage 1's YAML — record which one so the chain is auditable.
    base_params_label = str(Path(args.base_params).resolve()) if args.base_params else "SYSID_PARAMS (env.py default)"
    header = (
        f"# Generated by examples/analysis/sysid/study.py @ {timestamp}\n"
        f"{bag_header_lines}"
        f"# stage: {args.stage}\n"
        f"# base_params: {base_params_label}\n"
        f"# study: {name} (journal: {journal_path})\n"
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
