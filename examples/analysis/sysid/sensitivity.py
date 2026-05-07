"""STD system identification — sensitivity sweep (Phase 2 archive + re-run tool).

The Phase-2 analysis is locked (see docs/plan/OPTUNA_SYS_ID_SENSITIVITY_REPORT.md);
this module is the minimum viable tool to re-rank parameters on a new bag.

One-at-a-time perturbation of params around YAML defaults. Outputs a CSV
and prints a console ranking. Plot/markdown report generation, coverage
audit, and auto-bounds proposal were used during Phase 2 and are no longer
maintained — bounds come from the report + physical priors.

Usage::

    python -m examples.analysis.sysid.sensitivity \\
        --path examples/analysis/bags/<bag>.npz \\
        [--candidates stage12|vehicle_dyn|frozen] \\
        [--mode multiplicative|absolute] \\
        [--deltas -0.5,-0.25,0,0.25,0.5]
"""

from __future__ import annotations

import math
from copy import deepcopy
from dataclasses import dataclass
from typing import Sequence

from examples.analysis.sysid.dataset import CHANNELS, Dataset
from examples.analysis.sysid.env import SYSID_PARAMS
from examples.analysis.sysid.loss import dataset_loss
from examples.analysis.sysid.rollout import Rollout

DELTAS_DEFAULT: tuple[float, ...] = (-0.50, -0.25, -0.10, 0.0, +0.10, +0.25, +0.50)

# Stage 1/2 tire-coefficient candidates. The locked search space (in
# search_spaces.py) is a *subset* of these; the broader list is kept here so
# re-runs on a new bag can verify the demotion decisions in the report.
STAGE12_CANDIDATES: tuple[str, ...] = (
    "tire_p_cx1",
    "tire_p_dx1",
    "tire_p_ex1",
    "tire_p_kx1",
    "tire_p_cy1",
    "tire_p_dy1",
    "tire_p_ey1",
    "tire_p_ky1",
)

# Chassis / actuator params estimated rather than measured.
VEHICLE_DYN_CANDIDATES: tuple[str, ...] = ("I_z", "I_y_w", "sv_max", "sv_min", "T_steer", "a_max")

# Permanently frozen per OVERVIEW (camber-coupled + pure-shift). Routed through
# the audit gate; never promoted to the Optuna search space.
FROZEN_PARAMS: frozenset[str] = frozenset(
    {
        "tire_p_dx3",
        "tire_p_dy3",
        "tire_p_hy3",
        "tire_p_vy3",
        "tire_p_hx1",
        "tire_p_vx1",
        "tire_p_hy1",
        "tire_p_vy1",
    }
)

CANDIDATE_GROUPS: dict[str, tuple[str, ...]] = {
    "stage12": STAGE12_CANDIDATES,
    "vehicle_dyn": VEHICLE_DYN_CANDIDATES,
    "frozen": tuple(sorted(FROZEN_PARAMS)),
}


@dataclass(frozen=True)
class SweepRow:
    param: str
    delta: float
    mode: str  # "multiplicative" or "absolute"
    abs_value: float
    total: float
    per_channel: dict[str, float]


def _perturbed_params(base: dict, name: str, delta: float, mode: str) -> dict:
    p = deepcopy(base)
    if mode == "multiplicative":
        p[name] = base[name] * (1.0 + delta)
    elif mode == "absolute":
        p[name] = base[name] + delta
    else:
        raise ValueError(f"Unknown perturbation mode: {mode!r}")
    return p


def run_sweep(
    rollout: Rollout,
    dataset: Dataset,
    base_params: dict,
    candidates: Sequence[str] = STAGE12_CANDIDATES,
    deltas: Sequence[float] = DELTAS_DEFAULT,
    mode: str = "multiplicative",
    allow_frozen: bool = False,
) -> list[SweepRow]:
    """Sweep each candidate's value across `deltas`; return one row per (param, delta).

    Re-uses one Rollout (one GKEnv) — only set_params changes between iterations.
    Catches FloatingPointError from the rollout's NaN guard and records inf rows.
    Asserts the δ=0 baseline is identical across candidates (catches param-mutation bugs).
    """
    if mode not in ("multiplicative", "absolute"):
        raise ValueError(f"mode must be 'multiplicative' or 'absolute', got {mode!r}")
    for name in candidates:
        if name in FROZEN_PARAMS and not allow_frozen:
            raise ValueError(f"{name!r} is frozen — pass allow_frozen=True to audit it.")
        if name not in base_params:
            raise KeyError(f"Candidate {name!r} not in base_params")

    deltas_tuple = tuple(deltas)
    rows: list[SweepRow] = []
    baseline_total: float | None = None

    try:
        for name in candidates:
            for delta in deltas_tuple:
                params = _perturbed_params(base_params, name, delta, mode)
                rollout.set_params(params)
                try:
                    total, per_channel = dataset_loss(rollout.run, dataset)
                except FloatingPointError:
                    total = math.inf
                    per_channel = {ch: math.inf for ch in CHANNELS}

                if delta == 0.0:
                    if baseline_total is None:
                        baseline_total = total
                    elif not math.isclose(total, baseline_total, rel_tol=1e-12, abs_tol=1e-12):
                        raise AssertionError(
                            f"δ=0 baseline drift while sweeping {name!r}: got {total!r}, expected {baseline_total!r}."
                        )

                rows.append(
                    SweepRow(
                        param=name,
                        delta=float(delta),
                        mode=mode,
                        abs_value=float(params[name]),
                        total=float(total),
                        per_channel={ch: float(per_channel[ch]) for ch in CHANNELS},
                    )
                )
    finally:
        rollout.set_params(base_params)
    return rows


def rank(rows: list[SweepRow]) -> list[tuple[str, float]]:
    """Return [(param, max_abs_dtotal), ...] sorted descending. Console-friendly."""
    base = next((r for r in rows if r.delta == 0.0), None)
    if base is None:
        raise ValueError("No δ=0 row in sweep")
    by_param: dict[str, list[SweepRow]] = {}
    for r in rows:
        by_param.setdefault(r.param, []).append(r)
    out: list[tuple[str, float]] = []
    for name, entries in by_param.items():
        finite = [e.total - base.total for e in entries if math.isfinite(e.total)]
        out.append((name, max((abs(d) for d in finite), default=math.inf)))
    out.sort(key=lambda x: x[1], reverse=True)
    return out


# ----- CLI ------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    import csv
    from pathlib import Path

    from examples.analysis.sysid.dataset import load_dataset

    parser = argparse.ArgumentParser(description="Sysid sensitivity sweep")
    parser.add_argument("--path", required=True, help="Path to NPZ bag")
    parser.add_argument("--candidates", choices=tuple(CANDIDATE_GROUPS), default="stage12")
    parser.add_argument("--mode", choices=("multiplicative", "absolute"), default="multiplicative")
    parser.add_argument("--deltas", default=None, help=f"CSV deltas (default: {DELTAS_DEFAULT})")
    parser.add_argument("--out-csv", default=None, help="Default: figures/analysis/sysid/sensitivity/<bag>/sweep.csv")
    args = parser.parse_args()

    deltas = tuple(float(d) for d in args.deltas.split(",")) if args.deltas is not None else DELTAS_DEFAULT
    candidates = CANDIDATE_GROUPS[args.candidates]
    allow_frozen = args.candidates == "frozen"

    bag_path = Path(args.path).resolve()
    out_csv = (
        Path(args.out_csv)
        if args.out_csv
        else (Path("figures/analysis/sysid/sensitivity") / bag_path.stem / f"sweep_{args.candidates}.csv")
    )
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    print(f"Loading dataset {bag_path}...")
    ds = load_dataset(str(bag_path))
    print(f"  {len(ds.windows)} windows")
    print(f"Sweep: {args.candidates} × {len(deltas)} deltas = {len(candidates) * len(deltas)} evals")

    with Rollout() as r:
        rows = run_sweep(r, ds, dict(SYSID_PARAMS), candidates, deltas, mode=args.mode, allow_frozen=allow_frozen)

    with out_csv.open("w") as f:
        w = csv.writer(f)
        w.writerow(["param", "delta", "mode", "abs_value", "total", *CHANNELS])
        for r_ in rows:
            w.writerow([r_.param, r_.delta, r_.mode, r_.abs_value, r_.total, *(r_.per_channel[ch] for ch in CHANNELS)])
    print(f"Wrote {out_csv}")

    print("\nRanking by max |Δtotal|:")
    for name, dmax in rank(rows):
        print(f"  {name:18s}  Δmax = {dmax:8.4f}")
