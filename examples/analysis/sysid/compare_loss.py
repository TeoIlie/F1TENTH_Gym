"""Compare `dataset_loss` between two STD parameter sets on a single bag.

Sanity-checks that the loss actually distinguishes a known-better fit from a
known-worse one. `Rollout.set_params` hot-swaps params on the same env, so
both evaluations share an identical reset/step path.
"""

from __future__ import annotations

import argparse

from examples.analysis.sysid.dataset import load_dataset
from examples.analysis.sysid.env import SYSID_PARAMS
from examples.analysis.sysid.loss import CHANNEL_COEFFS, dataset_loss
from examples.analysis.sysid.rollout import Rollout

# Override the locked study coefficients here. Each coefficient bundles channel
# importance with unit-scaling, so contrib = coeff * MSE is directly comparable
# across channels.
COEFFS: dict[str, float] = {
    **CHANNEL_COEFFS,
}

# Edit these two dicts with your empirical "good" and "bad" parameter sets.
# Anything omitted falls back to SYSID_PARAMS (i.e. f1tenth_std.yaml).
PARAMS_GOOD: dict = {
    **SYSID_PARAMS,
}

PARAMS_BAD: dict = {**SYSID_PARAMS, "I_y_w": 0.00017}


def _print_result(label: str, total: float, per_channel: dict[str, float]) -> None:
    print(f"\n[{label}] total = {total:.6f}")
    for ch, contrib in per_channel.items():
        c = COEFFS.get(ch, 0.0)
        print(f"  {ch:>9s}: contrib={contrib:.6f}  coeff={c:.4f}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare dataset_loss between two STD param sets")
    parser.add_argument("--path", required=True, help="Path to 100Hz NPZ bag")
    parser.add_argument("--window-length-s", type=float, default=1.5)
    parser.add_argument("--stride-s", type=float, default=0.5)
    parser.add_argument("--min-speed", type=float, default=0.3)
    parser.add_argument("--warmup-s", type=float, default=0.2)
    parser.add_argument("--mirror", action="store_true", help="Include mirrored windows")
    args = parser.parse_args()

    ds = load_dataset(
        args.path,
        window_length_s=args.window_length_s,
        stride_s=args.stride_s,
        min_speed=args.min_speed,
        mirror=args.mirror,
    )
    print(f"loaded {len(ds.windows)} windows from {args.path} (dt={ds.dt})")

    with Rollout(params=PARAMS_GOOD) as rollout:
        total_good, per_good = dataset_loss(rollout.run, ds, coeffs=COEFFS, warmup_s=args.warmup_s)
        rollout.set_params(PARAMS_BAD)
        total_bad, per_bad = dataset_loss(rollout.run, ds, coeffs=COEFFS, warmup_s=args.warmup_s)

    _print_result("GOOD", total_good, per_good)
    _print_result("BAD ", total_bad, per_bad)

    delta = total_bad - total_good
    verdict = "OK (bad > good)" if delta > 0 else "UNEXPECTED (bad <= good)"
    print(f"\nΔ(bad - good) = {delta:+.6f}  →  {verdict}")


if __name__ == "__main__":
    main()
