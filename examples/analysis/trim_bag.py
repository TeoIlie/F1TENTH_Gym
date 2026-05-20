"""Permanently trim a 100Hz NPZ bag by timestep range.

Slices every array in the NPZ to [start:end+1] and overwrites the original.
The untrimmed file is moved to bags/pre-trim/<stem>.npz first as a backup
(only on the first trim — subsequent trims will refuse to overwrite the
backup so the original is never lost).

Usage:
    python examples/analysis/trim_bag.py --bag /path/to/bag.npz --start 0 --end 1500
    python examples/analysis/trim_bag.py --bag /path/to/bag.npz --end 1500          # keep start=0
    python examples/analysis/trim_bag.py --bag /path/to/bag.npz --start 200         # keep end=last
    python examples/analysis/trim_bag.py --bag /path/to/bag.npz --trim-tail 100     # drop last 100 timesteps (1s @ 100Hz)
    python examples/analysis/trim_bag.py --bag /path/to/bag.npz --trim-head 50 --trim-tail 100
"""

import argparse
import shutil
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description="Permanently trim an NPZ bag by timestep range")
    parser.add_argument("--bag", required=True, help="Path to 100Hz NPZ file (will be overwritten)")
    parser.add_argument("--start", type=int, default=None, help="Start timestep (inclusive, 0-based). Default: 0")
    parser.add_argument("--end", type=int, default=None, help="End timestep (inclusive, 0-based). Default: last")
    parser.add_argument(
        "--trim-head", type=int, default=None, help="Drop N timesteps from the start (alternative to --start)"
    )
    parser.add_argument(
        "--trim-tail", type=int, default=None, help="Drop N timesteps from the end (alternative to --end)"
    )
    parser.add_argument("--yes", action="store_true", help="Skip interactive confirmation")
    args = parser.parse_args()

    if args.trim_head is not None and args.start is not None:
        parser.error("--trim-head and --start are mutually exclusive")
    if args.trim_tail is not None and args.end is not None:
        parser.error("--trim-tail and --end are mutually exclusive")
    if args.trim_head is not None and args.trim_head < 0:
        parser.error("--trim-head must be >= 0")
    if args.trim_tail is not None and args.trim_tail < 0:
        parser.error("--trim-tail must be >= 0")

    bag_path = Path(args.bag).resolve()
    if not bag_path.is_file():
        raise FileNotFoundError(bag_path)

    data = np.load(bag_path)
    arrays = {k: data[k] for k in data.files}
    bag_len = len(arrays["t"])

    if args.trim_head is not None:
        start = args.trim_head
    else:
        start = args.start if args.start is not None else 0
    if args.trim_tail is not None:
        end = bag_len - 1 - args.trim_tail
    else:
        end = args.end if args.end is not None else bag_len - 1
    if not (0 <= start < bag_len):
        raise ValueError(f"--start must be in [0, {bag_len - 1}], got {start}")
    if not (start <= end < bag_len):
        raise ValueError(f"--end must be in [{start}, {bag_len - 1}], got {end}")
    if start == 0 and end == bag_len - 1:
        print("Nothing to trim (start=0, end=last). Exiting.")
        return

    n_dropped_head = start
    n_dropped_tail = bag_len - 1 - end
    n_kept = end - start + 1

    print(f"Bag: {bag_path}")
    print(f"Original length: {bag_len} timesteps")
    print(f"Keeping [{start}:{end}] -> {n_kept} timesteps")
    print(f"Dropping {n_dropped_head} from head, {n_dropped_tail} from tail")

    if not args.yes:
        resp = input("Proceed and overwrite original? [y/N]: ").strip().lower()
        if resp != "y":
            print("Aborted.")
            return

    # Backup before overwrite (only if no prior backup exists, to preserve true original)
    backup_dir = bag_path.parent / "pre-trim"
    backup_dir.mkdir(exist_ok=True)
    backup_path = backup_dir / bag_path.name
    if backup_path.exists():
        print(f"Backup already exists at {backup_path} — preserving original backup, not overwriting it.")
    else:
        shutil.copy2(bag_path, backup_path)
        print(f"Backup written to {backup_path}")

    sl = slice(start, end + 1)
    trimmed = {}
    for k, v in arrays.items():
        if isinstance(v, np.ndarray) and v.ndim >= 1 and len(v) == bag_len:
            trimmed[k] = v[sl]
        else:
            trimmed[k] = v  # scalar / metadata / mismatched-length array passes through untouched

    np.savez(bag_path, **trimmed)
    print(f"Wrote trimmed bag ({n_kept} timesteps) to {bag_path}")


if __name__ == "__main__":
    main()
