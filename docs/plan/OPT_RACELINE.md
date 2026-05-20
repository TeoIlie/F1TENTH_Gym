# Plan: `extract_raceline.py` for the gymkhana maps repo

## Context

The gymkhana simulator (parent of the `maps/` repo) needs a per-map `<MAP>_raceline.csv` file containing an optimized racing line. Today the repo has `extract_centerline.py`, which produces `<MAP>_centerline.csv` (`# x_m, y_m, w_tr_right_m, w_tr_left_m`) from the map PNG + YAML. The next step is to feed that centerline through the same trajectory optimizer that the ForzaETH race_stack uses (TUM's `global_racetrajectory_optimization`, ForzaETH fork) and write a raceline CSV in the format the simulator consumes.

The optimizer is already proven in the race_stack (`race_stack/planner/gb_optimizer/src/global_planner_node.py:529` calls `trajectory_optimizer(curv_opt_type='mincurv_iqp', safety_width=…)`). We will reuse it standalone, with no ROS dependency.

This plan should be executable from inside the **maps repo** (the one whose `extract_centerline.py` lives at the repo root). It does NOT depend on the race_stack repo being checked out; everything needed from race_stack (one `.ini`, two CSVs) is vendored as part of this plan, with the contents inlined below for copy-paste.

---

## Decisions already made

| # | Decision | Choice |
|---|---|---|
| 1 | Optimizer source | Submodule `https://github.com/ForzaETH/global_racetrajectory_optimization.git` into the maps repo |
| 2 | Vehicle params | Vendor `racecar_f110.ini` + `veh_dyn_info/{ggv.csv,ax_max_machines.csv}` from `race_stack/stack_master/config/gb_optimizer/` into the maps repo |
| 3 | Optimizer type | `mincurv_iqp` only, default safety widths |
| 4 | Output format | `<MAP>_raceline.csv` with header `# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2` (semicolon-separated). For now: `vx_mps=8.0`, `ax_mps2=0.0` constants; other 5 cols come from the optimizer |
| 5 | Dependencies | Add `trajectory-planning-helpers` and `quadprog` to `pyproject.toml` `[tool.poetry.dependencies]`. No CasADi/IPOPT (not needed without `mintime`) |
| 6 | Coupling style | Use the ForzaETH fork **as-is**; don't fork it. Hide the optimizer's CWD/output quirks inside our wrapper script |
| 7 | Submodule scope | Submodule lives in the **maps repo** (not in the race_stack) |

---

## Repo layout after this change

(All paths relative to the **maps repo root** — i.e. the directory containing the existing `extract_centerline.py`.)

```
.gitmodules                              # new
extract_centerline.py                    # existing, untouched
extract_raceline.py                      # NEW
global_racetrajectory_optimization/      # NEW git submodule (ForzaETH fork)
raceline_config/                         # NEW vendored from race_stack
  racecar_f110.ini
  veh_dyn_info/
    ggv.csv
    ax_max_machines.csv
<MAP>/<MAP>.png                          # existing
<MAP>/<MAP>_map.yaml                     # existing
<MAP>/<MAP>_centerline.csv               # existing (output of extract_centerline.py)
<MAP>/<MAP>_raceline.csv                 # NEW (output of extract_raceline.py)
<MAP>/generation/raceline_final.png      # NEW (when --visualize)
```

Parent gymkhana repo (one level up) has its `pyproject.toml` updated with the two new deps.

---

## Step-by-step implementation

### Step 1 — Add the optimizer as a git submodule

From the maps repo root:

```bash
git submodule add https://github.com/ForzaETH/global_racetrajectory_optimization.git global_racetrajectory_optimization
git submodule update --init --recursive
```

This produces `.gitmodules` and the `global_racetrajectory_optimization/` directory.

Update the maps repo `README.md` (if any) with: *"After cloning, run `git submodule update --init --recursive` before using `extract_raceline.py`."*

### Step 2 — Vendor the vehicle config

Create `raceline_config/` and copy three files **verbatim** from the race_stack. The reference paths in the race_stack repo are:

- `race_stack/stack_master/config/gb_optimizer/racecar_f110.ini` (282 lines)
- `race_stack/stack_master/config/gb_optimizer/veh_dyn_info/ggv.csv`
- `race_stack/stack_master/config/gb_optimizer/veh_dyn_info/ax_max_machines.csv`

If the race_stack repo isn't accessible, the same three files exist inside the ForzaETH submodule under `global_racetrajectory_optimization/inputs/veh_dyn_info/` and `global_racetrajectory_optimization/params/racecar.ini` — but **prefer the race_stack copies** because they are tuned for the F1TENTH platform (the upstream `racecar.ini` is for a full-size vehicle).

The first ~10 lines of the F1TENTH `.ini` look like:
```ini
[GENERAL_OPTIONS]
ggv_file="ggv.csv"
ax_max_machines_file="ax_max_machines.csv"

stepsize_opts={"stepsize_prep": 0.05,
               "stepsize_reg": 0.2,
               "stepsize_interp_after_opt": 0.1}

reg_smooth_opts={"k_reg": 3,
                 "s_reg": 1}
…
```

No edits needed to any of the three files — copy as-is.

### Step 3 — Add Python dependencies

Edit the **parent gymkhana repo's `pyproject.toml`**, in `[tool.poetry.dependencies]`, after the existing deps:

```toml
trajectory-planning-helpers = "^0.78"
quadprog = "^0.1.11"
```

(Pin loosely; the optimizer's own `requirements.txt` inside the submodule is the source of truth — check it after `git submodule update` and tighten the pins if needed.)

Then `poetry lock && poetry install`.

### Step 4 — Write `extract_raceline.py`

This is the main deliverable. Place at the maps repo root (alongside `extract_centerline.py`).

Key facts about the optimizer's `trajectory_optimizer()` (verified by reading `global_racetrajectory_optimization/trajectory_optimizer.py`):

- Signature: `trajectory_optimizer(input_path: str, track_name: str, curv_opt_type: str, safety_width: float = 0.8, plot: bool = False)`.
- Returns: `(traj, bound_r, bound_l, est_lap_time)` where `traj` columns are `[s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2]`.
- **Quirk 1**: `track_name` is resolved as `<track_name>.csv` **relative to the current working directory** (line 112 of `trajectory_optimizer.py`: `file_paths["track_file"] = os.path.join(file_paths["track_name"] + ".csv")` — `input_path` is NOT prefixed).
- **Quirk 2**: `input_path` is used to find `racecar_f110.ini` and the `veh_dyn_info/` subdirectory.
- **Quirk 3**: Hardcoded vehicle params filename `racecar_f110.ini` (line 57). Must match exactly.
- **Quirk 4**: With `plot=True` it pops up matplotlib windows. Pass `plot=False` and do our own visualization.
- **Quirk 5**: It writes `outputs/traj_race_cl.csv` inside the optimizer module dir. Harmless — we ignore that file and use the in-memory return.
- **Quirk 6**: Calls `helper_funcs_glob.src.import_track.import_track`, which expects the centerline CSV to be a **closed loop** with format `# x_m, y_m, w_tr_right_m, w_tr_left_m` — same format that `extract_centerline.py` already produces. ✓

The wrapper handles quirks 1, 2, 4, 5 by `chdir`ing into a tempdir, copying the centerline in under the expected name, and capturing the in-memory return.

```python
#!/usr/bin/env python3
"""
Generate an optimized racing line for a track using ForzaETH's fork of TUM's
global_racetrajectory_optimization (mincurv_iqp).

Prerequisite:
    python3 extract_centerline.py --map <MAP_NAME>

Usage:
    python3 extract_raceline.py --map <MAP_NAME> [--safety-width 0.8] [--visualize]
"""

import argparse
import os
import shutil
import sys
import tempfile
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

# Make the submodule importable
SCRIPT_DIR = Path(__file__).parent.resolve()
OPT_DIR = SCRIPT_DIR / "global_racetrajectory_optimization"
sys.path.insert(0, str(OPT_DIR.parent))  # so `import global_racetrajectory_optimization…` resolves
sys.path.insert(0, str(OPT_DIR))         # fallback for siblings

from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer  # noqa: E402

DEFAULT_VX_MPS = 8.0   # constant for now; future work: real velocity profile
DEFAULT_AX_MPS2 = 0.0
DEFAULT_SAFETY_WIDTH = 0.8  # matches race_stack default for the main raceline
RACELINE_CONFIG_DIR = SCRIPT_DIR / "raceline_config"


def run_optimizer(centerline_csv: Path, safety_width: float) -> np.ndarray:
    """
    Run mincurv_iqp via trajectory_optimizer().

    Returns traj with columns: s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2.
    """
    if not RACELINE_CONFIG_DIR.is_dir():
        raise FileNotFoundError(f"Vehicle config dir missing: {RACELINE_CONFIG_DIR}")
    if not (RACELINE_CONFIG_DIR / "racecar_f110.ini").exists():
        raise FileNotFoundError("raceline_config/racecar_f110.ini not found")

    # trajectory_optimizer() reads <track_name>.csv from CWD, so stage in a tempdir.
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        staged = tmp / "track_centerline.csv"
        shutil.copy(centerline_csv, staged)

        cwd_before = Path.cwd()
        try:
            os.chdir(tmp)
            traj, _bound_r, _bound_l, est_t = trajectory_optimizer(
                input_path=str(RACELINE_CONFIG_DIR),
                track_name="track_centerline",
                curv_opt_type="mincurv_iqp",
                safety_width=safety_width,
                plot=False,
            )
        finally:
            os.chdir(cwd_before)

    print(f"Estimated (uniform-speed) lap time placeholder from optimizer: {est_t:.3f} s")
    return traj


def write_raceline_csv(traj: np.ndarray, output_path: Path) -> None:
    """
    Write the raceline in gymkhana's expected format:
        header: '# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2'
        rows : semicolon-separated; vx and ax overridden to constants.
    """
    out = traj.copy()
    out[:, 5] = DEFAULT_VX_MPS
    out[:, 6] = DEFAULT_AX_MPS2

    with open(output_path, "w") as f:
        f.write("# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2\n")
        for row in out:
            f.write(
                f"{row[0]:.4f}; {row[1]:.4f}; {row[2]:.4f}; "
                f"{row[3]:.6f}; {row[4]:.6f}; {row[5]:.4f}; {row[6]:.4f}\n"
            )
    print(f"Wrote {len(out)} waypoints to {output_path}")


def visualize(map_dir: Path, map_name: str, traj: np.ndarray) -> None:
    """Overlay the raceline on the track PNG, similar to extract_centerline.py."""
    img_path = map_dir / f"{map_name}.png"
    yaml_path = map_dir / f"{map_name}_map.yaml"
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    resolution = meta["resolution"]
    origin = meta["origin"]
    img = np.array(Image.open(img_path).convert("L"))
    h = img.shape[0]

    # world (x, y) -> pixel (x, y) inverse of extract_centerline.convert_to_world_coordinates
    px_x = (traj[:, 1] - origin[0]) / resolution
    px_y = (h - 1) - (traj[:, 2] - origin[1]) / resolution

    out_dir = map_dir / "generation"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / "raceline_final.png"

    fig, ax = plt.subplots(figsize=(10, 12))
    ax.imshow(img, cmap="gray")
    ax.plot(px_x, px_y, "r-", linewidth=2, label="Raceline (mincurv_iqp)")
    ax.scatter(px_x[0], px_y[0], c="green", s=100, marker="o", zorder=10, label="Start")
    ax.set_title(f"{map_name} raceline ({len(traj)} waypoints)")
    ax.legend()
    ax.axis("equal")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved visualization to {out_path}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--map", required=True, help="Map name (subfolder of this directory)")
    p.add_argument("--safety-width", type=float, default=DEFAULT_SAFETY_WIDTH)
    p.add_argument("--visualize", action="store_true")
    args = p.parse_args()

    map_dir = SCRIPT_DIR / args.map
    centerline_csv = map_dir / f"{args.map}_centerline.csv"
    if not centerline_csv.exists():
        sys.exit(
            f"ERROR: {centerline_csv} not found. "
            f"Run `python3 extract_centerline.py --map {args.map}` first."
        )

    traj = run_optimizer(centerline_csv, args.safety_width)
    output_path = map_dir / f"{args.map}_raceline.csv"
    write_raceline_csv(traj, output_path)

    if args.visualize:
        visualize(map_dir, args.map, traj)


if __name__ == "__main__":
    main()
```

### Step 5 — Verification

End-to-end on `Drift` (a known-working map with an existing centerline):

```bash
# From maps repo root
git submodule update --init --recursive
poetry install   # from parent gymkhana repo

# Centerline already exists for Drift; if not, run extract first:
python3 extract_centerline.py --map Drift

# Generate the raceline
python3 extract_raceline.py --map Drift --visualize
```

Expected outputs:
- `Drift/Drift_raceline.csv` exists, ~same number of rows as the centerline (likely a bit different due to optimizer resampling).
- First line is exactly: `# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2`.
- Each data row has 7 semicolon-separated floats.
- Column 6 (vx_mps) is uniformly `8.0000`; column 7 (ax_mps2) is uniformly `0.0000`.
- `Drift/generation/raceline_final.png` shows a red curve clearly *inside* the track that cuts the apex of corners (visibly different from the centerline).
- Console prints an "estimated lap time" and final waypoint count.

Spot-check a second map (e.g. `Drift`) to confirm it generalizes.

Sanity assertions worth eyeballing in the CSV:
- `s_m` is monotonically increasing from 0.
- `x_m, y_m` stay within the bounds shown by `extract_centerline.py`'s logged "World coordinate range".
- `kappa_radpm` magnitudes are larger than the centerline's curvature in the corners (raceline cuts tighter) and smaller on straights.

### Step 6 — Commit

```bash
git add .gitmodules global_racetrajectory_optimization raceline_config extract_raceline.py
git commit -m "Add raceline extraction via ForzaETH global_racetrajectory_optimization submodule"
```

And in the parent gymkhana repo, commit the `pyproject.toml` change with the two added deps.

---

## Future-iteration notes (out of scope here, but worth recording)

- `vx_mps` and `ax_mps2` are placeholders. The optimizer already returns a real velocity/acceleration profile in columns 5 and 6 of `traj`; switching from constants to the optimizer's values is a one-line change inside `write_raceline_csv` (delete the two `out[:, 5] = …` / `out[:, 6] = …` overrides). Worth doing once the simulator is ready to consume a real profile.
- The race_stack also computes a *shortest-path* line for overtaking with a tighter `safety_width`. If gymkhana ever needs a second line, expose `--curv-opt-type {mincurv_iqp,shortest_path}` and re-run.
- `mintime` (true lap-time-optimal) is a one-arg change but adds CasADi + IPOPT — defer until needed.
- The hardcoded `racecar_f110.ini` filename is a quirk of the upstream `trajectory_optimizer()` function. If we ever want per-vehicle configs, we'd patch the submodule (now would be the time to fork it) — but right now there's only one F1TENTH config to worry about.
