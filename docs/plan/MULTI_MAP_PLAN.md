# Multi-Map Training Implementation Plan

## Overview
Enable the F1TENTH Gym to train reinforcement learning agents across multiple tracks simultaneously, improving policy generalization beyond single-map training.

## Background Context

### Current Limitation
- Track loaded once at environment initialization via `Track.from_track_name()` in `f110_env.py:189`
- All training happens on single map specified in `gym_config.yaml`
- Agents may overfit to specific track geometry and fail to generalize

### Available Resources
- **11 tracks total**: 7 large racing circuits (Austin, Catalunya, IMS, Monza, MoscowRaceway, Spielberg, Spielberg_blank) + 3 drift tracks (Drift, Drift2, Drift)
- **Parallel training**: `SubprocVecEnv` runs 32 parallel environments (2 × CPU cores)
- **Existing randomization**: `track_direction="random"` successfully randomizes driving direction per episode

---

## Approach Comparison

### Approach 1: Different Maps Per Parallel Environment ✅
**Description**: Each of n_envs parallel environments trains on a different map. PPO averages gradients across all environments.

**Pros**:
- **Zero runtime overhead**: Maps loaded once at initialization
- **Normalization solvable**: Requires global bounds across track pool (see implementation details)
- **Natural averaging**: PPO already averages gradients across parallel envs
- **Simple implementation**: ~60 lines of code changes
- **Proven pattern**: Similar to domain randomization in robotics
- **Every batch is diverse**: Each training batch contains experiences from multiple tracks

**Cons**:
- Limited by n_envs: With 32 envs and 3 tracks, each track gets ~10-11 environments
- All difficulties trained simultaneously (no curriculum)
- May slow initial learning if hard tracks dilute easy track signal

**Verdict**: **RECOMMENDED** - Simplest correct implementation with no downsides.

---

### Alternative Approach 2: Randomize Track at Each Reset ❌
**Description**: Every episode trains on a randomly selected map.

**Pros**:
- Maximum diversity - agent sees different tracks every episode
- No curriculum design needed
- Similar to existing `track_direction` randomization

**Cons**:
- **Critical issue**: Observation normalization bounds are track-specific (curvature, width) and become invalid when switching
- **Performance**: Expensive map loading (50-100ms) at every reset vs normal 1ms
- **Learning**: Sample inefficient - no time to learn track-specific patterns before switching
- **Reset system**: Must recreate `reset_fn` each time since it binds to specific track

**Verdict**: NOT RECOMMENDED due to normalization complexity and overhead.

---

### Alternative Approach 3: Curriculum Learning 🔄
**Description**: Train fixed number of steps on each map in sequence (easy → medium → hard).

**Pros**:
- Pedagogically sound - learn simple behaviors first
- Explicit control over progression
- Can combine with Approach 2 for curriculum across parallel envs
- Good for debugging - isolate problematic tracks

**Cons**:
- Manual curriculum design required (define "easy" vs "hard")
- Risk of catastrophic forgetting on earlier tracks
- Rigid schedule - can't adapt to agent's learning
- Arbitrary phase lengths (hyperparameter tuning needed)
- More complex implementation (requires callback system)

**Verdict**: NOT RECOMMENDED due to rigid schedule, and poor adaptation

---


## Recommended Implementation: Different Maps Per Parallel Environment
Implement the different map per parallel environment solution (Approach 1). This is most robust.


## Implementation Details

### Implementation Strategy
Modify environment creation to distribute a list of maps across parallel environments:
- Env 0, 3, 6, ... → Map 0
- Env 1, 4, 7, ... → Map 1
- Env 2, 5, 8, ... → Map 2

For example, with 32 parallel envs and 3 drift tracks, each track gets ~10-11 environments.

### Critical Issue: Global Normalization Required

**Problem**: Current implementation normalizes observations per-track, which causes inconsistent observation meanings across environments:
- Track A (max_curv=0.5): Tight turn → normalized to 1.0
- Track B (max_curv=1.0): Tight turn → normalized to 1.0
- Policy learns "1.0" has different real-world meanings, hurting generalization

**Solution**: Use **hard-coded global normalization bounds** pre-computed across ALL available tracks:
- Track A tight turn (κ=0.5) → normalized to ~0.2 (relative to global max 2.45)
- Track B tight turn (κ=1.0) → normalized to ~0.4 (relative to global max 2.45)
- Observation value 1.0 consistently means "sharpest possible turn across any track"
- Policy learns consistent meaning, improving transfer between tracks

**Pre-computed Global Bounds** (across all 10 tracks):
| Track           | Max Curv | Min Width | Max Width |
|-----------------|----------|-----------|-----------|
| Austin          | 1.4406   | 2.2000    | 2.2000    |
| Catalunya       | 1.1740   | 2.2000    | 2.2000    |
| Drift           | 2.4450   | 0.8000    | 1.0000    |
| Drift2          | 1.4424   | 1.8000    | 2.0000    |
| Drift     | 1.6023   | 1.2000    | 1.4000    |
| IMS             | 0.0750   | 2.2000    | 2.2000    |
| Monza           | 1.4637   | 2.2000    | 2.2000    |
| MoscowRaceway   | 1.3523   | 2.2000    | 2.2000    |
| Spielberg       | 1.9523   | 2.2000    | 2.2000    |
| Spielberg_blank | 1.9523   | 2.2000    | 2.2000    |
| **GLOBAL**      | **2.45** | **0.80**  | **2.20**  |

**Implementation** (simplified from original plan):
1. Add hard-coded constants `GLOBAL_MAX_CURVATURE`, `GLOBAL_MIN_WIDTH`, `GLOBAL_MAX_WIDTH` in `utils.py`
2. Use these constants as defaults in `calculate_norm_bounds()` instead of per-track computation
3. Keep `compute_global_track_bounds()` utility in `training_utils.py` for regenerating values when new tracks are added

**Why hard-coded constants are better than dynamic computation**:
- **Simplicity**: No need to compute bounds at runtime or pass them through config chains
- **Consistency**: Every `gym.make()` call gets same normalization - training, eval, standalone
- **Testability**: Fixed values are easy to verify and reason about
- **Robustness**: No risk of bounds mismatch between training and evaluation
- **Performance**: No track loading overhead at environment creation

**Adding new custom tracks**:
If you add a new track with geometry outside existing bounds (e.g., tighter curvature than 2.45), you must:
1. Run `python train/extract_global_track_norm_bounds.py` to recalculate bounds across all tracks
2. Update the hard-coded constants in `f1tenth_gym/envs/utils.py` with the printed GLOBAL values
3. Retrain any existing models (observation meanings have changed)

**Which features need global bounds?**
- **Lookahead curvatures**: YES - curvature varies significantly between tracks (drift tracks have tighter turns than racing circuits)
- **Track width (frenet_n, lookahead_widths)**: YES - track widths may vary, should also use global bounds
- **Other features**: NO - other features (velocity, steering, etc.) are vehicle-dependent, not track-dependent

### Code Changes Required

#### File 1: `train/training_utils.py`
**Location**: Lines 21-37 (function `make_subprocvecenv`)

**Change**: Add `track_pool` parameter to distribute maps across environments. Keep `compute_global_track_bounds()` as a utility for regenerating hard-coded constants when new tracks are added.

**Keep existing helper function** (modified to print table):
```python
def compute_global_track_bounds(track_pool: list[str], track_scale: float = 1.0) -> dict:
    """
    Compute global normalization bounds across all tracks in a pool.

    This is a UTILITY for regenerating hard-coded constants in utils.py
    when new tracks are added. It is NOT called at runtime.

    Usage:
        python train/extract_global_track_norm_bounds.py
        # Or directly:
        from train.training_utils import compute_global_track_bounds
        bounds = compute_global_track_bounds(["Drift", "Drift2", "Austin", ...])

    Args:
        track_pool: List of track names to compute bounds across
        track_scale: Scale factor for track loading (default 1.0)

    Returns:
        Dictionary with keys: track_max_curv, track_min_width, track_max_width
    """
    from f1tenth_gym.envs.track import Track
    from f1tenth_gym.envs.track.track_utils import get_min_max_curvature, get_min_max_track_width

    max_curvatures = []
    min_widths = []
    max_widths = []

    # Print table header
    print("\nTrack bounds analysis:")
    print("=" * 70)
    print(f"{'Track':<20} {'Max Curv':>12} {'Min Width':>12} {'Max Width':>12}")
    print("-" * 70)

    for track_name in track_pool:
        try:
            track = Track.from_track_name(track_name, track_scale=track_scale)
        except FileNotFoundError as e:
            raise ValueError(
                f"Invalid track name '{track_name}' in track_pool. "
                f"Please check available tracks in the 'maps/' directory."
            ) from e

        # get_min_max_curvature returns symmetric bounds (-max, +max)
        _, max_curv = get_min_max_curvature(track)
        max_curvatures.append(max_curv)

        min_width, max_width = get_min_max_track_width(track)
        min_widths.append(min_width)
        max_widths.append(max_width)

        # Print per-track values
        print(f"{track_name:<20} {max_curv:>12.4f} {min_width:>12.4f} {max_width:>12.4f}")

    # Compute global bounds
    global_max_curv = max(max_curvatures)
    global_min_width = min(min_widths)
    global_max_width = max(max_widths)

    # Print global bounds
    print("=" * 70)
    print(f"{'GLOBAL':<20} {global_max_curv:>12.4f} {global_min_width:>12.4f} {global_max_width:>12.4f}")
    print()
    print("Update these values in f1tenth_gym/envs/utils.py:")
    print(f"  GLOBAL_MAX_CURVATURE = {global_max_curv:.4f}")
    print(f"  GLOBAL_MIN_WIDTH = {global_min_width:.4f}")
    print(f"  GLOBAL_MAX_WIDTH = {global_max_width:.4f}")
    print()

    return {
        "track_max_curv": global_max_curv,
        "track_min_width": global_min_width,
        "track_max_width": global_max_width,
    }
```

**Modify `make_subprocvecenv`** (simplified - no bounds computation/return):
```python
def make_subprocvecenv(
    seed: int, config: dict, n_envs: int, track_pool: list[str] | None = None
) -> SubprocVecEnv:
    """
    Create a SubprocVecEnv parallelized environment.

    Args:
        seed: Random seed
        config: Base gym configuration dict
        n_envs: Number of parallel environments
        track_pool: Optional list of maps to distribute across envs.
                   If provided, envs will cycle through these maps.
                   Example: ["Drift", "Drift2", "Drift"]

    Returns:
        SubprocVecEnv with environments distributed across track pool (if provided)
    """
    if track_pool is not None:
        # Validate track pool
        if not isinstance(track_pool, list) or len(track_pool) == 0:
            raise ValueError("track_pool must be a non-empty list")

        # Create environments with different maps
        # Global normalization bounds are hard-coded in utils.py
        env_fns = []
        for i in range(n_envs):
            # Cycle through track pool
            map_name = track_pool[i % len(track_pool)]
            env_config = config.copy()
            env_config["map"] = map_name
            env_fns.append(make_env(seed=seed, rank=i, config=env_config))

        env = SubprocVecEnv(env_fns)

        # Print distribution summary
        from collections import Counter
        distribution = Counter(track_pool[i % len(track_pool)] for i in range(n_envs))
        print(f"[Multi-map] Track distribution: {dict(distribution)}")

        return env
    else:
        # Original single-map behavior
        env = SubprocVecEnv([make_env(seed=seed, rank=i, config=config) for i in range(n_envs)])
        print(f"Successfully created {n_envs} parallel environments as SubProcVecEnv with seed {seed}")
        return env
```

**Why simplified**:
- No bounds computation at runtime - uses hard-coded constants
- No tuple return - just returns the environment
- No breaking change to callers
- `compute_global_track_bounds()` kept as offline utility only

---

#### File 2: `train/config/gym_config.yaml`
**Location**: Anywhere in the file (suggested: after `map` parameter)

**Change**: Add optional `track_pool` parameter.

```yaml
# Multi-map training (optional)
# If specified, parallel environments will train on different maps from this list
# If null/commented, uses single 'map' parameter above
track_pool:
  - "Drift"
  - "Drift2"
  - "Drift"
# For single-map training, set to null or comment out:
# track_pool: null
```

**Why**: Allows easy switching between single-map and multi-map training via config.

---

#### File 3: `train/config/env_config.py`
**Location**: Line ~38 (after other constant definitions)

**Change**: Load `TRACK_POOL` from YAML.

```python
# Add after existing constant definitions
TRACK_POOL = _gym_config.get("track_pool", None)
```

**Location**: Line ~113 (inside `get_drift_train_config()` return dict)

**Change**: Add `track_pool` to config dict.

```python
def get_drift_train_config() -> dict:
    return {
        "seed": SEED,
        "map": MAP,
        # ... other config parameters ...
        "track_pool": TRACK_POOL,  # ADD THIS LINE
    }
```

**Why**: Makes track pool accessible to training scripts.

---

#### File 4: `train/ppo_race.py`
**Location**: Line ~73 (where `make_subprocvecenv` is called)

**Change**: Pass `TRACK_POOL` constant to environment creation.

```python
from train.config.env_config import (
    # ... existing imports ...
    TRACK_POOL,  # ADD THIS IMPORT
)

def train_ppo_race():
    TRAIN_CONFIG = get_drift_train_config()

    # Create vectorized environment with optional multi-map support
    env = make_subprocvecenv(SEED, TRAIN_CONFIG, N_ENVS, track_pool=TRACK_POOL)

    # Create eval env (uses same hard-coded global bounds automatically)
    eval_env = make_eval_env(EVAL_SEED, TRAIN_CONFIG)

    # ... rest of training code unchanged ...
```

**Why**:
- Simply passes track_pool for map distribution
- No bounds capture needed - hard-coded constants handle normalization
- Eval env automatically uses same constants as training envs

---

#### File 4b: `train/training_utils.py` - `make_eval_env` (NO CHANGES NEEDED)

The existing `make_eval_env` function requires no changes. Both training and eval environments automatically use the same hard-coded global bounds from `utils.py`.

---

#### File 5: `f1tenth_gym/envs/utils.py`
**Location**: Near top of file, after imports

**Change**: Add hard-coded global track bounds constants.

**Add constants**:
```python
# Global track bounds for observation normalization
# Pre-computed across all available tracks (Austin, Catalunya, Drift, Drift2,
# Drift, IMS, Monza, MoscowRaceway, Spielberg, Spielberg_blank)
#
# IMPORTANT: If you add a new track with geometry outside these bounds,
# run compute_global_track_bounds() from train/training_utils.py and update these values.
GLOBAL_MAX_CURVATURE = 2.45  # max abs curvature across all tracks (from Drift)
GLOBAL_MIN_WIDTH = 0.80  # min track width across all tracks (from Drift)
GLOBAL_MAX_WIDTH = 2.20  # max track width across all tracks (most racing tracks)
```

**Location**: Lines 188-201 (track-dependent bounds calculation)

**Change**: Use hard-coded constants instead of per-track computation.

**Find these lines**:
```python
# Frenet lateral distance and track widths
if "frenet_n" in features_set or "lookahead_widths" in features_set:
    min_width, max_width = get_min_max_track_width(track)

    if "frenet_n" in features_set:
        half_max_width = 0.5 * max_width
        bounds["frenet_n"] = (-half_max_width, half_max_width)

    if "lookahead_widths" in features_set:
        bounds["lookahead_widths"] = (min_width, max_width)

# Lookahead curvatures
if "lookahead_curvatures" in features_set:
    min_curv, max_curv = get_min_max_curvature(track)
    bounds["lookahead_curvatures"] = (min_curv, max_curv)
```

**Replace with**:
```python
# Frenet lateral distance and track widths (use global bounds for cross-track consistency)
if "frenet_n" in features_set or "lookahead_widths" in features_set:
    min_width, max_width = GLOBAL_MIN_WIDTH, GLOBAL_MAX_WIDTH

    if "frenet_n" in features_set:
        half_max_width = 0.5 * max_width
        bounds["frenet_n"] = (-half_max_width, half_max_width)

    if "lookahead_widths" in features_set:
        bounds["lookahead_widths"] = (min_width, max_width)

# Lookahead curvatures (use global bounds for cross-track consistency)
if "lookahead_curvatures" in features_set:
    bounds["lookahead_curvatures"] = (-GLOBAL_MAX_CURVATURE, GLOBAL_MAX_CURVATURE)
```

**Why**:
- Hard-coded constants are simple and consistent
- No helper functions or config lookups needed
- All environments automatically use same normalization
- Easy to update if new tracks are added

---

#### File 6: `f1tenth_gym/envs/f110_env.py` (NO CHANGES NEEDED)

With hard-coded constants in `utils.py`, no validation or logging is needed in `f110_env.py`. All environments automatically use the same global bounds.

---

#### File 7: `train/extract_global_track_norm_bounds.py` (NEW FILE)
**Location**: Create new file in `maps/` directory

**Change**: Add helper script to extract bounds from all available tracks.

```python
#!/usr/bin/env python3
"""
Extract global track normalization bounds across all available maps.

This script scans the maps/ directory for all track folders and computes
global normalization bounds for observation normalization.

Usage:
    python train/extract_global_track_norm_bounds.py

After running, update the constants in f1tenth_gym/envs/utils.py:
    GLOBAL_MAX_CURVATURE
    GLOBAL_MIN_WIDTH
    GLOBAL_MAX_WIDTH
"""

import sys
from pathlib import Path

# Add parent directory to path to import training_utils
sys.path.insert(0, str(Path(__file__).parent.parent))

from train.training_utils import compute_global_track_bounds


def get_all_track_names() -> list[str]:
    """Extract all track names from subdirectories in maps/ folder."""
    maps_dir = Path(__file__).parent
    track_names = []

    for subdir in maps_dir.iterdir():
        if subdir.is_dir() and not subdir.name.startswith('.'):
            track_names.append(subdir.name)

    return sorted(track_names)


if __name__ == "__main__":
    print("Scanning maps/ directory for available tracks...")
    track_names = get_all_track_names()
    print(f"Found {len(track_names)} tracks: {', '.join(track_names)}\n")

    # Compute global bounds across all tracks
    bounds = compute_global_track_bounds(track_names, track_scale=1.0)

    print("\nDone! Copy the values above to f1tenth_gym/envs/utils.py")
```

**Why**:
- Automates discovery of all available tracks
- Makes it easy to regenerate bounds when adding new custom maps
- Provides clear instructions on where to update constants

---

#### File 8: `maps/README.md` (UPDATE)
**Location**: Existing file in `maps/` directory

**Change**: Add section documenting the bounds regeneration process.

Add this section to the README:

```markdown
## Adding Custom Maps

When adding a new custom map to this directory:

1. Create a new subdirectory with your map name (e.g., `maps/MyCustomTrack/`)
2. Add required map files (see existing maps for structure)
3. **If your track has extreme geometry** (very tight turns or unusual width):
   - Run: `python train/extract_global_track_norm_bounds.py`
   - Update the constants in `f1tenth_gym/envs/utils.py`:
     - `GLOBAL_MAX_CURVATURE`
     - `GLOBAL_MIN_WIDTH`
     - `GLOBAL_MAX_WIDTH`
   - Note: This will change observation normalization, requiring model retraining

The normalization bounds ensure consistent observation meanings across all tracks
for multi-map training and policy generalization.
```

**Why**:
- Documents the workflow for adding custom maps
- Warns about when bounds regeneration is necessary
- Explains the impact on trained models

---

### Summary of Changes
- **5 files modified, 2 files created** (simplified from original 6 modified):
  1. `train/training_utils.py` - Modify `make_subprocvecenv()` to accept `track_pool` and distribute maps. Modify `compute_global_track_bounds()` to print table.
  2. `train/config/gym_config.yaml` - Add `track_pool` config parameter
  3. `train/config/env_config.py` - Load and export `TRACK_POOL` constant
  4. `train/ppo_race.py` - Pass `TRACK_POOL` to `make_subprocvecenv()`
  5. `f1tenth_gym/envs/utils.py` - Add hard-coded global bounds constants, use them in `calculate_norm_bounds()`
  6. `train/extract_global_track_norm_bounds.py` - NEW: Helper script to regenerate bounds
  7. `maps/README.md` - UPDATE: Document bounds regeneration workflow
- **~60 lines of code added** (including helper script and documentation)
- **Key simplifications**:
  - No runtime bounds computation - hard-coded constants work for all tracks
  - No tuple return from `make_subprocvecenv()` - no breaking change
  - No `global_bounds` parameter in `make_eval_env()` - not needed
  - No validation logic in `f110_env.py` - constants are always consistent
- **Developer experience**: Helper script automates bounds regeneration when adding custom tracks
- **Backward compatible**: Setting `track_pool: null` preserves original single-map behavior (now with global normalization)

---

## Testing & Verification
- To-do after implementation
### Evaluation Protocol
After training with multi-map support:

1. **Per-track evaluation**: Test trained policy on each individual track
   - Success rate (% episodes without collision)
   - Average lap time
   - Boundary violation frequency

2. **Cross-track generalization**: Train on 2 tracks, evaluate on held-out 3rd track
   - Example: Train on `["Drift", "Drift2"]`, test on `"Drift"`

3. **Compare to baseline**: Train identical policy on single map, compare generalization

4. **Metrics to track in wandb**:
   - Episode reward per track
   - Success rate per track
   - Steps to convergence
   - Policy entropy (measure of exploration)

---

## Configuration Examples

### Example 1: Multi-Map Drift Training
```yaml
# gym_config.yaml
map: "Drift"  # Fallback/default
track_pool:
  - "Drift"
  - "Drift2"
  - "Drift"
```

### Example 2: Single-Map Training (Original Behavior)
```yaml
# gym_config.yaml
map: "Drift"
track_pool: null  # Disable multi-map
```

### Example 3: Multi-Map Racing Training
```yaml
# gym_config.yaml
map: "Spielberg"
track_pool:
  - "Spielberg"
  - "Monza"
  - "Catalunya"
```

---

## Critical Files Reference

### Files Modified
- `train/training_utils.py`:
  - Modify `compute_global_track_bounds()` to print formatted table
  - Modify `make_subprocvecenv()` to accept `track_pool` parameter
- `train/config/gym_config.yaml` - Add `track_pool` parameter
- `train/config/env_config.py` - Load and export `TRACK_POOL` constant
- `train/ppo_race.py` - Pass `TRACK_POOL` to `make_subprocvecenv()`
- `f1tenth_gym/envs/utils.py` - Add hard-coded global bounds constants
- `maps/README.md` - Document bounds regeneration workflow

### Files Created
- `train/extract_global_track_norm_bounds.py` - Helper script to regenerate bounds from all tracks

### Files NOT Modified (simplified from original plan)
- `f1tenth_gym/envs/f110_env.py` - No changes needed with hard-coded constants
- `train/training_utils.py` (`make_eval_env`) - No changes needed

### Files Referenced (Context)
- `f1tenth_gym/envs/track/track.py` (lines 138-205) - `Track.from_track_name()`
- `f1tenth_gym/envs/track/track_utils.py` - `get_min_max_curvature()`, `get_min_max_track_width()`
- `f1tenth_gym/envs/reset/__init__.py` (lines 8-43) - Reset function factory
- `maps/` directory - All available track definitions

---

## Implementation Tasks (Step-by-Step Validation)

### [X] Task 1: Add Configuration Support
**Files**: `train/config/gym_config.yaml`, `train/config/env_config.py`

**Changes**:
- Add `track_pool` list parameter to `gym_config.yaml`
- Load and export `TRACK_POOL` constant in `env_config.py`

**Validation**:
```python
from train.config.env_config import TRACK_POOL
print(TRACK_POOL)  # Should print ["Drift", "Drift2", "Drift"] or None
```

---

### [X] Task 2: Add Global Bounds Computation Helper (UTILITY ONLY)
**Files**: `train/training_utils.py`

**Changes**:
- Add `compute_global_track_bounds(track_pool, track_scale)` function as offline utility

**Note**: This is kept for regenerating hard-coded constants when new tracks are added, NOT called at runtime.

**Validation**:
```python
from train.training_utils import compute_global_track_bounds
bounds = compute_global_track_bounds(["Drift", "Drift2"], track_scale=1.0)
print(bounds)  # Should show track_max_curv, track_min_width, track_max_width
```

---

### [X] Task 3: Add Hard-Coded Global Bounds Constants (REVISED)
**Files**: `f1tenth_gym/envs/utils.py`

**Changes**:
- Add hard-coded constants: `GLOBAL_MAX_CURVATURE`, `GLOBAL_MIN_WIDTH`, `GLOBAL_MAX_WIDTH`
- Update `calculate_norm_bounds()` to use these constants instead of per-track computation
- Remove any helper functions that were added for config-based bounds lookup

**Validation**:
```python
import gymnasium as gym
from train.config.env_config import get_drift_train_config
config = get_drift_train_config()
config["map"] = "Drift"
env = gym.make("f1tenth_gym:f1tenth-v0", config=config)
# Normalization should use global bounds (no special config needed)
env.close()

# Test on different track - should use same bounds
config["map"] = "Austin"
env2 = gym.make("f1tenth_gym:f1tenth-v0", config=config)
env2.close()
```

---

### [X] Task 4: Update `make_subprocvecenv()` for Multi-Map (SIMPLIFIED)
**Files**: `train/training_utils.py`

**Changes**:
- Modify `make_subprocvecenv()` to accept `track_pool` parameter
- Distribute maps across environments (NO bounds computation or tuple return)

**Validation**:
```python
from train.training_utils import make_subprocvecenv
from train.config.env_config import get_drift_train_config

config = get_drift_train_config()
env = make_subprocvecenv(seed=42, config=config, n_envs=6, track_pool=["Drift", "Drift2"])
# Should show track distribution: {"Drift": 3, "Drift2": 3}
env.close()
```

---

### [X] Task 5: Update Training Script
**Files**: `train/ppo_race.py`

**Changes**:
- Import `TRACK_POOL` from config
- Pass `track_pool=TRACK_POOL` to `make_subprocvecenv()`
- No changes to `make_eval_env()` - uses same hard-coded bounds automatically

**Validation**:
- Run `python train/ppo_race.py` with `track_pool` configured
- Verify logs show multi-map distribution
- Verify training starts without errors

---

### [X] Task 6: Add helper script and documentation
**Files**: `train/extract_global_track_norm_bounds.py` (new), `maps/README.md` (update)

**Changes**:
- Create `extract_global_track_norm_bounds.py` script to automate bounds extraction
- Update `maps/README.md` to document bounds regeneration workflow

**Validation**:
```bash
python train/extract_global_track_norm_bounds.py
# Should print table of all track bounds and suggested constants
```

---

### Validation Order
1. **Task 1** → Config loads correctly (DONE)
2. **Task 2** → Bounds computation utility works (DONE)
3. **Task 3** → Hard-coded constants in utils.py work for all tracks
4. **Task 4** → Multi-env creation distributes maps correctly
5. **Task 5** → Full integration works end-to-end
6. **Task 6** → Documentation
