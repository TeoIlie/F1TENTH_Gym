Here's the full plan — copy everything below the line:

---

# Domain Randomization for Vehicle Parameters

## Context

`gymkhana/envs/gymkhana_env.py` loads a fixed parameter dict from
`gymkhana/envs/params/*.yaml` and uses it for every episode. To improve
sim2real transfer for policies trained in this simulator, we want to perturb
selected parameters at the start of each training episode with multiplicative
Gaussian noise

    θ' = θ · X,    X ~ N(1, σ)

where each parameter has its own σ (the **standard deviation** of the
multiplier, not the variance — `numpy.random.Generator.normal(loc, scale)`
takes σ as `scale`, so passing the user's value directly is correct).
Randomization must be configurable per-parameter, reproducible (seeded), and
inert when the user omits the config (so eval configs need no extra flag).

## Design

### Flow

```
GKEnv.__init__
   └─ store _base_params = deepcopy(self.params)
   └─ parse self.dr_sigmas = config["domain_randomization"] or {}
        │
        ▼
GKEnv.reset(seed=…)
   ├─ if seed is not None: np.random.seed(seed)              # existing line, governs all reset RNG
   ├─ super().reset(seed=seed)
   ├─ if self.dr_sigmas:
   │     perturbed = deepcopy(self._base_params)            # always off base, no compounding
   │     for name, sigma in self.dr_sigmas.items():
   │         raw  = np.random.normal(1.0, sigma)             # σ is std dev (np.random.normal scale=σ)
   │         mult = clip(raw, 1 − K·σ, 1 + K·σ)              # K = _DR_CLIP_K = 3.0
   │         perturbed[name] = self._base_params[name] * mult
   │     self.update_params(perturbed)                       # → Simulator → all RaceCars
   └─ self.sim.reset(poses, states=states)                   # dynamics now use perturbed params
```

### Seeding

Uses the global `np.random` state, consistent with the rest of `reset()`:
recovery initial state (`np.random.uniform`) and random track direction
(`np.random.random`) already follow this pattern. The existing
`np.random.seed(seed)` at the top of `reset()` (gymkhana_env.py:1102-1103)
governs all reset-time randomness including DR draws. No dedicated
`np.random.Generator` is created.

Trade-off: DR draws share the global `np.random` stream with reset-pose
sampling and random-direction flips, so adding/removing other `np.random`
calls in `reset()` shifts the DR sequence at a fixed seed. This already
holds for recovery init and direction flips, so DR is no worse and the
simplicity is worth it.

### Configuration

Flat dict, no envelope:

```python
config = {
    "domain_randomization": {
        "m": 0.05,
        "lf": 0.03,
        "tire_p_dy1": 0.10,
    },
}
```

- Default in `default_config()`: `None`. Absence (or `{}`) ⇒ no randomization.
  No separate train/eval switch.
- The clip multiplier is a module-level constant `_DR_CLIP_K = 3.0` (not
  user-facing). Clipping is on the *multiplier*, so ≈0.27% of draws are
  clipped at default. This keeps the integrator out of pathological regimes
  and prevents non-physical sign flips for bounded params (`T_sb`, `T_se`,
  etc.) — sign flips at σ ≤ 0.10 are already ≥10σ tail events, so this is
  belt-and-braces, not a load-bearing safeguard.

### Reset semantics

- Always perturb off the deep-copied `_base_params` so noise never compounds
  across episodes.
- Reproducibility comes from the existing `np.random.seed(seed)` call at the
  top of `reset()`; no dedicated RNG stream.
- Apply perturbed params via the existing `GKEnv.update_params`
  (`gymkhana_env.py:1259`), which delegates through `Simulator.update_params`
  (`base_classes.py:544`) to every `RaceCar.params` (`base_classes.py:199`) —
  this is what the dynamics integrators read. No new plumbing.

### Caveats (documented, not enforced)

- **Action-space-defining params should not be randomized.** `s_max`,
  `sv_max`, `v_min`, `v_max` feed `CarAction` bounds built once in `__init__`
  (`gymkhana_env.py:177`). Perturbing them in `reset()` would leave the
  action space stale. Document in the config docstring; do not silently
  rebuild the action space.
- **Symmetry pairs.** `gymkhana_env.py:357-358` asserts `s_min == -s_max` and
  `sv_min == -sv_max`. The asserts run in `__init__` only, but a user adding
  `s_max` to the DR dict will end up with asymmetric bounds at runtime.
  Document; do not auto-mirror.

### Train-only wiring

DR must be active during training and **inert during evaluation** so that
eval episodes always run on the nominal `_base_params`. Strategy:

- `gymkhana/presets.py::drift_config` stays DR-agnostic (no
  `domain_randomization` key in the preset). Public users opt in via the
  `overrides` kwarg.
- `train/config/gym_config.yaml`: add a `domain_randomization` block (flat
  per-parameter σ dict). Absent ⇒ no DR.
- `train/config/env_config.py`:
  - Load it once at module import: `DOMAIN_RANDOMIZATION = _config.get("domain_randomization")`.
  - Inject it **only** in `_drift_overrides()` for the train path. Concretely,
    split into `_drift_train_overrides()` (adds
    `"domain_randomization": DOMAIN_RANDOMIZATION`) and
    `_drift_test_overrides()` (does not). Or: keep one
    `_drift_overrides()` and pass DR explicitly in
    `get_drift_train_config()` only.
  - `get_drift_test_config()` stays unchanged — DR key absent ⇒ env-level
    default of `None` kicks in ⇒ no perturbation.
- Same pattern applies to recovery (`get_recovery_train_config` vs
  `get_recovery_test_config`) if/when DR is wanted there. Out of scope for
  the initial PR; document as a future extension.

No changes to `train/ppo_race.py` — it already pulls train/test configs from
the helpers above, so the train-only routing is transparent.

## Files to modify

- `gymkhana/envs/gymkhana_env.py`
  - Add `import copy` at top.
  - Add module-level constant `_DR_CLIP_K = 3.0`.
  - `default_config()` (line 562): add `"domain_randomization": None`.
  - `__init__` (after `self.params = self.config["params"]` at line 169):
    - `self._base_params = copy.deepcopy(self.params)`
    - `self.dr_sigmas = self.config.get("domain_randomization") or {}`
  - `reset()` (after `super().reset(seed=seed)` at line 1104, before the
    `self.sim.reset(...)` call at line 1203): insert the DR application block
    shown in the diagram. No extra seeding — the existing
    `np.random.seed(seed)` at line 1103 already governs DR draws.
- `train/config/gym_config.yaml`: add a `domain_randomization` block (per-param σ).
- `train/config/env_config.py`:
  - Load `DOMAIN_RANDOMIZATION` from the YAML.
  - Inject into `get_drift_train_config()` only; leave
    `get_drift_test_config()` untouched.
- `gymkhana/presets.py`: no changes — preset stays DR-agnostic.
- `tests/test_domain_randomization.py` (new): see Verification.

No changes required to `base_classes.py`, `params/`, `train/ppo_race.py`, or
any other training script — the existing `update_params` path covers
propagation and the train/test split is already routed via the helper
functions.

## Verification

Unit tests in `tests/test_domain_randomization.py` (follow existing style in
`tests/test_action.py` / `tests/test_env_reset.py`):

1. **No-DR baseline.** Env without `domain_randomization` ⇒
   `env.sim.agents[0].params == base_params` after every reset.
2. **Reproducibility.** Two envs constructed with identical seed and DR dict
   produce identical perturbed-param sequences across N resets.
3. **Statistical correctness.** Single param at σ = 0.05, N = 5000 resets:
   sample mean of `params[name]/base[name]` within 1% of 1.0; sample std
   within 10% of 0.05 (confirms `scale=σ` is the std dev, not variance).
4. **Scope.** Params not listed in the DR dict are byte-identical to base
   after reset; listed params change between resets.
5. **No compounding.** σ = 0.5, N = 200 resets: rolling mean of the
   multiplier shows no drift away from 1.0 (would fail if perturbation
   compounded off the previous episode instead of the base).
6. **Propagation to dynamics.** After reset,
   `env.sim.agents[i].params["m"]` matches the env-level perturbed value —
   confirms the integrator actually uses the new values.
7. **Clipping.** σ = 0.1, N = 10000: `max(multiplier) ≤ 1 + 3·σ + ε` and
   `min(multiplier) ≥ 1 − 3·σ − ε` (with `_DR_CLIP_K = 3.0`).
8. **Train-only wiring.** `get_drift_train_config()` contains a non-empty
   `domain_randomization` key; `get_drift_test_config()` does not (or has it
   as `None`/`{}`). Asserted directly against the dicts returned by the
   helpers — no env construction needed.

Smoke runs:

```bash
python3 -m pytest tests/test_domain_randomization.py -v
python3 -m pytest                                      # no regressions
```

End-to-end: run a short training session (≈5k steps via
`train/ppo_race.py --m t` with a DR-enabled config containing e.g.
`{"m": 0.05, "tire_p_dy1": 0.10}`) and confirm no `instability_truncation`
spike vs baseline — sanity check that default σ values and `K = 3` clipping
do not destabilize the integrator.

## Step-by-step implementation

Designed to be tackled in order; each step is independently verifiable so a
mistake at step N doesn't poison step N+1. Run the relevant tests after
each step.

### Step 1 — Env-level plumbing in `gymkhana_env.py`

1. At the top of the file, add `import copy` alongside the existing imports.
2. Above the `GKEnv` class definition, add the module-level constant:
   `_DR_CLIP_K = 3.0`.
3. In `default_config()` (around line 562), add a new entry:
   `"domain_randomization": None`. Place it near the other reset/seed-related
   keys so it's easy to find.
4. In `__init__`, immediately after `self.params = self.config["params"]`
   (line 169), add:
   - `self._base_params = copy.deepcopy(self.params)`
   - `self.dr_sigmas = self.config.get("domain_randomization") or {}`
5. **Verify before moving on:** construct an env with no config override and
   confirm `env.dr_sigmas == {}` and `env._base_params == env.params`. Run
   `python3 -m pytest` — nothing should break since DR is dormant.

### Step 2 — DR application in `reset()`

1. In `reset()`, locate the block between `super().reset(seed=seed)` (line
   1104) and `self.sim.reset(poses, states=states)` (line 1203). The DR
   block must come **before** `self.sim.reset`.
2. Insert the perturbation loop:
   ```python
   if self.dr_sigmas:
       perturbed = copy.deepcopy(self._base_params)
       for name, sigma in self.dr_sigmas.items():
           mult = np.random.normal(1.0, sigma)
           mult = np.clip(mult, 1.0 - _DR_CLIP_K * sigma, 1.0 + _DR_CLIP_K * sigma)
           perturbed[name] = self._base_params[name] * mult
       self.update_params(perturbed)
   ```
3. **Verify:** manually construct an env with `config={"domain_randomization":
   {"m": 0.1}}`, call `env.reset(seed=0)` twice, and confirm
   `env.sim.agents[0].params["m"]` changes between resets and differs from
   `env._base_params["m"]`. Also confirm that all other params are
   byte-identical to `_base_params`.

### Step 3 — Unit tests in `tests/test_domain_randomization.py`

1. Create the file. Pattern after `tests/test_action.py` / `tests/test_env_reset.py`.
2. Write the seven env-level tests listed in Verification (no-DR baseline,
   reproducibility, statistical correctness, scope, no compounding,
   propagation, clipping).
3. Skip test #8 for now — wire it in step 5.
4. **Verify:** `python3 -m pytest tests/test_domain_randomization.py -v` — all
   pass. Then `python3 -m pytest` — no regressions elsewhere.

### Step 4 — Training pipeline wiring

1. In `train/config/gym_config.yaml`, add a new top-level block:
   ```yaml
   domain_randomization:
     m: 0.05
     # ... starter σ values for parameters you want to randomize
   ```
   (Pick a small initial set — `m`, `lf`, and one tire param is enough to
   smoke-test.)
2. In `train/config/env_config.py`:
   - Near the other `_config[...]` reads, add:
     `DOMAIN_RANDOMIZATION = _config.get("domain_randomization")`.
   - In `get_drift_train_config()`, fold DR into the returned dict, e.g.
     `return {**_base_config(TRAIN_DEBUG_RENDER), **_drift_overrides(),
     "domain_randomization": DOMAIN_RANDOMIZATION}`.
   - Leave `get_drift_test_config()` untouched.
3. **Verify:**
   - `get_drift_train_config()["domain_randomization"]` returns the YAML dict.
   - `"domain_randomization" not in get_drift_test_config()` (or is `None`).
   - `presets.py::drift_config()` still has no `domain_randomization` key.

### Step 5 — Train-only wiring test

1. Add test #8 to `tests/test_domain_randomization.py`: assert
   `get_drift_train_config()` contains a non-empty `domain_randomization`
   dict and `get_drift_test_config()` does not.
2. **Verify:** the test passes.

### Step 6 — End-to-end smoke

1. Run `python3 -m pytest` — full suite green.
2. Run a short training session: `python3 train/ppo_race.py --m t` for ≈5k
   steps. Watch `instability_truncation` rate in the wandb run; it should
   be comparable to a baseline (DR-disabled) run. If it spikes, lower σ in
   the YAML or revisit `_DR_CLIP_K`.

### Tips while implementing

- After each step, `git diff` and confirm the only changes are the ones the
  step describes. The plumbing is small enough that any unexpected diff is a
  signal to slow down.
- If a test fails at step 3, the most likely culprit is forgetting
  `deepcopy` — perturbing `self._base_params` in place will silently corrupt
  the baseline and the no-compounding test will catch it.
- The order matters: DR must run **before** `self.sim.reset(...)` because
  `update_params` writes into `RaceCar.params`, and the dynamics integrator
  reads from there during the very first integration step.