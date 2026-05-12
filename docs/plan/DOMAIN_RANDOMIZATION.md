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
   └─ store _base_params = deepcopy(self.params)            # always; defensive
   └─ parse self.dr_sigmas = config["domain_randomization"] or {}
   └─ self.dr_clip_k     = config["dr_clip_k"]               # config-driven, default 3.0
        │
        ▼
GKEnv.reset(seed=…)
   ├─ if seed is not None: np.random.seed(seed)              # existing line, governs all reset RNG
   ├─ super().reset(seed=seed)
   ├─ if self.dr_sigmas:
   │     perturbed = deepcopy(self._base_params)            # always off base, no compounding
   │     for name, sigma in self.dr_sigmas.items():
   │         raw  = np.random.normal(1.0, sigma)             # σ is std dev (np.random.normal scale=σ)
   │         mult = clip(raw, 1 − K·σ, 1 + K·σ)              # K = self.dr_clip_k (default 3.0)
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

- `domain_randomization` default in `default_config()`: `None`. Absence (or
  `{}`) ⇒ no randomization. No separate train/eval switch.
- `dr_clip_k` default in `default_config()`: `3.0`. Config-driven (not a
  module constant) so tests can adjust it without monkey-patching and YAML
  sweeps are possible. Clipping is on the *multiplier*, so ≈0.27% of draws
  are clipped at default. This keeps the integrator out of pathological
  regimes and prevents non-physical sign flips for bounded params (`T_sb`,
  `T_se`, etc.) — sign flips at σ ≤ 0.10 are already ≥10σ tail events, so
  this is belt-and-braces, not a load-bearing safeguard.

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
- **Non-RL pipelines that reuse `get_drift_train_config()` must opt out of
  DR explicitly.** Sysid replay (`examples/analysis/sysid/rollout.py`),
  baselines, and any deterministic-physics analysis that pulls the train
  config will inherit DR by default. Override with
  `"domain_randomization": None` in the consumer's local config overrides.
  Sysid's `_SYSID_OVERRIDES` does this as the canonical example.

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
  - `default_config()`: add `"domain_randomization": None` and `"dr_clip_k": 3.0`.
  - `__init__` (after `self.params = self.config["params"]`):
    - `self._base_params = copy.deepcopy(self.params)` (always, unconditional —
      cheap defensive snapshot)
    - `self.dr_sigmas = self.config.get("domain_randomization") or {}`
    - `self.dr_clip_k = self.config["dr_clip_k"]`
  - `reset()` (after `super().reset(seed=seed)`, before the `self.sim.reset(...)`
    call): insert the DR application block shown in the diagram. No extra
    seeding — the existing `np.random.seed(seed)` at the top of `reset()`
    already governs DR draws.
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
   `min(multiplier) ≥ 1 − 3·σ − ε` (with `dr_clip_k = 3.0`).
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

### Step 1 — Env-level plumbing in `gymkhana_env.py` ✅ DONE

1. Add `import copy` at the top of the file alongside `warnings`.
2. In `default_config()` (around line 562), add two entries near the other
   reset/seed-related keys:
   - `"domain_randomization": None` (absent ⇒ no perturbation)
   - `"dr_clip_k": 3.0` (multiplier-clip factor; configurable so tests and
     YAML sweeps can adjust it without monkey-patching)
3. In `__init__`, immediately after `self.params = self.config["params"]`,
   add (unconditional — `_base_params` is cheap defensive insurance even
   when DR is off, and avoids `hasattr` checks downstream):
   - `self._base_params = copy.deepcopy(self.params)`
   - `self.dr_sigmas = self.config.get("domain_randomization") or {}`
   - `self.dr_clip_k = self.config["dr_clip_k"]`
4. **Verified:** env without overrides → `dr_sigmas == {}`, `dr_clip_k == 3.0`,
   `_base_params == params` (value-equal) but `_base_params is not params`
   (distinct objects). Env with DR config plumbs `dr_sigmas` and `dr_clip_k`
   correctly.

### Step 2 — DR application in `reset()` ✅ DONE

1. In `reset()`, the DR block was inserted directly after
   `super().reset(seed=seed)` and **before** `self.track.set_direction(...)`
   so DR runs as part of the per-reset setup but the global seed is already
   established. Must execute before `self.sim.reset(poses, states=states)`
   so the dynamics integrators read the perturbed params on the very first
   integration step.
2. Implementation:
   ```python
   if self.dr_sigmas:
       perturbed = copy.deepcopy(self._base_params)
       for name, sigma in self.dr_sigmas.items():
           mult = np.random.normal(1.0, sigma)
           mult = np.clip(mult, 1.0 - self.dr_clip_k * sigma, 1.0 + self.dr_clip_k * sigma)
           perturbed[name] = self._base_params[name] * mult
       self.update_params(perturbed)
   ```
3. **Verified** (DR on `m` only, σ = 0.1):
   - `m` changes between successive resets and differs from `_base_params["m"]`.
   - All non-DR params byte-identical to `_base_params` after reset.
   - `_base_params["m"]` itself is unmutated after two perturbed resets
     (deepcopy invariant).
   - `np.random.normal(1, 0.1)` with seed 0 produces multiplier 1.176, which
     matches the expected first standard-normal draw — confirms σ is
     correctly used as scale (std dev), not variance.

### Step 3 — Unit tests in `tests/test_domain_randomization.py` ✅ DONE

1. File created using the pytest style of `test_action.py` (function tests +
   small helper, not a `unittest.TestCase`). Shared `_make_env(dr=..., dr_clip_k=...)`
   helper builds a minimal single-agent env; `_multipliers(env, name, seeds)`
   resets through a seed sequence and returns the per-reset `param/base` ratios.
2. Nine tests written (covers the seven from the plan, plus multi-agent
   broadcast and a clip-is-load-bearing sanity check):
   1. `test_no_dr_baseline` — env without DR ⇒ `sim.agents[0].params == _base_params`.
   2. `test_reproducibility_and_multi_param_independence` — same seed → identical multipliers across two envs, and `m` and `lf` get independent draws (not a shared draw replayed).
   3. `test_sigma_is_std_dev` — N=500, σ=0.05: mean within 1% of 1.0, std within 10% of σ.
   4. `test_scope_only_listed_params_perturbed` — listed params change, others byte-identical to base.
   5. `test_no_compounding_across_resets` — N=200, σ=0.2, K=3: mean ≈ 1.0 *and* sample std stays near σ (a multiplicative random walk would explode variance).
   6. `test_propagation_to_racecar` — `sim.agents[0].params["m"]` perturbed and matches `sim.params["m"]`.
   7. `test_multi_agent_broadcast` — with `num_agents=2`, both agents see the same perturbed value (guards against a regression that drops the `agent_idx=-1` broadcast).
   8. `test_clipping_bounds_respected` — N=1000, σ=0.1, K=3: no multiplier exceeds 1 ± K·σ.
   9. `test_tight_clip_reduces_spread` — tight `dr_clip_k=0.5` produces strictly smaller std than loose `dr_clip_k=5.0`; sanity check that the clip wiring is actually load-bearing.
3. Test #8 from the plan's verification list ("Train-only wiring") is
   deferred to Step 5 since it needs `env_config.py` changes.
4. **Performance:** module-scoped fixtures share one single-agent env and
   one two-agent env across all nine tests. Env construction is the dominant
   cost (~1.2s each); reuse drops the suite from 11 env builds to 2. Tests
   mutate `env.dr_sigmas` / `env.dr_clip_k` directly before each reset,
   which works because both are read fresh inside `reset()` with no
   construction-time dependency. No autouse cleanup — each test explicitly
   declares its own DR config at the top.
5. **Verified:** `python3 -m pytest tests/test_domain_randomization.py -v` →
   9 passed in 10.0s (down from 19.1s pre-optimization). Pylance warnings on
   `.unwrapped` are pre-existing typing noise (gymnasium stubs return
   `Env[Unknown, Unknown]`); same pattern in `test_env_reset.py`.

### Step 4 — Training pipeline wiring ✅ DONE

1. `train/config/gym_config.yaml` has a top-level `domain_randomization`
   block (`m: 0.05`, `I_z: 0.02`). Commented header notes that omitting
   the block disables DR.
2. `train/config/env_config.py`:
   - `DOMAIN_RANDOMIZATION = _config.get("domain_randomization")` loaded
     once at module import, alongside the other module-level constants.
   - `get_drift_train_config()` and `get_recovery_train_config()` both
     inject `"domain_randomization": DOMAIN_RANDOMIZATION` into the
     returned dict.
   - `get_drift_test_config()` and `get_recovery_test_config()` are
     untouched — DR key absent ⇒ env-level default of `None` kicks in ⇒
     no perturbation on eval.
3. `gymkhana/presets.py::drift_config` left DR-agnostic, as planned.

### Step 5 — Train-only wiring test ✅ DONE

1. `test_train_configs_have_dr_eval_configs_do_not` added at the bottom of
   `tests/test_domain_randomization.py`. Asserts:
   - `DOMAIN_RANDOMIZATION` is non-empty (otherwise the test is vacuous).
   - Both `get_drift_train_config()` and `get_recovery_train_config()`
     return a dict whose `domain_randomization` key equals
     `DOMAIN_RANDOMIZATION`.
   - Both `get_drift_test_config()` and `get_recovery_test_config()` return
     a dict whose `domain_randomization` key is falsy (absent or `None`).
2. **Verified:** `python3 -m pytest tests/test_domain_randomization.py -v`
   → 10 passed in 9.0s.

### Step 6 — End-to-end smoke

1. Run `python3 -m pytest` — full suite should be green.
2. Run a short training session: `python3 train/ppo_race.py --m t` for ≈5k
   steps. Watch `instability_truncation` rate in the wandb run; it should
   be comparable to a baseline (DR-disabled) run. If it spikes, lower σ in
   the YAML or revisit `dr_clip_k`.

### Tips while implementing

- The order matters: DR must run **before** `self.sim.reset(...)` because
  `update_params` writes into `RaceCar.params`, and the dynamics integrator
  reads from there during the very first integration step.
- If a regression appears in the no-compounding test, the most likely
  culprit is a missing `deepcopy` — perturbing `self._base_params` in place
  silently corrupts the baseline.