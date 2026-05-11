# Plan: JIT-compile vehicle dynamics (STD, ST, STP) with Numba

## Context

The simulator's vehicle dynamics functions (`vehicle_dynamics_std`, `vehicle_dynamics_st`, `vehicle_dynamics_stp`) are pure-Python NumPy code called inside a tight integration loop (`RK4Integrator` calls `f(x, u, params)` four times per env step, `EulerIntegrator` once). Profiling RL training runs (PPO with `n_envs` parallel envs) typically shows simulation as a major share of wall time. Adding `@njit(cache=True)` to these functions should yield ~10–50× speedup on the dynamics call itself and a meaningful (~1.5–2.5×) end-to-end training speedup when sim is the bottleneck.

The blocker is that all dynamics functions take a `params: dict` argument that Numba `nopython` mode cannot consume directly. The fix is to (a) convert the params dict to a `numba.typed.Dict[unicode_type, float64]` once at env init, (b) jit the dynamics functions and their callees, and (c) fix two return-type issues (Python `list` instead of tuple/array) that block `@njit`.

This is a typed-dict refactor with minimal API churn — the YAML schema, env config, sysid scripts, and tests keep working as-is.

## Scope

**Models to jit**: STD, ST, STP (single-track variants). MB (multi-body) stays Python — out of scope.

**Compatibility surface**: Tests and validation scripts that use `scipy.integrate.odeint` with a plain Python dict continue to work via a thin Python wrapper that converts dict → typed dict before delegating to the jitted core.

## Critical files to modify

### New / converted infrastructure
- `gymkhana/envs/params/__init__.py` — add `to_numba_dict(params: dict)` helper.
- `gymkhana/envs/base_classes.py:98, 199, 504, 544` — `RaceCar.__init__`, `RaceCar.update_params`, `Simulator.__init__`, `Simulator.update_params`: build and refresh `self._params_nb` alongside `self.params`.
- `gymkhana/envs/base_classes.py:375` — pass `self._params_nb` (not `self.params`) into the integrator's `f_dynamics` call.

### Dynamics functions to jit
- `gymkhana/envs/dynamic_models/tire_model.py` — add `@njit(cache=True)` to all four `formula_*` functions. **Fix `formula_lateral` to return a tuple `(F_y, mu_y)` instead of a Python list** (lines 75–78). Update the two call sites in `single_track_drift.py:120–125` and any in `single_track_pacejka.py`.
- `gymkhana/envs/dynamic_models/kinematic.py` — add `@njit(cache=True)` to `vehicle_dynamics_ks_cog`. **Change return from `list` to `np.array(..., dtype=np.float64)`** (lines 134–140).
- `gymkhana/envs/dynamic_models/single_track.py::vehicle_dynamics_st` — add `@njit(cache=True)`.
- `gymkhana/envs/dynamic_models/single_track_drift/single_track_drift.py::vehicle_dynamics_std` — add `@njit(cache=True)`. Indexing `f_ks` (the kinematic blend) already works once `vehicle_dynamics_ks_cog` returns an ndarray. Replace `x_ks = [X, Y, DELTA, V, PSI]` then `np.array(x_ks)` with a single `np.array([X, Y, DELTA, V, PSI])` for cleaner numba consumption.
- `gymkhana/envs/dynamic_models/single_track_pacejka/single_track_pacejka.py::vehicle_dynamics_stp` — add `@njit(cache=True)`.

### Reuse what's already jitted
- `gymkhana/envs/dynamic_models/utils.py` — `accl_constraints` and `steering_constraint` are **already `@njit`**. No changes.

### Python-wrapper layer (for scipy.odeint test code)
- `gymkhana/envs/dynamic_models/__init__.py` — export thin wrappers `vehicle_dynamics_std_py`, `vehicle_dynamics_st_py`, `vehicle_dynamics_stp_py` that accept a plain Python dict, lazily build a typed dict (cached by `id(params)` to avoid rebuild during odeint loops), and delegate to the jitted core. Tests in `tests/test_dynamics.py` and `examples/analysis/model_validation/*.py` switch to these wrappers.

## Design details

### `to_numba_dict(params: dict)` in `params/__init__.py`
```python
from numba import types
from numba.typed import Dict

def to_numba_dict(params: dict) -> "Dict":
    nb = Dict.empty(key_type=types.unicode_type, value_type=types.float64)
    for k, v in params.items():
        if isinstance(v, (int, float)):
            nb[k] = float(v)
    return nb
```
Non-numeric keys (none today, but defensive) are skipped silently. Cost is ~µs per build — done once at env init / `update_params`, never in the hot path.

### `RaceCar` lifecycle
```python
# __init__
self.params = params
self._params_nb = to_numba_dict(params)

# update_params
self.params = params
self._params_nb = to_numba_dict(params)

# step (line 375): pass self._params_nb instead of self.params
```

### Integrators (`gymkhana/envs/integrator.py`)
**No changes needed.** `RK4Integrator.integrate` and `EulerIntegrator.integrate` are regular Python methods that call `f(x, u, params)`. When `f` is `@njit` and `params` is a typed dict, each call enters native code with negligible boundary overhead. We do *not* need to jit the integrator class itself.

### Test wrapper pattern
```python
# in dynamic_models/__init__.py
_typed_cache = {}

def vehicle_dynamics_std_py(x, u_init, params: dict):
    key = id(params)
    if key not in _typed_cache:
        _typed_cache[key] = to_numba_dict(params)
    return vehicle_dynamics_std(x, u_init, _typed_cache[key])
```
Cache by `id(params)` so odeint's repeated calls within a single integration don't rebuild. Tests use `*_py` variants; production env path uses jitted versions directly with `self._params_nb`.

## Order of work

1. **`tire_model.py`**: fix `formula_lateral` return tuple, add `@njit(cache=True)` to all four. Smallest blast radius; easy to unit-test.
2. **`kinematic.py`**: fix `vehicle_dynamics_ks_cog` return to ndarray, add `@njit(cache=True)`.
3. **`single_track.py::vehicle_dynamics_st`**: add `@njit(cache=True)` (no return-type fix needed).
4. **`single_track_drift.py::vehicle_dynamics_std`**: add `@njit(cache=True)`, clean up `x_ks` list-then-array pattern.
5. **`single_track_pacejka.py::vehicle_dynamics_stp`**: add `@njit(cache=True)`.
6. **`params/__init__.py`**: add `to_numba_dict`.
7. **`base_classes.py`**: thread `_params_nb` through `RaceCar` / `Simulator`.
8. **`dynamic_models/__init__.py`**: add Python-wrapper exports + caching.
9. **Test/validation script edits**: switch `tests/test_dynamics.py` and `examples/analysis/model_validation/*.py` to `*_py` variants.

Each step is independently testable with `np.allclose` against the pre-refactor numerical output.

## Verification

1. **Unit tests pass**: `python3 -m pytest tests/test_dynamics.py tests/test_params_yaml.py tests/test_f110_env.py -v`. These already exercise the dynamics with reference outputs; if any numerical drift appears beyond float64 round-off, fix before proceeding.
2. **Model validation scripts produce equivalent figures**: rerun `examples/analysis/model_validation/test_compare_std.py`, `test_compare_stp.py`, `test_vehicle_models.py` and visually confirm the gymkhana vs CommonRoad / pre-refactor traces overlap.
3. **Smoke a training run**: short `python train/ppo_race.py --m t` with `n_envs=8` for ~30 seconds, confirm no Numba `TypingError` or `NumbaPerformanceWarning` at runtime. First call compiles (~seconds latency); subsequent calls use the on-disk cache (`__pycache__/*.nbi`).
4. **Benchmark**: `python -c "import timeit; ..."` — time `vehicle_dynamics_std` calls before vs after on identical inputs. Expect ~10–50× per-call speedup.
5. **End-to-end training benchmark**: run a fixed 100k-step training session before and after, compare wall time. Realistic expected speedup: 1.5–2.5× if dynamics is on the critical path.

## Out of scope

- Multi-body model (`multi_body/`) — significantly more code, low usage in RL training, jit-readiness uncertain.
- Jitting the integrator class (`integrator.py`) — not needed; per-step Python overhead is microseconds.
- Switching params storage to NamedTuple or jitclass — premature; revisit only if profiling shows typed-dict lookups dominating.
- Param sysid pipeline — already compatible because `update_params` rebuilds the typed dict on every perturbation.
