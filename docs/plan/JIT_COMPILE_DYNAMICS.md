# Plan: JIT-compile vehicle dynamics (STD, ST, STP) with Numba

## Context

The simulator's vehicle dynamics functions (`vehicle_dynamics_std`, `vehicle_dynamics_st`, `vehicle_dynamics_stp`) are pure-Python NumPy code called inside a tight integration loop (`RK4Integrator` calls `f(x, u, params)` four times per env step, `EulerIntegrator` once). Adding `@njit(cache=True)` to these functions should yield ~10–50× speedup on the dynamics call itself and a meaningful (~1.5–2.5×) end-to-end training speedup *when sim is the bottleneck* — this premise must be **profile-verified before committing to the refactor** (Step 0 below).

There are three blockers:
1. All dynamics functions take a `params: dict` which Numba `nopython` mode cannot consume.
2. `formula_lateral` returns a Python `list` of `(F_y, mu_y)`.
3. `vehicle_dynamics_ks_cog` returns a Python `list` instead of an ndarray.

The chosen fix is to (a) convert the params dict to a **`NamedTuple` of `float64` fields** (not a `numba.typed.Dict`) once at env init, (b) jit the dynamics functions and their callees, (c) jit the integrator class methods so Python↔nopython boundary crossings collapse to one per env step, and (d) fix the two return-type issues.

## Why NamedTuple, not `numba.typed.Dict`

`formula_lateral` alone does ~9 param lookups per call; combined with the other tyre formulas and the STD body itself, **the inner integration loop reads on the order of 80–100 param fields per env step**. `numba.typed.Dict` performs a unicode hash per lookup in nopython mode — non-negligible at this density. A `NamedTuple` of `float64` fields gives native struct attribute access at constant cost, pickles trivially for `SubprocVecEnv`, and avoids a documented churn of `typed.Dict` pickling issues across numba versions. Same refactor surface area, strictly better runtime, no pickling risk.

The params schema is static at env init — the NamedTuple's rigidity is a feature, not a cost.

## Scope

**Models to jit**: STD, ST, STP (single-track variants). MB (multi-body) stays Python — out of scope.

**Compatibility surface**: Tests and validation scripts that use `scipy.integrate.odeint` with a plain Python dict continue to work via thin Python wrappers that convert dict → NamedTuple before delegating to the jitted core.

## Step 0 — Profile first (mandatory)

Before any refactor, run a short profile to confirm dynamics is on the critical path. If dynamics is <30% of wall time, jitting it yields <1.4× end-to-end even at infinite speedup — the refactor is then not worth the risk.

```bash
python -m cProfile -o /tmp/ppo.prof train/ppo_race.py --m t  # let it run ~60s, then Ctrl-C
python -c "import pstats; pstats.Stats('/tmp/ppo.prof').sort_stats('cumulative').print_stats(40)"
```

Competing bottlenecks to rule out: `laser_models.py` ray-casting, `cartesian_to_frenet` (called 2–3× per step from `_check_boundary_frenet`, `_get_reward`, `_check_recovery_success`), observation factory + min/max tracker, `render_obs` dict construction in `step()`.

Proceed only if dynamics + integrator together account for ≥30% of cumulative time.

## Critical files to modify

### New / converted infrastructure
- `gymkhana/envs/params/__init__.py` — add `VehicleParams` NamedTuple and `to_named_tuple(params: dict) -> VehicleParams` helper.
- `gymkhana/envs/base_classes.py` — `RaceCar.__init__`, `RaceCar.update_params`, `Simulator.__init__`, `Simulator.update_params`: build and refresh `self._params_nt` alongside `self.params`. Pass `self._params_nt` (not `self.params`) into the integrator's `f_dynamics` call.

### Dynamics functions to jit
- `gymkhana/envs/dynamic_models/tire_model.py` — add `@njit(cache=True)` to all four `formula_*` functions. **Fix `formula_lateral` to return a tuple `(F_y, mu_y)` instead of a Python list** (lines 75–78). Update the two call sites in `single_track_drift.py:120–125` and any in `single_track_pacejka.py`. Replace `params["..."]` lookups with `params.<field>` attribute access.
- `gymkhana/envs/dynamic_models/kinematic.py` — add `@njit(cache=True)` to `vehicle_dynamics_ks_cog`. **Change return from `list` to `np.array(..., dtype=np.float64)`** (lines 134–140).
- `gymkhana/envs/dynamic_models/single_track.py::vehicle_dynamics_st` — add `@njit(cache=True)`.
- `gymkhana/envs/dynamic_models/single_track_drift/single_track_drift.py::vehicle_dynamics_std` — add `@njit(cache=True)`. Indexing `f_ks` (the kinematic blend) works once `vehicle_dynamics_ks_cog` returns an ndarray. Replace `x_ks = [X, Y, DELTA, V, PSI]` then `np.array(x_ks)` with a single `np.array([X, Y, DELTA, V, PSI], dtype=np.float64)` for explicit dtype.
- `gymkhana/envs/dynamic_models/single_track_pacejka/single_track_pacejka.py::vehicle_dynamics_stp` — add `@njit(cache=True)`.

### Integrator (collapse boundary crossings)
- `gymkhana/envs/integrator.py` — jit `RK4Integrator.integrate` and `EulerIntegrator.integrate`. Each RK4 stage currently re-enters nopython mode separately; jitting the integrator collapses 4 boundary crossings into 1 per env step. Implementation note: numba can't easily jit instance methods directly — extract module-level `_rk4_integrate(f, x, u, dt, params)` and `_euler_integrate(...)` njit functions that take the dynamics function as a first-class argument (numba supports this via `nb.types.FunctionType` or by direct closure if the dynamics function is also `@njit`).

### Reuse what's already jitted
- `gymkhana/envs/dynamic_models/utils.py` — `accl_constraints` and `steering_constraint` are **already `@njit`**. They will continue to receive scalar params unpacked from the NamedTuple at the call site (no signature change).

### Python-wrapper layer (for scipy.odeint test code)
- `gymkhana/envs/dynamic_models/__init__.py` — export thin wrappers `vehicle_dynamics_std_py`, `vehicle_dynamics_st_py`, `vehicle_dynamics_stp_py` that accept a plain Python dict, build the NamedTuple on every call (µs cost — no need for a cache, which avoids the `id(params)` reuse-after-GC hazard from the prior plan), and delegate to the jitted core. Tests in `tests/test_dynamics.py` and `examples/analysis/model_validation/*.py` switch to these wrappers.

### Sysid pipeline
- `gymkhana/sysid/` — audit for direct calls to `vehicle_dynamics_*` with plain dicts. Where present, route through the `*_py` wrappers. The sysid loop's perturbation step is already compatible because `update_params` will rebuild the NamedTuple on every iteration.

## Design details

### `VehicleParams` NamedTuple in `params/__init__.py`
```python
from typing import NamedTuple

class VehicleParams(NamedTuple):
    # Geometry / mass
    m: float
    lf: float
    lr: float
    h_s: float
    # Steering / accel limits
    s_min: float
    s_max: float
    sv_min: float
    sv_max: float
    v_min: float
    v_max: float
    v_switch: float
    a_max: float
    # Inertia / wheel
    I_z: float
    I_y_w: float
    R_w: float
    T_sb: float
    T_se: float
    # PAC2002 longitudinal
    tire_p_cx1: float
    tire_p_dx1: float
    tire_p_dx3: float
    tire_p_ex1: float
    tire_p_kx1: float
    tire_p_hx1: float
    tire_p_vx1: float
    # PAC2002 lateral
    tire_p_cy1: float
    tire_p_dy1: float
    tire_p_dy3: float
    tire_p_ey1: float
    tire_p_ky1: float
    tire_p_hy1: float
    tire_p_hy3: float
    tire_p_vy1: float
    tire_p_vy3: float
    # PAC2002 combined-slip longitudinal
    tire_r_bx1: float
    tire_r_bx2: float
    tire_r_cx1: float
    tire_r_ex1: float
    tire_r_hx1: float
    # PAC2002 combined-slip lateral
    tire_r_by1: float
    tire_r_by2: float
    tire_r_by3: float
    tire_r_cy1: float
    tire_r_ey1: float
    tire_r_hy1: float
    tire_r_vy1: float
    tire_r_vy3: float
    tire_r_vy4: float
    tire_r_vy5: float
    tire_r_vy6: float
    # STP (Pacejka simplified — 8 coeffs)
    B_f: float
    C_f: float
    D_f: float
    E_f: float
    B_r: float
    C_r: float
    D_r: float
    E_r: float

# Sentinel for fields not present in every preset (e.g. STD-only or STP-only keys).
# Numba requires every field be float64-typed; we default missing values to NaN
# so a misuse (e.g. ST model accidentally reading tire_p_cy1) surfaces as NaN
# in outputs rather than a silent zero.
_MISSING = float("nan")

def to_named_tuple(params: dict) -> VehicleParams:
    return VehicleParams(**{f: float(params.get(f, _MISSING)) for f in VehicleParams._fields})
```

Build cost is ~µs and only on env init / `update_params`, never in the hot path.

### `RaceCar` lifecycle
```python
# __init__
self.params = params
self._params_nt = to_named_tuple(params)

# update_params
self.params = params
self._params_nt = to_named_tuple(params)

# step: pass self._params_nt instead of self.params into the integrator
```

`self.params` (plain dict) is preserved for the existing read sites: `gymkhana_env.py` reads `self.params["s_min"]`, `self.params["v_max"]`, etc. for assertions, the action space, and reward clipping. Those stay on the dict.

### Test wrapper pattern
```python
# in dynamic_models/__init__.py
from ..params import to_named_tuple

def vehicle_dynamics_std_py(x, u_init, params: dict):
    return vehicle_dynamics_std(x, u_init, to_named_tuple(params))
```
No caching — rebuild is cheap and avoids `id()`-keyed-cache-after-GC hazards. Production env path uses jitted versions directly with `self._params_nt`.

### Integrator pattern
```python
# integrator.py
from numba import njit

@njit(cache=True)
def _rk4_step(f, x, u, dt, params):
    k1 = f(x, u, params)
    k2 = f(x + dt / 2 * k1, u, params)
    k3 = f(x + dt / 2 * k2, u, params)
    k4 = f(x + dt * k3, u, params)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

class RK4Integrator:
    def integrate(self, f, x, u, dt, params):
        return _rk4_step(f, x, u, dt, params)
```
Numba supports first-class function arguments when the callee is also `@njit` — verify on the first compile.

## Order of work

0. **Profile** (Step 0 above). Gate the rest of the work on the result.
1. **`tire_model.py`**: fix `formula_lateral` return tuple, convert dict lookups to attribute access, add `@njit(cache=True)` to all four. Smallest blast radius; easy to unit-test.
2. **`kinematic.py`**: fix `vehicle_dynamics_ks_cog` return to ndarray with explicit `dtype=np.float64`, convert dict lookups, add `@njit(cache=True)`.
3. **`single_track.py::vehicle_dynamics_st`**: convert dict lookups, add `@njit(cache=True)`.
4. **`single_track_drift.py::vehicle_dynamics_std`**: convert dict lookups, add `@njit(cache=True)`, clean up `x_ks` list-then-array pattern.
5. **`single_track_pacejka.py::vehicle_dynamics_stp`**: convert dict lookups, add `@njit(cache=True)`.
6. **`params/__init__.py`**: add `VehicleParams` and `to_named_tuple`.
7. **`base_classes.py`**: thread `_params_nt` through `RaceCar` / `Simulator`.
8. **`integrator.py`**: extract module-level `_rk4_step` / `_euler_step` njit functions; have integrator classes delegate.
9. **`dynamic_models/__init__.py`**: add `*_py` wrapper exports.
10. **Test/validation script edits**: switch `tests/test_dynamics.py` and `examples/analysis/model_validation/*.py` to `*_py` variants. Audit `gymkhana/sysid/` for direct dict-based calls.

Each step is independently testable with `np.allclose` against the pre-refactor numerical output.

## Verification

1. **Unit tests pass**: `python3 -m pytest tests/test_dynamics.py tests/test_params_yaml.py tests/test_f110_env.py -v`. Any numerical drift beyond float64 round-off → fix before proceeding.
2. **Model validation scripts produce equivalent figures**: rerun `examples/analysis/model_validation/test_compare_std.py`, `test_compare_stp.py`, `test_vehicle_models.py` and visually confirm gymkhana vs CommonRoad / pre-refactor traces overlap.
3. **SubprocVecEnv pickle check**: `python -c "import pickle; from gymkhana.envs.params import to_named_tuple; from gymkhana.envs.gymkhana_env import GKEnv; nt = to_named_tuple(GKEnv.f1tenth_std_vehicle_params()); pickle.loads(pickle.dumps(nt))"` — should round-trip cleanly. NamedTuples pickle natively, but worth one-shot confirming before launching an `n_envs=8` training run.
4. **Smoke a training run**: short `python train/ppo_race.py --m t` with `n_envs=8` for ~30 seconds, confirm no Numba `TypingError` or `NumbaPerformanceWarning`. First call compiles (~seconds); subsequent calls hit the on-disk cache.
5. **Per-call benchmark**: time `vehicle_dynamics_std` before vs after on identical inputs. Expect ~10–50× speedup.
6. **End-to-end training benchmark**: fixed 100k-step training session before and after, compare wall time. Realistic expected speedup: 1.5–2.5× **if Step 0 confirmed dynamics is on the critical path**.

## Out of scope

- Multi-body model (`multi_body/`) — significantly more code, low usage in RL training, jit-readiness uncertain.
- Switching to `jitclass` or `structref` for params — NamedTuple covers the access pattern at zero cost and far less complexity.
- Param sysid pipeline — compatible because `update_params` rebuilds the NamedTuple on every perturbation.
- Jitting `cartesian_to_frenet`, `laser_models`, etc. — if Step 0 profiling shows these dominate, they become their own follow-up plans.
