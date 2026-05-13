"""Vehicle parameter loading from YAML files."""

import copy
from pathlib import Path
from typing import NamedTuple

import yaml

_PARAMS_DIR = Path(__file__).parent


def load_params(name: str) -> dict:
    """Load a vehicle parameter set by name.

    Resolves *name* to ``<params_dir>/<name>.yaml`` and returns the contents
    as a plain ``dict``.  If the YAML contains ``_base`` and ``_overrides``
    keys, the base parameter set is loaded first and the overrides are applied
    on top (single level of inheritance).

    A fresh copy is returned on every call so callers may mutate the result
    without affecting subsequent loads.

    Args:
        name: Parameter set name (without ``.yaml`` extension), e.g.
            ``"f1tenth_std"`` or ``"f1tenth_std_drift_bias"``.

    Returns:
        Vehicle parameter dictionary.
    """
    path = _PARAMS_DIR / f"{name}.yaml"
    with open(path) as f:
        data = yaml.safe_load(f)

    if "_base" in data:
        base = load_params(data["_base"])
        base.update(data.get("_overrides", {}))
        return base

    return copy.deepcopy(data)


class VehicleParams(NamedTuple):
    """Static, numba-friendly view of the vehicle parameter dict.

    All fields are ``float64``. Fields absent from a given model's preset
    default to NaN — misuse (e.g. ST code accidentally reading ``tire_p_cy1``)
    surfaces as NaN rather than a silent zero. Built once at env init /
    ``update_params`` from a plain ``dict`` via :func:`to_named_tuple`; the
    build cost is microseconds and never on the hot path.

    Defaults live in the per-model YAML presets (e.g. STP blend thresholds in
    ``f1tenth_stp.yaml``), not in this module, so the YAML is the single
    source of truth for each preset's parameter values.

    NamedTuples pickle natively, so this works with ``SubprocVecEnv`` without
    the documented churn around ``numba.typed.Dict`` pickling.
    """

    # Geometry / mass
    m: float
    lf: float
    lr: float
    h: float
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
    I: float  # noqa: E741  - ST yaw moment of inertia (matches YAML key)
    I_z: float
    I_y_w: float
    R_w: float
    T_sb: float
    T_se: float
    # ST linear tyre
    mu: float
    C_Sf: float
    C_Sr: float
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
    # STP Pacejka (lateral-only, 8 coeffs)
    B_f: float
    C_f: float
    D_f: float
    E_f: float
    B_r: float
    C_r: float
    D_r: float
    E_r: float
    # STP kinematic↔dynamic blend thresholds (overridable via params for f110 parity tests)
    blend_v_s: float
    blend_v_b: float
    blend_v_min: float


_MISSING = float("nan")


def to_named_tuple(params: dict) -> VehicleParams:
    """Convert a parameter dict to a :class:`VehicleParams` NamedTuple.

    Missing fields are filled with NaN; presets that need a specific default
    (e.g. STP blend thresholds) declare it in their YAML. Build cost is ~µs
    and is intended to be called only at env init / ``update_params``, never
    per step.

    Args:
        params: Plain Python dict, typically loaded from a YAML preset.

    Returns:
        Fully populated :class:`VehicleParams` instance.
    """
    return VehicleParams(**{f: float(params.get(f, _MISSING)) for f in VehicleParams._fields})
