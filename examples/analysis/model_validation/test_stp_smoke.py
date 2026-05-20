"""Manual smoke checks for the STP (Single Track Pacejka) dynamics model.

Wiring + short rollouts in straight-line, moderate-cornering, and
hard-cornering regimes. Correctness against a reference is in
``test_stp_validation.py`` (same folder).

Run from the gymkhana root with the project's venv:
    python examples/analysis/model_validation/test_stp_smoke.py
"""

import numpy as np

from gymkhana.envs.dynamic_models import DynamicModel
from gymkhana.envs.integrator import RK4Integrator
from gymkhana.envs.params import load_params, to_named_tuple


def smoke_construction(model, params, params_nt):
    print("\n=== construction & wiring ===")
    print(f"  enum:               {model}")
    print(f"  f_dynamics:         {model.f_dynamics.__name__}")
    print(f"  standardized fn:    {model.get_standardized_state_fn().__name__}")
    print(f"  params loaded:      {len(params)} keys")
    x0 = model.get_initial_state(pose=np.array([0.0, 0.0, 0.0]), params=params)
    assert x0.shape == (7,), f"expected (7,), got {x0.shape}"
    print(f"  x0 shape:           {x0.shape}  [OK]")


def smoke_dynamics_at_rest(model, params, params_nt):
    print("\n=== single-step finiteness at rest ===")
    x0 = model.get_initial_state(pose=np.array([0.0, 0.0, 0.0]), params=params)
    u = np.array([0.0, 1.0])  # zero steer-vel, +1 m/s^2 accel
    xdot = model.f_dynamics(x0, u, params_nt)
    assert xdot.shape == (7,)
    assert np.all(np.isfinite(xdot)), "non-finite derivative at rest"
    print(f"  xdot at rest:       {xdot}  [OK]")


def smoke_straight_line(model, params_nt, integ):
    """V0=1 (above blend) + accel=1 for 2 s -> V≈3, no lateral drift."""
    print("\n=== straight-line acceleration ===")
    x = np.zeros(7)
    x[3] = 1.0
    u = np.array([0.0, 1.0])
    dt = 0.01
    for _ in range(200):
        x = integ.integrate(model.f_dynamics, x, u, dt, params_nt)

    print(f"  after 2 s straight + accel=1: V={x[3]:.3f}, X={x[0]:.3f}, Y={x[1]:.3e}, β={x[6]:.3e}")
    assert np.all(np.isfinite(x))
    assert 2.99 < x[3] < 3.01, f"unexpected V: {x[3]}"
    assert abs(x[1]) < 1e-6, f"unexpected lateral drift Y: {x[1]}"
    assert abs(x[6]) < 1e-9, f"slip angle should be zero on straight line: {x[6]}"
    print("  [OK]")


def smoke_moderate_cornering(model, params, params_nt, integ):
    """V0=3, ramp δ to ~0.1 rad, hold 1 s. Expect ψ̇>0 and bounded β."""
    print("\n=== moderate cornering ===")
    dt = 0.01
    x = np.zeros(7)
    x[3] = 3.0
    for _ in range(5):
        x = integ.integrate(model.f_dynamics, x, np.array([2.0, 0.0]), dt, params_nt)
    print(f"  after 0.05 s steer ramp:   δ={x[2]:.3f}, ψ̇={x[5]:.3f}, β={x[6]:.3f}")
    for _ in range(100):
        x = integ.integrate(model.f_dynamics, x, np.array([0.0, 0.0]), dt, params_nt)
    print(f"  after 1.05 s cornering:    ψ̇={x[5]:.3f} rad/s, β={x[6]:.3f}, V={x[3]:.3f}")

    assert np.all(np.isfinite(x))
    assert x[5] > 0, f"expected positive yaw rate for positive steer, got {x[5]}"
    assert abs(x[6]) < np.pi / 4, f"slip angle out of reasonable range: {x[6]}"
    assert params["s_min"] <= x[2] <= params["s_max"], f"steering out of limits: {x[2]}"
    print("  [OK]")


def smoke_hard_cornering(model, params_nt, integ):
    """δ=0.4 pre-loaded at V=6 for 1 s. Pacejka should saturate without diverging."""
    print("\n=== hard cornering (saturation) ===")
    dt = 0.01
    x = np.zeros(7)
    x[2] = 0.4
    x[3] = 6.0
    for _ in range(100):
        x = integ.integrate(model.f_dynamics, x, np.array([0.0, 0.0]), dt, params_nt)
    print(f"  hard corner (δ=0.4 @ 6 m/s, 1 s): ψ̇={x[5]:.3f}, β={x[6]:.3f}")
    assert np.all(np.isfinite(x)), "Pacejka rollout went non-finite"
    assert abs(x[5]) < 50.0, f"yaw rate exploded: {x[5]}"
    assert abs(x[6]) < np.pi / 2, f"slip angle past pi/2: {x[6]}"
    print("  [OK]")


if __name__ == "__main__":
    model = DynamicModel.from_string("stp")
    params = load_params("f1tenth_stp")
    params_nt = to_named_tuple(params)
    integ = RK4Integrator()

    smoke_construction(model, params, params_nt)
    smoke_dynamics_at_rest(model, params, params_nt)
    smoke_straight_line(model, params_nt, integ)
    smoke_moderate_cornering(model, params, params_nt, integ)
    smoke_hard_cornering(model, params_nt, integ)

    print("\nAll smoke checks passed.")
