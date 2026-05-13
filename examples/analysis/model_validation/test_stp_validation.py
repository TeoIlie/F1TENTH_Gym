"""Manual validation script for the STP dynamics model.

Independent checks, run for inspection (prints + plots, with assertions):
    1.  Parity vs. a Python port of f110's STDKinematics::update_pacejka in the
        dynamic regime (V=5 m/s, blend ≈ 1 in both formulations). Tests the
        math line-by-line.
    1b. Same parity check at low speed (V=0.1 m/s) with gymkhana's blend
        thresholds forced to match f110's. Exercises the kinematic-blend
        branch — ψ̇ excluded by design (semantic difference; see docstring).
    2.  Steady-state cornering radius matches the analytical understeer-
        gradient prediction at small slip across V ∈ {1, 2, 3} m/s.
    3.  Pacejka lateral-force curve has the expected shape (rises, peaks,
        falls) and the right peak magnitude.

Run from the gymkhana root with the project venv:
    python examples/analysis/model_validation/test_stp_validation.py
"""

import numpy as np

from gymkhana.envs.dynamic_models import vehicle_dynamics_stp_py as vehicle_dynamics_stp
from gymkhana.envs.integrator import EulerIntegrator, RK4Integrator
from gymkhana.envs.params import load_params

PLOT = True


# -----------------------------------------------------------------------------
# Reference: faithful Python port of f110's STDKinematics::update_pacejka.
# Mirrors std_kinematics.cpp:14-133 line by line. Uses Euler integration and
# the f110 blend thresholds (v_b=3, v_s=1).
# -----------------------------------------------------------------------------
def f110_pacejka_step(state, accel, steer_angle_vel, p, dt, blend_v_b=3.0, blend_v_s=1.0):
    g = 9.81
    v_min = blend_v_b - 2 * blend_v_s

    # Slip angles
    if state["v_x"] >= v_min:
        alpha_f = np.arctan2(-state["v_y"] - p["lf"] * state["w"], state["v_x"]) + state["delta"]
        alpha_r = np.arctan2(-state["v_y"] + p["lr"] * state["w"], state["v_x"])
    else:
        alpha_f = 0.0
        alpha_r = 0.0

    F_zf = p["m"] * (-accel * p["h_s"] + g * p["lr"]) / (p["lf"] + p["lr"])
    F_zr = p["m"] * (accel * p["h_s"] + g * p["lf"]) / (p["lf"] + p["lr"])
    F_yf = (
        p["mu"]
        * p["D_f"]
        * F_zf
        * np.sin(
            p["C_f"] * np.arctan(p["B_f"] * alpha_f - p["E_f"] * (p["B_f"] * alpha_f - np.arctan(p["B_f"] * alpha_f)))
        )
    )
    F_yr = (
        p["mu"]
        * p["D_r"]
        * F_zr
        * np.sin(
            p["C_r"] * np.arctan(p["B_r"] * alpha_r - p["E_r"] * (p["B_r"] * alpha_r - np.arctan(p["B_r"] * alpha_r)))
        )
    )

    x_dot = state["v_x"] * np.cos(state["theta"]) - state["v_y"] * np.sin(state["theta"])
    y_dot = state["v_x"] * np.sin(state["theta"]) + state["v_y"] * np.cos(state["theta"])
    vx_dot = accel + (1 / p["m"]) * (-F_yf * np.sin(state["delta"])) + state["v_y"] * state["w"]
    vy_dot = (1 / p["m"]) * (F_yr + F_yf * np.cos(state["delta"])) - state["v_x"] * state["w"]
    w_dot = (1 / p["I_z"]) * (-F_yr * p["lr"] + F_yf * p["lf"] * np.cos(state["delta"]))

    end = {
        "x": state["x"] + x_dot * dt,
        "y": state["y"] + y_dot * dt,
        "theta": state["theta"] + state["w"] * dt,
        "v_x": state["v_x"] + vx_dot * dt,
        "v_y": state["v_y"] + vy_dot * dt,
        "delta": state["delta"] + steer_angle_vel * dt,
        "w": state["w"] + w_dot * dt,
    }

    # Kinematic update (f110 update_k, integrates slip_angle separately)
    lwb = p["lf"] + p["lr"]
    theta_dot_k = state["v_x"] * np.tan(state["delta"]) / lwb
    end_kin_v_x = state["v_x"] + accel * dt
    end_kin_delta = state["delta"] + steer_angle_vel * dt
    # v_y from slip-angle integration (kinematic)
    slip = np.arctan2(state["v_y"], state["v_x"]) if state["v_x"] != 0 else 0.0
    slip_dot = (
        (1 / (1 + ((p["lr"] / lwb) * np.tan(state["delta"])) ** 2))
        * (p["lr"] / (lwb * np.cos(state["delta"]) ** 2))
        * steer_angle_vel
    )
    end_kin = {
        "x": state["x"] + (state["v_x"] * np.cos(state["theta"]) - state["v_y"] * np.sin(state["theta"])) * dt,
        "y": state["y"] + (state["v_x"] * np.sin(state["theta"]) + state["v_y"] * np.cos(state["theta"])) * dt,
        "theta": state["theta"] + theta_dot_k * dt,
        "v_x": end_kin_v_x,
        "delta": end_kin_delta,
        "w": state["w"],  # carried
    }
    end_kin["v_y"] = np.tan(slip + slip_dot * dt) * end_kin_v_x

    # Blend
    w_std = 0.5 * (1 + np.tanh((state["v_x"] - blend_v_b) / blend_v_s))
    if state["v_x"] < v_min:
        w_std = 0.0
    w_kin = 1.0 - w_std
    blended = {}
    for k in ("x", "y", "theta", "v_x", "v_y", "delta", "w"):
        blended[k] = w_std * end[k] + w_kin * end_kin[k]
    return blended


def stp_state_to_f110(x):
    """gymkhana 7-state -> f110 dict (v_x = V cos beta, v_y = V sin beta)."""
    return {
        "x": x[0],
        "y": x[1],
        "delta": x[2],
        "v_x": x[3] * np.cos(x[6]),
        "v_y": x[3] * np.sin(x[6]),
        "theta": x[4],
        "w": x[5],
    }


def f110_state_to_stp(s):
    V = np.hypot(s["v_x"], s["v_y"])
    beta = np.arctan2(s["v_y"], s["v_x"]) if V > 1e-9 else 0.0
    return np.array([s["x"], s["y"], s["delta"], V, s["theta"], s["w"], beta])


# -----------------------------------------------------------------------------
# Test 1 — parity with f110 reference at high V (where blend ≈ 1 in both)
# -----------------------------------------------------------------------------
def test_parity_with_f110_reference():
    print("\n=== Test 1: parity with f110 update_pacejka reference ===")
    params = load_params("f1tenth_stp")

    # Initial state at V=5 m/s, no slip, no steering — well above both blend
    # transitions so w_std ≈ 1 in both formulations.
    x_stp = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0])
    s_ref = stp_state_to_f110(x_stp)

    # Step the steering up gradually to produce non-trivial dynamics.
    dt = 0.001  # small dt so Euler error is small
    n_steps = 500  # 0.5 s
    integ = EulerIntegrator()

    # Override gymkhana blend thresholds in-place to match f110 for parity.
    # We'll do this by patching the function-local v_s, v_b at runtime:
    # easiest is to monkey-patch by passing through a wrapper that runs above
    # both transitions where the blend is irrelevant. V starts at 5 m/s, so
    # both blends are saturated to w_std = 1.

    t_hist = np.zeros(n_steps + 1)
    stp_hist = np.zeros((n_steps + 1, 7))
    ref_hist = np.zeros((n_steps + 1, 7))
    stp_hist[0] = x_stp
    ref_hist[0] = f110_state_to_stp(s_ref)

    for k in range(n_steps):
        # Steering velocity ramp: 1 rad/s for first 50 ms (-> delta=0.05), then hold
        sv = 1.0 if k < 50 else 0.0
        accel = 0.0
        u = np.array([sv, accel])
        x_stp = integ.integrate(vehicle_dynamics_stp, x_stp, u, dt, params)
        s_ref = f110_pacejka_step(s_ref, accel, sv, params, dt)
        t_hist[k + 1] = (k + 1) * dt
        stp_hist[k + 1] = x_stp
        ref_hist[k + 1] = f110_state_to_stp(s_ref)

    err_hist = np.abs(stp_hist - ref_hist)
    max_err = err_hist.max(axis=0)

    print("  max abs error per state component over rollout (state order [x,y,δ,V,ψ,ψ̇,β]):")
    print(f"    {max_err}")
    # In the dynamic regime (w_std ≈ 1 in both) and small dt, the only sources
    # of disagreement are the chain-rule integration order (gymkhana integrates
    # V, β; f110 integrates v_x, v_y) and the kinematic-blend definitions. At
    # V=5 with w_std≈1, both should match to O(dt²) — i.e. ~1e-3 over 0.5 s.
    tol = 5e-3
    assert np.all(max_err < tol), f"parity exceeded {tol}: {max_err}"
    print(f"  [OK] all components within {tol}")

    if PLOT:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 3, figsize=(13, 7))

        axes[0, 0].plot(stp_hist[:, 0], stp_hist[:, 1], label="STP")
        axes[0, 0].plot(ref_hist[:, 0], ref_hist[:, 1], "--", label="f110 ref")
        axes[0, 0].set_title("trajectory (X, Y)")
        axes[0, 0].set_xlabel("X [m]")
        axes[0, 0].set_ylabel("Y [m]")
        axes[0, 0].set_aspect("equal")
        axes[0, 0].legend()

        for ax, idx, name in [
            (axes[0, 1], 3, "V [m/s]"),
            (axes[0, 2], 6, "β [rad]"),
            (axes[1, 0], 5, "ψ̇ [rad/s]"),
            (axes[1, 1], 2, "δ [rad]"),
        ]:
            ax.plot(t_hist, stp_hist[:, idx], label="STP")
            ax.plot(t_hist, ref_hist[:, idx], "--", label="f110 ref")
            ax.set_title(name)
            ax.set_xlabel("t [s]")
            ax.legend()

        ax = axes[1, 2]
        labels = ["x", "y", "δ", "V", "ψ", "ψ̇", "β"]
        for idx, name in enumerate(labels):
            ax.semilogy(t_hist, err_hist[:, idx] + 1e-15, label=name)
        ax.axhline(tol, color="k", ls=":", label=f"tol={tol}")
        ax.set_title("|STP − f110 ref|")
        ax.set_xlabel("t [s]")
        ax.legend(fontsize=8, ncol=2)

        fig.suptitle("Test 1: STP vs f110 reference parity")
        plt.tight_layout()
        plt.show()


# -----------------------------------------------------------------------------
# Test 1b — parity at low speed where the kinematic blend dominates
# -----------------------------------------------------------------------------
def test_parity_with_f110_reference_low_speed():
    """Below v_min in both formulations, both reduce to the bicycle kinematic model.

    Forces gymkhana's blend thresholds to match f110's via params overrides
    (blend_v_s=3.0, blend_v_b=1.0, blend_v_min=1.0). With V well under v_min,
    f110's hard cutoff sets w_std=0 exactly while gymkhana's tanh evaluates to
    ~0.3% — the residual dynamic contribution uses α=0, β_dot=0, so its
    body-frame derivatives reduce to v_x_dot=ACCL, v_y_dot=0.

    Excluded from the parity bound: ψ̇ (state index 5). Gymkhana's blend keeps
    the yaw-rate state kinematically consistent at low speed (psi_ddot_ks
    propagates ψ̇ as the derivative of V·tan(δ)/L), while f110's kinematic
    branch carries ``w`` unchanged. This is a deliberate design difference,
    not a math error. The check below verifies (a) yaw *angle* ψ still agrees
    (both integrate via V·tan(δ)/L), and (b) gymkhana's ψ̇ tracks the
    expected kinematic value V·tan(δ)/L.
    """
    print("\n=== Test 1b: low-speed parity with f110 reference ===")
    base = load_params("f1tenth_stp")
    # f110 thresholds: v_b=3 (blend center), v_s=1 (sharpness), v_min=1 (hard cutoff).
    # In gymkhana's variable naming these map to blend_v_s, blend_v_b, blend_v_min.
    params = {**base, "blend_v_s": 3.0, "blend_v_b": 1.0, "blend_v_min": 1.0}

    x_stp = np.array([0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0])  # V0=0.1 m/s
    s_ref = stp_state_to_f110(x_stp)

    dt = 1e-3
    n_steps = 500  # 0.5 s; with accel=0.3 we end at V≈0.25, still well under v_min=1.
    integ = EulerIntegrator()

    t_hist = np.zeros(n_steps + 1)
    stp_hist = np.zeros((n_steps + 1, 7))
    ref_hist = np.zeros((n_steps + 1, 7))
    stp_hist[0] = x_stp
    ref_hist[0] = f110_state_to_stp(s_ref)

    for k in range(n_steps):
        sv = 0.5 if k < 100 else 0.0  # 0.1 s steering ramp -> δ ≈ 0.05 rad
        accel = 0.3
        u = np.array([sv, accel])
        x_stp = integ.integrate(vehicle_dynamics_stp, x_stp, u, dt, params)
        s_ref = f110_pacejka_step(s_ref, accel, sv, params, dt, blend_v_b=3.0, blend_v_s=1.0)
        t_hist[k + 1] = (k + 1) * dt
        stp_hist[k + 1] = x_stp
        ref_hist[k + 1] = f110_state_to_stp(s_ref)

    err_hist = np.abs(stp_hist - ref_hist)
    max_err = err_hist.max(axis=0)
    print("  max abs error per state component over rollout (state order [x,y,δ,V,ψ,ψ̇,β]):")
    print(f"    {max_err}")
    tol = 5e-3
    parity_idx = [0, 1, 2, 3, 4, 6]  # all except ψ̇ (see docstring)
    assert np.all(max_err[parity_idx] < tol), f"low-speed parity exceeded {tol}: {max_err}"
    print(f"  [OK] [x,y,δ,V,ψ,β] all within {tol}")

    # Positive correctness check on gymkhana ψ̇: should track V·tan(δ)/L.
    L = params["lf"] + params["lr"]
    psi_dot_kin = stp_hist[:, 3] * np.tan(stp_hist[:, 2]) / L
    psi_dot_err = np.abs(stp_hist[:, 5] - psi_dot_kin).max()
    assert psi_dot_err < 1e-3, f"gymkhana ψ̇ drifts from V·tan(δ)/L: {psi_dot_err}"
    print(f"  [OK] gymkhana ψ̇ tracks V·tan(δ)/L within {psi_dot_err:.2e}")

    if PLOT:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 3, figsize=(13, 7))
        axes[0, 0].plot(stp_hist[:, 0], stp_hist[:, 1], label="STP")
        axes[0, 0].plot(ref_hist[:, 0], ref_hist[:, 1], "--", label="f110 ref")
        axes[0, 0].set_title("trajectory (X, Y)")
        axes[0, 0].set_aspect("equal")
        axes[0, 0].legend()
        for ax, idx, name in [
            (axes[0, 1], 3, "V [m/s]"),
            (axes[0, 2], 6, "β [rad]"),
            (axes[1, 0], 5, "ψ̇ [rad/s]"),
            (axes[1, 1], 2, "δ [rad]"),
        ]:
            ax.plot(t_hist, stp_hist[:, idx], label="STP")
            ax.plot(t_hist, ref_hist[:, idx], "--", label="f110 ref")
            ax.set_title(name)
            ax.set_xlabel("t [s]")
            ax.legend()
        ax = axes[1, 2]
        labels = ["x", "y", "δ", "V", "ψ", "ψ̇", "β"]
        for idx, name in enumerate(labels):
            ax.semilogy(t_hist, err_hist[:, idx] + 1e-15, label=name)
        ax.axhline(tol, color="k", ls=":", label=f"tol={tol}")
        ax.set_title("|STP − f110 ref|")
        ax.set_xlabel("t [s]")
        ax.legend(fontsize=8, ncol=2)
        fig.suptitle("Test 1b: low-speed parity (V<v_min, kinematic dominates)")
        plt.tight_layout()
        plt.show()


# -----------------------------------------------------------------------------
# Test 2 — steady-state radius matches the analytical understeer-gradient prediction
# -----------------------------------------------------------------------------
def test_steady_state_cornering():
    """Steady-state radius ≈ (L + K_us·V²/g) / tan(δ).

    K_us = (m·g/L) · (lr/C_αf − lf/C_αr), where the small-slip cornering
    stiffnesses are C_α = μ·D·C·B·F_z (derivative of the Pacejka curve at α=0).
    Replaces the prior "within 50% of Ackermann" bound with a true linearised
    prediction that should hold to a few percent in the small-slip regime.
    Sweeps three operating points to catch direction-of-deviation regressions.
    """
    print("\n=== Test 2: steady-state cornering vs understeer-gradient prediction ===")
    params = load_params("f1tenth_stp")
    integ = RK4Integrator()
    dt = 0.005
    g = 9.81

    L = params["lf"] + params["lr"]
    F_zf_static = params["m"] * g * params["lr"] / L
    F_zr_static = params["m"] * g * params["lf"] / L
    mu = params["mu"]
    C_af = mu * params["D_f"] * params["C_f"] * params["B_f"] * F_zf_static
    C_ar = mu * params["D_r"] * params["C_r"] * params["B_r"] * F_zr_static
    K_us = (params["m"] * g / L) * (params["lr"] / C_af - params["lf"] / C_ar)
    print(f"  L={L:.3f} m, C_αf={C_af:.1f} N/rad, C_αr={C_ar:.1f} N/rad, K_us={K_us:.4f} rad")

    delta_target = 0.05
    cases = [(1.0, "linear-tire"), (2.0, "mild understeer"), (3.0, "moderate understeer")]
    settle_t = 5.0
    n_steps = int(settle_t / dt)
    tol = 0.05  # 5% — small-slip linearisation should be tight at δ=0.05.

    for V0, label in cases:
        x = np.zeros(7)
        x[2] = delta_target
        x[3] = V0
        for _ in range(n_steps):
            x = integ.integrate(vehicle_dynamics_stp, x, np.array([0.0, 0.0]), dt, params)

        psi_dot_ss = x[5]
        V_ss = x[3]
        radius_actual = V_ss / psi_dot_ss
        radius_predicted = (L + K_us * V_ss**2 / g) / np.tan(delta_target)
        rel_err = abs(radius_actual - radius_predicted) / radius_predicted
        print(
            f"  V0={V0} ({label}): V_ss={V_ss:.3f}, R_actual={radius_actual:.3f} m, "
            f"R_predicted={radius_predicted:.3f} m, rel_err={rel_err * 100:.2f}%"
        )
        assert rel_err < tol, f"V0={V0}: radius off by {rel_err * 100:.1f}% (tol {tol * 100:.0f}%)"
    print(f"  [OK] all cases within {tol * 100:.0f}% of understeer-gradient prediction")


# -----------------------------------------------------------------------------
# Test 3 — Pacejka tire force curve has the right shape
# -----------------------------------------------------------------------------
def test_tire_force_curve():
    """Pacejka curve: rises ~linearly near α=0, peaks, falls off."""
    print("\n=== Test 3: Pacejka lateral-force curve shape ===")
    params = load_params("f1tenth_stp")
    g = 9.81
    F_zf = params["m"] * g * params["lr"] / (params["lf"] + params["lr"])
    Bf, Cf, Df, Ef, mu = params["B_f"], params["C_f"], params["D_f"], params["E_f"], params["mu"]

    alphas = np.linspace(-0.5, 0.5, 201)
    Fy = mu * Df * F_zf * np.sin(Cf * np.arctan(Bf * alphas - Ef * (Bf * alphas - np.arctan(Bf * alphas))))

    # Slope at origin should be positive (linear stiffness)
    slope0 = (Fy[101] - Fy[99]) / (alphas[101] - alphas[99])
    assert slope0 > 0, f"unexpected slope at α=0: {slope0}"
    # Curve should peak somewhere in (0, 0.5)
    pos_idx = np.argmax(Fy)
    pos_alpha_peak = alphas[pos_idx]
    pos_peak = Fy[pos_idx]
    assert 0.05 < pos_alpha_peak < 0.4, f"peak at α={pos_alpha_peak:.3f} outside expected range"
    # Symmetric (odd function)
    assert np.allclose(Fy, -Fy[::-1], atol=1e-9)
    # Peak ≈ μ·D·F_z
    expected_peak = mu * Df * F_zf
    assert abs(pos_peak - expected_peak) / expected_peak < 0.02
    print(
        f"  front: peak F_y = {pos_peak:.2f} N at α = {pos_alpha_peak:.3f} rad "
        f"(expected ≈ {expected_peak:.2f} N at peak)"
    )
    print(f"  slope at α=0: {slope0:.1f} N/rad (linear stiffness Bf·Cf·Df·F_z·μ = {Bf * Cf * Df * F_zf * mu:.1f})")
    print("  [OK] curve is odd, slope > 0 at origin, peaks in valid range")

    if PLOT:
        import matplotlib.pyplot as plt

        # Also compute rear curve for the plot
        F_zr = params["m"] * g * params["lf"] / (params["lf"] + params["lr"])
        Br, Cr, Dr, Er = params["B_r"], params["C_r"], params["D_r"], params["E_r"]
        Fy_r = mu * Dr * F_zr * np.sin(Cr * np.arctan(Br * alphas - Er * (Br * alphas - np.arctan(Br * alphas))))

        _, ax = plt.subplots(figsize=(8, 5))
        ax.plot(alphas, Fy, label=f"front (peak {pos_peak:.1f} N @ α={pos_alpha_peak:.2f})")
        ax.plot(alphas, Fy_r, label=f"rear (peak {Fy_r.max():.1f} N @ α={alphas[np.argmax(Fy_r)]:.2f})")
        ax.axhline(mu * Df * F_zf, color="C0", ls=":", alpha=0.5)
        ax.axhline(mu * Dr * F_zr, color="C1", ls=":", alpha=0.5)
        ax.axhline(0, color="k", lw=0.5)
        ax.axvline(0, color="k", lw=0.5)
        ax.set_xlabel("slip angle α [rad]")
        ax.set_ylabel("F_y [N]")
        ax.set_title("Test 3: Pacejka lateral force curves (steady-state load)")
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    test_parity_with_f110_reference()
    test_parity_with_f110_reference_low_speed()
    test_steady_state_cornering()
    test_tire_force_curve()
    print("\nAll validation tests passed.")
