"""
Tests for the per-episode multiplicative-Gaussian domain randomization (DR)
applied in ``GKEnv.reset()``.

The DR loop perturbs selected vehicle parameters by ``X ~ N(1, sigma)`` and
clips the multiplier at ``[1 - K*sigma, 1 + K*sigma]``. Each test below
isolates one invariant of that contract.

Env construction is the dominant cost (~1.2s vs ~2ms per warm reset), so
fixtures share one single-agent env and one two-agent env across all tests.
Each test sets ``env.dr_sigmas`` / ``env.dr_clip_k`` explicitly at its top —
there is no autouse cleanup, so tests are responsible for declaring their own
DR config rather than inheriting the previous test's.
"""

import gymnasium as gym
import numpy as np
import pytest

from gymkhana.envs.gymkhana_env import GKEnv


def _make_env(num_agents=1):
    config = {
        "map": "Spielberg",
        "num_agents": num_agents,
        "observation_config": {"type": None},
        "reset_config": {"type": "rl_random_static"},
    }
    return gym.make("gymkhana:gymkhana-v0", config=config).unwrapped


@pytest.fixture(scope="module")
def env1():
    """Shared single-agent env. Tests mutate ``dr_sigmas`` / ``dr_clip_k``
    before calling reset; the env's internal `_base_params` is constant."""
    return _make_env(num_agents=1)


@pytest.fixture(scope="module")
def env2():
    """Shared two-agent env for the broadcast test."""
    return _make_env(num_agents=2)


def _multipliers(env, name, seeds):
    """Reset ``env`` once per seed and return the resulting param/base ratios."""
    base = env._base_params[name]
    out = np.empty(len(seeds))
    for i, s in enumerate(seeds):
        env.reset(seed=int(s))
        out[i] = env.sim.agents[0].params[name] / base
    return out


def test_no_dr_baseline(env1):
    """1. No-DR baseline.

    Empty ``dr_sigmas`` ⇒ RaceCar params == base after every reset.
    """
    env1.dr_sigmas = {}
    for s in range(5):
        env1.reset(seed=s)
        assert env1.sim.agents[0].params == env1._base_params


def test_reproducibility_and_multi_param_independence(env1):
    """2. Reproducibility + multi-param independence.

    Same seed twice ⇒ identical multipliers; m and I get independent draws.

    Reproducibility-across-envs reduces to reproducibility-across-resets-on-one-env
    because DR draws come solely from the global ``np.random`` stream, which
    is seeded at the top of ``reset()``. Construction-time RNG never touches
    the DR draws.
    """
    env1.dr_sigmas = {"m": 0.05, "I": 0.05}
    env1.dr_clip_k = 3.0
    base_m, base_I = env1._base_params["m"], env1._base_params["I"]

    def collect():
        out = []
        for s in range(20):
            env1.reset(seed=s)
            p = env1.sim.agents[0].params
            out.append((p["m"] / base_m, p["I"] / base_I))
        return np.asarray(out)

    pass1, pass2 = collect(), collect()
    np.testing.assert_array_equal(pass1, pass2)  # reproducibility
    # Independence: m and I within the same reset must not be the same draw.
    assert not np.allclose(pass1[:, 0], pass1[:, 1]), "m and I share an RNG draw"


def test_sigma_is_std_dev(env1):
    """3. Statistical correctness — sigma is std dev, not variance.

    Sample mean ≈ 1.0 and sample std ≈ sigma — confirms ``scale=sigma``.
    """
    sigma = 0.05
    env1.dr_sigmas = {"m": sigma}
    env1.dr_clip_k = 3.0
    mults = _multipliers(env1, "m", range(500))  # SE(mean) ≈ 2.2e-3 vs tol 1e-2
    assert abs(mults.mean() - 1.0) < 0.01, f"mean drifted: {mults.mean()}"
    assert abs(mults.std() - sigma) / sigma < 0.10, f"std off: {mults.std()}"


def test_scope_only_listed_params_perturbed(env1):
    """4. Scope — only listed params are perturbed.

    Listed params change across resets; everything else stays at base.
    """
    env1.dr_sigmas = {"m": 0.1}
    env1.dr_clip_k = 3.0
    env1.reset(seed=0)
    p0 = dict(env1.sim.agents[0].params)
    env1.reset(seed=1)
    p1 = dict(env1.sim.agents[0].params)
    assert p0["m"] != p1["m"], "listed param did not change between resets"
    for k in env1._base_params:
        if k == "m":
            continue
        assert p0[k] == env1._base_params[k], f"unlisted param {k} drifted"
        assert p1[k] == env1._base_params[k], f"unlisted param {k} drifted"


def test_no_compounding_across_resets(env1):
    """5. No compounding — perturbs off the base, not the previous episode.

    Mean ≈ 1.0 *and* sample std ≈ sigma. A compounding bug (random walk
    in multiplicative space) preserves the mean but explodes the variance.
    """
    sigma = 0.2
    env1.dr_sigmas = {"m": sigma}
    env1.dr_clip_k = 3.0
    mults = _multipliers(env1, "m", range(200))
    assert abs(mults.mean() - 1.0) < 0.05, f"mean drift: {mults.mean()}"
    assert mults.std() < 2 * sigma, f"std explosion suggests compounding: {mults.std()}"
    assert env1._base_params["m"] == GKEnv.f1tenth_vehicle_params()["m"]


def test_propagation_to_racecar(env1):
    """6. Propagation to dynamics — RaceCar.params holds the perturbed values.

    Single-agent: RaceCar.params and Simulator.params both hold the perturbed value.
    """
    env1.dr_sigmas = {"m": 0.2}
    env1.dr_clip_k = 3.0
    env1.reset(seed=7)
    rc_m = env1.sim.agents[0].params["m"]
    assert rc_m != env1._base_params["m"], "RaceCar.params['m'] not perturbed"
    assert env1.sim.params["m"] == rc_m


def test_multi_agent_broadcast(env2):
    """7. Multi-agent broadcast — every agent receives the same perturbed dict.

    ``update_params`` with ``agent_idx=-1`` writes to all agents. Guards
    against a regression that drops the broadcast and only updates agent 0.
    """
    env2.dr_sigmas = {"m": 0.2}
    env2.dr_clip_k = 3.0
    env2.reset(seed=3)
    m0 = env2.sim.agents[0].params["m"]
    m1 = env2.sim.agents[1].params["m"]
    assert m0 != env2._base_params["m"], "agent 0 not perturbed"
    assert m0 == m1, f"agents diverge: {m0} vs {m1}"


def test_clipping_bounds_respected(env1):
    """8. Clipping at ±K*sigma on the multiplier.

    No multiplier ever exceeds 1 ± K*sigma.
    """
    sigma, K = 0.1, 3.0
    env1.dr_sigmas = {"m": sigma}
    env1.dr_clip_k = K
    mults = _multipliers(env1, "m", range(1000))
    eps = 1e-9
    assert mults.max() <= 1.0 + K * sigma + eps, f"max={mults.max()} > {1 + K * sigma}"
    assert mults.min() >= 1.0 - K * sigma - eps, f"min={mults.min()} < {1 - K * sigma}"


def test_tight_clip_reduces_spread(env1):
    """9. dr_clip_k is load-bearing — tight clip squeezes the distribution.

    Tight ``dr_clip_k`` truncates the multiplier distribution measurably.
    """
    sigma = 0.1
    env1.dr_sigmas = {"m": sigma}
    env1.dr_clip_k = 5.0
    loose = _multipliers(env1, "m", range(200))
    env1.dr_clip_k = 0.5
    tight = _multipliers(env1, "m", range(200))
    assert tight.std() < loose.std()
    assert tight.max() <= 1.0 + 0.5 * sigma + 1e-9
    assert tight.min() >= 1.0 - 0.5 * sigma - 1e-9


def test_train_configs_have_dr_eval_configs_do_not():
    """10. Train-only wiring — train configs ship DR, eval configs do not.

    ``get_*_train_config`` must inject ``domain_randomization`` from the
    YAML; ``get_*_test_config`` must leave it absent (or ``None``) so eval
    episodes run on nominal vehicle params.
    """
    from train.config.env_config import (
        DOMAIN_RANDOMIZATION,
        get_drift_test_config,
        get_drift_train_config,
        get_recovery_test_config,
        get_recovery_train_config,
    )

    # YAML-disabled DR is a valid operational choice — skip rather than fail.
    if not DOMAIN_RANDOMIZATION:
        pytest.skip("DR disabled in gym_config.yaml")

    for getter in (get_drift_train_config, get_recovery_train_config):
        cfg = getter()
        assert cfg.get("domain_randomization") == DOMAIN_RANDOMIZATION, f"{getter.__name__} did not inject DR from YAML"

    for getter in (get_drift_test_config, get_recovery_test_config):
        cfg = getter()
        # Either the key is absent or it is explicitly None/empty — never an
        # active perturbation dict.
        assert not cfg.get("domain_randomization"), (
            f"{getter.__name__} leaks DR into eval: {cfg.get('domain_randomization')}"
        )


def test_lf_perturbation_preserves_wheelbase(env1):
    """`lf` randomization: `lf + lr` stays exactly equal to base wheelbase."""
    sigma = 0.05
    env1.dr_sigmas = {"lf": sigma}
    env1.dr_clip_k = 3.0
    L_base = env1._base_params["lf"] + env1._base_params["lr"]
    lf_mults = []
    for s in range(200):
        env1.reset(seed=s)
        p = env1.sim.agents[0].params
        assert p["lf"] + p["lr"] == pytest.approx(L_base), f"wheelbase drift: {p['lf'] + p['lr']} vs {L_base}"
        lf_mults.append(p["lf"] / env1._base_params["lf"])
    lf_mults = np.asarray(lf_mults)
    assert abs(lf_mults.mean() - 1.0) < 0.05
    assert abs(lf_mults.std() - sigma) / sigma < 0.20


def test_s_max_perturbation_mirrors_s_min(env1):
    """`s_max` randomization: `s_min == -s_max` exactly each reset."""
    sigma = 0.05
    env1.dr_sigmas = {"s_max": sigma}
    env1.dr_clip_k = 3.0
    s_max_mults = []
    for s in range(200):
        env1.reset(seed=s)
        p = env1.sim.agents[0].params
        assert p["s_min"] == -p["s_max"], f"asymmetry: {p['s_min']} vs {-p['s_max']}"
        s_max_mults.append(p["s_max"] / env1._base_params["s_max"])
    s_max_mults = np.asarray(s_max_mults)
    assert abs(s_max_mults.mean() - 1.0) < 0.05
    assert abs(s_max_mults.std() - sigma) / sigma < 0.20


def test_sv_max_perturbation_mirrors_sv_min(env1):
    """`sv_max` randomization: `sv_min == -sv_max` exactly each reset."""
    sigma = 0.05
    env1.dr_sigmas = {"sv_max": sigma}
    env1.dr_clip_k = 3.0
    sv_max_mults = []
    for s in range(200):
        env1.reset(seed=s)
        p = env1.sim.agents[0].params
        assert p["sv_min"] == -p["sv_max"], f"asymmetry: {p['sv_min']} vs {-p['sv_max']}"
        sv_max_mults.append(p["sv_max"] / env1._base_params["sv_max"])
    sv_max_mults = np.asarray(sv_max_mults)
    assert abs(sv_max_mults.mean() - 1.0) < 0.05
    assert abs(sv_max_mults.std() - sigma) / sigma < 0.20


def _make_env_with_dr(dr_config=None, dr_clip_k=3.0):
    """Construct an env with a custom DR config. Used by validation tests
    that need to assert construction itself raises — cannot reuse the
    module-scoped fixtures because validation runs in __init__."""
    config = {
        "map": "Spielberg",
        "num_agents": 1,
        "observation_config": {"type": None},
        "reset_config": {"type": "rl_random_static"},
        "domain_randomization": dr_config,
        "dr_clip_k": dr_clip_k,
    }
    return gym.make("gymkhana:gymkhana-v0", config=config).unwrapped


def test_validation_rejects_unknown_param():
    with pytest.raises(ValueError, match="not a vehicle parameter"):
        _make_env_with_dr(dr_config={"mass": 0.05})


@pytest.mark.parametrize("name,canonical", [("lr", "lf"), ("s_min", "s_max"), ("sv_min", "sv_max")])
def test_validation_rejects_forbidden_param(name, canonical):
    with pytest.raises(ValueError, match=f"randomize '{canonical}' instead"):
        _make_env_with_dr(dr_config={name: 0.05})


@pytest.mark.parametrize("bad_sigma", [0, 0.0, -0.01, 0.2, 0.5, 1.0])
def test_validation_rejects_out_of_range_sigma(bad_sigma):
    with pytest.raises(ValueError, match="sigma for"):
        _make_env_with_dr(dr_config={"m": bad_sigma})


def test_validation_accepts_valid_dr_config():
    """Sanity: a well-formed DR config builds without error."""
    env = _make_env_with_dr(dr_config={"m": 0.05, "I": 0.1}, dr_clip_k=2.5)
    assert env.dr_sigmas == {"m": 0.05, "I": 0.1}
    assert env.dr_clip_k == 2.5


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
