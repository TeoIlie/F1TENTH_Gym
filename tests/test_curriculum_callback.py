"""
Verification tests for CurriculumRange and CurriculumLearningCallback.
"""

from unittest.mock import MagicMock

import numpy as np
import pytest

from train.callbacks import CurriculumLearningCallback, CurriculumRange, make_curriculum_callback
from train.config import env_config as env_config_module

# ── CurriculumRange ──────────────────────────────────────────────────


def make_range(name="v", initial_lo=5.0, initial_hi=9.0, max_lo=2.0, max_hi=12.0, n_stages=6):
    return CurriculumRange(
        name=name, initial_lo=initial_lo, initial_hi=initial_hi, max_lo=max_lo, max_hi=max_hi, n_stages=n_stages
    )


def test_range_initial_state():
    r = make_range()
    assert r.get_range() == [5.0, 9.0]
    assert not r.is_at_max()
    assert r.increment == pytest.approx(0.5)


def test_range_expand_all_stages():
    r = make_range(n_stages=3, initial_lo=5.0, initial_hi=9.0, max_lo=2.0, max_hi=12.0)
    # increment = (12 - 9) / 3 = 1.0
    assert r.increment == pytest.approx(1.0)

    assert r.expand()
    assert r.get_range() == pytest.approx([4.0, 10.0])

    assert r.expand()
    assert r.get_range() == pytest.approx([3.0, 11.0])

    assert r.expand()
    assert r.get_range() == pytest.approx([2.0, 12.0])
    assert r.is_at_max()

    # no-op after max
    assert not r.expand()
    assert r.get_range() == pytest.approx([2.0, 12.0])


def test_range_rejects_asymmetric():
    with pytest.raises(ValueError, match="asymmetric"):
        CurriculumRange(name="bad", initial_lo=5.0, initial_hi=9.0, max_lo=3.0, max_hi=12.0, n_stages=3)


def test_range_rejects_zero_stages():
    with pytest.raises(ValueError, match="n_stages"):
        make_range(n_stages=0)


# ── CurriculumLearningCallback ──────────────────────────────────────


def test_callback_rejects_small_hysteresis():
    """min_episodes_between_expansions must be >= window_size."""
    with pytest.raises(ValueError, match="min_episodes_between_expansions"):
        CurriculumLearningCallback(
            v_range=make_range(),
            beta_range=make_range(name="beta", initial_lo=-0.1, initial_hi=0.1, max_lo=-0.4, max_hi=0.4, n_stages=6),
            r_range=make_range(name="r", initial_lo=-0.2, initial_hi=0.2, max_lo=-0.5, max_hi=0.5, n_stages=6),
            yaw_range=make_range(name="yaw", initial_lo=-0.2, initial_hi=0.2, max_lo=-0.5, max_hi=0.5, n_stages=6),
            n_stages=6,
            window_size=500,
            min_episodes_between_expansions=200,  # < 500 → should fail
        )


# ── make_curriculum_callback factory ─────────────────────────────────


VALID_CONFIG = {
    "enabled": True,
    "n_stages": 6,
    "window_size": 500,
    "success_threshold": 0.8,
    "min_episodes_between_expansions": 1000,
    "max_curriculum_timestep": None,
    "v_range": [5.0, 9.0, 2.0, 12.0],
    "beta_range": [-0.10, 0.10, -0.349, 0.349],
    "r_range": [-0.20, 0.20, -0.785, 0.785],
    "yaw_range": [-0.20, 0.20, -0.785, 0.785],
}


def test_factory_returns_callback():
    cb = make_curriculum_callback(VALID_CONFIG, training_mode="recover")
    assert isinstance(cb, CurriculumLearningCallback)
    assert cb.n_stages == 6
    assert cb.ranges["v"].get_range() == [5.0, 9.0]


def test_factory_returns_none_when_disabled():
    assert make_curriculum_callback({"enabled": False}, training_mode="recover") is None
    assert make_curriculum_callback({}, training_mode="recover") is None


def test_factory_returns_none_when_disabled_regardless_of_training_mode():
    """When curriculum is disabled, no error is raised even for non-recover modes."""
    assert make_curriculum_callback({"enabled": False}, training_mode="race") is None
    assert make_curriculum_callback({}, training_mode="race") is None


def test_factory_raises_on_non_recover_training_mode():
    """Curriculum learning is only valid for recovery training."""
    with pytest.raises(ValueError, match="only supported for recovery training"):
        make_curriculum_callback(VALID_CONFIG, training_mode="race")

    with pytest.raises(ValueError, match="only supported for recovery training"):
        make_curriculum_callback(VALID_CONFIG, training_mode="")

    with pytest.raises(ValueError, match="only supported for recovery training"):
        make_curriculum_callback(VALID_CONFIG, training_mode="drift")


def test_factory_uses_constructor_defaults_for_missing_keys():
    """Optional keys omitted from config should fall back to __init__ defaults."""
    minimal_config = {
        "enabled": True,
        "n_stages": 6,
        "v_range": [5.0, 9.0, 2.0, 12.0],
        "beta_range": [-0.10, 0.10, -0.349, 0.349],
        "r_range": [-0.20, 0.20, -0.785, 0.785],
        "yaw_range": [-0.20, 0.20, -0.785, 0.785],
    }
    cb = make_curriculum_callback(minimal_config, training_mode="recover")
    assert cb.window_size == 500
    assert cb.success_threshold == 0.8
    assert cb.min_episodes_between_expansions == 1000
    assert cb.max_curriculum_timestep is None


def test_factory_ranges_reach_max_after_n_stages():
    cb = make_curriculum_callback(VALID_CONFIG, training_mode="recover")
    for _ in range(6):
        for r in cb.ranges.values():
            r.expand()

    assert cb.ranges["v"].get_range() == pytest.approx([2.0, 12.0])
    assert cb.ranges["beta"].get_range() == pytest.approx([-0.349, 0.349])
    assert cb.ranges["r"].get_range() == pytest.approx([-0.785, 0.785])
    assert cb.ranges["yaw"].get_range() == pytest.approx([-0.785, 0.785])

    for r in cb.ranges.values():
        assert r.is_at_max()


# ── Callback step logic (mocked SB3 internals) ──────────────────────

N_STAGES = 3
WINDOW = 10
HYSTERESIS = 10  # == window_size (minimum allowed)


def _make_callback(max_curriculum_timestep=None):
    """Create a callback with small params for fast testing."""
    cb = CurriculumLearningCallback(
        v_range=make_range(n_stages=N_STAGES),
        beta_range=make_range(name="beta", initial_lo=-0.1, initial_hi=0.1, max_lo=-0.4, max_hi=0.4, n_stages=N_STAGES),
        r_range=make_range(name="r", initial_lo=-0.2, initial_hi=0.2, max_lo=-0.5, max_hi=0.5, n_stages=N_STAGES),
        yaw_range=make_range(name="yaw", initial_lo=-0.2, initial_hi=0.2, max_lo=-0.5, max_hi=0.5, n_stages=N_STAGES),
        n_stages=N_STAGES,
        window_size=WINDOW,
        success_threshold=0.8,
        min_episodes_between_expansions=HYSTERESIS,
        max_curriculum_timestep=max_curriculum_timestep,
    )
    # Mock SB3 internals: training_env and logger are properties off self.model
    cb.model = MagicMock()
    cb.num_timesteps = 0
    cb.locals = {}
    return cb


def _simulate_episodes(cb, n, recovered=True):
    """Simulate n completed episodes by calling _on_step repeatedly."""
    for _ in range(n):
        cb.num_timesteps += 1
        cb.locals = {
            "dones": np.array([True]),
            "infos": [{"recovered": recovered}],
        }
        cb._on_step()


def test_no_expansion_before_hysteresis():
    cb = _make_callback()
    _simulate_episodes(cb, HYSTERESIS - 1, recovered=True)
    assert cb.current_stage == 0


def test_no_expansion_with_low_success_rate():
    cb = _make_callback()
    _simulate_episodes(cb, HYSTERESIS, recovered=False)
    assert cb.current_stage == 0


def test_expansion_triggers_when_conditions_met():
    cb = _make_callback()
    _simulate_episodes(cb, HYSTERESIS, recovered=True)
    assert cb.current_stage == 1
    assert cb.ranges["v"].get_range() != [5.0, 9.0]  # range widened


def test_window_clears_after_expansion():
    cb = _make_callback()
    _simulate_episodes(cb, HYSTERESIS, recovered=True)
    assert cb.current_stage == 1
    assert cb.episodes_since_expansion == 0
    assert len(cb.success_window) == 0


def test_stops_at_n_stages():
    cb = _make_callback()
    for _ in range(N_STAGES):
        _simulate_episodes(cb, HYSTERESIS, recovered=True)
    assert cb.current_stage == N_STAGES

    # One more round of successes should not expand further
    _simulate_episodes(cb, HYSTERESIS, recovered=True)
    assert cb.current_stage == N_STAGES


def test_max_curriculum_timestep_blocks_expansion():
    cb = _make_callback(max_curriculum_timestep=5)
    # Push num_timesteps past the cap before feeding episodes
    cb.num_timesteps = 100
    _simulate_episodes(cb, HYSTERESIS, recovered=True)
    assert cb.current_stage == 0


# ── _recovery_overrides uses curriculum max ranges ─────────────────


def test_recovery_overrides_uses_curriculum_max_ranges_when_enabled():
    """When curriculum is enabled, recovery ranges should be [max_lo, max_hi] from curriculum config."""
    curriculum_config = {
        "enabled": True,
        "n_stages": 4,
        "v_range": [6.0, 8.0, 1.0, 13.0],
        "beta_range": [-0.05, 0.05, -0.5, 0.5],
        "r_range": [-0.1, 0.1, -1.0, 1.0],
        "yaw_range": [-0.15, 0.15, -0.9, 0.9],
    }
    saved = env_config_module.CURRICULUM_CONFIG
    try:
        env_config_module.CURRICULUM_CONFIG = curriculum_config
        overrides = env_config_module._recovery_overrides()
        assert overrides["recovery_v_range"] == [1.0, 13.0]
        assert overrides["recovery_beta_range"] == [-0.5, 0.5]
        assert overrides["recovery_r_range"] == [-1.0, 1.0]
        assert overrides["recovery_yaw_range"] == [-0.9, 0.9]
    finally:
        env_config_module.CURRICULUM_CONFIG = saved


def test_recovery_overrides_no_ranges_when_curriculum_disabled():
    """When curriculum is disabled, recovery ranges should not be set (fall back to GKEnv defaults)."""
    saved = env_config_module.CURRICULUM_CONFIG
    try:
        env_config_module.CURRICULUM_CONFIG = {"enabled": False}
        overrides = env_config_module._recovery_overrides()
        assert "recovery_v_range" not in overrides
        assert "recovery_beta_range" not in overrides
        assert "recovery_r_range" not in overrides
        assert "recovery_yaw_range" not in overrides
    finally:
        env_config_module.CURRICULUM_CONFIG = saved
