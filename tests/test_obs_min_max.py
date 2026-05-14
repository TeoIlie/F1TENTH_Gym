"""
Tests for the `record_obs_min_max` observation min/max tracking feature.

Covers all three observation classes (Vector, Original, Features), both
normalization modes, the printout's bounded vs unbounded branches, and the
multi-agent collapse in the FeaturesObservation adapter.
"""

import os
import warnings
from types import SimpleNamespace

import gymnasium as gym
import numpy as np

from gymkhana.envs.gymkhana_env import GKEnv, print_obs_min_max_stats
from train.callbacks import ObsMinMaxSnapshotCallback
from train.train_utils import aggregate_and_print_obs_min_max, merge_obs_min_max


def _make_drift_env(normalize_obs: bool):
    """Build a drift env (VectorObservation) with min/max tracking enabled."""
    cfg = {
        "map": "Spielberg",
        "num_agents": 1,
        "model": "std",
        "observation_config": {"type": "drift"},
        "control_input": ["accl", "steering_angle"],
        "params": GKEnv.f1tenth_std_vehicle_params(),
        "normalize_obs": normalize_obs,
        "record_obs_min_max": True,
    }
    # normalize_obs=False with drift fires a "recommended" warning at gym.make time
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        return gym.make("gymkhana:gymkhana-v0", config=cfg)


def _run_steps(env, n=10):
    env.reset()
    for _ in range(n):
        action = env.action_space.sample()
        _, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            env.reset()


def _assert_tracker_finite(unwrapped):
    assert unwrapped.obs_tracker_step_count > 0
    for feature in unwrapped.observation_type.features:
        entry = unwrapped.obs_min_max_tracker[feature]
        assert np.isfinite(entry["min"]), f"{feature} min not updated"
        assert np.isfinite(entry["max"]), f"{feature} max not updated"


def test_tracking_and_print_with_normalization(capsys):
    """normalize_obs=True: tracker fills, header reads ENABLED, table shows Coverage."""
    env = _make_drift_env(normalize_obs=True)
    try:
        unwrapped = env.unwrapped
        assert unwrapped.record_obs_min_max is True
        _run_steps(env, n=10)
        _assert_tracker_finite(unwrapped)

        unwrapped._print_obs_min_max_stats()
        out = capsys.readouterr().out
        assert "Normalization: ENABLED" in out
        assert "Coverage" in out
        # With drift+normalize, every feature should have a finite Theor. column,
        # i.e. no "N/A" rows for the populated features.
        for feature in unwrapped.observation_type.features:
            assert feature in out
    finally:
        env.close()


def test_tracking_and_print_without_normalization(capsys):
    """normalize_obs=False: tracker still fills, header reads DISABLED, no violation block."""
    env = _make_drift_env(normalize_obs=False)
    try:
        unwrapped = env.unwrapped
        assert unwrapped.record_obs_min_max is True
        # bounds remain unset in this mode (Vector.__init__ skips calculate_norm_bounds).
        assert unwrapped.observation_type.bounds == {}

        _run_steps(env, n=10)
        _assert_tracker_finite(unwrapped)

        unwrapped._print_obs_min_max_stats()
        out = capsys.readouterr().out
        assert "Normalization: DISABLED" in out
        assert "N/A" in out
        # Violation block references calculate_norm_bounds — must not appear.
        assert "calculate_norm_bounds" not in out
    finally:
        env.close()


def test_tracking_with_original_obs(capsys):
    """OriginalObservation is supported: tracker fills, printout prints N/A for unbounded features."""
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", UserWarning)
        env = gym.make(
            "gymkhana:gymkhana-v0",
            config={
                "map": "Spielberg",
                "num_agents": 1,
                "observation_config": {"type": "original"},
                "params": GKEnv.f1tenth_std_vehicle_params(),
                "normalize_obs": False,
                "record_obs_min_max": True,
            },
        )

    # No tracking-related warning should fire — the gate is gone.
    tracking_warnings = [w for w in caught if "min/max tracking" in str(w.message)]
    assert tracking_warnings == [], f"Unexpected tracking warning(s): {tracking_warnings}"

    try:
        unwrapped = env.unwrapped
        assert unwrapped.record_obs_min_max is True
        _run_steps(env, n=5)
        _assert_tracker_finite(unwrapped)

        unwrapped._print_obs_min_max_stats()
        out = capsys.readouterr().out
        # Original features have no entries in `bounds` → theoretical columns are N/A.
        assert "N/A" in out
        for feature in unwrapped.observation_type.features:
            assert feature in out
    finally:
        env.close()


def test_tracking_with_features_obs_single_agent():
    """FeaturesObservation tracker uses bare feature names (no agent_id prefix)."""
    env = gym.make(
        "gymkhana:gymkhana-v0",
        config={
            "map": "Spielberg",
            "num_agents": 1,
            "observation_config": {"type": "kinematic_state"},
            "params": GKEnv.f1tenth_std_vehicle_params(),
            "normalize_obs": False,
            "record_obs_min_max": True,
        },
    )
    try:
        unwrapped = env.unwrapped
        assert unwrapped.record_obs_min_max is True
        # Tracker keys mirror the obs class .features list — no agent_id prefix.
        assert set(unwrapped.obs_min_max_tracker.keys()) == set(unwrapped.observation_type.features)
        _run_steps(env, n=5)
        _assert_tracker_finite(unwrapped)
    finally:
        env.close()


def test_tracking_with_features_obs_multi_agent():
    """Multi-agent FeaturesObservation: across-agent collapse aggregates into one row per feature.

    Validates the adapter at gymkhana_env.py:_update_obs_min_max that flattens
    nested {agent_id: {feature: value}} into {feature: [values across agents]}.
    With two agents at different starting poses, the tracked range for `pose_x`
    must span both agents, i.e. (max - min) must be >= the distance between
    them after a single observation.
    """
    env = gym.make(
        "gymkhana:gymkhana-v0",
        config={
            "map": "Spielberg",
            "num_agents": 2,
            "observation_config": {"type": "kinematic_state"},
            "params": GKEnv.f1tenth_std_vehicle_params(),
            "normalize_obs": False,
            "record_obs_min_max": True,
        },
    )
    try:
        unwrapped = env.unwrapped
        # Tracker still keyed by bare feature names; agents collapse into the same row.
        assert set(unwrapped.obs_min_max_tracker.keys()) == set(unwrapped.observation_type.features)

        obs, _ = env.reset(seed=0)
        # After reset, env.step is called once internally with skip_integration=True
        # (see GKEnv.reset) so the tracker has already seen both agents' poses.
        agent_ids = list(obs.keys())
        assert len(agent_ids) == 2
        per_agent_x = [float(obs[a]["pose_x"]) for a in agent_ids]
        spread = max(per_agent_x) - min(per_agent_x)
        assert spread > 0, "Test setup invalid: agents are at the same x"

        entry = unwrapped.obs_min_max_tracker["pose_x"]
        recorded_range = entry["max"] - entry["min"]
        # Recorded range must span both agents — proves across-agent aggregation works.
        assert recorded_range >= spread - 1e-6, (
            f"pose_x recorded range {recorded_range:.4f} smaller than across-agent "
            f"spread {spread:.4f} — adapter is not aggregating across agents"
        )

        _run_steps(env, n=5)
        _assert_tracker_finite(unwrapped)
    finally:
        env.close()


class _FakeVecEnv:
    """Stand-in for SubprocVecEnv that records env_method calls.

    Implements only the three methods the aggregator uses (get_attr, set_attr,
    env_method) so tests run sub-millisecond without forking subprocs.
    """

    def __init__(self, per_env: list[dict]):
        self._envs = per_env
        self.method_calls: list[tuple[str, tuple]] = []

    def get_attr(self, name, indices=None):
        envs = self._envs if indices is None else [self._envs[i] for i in indices]
        return [e[name] for e in envs]

    @property
    def num_envs(self):
        return len(self._envs)

    def set_attr(self, name, value):
        for e in self._envs:
            e[name] = value

    def env_method(self, name, *args):
        self.method_calls.append((name, args))


def _fake_subproc(record: bool, tracker: dict, step_count: int, features, bounds, normalize_obs=True):
    return {
        "record_obs_min_max": record,
        "obs_min_max_tracker": tracker,
        "obs_tracker_step_count": step_count,
        "observation_type": SimpleNamespace(features=features, bounds=bounds),
        "normalize_obs": normalize_obs,
    }


class TestObsMinMaxAggregation:
    """Cover the pure print function and the SubprocVecEnv aggregator."""

    # --- print_obs_min_max_stats: callable with externally supplied data ---

    def test_print_pure_fn_with_violation(self, capsys):
        """Exceeding theor bounds prints the ⚠️ marker and the warning footer."""
        print_obs_min_max_stats(
            tracker={"f1": {"min": -2.0, "max": 5.0}},
            step_count=100,
            features=["f1"],
            bounds={"f1": (-1.0, 1.0)},
            normalize_obs=True,
        )
        out = capsys.readouterr().out
        assert "Normalization: ENABLED" in out
        assert "Tracked over 100 timesteps" in out
        assert "⚠️" in out
        assert "calculate_norm_bounds" in out

    def test_print_pure_fn_no_recorded_values(self, capsys):
        """Untouched tracker (inf/-inf) renders an all-N/A row, no violation footer."""
        print_obs_min_max_stats(
            tracker={"f1": {"min": float("inf"), "max": float("-inf")}},
            step_count=0,
            features=["f1"],
            bounds={"f1": (-1.0, 1.0)},
            normalize_obs=False,
        )
        out = capsys.readouterr().out
        assert "Normalization: DISABLED" in out
        assert "N/A" in out
        assert "calculate_norm_bounds" not in out

    # --- aggregate_and_print_obs_min_max ---

    def test_aggregator_noop_when_all_flags_false(self, capsys):
        """If no subproc has tracking enabled, print nothing and don't touch envs."""
        vec = _FakeVecEnv([_fake_subproc(False, {}, 0, [], {}) for _ in range(3)])
        aggregate_and_print_obs_min_max(vec)
        assert capsys.readouterr().out == ""
        assert vec.method_calls == []

    def test_aggregator_merges_and_disables(self, capsys):
        """Per-feature min-of-mins, max-of-maxes, sum of steps; mute via env_method."""
        features = ["f1"]
        bounds = {"f1": (-10.0, 10.0)}
        # Three subprocs with overlapping ranges. Global min = -3, global max = 7.
        per_env = [
            _fake_subproc(True, {"f1": {"min": -1.0, "max": 5.0}}, 100, features, bounds),
            _fake_subproc(True, {"f1": {"min": -3.0, "max": 4.0}}, 200, features, bounds),
            _fake_subproc(True, {"f1": {"min": 0.0, "max": 7.0}}, 300, features, bounds),
        ]
        vec = _FakeVecEnv(per_env)

        aggregate_and_print_obs_min_max(vec)
        out = capsys.readouterr().out

        # Step counts summed across subprocs.
        assert "Tracked over 600 timesteps" in out
        # Merged min/max land in the rec columns (printed via %12.4f).
        assert "-3.0000" in out
        assert "7.0000" in out
        # Mute path: must use env_method, not set_attr — guards the Monitor-wrapper bug.
        assert vec.method_calls == [("disable_obs_min_max_recording", ())]


class TestMergeObsMinMax:
    """Direct tests of the snapshot-producing merge function."""

    def test_returns_none_when_all_disabled(self):
        vec = _FakeVecEnv([_fake_subproc(False, {}, 0, [], {}) for _ in range(2)])
        assert merge_obs_min_max(vec) is None

    def test_snapshot_shape_and_merge(self):
        features = ["f1", "f2"]
        bounds = {"f1": (-1.0, 1.0)}  # f2 deliberately unbounded
        per_env = [
            _fake_subproc(
                True, {"f1": {"min": -2.0, "max": 0.5}, "f2": {"min": 10.0, "max": 20.0}}, 50, features, bounds
            ),
            _fake_subproc(
                True,
                {"f1": {"min": -1.0, "max": 3.0}, "f2": {"min": 5.0, "max": 15.0}},
                150,
                features,
                bounds,
                normalize_obs=False,
            ),
        ]
        snap = merge_obs_min_max(_FakeVecEnv(per_env))

        assert snap["total_steps"] == 200
        assert snap["bounds"] == bounds
        # normalize_obs is read from index 0 only.
        assert snap["normalize_obs"] is True
        assert snap["merged"]["f1"] == {"min": -2.0, "max": 3.0}
        assert snap["merged"]["f2"] == {"min": 5.0, "max": 20.0}


class _StubVecEnv:
    """Minimal VecEnv stub for callback tests — only what merge_obs_min_max touches."""

    def __init__(self, snapshot):
        # snapshot=None → tracking disabled; otherwise drives merge_obs_min_max output.
        self._snapshot = snapshot
        self.num_envs = 1

    def get_attr(self, name, indices=None):
        if name == "record_obs_min_max":
            return [self._snapshot is not None]
        if name == "obs_min_max_tracker":
            return [self._snapshot["tracker"]]
        if name == "obs_tracker_step_count":
            return [self._snapshot["steps"]]
        if name == "observation_type":
            return [SimpleNamespace(features=self._snapshot["features"], bounds=self._snapshot["bounds"])]
        if name == "normalize_obs":
            return [True]
        raise AssertionError(f"unexpected get_attr({name!r})")


def _make_callback(tmp_path, snapshot, save_freq=100):
    cb = ObsMinMaxSnapshotCallback(snapshot_path=str(tmp_path / "snap.yaml"), save_freq=save_freq)
    # training_env is a property reading model.get_env(); stub the model.
    cb.model = SimpleNamespace(get_env=lambda env=_StubVecEnv(snapshot): env)
    cb.num_timesteps = 0
    return cb


class TestObsMinMaxSnapshotCallback:
    def test_snapshot_writes_yaml_and_logs_violations(self, tmp_path, monkeypatch):
        import yaml as _yaml

        logged = []
        monkeypatch.setattr("train.callbacks.wandb.log", lambda m, step=None: logged.append((m, step)))

        cb = _make_callback(
            tmp_path,
            {
                "tracker": {"f1": {"min": -2.0, "max": 5.0}, "f2": {"min": 0.0, "max": 1.0}},
                "steps": 42,
                "features": ["f1", "f2"],
                "bounds": {"f1": (-1.0, 1.0)},  # f2 unbounded → skipped in metrics
            },
        )
        cb.num_timesteps = 1234
        cb._snapshot()

        payload = _yaml.safe_load(open(cb.snapshot_path))
        assert payload["total_steps"] == 42
        assert payload["features"]["f1"] == {"min": -2.0, "max": 5.0}

        assert len(logged) == 1
        metrics, step = logged[0]
        assert step == 1234
        assert metrics["obs_bounds/f1/over"] == 4.0  # 5 - 1
        assert metrics["obs_bounds/f1/under"] == 1.0  # -1 - (-2)
        assert not any(k.startswith("obs_bounds/f2/") for k in metrics)

    def test_snapshot_noop_when_disabled(self, tmp_path, monkeypatch):
        logged = []
        monkeypatch.setattr("train.callbacks.wandb.log", lambda m, step=None: logged.append(m))
        cb = _make_callback(tmp_path, snapshot=None)
        cb._snapshot()
        assert not os.path.exists(cb.snapshot_path)
        assert logged == []

    def test_on_step_respects_save_freq(self, tmp_path):
        cb = _make_callback(tmp_path, snapshot=None, save_freq=100)
        calls = []
        cb._snapshot = lambda: calls.append(cb.num_timesteps)

        cb.num_timesteps = 50
        cb._on_step()
        assert calls == []  # below save_freq

        cb.num_timesteps = 100
        cb._on_step()
        assert calls == [100]

        cb.num_timesteps = 150
        cb._on_step()
        assert calls == [100]  # only 50 since last snapshot

        cb.num_timesteps = 200
        cb._on_step()
        assert calls == [100, 200]

        cb._on_training_end()
        assert calls == [100, 200, 200]  # always snapshots at end
