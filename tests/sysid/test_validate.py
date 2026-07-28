"""Unit tests for examples.analysis.sysid.validate helpers.

Covers the metrics-writing layer:
- `_format_per_bag_raw_mse_table`: column-sort independence, raw_MSE = contrib/coeff math.
- `_format_continuity_lines`: empty PASS branch + bag-tagged failure lines.
- `_write_metrics`: single-bag vs multi-bag header shape, diagnostic-table gating.

We exercise these helpers directly rather than through `main()` so the tests stay fast
and don't need a Rollout / GKEnv.
"""

from __future__ import annotations

import argparse

from examples.analysis.sysid.dataset import CHANNELS, Dataset
from examples.analysis.sysid.loss import CHANNEL_COEFFS
from examples.analysis.sysid.validate import (
    _format_continuity_lines,
    _format_per_bag_raw_mse_table,
    _write_metrics,
)


def _make_args(mirror: bool = False) -> argparse.Namespace:
    return argparse.Namespace(
        window_length_s=1.5,
        stride_s=0.5,
        min_speed=0.3,
        warmup_s=0.2,
        mirror=mirror,
    )


def _empty_dataset() -> Dataset:
    return Dataset(windows=[], dt=0.01, n_candidates=0, n_dropped_low_speed=0, n_dropped_nonfinite=0)


# -- _format_per_bag_raw_mse_table --


def test_per_bag_table_sorts_columns():
    """Column order must be sorted alphabetically by bag stem regardless of dict insertion order."""
    contribs = {ch: 1.0 for ch in CHANNELS}
    per_bag = {"z_bag": contribs, "m_bag": contribs, "a_bag": contribs}
    lines = _format_per_bag_raw_mse_table(per_bag)
    header = next(line for line in lines if "channel" in line and "a_bag" in line)
    assert header.index("a_bag") < header.index("m_bag") < header.index("z_bag")


def test_per_bag_table_raw_mse_matches_contrib_over_coeff():
    """raw_MSE printed in the table equals contribution / CHANNEL_COEFFS[ch]."""
    # Pick distinct contribs so the two bags' raw values are easy to verify.
    per_bag = {
        "bag_a": {ch: 0.6 for ch in CHANNELS},
        "bag_b": {ch: 0.3 for ch in CHANNELS},
    }
    lines = _format_per_bag_raw_mse_table(per_bag)
    # yaw_rate row carries the raw values for both bags.
    yaw_row = next(line for line in lines if line.lstrip().startswith("yaw_rate"))
    expected_a = 0.6 / CHANNEL_COEFFS["yaw_rate"]
    expected_b = 0.3 / CHANNEL_COEFFS["yaw_rate"]
    assert f"{expected_a:.6f}" in yaw_row
    assert f"{expected_b:.6f}" in yaw_row


def test_per_bag_table_lists_all_channels():
    per_bag = {"a": {ch: 0.1 for ch in CHANNELS}, "b": {ch: 0.2 for ch in CHANNELS}}
    lines = _format_per_bag_raw_mse_table(per_bag)
    body = "\n".join(lines)
    for ch in CHANNELS:
        assert ch in body, f"channel {ch!r} missing from raw_MSE table"


# -- _format_continuity_lines --


def test_continuity_empty_returns_pass():
    assert _format_continuity_lines([]) == ["yaw_continuity: PASS"]


def test_continuity_failures_include_bag_tag_and_t0_idx():
    failures = [("bagA", 100, 1.6, 0.5), ("bagB", 200, 0.7, 1.8)]
    lines = _format_continuity_lines(failures)
    # First line summarizes count.
    assert "2 window(s)" in lines[0]
    # Each failure becomes its own line with bag stem and t0_idx prefix.
    assert any("bag=bagA" in ln and "t0_idx=100" in ln for ln in lines[1:])
    assert any("bag=bagB" in ln and "t0_idx=200" in ln for ln in lines[1:])


# -- _write_metrics --


def test_write_metrics_single_bag_header(tmp_path):
    """Single-bag mode writes one `bag: <path>` line; no `bags:` block, no diagnostic table."""
    bag = tmp_path / "bagX.npz"
    bag.touch()
    out = tmp_path / "metrics.txt"
    _write_metrics(
        str(out),
        bags=[bag],
        params_source="SYSID_PARAMS",
        args=_make_args(),
        dataset=_empty_dataset(),
        total_loss=1.234,
        per_channel={ch: 0.0 for ch in CHANNELS},
        continuity_failures=[],
    )
    content = out.read_text()
    assert content.startswith(f"bag: {bag}\n")
    assert "bags:" not in content
    assert "per-bag raw_MSE" not in content
    assert "total_loss: 1.234000" in content


def test_write_metrics_multi_bag_header_and_diagnostic(tmp_path):
    """Multi-bag mode writes `bags:` block and the per-bag raw_MSE diagnostic table."""
    bag_a = tmp_path / "a.npz"
    bag_b = tmp_path / "b.npz"
    bag_a.touch()
    bag_b.touch()
    out = tmp_path / "metrics.txt"
    per_bag = {"a": {ch: 0.1 for ch in CHANNELS}, "b": {ch: 0.2 for ch in CHANNELS}}
    _write_metrics(
        str(out),
        bags=[bag_a, bag_b],
        params_source="SYSID_PARAMS",
        args=_make_args(),
        dataset=_empty_dataset(),
        total_loss=0.5,
        per_channel={ch: 0.0 for ch in CHANNELS},
        continuity_failures=[],
        per_bag_per_channel=per_bag,
    )
    content = out.read_text()
    assert content.startswith("bags:\n")
    assert f"  - {bag_a}" in content
    assert f"  - {bag_b}" in content
    assert "per-bag raw_MSE" in content


def test_write_metrics_skips_diagnostic_when_single_bag_in_per_bag_dict(tmp_path):
    """Diagnostic table is gated on len(per_bag_per_channel) > 1, not just `is not None`."""
    bag = tmp_path / "only.npz"
    bag.touch()
    out = tmp_path / "metrics.txt"
    _write_metrics(
        str(out),
        bags=[bag],
        params_source="SYSID_PARAMS",
        args=_make_args(),
        dataset=_empty_dataset(),
        total_loss=0.1,
        per_channel={ch: 0.0 for ch in CHANNELS},
        continuity_failures=[],
        per_bag_per_channel={"only": {ch: 0.1 for ch in CHANNELS}},
    )
    content = out.read_text()
    assert "per-bag raw_MSE" not in content


def test_write_metrics_continuity_failures_render(tmp_path):
    bag = tmp_path / "bag.npz"
    bag.touch()
    out = tmp_path / "metrics.txt"
    _write_metrics(
        str(out),
        bags=[bag],
        params_source="SYSID_PARAMS",
        args=_make_args(),
        dataset=_empty_dataset(),
        total_loss=0.0,
        per_channel={ch: 0.0 for ch in CHANNELS},
        continuity_failures=[("bag", 42, 1.7, 0.9)],
    )
    content = out.read_text()
    assert "WARN: yaw continuity check failed for 1 window(s)" in content
    assert "bag=bag" in content and "t0_idx=42" in content
