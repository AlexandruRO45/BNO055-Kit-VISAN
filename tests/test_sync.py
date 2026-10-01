"""Tests for IMU/CAN clock alignment (offset recovery on synthetic data)."""
from __future__ import annotations

import json

from bno055_kit.analysis import align
from bno055_kit.analysis.sync import load_can_series, load_imu_session


def _session_start_wall(session_dir) -> float:
    manifest = json.loads((session_dir / "session.json").read_text(encoding="utf-8"))
    return manifest["start_wall_ns"] / 1e9


def test_correlation_recovers_injected_offset(imu_session, candump_capture):
    capture, injected_offset = candump_capture
    imu = load_imu_session(imu_session)
    can = load_can_series(capture)
    result = align(imu, can, method="correlation", fs=50.0, max_lag_s=1.0)
    assert result.method == "correlation"
    # Grid resolution 1/50 s = 20 ms; parabolic refinement beats that.
    assert abs(result.offset_s - injected_offset) < 0.010
    assert result.confidence in ("medium", "high")
    assert result.n > 100


def test_anchor_recovers_zero_offset(imu_session, tmp_path):
    """Capture stamped on the same wall clock as the session => offset ~0."""
    from conftest import can_line

    start_wall = _session_start_wall(imu_session)
    capture = tmp_path / "aligned.candump"
    with open(capture, "w", encoding="utf-8") as f:
        for i in range(500):
            f.write(can_line(start_wall + i * 0.02, "can0", 0x7AA, bytes(8)))
    imu = load_imu_session(imu_session)
    can = load_can_series(capture)
    result = align(imu, can, method="anchor")
    assert result.method == "anchor"
    # Nearest-frame residual bounded by the 20 ms capture spacing.
    assert abs(result.offset_s) < 0.010
    assert result.jitter_std_s < 0.010


def test_auto_falls_back_to_anchor(imu_session, tmp_path):
    """No IMU frames in the capture => correlation impossible => anchor used."""
    capture = tmp_path / "noimu.candump"
    capture.write_text(
        "(2026-10-01 11:10:15.000000)  can0  123#0000000000000000\n"
        "(2026-10-01 11:10:16.000000)  can0  123#0000000000000000\n",
        encoding="utf-8",
    )
    imu = load_imu_session(imu_session)
    can = load_can_series(capture)
    result = align(imu, can, method="auto")
    assert result.method == "anchor"
    assert result.confidence == "low"


def test_correlation_needs_overlap(imu_session, tmp_path):
    """A capture with too few frames must raise, so auto can fall back."""
    capture = tmp_path / "tiny.candump"
    capture.write_text(
        "(2026-10-01 11:10:15.000000)  can0  387#0000000000000000\n",
        encoding="utf-8",
    )
    imu = load_imu_session(imu_session)
    can = load_can_series(capture)
    import pytest

    from bno055_kit.analysis.sync import align_correlation

    with pytest.raises(ValueError, match="no overlapping"):
        align_correlation(imu, can)
