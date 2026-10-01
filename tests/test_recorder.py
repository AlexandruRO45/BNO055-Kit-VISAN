"""Tests for the session writer and manifest (no hardware needed)."""
from __future__ import annotations

import json

from bno055_kit.recorder import SessionWriter, make_manifest, session_id_from
from bno055_kit.sensor import Sample


def _sample() -> Sample:
    return Sample(
        quat=(1.0, 0.0, 0.0, 0.0),
        euler=(10.5, 1.25, -2.5),
        lin_acc=(0.01, 0.02, 9.80),
        gyro=(0.001, 0.0, -0.002),
        cal=(3, 3, 3, 2),
    )


def test_sample_confidence():
    assert _sample().confidence == (3 + 3 + 3 + 2) / 12.0
    assert Sample((0, 0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0, 0)).confidence == 0.0


def test_session_id_format():
    # 2026-10-01 11:10:15 UTC
    assert session_id_from(1790853015_123456789) == "bno_20261001T111015Z"


def test_session_write_round_trip(tmp_path):
    writer = SessionWriter(tmp_path / "sess")
    manifest = make_manifest(
        session_id="sess",
        wall_ns=1790881815_000000000,
        mono_ns=5_000_000_000,
        spawn_jitter_s=1e-5,
        bus=7,
        address=0x29,
        rate_hz=100.0,
        cal_file="/var/lib/bno055/active.json",
        cal_sha256="a" * 64,
        cal_status=(3, 3, 3, 3),
    )
    writer.start(manifest)
    writer.write_sample(1790881815_010000000, 5_010_000_000, _sample(), clock_ok=True)
    writer.write_sample(1790881815_020000000, 5_020_000_000, _sample(), clock_ok=False)
    writer.finish({"end_reason": "stop_requested", "samples_written": 2})

    written = json.loads((writer.session_dir / "session.json").read_text(encoding="utf-8"))
    assert written["start_wall_ns"] == 1790881815_000000000
    assert written["end_reason"] == "stop_requested"
    assert written["samples_written"] == 2
    assert "end_mono_ns" in written

    rows = [json.loads(line) for line in
            (writer.session_dir / "imu.jsonl").read_text(encoding="utf-8").splitlines() if line]
    assert len(rows) == 2
    assert rows[0]["clock_ok"] is True and rows[1]["clock_ok"] is False
    assert rows[0]["t_mono_ns"] < rows[1]["t_mono_ns"]
    assert rows[0]["c"] == round(11 / 12, 4)  # cal=(3,3,3,2)
    assert set(rows[0]) == {
        "t_wall_ns", "t_mono_ns", "qw", "qx", "qy", "qz", "head", "roll", "pitch",
        "lin_acc", "gyro", "cal", "c", "clock_ok",
    }
