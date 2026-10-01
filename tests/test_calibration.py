"""Tests for calibration loading/validation."""
from __future__ import annotations

import json

import pytest

from bno055_kit.calibration import Calibration, CalibrationError, file_sha256, load_calibration


def test_round_trip(cal_dict):
    cal = Calibration.from_dict(cal_dict, source="unit-test")
    assert cal.accel_offset == (-17, -21, -27)
    assert cal.mag_radius == 764
    assert cal.to_dict()["accel_offset"] == [-17, -21, -27]


def test_missing_key_rejected(cal_dict):
    del cal_dict["gyro_offset"]
    with pytest.raises(CalibrationError, match="gyro_offset"):
        Calibration.from_dict(cal_dict)


def test_out_of_range_offset_rejected(cal_dict):
    cal_dict["accel_offset"] = [0, 0, 40000]
    with pytest.raises(CalibrationError, match="int16"):
        Calibration.from_dict(cal_dict)


def test_negative_radius_rejected(cal_dict):
    cal_dict["mag_radius"] = -1
    with pytest.raises(CalibrationError, match="mag_radius"):
        Calibration.from_dict(cal_dict)


def test_load_from_file(tmp_path, cal_dict):
    path = tmp_path / "cal.json"
    path.write_text(json.dumps(cal_dict), encoding="utf-8")
    cal = load_calibration(path)
    assert cal.sha256 == file_sha256(path)
    assert cal.source == str(path)


def test_load_missing_file(tmp_path):
    with pytest.raises(CalibrationError, match="not found"):
        load_calibration(tmp_path / "nope.json")


def test_load_bad_json(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(CalibrationError, match="not valid JSON"):
        load_calibration(path)


def test_shipped_calibrations_validate():
    """The calibs shipped in the kit bundle must load cleanly."""
    from pathlib import Path

    kit_calibs = Path(__file__).resolve().parents[1] / "calibs"
    for name in ("active.json", "cal_20261001_114721.json"):
        cal = load_calibration(kit_calibs / name)
        assert cal.accel_radius > 0 and cal.mag_radius > 0
