"""Tests for the candump -L parser and IMU frame decode."""
from __future__ import annotations

from datetime import datetime

from bno055_kit.analysis.candump import decode_imu_frame, parse_line
from conftest import accel_payload, can_line, gyro_payload


def test_parse_classic_line():
    frame = parse_line("(2026-10-01 11:10:15.123456)  vcan0  384#1122334455667788")
    assert frame is not None
    assert frame.iface == "vcan0"
    assert frame.arbitration_id == 0x384
    assert frame.data == bytes.fromhex("1122334455667788")
    # candump -L stamps are local time: expected value computed the same way.
    expected = datetime(2026, 10, 1, 11, 10, 15).timestamp() + 0.123456
    assert abs(frame.t_wall_s - expected) < 1e-6


def test_parse_epoch_ts_form():
    frame = parse_line("(1790853015.5)  can0  387#0102000000000000")
    assert frame is not None
    assert abs(frame.t_wall_s - 1790853015.5) < 1e-9


def test_parse_bare_epoch_ts_form():
    frame = parse_line("1790853015.5 can0 387#0102000000000000")
    assert frame is not None
    assert frame.arbitration_id == 0x387


def test_parse_rtr_and_junk():
    frame = parse_line("(2026-10-01 11:10:15.000000)  vcan0  123#R8")
    assert frame is not None and frame.is_rtr and frame.data == b""
    assert parse_line("garbage line") is None
    assert parse_line("") is None


def test_decode_gyro_scaling():
    frame = parse_line("(2026-10-01 11:10:15.000000)  vcan0  387#" +
                      gyro_payload(1.0).hex().upper())
    dec = decode_imu_frame(frame)
    assert dec is not None and dec["kind"] == "gyro"
    # 1.0 rad/s -> 57.2958 deg/s -> 5730 LSB (0.01 deg/s) -> back to rad/s
    assert abs(dec["gx"] - 1.0) < 1e-3


def test_decode_accel_scaling():
    payload = accel_payload(9.81)
    frame = parse_line(f"(2026-10-01 11:10:15.000000)  vcan0  386#{payload.hex().upper()}")
    dec = decode_imu_frame(frame)
    assert dec is not None and dec["kind"] == "accel"
    assert abs(dec["ax"] - 9.81) < 0.01


def test_decode_ignores_other_ids():
    frame = parse_line("(2026-10-01 11:10:15.000000)  vcan0  385#0000000000000000")
    assert decode_imu_frame(frame) is None


def test_can_line_round_trip():
    line = can_line(1790853015.123456, "vcan0", 0x386, b"\x01\x02")
    frame = parse_line(line)
    assert frame is not None
    assert abs(frame.t_wall_s - 1790853015.123456) < 1e-6
    assert frame.data == b"\x01\x02"
