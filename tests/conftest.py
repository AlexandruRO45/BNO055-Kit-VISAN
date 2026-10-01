"""Shared fixtures: synthetic IMU sessions and candump captures (no hardware)."""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import pytest


def make_sample(wall_ns: int, mono_ns: int, phase: float, clock_ok: bool = True) -> dict:
    return {
        "t_wall_ns": wall_ns,
        "t_mono_ns": mono_ns,
        "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
        "head": 0.0, "roll": 0.0, "pitch": 0.0,
        "lin_acc": [0.1 * np.sin(phase), 0.0, 9.81],
        "gyro": [0.5 * np.sin(phase), 0.2 * np.cos(phase * 0.7), 0.0],
        "cal": [3, 3, 3, 3],
        "c": 1.0,
        "clock_ok": clock_ok,
    }


@pytest.fixture
def cal_dict() -> dict:
    return {
        "accel_offset": [-17, -21, -27],
        "mag_offset": [133, 91, -248],
        "gyro_offset": [-2, -3, -1],
        "accel_radius": 1000,
        "mag_radius": 764,
    }


@pytest.fixture
def imu_session(tmp_path: Path, cal_dict: dict) -> Path:
    """A synthetic session dir: 10 s @ 100 Hz sinusoidal motion.

    The session's wall stamps are the monotonic stamps shifted by a fixed
    (synthetic) wall/mono offset — exactly what the daemon writes.
    """
    session_dir = tmp_path / "bno_test_session"
    session_dir.mkdir()
    start_wall_ns = int(time.time() * 1e9)
    start_mono_ns = 1_000_000_000
    manifest = {
        "schema_version": 1,
        "session_id": "bno_test_session",
        "boot_id": "test-boot-id",
        "hostname": "test",
        "pid": 1,
        "start_wall_ns": start_wall_ns,
        "start_mono_ns": start_mono_ns,
        "spawn_jitter_s": 1e-5,
        "i2c_bus": 7,
        "i2c_address": 0x29,
        "rate_hz": 100.0,
        "cal_file": "active.json",
        "cal_sha256": "0" * 64,
        "cal_status_at_start": [3, 3, 3, 3],
        "end_reason": "stop_requested",
        "samples_written": 1000,
    }
    (session_dir / "session.json").write_text(json.dumps(manifest), encoding="utf-8")

    with open(session_dir / "imu.jsonl", "w", encoding="utf-8") as f:
        for i in range(1000):
            mono_ns = start_mono_ns + i * 10_000_000  # 100 Hz
            wall_ns = start_wall_ns + i * 10_000_000
            phase = 2 * np.pi * (i / 1000) * 3  # 3 Hz motion
            f.write(json.dumps(make_sample(wall_ns, mono_ns, phase)) + "\n")
    return session_dir


def can_line(t_wall_s: float, iface: str, arbitration_id: int, data: bytes) -> str:
    # Real candump -L prints local time; the fixture mirrors that contract so
    # the parser round-trips in any timezone.
    ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t_wall_s)) + \
        f".{int((t_wall_s % 1) * 1e6):06d}"
    return f"({ts})  {iface}  {arbitration_id:03X}#{data.hex().upper()}\n"


def _i16be_payload(value_lsb: float) -> bytes:
    """8-byte payload: one int16 BE signal value (DBC v3, Motorola), rest zero."""
    lsb = max(-32768, min(32767, int(round(value_lsb))))
    return lsb.to_bytes(2, "big", signed=True) + bytes(6)


def gyro_payload(mag_rad_s: float) -> bytes:
    """Gyro frame 0x387: int16 BE, 0.01 deg/s per LSB (DBC v3)."""
    deg = mag_rad_s * 180.0 / np.pi
    return _i16be_payload(deg / 0.01)


def accel_payload(mag_m_s2: float) -> bytes:
    """Accel frame 0x386: int16 BE, 0.01 m/s^2 per LSB (DBC v3)."""
    return _i16be_payload(mag_m_s2 / 0.01)


@pytest.fixture
def candump_capture(tmp_path: Path, imu_session: Path) -> tuple[Path, float]:
    """A candump -L capture replaying the same motion, offset by +0.250 s.

    Returns (capture_path, injected_offset) where injected_offset is
    can_wall - imu_wall for the same physical instants.
    """
    manifest = json.loads((imu_session / "session.json").read_text(encoding="utf-8"))
    start_wall = manifest["start_wall_ns"] / 1e9
    injected_offset = 0.250  # CAN stamps land 250 ms after the IMU wall stamps

    capture = tmp_path / "can.candump"
    with open(capture, "w", encoding="utf-8") as f:
        for i in range(1000):
            t = start_wall + i * 0.01 + injected_offset
            phase = 2 * np.pi * (i / 1000) * 3
            gyro_mag = float(np.linalg.norm([0.5 * np.sin(phase), 0.2 * np.cos(phase * 0.7), 0.0]))
            f.write(can_line(t, "can0", 0x387, gyro_payload(gyro_mag)))
            accel_mag = float(np.linalg.norm([0.1 * np.sin(phase), 0.0, 9.81]))
            f.write(can_line(t, "can0", 0x386, accel_payload(accel_mag)))
    return capture, injected_offset
