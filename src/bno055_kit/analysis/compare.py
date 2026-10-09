"""Side-by-side comparison of a BNO055 session against decoded CAN IMU signals.

The sync step (:mod:`bno055_kit.analysis.sync`) answers *when* the two
timelines line up (``can_wall = imu_wall + offset_s``). This module answers
*what* the two sensors actually disagree about: for every IMU sample it
resamples each CAN signal at the (offset-corrected) sample instant and emits
one row per (sample, signal) pair, plus per-signal summary statistics.

The default is a pure *as-is* comparison: every BNO value is shown next to
the matching CAN value at the same instant, and the delta is the raw
frame-to-frame difference. The two sensors sit in different body frames, so
those deltas are the real mounting relationship — never silently hidden by a
transform. The transforms are opt-in conveniences, recorded in the manifest:

``euler_convention``
    ``"none"`` (default) compares BNO head/roll/pitch against CAN
    Yaw/Roll/Pitch as-is. ``"ned_to_nwu"`` rotates the CAN heading by -90 deg
    (yaw_NWU = yaw_NED - 90, wrapped to [-180, 180)) for NED airframes.

``accel_convention``
    ``"raw"`` (default) compares BNO lin_acc (gravity removed) as-is.
    ``"add_gravity"`` adds [0, 0, +9.80665] m/s^2 to lin_acc so it lines up
    with the CAN accelerometer, which includes gravity.

``gyro_convention``
    ``"none"`` (default) compares as-is. ``"negate"`` flips all three axes;
    ``"rot_x180"`` applies the measured airframe mounting diag(1,-1,-1)
    (180 deg about X). BNO gyro is published in rad/s and converted to deg/s
    either way.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .candump import decode_imu_frame, parse_candump

GRAVITY_Z = 9.80665
RAD2DEG = 180.0 / np.pi

# CAN signal name -> BNO component index into EULER_NAMES = (head, roll, pitch).
EULER_MAP = {"Yaw": 0, "Roll": 1, "Pitch": 2}
ACC_MAP = {"Acceleration_X": 0, "Acceleration_Y": 1, "Acceleration_Z": 2}
ASPEED_MAP = {"AngSpeedX": 0, "AngSpeedY": 1, "AngSpeedZ": 2}

EULER_NAMES = ("head", "roll", "pitch")
CONVENTIONS = {
    "euler": ("none", "ned_to_nwu"),
    "accel": ("raw", "add_gravity"),
    "gyro": ("none", "negate", "rot_x180"),
}

# Per-axis sign applied to the BNO gyro before comparison. rot_x180 is the
# mounting measured on the first real flight pair (gyro corr vs CAN was
# diag(+1,-1,-1): X tracks as-is, Y/Z are negated).
GYRO_SIGNS = {
    "none": (1.0, 1.0, 1.0),
    "negate": (-1.0, -1.0, -1.0),
    "rot_x180": (1.0, -1.0, -1.0),
}


@dataclass
class SignalSeries:
    """One physical signal sampled on one timeline, with its unit."""

    name: str
    unit: str
    t_s: np.ndarray
    values: np.ndarray


@dataclass
class ImuSignalSeries:
    """All BNO sample series, keyed by component name (head/roll/pitch, x/y/z)."""

    series: dict[str, SignalSeries]
    session_id: str | None = None
    n_bad_lines: int = 0


@dataclass
class CanSignalSeries:
    """Decoded CAN IMU series, keyed by kind then DBC signal name."""

    by_kind: dict[str, dict[str, SignalSeries]] = field(default_factory=dict)
    n_frames: int = 0

    def kind_for(self, wanted: tuple[str, ...]) -> str | None:
        for kind in wanted:
            if kind in self.by_kind:
                return kind
        return None


@dataclass
class SignalSummary:
    name: str
    unit: str
    bno_key: str
    can_signal: str
    n: int
    rmse: float
    max_abs_delta: float
    mean_delta: float

    def to_dict(self) -> dict:
        return {
            "bno_key": self.bno_key,
            "can_signal": self.can_signal,
            "unit": self.unit,
            "n": self.n,
            "rmse": round(self.rmse, 6),
            "max_abs_delta": round(self.max_abs_delta, 6),
            "mean_delta": round(self.mean_delta, 6),
        }


@dataclass
class CompareResult:
    offset_s: float
    method: str
    confidence: str
    conventions: dict[str, str]
    summaries: list[SignalSummary]
    imu_session_id: str | None = None
    notes: list[str] = field(default_factory=list)

    def to_summary_dict(self) -> dict:
        return {
            "schema_version": 1,
            "imu_session_id": self.imu_session_id,
            "offset_s": round(self.offset_s, 6),
            "method": self.method,
            "confidence": self.confidence,
            "conventions": self.conventions,
            "signals": [s.to_dict() for s in self.summaries],
            "notes": self.notes,
        }


def _wrap_deg(x: np.ndarray) -> np.ndarray:
    return (x + 180.0) % 360.0 - 180.0


def load_imu_signals(session_dir: str | Path) -> ImuSignalSeries:
    """Read ``session.json`` + ``imu.jsonl`` into per-component series (wall clock)."""
    session_dir = Path(session_dir)
    manifest = json.loads((session_dir / "session.json").read_text(encoding="utf-8"))
    start_wall = int(manifest["start_wall_ns"])
    start_mono = int(manifest["start_mono_ns"])

    t_list, head, roll, pitch, acc, gyro = [], [], [], [], [], []
    n_bad = 0
    with open(session_dir / "imu.jsonl", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                n_bad += 1
                continue
            # Same projection as sync: monotonic timeline onto the wall clock.
            t_list.append(start_wall / 1e9 + (row["t_mono_ns"] / 1e9 - start_mono / 1e9))
            head.append(row["head"])
            roll.append(row["roll"])
            pitch.append(row["pitch"])
            acc.append(row["lin_acc"])
            gyro.append(row["gyro"])

    if not t_list:
        raise ValueError(f"no samples in {session_dir / 'imu.jsonl'}")

    t = np.asarray(t_list)
    eu = np.stack([np.asarray(head, float), np.asarray(roll, float),
                   np.asarray(pitch, float)])
    acc_a = np.asarray(acc, dtype=float)
    gyro_a = np.asarray(gyro, dtype=float)

    series: dict[str, SignalSeries] = {}
    for i, name in enumerate(EULER_NAMES):
        series[name] = SignalSeries(name, "deg", t, eu[i])
    for i, name in enumerate(("x", "y", "z")):
        series[f"acc_{name}"] = SignalSeries(f"acc_{name}", "m/s^2", t, acc_a[:, i])
        series[f"gyro_{name}"] = SignalSeries(f"gyro_{name}", "deg/s", t,
                                             gyro_a[:, i] * RAD2DEG)
    return ImuSignalSeries(series=series,
                           session_id=manifest.get("session_id"),
                           n_bad_lines=n_bad)


def load_can_signals(path: str | Path) -> CanSignalSeries:
    """Decode every IMU frame in a candump log into per-signal series."""
    cols: dict[str, dict[str, tuple[list, list]]] = {}
    n = 0
    for frame in parse_candump(path):
        n += 1
        dec = decode_imu_frame(frame)
        if dec is None:
            continue
        kind = dec["kind"]
        bucket = cols.setdefault(kind, {})
        for name, value in dec["signals"].items():
            bucket.setdefault(name, ([], []))
            bucket[name][0].append(dec["t_wall_s"])
            bucket[name][1].append(float(value))

    by_kind: dict[str, dict[str, SignalSeries]] = {}
    for kind, sigs in cols.items():
        by_kind[kind] = {
            name: SignalSeries(name, "", np.asarray(ts), np.asarray(vs))
            for name, (ts, vs) in sigs.items()
        }
    return CanSignalSeries(by_kind=by_kind, n_frames=n)



def compare(
    imu: ImuSignalSeries,
    can: CanSignalSeries,
    offset_s: float,
    *,
    method: str = "",
    confidence: str = "",
    euler_convention: str = "none",
    accel_convention: str = "raw",
    gyro_convention: str = "none",
    window_s: tuple[float, float] | None = None,
) -> tuple[CompareResult, list[dict]]:
    """Resample each CAN signal at the offset-corrected IMU instants and compare.

    ``offset_s`` is the sync manifest ``offset_s`` (add to IMU time to reach
    CAN time). Returns ``(result, rows)``: per-signal summaries plus the
    per-sample rows the caller writes as JSONL.
    """
    notes: list[str] = []
    if euler_convention not in CONVENTIONS["euler"]:
        raise ValueError(f"euler_convention must be one of {CONVENTIONS['euler']}")
    if accel_convention not in CONVENTIONS["accel"]:
        raise ValueError(f"accel_convention must be one of {CONVENTIONS['accel']}")
    if gyro_convention not in CONVENTIONS["gyro"]:
        raise ValueError(f"gyro_convention must be one of {CONVENTIONS['gyro']}")

    groups = [
        ("att", EULER_MAP, EULER_NAMES, "", euler_convention),
        ("acc", ACC_MAP, ("x", "y", "z"), "acc_", accel_convention),
        ("aspeed", ASPEED_MAP, ("x", "y", "z"), "gyro_", gyro_convention),
    ]

    summaries: list[SignalSummary] = []
    rows: list[dict] = []
    for kind, cmap, names, prefix, conv in groups:
        can_kind = can.kind_for((kind,))
        if can_kind is None:
            notes.append(f"no CAN {kind} frames in capture")
            continue
        for can_name, idx in cmap.items():
            imu_key = f"{prefix}{names[idx]}"
            if imu_key not in imu.series:
                continue
            imu_s = imu.series[imu_key]
            can_s = can.by_kind[can_kind][can_name]
            t_imu = imu_s.t_s + offset_s
            # window_s is relative to the start of the (offset-corrected) IMU
            # timeline, so it reads the same regardless of the clock offset.
            base = t_imu[0]
            lo, hi = (base, t_imu[-1]) if window_s is None else (base + window_s[0],
                                                               base + window_s[1])
            mask = (t_imu >= lo) & (t_imu <= hi)
            if not mask.any():
                notes.append(f"{imu_key}: no samples inside window")
                continue
            can_v = np.interp(t_imu[mask], can_s.t_s, can_s.values,
                             left=np.nan, right=np.nan)
            imu_v = imu_s.values[mask]
            if kind == "acc" and conv == "add_gravity" and idx == 2:
                imu_v = imu_v + GRAVITY_Z
            if kind == "att" and conv == "ned_to_nwu" and can_name == "Yaw":
                can_v = _wrap_deg(can_v - 90.0)
            if kind == "aspeed":
                imu_v = imu_v * GYRO_SIGNS[conv][idx]
            good = np.isfinite(can_v) & np.isfinite(imu_v)
            if good.sum() < 2:
                notes.append(f"{imu_key}: too few aligned points")
                continue
            tv, iv, cv = t_imu[mask][good], imu_v[good], can_v[good]
            # Attitude is angular: the shortest signed arc, so a BNO heading
            # of 359.9 vs a CAN yaw of 0.1 reads as -0.2, not -359.8.
            delta = _wrap_deg(iv - cv) if kind == "att" else iv - cv
            rows.extend(
                {"t": float(t), "signal": f"{imu_key}~{can_name}",
                 "bno": float(i), "can": float(c), "delta": float(d)}
                for t, i, c, d in zip(tv, iv, cv, delta, strict=True)
            )
            summaries.append(SignalSummary(
                name=f"{imu_key}~{can_name}", unit=imu_s.unit, bno_key=imu_key,
                can_signal=can_name, n=int(good.sum()),
                rmse=float(np.sqrt(np.mean(delta ** 2))),
                max_abs_delta=float(np.max(np.abs(delta))),
                mean_delta=float(np.mean(delta)),
            ))

    return CompareResult(
        offset_s=offset_s, method=method, confidence=confidence,
        conventions={"euler": euler_convention, "accel": accel_convention,
                     "gyro": gyro_convention},
        summaries=summaries, imu_session_id=imu.session_id, notes=notes,
    ), rows
