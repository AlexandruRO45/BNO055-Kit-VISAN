"""Parse ``candump -L`` logs into wall-clock-stamped CAN frames.

``candump -L <iface>`` (what the VISAN flight recorder spawns) writes one
frame per line::

    (2026-10-01 11:10:15.123456)  vcan0  384#1122334455667788

The timestamp is CLOCK_REALTIME (local time, microsecond resolution) — there
is no monotonic stamp in a candump log, which is exactly why the IMU session
manifest carries a wall/mono anchor pair: the anchor maps the IMU's
monotonic timeline onto this wall-clock timeline.

Supported timestamp forms (tolerant parser — unknown lines are skipped):
  * ``(YYYY-MM-DD HH:MM:SS.ffffff)``  — candump -L (default)
  * ``(1234567890.123456)``           — candump -t a (epoch seconds)
  * ``1234567890.123456``             — candump -t a without parens

Frame bodies: classic ``<id>#<hexdata>``, RTR ``<id>#R<n>``, and CAN-FD
``<id>##<flags><hexdata>``.
"""
from __future__ import annotations

import re
from collections.abc import Iterator
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

# (timestamp)  iface  id#data   — iface optional in some captures
LINE_RE = re.compile(
    r"^\s*(?:\((?P<ts>[^)]+)\)|(?P<ts_bare>\d+\.\d+))\s+(?P<iface>\S+)\s+(?P<frame>\S+)\s*$"
)
TS_DATE_RE = re.compile(r"^(?P<y>\d{4})-(?P<mo>\d{2})-(?P<d>\d{2}) "
                        r"(?P<h>\d{2}):(?P<mi>\d{2}):(?P<s>\d{2})\.(?P<frac>\d+)$")
TS_EPOCH_RE = re.compile(r"^\d+\.\d+$")
FRAME_RE = re.compile(r"^(?P<id>[0-9A-Fa-f]{3,8})#(?P<body>.*)$")


@dataclass(frozen=True)
class CandumpFrame:
    t_wall_s: float  # CLOCK_REALTIME seconds (float) — candump has no monotonic stamp
    iface: str
    arbitration_id: int
    data: bytes
    is_rtr: bool = False
    is_fd: bool = False


def _parse_ts(text: str) -> float | None:
    m = TS_EPOCH_RE.match(text)
    if m:
        return float(text)
    m = TS_DATE_RE.match(text)
    if not m:
        return None
    # candump -L prints CLOCK_REALTIME in *local* time (strftime %F %T).
    # A naive datetime's .timestamp() interprets it as local time, which is
    # exactly the contract; assuming UTC would shift every stamp by the
    # zone offset on any Kit not running UTC.
    g = m.groupdict()
    dt = datetime(
        int(g["y"]), int(g["mo"]), int(g["d"]),
        int(g["h"]), int(g["mi"]), int(g["s"]),
        tzinfo=None,
    )
    frac = g["frac"].ljust(9, "0")[:9]  # normalise to nanoseconds
    return dt.timestamp() + int(frac) / 1e9


def parse_line(line: str) -> CandumpFrame | None:
    m = LINE_RE.match(line)
    if not m:
        return None
    ts_text = m.group("ts") or m.group("ts_bare")
    if ts_text is None:
        return None
    t_wall_s = _parse_ts(ts_text.strip())
    if t_wall_s is None:
        return None

    fm = FRAME_RE.match(m.group("frame"))
    if not fm:
        return None
    body = fm.group("body")
    arbitration_id = int(fm.group("id"), 16)

    is_rtr = False
    is_fd = body.startswith("#")  # classic '#' separator consumed; '##' => FD
    if body.startswith("R"):
        is_rtr = True
        data = b""
    elif is_fd:
        payload = body[1:]  # drop FD flags nibble
        data = bytes.fromhex(payload) if payload else b""
    elif body:
        try:
            data = bytes.fromhex(body)
        except ValueError:
            return None
    else:
        data = b""

    return CandumpFrame(
        t_wall_s=t_wall_s,
        iface=m.group("iface"),
        arbitration_id=arbitration_id,
        data=data,
        is_rtr=is_rtr,
        is_fd=is_fd,
    )


def parse_candump(path: str | Path) -> Iterator[CandumpFrame]:
    """Yield frames in file order. Blank/unparseable lines are skipped."""
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if not line.strip():
                continue
            frame = parse_line(line)
            if frame is not None:
                yield frame


def decode_imu_frame(frame: CandumpFrame) -> dict | None:
    """Decode the HirrusUAS CAN IMU frames VISAN records (IDs 0x384-0x387).

    Scaling and byte order follow ``DBC_HirrusUAS_VISAN_v3.dbc`` exactly:
    all signals are 16-bit Motorola (``@0``, big-endian) with scale 0.01.
    Only the fields needed for time-domain correlation are decoded:
      0x384 (900) attitude : Pitch, Roll, Yaw (bytes 0-1, 2-3, 4-5; deg)
      0x386 (902) accel    : X, Y, Z          (bytes 0-1, 2-3, 4-5; m/s^2)
      0x387 (903) gyro     : X, Y, Z          (bytes 0-1, 2-3, 4-5; deg/s)
    Anything else returns None. Gyro is additionally converted to rad/s so it
    is directly comparable with the BNO055 sample units.
    """
    import math

    def be16s(data: bytes, off: int) -> int:
        return int.from_bytes(data[off : off + 2], "big", signed=True)

    n = frame.arbitration_id
    d = frame.data
    if n == 0x384 and len(d) >= 6:
        return {
            "kind": "attitude",
            "t_wall_s": frame.t_wall_s,
            "pitch_deg": be16s(d, 0) * 0.01,
            "roll_deg": be16s(d, 2) * 0.01,
            "yaw_deg": be16s(d, 4) * 0.01,
        }
    if n == 0x386 and len(d) >= 6:
        return {
            "kind": "accel",
            "t_wall_s": frame.t_wall_s,
            "ax": be16s(d, 0) * 0.01,
            "ay": be16s(d, 2) * 0.01,
            "az": be16s(d, 4) * 0.01,
        }
    if n == 0x387 and len(d) >= 6:
        dps2rads = math.pi / 180.0
        return {
            "kind": "gyro",
            "t_wall_s": frame.t_wall_s,
            "gx": be16s(d, 0) * 0.01 * dps2rads,
            "gy": be16s(d, 2) * 0.01 * dps2rads,
            "gz": be16s(d, 4) * 0.01 * dps2rads,
        }
    return None
