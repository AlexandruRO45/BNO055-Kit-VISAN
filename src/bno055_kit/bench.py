"""Bench metrics shared by ``bench`` and post-calibration verification.

Ported from the legacy bench tooling (``imu.py do_bench``): at rest, a good
calibration shows near-zero linear-accel bias, near-zero unwrapped heading
drift, and low heading noise. ``score = bias + |drift| + noise`` (lower is
better) — the same ranking key the legacy ``best`` command used.
"""
from __future__ import annotations

import time
from collections.abc import Callable, Sequence

import numpy as np


def bench_metrics(heads: Sequence[float], accs: Sequence[Sequence[float]],
                  duration_s: float) -> dict:
    """Score a bench window: bias (m/s^2), drift (deg/min), noise (deg)."""
    heads_arr = np.degrees(np.unwrap(np.radians(np.asarray(heads, dtype=float))))
    drift_per_min = float((heads_arr[-1] - heads_arr[0]) * 60.0 / duration_s)
    # Skip the first ~0.5 s (mode-switch settle); guard short windows so the
    # slice can never be empty (mean of [] would be NaN).
    accs_arr = np.asarray(accs[max(50, len(accs) // 4):], dtype=float)
    bias = float(np.linalg.norm(accs_arr, axis=1).mean())
    head_noise = float(np.degrees(np.std(np.radians(heads_arr))))
    return {
        "bias_linacc_m_s2": round(bias, 4),
        "drift_deg_per_min": round(drift_per_min, 3),
        "head_noise_deg": round(head_noise, 4),
        "score": round(bias + abs(drift_per_min) + head_noise, 4),
    }


def sample_bench_window(sensor, duration_s: float, *,
                        sleep: Callable[[float], None] = time.sleep,
                        monotonic: Callable[[], float] = time.monotonic,
                        dt: float = 0.01) -> tuple[list[float], list[tuple]]:
    """Sample heading + linear accel at ~100 Hz for ``duration_s`` seconds."""
    t0 = monotonic()
    heads, accs = [], []
    while monotonic() - t0 < duration_s:
        s = sensor.read()
        heads.append(s.euler[0])
        accs.append(s.lin_acc)
        sleep(dt)
    return heads, accs


def make_record(cal_file: str, cal_sha256: str | None, metrics: dict, *,
                when_utc: str | None = None) -> dict:
    """History record: legacy keys plus the kit's sha256/ISO-8601 extras."""
    if when_utc is None:
        when_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    return {
        "cal_file": str(cal_file),
        "cal_sha256": cal_sha256,
        "when_utc": when_utc,
        **metrics,
    }
