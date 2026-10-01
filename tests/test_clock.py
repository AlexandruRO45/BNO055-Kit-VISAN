"""Tests for the clock helpers and slew detection."""
from __future__ import annotations

from bno055_kit.clock import SlewDetector, now_mono_ns, now_wall_ns, stamp_pair


def test_stamp_pair_consistent():
    wall, mono = stamp_pair()
    assert wall > mono  # CLOCK_REALTIME epoch >> boot-relative monotonic
    assert now_mono_ns() >= mono
    assert now_wall_ns() >= wall


def test_no_step_reports_ok():
    det = SlewDetector(threshold_s=0.05)
    for i in range(10):
        ok = det.observe(wall_ns=1_000_000_000_000 + i * 10_000_000,
                         mono_ns=i * 10_000_000)
        assert ok
    assert det.clock_ok
    assert det.slew_wall_ns is None


def test_step_detected_and_latched():
    det = SlewDetector(threshold_s=0.05)
    w0 = 1_777_000_000_000_000_000  # ~2026 epoch, ns
    m0 = 5_000_000_000_000          # ~1.4 h uptime, ns
    det.observe(w0, m0)
    det.observe(w0 + 10_000_000, m0 + 10_000_000)
    # NTP steps the wall clock forward by 2 s in one sample.
    assert not det.observe(w0 + 20_000_000 + 2_000_000_000, m0 + 20_000_000)
    assert det.slewing
    assert det.slew_delta_ns == 2_000_000_000
    # Latched: even a clean sample afterwards stays not-ok.
    assert not det.observe(w0 + 30_000_000 + 2_000_000_000, m0 + 30_000_000)
    assert not det.clock_ok


def test_small_drift_under_threshold_ignored():
    det = SlewDetector(threshold_s=0.05)
    w0 = 1_777_000_000_000_000_000
    m0 = 5_000_000_000_000
    det.observe(w0, m0)
    # 10 ms delta change — normal NTP discipline, not a step.
    assert det.observe(w0 + 10_000_000 + 10_000_000, m0 + 10_000_000)
    assert det.clock_ok
