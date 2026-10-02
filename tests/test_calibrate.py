"""Hardware-free tests for the on-Kit calibration procedure and bench metrics."""
from __future__ import annotations

import math

import pytest

from bno055_kit.bench import bench_metrics, make_record
from bno055_kit.calibrate import (
    FACES,
    CalibrationAborted,
    CalibrationTimeout,
    dominant_axis,
    run_calibration,
    sample_face,
    validate_face,
)

G = 9.81


def face_mean(axis: int, sign: float = 1.0) -> tuple[float, float, float]:
    v = [0.0, 0.0, 0.0]
    v[axis] = sign * G
    return (v[0], v[1], v[2])


# ------------------------------------------------------------- validate_face
def test_dominant_axis():
    assert dominant_axis((0.1, -9.8, 0.2)) == 1
    assert dominant_axis((9.8, 0.0, 0.0)) == 0


def test_validate_face_ok():
    res = validate_face(face_mean(2, +1), std_max=0.1, axis=2)
    assert res.ok and res.sign == 1


def test_validate_face_magnitude_gate():
    res = validate_face((0, 0, 5.0), std_max=0.1, axis=2)
    assert not res.ok and "1g" in res.reason


def test_validate_face_axis_gate():
    res = validate_face(face_mean(0), std_max=0.1, axis=2)
    assert not res.ok and "dominant axis" in res.reason


def test_validate_face_jitter_gate():
    res = validate_face(face_mean(2), std_max=0.9, axis=2)
    assert not res.ok and "jitter" in res.reason


def test_validate_face_opposite_sign_gate():
    res = validate_face(face_mean(2, +1), std_max=0.1, axis=2, ref_sign=+1)
    assert not res.ok and "OPPOSITE" in res.reason
    res = validate_face(face_mean(2, -1), std_max=0.1, axis=2, ref_sign=+1)
    assert res.ok and res.sign == -1


# --------------------------------------------------------------- sample_face
class AccelSensor:
    """Replays a scripted list of accel vectors, then repeats the last."""

    def __init__(self, means):
        self._means = list(means)
        self._i = 0

    def raw_acceleration(self):
        v = self._means[min(self._i, len(self._means) - 1)]
        self._i += 1
        return v


def test_sample_face_stats():
    sensor = AccelSensor([(0.0, 0.0, G)] * 25)
    mean, std_max = sample_face(sensor, hold_s=0.5, dt=0.05,
                                sleep=lambda s: None)
    assert std_max < 1e-12
    assert all(abs(m) < 1e-12 for m in mean[:2])
    assert math.isclose(mean[2], G, abs_tol=1e-12)


# ------------------------------------------------------------------- bench
def test_bench_metrics_perfect():
    m = bench_metrics([0.0] * 200, [(0.0, 0.0, 0.0)] * 200, duration_s=30.0)
    assert m["score"] == 0.0


def test_bench_metrics_drift():
    heads = [i * 0.01 for i in range(300)]  # ~3 deg over 30 s -> ~6 deg/min
    m = bench_metrics(heads, [(0.1, 0.0, 0.0)] * 300, duration_s=30.0)
    assert m["drift_deg_per_min"] > 1.0
    assert m["score"] > 1.0


def test_make_record_keys():
    rec = make_record("cal.json", "abc", {"score": 1.0},
                      when_utc="2026-10-02T00:00:00Z")
    assert rec["cal_file"] == "cal.json"
    assert rec["cal_sha256"] == "abc"
    assert rec["when_utc"] == "2026-10-02T00:00:00Z"
    assert rec["score"] == 1.0


# --------------------------------------------------------- run_calibration
class FakeSample:
    def __init__(self, lin_acc, cal):
        self.lin_acc = lin_acc
        self.cal = cal


class ScriptedSensor:
    """Full fake for run_calibration: scripted face samples, levels driven
    by the prompts (each prompt converges the phase it announces)."""

    SAMPLES_PER_FACE = 5  # hold_s=0.5 / dt=0.1 in the tests

    def __init__(self, face_means=None):
        if face_means is None:
            face_means = [face_mean(2, +1), face_mean(2, -1), face_mean(0, +1),
                          face_mean(0, -1), face_mean(1, +1), face_mean(1, -1)]
        self._means = [m for m in face_means for _ in range(self.SAMPLES_PER_FACE)]
        self.levels = [0, 0, 0, 0]  # sys, gyro, accel, mag
        self.captured = None
        self._i = 0

    def read(self):
        return FakeSample((0.0, 0.0, G), tuple(self.levels))

    def raw_acceleration(self):
        v = self._means[min(self._i, len(self._means) - 1)]
        self._i += 1
        return v

    def capture_calibration(self, source=None, calibrated_at_utc=None):
        from bno055_kit.calibration import Calibration

        self.captured = Calibration(accel_offset=(-1, -2, -3),
                                    mag_offset=(4, 5, 6),
                                    gyro_offset=(7, 8, 9),
                                    accel_radius=1000, mag_radius=700,
                                    source=source,
                                    calibrated_at_utc=calibrated_at_utc)
        return self.captured


def phase_prompt(sensor):
    """input() replacement: each prompt converges the phase it announces.

    Levels must converge *at or before* the prompt that precedes their
    wait loop: gyro at step 1, accel during the faces, mag+sys during the
    figure-8 (the sys wait follows the mag wait, before the capture
    prompt).
    """
    def _input(_msg=""):
        if "STEP 1/3" in _msg:
            sensor.levels[1] = 3      # gyro
        elif "FACE" in _msg:
            sensor.levels[2] = 3      # accel (converges during the faces)
        elif "STEP 3/3" in _msg:
            sensor.levels[3] = 3      # mag
            sensor.levels[0] = 3      # sys follows during the figure-8
        return ""

    return _input


def _run(sensor, prompt, **kw):
    kwargs = dict(out=lambda *a, **k: None, sleep=lambda s: None,
                  monotonic=lambda: 0.0, hold_s=0.5, dt=0.1,
                  timeout_s=1e9)
    kwargs.update(kw)
    return run_calibration(sensor, prompt=prompt, **kwargs)


def test_run_calibration_full_success():
    sensor = ScriptedSensor()
    cal = _run(sensor, phase_prompt(sensor))
    assert cal is sensor.captured
    assert cal.accel_offset == (-1, -2, -3)
    assert cal.source == "on-kit calibrate"


def test_run_calibration_abort_on_eof():
    sensor = ScriptedSensor()

    def boom(_msg=""):
        raise EOFError

    with pytest.raises(CalibrationAborted):
        _run(sensor, boom)


def test_run_calibration_timeout_gyro():
    sensor = ScriptedSensor()  # levels stay 0 -> gyro never converges
    with pytest.raises(CalibrationTimeout, match="gyro"):
        _run(sensor, lambda _m: "", timeout_s=-1.0)


def test_run_calibration_too_few_faces():
    # every face garbage -> 0/6 validated -> incomplete
    sensor = ScriptedSensor(face_means=[(0.0, 0.0, 1.0)] * 6)
    with pytest.raises(CalibrationTimeout, match="faces validated"):
        _run(sensor, phase_prompt(sensor))


def test_run_calibration_skips_bad_face():
    # Z1 garbage on all 3 attempts (15 samples) is skipped; the other five
    # faces validate -> 5/6 >= min_faces(4) -> success (legacy behaviour).
    bad = (0.0, 0.0, 1.0)
    good = [face_mean(2, -1), face_mean(0, +1), face_mean(0, -1),
            face_mean(1, +1), face_mean(1, -1)]
    sensor = ScriptedSensor()
    sensor._means = [bad] * (3 * sensor.SAMPLES_PER_FACE) + \
        [m for m in good for _ in range(sensor.SAMPLES_PER_FACE)]
    cal = _run(sensor, phase_prompt(sensor))
    assert cal is sensor.captured


def test_faces_table_matches_legacy():
    assert tuple(f[0] for f in FACES) == ("Z1", "Z2", "X1", "X2", "Y1", "Y2")
    assert tuple(f[2] for f in FACES) == (2, 2, 0, 0, 1, 1)
