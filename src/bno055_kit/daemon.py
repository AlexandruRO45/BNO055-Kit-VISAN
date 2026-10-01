"""Headless BNO055 recorder daemon (systemd Type=simple, SIGINT/SIGTERM stop).

Behaviour:
  * load the shipped calibration BEFORE opening a session — if the
    calibration is missing or invalid the daemon exits with EX_CONFIG (78)
    and records nothing (better no data than wrong data);
  * sample at a fixed rate with drift-free scheduling;
  * on a transient I2C read failure, reopen with bounded exponential
    backoff; if reconnects are exhausted, exit non-zero and let systemd
    restart the unit;
  * stamp every sample with the (wall_ns, mono_ns) pair and latch
    ``clock_ok=false`` for the rest of the session if CLOCK_REALTIME steps.
"""
from __future__ import annotations

import errno
import logging
import signal
import time
from dataclasses import dataclass

from .calibration import Calibration, CalibrationError, load_calibration
from .clock import SlewDetector, stamp_pair
from .config import Config
from .recorder import SessionWriter, open_session
from .sensor import Bno055

log = logging.getLogger("bno055_kit.daemon")

EX_OK = 0
EX_CONFIG = 78  # refused to run (bad/missing calibration) — systemd keeps it failed
EX_SENSOR_LOST = 1


@dataclass
class DaemonStats:
    samples: int = 0
    read_errors: int = 0
    reconnects: int = 0


class Daemon:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._stop_requested = False
        self.stats = DaemonStats()

    # ------------------------------------------------------------------
    def _sleep_interruptible(self, seconds: float, slice_s: float = 1.0) -> None:
        """Sleep up to ``seconds`` in ``slice_s`` slices, returning early on stop."""
        deadline = time.monotonic() + seconds
        while not self._stop_requested:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(slice_s, remaining))

    def _install_signal_handlers(self) -> None:
        def _handler(signum, _frame):
            log.info("received signal %d — stopping", signum)
            self._stop_requested = True

        signal.signal(signal.SIGINT, _handler)
        signal.signal(signal.SIGTERM, _handler)

    def _connect_and_calibrate(self) -> tuple[Bno055, Calibration]:
        cal = load_calibration(self.cfg.cal_file)
        log.info(
            "calibration loaded: %s (sha256=%s)", cal.source, (cal.sha256 or "")[:12]
        )
        sensor = Bno055(bus=self.cfg.bus, address=self.cfg.address)
        sensor.connect()
        sensor.apply_calibration(cal)
        return sensor, cal

    def _is_transient_i2c(self, exc: Exception) -> bool:
        # EREMOTEIO (121, "Remote I/O error") is the Linux SMBus result of a
        # slave NACK / clock-stretch — the single most common transient BNO055
        # failure on blinka — so it must reconnect, not kill the session.
        return isinstance(exc, OSError) and getattr(exc, "errno", None) in (
            errno.EIO,
            errno.ENXIO,
            errno.EREMOTEIO,
            errno.ETIMEDOUT,
            errno.EAGAIN,
            errno.EBUSY,
        )

    # ------------------------------------------------------------------
    def run(self) -> int:
        self._install_signal_handlers()
        try:
            sensor, cal = self._connect_and_calibrate()
        except CalibrationError as exc:
            log.error("refusing to record — %s", exc)
            return EX_CONFIG
        except Exception as exc:
            log.error("sensor unavailable at start: %s: %s", type(exc).__name__, exc)
            return EX_SENSOR_LOST

        try:
            writer: SessionWriter = open_session(
                self.cfg.log_dir,
                bus=self.cfg.bus,
                address=self.cfg.address,
                rate_hz=self.cfg.rate_hz,
                cal_file=str(cal.source),
                cal_sha256=cal.sha256,
                cal_status=sensor.read().cal,
            )
        except Exception as exc:
            # Session creation / first read failed (transient I2C, read-only
            # log dir): exit non-zero and let systemd restart the unit rather
            # than escape run() as an uncaught traceback.
            log.error("could not open session: %s: %s", type(exc).__name__, exc)
            return EX_SENSOR_LOST
        slew = SlewDetector(threshold_s=self.cfg.slew_threshold_s)
        dt = 1.0 / self.cfg.rate_hz
        next_t = time.monotonic() + dt

        log.info(
            "recording at %.1f Hz -> %s", self.cfg.rate_hz, writer.session_dir
        )
        try:
            while not self._stop_requested:
                try:
                    sample = sensor.read()
                    wall_ns, mono_ns = stamp_pair()
                    clock_ok = slew.observe(wall_ns, mono_ns)
                    writer.write_sample(wall_ns, mono_ns, sample, clock_ok)
                    self.stats.samples += 1
                except Exception as exc:
                    self.stats.read_errors += 1
                    if not self._is_transient_i2c(exc):
                        raise
                    if not self._recover(sensor, cal):
                        writer.finish(
                            {"end_reason": "sensor_lost", **_stats_dict(self.stats)}
                        )
                        return EX_SENSOR_LOST
                next_t += dt
                delay = next_t - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:  # fell behind (e.g. reconnect backoff): reschedule
                    next_t = time.monotonic()
        except Exception:
            log.exception("fatal error — closing session")
            writer.finish({"end_reason": "error", **_stats_dict(self.stats)})
            return 1

        writer.finish({"end_reason": "stop_requested", **_stats_dict(self.stats)})
        log.info("stopped cleanly after %d samples", self.stats.samples)
        return EX_OK

    # ------------------------------------------------------------------
    def _recover(self, sensor: Bno055, cal: Calibration) -> bool:
        """Bounded exponential-backoff reopen. True if the sensor is back."""
        delay = self.cfg.reconnect_backoff_s
        for attempt in range(1, self.cfg.max_reconnects + 1):
            if self._stop_requested:
                log.info("stop requested during reconnect — abandoning recovery")
                return False
            self.stats.reconnects += 1
            log.warning(
                "I2C read failed — reconnect attempt %d/%d in %.1fs",
                attempt,
                self.cfg.max_reconnects,
                delay,
            )
            # Sleep in <=1 s slices so a SIGTERM/SIGINT stop lands well within
            # TimeoutStopSec instead of being SIGKILL'd mid-backoff.
            self._sleep_interruptible(delay)
            if self._stop_requested:
                return False
            try:
                sensor.close()
                sensor.connect()
                sensor.apply_calibration(cal)
                sensor.read()
                log.info("sensor recovered on attempt %d", attempt)
                return True
            except Exception as exc:
                log.warning("reconnect failed: %s: %s", type(exc).__name__, exc)
                delay = min(delay * 2, self.cfg.reconnect_backoff_max_s)
        log.error("sensor lost after %d reconnect attempts", self.cfg.max_reconnects)
        return False


def _stats_dict(stats: DaemonStats) -> dict:
    return {
        "samples_written": stats.samples,
        "read_errors": stats.read_errors,
        "reconnects": stats.reconnects,
    }


def run_daemon(cfg: Config) -> int:
    return Daemon(cfg).run()
