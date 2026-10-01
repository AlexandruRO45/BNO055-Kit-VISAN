# BNO055 Kit — boot-time IMU recorder for the VISAN Kit

A standardised, headless companion to the VISAN flight stack: a BNO055
I2C IMU daemon that starts **independently at boot** as its own systemd
unit, records every sample with a paired **CLOCK_REALTIME / CLOCK_MONOTONIC
nanosecond stamp**, and ships with an offline analysis script that aligns
its timeline against a VISAN `candump -L` capture — compensating for
pipeline jitter — so the two logs can be correlated post-flight.

This bundle is the productionised successor of the bench tooling in the
parent directory (`imu.py` / `imu_cli.py`), which stays untouched as the
interactive calibration tool. The calibration schema is byte-compatible:
the winning bench calibration is shipped in `calibs/` and loaded at boot.

## Design doctrine (inherited from VISAN)

- **Better no data than wrong data**: the daemon refuses to record if the
  calibration is missing or invalid (exit `EX_CONFIG=78`, unit stays failed).
- **Monotonic ns for math, wall clock for archival**: every sample carries
  both stamps; a `clock_ok` flag latches false if CLOCK_REALTIME steps
  (NTP) mid-session, mirroring the VISAN sub-source envelope contract.
- **Anchor pairs**: each session's `session.json` records a tight
  (wall, mono) anchor pair at start — the same reasoning as the VISAN
  flight recorder's `sync.json` — so the IMU's monotonic timeline can be
  projected onto the candump wall-clock timeline.

## Layout

```
kit/
  src/bno055_kit/            importable package
    clock.py                   paired clock stamps + NTP-step (slew) detection
    calibration.py             load/validate/hash the shipped calibration
    sensor.py                  BNO055 wrapper (lazy blinka imports, reopen-safe)
    recorder.py                session dirs: imu.jsonl + session.json manifest
    daemon.py                  fixed-rate record loop, signals, reconnect policy
    cli.py                     scan | run | bench | status
    analysis/candump.py        candump -L parser + HirrusUAS IMU frame decode
    analysis/sync.py           IMU<->CAN clock alignment (offset + jitter)
  scripts/deploy/
    systemd/bno055-imu.service     independent boot unit (multi-user.target)
    systemd/bno055_imu_wrapper.sh  systemd exec-wrapper
    install_kit.sh                 dry-run-default installer (own venv)
  scripts/ops/
    start_kit.py                   manual start/stop/status/logs helper
    health_check.py                one-shot Kit health report
  scripts/analysis/
    sync_candump.py                align a session with a candump capture
  configs/bno055_imu.yaml          daemon config (installed to /etc/bno055)
  calibs/                          shipped calibration + bench history
  tests/                           hardware-free unit tests (pytest)
```

## On-Kit runtime layout

| Path | Purpose |
|---|---|
| `/opt/bno055` | app root, own venv at `/opt/bno055/.venv` (VISAN venv carries no I2C deps) |
| `/var/lib/bno055/active.json` | active calibration (`StateDirectory`) |
| `/var/log/bno055/<session>/` | `imu.jsonl` samples + `session.json` manifest (`LogsDirectory`) |
| `/etc/bno055/bno055_imu.yaml` | daemon config |
| `/run/bno055` | runtime dir (`RuntimeDirectory`) |

The unit runs as `visan:visan` with `SupplementaryGroups=i2c` and is
**independent of `visan.service`** — the IMU timeline must cover the whole
boot, not only VISAN's lifetime.

## Install (on the Kit)

```bash
sudo bash scripts/deploy/install_kit.sh            # dry-run, shows every step
sudo bash scripts/deploy/install_kit.sh --confirm  # apply
sudo systemctl start bno055-imu.service            # or just reboot
```

Like the VISAN deb, install **enables** the unit for boot but does not
start it; the boot path is the tested path.

## Calibrate (bench, interactive)

Calibration stays an operator action on the bench, using the parent-dir
tooling (`python3 imu.py calibrate` → `best`). The winning file is copied
to `calibs/active.json` here and shipped to the Kit; the daemon only
*loads* it, never auto-calibrates in flight.

## Time-sync a session against a candump capture

```bash
python3 scripts/analysis/sync_candump.py \
    /var/log/bno055/bno_20261001T111015Z \
    /mnt/nvme-app/visan_flight_sessions/<session>/can.candump \
    --out sync.json
```

Two methods, tried in trust order (`--method auto` default):

1. **correlation** (preferred): decodes the HirrusUAS accel/gyro frames
   (IDs 0x386/0x387, DBC v3 scaling) from the capture, resamples both
   magnitude series onto a common grid, and cross-correlates to recover the
   true pipeline offset — no shared clock assumed.
2. **anchor** (fallback): takes the residual from each sample's real wall
   stamp to the nearest candump frame; assumes one shared CLOCK_REALTIME
   (an NTP step inside the session shows up in the residual spread and the
   `clock_ok` note).

Output manifest: `offset_s` (add to IMU time to reach CAN time),
`jitter_std_s`, `max_abs_residual_s`, `n`, `method`, `confidence`, `notes`.

Timestamp contract: `candump -L` prints **local** wall-clock time; the
parser interprets it as local time (the Kit and the analysis host must
share a timezone, or use `candump -t a` epoch captures, which are also
accepted).

## Develop / verify (host, no hardware)

```bash
pip install -e .[dev]
ruff check .
pytest -q
```

The test suite is hardware-free: calibration validation, slew detection,
candump parsing, and offset recovery on synthetic shifted series.
