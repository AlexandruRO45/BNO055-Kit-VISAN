"""Operator CLI for the BNO055 kit (scriptable; the daemon runs under systemd).

    bno055-kit scan     — probe the I2C bus for the sensor
    bno055-kit run      — run the recorder in the foreground (systemd ExecStart)
    bno055-kit bench    — quantify the active calibration at rest (bias/drift/noise)
    bno055-kit status   — show active calibration + last session summary
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path

from .calibration import CalibrationError, load_calibration
from .config import Config
from .recorder import MANIFEST_NAME
from .sensor import Bno055, probe_i2c

log = logging.getLogger("bno055_kit.cli")


def _setup_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        stream=sys.stdout,
    )


# --------------------------------------------------------------------- scan
def cmd_scan(args: argparse.Namespace) -> int:
    bus = args.bus if args.bus is not None else Config.load(args.config).bus
    addr = probe_i2c(bus)
    if addr is None:
        print(f"[FAIL] no BNO055 (CHIP_ID 0xA0) on i2c-{bus} at 0x28/0x29")
        print(f"        check wiring and run: sudo i2cdetect -y -r {bus}")
        return 1
    print(f"[OK]   BNO055 answers at 0x{addr:02x} on i2c-{bus} (CHIP_ID=0xa0)")
    return 0


# ---------------------------------------------------------------------- run
def cmd_run(args: argparse.Namespace) -> int:
    from .daemon import run_daemon

    cfg = Config.load(args.config)
    if args.bus is not None:
        cfg.bus = args.bus
    if args.address is not None:
        cfg.address = args.address
    if args.rate is not None:
        cfg.rate_hz = args.rate
    if args.log_dir is not None:
        cfg.log_dir = args.log_dir
    if args.cal_file is not None:
        cfg.cal_file = args.cal_file
    return run_daemon(cfg)


# --------------------------------------------------------------------- bench
def cmd_bench(args: argparse.Namespace) -> int:
    import numpy as np

    try:
        cal = load_calibration(args.cal_file)
    except CalibrationError as exc:
        print(f"[FAIL] {exc}")
        return 1

    cfg = Config.load(args.config)
    bus = args.bus if args.bus is not None else cfg.bus
    address = args.address if args.address is not None else cfg.address
    if args.time < 2.0:
        print("[FAIL] --time too short for a meaningful bench (use >= 2 s)")
        return 2

    sensor = Bno055(bus=bus, address=address)
    sensor.connect()
    sensor.apply_calibration(cal)

    print(f"[*] bench {args.time:.0f}s — keep the board perfectly still ...")
    t0 = time.monotonic()
    heads, accs = [], []
    while time.monotonic() - t0 < args.time:
        s = sensor.read()
        heads.append(s.euler[0])
        accs.append(s.lin_acc)
        time.sleep(0.01)

    heads_arr = np.degrees(np.unwrap(np.radians(np.asarray(heads, dtype=float))))
    drift_per_min = float((heads_arr[-1] - heads_arr[0]) * 60.0 / args.time)
    # Skip the first ~0.5 s (mode-switch settle); guard short windows so the
    # slice can never be empty (mean of [] would be NaN).
    accs_arr = np.asarray(accs[max(50, len(accs) // 4):], dtype=float)
    bias = float(np.linalg.norm(accs_arr, axis=1).mean())
    head_noise = float(np.degrees(np.std(np.radians(heads_arr))))
    score = round(bias + abs(drift_per_min) + head_noise, 4)

    rec = {
        "cal_file": str(cal.source),
        "cal_sha256": cal.sha256,
        "when_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "bias_linacc_m_s2": round(bias, 4),
        "drift_deg_per_min": round(drift_per_min, 3),
        "head_noise_deg": round(head_noise, 4),
        "score": score,
    }
    print(json.dumps(rec, indent=2))
    print(f"[score {score} — lower is better]")
    return 0


# -------------------------------------------------------------------- status
def cmd_status(args: argparse.Namespace) -> int:
    cfg = Config.load(args.config)
    rc = 0
    try:
        cal = load_calibration(cfg.cal_file)
        print(f"[OK]   active calibration: {cfg.cal_file}")
        print(f"       sha256={cal.sha256[:16]}...  source={cal.source}")
        print(f"       accel_offset={list(cal.accel_offset)} "
              f"gyro_offset={list(cal.gyro_offset)} mag_offset={list(cal.mag_offset)} "
              f"radii={cal.accel_radius}/{cal.mag_radius}")
    except CalibrationError as exc:
        print(f"[FAIL] calibration: {exc}")
        rc = 1

    log_dir = Path(cfg.log_dir)
    sessions = sorted((d for d in log_dir.iterdir() if d.is_dir()), reverse=True) \
        if log_dir.is_dir() else []
    if sessions:
        manifest = sessions[0] / MANIFEST_NAME
        if manifest.is_file():
            data = json.loads(manifest.read_text(encoding="utf-8"))
            print(f"[OK]   last session: {sessions[0].name}")
            for k in ("start_wall_ns", "start_mono_ns", "samples_written",
                      "end_reason", "clock_ok", "cal_status_at_start"):
                if k in data:
                    print(f"       {k} = {data[k]}")
        else:
            print(f"[WARN] last session {sessions[0].name} has no {MANIFEST_NAME}")
    else:
        print(f"[--]   no sessions under {log_dir}")
    return rc


# ---------------------------------------------------------------------- main
def _parse_args(p: argparse.ArgumentParser,
                argv: list[str] | None) -> argparse.Namespace:
    """Parse argv, accepting global flags before *or* after the subcommand.

    argparse only knows --bus/--address/--config/--log-level on the root
    parser, so `bno055-kit run --config X` would die with
    "unrecognized arguments". Operate (and the systemd wrapper) naturally
    put the flags after the verb, so retry with globals hoisted in front
    of the subcommand rather than depending on one invocation order.
    """
    argv = list(sys.argv[1:] if argv is None else argv)
    first_err: SystemExit | None = None
    try:
        return p.parse_args(argv)
    except SystemExit as err:  # noqa: PERF203 - retry path below
        first_err = err

    globals_: list[str] = []
    rest: list[str] = []
    i = 0
    while i < len(argv):
        tok = argv[i]
        if tok in ("--bus", "--address", "--config", "--log-level"):
            globals_.extend((tok, argv[i + 1]))
            i += 2
        elif tok.startswith(("--bus=", "--address=", "--config=", "--log-level=")):
            globals_.append(tok)
            i += 1
        else:
            rest.append(tok)
            i += 1
    if not globals_:
        raise first_err  # not a global-flag problem: report the original error
    return p.parse_args(globals_ + rest)


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="bno055-kit",
                                description="BNO055 kit recorder (VISAN companion)")
    p.add_argument("--bus", type=int, default=None, help="I2C bus (default from config)")
    p.add_argument("--address", type=lambda x: int(x, 0), default=None,
                   help="I2C address, e.g. 0x29 (default from config)")
    p.add_argument("--config", default=None, help="YAML config file")
    p.add_argument("--log-level", default="INFO")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("scan", help="probe the I2C bus for the sensor")

    pr = sub.add_parser("run", help="run the recorder (foreground; systemd ExecStart)")
    pr.add_argument("--rate", type=float, default=None, help="sample rate Hz")
    pr.add_argument("--log-dir", default=None, help="session log root dir")
    pr.add_argument("--cal-file", default=None, help="active calibration file")

    pb = sub.add_parser("bench", help="quantify the active calibration at rest")
    pb.add_argument("--time", type=float, default=30.0, help="seconds at rest")
    pb.add_argument("--cal-file", default=None, help="calibration file to bench")

    sub.add_parser("status", help="show active calibration + last session")
    return p


def main(argv: list[str] | None = None) -> int:
    p = _build_parser()
    args = _parse_args(p, argv)
    _setup_logging(args.log_level)

    if args.cmd == "scan":
        return cmd_scan(args)
    if args.cmd == "run":
        return cmd_run(args)
    if args.cmd == "bench":
        if args.cal_file is None:
            args.cal_file = Config.load(args.config).cal_file
        return cmd_bench(args)
    if args.cmd == "status":
        return cmd_status(args)
    raise AssertionError(f"unhandled command {args.cmd!r}")


if __name__ == "__main__":
    sys.exit(main())
