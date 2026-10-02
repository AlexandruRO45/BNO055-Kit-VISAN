#!/usr/bin/env python3
"""start_kit.py — manual start/stop/status helper for the BNO055 kit service.

For bench use; at boot the unit starts on its own (WantedBy=multi-user.target).

Usage:
    start_kit.py status|start|stop|restart|logs [--unit bno055-imu.service]
    start_kit.py calibrate [--unit ...] [extra cli calibrate args...]
    start_kit.py best [--unit ...]

``calibrate`` is the on-Kit operator toggle: it stops the recorder, runs the
interactive calibration (sensor already glued to the drone), and ALWAYS
restarts the unit afterwards — including on Ctrl-C or failure — so the
drone never sits unrecorded. ``best`` promotes the lowest-score session
from history.jsonl to the active calibration and restarts the unit.

Exit codes: 0 ok, 1 command failed, 2 bad arguments.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from bno055_kit.calibration import (  # noqa: E402
    CalibrationError,
    load_calibration,
    pick_best,
    read_history,
    save_calibration,
)

ACTIONS = {
    "status": ["systemctl", "status", "{unit}", "--no-pager", "-n", "30"],
    "start": ["systemctl", "start", "{unit}"],
    "stop": ["systemctl", "stop", "{unit}"],
    "restart": ["systemctl", "restart", "{unit}"],
    "logs": ["journalctl", "-u", "{unit}", "--no-pager", "-n", "100", "--follow"],
}

DEFAULT_UNIT = "bno055-imu.service"
DEFAULT_CONFIG = "/etc/bno055/bno055_imu.yaml"


def _systemd(*args: str) -> int:
    cmd = ["systemctl", *args]
    print("$ " + " ".join(cmd), flush=True)
    return subprocess.call(cmd)


def _kit_cli(argv: list[str]) -> int:
    """Run the kit CLI in the app venv when present, else this interpreter."""
    venv_py = Path("/opt/bno055/.venv/bin/python")
    exe = str(venv_py) if venv_py.is_file() else sys.executable
    cmd = [exe, "-m", "bno055_kit.cli", *argv]
    print("$ " + " ".join(cmd), flush=True)
    return subprocess.call(cmd)


def cmd_calibrate(unit: str, config: str, extra: list[str]) -> int:
    """Stop the recorder, calibrate interactively, ALWAYS restart recording."""
    _systemd("stop", unit)
    rc = 1
    try:
        rc = _kit_cli(["--config", config, "calibrate", "--force", "--unit",
                       unit, *extra])
    except KeyboardInterrupt:
        print("\n[ABORT] calibration interrupted — nothing saved.")
        rc = 130
    finally:
        # Recording must resume even on abort/failure: an unrecorded drone
        # is the worse failure mode, and the boot path re-applies whatever
        # active.json holds.
        restart_rc = _systemd("start", unit)
        if rc == 0 and restart_rc != 0:
            print(f"[FAIL] calibration saved but {unit} did not start "
                  f"(systemctl start rc={restart_rc})")
            rc = 1
    if rc == 0:
        print(f"[OK]   calibration complete — {unit} restarted, "
              "next session uses the new calibration.")
    return rc


def cmd_best(unit: str, config: str) -> int:
    from bno055_kit.config import Config

    cfg = Config.load(config if Path(config).is_file() else None)
    hist_path = Path(cfg.cal_file).parent / "history.jsonl"
    records = read_history(hist_path)
    if not records:
        print(f"[!] no history at {hist_path} — run: "
              f"python3 scripts/ops/start_kit.py calibrate")
        return 1
    ranked = sorted((r for r in records if r.get("score") is not None
                     and Path(r["cal_file"]).exists()),
                    key=lambda r: r["score"])
    if not ranked:
        print(f"[!] history at {hist_path} exists but no record has a score "
              "and a readable cal file.")
        return 1
    print(f"{'#':>2} {'score':>8} {'bias m/s2':>10} {'drift deg/min':>14} "
          f"{'noise deg':>10}  file")
    for i, r in enumerate(ranked, 1):
        print(f"{i:>2} {r['score']:>8} {r['bias_linacc_m_s2']:>10} "
              f"{r['drift_deg_per_min']:>14} {r['head_noise_deg']:>10}  "
              f"{r['cal_file']}")
    best = pick_best(records)
    src = best["cal_file"]
    try:
        cal = load_calibration(src)
    except CalibrationError as exc:
        print(f"[FAIL] winner unreadable: {exc}")
        return 1
    sha = save_calibration(cal, cfg.cal_file)
    print(f"\n[OK]   active calibration <- {src} (score {best['score']}) "
          f"sha256={sha[:12]}...")
    _systemd("stop", unit)
    _systemd("start", unit)
    print(f"[OK]   {unit} restarted with the promoted calibration.")
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="start_kit", description=__doc__.splitlines()[0])
    p.add_argument("action", choices=sorted([*ACTIONS, "calibrate", "best"]))
    p.add_argument("--unit", default=DEFAULT_UNIT)
    p.add_argument("--config", default=DEFAULT_CONFIG)
    args, extra = p.parse_known_args(argv)

    if args.action == "calibrate":
        return cmd_calibrate(args.unit, args.config, extra)
    if args.action == "best":
        return cmd_best(args.unit, args.config)

    if extra:
        p.error(f"unrecognized arguments: {' '.join(extra)}")
    cmd = [a.format(unit=args.unit) for a in ACTIONS[args.action]]
    print("$ " + " ".join(cmd), flush=True)
    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
