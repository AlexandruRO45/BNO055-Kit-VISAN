#!/usr/bin/env python3
"""start_kit.py — manual start/stop/status helper for the BNO055 kit service.

For bench use; at boot the unit starts on its own (WantedBy=multi-user.target).

Usage:
    start_kit.py status|start|stop|restart|logs [--unit bno055-imu.service]

Exit codes: 0 ok, 1 command failed, 2 bad arguments.
"""
from __future__ import annotations

import argparse
import subprocess
import sys

ACTIONS = {
    "status": ["systemctl", "status", "{unit}", "--no-pager", "-n", "30"],
    "start": ["systemctl", "start", "{unit}"],
    "stop": ["systemctl", "stop", "{unit}"],
    "restart": ["systemctl", "restart", "{unit}"],
    "logs": ["journalctl", "-u", "{unit}", "--no-pager", "-n", "100", "--follow"],
}


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="start_kit", description=__doc__.splitlines()[0])
    p.add_argument("action", choices=sorted(ACTIONS))
    p.add_argument("--unit", default="bno055-imu.service")
    args = p.parse_args(argv)

    cmd = [a.format(unit=args.unit) for a in ACTIONS[args.action]]
    print("$ " + " ".join(cmd), flush=True)
    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
