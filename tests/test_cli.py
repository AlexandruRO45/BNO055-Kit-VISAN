"""Hardware-free tests for the CLI argument parsing.

Regression: the systemd wrapper used to invoke
``bno055-kit run --config /etc/bno055/bno055_imu.yaml`` while ``--config``
is a root-parser flag, so every boot failed with
``error: unrecognized arguments: --config ...`` (exit 2, restart loop).
Global flags must parse on either side of the subcommand.
"""
from __future__ import annotations

from bno055_kit.cli import _build_parser, _parse_args


def test_global_flags_before_subcommand() -> None:
    p = _build_parser()
    args = _parse_args(p, ["--config", "/etc/bno055/bno055_imu.yaml", "run",
                           "--log-dir", "/var/log/bno055"])
    assert args.cmd == "run"
    assert args.config == "/etc/bno055/bno055_imu.yaml"
    assert args.log_dir == "/var/log/bno055"


def test_global_flags_after_subcommand() -> None:
    p = _build_parser()
    args = _parse_args(p, ["run", "--config", "/etc/bno055/bno055_imu.yaml",
                           "--log-dir", "/var/log/bno055"])
    assert args.cmd == "run"
    assert args.config == "/etc/bno055/bno055_imu.yaml"
    assert args.log_dir == "/var/log/bno055"


def test_global_flags_split_across_subcommand() -> None:
    p = _build_parser()
    args = _parse_args(p, ["--bus", "7", "bench", "--config", "cal.json",
                           "--time", "30"])
    assert args.cmd == "bench"
    assert args.bus == 7
    assert args.config == "cal.json"
    assert args.time == 30.0


def test_equals_style_global_flags_after_subcommand() -> None:
    p = _build_parser()
    args = _parse_args(p, ["scan", "--address=0x29"])
    assert args.cmd == "scan"
    assert args.address == 0x29


def test_unknown_flag_still_rejected() -> None:
    import pytest

    p = _build_parser()
    with pytest.raises(SystemExit):
        _parse_args(p, ["run", "--nope", "1"])
