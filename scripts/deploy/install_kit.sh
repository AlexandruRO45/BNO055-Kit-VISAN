#!/usr/bin/env bash
# install_kit.sh — install the BNO055 kit on the Kit (Jetson Orin, JetPack 6).
#
# Installs to /opt/bno055 with its OWN venv (the VISAN venv deliberately
# carries no I2C/sensor deps), seeds the shipped calibration into
# /var/lib/bno055, installs the systemd unit, and ENABLES it at boot.
# Like the VISAN deb postinst, it deliberately does NOT start the service
# during install — the boot-time start is the tested path.
#
# Usage:
#   install_kit.sh [--confirm] [--src DIR] [--no-enable]
#
#   --confirm     actually apply changes (default is DRY-RUN)
#   --src DIR     kit source tree (default: this script's ../..)
#   --no-enable   install files but do not systemctl enable
#
# Exit codes: 0 success, 1 stage failure, 2 bad arguments.
set -euo pipefail

SRC_DIR=""
CONFIRM=0
ENABLE=1
DRY_PREFIX="[DRY-RUN] "

usage() { sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case "$1" in
        --confirm)  CONFIRM=1; DRY_PREFIX=""; shift ;;
        --src)      SRC_DIR="${2:?--src needs a directory}"; shift 2 ;;
        --no-enable) ENABLE=0; shift ;;
        -h|--help)  usage; exit 0 ;;
        *)          echo "[FAIL] unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

[ -n "$SRC_DIR" ] || SRC_DIR="$(cd "$(dirname "$0")/../.." && pwd)"

APP_ROOT="/opt/bno055"
SERVICE="bno055-imu.service"
UNIT_SRC="${SRC_DIR}/scripts/deploy/systemd/${SERVICE}"
WRAPPER_SRC="${SRC_DIR}/scripts/deploy/systemd/bno055_imu_wrapper.sh"
CAL_SRC="${SRC_DIR}/calibs/active.json"
CONFIG_SRC="${SRC_DIR}/configs/bno055_imu.yaml"

# Apply runs tee the whole session to a UTC-stamped log (VISAN ops-script
# convention); dry-runs stay silent so they make no filesystem changes.
if [ "$CONFIRM" -eq 1 ]; then
    LOG_FILE="/tmp/install_kit_$(date -u +%Y%m%dT%H%M%SZ).log"
    exec > >(tee -a "$LOG_FILE") 2>&1
fi

stage() { echo ""; echo "=== [STAGE] $* ==="; }
pass()  { echo "[KIT OK] $*"; }
fail()  { echo "[KIT FAIL: $1] $2" >&2; exit 1; }

run() {
    if [ "$CONFIRM" -eq 1 ]; then
        "$@"
    else
        echo "${DRY_PREFIX}$*"
    fi
}

echo "BNO055 kit install — $(date -u +%Y-%m-%dT%H:%M:%SZ) UTC"
echo "source : ${SRC_DIR}"
echo "mode   : $([ "$CONFIRM" -eq 1 ] && echo APPLY || echo DRY-RUN)"
echo "log    : ${LOG_FILE}"

# ------------------------------------------------------------------ preflight
stage "preflight"
[ "$(id -u)" -eq 0 ] || fail preflight "must run as root (sudo bash install_kit.sh)"
[ -d "$SRC_DIR/src/bno055_kit" ] || fail preflight "kit source tree not found at $SRC_DIR"
[ -f "$UNIT_SRC" ]     || fail preflight "missing unit: $UNIT_SRC"
[ -f "$WRAPPER_SRC" ]   || fail preflight "missing wrapper: $WRAPPER_SRC"
[ -f "$CAL_SRC" ]       || fail preflight "missing calibration: $CAL_SRC"
[ -f "$CONFIG_SRC" ]    || fail preflight "missing config: $CONFIG_SRC"
PYTHON_BIN="$(command -v python3)" || fail preflight "python3 not found"
PYVER="$("$PYTHON_BIN" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
case "$PYVER" in
    3.10|3.11|3.12) pass "python $PYVER" ;;
    *) fail preflight "python $PYVER outside supported range 3.10-3.12" ;;
esac
id -u visan >/dev/null 2>&1 \
    || fail preflight "user 'visan' not found — install the VISAN deb first"
pass "preflight"

# --------------------------------------------------------------------- install
stage "install tree -> ${APP_ROOT}"
run install -d -o visan -g visan "$APP_ROOT"
# --delete keeps the tree clean but MUST NOT touch the venv or the editable
# install's egg-info — wiping .venv would force a full network reinstall.
run rsync -a --delete --exclude __pycache__ --exclude '*.py[co]' \
    --exclude .venv --exclude '*.egg-info' \
    --chown visan:visan "${SRC_DIR}/src" "${SRC_DIR}/scripts" \
    "${SRC_DIR}/configs" "${SRC_DIR}/pyproject.toml" "${APP_ROOT}/"
pass "tree installed"

# ----------------------------------------------------------------------- venv
stage "venv ${APP_ROOT}/.venv"
if [ -x "${APP_ROOT}/.venv/bin/python" ]; then
    pass "venv already present — refreshing deps"
else
    run sudo -u visan "$PYTHON_BIN" -m venv "${APP_ROOT}/.venv"
fi
run sudo -u visan "${APP_ROOT}/.venv/bin/pip" install -e "${APP_ROOT}"
pass "venv"

# ------------------------------------------------------------- state + config
stage "state + config"
run install -d -o visan -g visan /var/lib/bno055 /var/log/bno055 /run/bno055
# Seed the shipped calibration ONLY if no active cal exists yet — never
# overwrite a newer bench result on the Kit.
if [ -f /var/lib/bno055/active.json ]; then
    pass "keeping existing /var/lib/bno055/active.json (not overwriting)"
else
    run install -o visan -g visan -m 0644 "$CAL_SRC" /var/lib/bno055/active.json
    pass "calibration seeded from $CAL_SRC"
fi
# Config: seed only when absent — an operator's tuned config survives a
# re-install (same keep-existing rule as the calibration above).
run install -d -m 0755 /etc/bno055
if [ -f /etc/bno055/bno055_imu.yaml ]; then
    pass "keeping existing /etc/bno055/bno055_imu.yaml (not overwriting)"
else
    run install -m 0644 "$CONFIG_SRC" /etc/bno055/bno055_imu.yaml
    pass "config seeded from $CONFIG_SRC"
fi
pass "state + config"

# ---------------------------------------------------------------------- unit
stage "systemd unit"
run install -m 0755 "$WRAPPER_SRC" /opt/bno055/scripts/deploy/systemd/bno055_imu_wrapper.sh
run install -m 0644 "$UNIT_SRC" "/lib/systemd/system/${SERVICE}"
run systemctl daemon-reload
if [ "$ENABLE" -eq 1 ]; then
    run systemctl enable "$SERVICE"
    pass "unit installed and enabled (NOT started — boots with the Kit)"
else
    pass "unit installed (--no-enable: not enabled)"
fi

echo ""
echo "=== [INSTALL ${DRY_PREFIX:+DRY-RUN }OK] ${APP_ROOT} ==="
echo "next: sudo systemctl start ${SERVICE}   # or reboot to test the boot path"
echo "verify: systemctl status ${SERVICE} ; journalctl -u ${SERVICE} -n 50"
exit 0
