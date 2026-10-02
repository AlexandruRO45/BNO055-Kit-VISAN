#!/usr/bin/env python3
"""imu — tool TUI/CLI pentru BNO055 pe Jetson Orin Nano

  just user-level: pune fisierul in ~/bno055_imu/imu, apoi ln -s in ~/bin

  Rulare:
    imu                 -> meniu interactiv (TUI)
    imu scan|calibrate|bench|best|run|log|reset [optiuni]  -> mod CLI (scriptabil)
"""
import argparse, csv, json, math, os, shutil, sys, time
from types import SimpleNamespace
import numpy as np

CAL_FILE = "bno055_cal.json"
CAL_DIR = "calibrari"
HIST_FILE = os.path.join(CAL_DIR, "history.jsonl")
G = 9.81


def connect(bus_addr):
    import board, busio           # import intarziat: meniul porneste chiar daca senzorul e scos
    import adafruit_bno055
    i2c = busio.I2C(board.SCL, board.SDA)
    return adafruit_bno055.BNO055_I2C(i2c, address=bus_addr)


def wait_level(s, idx, label, hint):
    input(f"\n>>> {hint}\n    [Enter] cand esti gata...")
    print(f"    astept {label} = 3 ...")
    while s.calibration_status[idx] < 3:
        st = s.calibration_status
        print(f"\r    sys={st[0]} gyro={st[1]} accel={st[2]} mag={st[3]}   ",
              end="", flush=True)
        time.sleep(0.3)
    print(f"\r    {label}: OK (3)                              ")


# ------------------------------------------------------------------ scan
def cmd_scan(args):
    from smbus2 import SMBus
    bus = SMBus(args.bus)
    for addr in (0x28, 0x29):
        try:
            chip_id = bus.read_byte_data(addr, 0x00)
            print(f"[OK] raspunde la 0x{addr:02x}  (CHIP_ID=0x{chip_id:02x}, asteptat 0xa0)")
            return
        except OSError:
            pass
    print(f"[FAIL] nimic la 0x28/0x29 pe i2c-{args.bus}. Ruleaza: sudo i2cdetect -y -r {args.bus}")


# ----------------------------------------------------- validare 6 fete
def sample_face(s, hold_s=2.5, dt=0.1):
    xs, ys, zs = [], [], []
    for _ in range(int(hold_s / dt)):
        x, y, z = s.acceleration or (0, 0, 0)
        xs.append(x); ys.append(y); zs.append(z)
        time.sleep(dt)

    def stats(v):
        m = sum(v) / len(v)
        var = sum((x - m) ** 2 for x in v) / len(v)
        return m, math.sqrt(var)

    mx, sx = stats(xs); my, sy = stats(ys); mz, sz = stats(zs)
    return (mx, my, mz), max(sx, sy, sz)


def dominant_axis(v):
    a = [abs(x) for x in v]
    return a.index(max(a))


def check_face(s, name, how, axis, ref_sign):
    for attempt in range(1, 4):
        input(f"\n>>> FATA {name}: {how}\n"
              f"    Nemiscata 2.5s. [Enter] + nu te misca...")
        mean, std_max = sample_face(s)
        mag = math.sqrt(sum(c * c for c in mean))
        dom = dominant_axis(mean)
        sign = math.copysign(1, mean[dom])
        print(f"    citit : mag={mag:5.2f} m/s2  axa_dom={'XYZ'[dom]}  "
              f"semn={sign:+.0f}  jitter={std_max:.3f}")
        if not (G - 2.0 < mag < G + 2.0):
            print(f"    [X] magnitudinea nu e ~1g. Incercarea {attempt}/3."); continue
        if dom != axis:
            print(f"    [X] axa dominanta e {'XYZ'[dom]}, asteptam {'XYZ'[axis]}. Incercarea {attempt}/3."); continue
        if std_max > 0.4:
            print(f"    [X] prea multa miscare (jitter). Stai fix. Incercarea {attempt}/3."); continue
        if ref_sign is not None and sign != -ref_sign:
            print(f"    [X] semnul trebuia OPUS fata de prima pe axa {'XYZ'[axis]}. Incercarea {attempt}/3."); continue
        print(f"    [OK] fata {name} validata ({'XYZ'[dom]}{'+' if sign > 0 else '-'})")
        return sign
    print(f"    [!] fata {name} sarita dupa 3 incercari.")
    return None


# ------------------------------------------------------------ calibrate
def cmd_calibrate(args):
    s = connect(args.address)
    print("=== CALIBRARE BNO055 ===")
    wait_level(s, 1, "gyro", "PAS 1/3 (GYRO): placa pe masa, NEMISCATA.")

    print("\nPAS 2/3 (ACCEL): cele 6 fete, fiecare validata pe vectorul de gravitatie.")
    faces = [
        ("Z1", "orizontala, pe masa, chip in SUS", 2),
        ("Z2", "rasturnata 180 deg, chip in JOS (sprijina de o carte/cutie)", 2),
        ("X1", "VERTICALA pe o muchie", 0),
        ("X2", "aceeasi muchie, INTOARSA 180 deg fata de X1", 0),
        ("Y1", "VERTICALA pe cealalta muchie (rotita 90 deg fata de X1)", 1),
        ("Y2", "aceeasi muchie, INTOARSA 180 deg fata de Y1", 1),
    ]
    axis_sign, ok_faces = {}, 0
    for name, how, axis in faces:
        sign = check_face(s, name, how, axis, axis_sign.get(axis))
        if sign is not None:
            ok_faces += 1
            axis_sign.setdefault(axis, sign)

    print(f"\n    fete validate: {ok_faces}/6")
    print(f"    status dupa fete: accel={s.calibration_status[2]}")
    wait_level(s, 2, "accel", "asteapta convergenta interna (5-20s; poti misca incet placa intre pozitii)")
    wait_level(s, 3, "mag",  "PAS 3/3 (MAG): in aer, departe de metal/motoare, figura 8 lent.")

    print("\n    astept sys sa urce la 3 ...")
    while s.calibration_status[0] < 3:
        time.sleep(0.3)

    input("\n>>> Toate la 3! Nu misca placa, [Enter] pentru salvare...")
    cal = {"accel_offset": list(s.offsets_accelerometer),
           "mag_offset":   list(s.offsets_magnetometer),
           "gyro_offset":  list(s.offsets_gyroscope),
           "accel_radius": s.radius_accelerometer,
           "mag_radius":   s.radius_magnetometer}
    os.makedirs(CAL_DIR, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    cal_path = os.path.join(CAL_DIR, f"cal_{ts}.json")
    with open(cal_path, "w") as f:
        json.dump(cal, f, indent=2)
    shutil.copy(cal_path, CAL_FILE)
    print(f"[OK] salvat {cal_path} (activ: {CAL_FILE})")

    if input("    [Enter]=bench 30s (nemiscata) / s=skip >> ").strip().lower() != "s":
        do_bench(SimpleNamespace(bus=args.bus, address=args.address,
                                 time=30.0, cal_file=cal_path))


# ------------------------------------------------------------------ load
def load_calibration(s):
    try:
        with open(CAL_FILE) as f:
            cal = json.load(f)
    except FileNotFoundError:
        print(f"[!] {CAL_FILE} lipseste. Ruleaza: imu -> 2 (calibrate)")
        return False
    from adafruit_bno055 import CONFIG_MODE, NDOF_MODE
    s.mode = CONFIG_MODE
    time.sleep(0.05)
    s.offsets_accelerometer = tuple(cal["accel_offset"])
    s.offsets_magnetometer  = tuple(cal["mag_offset"])
    s.offsets_gyroscope     = tuple(cal["gyro_offset"])
    s.radius_accelerometer  = cal["accel_radius"]
    s.radius_magnetometer   = cal["mag_radius"]
    time.sleep(0.05)
    s.mode = NDOF_MODE
    time.sleep(0.05)
    print(f"[OK] calibrare incarcata, status: {s.calibration_status}")
    return True


# ------------------------------------------------------------------ bench
def do_bench(args):
    s = connect(args.address)
    load_calibration(s)
    print(f"[*] bench {args.time}s — LASA PLACA NEMISCATA ...")
    t0 = time.monotonic()
    heads, accs = [], []
    while time.monotonic() - t0 < args.time:
        e = s.euler or (0, 0, 0)
        a = s.linear_acceleration or (0, 0, 0)
        heads.append(e[0]); accs.append(a)
        time.sleep(0.01)

    heads_un = np.degrees(np.unwrap(np.radians(np.array(heads))))
    drift_per_min = (heads_un[-1] - heads_un[0]) * 60.0 / args.time
    la = float(np.linalg.norm(np.array(accs[50:]), axis=1).mean())
    head_noise = float(np.degrees(np.std(np.radians(heads_un))))
    score = round(la + abs(drift_per_min) + head_noise, 4)

    rec = {"cal_file": args.cal_file,
           "when": time.strftime("%Y-%m-%d %H:%M:%S"),
           "bias_linacc_m_s2": round(la, 4),
           "drift_deg_per_min": round(float(drift_per_min), 3),
           "head_noise_deg": round(head_noise, 4),
           "score": score}
    os.makedirs(CAL_DIR, exist_ok=True)
    with open(HIST_FILE, "a") as f:
        f.write(json.dumps(rec) + "\n")
    print(json.dumps(rec, indent=2))
    print(f"[scor {score} — cu cat mai mic, cu atat mai bine]")
    return rec


def cmd_bench(args):
    args.cal_file = CAL_FILE
    do_bench(args)


def cmd_best(args):
    if not os.path.exists(HIST_FILE):
        print("[!] nicio istorie. Ruleaza un bench mai intai.")
        return
    with open(HIST_FILE) as f:
        recs = [json.loads(l) for l in f if l.strip()]
    recs = [r for r in recs if os.path.exists(r["cal_file"])]
    if not recs:
        print("[!] istoric gasit, dar fisierele de calibrare au disparut.")
        return
    recs.sort(key=lambda r: r["score"])
    print(f"{'#':>2} {'scor':>8} {'bias m/s2':>10} {'drift deg/min':>14} {'noise deg':>10}  fisier")
    for i, r in enumerate(recs, 1):
        print(f"{i:>2} {r['score']:>8} {r['bias_linacc_m_s2']:>10} "
              f"{r['drift_deg_per_min']:>14} {r['head_noise_deg']:>10}  {r['cal_file']}")
    src = recs[0]["cal_file"]
    shutil.copy(src, CAL_FILE)
    print(f"\n[OK] {CAL_FILE} activ <- {src} (scor {recs[0]['score']})")


# -------------------------------------------------------------------- run
def cmd_run(args):
    s = connect(args.address)
    load_calibration(s)
    print("\033[2J", end="")
    try:
        while True:
            e = s.euler or (0, 0, 0)
            q = s.quaternion or (0, 0, 0, 0)
            a = s.linear_acceleration or (0, 0, 0)
            c = s.calibration_status
            print("\033[H", end="")
            print(f"Euler  : head={e[0]:7.2f}  roll={e[1]:7.2f}  pitch={e[2]:7.2f}        ")
            print(f"Quat   : w={q[0]:6.3f}  x={q[1]:6.3f}  y={q[2]:6.3f}  z={q[3]:6.3f}    ")
            print(f"LinAcc : x={a[0]:7.3f}  y={a[1]:7.3f}  z={a[2]:7.3f}                ")
            print(f"Calibr : sys={c[0]} gyro={c[1]} accel={c[2]} mag={c[3]}              ")
            print("\nCtrl+C = inapoi la meniu")
            time.sleep(1 / args.rate)
    except KeyboardInterrupt:
        print("\n")
        return


# -------------------------------------------------------------------- log
def cmd_log(args):
    s = connect(args.address)
    load_calibration(s)
    dt = 1.0 / args.rate
    t0 = time.monotonic()
    rows = 0
    print(f"[*] loghez la {args.rate} Hz in {args.out} — Ctrl+C = stop")
    try:
        with open(args.out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "qw", "qx", "qy", "qz",
                        "head", "roll", "pitch",
                        "lax", "lay", "laz",
                        "gx", "gy", "gz",
                        "sys", "cal_gyro", "cal_accel", "cal_mag"])
            next_t = t0
            while True:
                q = s.quaternion or (0, 0, 0, 0)
                e = s.euler or (0, 0, 0)
                a = s.linear_acceleration or (0, 0, 0)
                g = s.gyro or (0, 0, 0)
                c = s.calibration_status
                w.writerow([f"{time.monotonic() - t0:.6f}", *q, *e, *a, *g, *c])
                f.flush()
                rows += 1
                next_t += dt
                delay = next_t - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
    except KeyboardInterrupt:
        print(f"\n[OK] {rows} esantioane salvate in {args.out}")


# ------------------------------------------------------------------ reset
def cmd_reset(args):
    s = connect(args.address)
    s._reset()
    print("[OK] reset trimis. Calibrarea se pierde — reia cu opțiunea 2.")


# ================================================================ TUI menu
MENU = """
=================== BNO055 @ Jetson Orin Nano ===================
  1) scan        — verifica senzorul pe i2c-{bus}
  2) calibrate   — ghid pas cu pas + bench automat
  3) bench       — masoara calitatea calibrarii active ({t}s)
  4) best        — clasament istoric, activeaza castigatorul
  5) run         — dashboard live
  6) log         — CSV la {rate} Hz
  7) reset       — reset software al cipului
  q) quit
  adresa curenta: {addr}
================================================================="""


def tui(bus, address):
    while True:
        print("\033[2J\033[H", end="")          # ecran nou de fiecare data
        print(MENU.format(bus=bus, t=30, rate=100, addr=hex(address)))
        ch = input("imu> ").strip().lower()
        print("\033[2J\033[H", end="")          # ecran curat si pentru actiune
        pauser = True
        try:
            if ch == "1":
                cmd_scan(SimpleNamespace(bus=bus))
            elif ch == "2":
                cmd_calibrate(SimpleNamespace(bus=bus, address=address))
            elif ch == "3":
                t = input("    durata bench [30]: ").strip()
                do_bench(SimpleNamespace(bus=bus, address=address,
                                         time=float(t) if t else 30.0,
                                         cal_file=CAL_FILE))
            elif ch == "4":
                cmd_best(SimpleNamespace(bus=bus))
            elif ch == "5":
                cmd_run(SimpleNamespace(bus=bus, address=address, rate=20))
                pauser = False
            elif ch == "6":
                out = input("    fisier out [imu_log_<ts>.csv]: ").strip()
                cmd_log(SimpleNamespace(bus=bus, address=address, rate=100,
                                        out=out or f"imu_log_{int(time.time())}.csv"))
                pauser = False
            elif ch == "7":
                cmd_reset(SimpleNamespace(bus=bus, address=address))
            elif ch in ("q", "quit", "exit"):
                print("\033[2J\033[H", end="")  # iesi cu terminalul curat
                return
            else:
                print("optiune necunoscuta"); pauser = False
        except Exception as e:
            print(f"[ERR] {type(e).__name__}: {e}")
        if pauser:
            input("\n[Enter] inapoi la meniu...")


# ========================================================= CLI (scriptabil)
def cli():
    p = argparse.ArgumentParser(prog="imu", description="Tool BNO055 @ Jetson")
    p.add_argument("--bus", type=int, default=7)
    p.add_argument("--address", type=lambda x: int(x, 0), default=0x29)
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("scan").set_defaults(f=cmd_scan)
    sub.add_parser("calibrate").set_defaults(f=cmd_calibrate)

    pb = sub.add_parser("bench"); pb.add_argument("--time", type=float, default=30)
    pb.set_defaults(f=cmd_bench)
    sub.add_parser("best").set_defaults(f=cmd_best)

    pr = sub.add_parser("run"); pr.add_argument("--rate", type=float, default=20)
    pr.set_defaults(f=cmd_run)

    pl = sub.add_parser("log")
    pl.add_argument("--rate", type=float, default=100)
    pl.add_argument("--out", default=f"imu_log_{int(time.time())}.csv")
    pl.set_defaults(f=cmd_log)

    sub.add_parser("reset").set_defaults(f=cmd_reset)

    args = p.parse_args()
    args.f(args)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        cli()
    else:
        tui(bus=7, address=0x29)
