"""
TapTable Stage 2 - tap logger (runs on the laptop)

Reads the CSV lines the board sends over the serial port and saves them,
each tagged with the zone you were tapping.

Install once:   pip install pyserial numpy
Find the port:  python logger.py --list
Run:            python logger.py --port COM5 --out taps.csv

How a session goes:
  1. The script asks for a zone label (e.g. S1, S2, S3, C, M12 ...).
  2. It asks how many taps (Enter = 20).
  3. Tap that zone at a steady pace, about one tap per second.
     Each tap prints one line; the script stops at the count you gave.
  4. Repeat for the next zone. Type q as the label to finish.
Taps are appended, so you can run the script again to add more data.

Board messages (lines starting with '#') are also saved, with the time and
the current label, to a side file next to the CSV, e.g. taps_board.txt.
After the last tap of a label the script keeps listening: if the board is
finishing a calibration it waits until the board says "# ready" (max 15 s),
so the calibration summary is shown and saved.

DEMO MODE (for volunteers; needs demo_eval.py in the same folder):
    python logger.py --port COM11 --demo                 10 taps per zone
    python logger.py --port COM11 --demo --taps 5        5 taps per zone
    python logger.py --port COM11 --demo --shuffle       zones in random order
  1. Type the volunteer's name (Enter = guest1, guest2 ...; q to finish).
  2. The script asks for each zone in turn, Z1 to Z6: press Enter, then the
     volunteer taps that zone. Every tap shows what the board decided:
        tap  3/10   board: Z2   OK      (votes 9/9)
        tap  4/10   board: Z5   WRONG   (votes 6/9)
        tap  5/10   board: unsure       (votes 4/9)
  3. After Z6 it prints that volunteer's score, then the running total of
     every volunteer in this demo.
  Each volunteer's taps are saved in full (same columns as normal logging)
  to demo\\demo_<date>_<name>.csv, so they can also be used later for the
  dataset. Score any of them again with:  python demo_eval.py
  Ctrl+C during a zone ends that zone early.
"""

import argparse
import csv
import os
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is missing: run  pip install pyserial")

FIELDS = ["n", "rng", "thr", "p1", "p2", "p3", "e1", "e2", "e3",
          "o1", "o2", "o3", "k1", "k2", "k3", "sat", "first", "win"]
HEADER = ["time", "label"] + FIELDS
EXTRA = ["pred", "votes"]   # board's own decision (Stage 3 onwards), for reading only;
                            # analyze.py ignores these columns


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found. Is the USB-serial adapter plugged in?")
    for p in ports:
        print(f"{p.device:10s} {p.description}")


def parse_tap(line):
    """'T,1,16,700,...' -> list of 18 ints, or None if not a tap line."""
    if not line.startswith("T,"):
        return None
    parts = line.split(",")[1:]
    if len(parts) != len(FIELDS):
        return None
    try:
        return [int(x) for x in parts]
    except ValueError:
        return None


def parse_pred(line):
    """Board info line after a tap -> (pred, votes) or None.
    '# zone 3  votes 5/5  vol 6' -> ('Z3', '5/5')
    '# unsure, ignored (votes 2/5)' -> ('unsure', '2/5')
    '# CAL zone 2: 7/20' -> ('cal', '')"""
    t = line.lstrip("#").strip()
    if t.startswith("zone "):
        parts = t.split()
        return ("Z" + parts[1], parts[3] if len(parts) > 3 else "")
    if t.startswith("unsure"):
        v = t.split("votes")[-1].strip(" )") if "votes" in t else ""
        return ("unsure", v)
    if t.startswith("CAL zone"):
        return ("cal", "")
    return None


def is_cal_summary(line):
    """Board lines sent after a calibration's last tap (not the per-tap counts)."""
    t = line.lstrip("#").strip()
    return (t.startswith("CAL") and not t.startswith("CAL zone")) or t.startswith("TAMPER")


def read_line(ser):
    raw = ser.readline()
    if not raw:
        return None
    return raw.decode("ascii", errors="replace").strip()


def run_demo(ser, args):
    """Demo mode: each volunteer taps every zone a fixed number of times;
    the board's decision is shown per tap and scored at the end."""
    try:
        import demo_eval
    except ImportError:
        sys.exit("demo_eval.py is missing: put it in the same folder as logger.py")
    import random

    os.makedirs(args.demo_dir, exist_ok=True)
    day = time.strftime("%Y%m%d")
    msg_path = os.path.join(args.demo_dir, f"demo_{day}_board.txt")
    everyone = []          # (name, rows) for the running total
    guest = 0

    print("\n" + "=" * 62)
    print(" TapTable demo: tap each zone, see what the board decides")
    print("=" * 62)
    print(f"{args.taps} taps per zone. Results are saved in the folder '{args.demo_dir}'.")
    print("Press the board's RESET now if it is not running Stage 3a or later.")

    with open(msg_path, "a") as fm:
        def log_msg(who, line):
            fm.write(f"{time.strftime('%H:%M:%S')}\t{who}\t{line}\n")
            fm.flush()

        while True:
            name = input("\nVolunteer name (Enter = guest, q = finish): ").strip()
            if name.lower() == "q":
                break
            if not name:
                guest += 1
                name = f"guest{guest}"
            safe = "".join(ch for ch in name if ch.isalnum() or ch in "-") or "guest"
            path = os.path.join(args.demo_dir, f"demo_{day}_{safe}.csv")
            new_file = not os.path.exists(path)
            zones = [1, 2, 3, 4, 5, 6]
            if args.shuffle:
                random.shuffle(zones)
            rows = []

            with open(path, "a", newline="") as f:
                w = csv.writer(f)
                if new_file:
                    w.writerow(HEADER + EXTRA)
                for z in zones:
                    label = f"Z{z}"
                    input(f"\n>>> {name}: next is zone {label}. Press Enter, then tap {label} "
                          f"{args.taps} times (about one tap a second).")
                    while ser.in_waiting:              # ignore taps made before Enter
                        line = read_line(ser)
                        if line and line.startswith("#"):
                            log_msg(name, line)
                    print(f"    Tap {label} now...")
                    got, pending, warned = 0, None, False
                    try:
                        while got < args.taps:
                            line = read_line(ser)
                            if not line:
                                continue
                            tap = parse_tap(line)
                            if tap is not None:
                                pending = [time.strftime("%H:%M:%S"), label] + tap
                                continue
                            if not line.startswith("#"):
                                continue
                            log_msg(name, line)
                            pr = parse_pred(line)
                            if pr is None or pending is None:
                                continue
                            if pr[0] == "cal":
                                if not warned:
                                    print("    !! The board is CALIBRATING, not classifying."
                                          " Press TAMPER briefly to cancel, then continue.")
                                    warned = True
                                pending = None
                                continue
                            got += 1
                            w.writerow(pending + list(pr))
                            f.flush()
                            pending = None
                            rows.append({"label": label, "pred": pr[0], "votes": pr[1]})
                            if pr[0] == "unsure":
                                verdict = "board: unsure      "
                            else:
                                verdict = (f"board: {pr[0]:3s}  "
                                           f"{'OK   ' if pr[0] == label else 'WRONG'}")
                            print(f"    tap {got:2d}/{args.taps}   {verdict}  (votes {pr[1]})")
                    except KeyboardInterrupt:
                        print(f"\n    Stopped {label} early after {got} taps.")

            everyone.append((name, rows))
            demo_eval.summary(rows, f"{name}")
            if len(everyone) > 1:
                print("\n  person            taps  correct   unsure    wrong")
                for nm, rr in everyone:
                    n = len(rr)
                    c = sum(demo_eval.outcome(r) == "correct" for r in rr)
                    u = sum(demo_eval.outcome(r) == "unsure" for r in rr)
                    wr = sum(demo_eval.outcome(r) == "wrong" for r in rr)
                    print(f"  {nm[:16]:16s} {n:5d}   {demo_eval.pct(c, n)}   "
                          f"{demo_eval.pct(u, n)}   {demo_eval.pct(wr, n)}")
                demo_eval.summary([r for _, rr in everyone for r in rr],
                                  f"ALL {len(everyone)} volunteers so far", details=False)
            print(f"\n  Saved to {path}")

    print(f"\nDemo finished. Score everything again any time with:  python demo_eval.py {args.demo_dir}")


def main():
    ap = argparse.ArgumentParser(description="Log TapTable taps to CSV")
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--out", default="taps.csv", help="CSV file to append to")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true", help="list serial ports")
    ap.add_argument("--demo", action="store_true",
                    help="demo mode: volunteers tap each zone, board decisions are scored")
    ap.add_argument("--taps", type=int, default=10, help="demo: taps per zone (default 10)")
    ap.add_argument("--shuffle", action="store_true", help="demo: zones in random order")
    ap.add_argument("--demo-dir", default="demo", help="demo: folder for results (default demo)")
    args = ap.parse_args()

    if args.list or not args.port:
        list_ports()
        if not args.port:
            print("\nRun again with --port <name>")
        return

    if args.demo:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
        try:
            run_demo(ser, args)
        finally:
            ser.close()
        return

    new_file = not os.path.exists(args.out)
    with_pred = True
    if not new_file:                      # an older file without the extra columns:
        with open(args.out, newline="") as f0:   # keep its format so columns line up
            first = f0.readline()
        with_pred = "pred" in first.split(",")[-2:] or "pred" in first
    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    msg_path = os.path.splitext(args.out)[0] + "_board.txt"
    print(f"Listening on {args.port}. Press the board's RESET to see its settings line.")
    print(f"Board messages are also saved to {msg_path}")

    with open(args.out, "a", newline="") as f, open(msg_path, "a") as fm:
        cur_label = ""

        def log_msg(line, show=True):
            fm.write(f"{time.strftime('%H:%M:%S')}\t{cur_label}\t{line}\n")
            fm.flush()
            if show:
                print("  board:", line)
        w = csv.writer(f)
        if new_file:
            w.writerow(HEADER + EXTRA)
        elif not with_pred:
            print("Note: this file has no 'pred' column; board predictions are only shown,"
                  " not saved. Use a new --out file to save them.")

        pending = None                    # last tap row, waiting for the board's decision

        def flush(pred=("", "")):
            nonlocal pending
            if pending is not None:
                w.writerow(pending + (list(pred) if with_pred else []))
                f.flush()
                pending = None

        while True:
            label = input("\nZone label (q to quit): ").strip()
            if label.lower() == "q":
                break
            if not label:
                continue
            count_txt = input("How many taps? [20]: ").strip()
            count = int(count_txt) if count_txt.isdigit() else 20

            flush()
            while ser.in_waiting:             # messages sent while typing: save them,
                line = read_line(ser)         # drop taps made while typing
                if line and line.startswith("#"):
                    log_msg(line)
            cur_label = label
            print(f"Tap zone {label} {count} times...")
            got, clipped = 0, 0
            while got < count:
                try:
                    line = read_line(ser)
                except KeyboardInterrupt:
                    print("\nStopped this zone early.")
                    break
                if not line:
                    continue
                if line.startswith("#"):
                    pr = parse_pred(line)
                    if pr is not None and pending is not None:
                        log_msg(line, show=False)
                        flush(pr)
                        if pr[0] != "cal":
                            print(f"         board says: {pr[0]} {pr[1]}")
                        continue
                    log_msg(line)
                    continue
                tap = parse_tap(line)
                if tap is None:
                    continue
                got += 1
                d = dict(zip(FIELDS, tap))
                if d["sat"]:
                    clipped += 1
                flush()                       # previous tap had no decision line
                pending = [time.strftime("%H:%M:%S"), label] + tap
                print(f"  {got:3d}/{count}  peaks {d['p1']:6d} {d['p2']:6d} {d['p3']:6d}"
                      f"   first S{d['first']}  win S{d['win']}"
                      f"{'   CLIPPED' if d['sat'] else ''}")
            # keep listening: the last tap's decision, and a calibration summary
            t_end = time.time() + 0.8
            in_cal = False
            while time.time() < t_end:
                line = read_line(ser)
                if not line:
                    if pending is None and not in_cal:
                        break
                    continue
                if not line.startswith("#"):
                    continue                  # a tap after the count: not saved
                pr = parse_pred(line)
                if pr is not None and pending is not None:
                    log_msg(line, show=False)
                    flush(pr)
                    if pr[0] != "cal":
                        print(f"         board says: {pr[0]} {pr[1]}")
                    continue
                log_msg(line)
                if is_cal_summary(line) and not in_cal:
                    in_cal = True
                    t_end = time.time() + 15.0
                    print("  (calibration finishing: hands off, waiting for the board...)")
                if in_cal and line.lstrip("#").strip().startswith("ready"):
                    break
            flush()
            print(f"Saved {got} taps for {label}"
                  f"{f' ({clipped} clipped)' if clipped else ''}.")

    ser.close()
    print(f"\nData is in {args.out}. Next: python analyze.py {args.out}")


if __name__ == "__main__":
    main()
