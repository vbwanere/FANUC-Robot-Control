"""Command line: run / check / pendant / ftp."""
import argparse
import glob
import os
import re
import sys

from .lsparse import Program
from .runtime import Controller, TPAlarm, _strip_comment
from .cell import Cell


def load_dir(c, path):
    files = ([path] if path.lower().endswith(".ls")
             else sorted(glob.glob(os.path.join(path, "*.LS")) +
                         glob.glob(os.path.join(path, "*.ls"))))
    progs = []
    for f in files:
        p = Program.load(f)
        c.load(p)
        progs.append(p)
    return progs


def build(path):
    cell = Cell()
    c = Controller(cell)
    cell.configure(c)
    progs = load_dir(c, path)
    return c, cell, progs


# --------------------------------------------------------------------- check
def cmd_check(args):
    c, cell, progs = build(args.path)
    bad = 0
    for p in progs:
        labels = set()
        for l in p.lines:
            m = re.match(r"^LBL\[(\d+)\]", _strip_comment(l).strip())
            if m:
                labels.add(int(m.group(1)))
        for i, l in enumerate(p.lines, 1):
            sc = _strip_comment(l).strip().rstrip(";").strip()
            for m in re.finditer(r"LBL\[(\d+)\]", sc):
                if int(m.group(1)) not in labels and "LBL[" in sc:
                    if not sc.startswith("LBL["):
                        print("%s:%d  undefined LBL[%s]"
                              % (p.name, i, m.group(1)))
                        bad += 1
            for m in re.finditer(r"(?<![A-Z])P\[(\d+)\]", sc):
                if int(m.group(1)) not in p.pos:
                    print("%s:%d  P[%s] not taught" % (p.name, i, m.group(1)))
                    bad += 1
            m = re.match(r"^CALL\s+(\w+)", sc)
            if m and m.group(1) not in c.programs:
                print("%s:%d  CALL %s -- program not loaded"
                      % (p.name, i, m.group(1)))
                bad += 1
        print("%-10s %3d lines  %2d positions" % (p.name, len(p.lines),
                                                  len(p.pos)))
    print("--- %d problem(s)" % bad)
    return 1 if bad else 0


# ----------------------------------------------------------------------- run
def cmd_run(args):
    c, cell, progs = build(args.path)
    name = args.prog or progs[0].name
    c.select(name)
    c.paused = False
    c.running = True
    c.override = args.override
    dt = 0.02
    last = None
    while c.time < args.limit:
        f = c.top
        cur = (f.prog.name, f.pc) if f else None
        if args.trace and cur != last and f and f.pc < len(f.prog.lines) \
                and c.motion is None and c.wait is None:
            print("%7.2fs %-8s %3d: %s" % (c.time, f.prog.name, f.pc + 1,
                                           f.prog.lines[f.pc].strip()))
            last = cur
        c.step(dt)
        if c.alarm:
            print("ALARM %s %s   at %s" % (c.alarm[0], c.alarm[1],
                                           c.trace[-1] if c.trace else "?"))
            print(cell.status())
            return 2
        if not c.stack:
            break
    print("stopped at t=%.2fs  %s" % (c.time, c.msg))
    print(cell.status())
    print("R[1]=%s  R[2]=%s" % (c.R[1], c.R[2]))
    return 0


# ------------------------------------------------------------------- pendant
def cmd_pendant(args):
    from .pendant import main as pmain
    pmain(args.path, args.prog)
    return 0


def cmd_view(args):
    from .viewer import main as vmain
    return vmain(args.path, args.prog, args.follow)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="fanuc_tp_sim")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("check", help="static check of .LS files")
    p.add_argument("path")
    p.set_defaults(fn=cmd_check)

    p = sub.add_parser("run", help="headless run with trace")
    p.add_argument("path")
    p.add_argument("prog", nargs="?")
    p.add_argument("--limit", type=float, default=120.0)
    p.add_argument("--override", type=int, default=100)
    p.add_argument("--trace", action="store_true")
    p.set_defaults(fn=cmd_run)

    p = sub.add_parser("pendant", help="interactive teach pendant")
    p.add_argument("path")
    p.add_argument("prog", nargs="?")
    p.set_defaults(fn=cmd_pendant)

    p = sub.add_parser("view", help="3D view (needs matplotlib)")
    p.add_argument("path")
    p.add_argument("prog", nargs="?")
    p.add_argument("--follow", action="store_true",
                   help="mirror a running pendant instead of simulating")
    p.set_defaults(fn=cmd_view)

    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())