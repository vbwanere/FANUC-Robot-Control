"""A text-mode teach pendant.

Not a pixel copy of a FANUC iPendant -- a copy of its *behaviour*: SHIFT-gated
jogging, a coordinate system that changes what the jog keys mean, FWD stepping
that holds inside a WAIT, override that scales every motion, live register and
I/O screens, and TOUCHUP that writes the current pose back into P[n].
"""
import curses
import socket
import time

from . import kin
from .lsparse import Program
from .runtime import Controller
from .cell import Cell, DOOR_Y, INFEED, CHUCK, OUTFEED
from .viewer import broadcast

SCREENS = ["EDIT", "DATA R", "DATA PR", "I/O", "FRAMES", "POSN", "HELP"]
COORDS = ["JOINT", "WORLD", "TOOL", "USER"]
OVR = [1, 5, 10, 25, 50, 100]
JOG_INC = {"JOINT": 2.0, "WORLD": 20.0, "TOOL": 20.0, "USER": 20.0}
PLUS = "123456"
MINUS = "qwerty"

HELP = """
 KEY            ACTION
 up/down        move cursor          s        toggle SHIFT (deadman)
 PgUp/PgDn      page cursor          c        cycle coord: JOINT/WORLD/TOOL/USER
 v              next screen          1..6     jog axis + (needs SHIFT)
 V              prev screen          qwerty   jog axis -
 f              FWD one line         + / -    speed override up/down
 g              cycle start (AUTO)   b        BWD one line
 h              HOLD                 k        RESET alarm
 a              abort to line 1      p        TOUCHUP P[] on cursor line
 i              force/unforce DI     e        edit value under cursor
 S              save program .LS     x        exit

 On the EDIT screen the right pane is a top view of the cell.
 Rotational jog axes in WORLD/TOOL/USER are 4=W 5=P 6=R.
 Jogging into the machine (Y > %.0f) with the door shut will fault, exactly
 like the interlock on the real cell.
""" % DOOR_Y


class Pendant:
    def __init__(self, stdscr, path, progname):
        self.s = stdscr
        self.cell = Cell()
        self.c = Controller(self.cell)
        self.cell.configure(self.c)
        from .__main__ import load_dir
        self.progs = load_dir(self.c, path)
        self.path = path
        name = progname or self.progs[0].name
        self.c.select(name)
        self.cur = 0
        self.screen = 0
        self.coord = "JOINT"
        self.shift = False
        self.mode = "hold"          # hold | step | auto
        self.step_from = None
        self.note = ""
        self.datacur = 1
        self.iocur = 1
        self.iokind = "DI"
        self.framecur = 1
        self.frametype = "UTOOL"
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ------------------------------------------------------------------ loop
    def run(self):
        curses.curs_set(0)
        self.s.nodelay(True)
        last = time.monotonic()
        while True:
            now = time.monotonic()
            dt = min(0.1, now - last)
            last = now
            self.tick(dt)
            broadcast(self.sock, self.c, self.cell)
            self.draw()
            ch = self.s.getch()
            if ch != -1:
                if self.key(ch) == "quit":
                    return
            time.sleep(0.02)

    def tick(self, dt):
        c = self.c
        if self.mode == "hold":
            c.paused = True
            c.step(dt)                       # plant keeps running
            return
        c.paused = False
        c.step(dt)
        if c.alarm:
            self.mode = "hold"
            return
        if self.mode == "step":
            f = c.top
            here = (f.prog.name, f.pc) if f else None
            if here != self.step_from and c.motion is None and c.wait is None:
                self.mode = "hold"
                if f:
                    self.cur = f.pc

    # ------------------------------------------------------------------ keys
    def key(self, ch):
        c = self.c
        prog = c.top.prog if c.top else self.progs[0]
        self.note = ""
        if ch in (ord("x"),):
            return "quit"
        if ch == ord("v"):
            self.screen = (self.screen + 1) % len(SCREENS)
            return
        if ch == ord("V"):
            self.screen = (self.screen - 1) % len(SCREENS)
            return
        if ch == ord("s"):
            self.shift = not self.shift
            return
        if ch == ord("c"):
            self.coord = COORDS[(COORDS.index(self.coord) + 1) % 4]
            return
        if ch in (ord("+"), ord("=")):
            i = min(len(OVR) - 1, OVR.index(c.override) + 1) \
                if c.override in OVR else 3
            c.override = OVR[i]
            return
        if ch in (ord("-"), ord("_")):
            i = max(0, OVR.index(c.override) - 1) if c.override in OVR else 0
            c.override = OVR[i]
            return
        if ch == ord("h"):
            self.mode = "hold"
            return
        if ch == ord("k"):
            c.reset()
            self.cell.collision = None
            return
        if ch == ord("a"):
            c.select(prog.name)
            self.mode = "hold"
            self.cur = 0
            return
        if ch == ord("g"):
            if c.alarm:
                self.note = "reset the alarm first (k)"
            else:
                self.mode = "auto"
                c.running = True
            return
        if ch == ord("f"):
            if c.alarm:
                self.note = "reset the alarm first (k)"
                return
            f = c.top
            self.step_from = (f.prog.name, f.pc) if f else None
            self.mode = "step"
            return
        if ch == ord("b"):
            if c.top and c.top.pc > 0:
                c.top.pc -= 1
                self.cur = c.top.pc
            return
        if ch == ord("p"):
            self.touchup(prog)
            return
        if ch == ord("S"):
            import os
            p = os.path.join(self.path if not self.path.lower().endswith(".ls")
                             else os.path.dirname(self.path),
                             prog.name + ".LS")
            prog.save(p)
            self.note = "saved " + p
            return
        if ch == ord("e"):
            self.edit_value()
            return
        if ch == ord("i") and SCREENS[self.screen] == "I/O":
            k = ("DI+", self.iocur)
            k2 = ("DI-", self.iocur)
            if k in c.forced:
                c.forced.discard(k)
                c.forced.add(k2)
            elif k2 in c.forced:
                c.forced.discard(k2)
            else:
                c.forced.add(k)
            return

        # cursor movement, per screen
        up = ch in (curses.KEY_UP, ord("K"))
        dn = ch in (curses.KEY_DOWN, ord("J"))
        pg = 10 if ch in (curses.KEY_PPAGE, curses.KEY_NPAGE) else 0
        d = (-1 if up else 1 if dn else 0)
        if ch == curses.KEY_PPAGE:
            d = -pg
        if ch == curses.KEY_NPAGE:
            d = pg
        if d:
            sc = SCREENS[self.screen]
            if sc == "EDIT":
                self.cur = max(0, min(len(prog.lines) - 1, self.cur + d))
            elif sc == "DATA R":
                self.datacur = max(1, min(c.NUM_R, self.datacur + d))
            elif sc == "DATA PR":
                self.datacur = max(1, min(c.NUM_PR, self.datacur + d))
            elif sc == "I/O":
                self.iocur = max(1, min(32, self.iocur + d))
            elif sc == "FRAMES":
                self.framecur = max(1, min(9, self.framecur + d))
            return
        if ch == ord("\t") and SCREENS[self.screen] == "I/O":
            self.iokind = {"DI": "DO", "DO": "RI", "RI": "RO",
                           "RO": "DI"}[self.iokind]
            return
        if ch == ord("\t") and SCREENS[self.screen] == "FRAMES":
            self.frametype = "UFRAME" if self.frametype == "UTOOL" else "UTOOL"
            return

        # jogging
        if chr(ch) in PLUS or chr(ch) in MINUS:
            if not self.shift:
                self.note = "jog needs SHIFT (press s)"
                return
            if c.alarm:
                self.note = "alarm active"
                return
            sign = 1 if chr(ch) in PLUS else -1
            axis = (PLUS if sign > 0 else MINUS).index(chr(ch))
            err = c.jog(self.coord, axis, sign, JOG_INC[self.coord])
            if err:
                self.note = err
            return

    def touchup(self, prog):
        c = self.c
        line = prog.lines[self.cur] if self.cur < len(prog.lines) else ""
        import re
        m = re.search(r"P\[(\d+)", line)
        if not m:
            self.note = "cursor line has no P[]"
            return
        p = prog.pos.get(int(m.group(1)))
        if p is None:
            self.note = "P[%s] not in /POS" % m.group(1)
            return
        if p.kind == "joint":
            p.val = list(c.joints)
        else:
            T = kin.mat_mul(kin.mat_inv(kin.xyzwpr_to_mat(c.uframe[p.uf])),
                            kin.fk(c.joints, kin.xyzwpr_to_mat(c.utool[p.ut])))
            p.val = kin.mat_to_xyzwpr(T)
        self.note = "TOUCHUP P[%s] recorded" % m.group(1)

    def edit_value(self):
        sc = SCREENS[self.screen]
        val = self.prompt("value: ")
        if val is None:
            return
        try:
            f = float(val)
        except ValueError:
            self.note = "not a number"
            return
        c = self.c
        if sc == "DATA R":
            c.R[self.datacur] = f
        elif sc == "FRAMES":
            idx = self.prompt("element 1-6: ")
            tgt = c.utool if self.frametype == "UTOOL" else c.uframe
            try:
                tgt[self.framecur][int(idx) - 1] = f
            except Exception:
                self.note = "bad element"
        else:
            self.note = "nothing editable here"

    def prompt(self, text):
        curses.echo()
        self.s.nodelay(False)
        h, w = self.s.getmaxyx()
        self.s.move(h - 1, 0)
        self.s.clrtoeol()
        self.s.addstr(h - 1, 0, text)
        try:
            v = self.s.getstr(h - 1, len(text), 20).decode()
        except Exception:
            v = None
        curses.noecho()
        self.s.nodelay(True)
        return v

    # ------------------------------------------------------------------ draw
    def draw(self):
        s = self.s
        s.erase()
        h, w = s.getmaxyx()
        c = self.c
        left = max(40, w * 3 // 5)
        self.hdr(w)
        body = h - 6
        sc = SCREENS[self.screen]
        if sc == "EDIT":
            self.draw_edit(3, body, left)
            self.draw_view(3, body, left + 1, w - left - 1)
        elif sc == "DATA R":
            self.draw_regs(3, body, w)
        elif sc == "DATA PR":
            self.draw_pr(3, body, w)
        elif sc == "I/O":
            self.draw_io(3, body, w)
        elif sc == "FRAMES":
            self.draw_frames(3, body, w)
        elif sc == "POSN":
            self.draw_posn(3, body, w)
        else:
            for i, l in enumerate(HELP.strip("\n").splitlines()[:body]):
                self.put(3 + i, 0, l[:w - 1])
        self.footer(h, w)
        s.refresh()

    def put(self, y, x, text, attr=0):
        h, w = self.s.getmaxyx()
        if 0 <= y < h and x < w:
            try:
                self.s.addstr(y, x, text[:max(0, w - x - 1)], attr)
            except curses.error:
                pass

    def hdr(self, w):
        c = self.c
        prog = c.top.prog.name if c.top else "-"
        st = ("ALARM" if c.alarm else
              "RUN" if self.mode == "auto" and not c.paused else
              "STEP" if self.mode == "step" else "HOLD")
        self.put(0, 0, " %-10s  %s  line %3d   %s   %3d%%   %s%s"
                 % (prog, st, (c.top.pc + 1) if c.top else 0,
                    self.coord, c.override,
                    "[SHIFT] " if self.shift else "",
                    SCREENS[self.screen]), curses.A_REVERSE)
        j = c.joints
        p = c.lpos()
        self.put(1, 0, " J %s" % " ".join("%7.1f" % v for v in j))
        self.put(2, 0, " X %8.1f Y %8.1f Z %8.1f  W %7.1f P %7.1f R %7.1f"
                       "   UF%d UT%d" % (p[0], p[1], p[2], p[3], p[4], p[5],
                                         c.ufnum, c.utnum))

    def footer(self, h, w):
        c = self.c
        if c.alarm:
            self.put(h - 3, 0, " ALARM %s %s" % c.alarm,
                     curses.A_REVERSE | curses.A_BOLD)
        elif c.msg:
            self.put(h - 3, 0, " MSG: " + c.msg)
        self.put(h - 2, 0, " " + self.cell.status()[:w - 2])
        self.put(h - 1, 0, " %-40s  v screen  s SHIFT  f FWD  g AUTO  x exit"
                 % self.note)

    def draw_edit(self, y0, body, w):
        c = self.c
        prog = c.top.prog if c.top else self.progs[0]
        pc = c.top.pc if c.top else -1
        first = max(0, min(self.cur - body // 2, len(prog.lines) - body))
        for i in range(body):
            n = first + i
            if n >= len(prog.lines):
                break
            attr = 0
            if n == pc:
                attr |= curses.A_REVERSE
            if n == self.cur:
                attr |= curses.A_BOLD
            mark = ">" if n == self.cur else " "
            self.put(y0 + i, 0, "%s%4d:%s" % (mark, n + 1, prog.lines[n]),
                     attr)

    def draw_view(self, y0, body, x0, w):
        """Top view (X-Y plane) of the cell -- crude, but enough to jog by."""
        if w < 20:
            return
        c = self.c
        H = min(body, 24)
        grid = [[" "] * w for _ in range(H)]
        # world x: 0..1400 -> rows (top = far),  world y: -1000..800 -> cols
        def cell_xy(x, y):
            r = int((1500 - x) / 1500.0 * (H - 1))
            col = int((y + 1000) / 1800.0 * (w - 1))
            return r, col
        for name, st in (("I", INFEED), ("C", CHUCK), ("O", OUTFEED)):
            r, col = cell_xy(st[0], st[1])
            if 0 <= r < H and 0 <= col < w:
                grid[r][col] = name
        rr, cc = cell_xy(0, DOOR_Y)
        for r in range(H):
            if 0 <= cc < w and grid[r][cc] == " ":
                grid[r][cc] = "|" if self.cell.door != "open" else ":"
        pts = [(T[0][3], T[1][3]) for T in kin.fk_links(c.joints)]
        tcp = c.tcp_world()
        pts.append((tcp[0][3], tcp[1][3]))
        for i in range(len(pts) - 1):
            x1, y1 = pts[i]
            x2, y2 = pts[i + 1]
            for k in range(9):
                t = k / 8.0
                r, col = cell_xy(x1 + (x2 - x1) * t, y1 + (y2 - y1) * t)
                if 0 <= r < H and 0 <= col < w:
                    grid[r][col] = "*"
        r, col = cell_xy(tcp[0][3], tcp[1][3])
        if 0 <= r < H and 0 <= col < w:
            grid[r][col] = "@"
        self.put(y0, x0, "top view  I=infeed C=chuck O=outfeed @=TCP")
        for i in range(H):
            self.put(y0 + 1 + i, x0, "".join(grid[i]))

    def draw_regs(self, y0, body, w):
        c = self.c
        first = max(1, self.datacur - body // 2)
        for i in range(body):
            n = first + i
            if n > c.NUM_R:
                break
            attr = curses.A_BOLD if n == self.datacur else 0
            self.put(y0 + i, 0, " R[%3d] = %-12s" % (n, c.R[n]), attr)
        self.put(y0 + body - 1, 40, "e = edit value")

    def draw_pr(self, y0, body, w):
        c = self.c
        first = max(1, self.datacur - body // 2)
        for i in range(body):
            n = first + i
            if n > c.NUM_PR:
                break
            v = c.PR[n]
            txt = ("uninit" if v is None else
                   " ".join("%8.1f" % x for x in v))
            attr = curses.A_BOLD if n == self.datacur else 0
            self.put(y0 + i, 0, " PR[%3d] %s" % (n, txt), attr)

    def draw_io(self, y0, body, w):
        c = self.c
        arr = getattr(c, self.iokind)
        self.put(y0, 0, " %s   (TAB switches port, i forces DI)" % self.iokind)
        first = max(1, self.iocur - body // 2)
        for i in range(body - 1):
            n = first + i
            if n >= len(arr):
                break
            cm = c.io_comment.get((self.iokind, n), "")
            forced = ("F+" if ("DI+", n) in c.forced else
                      "F-" if ("DI-", n) in c.forced else "  ")
            attr = curses.A_BOLD if n == self.iocur else 0
            self.put(y0 + 1 + i, 0, " %s[%2d] %-3s %s  %-16s"
                     % (self.iokind, n, "ON" if arr[n] else "OFF", forced, cm),
                     attr)

    def draw_frames(self, y0, body, w):
        c = self.c
        self.put(y0, 0, " %s table  (TAB switches, e edits, active UF%d UT%d)"
                 % (self.frametype, c.ufnum, c.utnum))
        tgt = c.utool if self.frametype == "UTOOL" else c.uframe
        for n in range(1, 10):
            attr = curses.A_BOLD if n == self.framecur else 0
            self.put(y0 + n, 0, " %s[%d] %s"
                     % (self.frametype, n,
                        " ".join("%8.2f" % v for v in tgt[n])), attr)
        self.put(y0 + 11, 0, " tool offsets are X Y Z W P R from the flange;")
        self.put(y0 + 12, 0, " user frames are the same, measured from world.")

    def draw_posn(self, y0, body, w):
        c = self.c
        names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        for i in range(6):
            lo, hi = kin.JOINT_LIMITS[i]
            self.put(y0 + i, 0, " %s %9.2f deg   limits %6d %6d"
                     % (names[i], c.joints[i], lo, hi))
        p = c.lpos()
        lab = ["X", "Y", "Z", "W", "P", "R"]
        for i in range(6):
            self.put(y0 + i, 40, " %s %10.2f" % (lab[i], p[i]))
        T = c.tcp_world()
        self.put(y0 + 8, 0, " TCP in WORLD  %s"
                 % " ".join("%9.2f" % v for v in kin.mat_to_xyzwpr(T)))
        self.put(y0 + 9, 0, " flange WORLD  %s"
                 % " ".join("%9.2f" % v
                            for v in kin.mat_to_xyzwpr(kin.fk(c.joints))))


def main(path, prog=None):
    curses.wrapper(lambda scr: Pendant(scr, path, prog).run())