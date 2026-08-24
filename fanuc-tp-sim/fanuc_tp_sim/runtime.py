"""TP (teach-pendant language) interpreter.

Execution model
---------------
The controller is a small state machine advanced by `step(dt)`. One call either

  * executes one non-motion instruction, or
  * advances an in-progress motion by dt seconds.

That split is what makes single-step (FWD), HOLD and speed override behave the
way they do on a real pendant: motion is a *state*, not a blocking call.

Everything a TP program can touch lives in `Controller`: numeric registers,
position registers, digital/analog/group I/O, flags, timers, UFRAME/UTOOL
tables, the call stack, and the alarm state.
"""
import re
from math import sin, cos, radians, degrees, sqrt

from . import kin

ON, OFF = 1, 0


class TPAlarm(Exception):
    def __init__(self, code, text):
        super().__init__("%s %s" % (code, text))
        self.code = code
        self.text = text


def _strip_comment(s):
    """R[1:count] -> R[1] ; DI[3:door open] -> DI[3]"""
    return re.sub(r"\[\s*(\d+)\s*:[^\]]*\]", r"[\1]", s)


class Frame:
    """One entry on the call stack."""
    def __init__(self, prog, pc=0, args=None):
        self.prog = prog
        self.pc = pc
        self.args = args or []
        self.forstack = []          # (line_index, reg, limit)


class Motion:
    def __init__(self, kind, q_start, q_end, T_start, T_end, secs, term):
        self.kind = kind            # 'J' or 'L'
        self.q_start = q_start
        self.q_end = q_end
        self.T_start = T_start
        self.T_end = T_end
        self.secs = max(secs, 1e-3)
        self.t = 0.0
        self.term = term            # 'FINE' or int CNT value
        self.skip_cond = None


class Controller:
    NUM_R = 200
    NUM_PR = 100
    NUM_IO = 64

    def __init__(self, cell=None):
        self.joints = [0.0, 0.0, 0.0, 0.0, -30.0, 0.0]
        self.R = [0.0] * (self.NUM_R + 1)
        self.PR = [None] * (self.NUM_PR + 1)
        self.PR_cmt = [""] * (self.NUM_PR + 1)
        self.DI = [0] * (self.NUM_IO + 1)
        self.DO = [0] * (self.NUM_IO + 1)
        self.RI = [0] * 17
        self.RO = [0] * 17
        self.UI = [0] * 33
        self.UO = [0] * 33
        self.GI = [0] * 17
        self.GO = [0] * 17
        self.AI = [0.0] * 17
        self.AO = [0.0] * 17
        self.F = [0] * 257
        self.io_comment = {}                       # ('DI',3) -> 'chuck closed'
        self.forced = set()                        # inputs held by the operator
        self.TIMER = [0.0] * 11
        self.timer_run = [False] * 11
        self.uframe = [[0.0] * 6 for _ in range(10)]
        self.utool = [[0.0] * 6 for _ in range(10)]
        self.ufnum = 0
        self.utnum = 0
        self.override = 100
        self.programs = {}
        self.stack = []
        self.motion = None
        self.wait = None                           # (cond, timeout, label)
        self.alarm = None
        self.paused = True
        self.running = False
        self.msg = ""
        self.time = 0.0
        self.trace = []
        self.cell = cell
        self.skip_cond = None
        self.last_skipped = False
        self.wait_timeout = 30.0

    # -------------------------------------------------------------- frames
    def uf_mat(self, n=None):
        return kin.xyzwpr_to_mat(self.uframe[self.ufnum if n is None else n])

    def ut_mat(self, n=None):
        return kin.xyzwpr_to_mat(self.utool[self.utnum if n is None else n])

    def tcp_world(self):
        return kin.fk(self.joints, self.ut_mat())

    def lpos(self):
        """Current TCP pose expressed in the active UFRAME (what TP calls LPOS)."""
        T = kin.mat_mul(kin.mat_inv(self.uf_mat()), self.tcp_world())
        return kin.mat_to_xyzwpr(T)

    def jpos(self):
        return list(self.joints)

    # ------------------------------------------------------------ programs
    def load(self, prog):
        self.programs[prog.name] = prog
        return prog

    def select(self, name):
        self.stack = [Frame(self.programs[name])]
        self.motion = None
        self.wait = None
        self.alarm = None
        self.msg = ""

    @property
    def top(self):
        return self.stack[-1] if self.stack else None

    def cur_line(self):
        f = self.top
        if not f or f.pc >= len(f.prog.lines):
            return None
        return f.prog.lines[f.pc]

    # ------------------------------------------------- expression evaluation
    def _operand(self, m):
        kindt, idx = m.group(1), int(m.group(2))
        if kindt == "R":
            return repr(self.R[idx])
        if kindt in ("DI", "DO", "RI", "RO", "UI", "UO", "F"):
            return repr(getattr(self, kindt)[idx])
        if kindt in ("GI", "GO", "AI", "AO"):
            return repr(getattr(self, kindt)[idx])
        if kindt == "TIMER":
            return repr(round(self.TIMER[idx], 2))
        raise TPAlarm("INTP-201", "unknown operand %s[%d]" % (kindt, idx))

    def eval(self, expr):
        e = _strip_comment(expr).strip()
        e = re.sub(r"AR\[(\d+)\]",
                   lambda m: repr(self._ar(int(m.group(1)))), e)
        e = re.sub(r"PR\[(\d+),\s*(\d+)\]",
                   lambda m: repr(self._pr_elem(int(m.group(1)),
                                                int(m.group(2)))), e)
        e = re.sub(r"(R|DI|DO|RI|RO|UI|UO|GI|GO|AI|AO|F|TIMER)\[(\d+)\]",
                   self._operand, e)
        e = re.sub(r"\bON\b", "1", e)
        e = re.sub(r"\bOFF\b", "0", e)
        e = re.sub(r"\bAND\b", " and ", e)
        e = re.sub(r"\bOR\b", " or ", e)
        e = e.replace("<>", "!=").replace("=<", "<=").replace("=>", ">=")
        e = re.sub(r"(?<![<>!=])=(?!=)", "==", e)
        e = e.replace("!", " not ") if re.search(r"!\s*(?:1|0|not)", e) else e
        try:
            return eval(e, {"__builtins__": {}}, {})
        except Exception:
            raise TPAlarm("INTP-202", "bad expression: %s" % expr)

    def _ar(self, n):
        f = self.top
        if not f or n > len(f.args):
            raise TPAlarm("INTP-238", "AR[%d] not supplied by caller" % n)
        return f.args[n - 1]

    def _pr_elem(self, i, j):
        p = self.PR[i] or [0.0] * 6
        return p[j - 1]

    # ------------------------------------------------------------- helpers
    def set_io(self, kindt, idx, val):
        arr = getattr(self, kindt)
        arr[idx] = val

    def target_pose_mat(self, prog, spec, offset=None, toff=None):
        """Resolve a motion target to a *flange* matrix + optional joint target."""
        spec = _strip_comment(spec)
        m = re.match(r"P\[(\d+)\]", spec)
        if m:
            p = prog.pos.get(int(m.group(1)))
            if p is None:
                raise TPAlarm("INTP-105", "position P[%s] not taught"
                              % m.group(1))
            if p.kind == "joint":
                return None, list(p.val)
            T = kin.mat_mul(kin.xyzwpr_to_mat(self.uframe[p.uf]),
                            kin.xyzwpr_to_mat(p.val))
            ut = self.utool[p.ut]
        else:
            m = re.match(r"PR\[(\d+)\]", spec)
            if not m:
                raise TPAlarm("INTP-105", "bad motion target %s" % spec)
            v = self.PR[int(m.group(1))]
            if v is None:
                raise TPAlarm("INTP-105", "PR[%s] is empty" % m.group(1))
            T = kin.mat_mul(self.uf_mat(), kin.xyzwpr_to_mat(v))
            ut = self.utool[self.utnum]
        if offset is not None:
            T = kin.mat_mul(kin.xyzwpr_to_mat(offset), T) if False else \
                kin.mat_mul(kin.mat_mul(self.uf_mat(),
                                        kin.xyzwpr_to_mat(offset)),
                            kin.mat_mul(kin.mat_inv(self.uf_mat()), T))
        if toff is not None:
            T = kin.mat_mul(T, kin.xyzwpr_to_mat(toff))
        return kin.mat_mul(T, kin.mat_inv(kin.xyzwpr_to_mat(ut))), None

    # ---------------------------------------------------------- instructions
    MOTION_RE = re.compile(
        r"^(J|L|C)\s+(P\[\d+[^\]]*\]|PR\[\d+[^\]]*\])\s+"
        r"([\d.]+)\s*(%|mm/sec|cm/min|inch/min|deg/sec|sec)\s*"
        r"(FINE|CNT\d+)(.*)$")

    def exec_line(self, line):
        """Execute one instruction. Returns True if the program counter should
        advance normally."""
        f = self.top
        prog = f.prog
        s = line.strip().rstrip(";").strip()
        if not s or s.startswith("!") or s.startswith("--"):
            return True

        m = self.MOTION_RE.match(s)
        if m:
            return self._do_motion(prog, m)

        sc = _strip_comment(s)

        # ---- labels / jumps / calls
        if re.match(r"^LBL\[\d+\]$", sc):
            return True
        m = re.match(r"^JMP LBL\[(\d+)\]$", sc)
        if m:
            self._jump(int(m.group(1)))
            return False
        m = re.match(r"^CALL\s+(\w+)\s*(?:\((.*)\))?$", sc)
        if m:
            self._call(m.group(1), m.group(2))
            return False
        if sc in ("END", "ABORT"):
            self._return()
            return False
        if sc == "PAUSE":
            self.paused = True
            self.msg = "PAUSE instruction"
            return True

        # ---- IF
        m = re.match(r"^IF\s+(.*?),\s*(JMP LBL\[\d+\]|CALL .*|"
                     r"(?:DO|RO|F|R)\[\d+\]\s*=.*)$", sc)
        if m:
            if self.eval(m.group(1)):
                return self.exec_line(m.group(2))
            return True

        # ---- SELECT / continuation "=n,JMP LBL[x]"
        m = re.match(r"^SELECT\s+R\[(\d+)\]\s*=\s*(.*)$", sc)
        if m:
            return self._select(int(m.group(1)), m.group(2))
        m = re.match(r"^=\s*(-?[\d.]+)\s*,\s*(.*)$", sc)
        if m or sc.startswith("ELSE,"):
            return True                     # already handled by SELECT

        # ---- WAIT
        m = re.match(r"^WAIT\s+([\d.]+)\s*\(sec\)$", sc)
        if m:
            self.wait = ("time", self.time + float(m.group(1)), None)
            return True
        m = re.match(r"^WAIT\s+(.*?)(?:\s+TIMEOUT\s*,\s*LBL\[(\d+)\])?$", sc)
        if m:
            self.wait = ("cond", m.group(1),
                         int(m.group(2)) if m.group(2) else None)
            self._wait_deadline = self.time + self.wait_timeout
            return True

        # ---- frames
        m = re.match(r"^(UFRAME_NUM|UTOOL_NUM)\s*=\s*(\d+)$", sc)
        if m:
            n = int(m.group(2))
            if m.group(1).startswith("UF"):
                self.ufnum = n
            else:
                self.utnum = n
            return True
        m = re.match(r"^(UFRAME|UTOOL)\[(\d+)\]\s*=\s*(.*)$", sc)
        if m:
            tgt = self.uframe if m.group(1) == "UFRAME" else self.utool
            tgt[int(m.group(2))] = self._pose_value(prog, m.group(3))
            return True

        # ---- registers
        m = re.match(r"^R\[(\d+)\]\s*=\s*(.*)$", sc)
        if m:
            self.R[int(m.group(1))] = self.eval(m.group(2))
            return True
        m = re.match(r"^PR\[(\d+),\s*(\d+)\]\s*=\s*(.*)$", sc)
        if m:
            i, j = int(m.group(1)), int(m.group(2))
            if self.PR[i] is None:
                self.PR[i] = [0.0] * 6
            self.PR[i][j - 1] = self.eval(m.group(3))
            return True
        m = re.match(r"^PR\[(\d+)\]\s*=\s*(.*)$", sc)
        if m:
            self.PR[int(m.group(1))] = self._pose_value(prog, m.group(2))
            return True

        # ---- outputs
        m = re.match(r"^(DO|RO|F|GO|AO)\[(\d+)\]\s*=\s*(.*)$", sc)
        if m:
            kindt, idx, val = m.group(1), int(m.group(2)), m.group(3).strip()
            pm = re.match(r"PULSE\s*,?\s*([\d.]+)\s*sec", val)
            if pm:
                self.set_io(kindt, idx, ON)
                self._pulse = (kindt, idx, self.time + float(pm.group(1)))
            else:
                self.set_io(kindt, idx, self.eval(val))
            return True

        # ---- timers / misc
        m = re.match(r"^TIMER\[(\d+)\]\s*=\s*\(?(RESET|START|STOP)\)?$", sc)
        if m:
            n = int(m.group(1))
            act = m.group(2)
            if act == "RESET":
                self.TIMER[n] = 0.0
                self.timer_run[n] = False
            else:
                self.timer_run[n] = (act == "START")
            return True
        m = re.match(r"^FOR\s+R\[(\d+)\]\s*=\s*(.*?)\s+TO\s+(.*)$", sc)
        if m:
            return self._for(int(m.group(1)), m.group(2), m.group(3))
        if sc == "ENDFOR":
            return self._endfor()
        m = re.match(r"^OVERRIDE\s*=\s*(\d+)%$", sc)
        if m:
            self.override = int(m.group(1))
            return True
        m = re.match(r'^MESSAGE\[(.*)\]$', s)
        if m:
            self.msg = m.group(1)
            return True
        if re.match(r"^(PAYLOAD|UALM|RSR|\$)", sc):
            return True
        m = re.match(r"^SKIP CONDITION\s+(.*)$", sc)
        if m:
            self.skip_cond = m.group(1)
            return True

        raise TPAlarm("INTP-311", "unsupported instruction: %s" % s)

    # -------------------------------------------------------------- helpers
    def _pose_value(self, prog, expr):
        e = _strip_comment(expr).strip()
        if e == "LPOS":
            return self.lpos()
        if e == "JPOS":
            return list(self.joints)
        m = re.match(r"^P\[(\d+)\]$", e)
        if m:
            return list(prog.pos[int(m.group(1))].val)
        m = re.match(r"^PR\[(\d+)\]$", e)
        if m:
            v = self.PR[int(m.group(1))]
            return list(v) if v else [0.0] * 6
        m = re.match(r"^PR\[(\d+)\]\s*([-+])\s*PR\[(\d+)\]$", e)
        if m:
            a = self.PR[int(m.group(1))] or [0.0] * 6
            b = self.PR[int(m.group(3))] or [0.0] * 6
            sg = 1 if m.group(2) == "+" else -1
            return [a[i] + sg * b[i] for i in range(6)]
        m = re.match(r"^UFRAME\[(\d+)\]$", e)
        if m:
            return list(self.uframe[int(m.group(1))])
        raise TPAlarm("INTP-202", "bad position value: %s" % expr)

    def _jump(self, label):
        f = self.top
        tgt = "LBL[%d]" % label
        for i, l in enumerate(f.prog.lines):
            if _strip_comment(l).strip().rstrip(";").strip() == tgt:
                f.pc = i + 1
                return
        raise TPAlarm("INTP-224", "label %d not found" % label)

    def _call(self, name, argstr):
        if name not in self.programs:
            raise TPAlarm("INTP-106", "program %s not found" % name)
        args = []
        if argstr:
            for a in argstr.split(","):
                a = a.strip()
                try:
                    args.append(self.eval(a))
                except TPAlarm:
                    args.append(a.strip("'\""))
        self.top.pc += 1
        self.stack.append(Frame(self.programs[name], 0, args))

    def _return(self):
        self.stack.pop()
        if not self.stack:
            self.running = False
            self.paused = True
            self.msg = "program complete"

    def _select(self, reg, rest):
        f = self.top
        val = self.R[reg]
        branches = [(rest, f.pc)]
        i = f.pc + 1
        while i < len(f.prog.lines):
            l = _strip_comment(f.prog.lines[i]).strip().rstrip(";").strip()
            if l.startswith("=") or l.startswith("ELSE,"):
                branches.append((l, i))
                i += 1
            else:
                break
        for text, _ in branches:
            if text.startswith("ELSE,"):
                return self.exec_line(text[5:])
            m = re.match(r"^=?\s*(-?[\d.]+)\s*,\s*(.*)$", text)
            if m and abs(float(m.group(1)) - val) < 1e-9:
                return self.exec_line(m.group(2))
        f.pc = i
        return False

    def _for(self, reg, start, end):
        f = self.top
        lo, hi = self.eval(start), self.eval(end)
        if f.forstack and f.forstack[-1][0] == f.pc:
            return True
        self.R[reg] = lo
        if lo > hi:
            depth = 0
            for i in range(f.pc + 1, len(f.prog.lines)):
                t = _strip_comment(f.prog.lines[i]).strip().rstrip(";").strip()
                if t.startswith("FOR "):
                    depth += 1
                elif t == "ENDFOR":
                    if depth == 0:
                        f.pc = i + 1
                        return False
                    depth -= 1
            raise TPAlarm("INTP-225", "ENDFOR not found")
        f.forstack.append((f.pc, reg, hi))
        return True

    def _endfor(self):
        f = self.top
        if not f.forstack:
            raise TPAlarm("INTP-225", "ENDFOR without FOR")
        pc, reg, hi = f.forstack[-1]
        self.R[reg] += 1
        if self.R[reg] > hi:
            f.forstack.pop()
            return True
        f.pc = pc + 1
        return False

    # ---------------------------------------------------------------- motion
    def _do_motion(self, prog, m):
        kindt, spec, spd, unit, term, opts = m.groups()
        offset = toff = None
        om = re.search(r"Offset\s*,\s*PR\[(\d+)", opts)
        if om:
            offset = self.PR[int(om.group(1))] or [0.0] * 6
        tm = re.search(r"Tool_Offset\s*,\s*PR\[(\d+)", opts)
        if tm:
            toff = self.PR[int(tm.group(1))] or [0.0] * 6

        Tf, qjoint = self.target_pose_mat(prog, spec, offset, toff)
        if qjoint is not None:
            q_end = qjoint
            Tf = kin.fk(q_end)
        else:
            q_end = kin.ik(Tf, self.joints)
            if q_end is None:
                raise TPAlarm("MOTN-023",
                              "position unreachable / in singularity")

        ov = max(self.override, 1) / 100.0
        T0 = kin.fk(self.joints)
        if unit == "%":
            dq = max(abs(q_end[i] - self.joints[i]) for i in range(6))
            secs = dq / (180.0 * (float(spd) / 100.0) * ov + 1e-9)
        elif unit == "sec":
            secs = float(spd) / ov
        else:
            v = float(spd)
            if unit == "cm/min":
                v = v * 10.0 / 60.0
            elif unit == "inch/min":
                v = v * 25.4 / 60.0
            d = kin.dist(kin.mat_to_xyzwpr(T0), kin.mat_to_xyzwpr(Tf))
            secs = d / (v * ov + 1e-9)
        self.motion = Motion("J" if kindt == "J" else "L", list(self.joints),
                             q_end, T0, Tf, secs,
                             "FINE" if term == "FINE" else int(term[3:]))
        self.motion.skip_cond = self.skip_cond
        self.skip_cond = None
        m2 = re.search(r"Skip\s*,\s*LBL\[(\d+)\]", opts)
        self.motion.skip_lbl = int(m2.group(1)) if m2 else None
        return True

    def _advance_motion(self, dt):
        mo = self.motion
        if mo.skip_cond and self.eval(mo.skip_cond):
            self.motion = None
            self.last_skipped = True
            if getattr(mo, "skip_lbl", None):
                self._jump(mo.skip_lbl)
            return
        mo.t += dt
        u = min(1.0, mo.t / mo.secs)
        if mo.kind == "J":
            self.joints = [mo.q_start[i] + (mo.q_end[i] - mo.q_start[i]) * u
                           for i in range(6)]
        else:
            a = kin.mat_to_xyzwpr(mo.T_start)
            b = kin.mat_to_xyzwpr(mo.T_end)
            p = [a[i] + (b[i] - a[i]) * u for i in range(3)]
            e = kin.pose_error(mo.T_start, mo.T_end)
            Tt = self._interp_rot(mo.T_start, e[3:], u, p)
            q = kin.ik(Tt, self.joints)
            if q is None:
                self.motion = None
                raise TPAlarm("MOTN-023", "linear path leaves workspace")
            self.joints = q
        if u >= 1.0:
            if mo.kind == "J":
                self.joints = list(mo.q_end)
            self.motion = None

    @staticmethod
    def _interp_rot(T0, axis_world, u, p):
        th = sqrt(sum(a * a for a in axis_world))
        R0 = [[T0[i][j] for j in range(3)] for i in range(3)]
        if th < 1e-9:
            R = R0
        else:
            k = [a / th for a in axis_world]
            a_ = th * u
            c, s, C = cos(a_), sin(a_), 1 - cos(a_)
            Rk = [[k[0] * k[0] * C + c, k[0] * k[1] * C - k[2] * s,
                   k[0] * k[2] * C + k[1] * s],
                  [k[1] * k[0] * C + k[2] * s, k[1] * k[1] * C + c,
                   k[1] * k[2] * C - k[0] * s],
                  [k[2] * k[0] * C - k[1] * s, k[2] * k[1] * C + k[0] * s,
                   k[2] * k[2] * C + c]]
            R = [[sum(Rk[i][x] * R0[x][j] for x in range(3)) for j in range(3)]
                 for i in range(3)]
        return [[R[0][0], R[0][1], R[0][2], p[0]],
                [R[1][0], R[1][1], R[1][2], p[1]],
                [R[2][0], R[2][1], R[2][2], p[2]],
                [0, 0, 0, 1.0]]

    # ------------------------------------------------------------------ tick
    def step(self, dt=0.02):
        self.time += dt
        for i in range(1, 11):
            if self.timer_run[i]:
                self.TIMER[i] += dt
        pu = getattr(self, "_pulse", None)
        if pu and self.time >= pu[2]:
            self.set_io(pu[0], pu[1], OFF)
            self._pulse = None
        if self.cell:
            self.cell.step(self, dt)
        if self.alarm or self.paused or not self.stack:
            return
        try:
            if self.motion is not None:
                self._advance_motion(dt)
                if self.motion is None:
                    self.top.pc += 1
                return
            if self.wait is not None:
                kindt, a, lbl = self.wait
                if kindt == "time":
                    if self.time >= a:
                        self.wait = None
                        self.top.pc += 1
                else:
                    if self.eval(a):
                        self.wait = None
                        self.top.pc += 1
                    elif lbl is not None and self.time > self._wait_deadline:
                        self.wait = None
                        self._jump(lbl)
                return
            f = self.top
            if f.pc >= len(f.prog.lines):
                self._return()
                return
            line = f.prog.lines[f.pc]
            self.trace.append((f.prog.name, f.pc + 1, line))
            if self.exec_line(line):
                if self.motion is None and self.wait is None:
                    self.top.pc += 1
        except TPAlarm as e:
            self.fault(e.code, e.text)

    def fault(self, code, text):
        self.alarm = (code, text)
        self.paused = True
        self.running = False
        self.motion = None

    def reset(self):
        self.alarm = None
        self.msg = ""

    # ----------------------------------------------------------------- jog
    def jog(self, frame, axis, sign, amount):
        """frame: JOINT | WORLD | TOOL | USER.  axis 0..5.  Returns error str."""
        step = amount * max(self.override, 1) / 100.0
        if frame == "JOINT":
            q = list(self.joints)
            q[axis] += sign * step
            lo, hi = kin.JOINT_LIMITS[axis]
            if not (lo <= q[axis] <= hi):
                return "J%d limit" % (axis + 1)
            self.joints = q
            return None
        T = kin.fk(self.joints, self.ut_mat())
        d = [0.0] * 6
        d[axis] = sign * step
        if frame == "TOOL":
            delta = kin.xyzwpr_to_mat([d[0], d[1], d[2], d[3], d[4], d[5]])
            Td = kin.mat_mul(T, delta)
        else:
            base = self.uf_mat() if frame == "USER" else kin.IDENTITY
            delta = kin.xyzwpr_to_mat(d)
            Td = kin.mat_mul(kin.mat_mul(base, delta),
                             kin.mat_mul(kin.mat_inv(base), T))
        q = kin.ik(kin.mat_mul(Td, kin.mat_inv(self.ut_mat())), self.joints)
        if q is None:
            return "unreachable / singularity"
        self.joints = q
        return None
