"""Parser and writer for FANUC .LS (ASCII teach-pendant) files.

An .LS file is just text, which is the whole reason offline TP work is
possible at all. Structure:

    /PROG  NAME
    /ATTR                 <- metadata (owner, comment, group mask ...)
    /MN                   <- the instruction lines you see on the pendant
       1:  UFRAME_NUM=1 ;
       2:J P[1] 100% FINE ;
    /POS                  <- the taught position data behind P[n]
    P[1]{ GP1: UF : 1, UT : 1, X = ..., ... };
    /END

This module keeps the raw text of every line so that a program can be edited
(taught) and written back out without corrupting anything it does not
understand.
"""
import re

POS_RE = re.compile(r"P\[(\d+)(?::\s*\"([^\"]*)\")?\]\s*\{(.*?)\n\};",
                    re.S)
FIELD_RE = re.compile(r"([XYZWPR]|J[1-6])\s*=\s*(-?\d+\.?\d*)")
UFUT_RE = re.compile(r"UF\s*:\s*(\d+)\s*,\s*UT\s*:\s*(\d+)")
CFG_RE = re.compile(r"CONFIG\s*:\s*'([^']*)'")
LINE_RE = re.compile(r"^\s*(\d+):\s?(.*?)\s*;?\s*$")


class Position:
    def __init__(self, idx, comment="", uf=0, ut=0, kind="xyzwpr",
                 val=None, config="N U T, 0, 0, 0"):
        self.idx = idx
        self.comment = comment
        self.uf = uf
        self.ut = ut
        self.kind = kind                    # 'xyzwpr' or 'joint'
        self.val = val or [0.0] * 6
        self.config = config

    def copy(self):
        return Position(self.idx, self.comment, self.uf, self.ut,
                        self.kind, list(self.val), self.config)

    def to_ls(self):
        head = "P[%d%s]{" % (self.idx,
                             ':   "%s"' % self.comment if self.comment else "")
        if self.kind == "joint":
            body = ("\n   GP1:\n\tUF : %d, UT : %d,\n" % (self.uf, self.ut) +
                    ",\n".join("\tJ%d=  %9.3f deg" % (i + 1, self.val[i])
                               for i in range(6)))
        else:
            n = ["X", "Y", "Z", "W", "P", "R"]
            u = ["mm", "mm", "mm", "deg", "deg", "deg"]
            body = ("\n   GP1:\n\tUF : %d, UT : %d,\t\tCONFIG : '%s',\n"
                    % (self.uf, self.ut, self.config) +
                    ",\n".join("\t%s =  %9.3f  %s" % (n[i], self.val[i], u[i])
                               for i in range(6)))
        return head + body + "\n};"


class Program:
    def __init__(self, name="UNTITLED"):
        self.name = name
        self.comment = ""
        self.attr = []
        self.lines = []                     # list of instruction strings
        self.pos = {}                       # idx -> Position

    # ------------------------------------------------------------ properties
    def next_pos_index(self):
        return max(self.pos) + 1 if self.pos else 1

    # ------------------------------------------------------------------ i/o
    @classmethod
    def parse(cls, text, name=None):
        p = cls(name or "UNTITLED")
        m = re.search(r"/PROG\s+(\S+)", text)
        if m:
            p.name = m.group(1)
        mn = re.search(r"/MN(.*?)(?=/POS|/END)", text, re.S)
        if mn:
            for raw in mn.group(1).splitlines():
                if not raw.strip():
                    continue
                lm = LINE_RE.match(raw)
                if lm:
                    p.lines.append(lm.group(2).strip())
        at = re.search(r"/ATTR(.*?)(?=/MN|/POS|/END)", text, re.S)
        if at:
            p.attr = [l.strip() for l in at.group(1).splitlines() if l.strip()]
            cm = re.search(r'COMMENT\s*=\s*"([^"]*)"', at.group(1))
            if cm:
                p.comment = cm.group(1)
        for pm in POS_RE.finditer(text):
            idx = int(pm.group(1))
            body = pm.group(3)
            uf, ut = 0, 0
            um = UFUT_RE.search(body)
            if um:
                uf, ut = int(um.group(1)), int(um.group(2))
            cfg = CFG_RE.search(body)
            fields = FIELD_RE.findall(body)
            kind = "joint" if any(k.startswith("J") for k, _ in fields) \
                else "xyzwpr"
            order = (["J%d" % i for i in range(1, 7)] if kind == "joint"
                     else ["X", "Y", "Z", "W", "P", "R"])
            d = {k: float(v) for k, v in fields}
            val = [d.get(k, 0.0) for k in order]
            p.pos[idx] = Position(idx, pm.group(2) or "", uf, ut, kind, val,
                                  cfg.group(1) if cfg else "N U T, 0, 0, 0")
        return p

    @classmethod
    def load(cls, path):
        with open(path, "r", errors="replace") as f:
            return cls.parse(f.read(), name=None)

    def to_ls(self):
        out = ["/PROG  %s" % self.name, "/ATTR"]
        out += self.attr or [
            "OWNER\t\t= MNEDITOR;",
            'COMMENT\t\t= "%s";' % self.comment,
            "PROG_SIZE\t= 0;",
            "MEMORY_SIZE\t= 0;",
            "PROTECT\t\t= READ_WRITE;",
            "TCD:  STACK_SIZE\t= 0,",
            "      TASK_PRIORITY\t= 50,",
            "      TIME_SLICE\t= 0,",
            "      BUSY_LAMP_OFF\t= 0,",
            "      ABORT_REQUEST\t= 0,",
            "      PAUSE_REQUEST\t= 0;",
            "DEFAULT_GROUP\t= 1,*,*,*,*;",
            "CONTROL_CODE\t= 00000000 00000000;",
        ]
        out.append("/MN")
        for i, l in enumerate(self.lines, 1):
            motion = re.match(r"^[JLC]\s+P", l)
            out.append("  %3d:%s%s ;" % (i, "" if motion else "  ", l))
        out.append("/POS")
        for idx in sorted(self.pos):
            out.append(self.pos[idx].to_ls())
        out.append("/END")
        return "\n".join(out) + "\n"

    def save(self, path):
        with open(path, "w") as f:
            f.write(self.to_ls())
