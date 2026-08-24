"""A simulated lathe machine-tending cell.

The point of this module is the part your text editor cannot teach you: the
*plant*. Outputs do not take effect instantly, inputs come back after a delay,
and violating an interlock faults the robot instead of quietly succeeding.

I/O map (edit freely -- this is just a dict-driven model)
--------------------------------------------------------
  DO[1] gripper close        DI[1] gripper closed
  DO[2] gripper open         DI[2] gripper open
  DO[3] chuck close          DI[3] chuck closed
  DO[4] chuck open           DI[4] chuck open
  DO[5] cycle start          DI[5] door open
  DO[6] door open            DI[6] door closed
  DO[7] door close           DI[7] machine in cycle
                             DI[8] cycle complete
                             DI[9] part present in gripper
                             DI[10] part present at infeed
                             DI[11] machine alarm

Geometry: the machine sits at +Y. Anything with the TCP at Y > DOOR_Y is
"inside the machine". Entering with the door not fully open, or while the
spindle is in cycle, is a collision.
"""
DOOR_Y = 250.0
INFEED = (650.0, -550.0, 300.0)
CHUCK = (900.0, 500.0, 700.0)
OUTFEED = (650.0, -800.0, 300.0)
NEAR = 90.0                     # mm tolerance for "at" a station

IO_NAMES = {
    ("DO", 1): "grip close", ("DO", 2): "grip open",
    ("DO", 3): "chuck close", ("DO", 4): "chuck open",
    ("DO", 5): "cycle start", ("DO", 6): "door open", ("DO", 7): "door close",
    ("DI", 1): "grip closed", ("DI", 2): "grip open",
    ("DI", 3): "chuck closed", ("DI", 4): "chuck open",
    ("DI", 5): "door open", ("DI", 6): "door closed",
    ("DI", 7): "in cycle", ("DI", 8): "cycle done",
    ("DI", 9): "part in grip", ("DI", 10): "part at infeed",
    ("DI", 11): "mach alarm",
}


def _near(p, s, tol=NEAR):
    return (abs(p[0] - s[0]) < tol and abs(p[1] - s[1]) < tol
            and abs(p[2] - s[2]) < tol)


class Cell:
    GRIP_T = 0.4
    CHUCK_T = 0.6
    DOOR_T = 1.5
    CYCLE_T = 6.0

    def __init__(self):
        self.reset()

    def reset(self):
        self.grip = "open"        # open|closing|closed|opening
        self.chuck = "open"
        self.door = "closed"
        self.t = {"grip": 0.0, "chuck": 0.0, "door": 0.0, "cycle": 0.0}
        self.cycle = "idle"       # idle|running|done
        self.part_grip = False
        self.part_chuck = False
        self.part_done = False
        self.infeed_parts = 20
        self.produced = 0
        self.collision = None
        self.inside_prev = False

    # ------------------------------------------------------------------ tick
    def step(self, c, dt):
        for k, v in IO_NAMES.items():
            c.io_comment[k] = v
        p = c.tcp_world()
        tcp = (p[0][3], p[1][3], p[2][3])
        inside = tcp[1] > DOOR_Y

        # --- actuators -------------------------------------------------
        self._grip(c, dt, tcp)
        self._chuck(c, dt, tcp)
        self._door(c, dt, inside)
        self._cycle(c, dt, inside)

        # --- sensor feedback -------------------------------------------
        c.DI[1] = 1 if self.grip == "closed" else 0
        c.DI[2] = 1 if self.grip == "open" else 0
        c.DI[3] = 1 if self.chuck == "closed" else 0
        c.DI[4] = 1 if self.chuck == "open" else 0
        c.DI[5] = 1 if self.door == "open" else 0
        c.DI[6] = 1 if self.door == "closed" else 0
        c.DI[7] = 1 if self.cycle == "running" else 0
        c.DI[8] = 1 if self.cycle == "done" else 0
        c.DI[9] = 1 if self.part_grip else 0
        c.DI[10] = 1 if self.infeed_parts > 0 else 0
        for k in c.forced:
            c.DI[k[1]] = 1 if k[0] == "DI+" else 0

        # --- interlock violations --------------------------------------
        # Edge-triggered on entry: otherwise a faulted robot that is already
        # inside re-alarms forever and you can never jog it back out.
        if inside and not self.inside_prev:
            if self.door != "open":
                self._crash(c, "robot entered machine, door not open")
            elif self.cycle == "running":
                self._crash(c, "robot inside while spindle in cycle")
        self.inside_prev = inside

    # ------------------------------------------------------------ actuators
    def _grip(self, c, dt, tcp):
        want = "closed" if c.DO[1] else ("open" if c.DO[2] else None)
        if want == "closed" and self.grip in ("open", "opening"):
            self.grip, self.t["grip"] = "closing", 0.0
        if want == "open" and self.grip in ("closed", "closing"):
            self.grip, self.t["grip"] = "opening", 0.0
        if self.grip in ("closing", "opening"):
            self.t["grip"] += dt
            if self.t["grip"] >= self.GRIP_T:
                if self.grip == "closing":
                    self.grip = "closed"
                    self._on_grip_closed(c, tcp)
                else:
                    self.grip = "open"
                    self._on_grip_open(c, tcp)

    def _on_grip_closed(self, c, tcp):
        if _near(tcp, INFEED) and self.infeed_parts > 0 and not self.part_grip:
            self.infeed_parts -= 1
            self.part_grip = True
        elif _near(tcp, CHUCK) and self.part_chuck and self.chuck == "open":
            self.part_chuck = False
            self.part_grip = True
        # closing on a part still held by a closed chuck is normal practice:
        # grip first, then unchuck. Transfer happens when the chuck opens.

    def _on_grip_open(self, c, tcp):
        if not self.part_grip:
            return
        if _near(tcp, CHUCK):
            if self.chuck == "closed" and self.part_chuck:
                self.part_grip = False          # handed off to the chuck
            else:
                self._crash(c, "part released with chuck not holding it")
        elif _near(tcp, OUTFEED):
            self.part_grip = False
            if self.part_done:
                self.produced += 1
                self.part_done = False
        else:
            self._crash(c, "part released away from a station")

    def _chuck(self, c, dt, tcp):
        want = "closed" if c.DO[3] else ("open" if c.DO[4] else None)
        if want and want != self.chuck and self.t["chuck"] == 0.0:
            self.t["chuck"] = 1e-6
        if self.t["chuck"] > 0.0:
            self.t["chuck"] += dt
            if self.t["chuck"] >= self.CHUCK_T:
                self.t["chuck"] = 0.0
                if want == "closed":
                    if self.part_grip and _near(tcp, CHUCK):
                        self.part_chuck = True
                    self.chuck = "closed"
                elif want == "open":
                    self.chuck = "open"
                    if self.part_chuck:
                        if self.grip == "closed" and _near(tcp, CHUCK):
                            self.part_chuck, self.part_grip = False, True
                        else:
                            self.part_chuck = False
                            self._crash(c, "chuck opened, nothing holding "
                                           "the part -- part dropped")

    def _door(self, c, dt, inside):
        want = "open" if c.DO[6] else ("closed" if c.DO[7] else None)
        if want and want != self.door and self.t["door"] == 0.0:
            if want == "closed" and inside:
                self._crash(c, "door commanded closed with robot inside")
                return
            self.t["door"] = 1e-6
            self.door = "moving"
        if self.t["door"] > 0.0:
            self.t["door"] += dt
            if self.t["door"] >= self.DOOR_T:
                self.t["door"] = 0.0
                self.door = want or "closed"

    def _cycle(self, c, dt, inside):
        if c.DO[5] and self.cycle == "idle":
            if inside:
                self._crash(c, "cycle start with robot inside")
                return
            if not self.part_chuck or self.chuck != "closed":
                c.DI[11] = 1
                return
            self.cycle, self.t["cycle"] = "running", 0.0
        if self.cycle == "running":
            self.t["cycle"] += dt
            if self.t["cycle"] >= self.CYCLE_T:
                self.cycle = "done"
                self.part_done = True
        if self.cycle == "done" and not c.DO[5] and not self.part_chuck:
            self.cycle = "idle"

    def _crash(self, c, why):
        if self.collision:
            return
        self.collision = why
        c.fault("SRVO-050", "collision detect: " + why)

    @staticmethod
    def configure(c):
        """Frames as they would be taught on the real cell."""
        c.utool[1] = [0.0, 0.0, 200.0, 0.0, 0.0, 0.0]     # gripper TCP
        c.uframe[1] = [900.0, 500.0, 700.0, -90.0, 0.0, 0.0]   # chuck frame
        c.uframe[2] = [650.0, -550.0, 300.0, 180.0, 0.0, 0.0]  # infeed frame

    # ------------------------------------------------------------- display
    def status(self):
        return ("grip:%-7s chuck:%-7s door:%-7s spindle:%-7s  "
                "part[grip:%d chuck:%d done:%d]  infeed:%d  made:%d"
                % (self.grip, self.chuck, self.door, self.cycle,
                   self.part_grip, self.part_chuck, self.part_done,
                   self.infeed_parts, self.produced))
