"""3D view of the arm.

Two modes:

    python3 -m fanuc_tp_sim view programs MTEND      # runs the program itself
    python3 -m fanuc_tp_sim view programs --follow   # mirrors a running pendant

In --follow mode the pendant broadcasts its joint angles on UDP 127.0.0.1:5999
after every tick, and this window just draws whatever arrives. That keeps the
pendant dependency-free: matplotlib is needed only for the picture.

The arm is drawn as the polyline through the origins of the DH frames, so what
you see is exactly what the kinematics computes -- link *shapes* are not
modelled, only the joint centres and the tool.
"""
import json
import socket
import sys

from . import kin
from .cell import Cell, DOOR_Y, INFEED, CHUCK, OUTFEED

PORT = 5999
ADDR = ("127.0.0.1", PORT)


def broadcast(sock, c, cell):
    """Called by the pendant; silently does nothing if no viewer is listening."""
    try:
        sock.sendto(json.dumps({
            "j": [round(v, 3) for v in c.joints],
            "ut": c.utool[c.utnum],
            "door": cell.door if cell else "open",
            "grip": cell.grip if cell else "open",
            "part": bool(cell and cell.part_grip),
            "msg": ("ALARM %s %s" % c.alarm) if c.alarm else c.msg,
        }).encode(), ADDR)
    except OSError:
        pass


def _box(ax, x0, x1, y0, y1, z0, z1, **kw):
    pts = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
           (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    for a, b in edges:
        ax.plot(*zip(pts[a], pts[b]), **kw)


def main(path, progname=None, follow=False):
    try:
        import matplotlib
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError:
        print("needs matplotlib:  pip install matplotlib")
        return 1

    sock = None
    c = cell = None
    if follow:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(ADDR)
        sock.setblocking(False)
        state = {"j": [0, 0, 0, 0, -30, 0], "ut": [0, 0, 200, 0, 0, 0],
                 "door": "closed", "grip": "open", "part": False, "msg": ""}
    else:
        from .__main__ import build
        c, cell, progs = build(path)
        c.select(progname or progs[0].name)
        c.paused = False
        c.running = True
        state = None

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    def draw(_frame):
        nonlocal state
        if follow:
            try:
                while True:
                    data, _ = sock.recvfrom(4096)
                    state = json.loads(data.decode())
            except (BlockingIOError, OSError):
                pass
            j = state["j"]
            ut = kin.xyzwpr_to_mat(state["ut"])
            door, grip, part, msg = (state["door"], state["grip"],
                                     state["part"], state["msg"])
        else:
            for _ in range(3):
                c.step(0.05)
            j = c.joints
            ut = c.ut_mat()
            door, grip = cell.door, cell.grip
            part = cell.part_grip
            msg = ("ALARM %s %s" % c.alarm) if c.alarm else c.msg

        ax.clear()
        ax.set_xlim(-200, 1500)
        ax.set_ylim(-1100, 900)
        ax.set_zlim(0, 1700)
        ax.set_box_aspect((1700, 2000, 1700))
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        # machine enclosure and its door plane
        _box(ax, 500, 1300, DOOR_Y, 900, 0, 1300, color="0.75", lw=0.7)
        dc = "tab:green" if door == "open" else "tab:red"
        _box(ax, 500, 1300, DOOR_Y - 5, DOOR_Y + 5, 200, 1200,
             color=dc, lw=1.6 if door != "open" else 0.6)

        for name, st, col in (("infeed", INFEED, "tab:blue"),
                              ("chuck", CHUCK, "tab:orange"),
                              ("outfeed", OUTFEED, "tab:green")):
            ax.scatter(*st, s=60, color=col)
            ax.text(st[0], st[1], st[2] + 60, name, fontsize=8, color=col)

        frames = kin.fk_links(j)
        pts = [(T[0][3], T[1][3], T[2][3]) for T in frames]
        tcp = kin.mat_mul(frames[-1], ut)
        pts.append((tcp[0][3], tcp[1][3], tcp[2][3]))
        ax.plot(*zip(*pts), "-o", color="tab:gray", lw=4, ms=5)
        ax.plot(*zip(pts[-2], pts[-1]),
                color="tab:red" if grip == "closed" else "0.4", lw=3)

        # tool frame axes at the TCP
        o = pts[-1]
        for k, col in enumerate(("r", "g", "b")):
            v = [tcp[i][k] * 120 for i in range(3)]
            ax.plot(*zip(o, (o[0] + v[0], o[1] + v[1], o[2] + v[2])),
                    color=col, lw=2)
        if part:
            ax.scatter(*o, s=90, marker="s", color="tab:purple")

        ax.set_title("door:%s  grip:%s  %s" % (door, grip, msg), fontsize=10)
        ax.text2D(0.02, 0.02, "J " + " ".join("%.0f" % v for v in j),
                  transform=ax.transAxes, fontsize=8)

    anim = FuncAnimation(fig, draw, interval=100, cache_frame_data=False)
    fig._anim = anim
    plt.show()
    return 0


if __name__ == "__main__":
    a = sys.argv[1:]
    main(a[0] if a else "programs",
         a[1] if len(a) > 1 and not a[1].startswith("-") else None,
         "--follow" in a)
