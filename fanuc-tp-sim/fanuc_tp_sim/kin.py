"""Kinematics for a generic 6R FANUC-style arm (approximate M-10iD/12 geometry).

Conventions
-----------
* Modified (Craig) DH.
* Pose is expressed FANUC-style as XYZWPR:
      X,Y,Z in mm, W,P,R in degrees,
      R_matrix = Rz(R) * Ry(P) * Rx(W)          (fixed-axis XYZ)
* Joint angles in degrees, FANUC sign convention approximated.

The link lengths are close to an M-10iD/12 but NOT certified. Reach and
singularity locations are therefore indicative, not exact. Everything that
matters for TP practice -- frames, registers, I/O, motion types -- is exact.
"""
from math import sin, cos, asin, atan2, radians, degrees, sqrt, hypot

# alpha_{i-1} (deg), a_{i-1} (mm), d_i (mm), theta_offset_i (deg), j3_couple
DH = [
    (0.0,     0.0,   450.0,   0.0),
    (-90.0, 150.0,     0.0, -90.0),
    (0.0,   740.0,     0.0,   0.0),
    (-90.0, 100.0,   551.0,   0.0),
    (90.0,    0.0,     0.0,   0.0),
    (-90.0,   0.0,   100.0, 180.0),
]

# J3 in FANUC is measured relative to the upper arm, so DH theta3 = J3 + J2.
J3_COUPLED = True

JOINT_LIMITS = [(-170, 170), (-90, 160), (-180, 180),
                (-190, 190), (-140, 140), (-360, 360)]

IDENTITY = [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]


# ---------------------------------------------------------------- matrix math
def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def mat_inv(T):
    """Inverse of a homogeneous transform."""
    R = [[T[i][j] for j in range(3)] for i in range(3)]
    p = [T[i][3] for i in range(3)]
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(Rt[i][k] * p[k] for k in range(3)) for i in range(3)]
    return [[Rt[0][0], Rt[0][1], Rt[0][2], t[0]],
            [Rt[1][0], Rt[1][1], Rt[1][2], t[1]],
            [Rt[2][0], Rt[2][1], Rt[2][2], t[2]],
            [0, 0, 0, 1.0]]


def xyzwpr_to_mat(p):
    x, y, z, w, pt, r = [float(v) for v in p]
    cw, sw = cos(radians(w)), sin(radians(w))
    cp, sp = cos(radians(pt)), sin(radians(pt))
    cr, sr = cos(radians(r)), sin(radians(r))
    return [
        [cr * cp, cr * sp * sw - sr * cw, cr * sp * cw + sr * sw, x],
        [sr * cp, sr * sp * sw + cr * cw, sr * sp * cw - cr * sw, y],
        [-sp,     cp * sw,                cp * cw,                z],
        [0, 0, 0, 1.0],
    ]


def mat_to_xyzwpr(T):
    sp = -T[2][0]
    sp = max(-1.0, min(1.0, sp))
    pt = asin(sp)
    if abs(cos(pt)) < 1e-8:                       # gimbal lock
        r = 0.0
        w = atan2(-T[0][1], T[1][1])
    else:
        r = atan2(T[1][0], T[0][0])
        w = atan2(T[2][1], T[2][2])
    return [T[0][3], T[1][3], T[2][3], degrees(w), degrees(pt), degrees(r)]


# ------------------------------------------------------------------- kinematics
def _dh_mat(alpha, a, d, theta):
    ca, sa = cos(radians(alpha)), sin(radians(alpha))
    ct, st = cos(radians(theta)), sin(radians(theta))
    return [[ct,      -st,      0.0,  a],
            [st * ca,  ct * ca, -sa, -sa * d],
            [st * sa,  ct * sa,  ca,  ca * d],
            [0, 0, 0, 1.0]]


def _dh_thetas(j):
    th = list(j)
    if J3_COUPLED:
        th[2] = j[2] + j[1]
    return th


def fk(j, utool=None):
    """Joint angles (deg) -> flange (or TCP if utool given) pose matrix."""
    th = _dh_thetas(j)
    T = IDENTITY
    for i, (alpha, a, d, off) in enumerate(DH):
        T = mat_mul(T, _dh_mat(alpha, a, d, th[i] + off))
    if utool is not None:
        T = mat_mul(T, utool)
    return T


def fk_links(j):
    """All intermediate frames -- used by the 3D pendant view."""
    th = _dh_thetas(j)
    T = IDENTITY
    out = [T]
    for i, (alpha, a, d, off) in enumerate(DH):
        T = mat_mul(T, _dh_mat(alpha, a, d, th[i] + off))
        out.append(T)
    return out


def pose_error(Tc, Td):
    """6-vector [dx dy dz  rx ry rz] taking current pose to desired pose."""
    e = [Td[i][3] - Tc[i][3] for i in range(3)]
    # rotation error as axis*angle from Rc^T Rd
    R = [[sum(Tc[k][i] * Td[k][j] for k in range(3)) for j in range(3)]
         for i in range(3)]
    ct = max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0))
    ang = atan2(sqrt(max(0.0, 1 - ct * ct)), ct)
    if ang < 1e-9:
        axis = [0.0, 0.0, 0.0]
    else:
        s = 2 * sin(ang)
        axis = [(R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s,
                (R[1][0] - R[0][1]) / s]
        axis = [a * ang for a in axis]
    # rotate axis from tool frame into world
    aw = [sum(Tc[i][k] * axis[k] for k in range(3)) for i in range(3)]
    return e + aw


def jacobian(j, utool=None, h=1e-4):
    """Numeric world-frame Jacobian of the TCP pose (6x6)."""
    T0 = fk(j, utool)
    J = [[0.0] * 6 for _ in range(6)]
    for c in range(6):
        jp = list(j)
        jp[c] += degrees(h)
        e = pose_error(T0, fk(jp, utool))
        for r in range(6):
            J[r][c] = e[r] / h
    return J


def _solve(A, b):
    """Gaussian elimination with partial pivoting; A is n x n (destroyed)."""
    n = len(A)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        piv = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[piv][c]) < 1e-12:
            return None
        M[c], M[piv] = M[piv], M[c]
        for r in range(n):
            if r == c:
                continue
            f = M[r][c] / M[c][c]
            for k in range(c, n + 1):
                M[r][k] -= f * M[c][k]
    return [M[i][n] / M[i][i] for i in range(n)]


def _seed_set(Td, seed):
    """Seeds for the IK retry loop.

    A damped-least-squares solver is a local method: it walks downhill from
    wherever you start it, so a large wrist reorientation can leave it stuck
    against a joint limit. Retrying from a spread of postures is the cheap fix,
    and it is also what the real controller effectively does when it picks a
    configuration (N/F, U/D, T/B) for a taught point.
    """
    j1 = degrees(atan2(Td[1][3], Td[0][3]))
    out = [list(seed)]
    for j4 in (0.0, 180.0, -180.0):
        for j2, j3, j5 in ((0, 0, -30), (25, -25, -60), (50, -60, -40),
                           (-15, 15, -90), (30, 10, -110), (10, -40, 40)):
            out.append([j1, float(j2), float(j3), j4, float(j5), 0.0])
    return out


def ik(Td, seed, utool=None, iters=120, damp=0.08, tol=1e-3, retry=True):
    """Damped-least-squares IK. Returns joints (deg) or None if it fails.

    Damping is what keeps this stable through wrist singularities: instead of
    inverting J it solves (J^T J + lambda^2 I) dq = J^T e, which trades a little
    accuracy for a bounded joint step when J loses rank.
    """
    for s_ in (_seed_set(Td, seed) if retry else [list(seed)]):
        q = _ik_once(Td, s_, utool, iters, damp, tol)
        if q is not None:
            return q
    return None


def _ik_once(Td, seed, utool, iters, damp, tol):
    q = list(seed)
    for _ in range(iters):
        Tc = fk(q, utool)
        e = pose_error(Tc, Td)
        if max(abs(e[i]) for i in range(3)) < tol and \
           max(abs(e[i]) for i in range(3, 6)) < 1e-4:
            return q
        J = jacobian(q, utool)
        JtJ = [[sum(J[k][i] * J[k][j] for k in range(6)) +
                (damp * damp if i == j else 0.0) for j in range(6)]
               for i in range(6)]
        Jte = [sum(J[k][i] * e[k] for k in range(6)) for i in range(6)]
        dq = _solve(JtJ, Jte)
        if dq is None:
            return None
        step = max(abs(d) for d in dq)
        scale = 1.0 if step < 0.15 else 0.15 / step
        for i in range(6):
            q[i] += degrees(dq[i] * scale)
            lo, hi = JOINT_LIMITS[i]
            q[i] = max(lo, min(hi, q[i]))
    Tc = fk(q, utool)
    e = pose_error(Tc, Td)
    if max(abs(v) for v in e[:3]) < 1.0:
        return q
    return None


def dist(p, q):
    return sqrt(sum((p[i] - q[i]) ** 2 for i in range(3)))
