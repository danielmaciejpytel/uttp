"""Two-bone IK and the 'duck' correction (lower the hips under a head-height cap, keep feet/hands in world space)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
import skel as S  # noqa: E402


def _norm(v):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-9)


def rot_between(u, v):
    """(T,3),(T,3) -> (T,3,3) minimal rotation taking direction u to v."""
    u = _norm(u)
    v = _norm(v)
    ax = np.cross(u, v)
    s = np.linalg.norm(ax, axis=-1)
    c = np.sum(u * v, axis=-1)
    k = ax / np.maximum(s, 1e-12)[:, None]
    K = np.zeros(u.shape[:-1] + (3, 3))
    K[:, 0, 1], K[:, 0, 2] = -k[:, 2], k[:, 1]
    K[:, 1, 0], K[:, 1, 2] = k[:, 2], -k[:, 0]
    K[:, 2, 0], K[:, 2, 1] = -k[:, 1], k[:, 0]
    I = np.broadcast_to(np.eye(3), K.shape)
    R = I + s[:, None, None] * K + (1 - c)[:, None, None] * (K @ K)
    small = s < 1e-9
    R[small] = np.eye(3)
    return R


def set_world_rot(sk, lt, lr, bone, Wr_new):
    """Set the local rotation of `bone` so that its world rotation becomes Wr_new (parents unchanged)."""
    Wt, Wr = sk.world(lt, lr)
    p = sk.parent[bone]
    loc = np.transpose(Wr[:, p], (0, 2, 1)) @ Wr_new if p >= 0 else Wr_new
    lr[:, bone] = S.qcontinuous(S.m2q(loc))


def two_bone(sk, lt, lr, a, b, c, target, weight=None, max_extend=None, bend_sigma=None, pole=None, pole_weight=None):
    """Move chain a-b-c so that c reaches `target` (T,3); c keeps its world rotation. Modifies lt/lr in place.
    max_extend: reach cap as a fraction of the chain length (None = fully straight), keeps a bend so the knee/elbow
    direction stays defined. bend_sigma: Gaussian smoothing (frames) of the bend direction over time, so the knee / elbow
    cannot swing around the limb axis from one frame to the next.
    pole (T,3) (roofpack3): world position the middle joint (knee / elbow) should stay near; the bend direction is blended toward the direction of its component
    perpendicular to the a->target axis by pole_weight (T,; default 1) - 'keep the elbow where it was while the shoulder moved'."""
    Wt, Wr = sk.world(lt, lr)
    Wc_rot = Wr[:, c].copy()
    A, B, C = Wt[:, a], Wt[:, b], Wt[:, c]
    if weight is not None:
        target = C + (target - C) * weight[:, None]
    la = np.linalg.norm(B - A, axis=-1)
    lb = np.linalg.norm(C - B, axis=-1)
    dv = target - A
    dmax = la + lb - 1e-4 if max_extend is None else max_extend * (la + lb)
    d = np.clip(np.linalg.norm(dv, axis=-1), np.abs(la - lb) + 1e-4, dmax)
    u = _norm(dv)
    # bend direction: the original knee/elbow side; when the limb is nearly straight that direction is undefined,
    # so carry the previous frame's bend direction (re-orthogonalised) to avoid knee flips
    perp = (B - A) - np.sum((B - A) * u, axis=-1, keepdims=True) * u
    pn = np.linalg.norm(perp, axis=-1)
    v = np.zeros_like(perp)
    prev = None
    for i in range(len(u)):
        cand = perp[i] / max(pn[i], 1e-9)
        if prev is not None:
            pp = prev - np.dot(prev, u[i]) * u[i]
            pp /= max(np.linalg.norm(pp), 1e-9)
            wgt = np.clip(pn[i] / (0.15 * la[i]), 0, 1)  # trust the original direction only when clearly bent
            cand = wgt * cand + (1 - wgt) * pp
            cand -= np.dot(cand, u[i]) * u[i]
            cand /= max(np.linalg.norm(cand), 1e-9)
        v[i] = cand
        prev = cand
    if pole is not None:
        pw = np.ones(len(u)) if pole_weight is None else np.asarray(pole_weight, float)
        pp_ = pole - A
        pp_ = pp_ - np.sum(pp_ * u, axis=-1, keepdims=True) * u
        ok_ = np.linalg.norm(pp_, axis=-1) > 1e-4
        vp = _norm(pp_)
        pw = np.where(ok_, pw, 0.0)
        v = _norm(pw[:, None] * vp + (1 - pw[:, None]) * v)
    if bend_sigma:
        vs = smooth_rows(v, bend_sigma)
        if weight is not None:  # frames with a low IK weight keep their own bend direction
            vs = weight[:, None] * vs + (1 - weight[:, None]) * v
        v = vs - np.sum(vs * u, axis=-1, keepdims=True) * u
        v = _norm(v)
    cosa = np.clip((la ** 2 + d ** 2 - lb ** 2) / (2 * la * d), -1, 1)
    sina = np.sqrt(1 - cosa ** 2)
    Bn = A + la[:, None] * (cosa[:, None] * u + sina[:, None] * v)
    R1 = rot_between(B - A, Bn - A)
    set_world_rot(sk, lt, lr, a, R1 @ Wr[:, a])
    Wt2, Wr2 = sk.world(lt, lr)
    R2 = rot_between(Wt2[:, c] - Wt2[:, b], (A + u * d[:, None]) - Wt2[:, b])
    set_world_rot(sk, lt, lr, b, R2 @ Wr2[:, b])
    set_world_rot(sk, lt, lr, c, Wc_rot)
    Wt3, _ = sk.world(lt, lr)
    return float(np.linalg.norm(Wt3[:, c] - target, axis=-1).max())


def smooth(x, sigma):
    if sigma <= 0:
        return x
    r = int(3 * sigma)
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    xp = np.pad(x, r, mode='edge')
    return np.convolve(xp, k, mode='valid')


def smooth_rows(x, sigma):
    """Gaussian smoothing of a (T, k) array along time (edge replicate)."""
    return np.stack([smooth(x[:, k], sigma) for k in range(x.shape[1])], -1)


def maxfilter(x, r):
    xp = np.pad(x, r, mode='edge')
    return np.array([xp[i:i + 2 * r + 1].max() for i in range(len(x))])


LIMBS = {'LeftFoot': ('LeftUpLeg', 'LeftLeg'), 'RightFoot': ('RightUpLeg', 'RightLeg'),
         'LeftHand': ('LeftArm', 'LeftForeArm'), 'RightHand': ('RightArm', 'RightForeArm')}


def follow_weight(p, fps=30.0):
    """0 while an effector is planted/slow, 1 while it moves fast (swing)."""
    v = np.r_[0, np.linalg.norm(np.diff(p, axis=0), axis=-1) * fps]
    return smooth(np.clip((v - 0.6) / 1.4, 0, 1), 2.0)


def hips_drop(sk, lt, lr, cap, sigma=3.0):
    """Lower the hips (in place) so the head stays under `cap` (T,) world height. Returns the drop per frame."""
    Wt, _ = sk.world(lt, lr)
    head = Wt[:, sk.index['Head'], 1]
    over = np.maximum(0.0, head - cap)
    delta = np.maximum(smooth(maxfilter(over, 3), sigma), over)
    delta = smooth(delta, sigma / 2)
    lt[:, sk.index['Hips'], 1] -= delta
    return delta, float(head.max())


def lock_limbs(sk, lt, lr, goals, limbs=('LeftFoot', 'RightFoot', 'LeftHand', 'RightHand'), max_extend=None, bend_sigma=None):
    errs = {}
    for k in limbs:
        a, b = LIMBS[k]
        errs[k] = two_bone(sk, lt, lr, sk.index[a], sk.index[b], sk.index[k], goals[k], max_extend=max_extend, bend_sigma=bend_sigma)
    return errs


def duck(sk, lt, lr, cap, hands=True, sigma=3.0, extra=0.0):
    """Lower the hips so that the head stays under `cap` (T,) world height; feet (and hands) keep their world
    positions through two-bone IK. Returns diagnostics."""
    Wt, _ = sk.world(lt, lr)
    ix = sk.index
    head = Wt[:, ix['Head'], 1]
    over = np.maximum(0.0, head - cap + extra)
    delta = smooth(maxfilter(over, 3), sigma)
    delta = np.maximum(delta, over)  # never under-correct
    delta = smooth(delta, sigma / 2)
    # supporting (slow) feet/hands keep their world position; fast-moving (swinging) ones follow the lowered body
    goals = {}
    follow = {}
    for k in ('LeftFoot', 'RightFoot', 'LeftHand', 'RightHand'):
        p = Wt[:, ix[k]].copy()
        v = np.r_[0, np.linalg.norm(np.diff(p, axis=0), axis=-1) * 30.0]
        f = smooth(np.clip((v - 0.6) / 1.4, 0, 1), 2.0)
        follow[k] = f
        p[:, 1] -= delta * f
        goals[k] = p
    lt[:, ix['Hips'], 1] -= delta  # Root carries yaw only: local Y of Hips is world vertical
    errs = {}
    errs['LeftFoot'] = two_bone(sk, lt, lr, ix['LeftUpLeg'], ix['LeftLeg'], ix['LeftFoot'], goals['LeftFoot'])
    errs['RightFoot'] = two_bone(sk, lt, lr, ix['RightUpLeg'], ix['RightLeg'], ix['RightFoot'], goals['RightFoot'])
    if hands:
        errs['LeftHand'] = two_bone(sk, lt, lr, ix['LeftArm'], ix['LeftForeArm'], ix['LeftHand'], goals['LeftHand'])
        errs['RightHand'] = two_bone(sk, lt, lr, ix['RightArm'], ix['RightForeArm'], ix['RightHand'], goals['RightHand'])
    Wt2, _ = sk.world(lt, lr)
    return dict(head_peak_before=float(head.max()), head_peak_after=float(Wt2[:, ix['Head'], 1].max()),
                hips_drop_max_m=float(delta.max()), ik_max_err_m=errs,
                frames_lowered=int((delta > 0.005).sum()))
