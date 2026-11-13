"""Real stepping for a car EXIT clip: splice the bones of a generated 'steps' npz (Source\\tools\\ai_motion\\exit_steps_gen.py assemble: world matrices of the
73 rig bones for frames c..g in game axes, produced by MotionBricks in-betweening + foot pinning + leg IK) into the exit clip's local tracks.

Used by build_exit_duck.py --steps <npz>. What changes: the locals of the rig bones 2, 4..72 (everything except Root / Trajectory / reference_joint) on the frames c..g,
blended with the existing pose over --steps-ramp frames at both ends (weight 0 at c and at g: those two frames and every frame outside c..g stay bit-identical);
the limb IK helpers (feet, hands, knee / elbow poles) of those frames are re-anchored on the new limbs with the vanilla per-frame offsets (build_entry_clip.limb_anchor_offsets,
the same rule as the entry clips' --limb-ik-final), so the game's leg / arm IK pulls toward the new feet and not the old glide. The Root track (root motion) is never touched:
the generated world poses are re-expressed relative to the existing Root, so planted feet stay planted in world while the game moves the character along the vanilla root path.
"""
import numpy as np

import skel as S

CR = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])  # game axes (Z up, +Y forward) -> glTF (Y up); game_skeleton_bridge.C
SKIP = ('Root', 'Trajectory', 'reference_joint')
# 'lower': Hips, the spine up to Spine2 and both legs (+ feet / toes) come from the generator; Spine3, clavicles, arms, hands, neck and head keep the existing clip (the head duck, the arm / door
# clearance of the vanilla exit), with Spine3 counter-rotated so the upper body keeps its WORLD orientation when the generated lower spine differs
LOWER = ('Hips', 'Spine', 'Spine1', 'Spine2', 'LeftUpLeg', 'LeftLeg', 'LeftFoot', 'LeftToeBase', 'RightUpLeg', 'RightLeg', 'RightFoot', 'RightToeBase')


def smooth01(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def blend_weights(c, g, ramp, ramp_end=None):
    """ramp_end (default = ramp): length of the hand-over to the unchanged last frame g (a steps npz that holds the feet until the end carries ramp_end = 1)"""
    f = np.arange(c, g + 1)
    re_ = ramp if ramp_end is None else int(ramp_end)
    if ramp <= 0 or re_ <= 0:
        w = np.ones(len(f))
        w[0] = w[-1] = 0.0
        return w
    return smooth01((f - c) / float(ramp)) * smooth01((g - f) / float(re_))


def splice(tgt, lt, lr, st, ramp, bones='lower'):
    """lt / lr (T,N,*) modified in place; returns a report dict."""
    c, g = int(st['c']), int(st['g'])
    Wg = np.asarray(st['W'])
    names = [str(x) for x in st['names']]
    par = np.asarray(st['parents'])
    ix = tgt.index
    n = g - c + 1
    assert Wg.shape[0] == n, 'steps npz has %d frames, c..g is %d' % (Wg.shape[0], n)
    Wt0, Wr0 = tgt.world(lt, lr)   # current pose (after the duck): Root / Trajectory worlds stay
    Gr = np.einsum('ij,tbjk,lk->tbil', CR, Wg[:, :, :3, :3], CR)
    Gt = np.einsum('ij,tbj->tbi', CR, Wg[:, :, :3, 3])
    nlt, nlr = lt[c:g + 1].copy(), lr[c:g + 1].copy()
    written = []
    for b, nm in enumerate(names):
        if nm in SKIP or (bones == 'lower' and nm not in LOWER):
            continue
        j, p = ix[nm], int(par[b])
        if names[p] in SKIP:
            pr, pt = Wr0[c:g + 1, ix[names[p]]], Wt0[c:g + 1, ix[names[p]]]
        else:
            pr, pt = Gr[:, p], Gt[:, p]
        inv = np.transpose(pr, (0, 2, 1))
        nlt[:, j] = np.einsum('tij,tj->ti', inv, Gt[:, b] - pt)
        nlr[:, j] = S.m2q(inv @ Gr[:, b])
        written.append(j)
    old_t, old_r = lt[c:g + 1], lr[c:g + 1]
    w = blend_weights(c, g, ramp, int(st['ramp_end']) if 'ramp_end' in st.files else None)[:, None]
    for j in written:
        q1 = nlr[:, j]
        flip = np.sum(q1 * old_r[:, j], -1) < 0
        q1[flip] *= -1
        q = (1 - w) * old_r[:, j] + w * q1
        lr[c:g + 1, j] = q / np.linalg.norm(q, axis=-1, keepdims=True)
        lt[c:g + 1, j] = (1 - w) * old_t[:, j] + w * nlt[:, j]
    if bones == 'lower':  # Spine3 keeps its world orientation (frames with a blend weight > 0)
        k3, k2 = ix['Spine3'], ix['Spine2']
        _, Wr_new = tgt.world(lt, lr)
        Wr_old = Wr0
        for f in range(c, g + 1):
            if w[f - c, 0] > 0:
                lr[f, k3] = S.m2q(Wr_new[f, k2].T @ Wr_old[f, k3])
    lr[:] = S.qcontinuous(lr)
    return dict(c=c, g=g, bones_mode=bones, ramp=int(ramp), bones=[tgt.names[j] for j in written], weights=[round(float(x), 3) for x in w[:, 0]],
                src=str(st['src']) if 'src' in st.files else '', leg_ik=bool(st['leg_ik']) if 'leg_ik' in st.files else None)


def reanchor_helpers(tgt, lt, lr, vWt, vWr, c, g, arm_span=None):
    """feet / hand IK helpers and knee / elbow poles of the frames c+1..g-1 on the current limbs; bones below them keep their world pose (local_from_world).
    arm_span = (c, g) of the GENERATED part (steps npz keys c_gen / g_gen): a steps npz that also covers frames outside it (head / tail extension: only the legs change there)
    re-anchors the arm helpers only inside that span (the duck leaves the arm helpers on the vanilla offsets, re-anchoring them would jump)."""
    import build_entry_clip as B
    from build_exit_duck import limb_frame
    Wt, Wr = tgt.world(lt, lr)
    loffs = B.limb_anchor_offsets(tgt, vWt, vWr)
    done = []
    for b, (a, ot, orr) in loffs.items():
        nm = tgt.names[b]
        if 'PoleVector' in nm:
            continue
        Wr[:, b] = Wr[:, a] @ orr
        Wt[:, b] = Wt[:, a] + np.einsum('tij,tj->ti', Wr[:, a], ot)
        done.append(nm)
    for s_ in ('Left', 'Right'):
        for limb, chain in (('Leg', (s_ + 'UpLeg', s_ + 'Leg', s_ + 'Foot')), ('Arm', (s_ + 'Arm', s_ + 'ForeArm', s_ + 'Hand'))):
            ch = [tgt.index[x] for x in chain]
            vK, vu, vv, vw = limb_frame(vWt, *ch)
            nK, nu, nv, nw = limb_frame(Wt, *ch)
            for pn in ('Torso_%s%s_PoleVector_JNT' % (s_, limb), 'Torso_World_%s%s_PoleVector_JNT' % (s_, limb)):
                if pn not in tgt.index:
                    continue
                pb = tgt.index[pn]
                d = vWt[:, pb] - vK
                cc = np.stack([np.sum(d * vu, -1), np.sum(d * vv, -1), np.sum(d * vw, -1)], -1)
                cc = np.stack([cc[:, 0], np.hypot(cc[:, 1], cc[:, 2]), np.zeros(len(cc))], -1)
                Wt[:, pb] = nK + cc[:, :1] * nu + cc[:, 1:2] * nv + cc[:, 2:] * nw
                done.append(pn)
    flt, flr = tgt.local_from_world(Wt, Wr)
    sl = slice(c + 1, g)
    first = [tgt.index[x] for x in done]
    # the helpers and their descendants (a descendant keeps its WORLD pose, so its local follows the moved helper)
    todo = set(first)
    for i in range(len(tgt.names)):
        if tgt.parent[i] in todo:
            todo.add(i)
    for j in todo:
        sj = sl
        if arm_span is not None:
            k = j
            while k >= 0 and k not in first:
                k = int(tgt.parent[k])
            if k >= 0 and 'Arm' in tgt.names[k]:
                sj = slice(max(c, int(arm_span[0])) + 1, min(g, int(arm_span[1])))
        lt[sj, j] = flt[sj, j]
        lr[sj, j] = flr[sj, j]
    lr[:] = S.qcontinuous(lr)
    return sorted(done)
