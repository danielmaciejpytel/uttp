"""Real stepping for a game car EXIT clip (the vanilla exit glides: both feet stay on the floor and slide). Offline generator side:
the MotionBricks in-betweening of the exit tail (context = frames c-3..c of the exit clip, target = frames g.. of the same clip), foot pinning,
scoring and the conversion of the chosen sample back to the game rig. The result is a 'steps' npz that build_exit_duck.py --steps applies to the
GLB (locals of the 73 rig bones for frames c..g; Root / motion extraction untouched).

  python -I exit_steps_gen.py prep   <cur.dump.npz> <out_scene.dump.npz>        floor fix (feet on z = rest ankle height), saves dz in the npz
  python -I exit_steps_gen.py score  <scene.dump.npz> <tpose.npz> <soma_scene.npz> <span.npz> [...]   metrics of generated spans in the SOMA domain
  python -I exit_steps_gen.py assemble <scene.dump.npz> <tpose.npz> <span.npz> <c> <g> <out_steps.npz> [--leg-ik]
        SOMA span -> game rig world matrices (game_skeleton_bridge.cp_inv, template = the scene, gap blend), floor fix removed again
  python -I exit_steps_gen.py metrics <dump.npz> [<dump2.npz> ...] [--span c:g]    foot slide while in contact / steps / hips deviation (game rig, metres)
  python -I exit_steps_gen.py plan <scene.dump.npz> <tpose.npz> <out_steps.npz> [--k0 21] [--tail 60] [--stance 52] [--lift 10] [--rlift 8]     (opt-in, 2026-11-05) FOOTSTEP PLANNER instead of MotionBricks:
        the hips / spine / arms stay the scene's (head-duck clip), only the legs change (two-bone leg IK, the feet keep the scene's world orientation): the left support foot is locked from k0 to its
        reach limit, the right foot keeps the vanilla path over the sill and lands <= --stance cm from the final left foot, then the left foot swings to its final spot (smooth, one long stride), the right foot
        follows with a short step, both feet are held on the last frame's ankles from the moment they land. Same npz format as assemble --head / --tail (c = k0-3, g = tail, ramp_end 1)
  python -I exit_steps_gen.py pinpre <scene.dump.npz> <tpose.npz> <out.dump.npz> <side> <k0> <k1>      (opt-in, 2026-11-05) foot lock of the SUPPORT foot before the generated span: the ankle of
        frames k0..k1 stays at its frame-k0 position (two-bone leg IK, the foot keeps its world orientation); feed the result to fwd / clip_inbetween (context c = k1) and assemble
  assemble ... [--head C0:K0] [--tail T]   (opt-in) the steps npz also covers frames C0..c-1 (copied from the, possibly pinned, scene; foot contacts: left planted from K0) and g+1..T
        (T = last frame of the clip): the feet that are planted at g are held at their frame-T ankle position over the whole last planted run, frames g+1..T = the scene with that leg IK, the npz
        carries ramp_end = 1 so that exit_steps.splice hands over to the unchanged last frame in ONE frame (the foot does not glide in the last frames); c / g of the npz = C0 / T

Scene conventions: game rig dump of game_skeleton_bridge.py (W [T,73,4,4], Z up, root motion included). The generator assumes the floor at height 0; game exit
clips have the seat origin (floor ~ -0.37 m), so the whole scene is shifted by dz before the bridge and the result is shifted back by -dz.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
SIDES = ('Left', 'Right')


def floor_height(W, names, frames):
    """floor level of the clip: lowest ankle height over the frames (the feet stand on it for the whole tail)."""
    fa = [names.index(s + 'Foot') for s in SIDES]
    return float(W[frames][:, fa, 2, 3].min())


def prep(a):
    d = np.load(a[0], allow_pickle=True)
    W = d['W'].copy()
    names = [str(x) for x in d['names']]
    fa = [names.index(s + 'Foot') for s in SIDES]
    T = len(W)
    floor = floor_height(W, names, range(int(T * 0.65), T))
    dz = float(min(d['rest'][i, 2, 3] for i in fa)) - floor
    W[:, :, 2, 3] += dz
    keep = {k: d[k] for k in d.files if k not in ('W', 'dz')}
    np.savez(a[1], W=W, dz=dz, **keep)
    print('PREP %s floor %.3f m dz %.3f m frames %d' % (a[1], floor, dz, T))


def contact_metrics(W, names, f0, f1, h=0.03, rest=None):
    """per side: contact frames (pivot = lower of ankle / toe joint, height above that joint's floor level < h) and horizontal speed of the pivot
    in cm / frame over consecutive contact frames inside [f0, f1]. Floor level per joint = its minimum height over the whole clip."""
    out = {}
    for s in SIDES:
        ja, jt = names.index(s + 'Foot'), names.index(s + 'ToeBase')
        za, zt = W[:, ja, 2, 3], W[:, jt, 2, 3]
        ha, ht = za - za.min(), zt - zt.min()
        piv = np.where(ha <= ht, ja, jt)
        hp = np.minimum(ha, ht)
        P = np.array([W[f, piv[f], :3, 3] for f in range(len(W))])
        con = hp < h
        sl = []
        for f in range(max(f0, 1), f1 + 1):
            if con[f] and con[f - 1] and piv[f] == piv[f - 1]:
                sl.append(np.linalg.norm((P[f] - P[f - 1])[:2]) * 100)
        # separate raw foot pin check: ankle speed in contact
        out[s] = dict(con=con, piv=piv, slide=np.array(sl))
    return out


def swing_phases(con, f0, f1, min_len=3):
    """number of swing phases (runs of non-contact frames of at least min_len) strictly inside (f0, f1)"""
    runs, f = [], f0
    while f <= f1:
        if not con[f]:
            e = f
            while e + 1 <= f1 and not con[e + 1]:
                e += 1
            if e - f + 1 >= min_len and f > f0 and e < f1:
                runs.append((f, e))
            f = e + 1
        else:
            f += 1
    return runs


def metrics(a):
    span = None
    if '--span' in a:
        span = tuple(int(x) for x in a[a.index('--span') + 1].split(':'))
        a = [x for i, x in enumerate(a) if x != '--span' and (i == 0 or a[i - 1] != '--span')]
    for p in a:
        d = np.load(p, allow_pickle=True)
        W, names = d['W'], [str(x) for x in d['names']]
        f0, f1 = span if span else (0, len(W) - 1)
        m = contact_metrics(W, names, f0, f1)
        line = []
        for s in SIDES:
            sl = m[s]['slide']
            line.append('%s contact %d fr slide mean %.2f p90 %.2f max %.2f cm/fr swings %s' % (
                s, int(m[s]['con'][f0:f1 + 1].sum()), sl.mean() if len(sl) else -1, np.percentile(sl, 90) if len(sl) else -1,
                sl.max() if len(sl) else -1, swing_phases(m[s]['con'], f0, f1)))
        print('METRICS %-34s [%d..%d] %s' % (os.path.basename(p), f0, f1, ' | '.join(line)))


def quality(W, names, base_W, c, g, fcs, floor_ref):
    """numbers of one candidate clip (game rig dump, metres) over the span c..g: slide = horizontal ankle speed (cm / frame) over frames that are planted
    in the generator's own contact labels on both consecutive frames; steps = swing runs (not planted, >= 3 frames) with the lift above the planted height and
    the travel; floor = lowest ankle / toe height of the span against the vanilla last frame (floor_ref: ankle, toe heights of the last vanilla frame)."""
    ix = {n: i for i, n in enumerate(names)}
    n = g - c + 1
    out = {}
    for s_ in SIDES:
        ja, jt = ix[s_ + 'Foot'], ix[s_ + 'ToeBase']
        pl = planted_mask(fcs, s_, n)
        P = W[c:g + 1, ja, :3, 3]
        z = P[:, 2]
        sp = np.r_[0, np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1)] * 100
        sl = np.array([sp[f] for f in range(1, n) if pl[f] and pl[f - 1]])
        rest_h = float(np.median(z[pl])) if pl.any() else float(z.min())
        swings, f = [], 0
        while f < n:
            if not pl[f]:
                e = f
                while e + 1 < n and not pl[e + 1]:
                    e += 1
                if e - f + 1 >= 3:
                    a_, b_ = max(f - 1, 0), min(e + 1, n - 1)
                    swings.append((c + f, c + e, round(float(z[f:e + 1].max() - rest_h) * 100, 1), round(float(np.linalg.norm(P[b_, :2] - P[a_, :2])) * 100),
                                   round(float(sp[f:e + 1].max()), 1)))
                f = e + 1
            else:
                f += 1
        low = min(W[c:g + 1, ja, 2, 3].min() - floor_ref[s_][0], W[c:g + 1, jt, 2, 3].min() - floor_ref[s_][1]) * 100
        out[s_] = dict(slide_mean=float(sl.mean()) if len(sl) else -1, slide_p90=float(np.percentile(sl, 90)) if len(sl) else -1, slide_max=float(sl.max()) if len(sl) else -1,
                       swings=swings, below_floor_cm=float(low), planted_frames=int(pl.sum()))
    hips, hb = W[:, ix['Hips'], :3, 3], base_W[:, ix['Hips'], :3, 3]
    out['hips_dev_xy_cm'] = float(np.linalg.norm((hips - hb)[c:g + 1, :2], axis=1).max() * 100)
    out['hips_dev_z_cm'] = float(np.abs((hips - hb)[c:g + 1, 2]).max() * 100)
    la, ra = W[:, ix['LeftFoot'], :3, 3], W[:, ix['RightFoot'], :3, 3]
    out['feet_min_dist_cm'] = float(np.linalg.norm((la - ra)[c:g + 1, :2], axis=1).min() * 100)
    kn = []
    for s_ in SIDES:
        u, k_, a_ = (W[:, ix[s_ + x], :3, 3] for x in ('UpLeg', 'Leg', 'Foot'))
        v1, v2 = u - k_, a_ - k_
        cs = (v1 * v2).sum(1) / (np.linalg.norm(v1, axis=1) * np.linalg.norm(v2, axis=1))
        kn.append(np.abs(np.diff(np.degrees(np.arccos(np.clip(cs, -1, 1)))[c:g + 1])).max())
    out['knee_step_deg'] = float(max(kn))
    out['n_steps'] = sum(1 for s_ in SIDES for r in out[s_]['swings'] if r[3] >= 20)
    return out


def rank(a):
    base = np.load(a[0], allow_pickle=True)
    names = [str(x) for x in base['names']]
    ix = {n: i for i, n in enumerate(names)}
    fr = {s_: (float(base['W'][40:, ix[s_ + 'Foot'], 2, 3].min()), float(base['W'][40:, ix[s_ + 'ToeBase'], 2, 3].min())) for s_ in SIDES}
    rows = []
    for p in a[1:]:
        st = np.load(p, allow_pickle=True)
        c, g = int(st['c']), int(st['g'])
        W = base['W'].copy()
        W[c:g + 1, 2:] = st['W'][:, 2:]
        q = quality(W, names, base['W'], c, g, st['foot_contacts'], fr)
        rows.append((p, q))
        print('RANK %-26s steps %d slide L %.2f/%.2f/%.1f R %.2f/%.2f/%.1f (mean/p90/max) floor L %.1f R %.1f hipsdev xy %.0f z %.0f knee %.0f feetmin %.0f | L %s R %s' % (
            os.path.basename(p).replace('.steps.npz', ''), q['n_steps'], q['Left']['slide_mean'], q['Left']['slide_p90'], q['Left']['slide_max'], q['Right']['slide_mean'],
            q['Right']['slide_p90'], q['Right']['slide_max'], q['Left']['below_floor_cm'], q['Right']['below_floor_cm'], q['hips_dev_xy_cm'], q['hips_dev_z_cm'], q['knee_step_deg'],
            q['feet_min_dist_cm'], q['Left']['swings'], q['Right']['swings']))
    return rows


def planted_mask(fc, side, n):
    """generator foot contact probabilities [n,4] (L heel / L toe / R heel / R toe) -> planted flags of one side (one-frame gaps closed)"""
    cols = (0, 1) if side == 'Left' else (2, 3)
    m = np.asarray(fc)[:, cols].max(1) > 0.5 if len(fc) == n else np.zeros(n, bool)
    for f in range(1, n - 1):
        if not m[f] and m[f - 1] and m[f + 1]:
            m[f] = True
    return m


def smooth01(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def end_offsets(P, planted, mc, mg, ramp=4):
    """per-frame ankle offset (n,3) that moves the generated ankle path onto the original ankle at the first (mc) and the last (mg) frame
    (the SOMA -> game leg proportions differ, so the leg IK lands a few cm off the original there): constant over the planted run that touches
    an end, faded out over the swing next to it by the progress of the foot between its two planted positions."""
    n = len(P)
    off = np.zeros((n, 3))
    runs = []
    f = 0
    while f < n:
        if planted[f]:
            e = f
            while e + 1 < n and planted[e + 1]:
                e += 1
            runs.append((f, e))
            f = e + 1
        else:
            f += 1

    def fade(a, b, t_idx, rising):  # weight 0..1 over the swing frames t_idx between planted positions a..b (frame indices)
        d = np.linalg.norm(P[b] - P[a])
        w = np.array([np.linalg.norm(P[t] - P[a]) / d if d > 1e-6 else (t - a) / max(1, b - a) for t in t_idx])
        w = smooth01(w)
        return w if rising else 1 - w
    # last frame
    if runs and runs[-1][1] == n - 1:
        s_e = runs[-1][0]
        off[s_e:] += mg
        if len(runs) > 1:
            pe = runs[-2][1]
            idx = list(range(pe + 1, s_e))
            if idx:
                off[idx] += mg * fade(pe, s_e, idx, True)[:, None]
        else:
            idx = list(range(0, s_e))
            off[idx] += mg * smooth01(np.array(idx) / max(1, s_e))[:, None]
    else:
        idx = np.arange(max(0, n - ramp), n)
        off[idx] += mg * smooth01((idx - (n - 1 - ramp)) / ramp)[:, None]
    # first frame
    if runs and runs[0][0] == 0:
        e_s = runs[0][1]
        off[:e_s + 1] += mc
        if len(runs) > 1:
            ns = runs[1][0]
            idx = list(range(e_s + 1, ns))
            if idx:
                off[idx] += mc * fade(e_s, ns, idx, False)[:, None]
        else:
            idx = np.arange(e_s + 1, n)
            off[idx] += mc * (1 - smooth01((idx - e_s) / max(1, n - 1 - e_s)))[:, None]
    else:
        idx = np.arange(0, min(n, ramp + 1))
        off[idx] += mc * (1 - smooth01(idx / ramp))[:, None]
    return off


def ik_to(br, Wt, side, goal, pole_dir=None):
    """two-bone leg IK of one side: the ankle goes to goal (world, rig units), the knee keeps its bending plane, the foot keeps its world orientation
    (same maths as skeleton_bridge.Bridge._leg_ik with an explicit goal)"""
    import skeleton_bridge as SB
    th, ca, fo = (br.uidx[x] for x in br.legs[side])
    hip, knee0, ank0 = Wt[th, :3, 3].copy(), Wt[ca, :3, 3].copy(), Wt[fo, :3, 3].copy()
    la, lb = np.linalg.norm(knee0 - hip), np.linalg.norm(ank0 - knee0)
    d = goal - hip
    dist = min(np.linalg.norm(d), la + lb - 1e-5)
    n = d / np.linalg.norm(d)
    pole = knee0 - hip if pole_dir is None else pole_dir
    pp = pole - n * pole.dot(n)
    pp /= np.linalg.norm(pp)
    x = (la * la - lb * lb + dist * dist) / (2 * dist)
    h = np.sqrt(max(0.0, la * la - x * x))
    knee = hip + n * x + pp * h
    rf = Wt[fo, :3, :3].copy()
    br._rotate_subtree(Wt, th, SB.rot_between(knee0 - hip, knee - hip), hip)
    br._rotate_subtree(Wt, ca, SB.rot_between(Wt[fo, :3, 3] - Wt[ca, :3, 3], goal - Wt[ca, :3, 3]), Wt[ca, :3, 3].copy())
    br._rotate_subtree(Wt, fo, rf @ Wt[fo, :3, :3].T, Wt[fo, :3, 3].copy())


def shape_feet(br, W, fcs, k, wnat, pin_cm=1.0, smax=0.25, ease=1.0, masks=None):
    """(opt-in, assemble --narrow K [--natural CM] [--pin-glide CM]) GENTLE correction of the MotionBricks stepping, hips / spine / step timing untouched:
    per side the ankle gets a horizontal offset that is CONSTANT over every planted run (no new glide) and is faded in / out over the swings between the runs (smooth01 of time).
    Runs that touch the first / last frame get no offset (the end fix already pinned them). An interior planted run is
      (1) pinned: when its ankle travels more than pin_cm the ankle is held at the mean position of the run (glide while carrying weight removed, no new step),
      (2) moved toward the other foot by k * (widest ankle distance during the run - wnat) (at most smax m): the stride is only partly narrowed to a natural width.
    Interior runs are processed in time order, the other foot's already decided offsets count. Returns a report dict."""
    n = len(W)
    fo = {sd: br.uidx[br.legs[sd][2]] for sd in SIDES}
    P = {sd: W[:, fo[sd], :3, 3].copy() for sd in SIDES}
    runs = {}
    for sd in SIDES:
        pm = planted_mask(fcs, sd, n) if masks is None else masks[sd]
        rr, f = [], 0
        while f < n:
            if pm[f]:
                e = f
                while e + 1 < n and pm[e + 1]:
                    e += 1
                rr.append((f, e))
                f = e + 1
            else:
                f += 1
        runs[sd] = rr
    runoff = {sd: {} for sd in SIDES}   # (a, b) -> per-frame xy offset array of the run (fixed runs: zeros)

    def offsets(sd):
        off = np.zeros((n, 3))
        rr = runs[sd]
        key_frames = []   # (frame, offset) anchors: run first / last frame
        for (a_, b_) in rr:
            o = runoff[sd].get((a_, b_))
            if o is None:
                o = np.zeros((b_ - a_ + 1, 3))
            off[a_:b_ + 1] = o
            key_frames += [(a_, o[0]), (b_, o[-1])]
        if not key_frames or key_frames[0][0] > 0:
            key_frames.insert(0, (0, np.zeros(3)))
        if key_frames[-1][0] < n - 1:
            key_frames.append((n - 1, np.zeros(3)))
        pm = np.zeros(n, bool)
        for (a_, b_) in rr:
            pm[a_:b_ + 1] = True
        for (fa, oa), (fb, ob) in zip(key_frames[:-1], key_frames[1:]):
            for t in range(fa + 1, fb):
                if not pm[t]:
                    u = (t - fa) / float(fb - fa)
                    u = (1 - ease) * u + ease * smooth01(u) if ease >= 0 else (1 + ease) * u - ease * (1 - (1 - u) ** 2)   # ease 1 = smoothstep, 0 = linear, < 0: blend toward ease-out (fast start, soft landing)
                    off[t] = oa + (ob - oa) * u
        return off

    rep = dict(pinned=[], shifted=[])
    todo = sorted([(a_, b_, sd) for sd in SIDES for (a_, b_) in runs[sd] if a_ > 0 and b_ < n - 1])
    for a_, b_, sd in todo:
        other = SIDES[1] if sd == SIDES[0] else SIDES[0]
        seg = P[sd][a_:b_ + 1].copy()
        o = np.zeros_like(seg)
        if pin_cm > 0 and len(seg) >= 3:
            travel = float(np.linalg.norm(seg[:, :2].max(0) - seg[:, :2].min(0))) * 100
            if travel > pin_cm:
                o[:, :2] = seg[:, :2].mean(0) - seg[:, :2]
                rep['pinned'].append((sd, a_, b_, round(travel, 1)))
        if k > 0:
            q = P[other] + offsets(other)
            pos = seg + o
            dd = np.linalg.norm((q[a_:b_ + 1] - pos)[:, :2], axis=1)
            tm = int(dd.argmax())
            dmax = float(dd[tm])
            if dmax > wnat:
                m = min(k * (dmax - wnat), smax)
                u = (q[a_ + tm] - pos[tm])[:2]
                u = u / max(np.linalg.norm(u), 1e-9)
                o[:, :2] += m * u
                rep['shifted'].append((sd, a_, b_, round(dmax * 100, 1), round(m * 100, 1)))
        runoff[sd][(a_, b_)] = o
    for sd in SIDES:
        off = offsets(sd)
        for t in range(n):
            if np.linalg.norm(off[t]) > 1e-5:
                ik_to(br, W[t], sd, P[sd][t] + off[t])
    return rep


def pin_by_height(br, d, W, pc):
    """glide pin of the planted runs of W by HEIGHT (the checker's contact rule: lower of ankle / toe within 3 cm of that joint's floor, runs >= 5 frames; runs touching the first / last frame are left alone)"""
    masks = {}
    for sd in SIDES:
        fo, tb = br.uidx[br.legs[sd][2]], br.uidx[sd + 'ToeBase']
        fl_a, fl_t = d['W'][40:, fo, 2, 3].min(), d['W'][40:, tb, 2, 3].min()
        con = np.minimum(W[:, fo, 2, 3] - fl_a, W[:, tb, 2, 3] - fl_t) < 0.03
        m = np.zeros(len(W), bool)
        f = 0
        while f < len(W):
            if con[f]:
                e = f
                while e + 1 < len(W) and con[e + 1]:
                    e += 1
                if e - f + 1 >= 5:
                    m[f:e + 1] = True
                f = e + 1
            else:
                f += 1
        masks[sd] = m
    rep = shape_feet(br, W, None, 0.0, 0.58, pin_cm=pc, masks=masks)
    print('PINH pinned %s' % (rep['pinned'],))


def pinpre(a):
    """foot lock of one side over frames k0..k1 (game rig dump in, dump out; same keys). The ankle keeps its frame-k0 position, the knee its bending plane, the foot its world orientation."""
    import game_skeleton_bridge as CB
    scene, tpose, out, side, k0, k1 = a[0], a[1], a[2], a[3], int(a[4]), int(a[5])
    br, d = CB.load_cp_bridge(tpose, scene)
    W = d['W'].copy()
    fo = br.uidx[br.legs[side][2]]
    goal = W[k0, fo, :3, 3].copy()
    for t in range(k0 + 1, k1 + 1):
        ik_to(br, W[t], side, goal)
    keep = {k: d[k] for k in d.files if k != 'W'}
    np.savez(out, W=W, **keep)
    print('PINPRE %s %s frames %d..%d ankle (%.1f %.1f %.1f) cm' % (out, side, k0, k1, *(goal * 100)))


def sm_(x):
    return smooth01(x)


def plan(a):
    import game_skeleton_bridge as CB
    scene, tpose, out = a[0], a[1], a[2]
    a_all = a
    opt = lambda k, dflt: float(a[a.index(k) + 1]) if k in a else dflt
    k0, tail = int(opt('--k0', 21)), int(opt('--tail', 60))
    stance, lift, rlift = opt('--stance', 52) / 100.0, opt('--lift', 10) / 100.0, opt('--rlift', 8) / 100.0
    br, d = CB.load_cp_bridge(tpose, scene)
    W0 = d['W'].copy()
    names = [str(x) for x in d['names']]
    ix = {n: i for i, n in enumerate(names)}
    par = np.asarray(d['parents'])
    fo = {sd: br.uidx[br.legs[sd][2]] for sd in SIDES}
    th = {sd: br.uidx[br.legs[sd][0]] for sd in SIDES}
    chain = float(np.linalg.norm(W0[0, th['Left'], :3, 3] - W0[0, br.uidx[br.legs['Left'][1]], :3, 3]) + np.linalg.norm(W0[0, br.uidx[br.legs['Left'][1]], :3, 3] - W0[0, fo['Left'], :3, 3]))
    T = tail + 1
    A = {sd: W0[:, fo[sd], :3, 3].copy() for sd in SIDES}              # scene ankles (before the hips lag)
    # ---- hips lead (--lag < 0 = ahead of the vanilla path along the travel direction, > 0 = behind): the last-frame ankles (the feet are held there) are out of the 0.90 m leg's reach until frame ~54 when the
    # pelvis follows the vanilla path, so the pelvis gets ahead by up to |--lag| cm at --lag-peak (weight shift over the leading foot), 0 before --lag-from and on the last frame
    lag, lag_from, lag_pk = opt('--lag', 10) / 100.0, int(opt('--lag-from', 34)), int(opt('--lag-peak', 46))
    hp = W0[:, ix['Hips'], :2, 3]
    dirv = hp[tail] - hp[lag_from]
    dirv = dirv / np.linalg.norm(dirv)
    desc = []
    for j in range(len(names)):
        k = j
        while k >= 0 and k != ix['Hips']:
            k = int(par[k])
        if k == ix['Hips']:
            desc.append(j)
    for t in range(lag_from, T):
        sl = lag * (sm_(((t - lag_from) / float(lag_pk - lag_from))) if t <= lag_pk else 1.0 - sm_((t - lag_pk) / float(tail - lag_pk)))
        W0[t, desc, 0, 3] -= sl * dirv[0]
        W0[t, desc, 1, 3] -= sl * dirv[1]
    lim = opt('--lim', 0.985) * chain   # landing reach (the knee straightens quickly near full extension); not checked for the last 6 frames (the vanilla last frame has the legs 99.6 % extended)
    reach = lambda sd, t, p: float(np.linalg.norm(W0[t, th[sd], :3, 3] - p))
    ease = opt('--ease', 0.35)   # swing speed profile: (1 - ease) * smoothstep + ease * linear (flatter speed peak, the foot still starts and ends slowly)
    sm = lambda x: smooth01(np.asarray(x, dtype=float))
    rease = opt('--rease', ease)   # same for the right foot's short step
    sw = lambda u, e=ease: (1.0 - e) * smooth01(u) + e * float(np.clip(u, 0.0, 1.0))
    # ---- schedule
    zf = {sd: float(A[sd][tail, 2]) for sd in SIDES}                    # planted ankle height = the last (vanilla) frame
    Lf, Rf = A['Left'][tail].copy(), A['Right'][tail].copy()
    # opt-in (2026-11-05, rows whose vanilla foot dips below the final floor): --clampz 1 = no goal below the last frame's ankle height; --lland F = the left foot is locked from frame F (default k0): between k0 and F it follows
    # the scene's path hovering --lhover cm above the floor (no skid on the floor while it still moves fast); --rl-from F = first frame the right landing is searched from (default 30)
    clampz, lland, lhover, rl_from = int(opt('--clampz', 0)), int(opt('--lland', k0)), opt('--lhover', 0) / 100.0, int(opt('--rl-from', 30))
    keepchord = int(opt('--keepchord', 0))

    def chord_keep(t, p):
        # opt-in: the clamped left ankle keeps the scene's hip-ankle distance (the leg is as bent as in the vanilla frame, no extra knee lift): the foot moves out horizontally by what the raise took off the vertical part
        hp_ = W0[t, th['Left'], :3, 3]
        ch0 = float(np.linalg.norm(A['Left'][t] - hp_))
        dv = hp_[2] - p[2]
        dh = float(np.sqrt(max(ch0 * ch0 - dv * dv, 0.0)))
        v_ = A['Left'][t][:2] - hp_[:2]
        if np.linalg.norm(v_) > 1e-6 and dh > 0:
            p = p.copy()
            p[:2] = hp_[:2] + v_ / np.linalg.norm(v_) * dh
        return p
    P1 = A['Left'][lland].copy()
    if clampz:
        P1[2] = max(P1[2], zf['Left'])
        if keepchord and P1[2] > A['Left'][lland][2] + 1e-4:
            P1 = chord_keep(lland, P1)
    tLoff = lland
    while tLoff + 1 < tail and reach('Left', tLoff + 1, P1) <= 0.975 * chain:
        tLoff += 1
    tRl = next(t for t in range(rl_from, tail) if A['Right'][t, 2] - zf['Right'] < 0.01)   # vanilla landing of the right foot (over the sill)
    v = A['Right'][tRl, :2] - Lf[:2]
    R1 = np.r_[Lf[:2] + v / np.linalg.norm(v) * min(stance, float(np.linalg.norm(v))), zf['Right']]   # the landing site: as close to the vanilla one as the stance width allows
    tLon = next(t for t in range(tLoff + int(opt('--lswing', 10)), tail + 1) if all(reach('Left', u, Lf) <= lim for u in range(t, tail - 5)))
    tRoff = tLon + 1 - int(opt('--rover', 0))   # --rover: frames the right foot lifts before the left one has landed
    tRon = next(t for t in range(tRoff + int(opt('--rswing', 5)), tail + 1) if all(reach('Right', u, Rf) <= lim for u in range(t, tail - 5)))
    print('PLAN k0 %d: left locked %d..%d at (%.0f %.0f), right lands %d at (%.0f %.0f) [vanilla (%.0f %.0f)], left swing %d..%d to (%.0f %.0f), right swing %d..%d to (%.0f %.0f); lag %.0f cm' % (
        k0, lland, tLoff, *(P1[:2] * 100), tRl, *(R1[:2] * 100), *(A['Right'][tRl, :2] * 100), tLoff, tLon, *(Lf[:2] * 100), tRoff, tRon, *(Rf[:2] * 100), lag * 100))
    print('PLAN reach (m, chain %.3f): R1 planted %.3f max' % (chain, max(reach('Right', t, R1) for t in range(tRl, tRoff + 1))))
    goal = {sd: [None] * T for sd in SIDES}
    # left: lock, long swing, final spot
    for t in range(k0, T):
        if t < lland:
            p = A['Left'][t].copy()
            p[2] = max(p[2], zf['Left'] + lhover - opt('--fslack', 0) / 100.0) if clampz else p[2]   # --fslack cm: the approach may stay this far below the final floor (the floor gate allows 1.5 cm)
            if clampz and keepchord and p[2] > A['Left'][t][2] + 1e-4:
                p = chord_keep(t, p)
        elif t <= tLoff:
            p = P1.copy()
        elif t < tLon:
            u = (t - tLoff) / float(tLon - tLoff)
            p = P1 + (Lf - P1) * sw(u)
            p[2] = max(zf['Left'], P1[2] + (Lf[2] - P1[2]) * sw(u)) + lift * np.sin(np.pi * u) ** 1.0
        else:
            p = Lf.copy()
        goal['Left'][t] = p
    # right: the vanilla path over the sill, from t_a on shifted toward the landing site, planted at R1, short step to the final spot
    t_a = tRl - int(opt('--ta', 7))
    for t in range(t_a, T):
        if t < tRl:
            p = A['Right'][t].copy()
            p[:2] += (R1[:2] - A['Right'][tRl, :2]) * sm((t - t_a) / float(tRl - t_a))
            if clampz:
                p[2] = max(p[2], zf['Right'] + (opt('--rhover', 0) / 100.0 if t_a < t < tRl else 0.0))   # --rhover cm: the right foot does not skate on the floor before its landing (vanilla gliding foot, e.g. sport1)
        elif t <= tRoff:
            p = R1.copy()
        elif t < tRon:
            u = (t - tRoff) / float(tRon - tRoff)
            p = R1 + (Rf - R1) * sw(u, rease)
            p[2] = zf['Right'] + rlift * np.sin(np.pi * u)
        else:
            p = Rf.copy()
        goal['Right'][t] = p
    eb = int(opt('--endblend', 0))   # opt-in: over the last --endblend frames the landed feet hand over to the scene's own ankles (the vanilla last frames have the legs straight: a foot held exactly on its final spot while the hips are 1-2 cm off makes the knee jump on the last frame)
    if eb > 0:
        for sd in SIDES:
            fin_ = Lf if sd == 'Left' else Rf
            for t in range(tail - eb + 1, T):
                if goal[sd][t] is not None and np.linalg.norm(goal[sd][t] - fin_) < 1e-9:
                    w_ = float(smooth01((t - (tail - eb)) / float(eb)))
                    goal[sd][t] = fin_ + w_ * (A[sd][t] - fin_)
    rclear = opt('--rclear', 0) / 100.0   # opt-in: the right foot's way over the sill keeps >= --rclear cm (xy) from the left goal (the vanilla right foot passes the spot where the left foot is locked); smooth push away from it
    if rclear > 0:
        for _ in range(12):
            off = np.zeros((T, 2))
            for t in range(t_a, tRl):
                lp_ = (goal['Left'][t] if goal['Left'][t] is not None else A['Left'][t])[:2]
                v = goal['Right'][t][:2] - lp_
                dd = float(np.linalg.norm(v))
                if dd < rclear:
                    off[t] = (v / dd if dd > 1e-6 else np.array([1.0, 0.0])) * (rclear - dd)
            if not off.any():
                break
            nk = int(opt('--rclear-w', 5)) // 2   # half width of the triangular smoothing kernel (default 2 = 5 taps)
            ker = np.r_[np.arange(1, nk + 2), np.arange(nk, 0, -1)].astype(float)
            ker /= ker.sum()
            so = np.stack([np.convolve(off[:, k], ker, mode='same') for k in range(2)], 1)
            for t in range(t_a, tRl):
                tw = min(1.0, (t - t_a) / 2.0, (tRl - t) / 2.0)
                goal['Right'][t] = goal['Right'][t].copy()
                goal['Right'][t][:2] += so[t] * max(tw, 0.0) * (1.15 if nk == 2 else 1.0 + 0.075 * nk)
    c0 = k0 - 3
    W = W0[c0:T].copy()
    # opt-in (--pole BITS: 1 = left leg, 2 = right leg): knee direction = the pelvis-fixed anatomical 'forward' (the foot's rest-pose toe direction in the Hips frame) turned by --lrot / --rrot degrees about the hip->ankle axis, instead of the scene's knee
    # (the scene's knee plane of the nearly straight vanilla leg is ill-defined: it flips between frames as soon as the leg gets bent by a raised foot goal)
    use_pole = int(opt('--pole', 0))
    rot_deg = {'Left': opt('--lrot', 0), 'Right': opt('--rrot', 0)}
    pole_loc = {}
    if use_pole:
        rest = np.asarray(d['rest'])
        for sd in SIDES:
            if not (use_pole & (1 if sd == 'Left' else 2)):
                continue
            tb = ix[sd + 'ToeBase']
            v0 = rest[tb, :3, 3] - rest[fo[sd], :3, 3]
            pole_loc[sd] = rest[ix['Hips'], :3, :3].T @ v0
            pole_loc[sd] = pole_loc[sd] / np.linalg.norm(pole_loc[sd])

    rot2 = {}
    for sd_, k_ in (('Left', 'l'), ('Right', 'r')):
        if '--%srot2' % k_ in a:
            ab = [float(x) for x in a[a.index('--%srt' % k_) + 1].split(',')]
            rot2[sd_] = (float(a[a.index('--%srot2' % k_) + 1]), ab[0], ab[1])
    pf = {'Left': [float(x) for x in a[a.index('--lpf') + 1].split(',')] if '--lpf' in a else [0.0, 0.0], 'Right': [float(x) for x in a[a.index('--rpf') + 1].split(',')] if '--rpf' in a else [0.0, 0.0]}   # --lpf / --rpf A,B: weight of the anatomical knee direction against the scene's own (0 before frame A, smooth to 1 at frame B; default 0,0 = always)

    def pole_of(sd, Wt, gl, t):
        if sd not in pole_loc:
            return None
        v = Wt[ix['Hips'], :3, :3] @ pole_loc[sd]
        ax = gl - Wt[th[sd], :3, 3]
        ax = ax / np.linalg.norm(ax)
        rd = rot_deg[sd]
        if sd in rot2 and rot2[sd][2] > rot2[sd][1]:   # --lrot2 / --rrot2 D with --lrt / --rrt A,B: the turn goes from --lrot to D between frames A and B (the vanilla last frame's knee direction differs from the planted one)
            rd = rd + (rot2[sd][0] - rd) * float(smooth01((t - rot2[sd][1]) / float(rot2[sd][2] - rot2[sd][1])))
        a_ = np.radians(rd)
        v = v * np.cos(a_) + np.cross(ax, v) * np.sin(a_) + ax * np.dot(ax, v) * (1 - np.cos(a_))
        pa, pb = pf[sd]
        w = (1.0 if t >= pa else 0.0) if pb <= pa else float(smooth01((t - pa) / (pb - pa)))
        if w >= 1.0:
            return v
        kd = Wt[br.uidx[br.legs[sd][1]], :3, 3] - Wt[th[sd], :3, 3]
        kd = kd - ax * np.dot(kd, ax)
        if np.linalg.norm(kd) < 1e-6:
            return v
        kd = kd / np.linalg.norm(kd)
        so_d = opt('--lsrot' if sd == 'Left' else '--rsrot', 0)
        if sd == 'Left' and '--lsrot2' in a_all:   # --lsrot2 D with --lsrt A,B: the turn goes from --lsrot to D between frames A and B
            ab_ = [float(x) for x in a_all[a_all.index('--lsrt') + 1].split(',')]
            so_d = so_d + (opt('--lsrot2', 0) - so_d) * float(smooth01((t - ab_[0]) / float(ab_[1] - ab_[0])))
        so_ = np.radians(so_d)   # --lsrot / --rsrot D: the scene's own knee direction turned by D degrees about the hip->ankle axis (where the raised foot pushes the knee into the door / shell)
        if so_ != 0.0:
            kd = kd * np.cos(so_) + np.cross(ax, kd) * np.sin(so_)
        v = v - ax * np.dot(v, ax)
        v = v / np.linalg.norm(v)
        m = (1 - w) * kd + w * v
        return m if np.linalg.norm(m) > 1e-6 else v
    for sd in SIDES:
        for t in range(c0, T):
            gl = goal[sd][t] if goal[sd][t] is not None else A[sd][t]   # no goal: the ankle stays on the scene's path while the pelvis lags
            if sd in pole_loc or np.linalg.norm(gl - W[t - c0, fo[sd], :3, 3]) > 1e-4:   # --pole: also the frames without a goal (the knee direction blends in over the ramp, no jump at the first goal frame)
                ik_to(br, W[t - c0], sd, gl, pole_dir=pole_of(sd, W[t - c0], gl, t))
    if int(opt('--footlock', 0)):   # opt-in: the planted feet keep ONE world orientation per planted run (the scene's foot at the first planted frame; at the end the last frame's): the toe does not slide while the ankle is locked.
        # the orientation is handed over to / from the scene's foot over --flramp frames (slerp) before the take-off / after the release, no jump
        from scipy.spatial.transform import Rotation as Rot, Slerp
        K = int(opt('--flramp', 5))
        anchors = {'Left': [(lland, tLoff, lland), (tLon, tail, tail)], 'Right': [(tRl, tRoff, tRl), (tRon, tail, tail)]}
        for sd in SIDES:
            for t0_, t1_, an in anchors[sd]:
                Ra = W0[an, fo[sd], :3, :3].copy()
                for t in range(max(t0_ - K, c0), min(t1_ + K, tail) + 1):
                    if t < t0_ and an != tail:
                        continue    # the first plant starts on the scene's own orientation (anchor = its first frame)
                    if an == tail and t > t1_:
                        continue
                    w = 1.0 if t0_ <= t <= t1_ else float(smooth01(1.0 - (t0_ - t) / float(K + 1) if t < t0_ else 1.0 - (t - t1_) / float(K + 1)))
                    Wt_ = W[t - c0]
                    cur = Wt_[fo[sd], :3, :3].copy()
                    tgt = Slerp([0, 1], Rot.from_matrix(np.stack([cur, Ra])))([w]).as_matrix()[0] if w < 1.0 else Ra
                    br._rotate_subtree(Wt_, fo[sd], tgt @ cur.T, Wt_[fo[sd], :3, 3].copy())
    fcs = np.zeros((len(W), 4))
    lp = lambda t: (lland <= t <= tLoff) or t >= tLon
    rp = lambda t: (tRl <= t <= tRoff) or t >= tRon
    for t in range(c0, T):
        fcs[t - c0] = [lp(t), lp(t), rp(t), rp(t)]
    dz = float(d['dz']) if 'dz' in d.files else 0.0
    W[:, :, 2, 3] -= dz
    np.savez(out, W=W, c=c0, g=tail, dz=dz, foot_contacts=fcs, ramp_end=1, c_gen=(lag_from if lag != 0 else tail), g_gen=tail, names=np.array(names), parents=d['parents'], leg_ik=True, lift='plan', end_fix=False, floor=False,
             knee_smooth='', src='plan')
    print('PLAN %s frames %d..%d' % (out, c0, tail))


def assemble(a):
    import game_skeleton_bridge as CB
    scene, tpose, span, c, g, out = a[0], a[1], a[2], int(a[3]), int(a[4]), a[5]
    br, d = CB.load_cp_bridge(tpose, scene)
    s = np.load(span)
    frames = list(range(c, g + 1))
    assert len(s['G']) == len(frames), 'span has %d frames, c..g is %d' % (len(s['G']), len(frames))
    W = CB.cp_inv(br, d, s['G'].astype(np.float64), s['hips'].astype(np.float64), frames, leg_ik='--leg-ik' in a, blend=(c, g))
    fcs = np.asarray(s['foot_contacts']) if 'foot_contacts' in s.files else np.zeros((0, 4))
    tsides = tuple(a[a.index('--tail-sides') + 1].split(',')) if '--tail-sides' in a else SIDES   # (opt-in) the feet that are held until the last frame (default both)
    tail = int(a[a.index('--tail') + 1]) if '--tail' in a else None   # last frame of the clip: planted-at-g feet are held at their frame-tail position
    if '--end-fix' in a:  # leg IK lands the ankles a few cm off the original at c / g (different leg proportions): move them back, planted runs stay planted
        n = len(W)
        for sd in SIDES:
            fo = br.uidx[br.legs[sd][2]]
            P = W[:, fo, :3, 3].copy()
            mc = d['W'][c, fo, :3, 3] - P[0]
            pm = planted_mask(fcs, sd, n)
            mg = d['W'][tail if (tail and pm[-1] and sd in tsides) else g, fo, :3, 3] - P[-1]
            off = end_offsets(P, pm, mc, mg)
            for t in range(n):
                if np.linalg.norm(off[t]) > 1e-5:
                    ik_to(br, W[t], sd, P[t] + off[t])
    if '--narrow' in a or '--pin-glide' in a:   # opt-in (2026-11-06): gentle stride narrowing + glide pin of the planted runs, see shape_feet
        rep = shape_feet(br, W, fcs, float(a[a.index('--narrow') + 1]) if '--narrow' in a else 0.0, float(a[a.index('--natural') + 1]) / 100.0 if '--natural' in a else 0.58,
                         pin_cm=float(a[a.index('--pin-glide') + 1]) if '--pin-glide' in a else 0.0, ease=float(a[a.index('--shape-ease') + 1]) if '--shape-ease' in a else 1.0)
        print('SHAPE pinned %s shifted %s' % (rep['pinned'], rep['shifted']))
    if '--floor' in a:  # nothing below the floor: ankle / toe heights against the lowest ones of the vanilla tail (frames 40..), per side
        for sd in SIDES:
            fo, tb = br.uidx[br.legs[sd][2]], br.uidx[sd + 'ToeBase']
            fl_a, fl_t = d['W'][40:, fo, 2, 3].min(), d['W'][40:, tb, 2, 3].min()
            for t in range(len(W)):
                low = min(W[t, fo, 2, 3] - fl_a, W[t, tb, 2, 3] - fl_t)
                if low < -1e-4:
                    goal = W[t, fo, :3, 3].copy()
                    goal[2] -= low
                    ik_to(br, W[t], sd, goal)
    if '--lift' in a:  # swing lift: MotionBricks sometimes drags the foot at 1-2 cm; raise the ankle on an arch of --lift cm above the planted height (only where lower)
        lift = float(a[a.index('--lift') + 1]) / 100.0
        n = len(W)
        for sd in SIDES:
            fo = br.uidx[br.legs[sd][2]]
            pl = planted_mask(fcs, sd, n)
            P = W[:, fo, :3, 3].copy()
            rest_h = float(np.median(P[pl, 2])) if pl.any() else float(P[:, 2].min())
            f = 0
            while f < n:
                if not pl[f]:
                    e = f
                    while e + 1 < n and not pl[e + 1]:
                        e += 1
                    if e - f + 1 >= 3:
                        for t in range(f, e + 1):
                            a_, b_ = (f - 1 if f > 0 else f), (e + 1 if e < n - 1 else e)
                            u = (t - a_) / float(b_ - a_)
                            need = lift * np.sin(np.pi * u) - (P[t, 2] - rest_h)
                            if need > 1e-4:
                                goal = P[t].copy()
                                goal[2] += need
                                ik_to(br, W[t], sd, goal)
                    f = e + 1
                else:
                    f += 1
    if '--knee-smooth' in a:  # the knee bending plane of a nearly straight leg is ill-defined (cross product of two almost parallel bones): MotionBricks' twist flips it by up to 85 deg
        sig = float(a[a.index('--knee-smooth') + 1])   # between two frames; low-pass the knee direction (perpendicular to the hip-ankle chord) and re-solve the knee, ankle fixed
        n = len(W)
        k = np.arange(n)
        for sd in SIDES:
            th, ca, fo = (br.uidx[x] for x in br.legs[sd])
            pp = np.zeros((n, 3))
            for t in range(n):
                ch = W[t, fo, :3, 3] - W[t, th, :3, 3]
                ch /= np.linalg.norm(ch)
                v = W[t, ca, :3, 3] - W[t, th, :3, 3]
                v = v - ch * v.dot(ch)
                pp[t] = v / max(np.linalg.norm(v), 1e-9)
            for t in range(1, n):  # sign continuity
                if pp[t].dot(pp[t - 1]) < 0:
                    pp[t] = -pp[t]
            ps = np.zeros_like(pp)
            for t in range(n):
                wgt = np.exp(-0.5 * ((k - t) / sig) ** 2)
                ps[t] = (wgt[:, None] * pp).sum(0)
            ramp = np.array([min(1.0, min(t, n - 1 - t) / 3.0) for t in range(n)])
            for t in range(n):
                wv = ramp[t] * ramp[t] * (3 - 2 * ramp[t])
                target = pp[t] + wv * (ps[t] / max(np.linalg.norm(ps[t]), 1e-9) - pp[t])
                if np.linalg.norm(target - pp[t]) > 1e-4:
                    ik_to(br, W[t], sd, W[t, fo, :3, 3].copy(), pole_dir=target)
    if '--pin-h' in a:   # opt-in (2026-11-06), after lift / knee smoothing: glide pin on the planted runs by HEIGHT (the checker's contact rule: lower of ankle / toe within 3 cm of that joint's floor, runs >= 5 frames)
        pc = float(a[a.index('--pin-h') + 1])
        masks = {}
        for sd in SIDES:
            fo, tb = br.uidx[br.legs[sd][2]], br.uidx[sd + 'ToeBase']
            fl_a, fl_t = d['W'][40:, fo, 2, 3].min(), d['W'][40:, tb, 2, 3].min()
            con = np.minimum(W[:, fo, 2, 3] - fl_a, W[:, tb, 2, 3] - fl_t) < 0.03
            m = np.zeros(len(W), bool)
            f = 0
            while f < len(W):
                if con[f]:
                    e = f
                    while e + 1 < len(W) and con[e + 1]:
                        e += 1
                    if e - f + 1 >= 5:
                        m[f:e + 1] = True
                    f = e + 1
                else:
                    f += 1
            masks[sd] = m
        rep = shape_feet(br, W, fcs, 0.0, 0.58, pin_cm=pc, masks=masks)
        print('PINH pinned %s' % (rep['pinned'],))
    if '--hips-vanilla' in a:   # opt-in (2026-11-06) --hips-vanilla K1:K2: the whole body above the legs follows the SCENE hips (vanilla / head-duck) up to frame K1 and fades to the generated hips at K2 (rigid translation, ankles kept by leg IK):
        k1, k2 = (int(x) for x in a[a.index('--hips-vanilla') + 1].split(':'))   # keeps the early frames near the vanilla body (door / shell gates of the arms and hips)
        names_ = [str(x) for x in d['names']]
        hi = names_.index('Hips')
        for t in range(len(W)):
            f = c + t
            wv = 1.0 if f <= k1 else (1.0 - float(smooth01((f - k1) / float(k2 - k1))) if f < k2 else 0.0)
            if wv <= 1e-4:
                continue
            goals = {sd: W[t, br.uidx[br.legs[sd][2]], :3, 3].copy() for sd in SIDES}
            W[t, 2:, :3, 3] += wv * (d['W'][f, hi, :3, 3] - W[t, hi, :3, 3])
            for sd in SIDES:
                ik_to(br, W[t], sd, goals[sd])
    spanW = W
    if '--head' in a or tail:
        n0 = len(W)
        hc0, hk0 = (int(x) for x in a[a.index('--head') + 1].split(':')) if '--head' in a else (c, c)
        head = d['W'][hc0:c].copy()
        tailW = d['W'][g + 1:tail + 1].copy() if tail else np.zeros((0,) + W.shape[1:])
        W = np.concatenate([head, W, tailW])
        fc = np.asarray(fcs) if len(fcs) == n0 else np.zeros((n0, 4))
        hrows = np.zeros((len(head), 4))
        if '--head' in a:
            hrows[max(hk0 - hc0, 0):, 0:2] = 1.0   # left foot lock of pinpre
        trows = np.repeat(fc[-1:], len(tailW), axis=0) if len(tailW) else np.zeros((0, 4))
        fcs = np.concatenate([hrows, fc, trows])
        if '--floor-head' in a and len(head):   # opt-in (2026-11-06): the head frames (copied from the scene) are floor-fixed like --floor (a vanilla foot that sinks below the floor in the frames before c would fail the p_floor gate)
            for sd in SIDES:
                fo, tb = br.uidx[br.legs[sd][2]], br.uidx[sd + 'ToeBase']
                fl_a, fl_t = d['W'][40:, fo, 2, 3].min(), d['W'][40:, tb, 2, 3].min()
                for t in range(len(head)):
                    low = min(W[t, fo, 2, 3] - fl_a, W[t, tb, 2, 3] - fl_t)
                    if low < -1e-4:
                        goal = W[t, fo, :3, 3].copy()
                        goal[2] -= low
                        ik_to(br, W[t], sd, goal)
        if tail:
            for sd in SIDES:   # planted at g: constant ankle goal (the frame-tail position) over the extension, leg IK on the scene legs
                if not planted_mask(fc, sd, n0)[-1] or sd not in tsides:
                    continue
                fo = br.uidx[br.legs[sd][2]]
                goal = d['W'][tail, fo, :3, 3].copy()
                for t in range(len(head) + n0, len(W)):
                    ik_to(br, W[t], sd, goal)
        if '--pin-h-full' in a:   # opt-in: the same height-based glide pin over the WHOLE extended npz (head frames included)
            pin_by_height(br, d, W, float(a[a.index('--pin-h-full') + 1]))
        c, g = hc0, (tail if tail else g)
    dz = float(d['dz']) if 'dz' in d.files else 0.0
    W[:, :, 2, 3] -= dz
    names = [str(x) for x in d['names']]
    fcn = fcs
    extra = dict(ramp_end=1, c_gen=int(a[3]), g_gen=int(a[4])) if tail else {}
    np.savez(out, W=W, c=c, g=g, dz=dz, foot_contacts=fcn, **extra, names=np.array(names), parents=d['parents'], leg_ik=('--leg-ik' in a), lift=(a[a.index('--lift') + 1] if '--lift' in a else ''), end_fix=('--end-fix' in a), floor=('--floor' in a), knee_smooth=(a[a.index('--knee-smooth') + 1] if '--knee-smooth' in a else ''), src=os.path.basename(span))
    print('ASSEMBLE %s frames %d..%d leg_ik=%s%s' % (out, c, g, '--leg-ik' in a, ' head/tail extended (generated %s..%s)' % (a[3], a[4]) if ('--head' in a or tail) else ''))


def clip_dump(steps, base_dump, out):
    """the base dump (original coordinates) with frames c..g replaced by the steps (for metrics / render): Root and Trajectory stay."""
    st = np.load(steps, allow_pickle=True)
    d = np.load(base_dump, allow_pickle=True)
    W = d['W'].copy()
    c, g = int(st['c']), int(st['g'])
    W[c:g + 1, 2:] = st['W'][:, 2:]
    keep = {k: d[k] for k in d.files if k != 'W'}
    np.savez(out, W=W, **keep)
    print('CLIP_DUMP %s' % out)


if __name__ == '__main__':
    cmd = sys.argv[1]
    {'prep': prep, 'pinpre': pinpre, 'plan': plan, 'metrics': metrics, 'assemble': assemble, 'rank': rank, 'clipdump': lambda a: clip_dump(*a[:3])}[cmd](sys.argv[2:])
