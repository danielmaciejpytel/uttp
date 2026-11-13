"""Planted-foot glide of the car ENTRY clips (TPP get-in) and the MotionBricks fix of the gliding span (2026-11-06). Same toolchain as the exit
(exit_steps_gen.py / clip_inbetween.py / foot_pin.py are IMPORTED / called, not edited). Kimodo-env python for every subcommand here.

  python -I entry_steps_gen.py rows                              shipped rows (sources-final-R13j.txt) -> 'name<TAB>run dir'
  python -I entry_steps_gen.py measure-all <outdir>              dump every shipped roundtrip clip (<outdir>/<row>/cur.dump.npz) and print the glide table (16 processes)
  python -I entry_steps_gen.py measure <dump.npz> [<dump.npz> ..]  glide numbers of dumps (game rig, metres, root motion included)
  python -I entry_steps_gen.py prep <cur.dump.npz> <scene.dump.npz> [--f1 N]   ground shift for the generator (floor = lowest ankle of frames 0..f1, default whole clip)
  python -I entry_steps_gen.py plan <cur.dump.npz> [--v 1.0]       suggested span (c, N, g) covering the problem frames

Contact model (a game clip has no foot labels): per foot the foot is FLAT when its ankle AND toe joints are within H (3 cm) of their own lowest height inside a +-8 frame window;
a planted run is >= 3 consecutive flat frames. GLIDE = horizontal ankle speed (cm / frame) between two consecutive frames of the same run, the first step of a run (landing) left out.
A foot that carries weight should not move: <~0.3 cm/frame (9 cm/s) is retarget noise. SKATE = a swing (non-planted run >= 3 frames between two planted runs) that travels >= 25 cm with a lift
of < 6 cm over the higher end: a shuffle without a step (long swings that never leave the floor).
"""
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
SIDES = ('Left', 'Right')
HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (HERE, os.path.join(HERE, '..', 'anim', 'vehicle-entry')):   # python -I does not put the script folder on the path
    if _p not in sys.path:
        sys.path.insert(0, _p)
SRC_LIST = ROOT + r'\Build\work\vehicle-entry-car\r13\sources-final-R13j.txt'
H, WIN, MINRUN = 0.03, 8, 3


def rows():
    out = []
    for line in open(SRC_LIST):
        n, d = line.strip().split(None, 1)
        d = d.replace('/', '\\').replace('\\Build\\vehicle-entry-car', '\\Build\\work\\vehicle-entry-car')
        out.append((n, d))
    return out


def runs_of(m, minrun=1):
    out, f, n = [], 0, len(m)
    while f < n:
        if m[f]:
            e = f
            while e + 1 < n and m[e + 1]:
                e += 1
            if e - f + 1 >= minrun:
                out.append((f, e))
            f = e + 1
        else:
            f += 1
    return out


def foot_state(W, names, side, h=H, win=WIN):
    ix = {n: i for i, n in enumerate(names)}
    A, Tt = W[:, ix[side + 'Foot'], :3, 3], W[:, ix[side + 'ToeBase'], :3, 3]
    T = len(A)

    def lowmin(z):
        return np.array([z[max(0, f - win):f + win + 1].min() for f in range(T)])
    flat = ((A[:, 2] - lowmin(A[:, 2])) < h) & ((Tt[:, 2] - lowmin(Tt[:, 2])) < h)
    pl = np.zeros(T, bool)
    for a, b in runs_of(flat, MINRUN):
        pl[a:b + 1] = True
    return A, Tt, pl


def glide(W, names, side, h=H):
    """per-frame glide speed (cm / frame, 0 where not planted), planted mask, runs, skate swings [(f0, f1, travel_cm, lift_cm)]"""
    A, Tt, pl = foot_state(W, names, side, h)
    T = len(A)
    runs = runs_of(pl)
    sp = np.zeros(T)
    for a, b in runs:
        for f in range(a + 2, b + 1):
            sp[f] = np.linalg.norm((A[f] - A[f - 1])[:2]) * 100.0
    skates = []
    for (a0, b0), (a1, b1) in zip(runs[:-1], runs[1:]):
        f0, f1 = b0 + 1, a1 - 1
        if f1 - f0 + 1 < 3:
            continue
        travel = float(np.linalg.norm((A[a1] - A[b0])[:2])) * 100
        lift = float(A[f0:f1 + 1, 2].max() - max(A[b0, 2], A[a1, 2])) * 100
        if travel >= 25 and lift < 6:
            skates.append((f0, f1, round(travel), round(lift, 1)))
    return dict(sp=sp, pl=pl, runs=runs, skates=skates, A=A, T=Tt)


def report_plants(run_dir, row):
    """plant intervals of the source (NPC mirror) clip from the build report of the run: {'Left': [[a, b], ..], 'Right': ..}"""
    import json
    rep = json.load(open(os.path.join(run_dir, 'glb', row + '.build-report.json')))
    seg = rep['duck']['plant_segments']
    return {s: [list(x) for x in seg[s + 'Foot']['plants']] for s in SIDES}


def plant_glide(W, names, plants, ext=4, hz_toe=0.02, hz_ank=0.03):
    """glide of the PLANTED feet from the build report intervals: speed (cm / frame) of the ankle between consecutive frames of one interval; the interval is first EXTENDED by up to `ext`
    frames on both sides while the foot is still on the floor of that plant (toe within hz_toe, ankle within hz_ank of the interval's median height): the landing / take-off slide.
    Returns per side {sp, mask, ext_mask} (the speeds count only inside the extended mask, trivial one-frame intervals skipped)."""
    ix = {n: i for i, n in enumerate(names)}
    out = {}
    T = len(W)
    for s in SIDES:
        A, Tt = W[:, ix[s + 'Foot'], :3, 3], W[:, ix[s + 'ToeBase'], :3, 3]
        mask = np.zeros(T, bool)
        em = np.zeros(T, bool)
        for a, b in plants[s]:
            if b - a < 2:
                continue
            mask[a:b + 1] = True
            za, zt = np.median(A[a:b + 1, 2]), np.median(Tt[a:b + 1, 2])
            lo, hi = a, b
            while lo > 0 and a - lo < ext and abs(Tt[lo - 1, 2] - zt) < hz_toe and abs(A[lo - 1, 2] - za) < hz_ank:
                lo -= 1
            while hi < T - 1 and hi - b < ext and abs(Tt[hi + 1, 2] - zt) < hz_toe and abs(A[hi + 1, 2] - za) < hz_ank:
                hi += 1
            em[lo:hi + 1] = True
        sp = np.zeros(T)
        for f in range(1, T):
            if em[f] and em[f - 1]:
                sp[f] = np.linalg.norm((A[f] - A[f - 1])[:2]) * 100.0
        out[s] = dict(sp=sp, mask=mask, ext=em)
    return out


def after_plants(plants, st):
    """plant intervals of a spliced clip: the old intervals outside c..g (clipped), inside c..g the generator's own foot contact labels (planted runs of >= 2 frames)"""
    from exit_steps_gen import planted_mask
    c, g = int(st['c']), int(st['g'])
    n = g - c + 1
    fcs = np.asarray(st['foot_contacts'])
    out = {}
    for s_ in SIDES:
        keep = []
        for a, b in plants[s_]:
            if b < c or a > g:
                keep.append([a, b])
            else:
                if a < c:
                    keep.append([a, c - 1])
                if b > g:
                    keep.append([g + 1, b])
        m = planted_mask(fcs, s_, n) if len(fcs) == n else np.zeros(n, bool)
        keep += [[c + a, c + b] for a, b in runs_of(m, 2)]
        out[s_] = sorted(keep)
    return out


def plant_summary(W, names, plants, v=1.0):
    g = plant_glide(W, names, plants)
    res = {}
    for s in SIDES:
        sp, em = g[s]['sp'], g[s]['ext']
        cons = sp[[f for f in range(1, len(sp)) if em[f] and em[f - 1]]]
        res[s] = dict(max=float(cons.max()) if len(cons) else 0.0, mean=float(cons.mean()) if len(cons) else 0.0, n=int(em.sum()),
                      over=[int(f) for f in np.flatnonzero(sp > v)], sp=sp, ext=em)
    return res


def fmt_plant(n, T, r, v=1.0):
    return '%-30s T%3d | %s' % (n, T, ' | '.join('%s ext%2d max %5.2f mean %4.2f >%.1f %s' % (s[0], r[s]['n'], r[s]['max'], r[s]['mean'], v, r[s]['over']) for s in SIDES))


def summary(W, names, v=1.0):
    res = {}
    for s in SIDES:
        g = glide(W, names, s)
        sp, pl = g['sp'], g['pl']
        cons = sp[[f for f in range(len(sp)) if pl[f] and f >= 1 and pl[f - 1]]]
        res[s] = dict(max=float(cons.max()) if len(cons) else 0.0, mean=float(cons.mean()) if len(cons) else 0.0, n=int(pl.sum()),
                      over=[int(f) for f in np.flatnonzero(sp > v)], skates=g['skates'], runs=g['runs'])
    return res


def fmt(n, T, r, v=1.0):
    return '%-34s T%3d | %s' % (n, T, ' | '.join('%s pl%2d max %5.2f mean %4.2f >%.1f %s skate %s' % (s[0], r[s]['n'], r[s]['max'], r[s]['mean'], v, r[s]['over'], r[s]['skates']) for s in SIDES))


def _dump_one(args):
    n, d, outdir = args
    import game_skeleton_bridge as CB
    names, par, ref = CB.load_rig()
    rest = CB.rest_world(par, ref)
    os.makedirs(os.path.join(outdir, n), exist_ok=True)
    W, info = CB.world_from_glb(os.path.join(d, 'roundtrip', n + '.anims.glb'), 'enter')
    p = os.path.join(outdir, n, 'cur.dump.npz')
    CB.save_dump(p, W, names, par, rest, src='%s:%s' % (info['glb'], info['clip']), t0=info['t0'], t1=info['t1'])
    import json
    pl = report_plants(d, n)
    json.dump(pl, open(os.path.join(outdir, n, 'plants.json'), 'w'))
    return n, len(W), plant_summary(W, names, pl)


def cmd_measure_all(a):
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(16) as ex:
        res = list(ex.map(_dump_one, [(n, d, a[0]) for n, d in rows()]))
    for n, T, r in sorted(res):
        print(fmt_plant(n[9:], T, r))


def feet_motion(W, names):
    """per foot: max ankle speed (cm / frame), max speed change between frames (cm / frame^2), max ankle height (cm)"""
    ix = {n: i for i, n in enumerate(names)}
    out = {}
    for s in SIDES:
        A = W[:, ix[s + 'Foot'], :3, 3]
        v = np.linalg.norm(np.diff(A, axis=0), axis=1) * 100
        out[s] = (float(v.max()), float(np.abs(np.diff(v)).max()), float(A[:, 2].max() * 100))
    return out


def swings(W, names, plants, tmin=15.0, lmax=5.0):
    """swings between consecutive CORE plant intervals (report intervals of >= 3 frames): travel (cm, xy of the ankle between the last frame of one and the first of the next) and lift (cm, highest
    ankle height in between over the higher end). 'shuffle' = travel >= tmin and lift < lmax: a foot that crosses the floor without a step. Returns (all swings, shuffles)."""
    ix = {n: i for i, n in enumerate(names)}
    out, sh = {}, {}
    for s_ in SIDES:
        A = W[:, ix[s_ + 'Foot'], :3, 3]
        iv = sorted([x for x in plants[s_] if x[1] - x[0] >= 2])
        L = []
        for (a0, b0), (a1, b1) in zip(iv[:-1], iv[1:]):
            if a1 - b0 < 2:
                continue
            trav = float(np.linalg.norm((A[a1] - A[b0])[:2])) * 100
            lift = float(A[b0:a1 + 1, 2].max() - max(A[b0, 2], A[a1, 2])) * 100
            L.append((b0 + 1, a1 - 1, round(trav), round(lift, 1)))
        out[s_] = L
        sh[s_] = [x for x in L if x[2] >= tmin and x[3] < lmax]
    return out, sh


def cmd_measure(a):
    """measure <cur.dump.npz> [after_*.dump.npz ...]: glide on the report plant intervals (same intervals for before and after; the steps npz frames c..g are the changed span), plus
    continuity: max ankle speed / speed change per foot and the largest ankle shift against the first dump"""
    import json
    dumps = [x for x in a if x.endswith('.dump.npz')]
    d0 = np.load(dumps[0], allow_pickle=True)
    names = [str(x) for x in d0['names']]
    ix = {n: i for i, n in enumerate(names)}
    pl = json.load(open(os.path.join(os.path.dirname(dumps[0]), 'plants.json')))
    for p in dumps:
        d = np.load(p, allow_pickle=True)
        nm = os.path.basename(p)
        print(fmt_plant(nm, len(d['W']), plant_summary(d['W'], names, pl)))
        fm = feet_motion(d['W'], names)
        _, shf = swings(d['W'], names, pl)
        print('   shuffles (frames, travel cm, lift cm) %s' % shf)
        sh = ' '.join('%s %.1f' % (s[0], float(np.linalg.norm((d['W'][:, ix[s + 'Foot'], :3, 3] - d0['W'][:, ix[s + 'Foot'], :3, 3]), axis=1).max()) * 100) for s in SIDES)
        print('   feet %s | vmax L %.1f R %.1f | dv max L %.1f R %.1f | ankle shift vs first (cm) %s' % (nm, fm['Left'][0], fm['Right'][0], fm['Left'][1], fm['Right'][1], sh))
        sf = p.replace('after_', 'steps_').replace('.dump.npz', '.steps.npz')
        if nm.startswith('after_') and os.path.exists(sf) and 'pin' not in nm:
            print(fmt_plant(nm + ' own-labels', len(d['W']), plant_summary(d['W'], names, after_plants(pl, np.load(sf, allow_pickle=True)))))


def cmd_prep(a):
    d = np.load(a[0], allow_pickle=True)
    W = d['W'].copy()
    names = [str(x) for x in d['names']]
    ix = {n: i for i, n in enumerate(names)}
    f1 = int(a[a.index('--f1') + 1]) if '--f1' in a else len(W) - 1
    fa = [ix[s + 'Foot'] for s in SIDES]
    floor = float(W[:f1 + 1][:, fa, 2, 3].min())
    dz = float(min(d['rest'][i, 2, 3] for i in fa)) - floor
    W[:, :, 2, 3] += dz
    keep = {k: d[k] for k in d.files if k not in ('W', 'dz')}
    np.savez(a[1], W=W, dz=dz, **keep)
    print('PREP %s floor %.3f m dz %.3f m frames %d' % (a[1], floor, dz, len(W)))


def smooth01(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def cmd_pinonly(a):
    """pinonly <cur.dump.npz> <plants.json> <tpose.npz> <out.steps.npz> [--vthr 0.5] [--margin 4] [--c C --g G]
    GENTLE glide fix without a generator: every planted run (report interval + the landing / take-off frames where the foot is still on its floor) whose ankle glides > --vthr cm/frame
    is held at the position of its CORE plant (the report interval, static); the horizontal offset (xy only, the ankle height is the clip's) is smooth01-faded over the swing frames to the
    neighbouring runs, hips / spine / the swing path stay the clip's (two-bone leg IK, the foot keeps its world orientation). Runs that touch the last frame hold the LAST frame's ankle
    (the seated end pose is a gate). The steps npz covers c..g (default: first / last pinned frame -/+ margin, clamped to 1..T-2) and has the exit's format (exit_steps.splice)."""
    import json
    import game_skeleton_bridge as CB
    from exit_steps_gen import ik_to
    dump, pj, tpose, out = a[0], a[1], a[2], a[3]
    opt = lambda k, dflt: float(a[a.index(k) + 1]) if k in a else dflt
    vthr, margin, maxshift, lift = opt('--vthr', 0.5), int(opt('--margin', 4)), opt('--maxshift', 10.0) / 100.0, opt('--lift', 0.0) / 100.0
    br, d = CB.load_cp_bridge(tpose, dump)
    W = d['W']
    names = [str(x) for x in d['names']]
    ix = {n: i for i, n in enumerate(names)}
    T = len(W)
    plants = json.load(open(pj))
    g_ = plant_glide(W, names, plants)
    offs, rep, lo, hi = {}, [], T, 0
    for s in SIDES:
        A = W[:, ix[s + 'Foot'], :3, 3]
        ext, sp = g_[s]['ext'], g_[s]['sp']
        off = np.zeros((T, 3))
        keyed = np.zeros(T, bool)
        for a_, b_ in runs_of(ext):
            if sp[a_:b_ + 1].max() <= vthr:
                continue
            core = [(x, y) for x, y in plants[s] if y - x >= 2 and x <= b_ and y >= a_]
            cm = np.zeros(T, bool)
            for x, y in core:
                cm[max(x, a_):min(y, b_) + 1] = True
            if b_ == T - 1:
                goal = A[T - 1, :2].copy()
            elif cm.any():
                goal = np.median(A[cm][:, :2], axis=0)
            else:
                goal = A[a_:b_ + 1, :2].mean(0)
            while a_ < b_ and not cm[a_] and np.linalg.norm(goal - A[a_, :2]) > maxshift:   # --maxshift: the landing / take-off extension is trimmed where holding the foot would move it farther
                a_ += 1
            while b_ > a_ and b_ != T - 1 and not cm[b_] and np.linalg.norm(goal - A[b_, :2]) > maxshift:
                b_ -= 1
            gl = [t for t in range(a_, b_ + 1) if sp[t] > vthr]   # locality: only the frames around the gliding ones are held (the rest of a long run is static anyway)
            if not gl:
                continue
            a_, b_ = max(a_, min(gl) - 1), min(b_, max(gl) + 1)
            for t in range(a_, b_ + 1):
                off[t, :2] = goal - A[t, :2]
            keyed[a_:b_ + 1] = True
            lo, hi = min(lo, a_), max(hi, b_)
            rep.append((s, a_, b_, round(float(np.linalg.norm(off[a_:b_ + 1, :2], axis=1).max()) * 100, 1)))
        offs[s] = (off, keyed)
    liftsw = {sd: [] for sd in SIDES}
    if lift > 0:   # --lift CM: a SHORT swing (<= 12 frames) between two core plant intervals that travels >= 20 cm and rises less than CM above its higher end gets an arch (sin, z only): a fast shuffle becomes a step
        for sd in SIDES:
            A = W[:, ix[sd + 'Foot'], :3, 3]
            iv = sorted([x for x in plants[sd] if x[1] - x[0] >= 2])
            for (a0, b0), (a1, b1) in zip(iv[:-1], iv[1:]):
                f0, f1 = b0 + 1, a1 - 1
                if f1 - f0 + 1 < 3 or f1 - f0 + 1 > 12:
                    continue
                trav = float(np.linalg.norm((A[a1] - A[b0])[:2]))
                cur = float(A[f0:f1 + 1, 2].max() - max(A[b0, 2], A[a1, 2]))
                if trav >= 0.20 and cur < lift:
                    liftsw[sd].append((f0, f1, b0, a1, cur, trav))
                    lo, hi = min(lo, b0), max(hi, a1)
    if not rep and not any(liftsw.values()):
        print('PINONLY nothing to pin (no run glides > %.2f cm/frame)' % vthr)
        return
    c = int(a[a.index('--c') + 1]) if '--c' in a else max(0, lo - margin)
    g = int(a[a.index('--g') + 1]) if '--g' in a else min(T - 1, hi + margin)
    n = g - c + 1
    Wn = W[c:g + 1].copy()
    fcs = np.zeros((n, 4))
    F = margin   # --margin = fade window (frames): the offset leaves / joins a pinned run inside F frames (smooth01), 0 outside
    for s in SIDES:
        off, keyed = offs[s]
        A = W[:, ix[s + 'Foot'], :3, 3]
        kf = []
        for a_, b_ in runs_of(keyed):
            if b_ < c or a_ > g:
                continue
            kf += [(a_, off[a_]), (b_, off[b_])]
        full = off.copy()
        edges = [(c - F - 1, np.zeros(3))] + kf + [(g + F + 1, np.zeros(3))]
        for (fa, oa), (fb, ob) in zip(edges[:-1], edges[1:]):
            L = fb - fa
            for t in range(max(fa + 1, c), min(fb, g + 1)):
                if keyed[t]:
                    continue
                if L <= 2 * F:
                    full[t] = oa + (ob - oa) * smooth01((t - fa) / float(L))
                else:
                    full[t] = oa * (1.0 - smooth01((t - fa) / float(F))) + ob * smooth01((t - (fb - F)) / float(F))
        for f0, f1, b0, a1, cur, trav in liftsw[s]:
            for t in range(f0, f1 + 1):
                full[t, 2] += (lift - cur) * np.sin(np.pi * (t - b0) / float(a1 - b0))
            rep.append((s, 'lift', f0, f1, round(trav * 100), round(cur * 100, 1)))
        for t in range(c, g + 1):
            if np.linalg.norm(full[t]) > 1e-5:
                ik_to(br, Wn[t - c], s, A[t] + full[t])
        cols = (0, 1) if s == 'Left' else (2, 3)
        for t in range(c, g + 1):
            if g_[s]['ext'][t]:
                fcs[t - c, cols[0]] = fcs[t - c, cols[1]] = 1.0
    np.savez(out, W=Wn, c=c, g=g, dz=0.0, foot_contacts=fcs, names=np.array(names), parents=d['parents'], leg_ik=True, src='pinonly')
    print('PINONLY %s span %d..%d pinned %s' % (out, c, g, rep))


def main(a):
    cmd = a[0]
    if cmd == 'pinonly':
        cmd_pinonly(a[1:])
        return
    if cmd == 'rows':
        for n, d in rows():
            print('%s\t%s' % (n, d))
    elif cmd == 'rundir':   # rundir <row> -> run dir of the shipped builder / roundtrip clip
        print(dict(rows())[a[1]])
    elif cmd == 'measure-all':
        cmd_measure_all(a[1:])
    elif cmd == 'measure':
        cmd_measure(a[1:])
    elif cmd == 'prep':
        cmd_prep(a[1:])
    else:
        raise SystemExit(__doc__)


if __name__ == '__main__':
    main(sys.argv[1:])
