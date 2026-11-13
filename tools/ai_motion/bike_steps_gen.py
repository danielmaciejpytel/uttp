"""Motorcycle clips (get-on / get-off): foot glide measurement, MotionBricks candidate screening and the build of the chosen candidates (offline, no game).

  python -I bike_steps_gen.py measure <name=dump.npz|name=glb:clip> ... [--json out.json] [--span a:b]
        glide of the feet that are in ground contact (exit_steps_gen.contact_metrics: pivot = lower of ankle / toe, height above that joint's lowest level of the clip < 3 cm):
        travel (sum of the horizontal pivot motion over consecutive contact frames, cm), glide = the part of it that moves faster than 1 cm / frame (0.3 m/s), glide frames, max cm / frame;
        stance = largest horizontal ankle distance of the clip (cm).
  python -I bike_steps_gen.py pick <run dir> <vanilla set glb> <clip> <out_glb_prefix> [--span a:b] [--hips-w 0.15]
        every <run dir>/bike_*.steps.npz is spliced into the vanilla clip (build_exit_duck.py --identity --steps), dumped and measured; the candidate with the lowest
        score = glide cm (both feet, inside --span; contact + vertical-speed rule of slide_list) + hips-w * max hips deviation (cm) + 0.5 * max(0, ground stance - vanilla ground stance - 5) + 1000 when the bike clearance proxy fails (clearance(): inward / downward motion against the vanilla path next to the bike, hands) wins; table to stdout, json to <run dir>/pick.json,
        the winner's GLB is copied to <out_glb_prefix>.
Set-ups: bike_steps_run.sh generates the candidates (seeds).
"""
import json
import os
import shutil
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, HERE)
import exit_steps_gen as ESG  # noqa: E402

PYK = sys.executable
BUILD_DUCK = os.path.join(ROOT, 'Source', 'tools', 'anim', 'vehicle-entry', 'build_exit_duck.py')
BRIDGE = os.path.join(HERE, 'game_skeleton_bridge.py')


def load(spec):
    """name=dump.npz or name=glb:clip -> (W, names)"""
    if ':' in spec and spec.lower().rsplit(':', 1)[0].endswith('.glb'):
        glb, clip = spec.rsplit(':', 1)
        out = os.path.join(os.environ.get('TEMP', '.'), 'bike_measure_%d.npz' % os.getpid())
        subprocess.run([PYK, '-I', BRIDGE, 'dump-glb', glb, clip, out], check=True, capture_output=True)
        with np.load(out, allow_pickle=True) as d:
            W, names = d['W'], [str(x) for x in d['names']]
        os.remove(out)
        return W, names
    with np.load(spec, allow_pickle=True) as d:
        return d['W'], [str(x) for x in d['names']]


def floors_of(W, names, robust=False):
    """ground reference of slide_list (pass the VANILLA / before clip's floors when judging an edited clip): (ankle, toe) height of each foot.
    robust=False (legacy, entry tools): the lowest ankle / toe height over the clip.
    robust=True (2026-11-07, exit checks): min(20 % quantile of the clip, median of the first 3 frames, median of the last 3 frames) per joint. The global minimum is fooled by a short
    dip of a planted foot (sportbike3 exit_l: the planted foot sinks 3-4 cm at frames 13-17, then the whole stance sits 4-6 cm above that dip, so no frame of it counted as contact and the
    61 cm slide read 0.0 cm); a dip of a few frames is below the 20 % quantile, the ends of an exit / entry clip are stance poses on the ground."""
    out = {}
    for s_ in ESG.SIDES:
        fl = []
        for j in (s_ + 'Foot', s_ + 'ToeBase'):
            z = W[:, names.index(j), 2, 3]
            if robust:
                fl.append(float(min(np.quantile(z, 0.2), np.median(z[:3]), np.median(z[-3:]))))
            else:
                fl.append(float(z.min()))
        out[s_] = (fl[0], fl[1])
    return out


ROBUST_H = 0.04   # m: contact band above the robust floor (the stance toe rides 0-3 cm above the end-stance level, the ankle up to 4.5 cm)


def slide_list(W, names, side, f0, f1, h=0.03, vz_max=0.005, floors=None, robust=False):
    """horizontal pivot speed (cm / frame) over consecutive contact frames of one foot: contact = the pivot (lower of ankle / toe) is below h above that joint's lowest level of the clip
    and does not move vertically faster than vz_max (m / frame): a foot that lifts off or lands is not counted (toe-off / touch-down are not glide).
    robust=True: floors = floors_of(robust=True) unless given, band ROBUST_H, and the motion of a frame is that of the CURRENT pivot joint between the two frames (a heel / toe pivot change inside
    a planted foot no longer breaks the chain: the old rule skipped such frames)."""
    ja, jt = names.index(side + 'Foot'), names.index(side + 'ToeBase')
    za, zt = W[:, ja, 2, 3], W[:, jt, 2, 3]
    if robust:
        h = max(h, ROBUST_H)
        floors = floors or floors_of(W, names, True)
    fa, ft = floors[side] if floors else (za.min(), zt.min())
    ha, ht = za - fa, zt - ft
    piv = np.where(ha <= ht, ja, jt)
    hp = np.minimum(ha, ht)
    con = hp < h
    if robust:
        sl = []
        for f in range(max(f0, 1), f1 + 1):
            if con[f] and con[f - 1]:
                a_, b_ = W[f - 1, piv[f], :3, 3], W[f, piv[f], :3, 3]
                if abs(b_[2] - a_[2]) < vz_max:
                    sl.append(np.linalg.norm((b_ - a_)[:2]) * 100)
        return np.array(sl)
    P = np.array([W[f, piv[f], :3, 3] for f in range(len(W))])
    sl = []
    for f in range(max(f0, 1), f1 + 1):
        if con[f] and con[f - 1] and piv[f] == piv[f - 1] and abs(P[f, 2] - P[f - 1, 2]) < vz_max:
            sl.append(np.linalg.norm((P[f] - P[f - 1])[:2]) * 100)
    return np.array(sl)


def glide(W, names, span=None, floors=None, robust=False):
    f0, f1 = span if span else (0, len(W) - 1)
    out = {}
    for s in ESG.SIDES:
        sl = slide_list(W, names, s, f0, f1, floors=floors, robust=robust)
        out[s] = dict(contact=len(sl), travel_cm=round(float(sl.sum()), 1), glide_cm=round(float(sl[sl > 1.0].sum()), 1),
                      glide_frames=int((sl > 1.0).sum()), max_cm_fr=round(float(sl.max()), 2) if len(sl) else 0.0)
    ix = {n: i for i, n in enumerate(names)}
    a = W[:, ix['LeftFoot'], :2, 3] - W[:, ix['RightFoot'], :2, 3]
    out['stance_cm'] = round(float(np.linalg.norm(a, axis=1)[f0:f1 + 1].max()) * 100, 1)
    out['glide_total_cm'] = round(out['Left']['glide_cm'] + out['Right']['glide_cm'], 1)
    return out


def contact_phases(W, names, side, floors=None, h=ROBUST_H, vz_max=0.005, gap=1):
    """planted-foot phases of one foot (robust floors): runs of contact frames (a gap of <= `gap` non-contact frames is bridged: toe-off flicker) -> list of dict(f0, f1, net_cm = horizontal
    distance between the pivot (lower of ankle / toe) at the first and the last frame of the phase, path_cm = sum of the horizontal pivot motion, max_cm_fr). Slide of a phase = net_cm (a foot that only
    pivots in place has net 0); the gate of the exit rebuilds is net_cm <= 3 and path_cm - net_cm small."""
    floors = floors or floors_of(W, names, True)
    ja, jt = names.index(side + 'Foot'), names.index(side + 'ToeBase')
    ha, ht = W[:, ja, 2, 3] - floors[side][0], W[:, jt, 2, 3] - floors[side][1]
    con = np.minimum(ha, ht) < h
    piv = np.where(ha <= ht, ja, jt)
    n = len(W)
    runs, f = [], 0
    while f < n:
        if con[f]:
            e = f
            while e + 1 < n and (con[e + 1] or (e + 1 + gap < n and con[e + 1:e + 2 + gap].any())):
                e += 1
            while not con[e]:
                e -= 1
            runs.append((f, e))
            f = e + 1
        else:
            f += 1
    out = []
    for a, b in runs:
        if b - a < 1:
            continue
        pj = int(np.bincount(piv[a:b + 1]).argmax())   # the joint that is the pivot for most of the phase (the same joint at both ends: heel / toe roll does not read as slide)
        P = W[a:b + 1, pj, :3, 3]
        st = np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1) * 100
        out.append(dict(f0=int(a), f1=int(b), joint=names[pj], net_cm=round(float(np.linalg.norm(P[-1, :2] - P[0, :2]) * 100), 1), path_cm=round(float(st.sum()), 1),
                        max_cm_fr=round(float(st.max()), 2)))
    return out


def hips_dev(W0, W1, names):
    i = names.index('Hips')
    return float(np.linalg.norm(W1[:, i, :3, 3] - W0[:, i, :3, 3], axis=1).max()) * 100


def parse_span(a):
    if '--span' in a:
        k = a.index('--span')
        s = tuple(int(x) for x in a[k + 1].split(':'))
        return s, a[:k] + a[k + 2:]
    return None, a


BIKE_BAND = 0.30     # m: |x| (seat frame, x = 0 is the bike centre line) inside which a joint is next to / over the bike body
INWARD_CM, DOWN_CM, HAND_CM = 5.0, 5.0, 6.0
GATE_JOINTS = ('Hips', 'LeftLeg', 'RightLeg', 'LeftFoot', 'RightFoot', 'LeftToeBase', 'RightToeBase')


def clearance(W0, W1, names):
    """no bike geometry offline: proxy against the vanilla path. For every joint of GATE_JOINTS and frame where the VANILLA joint is within BIKE_BAND of the centre line and more than 12 cm above the
    floor of the clip, the generated joint must not move toward the centre line (inward) or downward more than the gates (cm); hands: largest deviation from vanilla (cm)."""
    ix = {n: i for i, n in enumerate(names)}
    floor = float(min(W0[:, ix['LeftToeBase'], 2, 3].min(), W0[:, ix['RightToeBase'], 2, 3].min()))
    inward = down = 0.0
    for j in GATE_JOINTS:
        a, b = W0[:, ix[j], :3, 3], W1[:, ix[j], :3, 3]
        m = (np.abs(a[:, 0]) < BIKE_BAND) & (a[:, 2] > floor + 0.12)
        if not m.any():
            continue
        d = (b - a)[m]
        inw = -np.sign(a[m, 0]) * d[:, 0]
        inward = max(inward, float(inw.max()) * 100)
        down = max(down, float((-d[:, 2]).max()) * 100)
    hand = 0.0
    for h in ESG.SIDES:   # hands: deviation from vanilla while the vanilla hand is within the band (on the grips / over the tank); after the release they are free
        a, b = W0[:, ix[h + 'Hand'], :3, 3], W1[:, ix[h + 'Hand'], :3, 3]
        m = (np.abs(a[:, 0]) < BIKE_BAND)
        if m.any():
            hand = max(hand, float(np.linalg.norm((b - a)[m], axis=1).max()) * 100)
    return dict(inward_cm=round(inward, 1), down_cm=round(down, 1), hand_cm=round(hand, 1),
                ok=bool(inward <= INWARD_CM and down <= DOWN_CM and hand <= HAND_CM))


def dynamics(W0, W1, names, par, max_accel_cm=16.0, max_step_deg=15.0):
    """the dynamic gates of check_bike_exit.py (d, e) on the 73 rig bones of the candidate: joint acceleration (cm / frame^2) of hips / spine / knees / feet / toes <= max(16, vanilla's own) and
    the largest local rotation step of the leg bones <= max(15 deg, 1.1 x vanilla's own)"""
    import bike_enter_lift as BEL
    ix = {n: i for i, n in enumerate(names)}
    jn = ('Hips', 'Spine', 'Spine2', 'LeftLeg', 'RightLeg', 'LeftFoot', 'RightFoot', 'LeftToeBase', 'RightToeBase')
    acc = lambda W: max(float(np.linalg.norm(np.diff(W[:, ix[n], :3, 3], 2, axis=0), axis=1).max()) * 100 for n in jn)
    a0, a1 = acc(W0), acc(W1)
    s0, s1 = BEL.leg_step(W0, par, names, 0, len(W0) - 1), BEL.leg_step(W1, par, names, 0, len(W1) - 1)
    return dict(accel_cm=round(a1, 2), accel_vanilla_cm=round(a0, 2), step_deg=round(s1, 1), step_vanilla_deg=round(s0, 1),
                ok=bool(a1 <= max(max_accel_cm, a0) and s1 <= max(max_step_deg, 1.1 * s0)))


def ground_stance(W, names, W0):
    """largest horizontal ankle distance (cm) over frames where both toes are on the ground (within 4 cm of the VANILLA last frame, from frame 30)"""
    ix = {n: i for i, n in enumerate(names)}
    m = np.ones(len(W), bool)
    for s in ESG.SIDES:
        z = W[:, ix[s + 'ToeBase'], 2, 3]
        m &= z < W0[-1, ix[s + 'ToeBase'], 2, 3] + 0.04
    m[:30] = False
    d = np.linalg.norm(W[:, ix['LeftFoot'], :2, 3] - W[:, ix['RightFoot'], :2, 3], axis=1) * 100
    return round(float(d[m].max()), 1) if m.any() else 0.0


def cmd_plan(a):
    """plan <dump.npz>: support foot side, k0 (first frame the support toe is within 4 cm of its lowest level), crossing frame (the swing foot has crossed the centre line to the exit side) and a suggested c"""
    W, names = load(a[0])
    ix = {n: i for i, n in enumerate(names)}
    side = 'Left' if os.path.basename(a[0]).split('.')[0].endswith('_l') else 'Right'
    other = 'Right' if side == 'Left' else 'Left'
    sgn = -1.0 if side == 'Left' else 1.0     # game: left = -X
    tz = W[:, ix[side + 'ToeBase'], 2, 3]
    zmin = float(tz[10:45].min())
    k0 = int(np.flatnonzero(tz[5:] < zmin + 0.04)[0]) + 5
    xo = sgn * W[:, ix[other + 'Foot'], 0, 3]
    zo = W[:, ix[other + 'Foot'], 2, 3]
    pk = int(np.argmax(zo[10:50])) + 10
    cr = next((f for f in range(pk, len(W)) if xo[f] > 0.15), len(W) - 1)
    print('PLAN %s side %s k0 %d swing-peak %d crossing %d' % (os.path.basename(a[0]), side, k0, pk, cr))
    return side, k0, pk, cr


def cmd_sweep(a):
    """sweep <out dir> <jobs> <name=dump.npz> ... [--cs 26,30,34,36,38] [--seeds 10,11,12,13] [--targets 4]: bike_steps_run.sh for every clip x c x seed (N chosen so that g = c + 4N - 7 <= 57; MotionBricks needs N >= 6; targets = min(--targets, frames - g); the support foot lock starts at k0, c - 8 or c - 3 (three variants))"""
    from concurrent.futures import ThreadPoolExecutor
    cs, seeds, tg = [26, 30, 34, 36, 38], [10, 11, 12, 13], 4
    for k, v in (('--cs', 'cs'), ('--seeds', 'seeds'), ('--targets', 'tg')):
        if k in a:
            i = a.index(k)
            val = a[i + 1]
            a = a[:i] + a[i + 2:]
            if v == 'cs':
                cs = [int(x) for x in val.split(',')]
            elif v == 'seeds':
                seeds = [int(x) for x in val.split(',')]
            else:
                tg = int(val)
    out, jobs, specs = a[0], int(a[1]), a[2:]
    todo = []
    for spec in specs:
        name, dump = spec.split('=', 1)
        side, k0, pk, cr = cmd_plan([dump])
        d = os.path.join(out, name)
        for c in cs:
            n = (57 - c + 7) // 4
            if n < 6:
                continue
            ks = {k0, max(k0, c - 8), c - 3} if c >= k0 + 4 else {c - 3}   # k0 detected late (the support foot lands after c - 4): lock only the context frames
            g = c + 4 * n - 7
            ntg = min(tg, 59 - g)
            for k in sorted(ks):
                for sd in seeds:
                    tag = 'c%d_k%d_N%d_t%d_s%d' % (c, k, n, ntg, sd)
                    if os.path.exists(os.path.join(d, 'bike_%s.steps.npz' % tag)):
                        continue
                    todo.append(['bash', os.path.join(HERE, 'bike_steps_run.sh'), dump, os.path.relpath(d, ROOT).replace(os.sep, '/'), side, str(k), str(c), str(n), str(ntg), str(sd), '58'])
    print('SWEEP %d jobs' % len(todo))

    def run(cmd):
        r = subprocess.run(cmd, capture_output=True, text=True)
        return (r.stdout.strip().splitlines() or ['no output ' + r.stderr[-200:]])[-1]
    with ThreadPoolExecutor(jobs) as ex:
        for line in ex.map(run, todo):
            print(line[:170])


def cmd_refine(a):
    """refine <out dir> <jobs> <name=dump.npz> ... [--top 6] [--narrow K,natural,pin,ease]: re-run the best candidates of every clip (pick.json, bike clearance ok) with the gentle correction BIKE_NARROW
    (default 0.5,58,1.0,-1: planted runs that glide > 1 cm are pinned, an over-wide stride is narrowed by 0.5 x the excess over 58 cm); new candidates carry the suffix _nw, run pick again afterwards."""
    from concurrent.futures import ThreadPoolExecutor
    top, nar = 6, '0.5,58,1.0,-1'
    for k in ('--top', '--narrow'):
        if k in a:
            i = a.index(k)
            v = a[i + 1]
            a = a[:i] + a[i + 2:]
            if k == '--top':
                top = int(v)
            else:
                nar = v
    out, jobs, specs = a[0], int(a[1]), a[2:]
    todo = []
    for spec in specs:
        name, dump = spec.split('=', 1)
        side, k0, pk, cr = cmd_plan([dump])
        d = os.path.join(out, name)
        pj = json.load(open(os.path.join(d, 'pick.json')))
        good = [r for r in pj['candidates'] if r['clearance']['ok'] and not r['tag'].endswith('_nw')][:top]
        for r in good:
            m = {x[0]: int(x[1:]) for x in r['tag'][len('bike_'):].split('_')}
            todo.append(['bash', os.path.join(HERE, 'bike_steps_run.sh'), dump, os.path.relpath(d, ROOT).replace(os.sep, '/'), side, str(m['k']), str(m['c']), str(m['N']), str(m['t']), str(m['s']), '58'])
    print('REFINE %d jobs' % len(todo))
    env = dict(os.environ, BIKE_NARROW=nar)

    def run(cmd):
        r = subprocess.run(cmd, capture_output=True, text=True, env=env)
        return (r.stdout.strip().splitlines() or ['no output ' + r.stderr[-200:]])[-1]
    with ThreadPoolExecutor(jobs) as ex:
        for line in ex.map(run, todo):
            print(line[:170])


def cmd_measure(a):
    span, a = parse_span(a)
    js = None
    if '--json' in a:
        k = a.index('--json')
        js, a = a[k + 1], a[:k] + a[k + 2:]
    res = {}
    for spec in a:
        name, src = spec.split('=', 1)
        W, names = load(src)
        res[name] = glide(W, names, span)
        r = res[name]
        print('GLIDE %-26s L travel %6.1f glide %6.1f (%2d fr, max %.1f) | R travel %6.1f glide %6.1f (%2d fr, max %.1f) | total glide %6.1f cm stance %.0f cm' % (
            name, r['Left']['travel_cm'], r['Left']['glide_cm'], r['Left']['glide_frames'], r['Left']['max_cm_fr'], r['Right']['travel_cm'], r['Right']['glide_cm'],
            r['Right']['glide_frames'], r['Right']['max_cm_fr'], r['glide_total_cm'], r['stance_cm']))
    if js:
        json.dump(res, open(js, 'w'), indent=1)


def cmd_pick(a):
    span, a = parse_span(a)
    hw = 0.15
    if '--hips-w' in a:
        k = a.index('--hips-w')
        hw, a = float(a[k + 1]), a[:k] + a[k + 2:]
    run, glb, clip, outp = a[:4]
    W0, names = load('%s:%s' % (glb, clip))
    base = glide(W0, names, span)
    fl0 = floors_of(W0, names)
    base_gs = ground_stance(W0, names, W0)
    import game_skeleton_bridge as _CB
    par_ = _CB.load_rig()[1]
    print('VANILLA glide %.1f cm ground stance %.0f cm' % (base['glide_total_cm'], base_gs))
    rows = []
    cands = sorted(f for f in os.listdir(run) if f.startswith('bike_') and f.endswith('.steps.npz'))
    os.makedirs(os.path.join(run, 'glb'), exist_ok=True)
    for f in cands:
        tag = f[:-len('.steps.npz')]
        out = os.path.join(run, 'glb', tag + '.anims.glb')
        if not (os.path.exists(out) and os.path.getmtime(out) >= os.path.getmtime(os.path.join(run, f))):   # a candidate built from the same steps npz is reused
            r = subprocess.run([PYK, '-I', BUILD_DUCK, '--glb', glb, '--clip', clip, '--name', clip, '--out', out, '--identity', '--steps', os.path.join(run, f),
                                '--report', os.path.join(run, 'glb', tag + '.json')], capture_output=True, text=True)
            if r.returncode:
                print('BUILD FAILED', tag, r.stderr[-300:])
                continue
        W, _ = load('%s:%s' % (out, clip))
        g = glide(W, names, span, fl0)
        hd = hips_dev(W0, W, names)
        cl = clearance(W0, W, names)
        dyn = dynamics(W0, W, names, par_)
        gs = ground_stance(W, names, W0)
        score = g['glide_total_cm'] + hw * hd + 0.5 * max(0.0, gs - base_gs - 5.0) + (1000.0 if not (cl['ok'] and dyn['ok']) else 0.0)
        rows.append(dict(tag=tag, glb=out, glide=g, hips_dev_cm=round(hd, 1), clearance=cl, dynamics=dyn, ground_stance_cm=gs, score=round(score, 2)))
        print('CAND %-24s glide %6.1f cm (L %.1f R %.1f) gstance %.0f hipsdev %.0f cm clear in %.1f dn %.1f hand %.1f %s score %.1f' % (
            tag, g['glide_total_cm'], g['Left']['glide_cm'], g['Right']['glide_cm'], gs, hd, cl['inward_cm'], cl['down_cm'], cl['hand_cm'], ('ok' if (cl['ok'] and dyn['ok']) else 'FAIL') + ' acc %.1f/%.1f step %.1f/%.1f' % (dyn['accel_cm'], dyn['accel_vanilla_cm'], dyn['step_deg'], dyn['step_vanilla_deg']), score))
    rows.sort(key=lambda r: r['score'])
    json.dump(dict(vanilla=base, candidates=rows), open(os.path.join(run, 'pick.json'), 'w'), indent=1)
    if rows:
        shutil.copyfile(rows[0]['glb'], outp)
        print('PICK %s score %.1f -> %s' % (rows[0]['tag'], rows[0]['score'], outp))


if __name__ == '__main__':
    cmd = sys.argv[1]
    {'measure': cmd_measure, 'pick': cmd_pick, 'plan': cmd_plan, 'sweep': cmd_sweep, 'refine': cmd_refine}[cmd](sys.argv[2:])
