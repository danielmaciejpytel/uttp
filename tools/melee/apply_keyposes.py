"""Apply a key-pose correction file (keypose_schema.json) to a melee clip dump.

  python -I apply_keyposes.py <correction.json> [--out-dir DIR] [--plan] [--utppclip] [--no-gate] [--verbose]
  python -I apply_keyposes.py --selftest
Reads npz\\<set>\\<clip>.npz (dump layout, melee_common.py), writes to <out-dir> (default corrected\\<set>\\):
  <clip>.offset.npz  corrected clip on the SOURCE timeline: every key's correction is exact on its window (key frame +- window,
                     shrunk when keys are close), between two windows the correction (a local rotation delta per joint and a
                     local translation delta) is interpolated with smoothstep, before the first / after the last key it is held
                     (or faded with fade_in_frames / fade_out_frames).
  <clip>.timed.npz  the same clip re-timed so that key k sits at its time_frame (piecewise-linear time map between keys, 1:1 before the
                     first and after the last key; rotations slerped). This is the template + the 'offset' fallback for melee_keyposes.py
  <clip>.keys.json  keys with src / time frames, windows, output gaps, planned segment modes (mb: gap = 4N-7, N 6..16)
  <clip>.timed.utppclip.json (--utppclip)  the timed clip as a game clip (game_skeleton_bridge.to_utppclip, Trajectory = hips ground point)
Correction order per key frame: rot offsets (list order) -> hips offset -> keep (effectors back to their source world position)
-> ik targets. IK is two-bone (ik.two_bone from the vehicle-entry tools): the original bend side is kept, the end bone keeps its
world rotation. A key with no corrections reproduces the source exactly (checked by --selftest).
GATE (default; --no-gate disables): the timed clip must pass anatomy_gate.py; otherwise the corrections of the keys around the worst
violating frame are scaled down (x0.75 per step, 0 = source pose) until it does. Scales + margins go to <clip>.keys.json['gate'], the
scaled correction to <clip>.scaled.json (the authored file is never rewritten).
"""
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'anim', 'vehicle-entry'))
import melee_common as MC  # noqa: E402
import ik as IK  # noqa: E402
import hinge_ik as HIK  # noqa: E402

CHAINS = {'LeftHand': ('LeftArm', 'LeftForeArm', 'LeftHand'), 'RightHand': ('RightArm', 'RightForeArm', 'RightHand'),
          'LeftFoot': ('LeftUpLeg', 'LeftLeg', 'LeftFoot'), 'RightFoot': ('RightUpLeg', 'RightLeg', 'RightFoot')}
MB_MIN_N, MB_MAX_N = 6, 16
LEGACY_IK = False   # True = the old ik.two_bone (minimal swing; produced bent-back knees / deviated elbows)


def mb_gap(n):
    return 4 * n - 7


def plan_gap(gap):
    """gap = output frames between two keys -> (mode, N, generated gap) """
    if gap < mb_gap(MB_MIN_N) - 2 or gap > mb_gap(MB_MAX_N) + 2:
        return 'offset', None, None
    n = int(min(MB_MAX_N, max(MB_MIN_N, round((gap + 7) / 4.0))))
    return 'mb', n, mb_gap(n)


class Skel:
    """Minimal skeleton shim for ik.two_bone: .parent, .world(lt, lr)."""
    def __init__(self, parents):
        self.parent = [int(p) for p in parents]

    def world(self, lt, lr):
        W = MC.fk_world(self.parent, lt, lr)
        return W[..., :3, 3], W[..., :3, :3]


def validate(corr, d=None):
    errs = []
    for k in ('schema', 'set', 'clip', 'keys'):
        if k not in corr:
            errs.append('missing "%s"' % k)
    if errs:
        return errs
    if corr['schema'] != 'utpp-keypose-correction/1':
        errs.append('schema must be utpp-keypose-correction/1')
    allowed_key = {'id', 'src_frame', 'time_frame', 'window', 'note', 'rot', 'hips_offset_cm', 'keep', 'ik'}
    ids, last_t = set(), -1
    names = set(d['names152']) if d is not None else None
    T = d['W'].shape[0] if d is not None else None
    for i, k in enumerate(corr['keys']):
        p = 'keys[%d]' % i
        for x in k:
            if x not in allowed_key:
                errs.append('%s: unknown field %s' % (p, x))
        if 'id' not in k or k['id'] in ids:
            errs.append('%s: id missing or duplicated' % p)
        ids.add(k.get('id'))
        sf = k.get('src_frame')
        if not isinstance(sf, int) or sf < 0 or (T is not None and sf >= T):
            errs.append('%s: src_frame %r outside 0..%s' % (p, sf, None if T is None else T - 1))
        t = k.get('time_frame', sf)
        if not isinstance(t, int) or t <= last_t:
            errs.append('%s: time_frame %r must be an int strictly greater than the previous key (%d)' % (p, t, last_t))
        last_t = t if isinstance(t, int) else last_t
        for r in k.get('rot', []):
            if r.get('space', 'character') not in ('local', 'character') or len(r.get('euler_deg', [])) != 3:
                errs.append('%s: bad rot entry %r' % (p, r))
            if names is not None and r.get('bone') not in names:
                errs.append('%s: unknown bone %r' % (p, r.get('bone')))
        if 'hips_offset_cm' in k and len(k['hips_offset_cm']) != 3:
            errs.append('%s: hips_offset_cm needs 3 numbers' % p)
        for e in k.get('keep', []):
            if e not in CHAINS:
                errs.append('%s: keep %r not in %s' % (p, e, list(CHAINS)))
        for e in k.get('ik', []):
            if e.get('effector') not in CHAINS or len(e.get('target_cm', [])) != 3:
                errs.append('%s: bad ik entry %r' % (p, e))
            if e.get('frame', 'character_offset') not in ('character_offset', 'character_abs', 'world'):
                errs.append('%s: bad ik frame %r' % (p, e.get('frame')))
    for s in corr.get('segments', []):
        if s.get('from') not in ids or s.get('to') not in ids or s.get('mode') not in ('auto', 'mb', 'offset'):
            errs.append('segments: bad entry %r' % s)
    return errs


def windows(corr, T):
    """Effective half-width per key (default window, shrunk so windows of neighbouring keys do not touch)."""
    keys = corr['keys']
    base = corr.get('window', 3)
    out = []
    for i, k in enumerate(keys):
        w = k.get('window', base)
        for j in (i - 1, i + 1):
            if 0 <= j < len(keys):
                w = min(w, (abs(keys[j]['src_frame'] - k['src_frame']) - 1) // 2)
        out.append(max(0, int(w)))
    return out


def correct_frames(d, ix, sk, frames, key):
    """Corrected local lt, lr of `frames` for one key (arrays [F,152,3], [F,152,4])."""
    fr = np.asarray(frames)
    lt = d['lt'][fr].astype(float).copy()
    lr = d['lr'][fr].astype(float).copy()
    W0 = MC.fk_world(sk.parent, lt, lr)
    P0 = W0[:, :, :3, 3]
    H = MC.heading_frame(P0[:, ix['LeftUpLeg']], P0[:, ix['RightUpLeg']])      # [F,3,3]
    origin = np.concatenate([P0[:, ix['Hips'], :2], np.zeros((len(fr), 1))], 1)
    for r in key.get('rot', []):
        b = ix[r['bone']]
        Rc = MC.euler_xyz(r['euler_deg'])
        if r.get('space', 'character') == 'local':
            lr[:, b] = (R.from_quat(lr[:, b]) * R.from_matrix(Rc)).as_quat()
        else:
            W = MC.fk_world(sk.parent, lt, lr)
            p = sk.parent[b]
            Rp = W[:, p, :3, :3] if p >= 0 else np.broadcast_to(np.eye(3), (len(fr), 3, 3))
            Rw = W[:, b, :3, :3]
            Rw2 = (H @ Rc @ np.swapaxes(H, 1, 2)) @ Rw
            lr[:, b] = R.from_matrix(np.swapaxes(Rp, 1, 2) @ Rw2).as_quat()
    if 'hips_offset_cm' in key:
        off = np.asarray(key['hips_offset_cm'], float) / 100.0
        hb = ix['Hips']
        W = MC.fk_world(sk.parent, lt, lr)
        p = sk.parent[hb]
        Pp = W[:, p] if p >= 0 else np.broadcast_to(np.eye(4), (len(fr), 4, 4))
        newpos = W[:, hb, :3, 3] + np.einsum('fij,j->fi', H, off)
        lt[:, hb] = np.einsum('fij,fj->fi', np.swapaxes(Pp[:, :3, :3], 1, 2), newpos - Pp[:, :3, 3])
    goals = {}
    for e in key.get('keep', []):
        goals[e] = (P0[:, ix[e]], 1.0)
    for e in key.get('ik', []):
        eff = e['effector']
        fm = e.get('frame', 'character_offset')
        tc = np.asarray(e['target_cm'], float) / 100.0
        if fm == 'world':
            tgt = np.broadcast_to(tc, (len(fr), 3)).copy()
        elif fm == 'character_abs':
            tgt = origin + np.einsum('fij,j->fi', H, tc)
        else:
            cur = MC.fk_world(sk.parent, lt, lr)[:, ix[eff], :3, 3]
            tgt = cur + np.einsum('fij,j->fi', H, tc)
        goals[eff] = (tgt, float(e.get('weight', 1.0)))
    for eff, (tgt, w) in goals.items():
        a, b, c = (ix[n] for n in CHAINS[eff])
        if LEGACY_IK:
            IK.two_bone(sk, lt, lr, a, b, c, tgt, weight=np.full(len(fr), w))
        else:   # hinge-first solver: elbow / knee stay hinges, wrist keeps its local pose, planted feet keep their world rotation
            HIK.solve(lambda l_t, l_r: MC.fk_world(sk.parent, l_t, l_r), sk.parent, d['rest_lr'], lt, lr, a, b, c, tgt, weight=w,
                      end_mode='world' if eff.endswith('Foot') else 'local')
    return lt, lr


def _smooth(s):
    return s * s * (3 - 2 * s)


def build_offset_layer(d, corr):
    """-> (lt, lr, keyinfo) corrected clip on the source timeline."""
    ix = MC.idx(d)
    sk = Skel(d['parents152'])
    T = d['lt'].shape[0]
    keys = corr['keys']
    wins = windows(corr, T)
    deltas = []   # per key: dict frame -> (dq [152] Rotation, dt [152,3])
    for k, w in zip(keys, wins):
        f0, f1 = max(0, k['src_frame'] - w), min(T - 1, k['src_frame'] + w)
        frames = list(range(f0, f1 + 1))
        lt, lr = correct_frames(d, ix, sk, frames, k)
        dq = R.from_quat(d['lr'][frames].reshape(-1, 4).astype(float)).inv() * R.from_quat(lr.reshape(-1, 4))
        dt = lt - d['lt'][frames].astype(float)
        deltas.append(dict(frames=frames, dq=dq.as_quat().reshape(len(frames), -1, 4), dt=dt))
    out_lt = d['lt'].astype(float).copy()
    out_lr = d['lr'].astype(float).copy()
    N = out_lt.shape[1]
    ident = np.tile([0.0, 0.0, 0.0, 1.0], (N, 1))

    def apply_delta(t, dq, dt, weight=1.0):
        q = R.from_quat(dq)
        if weight != 1.0:
            q = R.from_rotvec(q.as_rotvec() * weight)
        out_lr[t] = (R.from_quat(d['lr'][t].astype(float)) * q).as_quat()
        out_lt[t] = d['lt'][t].astype(float) + dt * weight

    def edge(i, which):
        dl = deltas[i]
        j = 0 if which == 'start' else len(dl['frames']) - 1
        return dl['dq'][j], dl['dt'][j], dl['frames'][j]

    for i, dl in enumerate(deltas):
        for j, t in enumerate(dl['frames']):
            apply_delta(t, dl['dq'][j], dl['dt'][j])
    for i in range(len(keys) - 1):                       # between windows: smoothstep of the deltas
        qa, ta, fa = edge(i, 'end')
        qb, tb, fb = edge(i + 1, 'start')
        rv = (R.from_quat(qa).inv() * R.from_quat(qb)).as_rotvec()
        for t in range(fa + 1, fb):
            s = _smooth((t - fa) / float(fb - fa))
            q = (R.from_quat(qa) * R.from_rotvec(rv * s)).as_quat()
            apply_delta(t, q, ta + (tb - ta) * s)
    q0, t0, f0 = edge(0, 'start')
    fin = corr.get('fade_in_frames', 0)
    for t in range(0, f0):
        wgt = 1.0 if fin <= 0 else _smooth(max(0.0, 1.0 - (f0 - t) / float(fin)))
        apply_delta(t, q0, t0, wgt)
    q1, t1, f1 = edge(len(keys) - 1, 'end')
    fout = corr.get('fade_out_frames', 0)
    for t in range(f1 + 1, T):
        wgt = 1.0 if fout <= 0 else _smooth(max(0.0, 1.0 - (t - f1) / float(fout)))
        apply_delta(t, q1, t1, wgt)
    return out_lt, out_lr, wins


def make_dump(d, lt, lr, meta_update):
    """Dump dict with W / W_extra recomputed from lt / lr."""
    out = dict(d)
    out['lt'], out['lr'] = lt.astype(np.float32), lr.astype(np.float32)
    Wl = MC.fk_world(d['parents152'], lt, lr)
    ix = MC.idx(d)
    out['W'] = np.stack([Wl[:, ix[n]] for n in d['names']], 1)
    out['W_extra'] = np.stack([Wl[:, ix[str(n)]] for n in d['names_extra']], 1).astype(np.float32)
    m = dict(d['meta'])
    m.update(meta_update)
    out['meta'] = m
    return out


def save_dump(path, d):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    x = dict(d)
    x['meta'] = json.dumps(d['meta'])
    np.savez_compressed(path, **x)


def time_map(corr, T):
    """Piecewise-linear map output frame -> source frame; returns (N_out, f(o) array of source positions)."""
    keys = corr['keys']
    tf = [k.get('time_frame', k['src_frame']) for k in keys]
    sf = [k['src_frame'] for k in keys]
    lead = sf[0]
    n_out = tf[-1] + (T - 1 - sf[-1]) + 1
    o = np.arange(n_out, dtype=float)
    src = np.interp(o, tf, sf)
    src[o < tf[0]] = sf[0] - (tf[0] - o[o < tf[0]])
    src[o > tf[-1]] = sf[-1] + (o[o > tf[-1]] - tf[-1])
    return n_out, np.clip(src, 0, T - 1)


def retime(d, src):
    """Resample a dump's local transforms at fractional source frames (slerp / lerp)."""
    f0 = np.floor(src).astype(int)
    f1 = np.minimum(f0 + 1, d['lt'].shape[0] - 1)
    w = (src - f0)[:, None, None]
    lt = d['lt'][f0].astype(float) * (1 - w) + d['lt'][f1].astype(float) * w
    q0, q1 = d['lr'][f0].astype(float), d['lr'][f1].astype(float)
    rv = (R.from_quat(q0.reshape(-1, 4)).inv() * R.from_quat(q1.reshape(-1, 4))).as_rotvec().reshape(len(src), -1, 3)
    lr = (R.from_quat(q0.reshape(-1, 4)) * R.from_rotvec((rv * w).reshape(-1, 3))).as_quat().reshape(len(src), -1, 4)
    return lt, lr


def segment_plan(corr):
    keys = corr['keys']
    forced = {(s['from'], s['to']): s['mode'] for s in corr.get('segments', [])}
    plan = []
    for a, b in zip(keys[:-1], keys[1:]):
        gap = b.get('time_frame', b['src_frame']) - a.get('time_frame', a['src_frame'])
        mode, n, g = plan_gap(gap)
        want = forced.get((a['id'], b['id']), 'auto')
        if want == 'offset':
            mode, n, g = 'offset', None, None
        elif want == 'mb' and mode == 'offset':
            n = int(min(MB_MAX_N, max(MB_MIN_N, round((gap + 7) / 4.0))))
            mode, g = 'mb', mb_gap(n)
        plan.append(dict(**{'from': a['id'], 'to': b['id']}, gap=gap, mode=mode, N=n, generated_gap=g))
    return plan


def ik_groups(k):
    """Indices of the ik entries of a key grouped by identical target (both hands of a two-hand grip move TOGETHER, so the grip never opens)."""
    g = {}
    for i, e in enumerate(k.get('ik', [])):
        g.setdefault(json.dumps([e['target_cm'], e.get('frame', 'character_offset'), e.get('weight', 1.0)]), []).append(i)
    return list(g.values())


def components(corr):
    """Every scalable part of the corrections: (key index, 'rot', i) | (key index, 'hips', 0) | (key index, 'ik', i)."""
    out = []
    for ki, k in enumerate(corr['keys']):
        out += [(ki, 'rot', i) for i in range(len(k.get('rot', [])))]
        if 'hips_offset_cm' in k:
            out.append((ki, 'hips', 0))
        out += [(ki, 'ik', g[0]) for g in ik_groups(k)]
    return out


def scaled_corr(corr, cs):
    """Copy of the correction with component scales cs {component: s} (0 = source pose, 1 = authored): rotation vectors, hips offset, ik weight."""
    c = dict(corr)
    c['keys'] = []
    for ki, k in enumerate(corr['keys']):
        if not any(key[0] == ki and v < 1.0 for key, v in cs.items()):
            c['keys'].append(k)
            continue
        k = json.loads(json.dumps(k))
        for (kj, kind, i), sc in cs.items():
            if kj != ki or sc >= 1.0:
                continue
            if kind == 'rot':
                rv = R.from_euler('xyz', k['rot'][i]['euler_deg'], degrees=True).as_rotvec() * sc
                k['rot'][i]['euler_deg'] = [round(float(x), 4) for x in R.from_rotvec(rv).as_euler('xyz', degrees=True)]
            elif kind == 'hips':
                k['hips_offset_cm'] = [round(float(x) * sc, 4) for x in k['hips_offset_cm']]
            else:
                for m in next(g for g in ik_groups(corr['keys'][ki]) if g[0] == i):
                    k['ik'][m]['weight'] = float(k['ik'][m].get('weight', 1.0)) * sc
        c['keys'].append(k)
    return c


def key_scale_summary(corr, cs):
    out = {}
    for ki, k in enumerate(corr['keys']):
        d = {}
        for (kj, kind, i), sc in cs.items():
            if kj == ki and sc < 1.0:
                name = k['rot'][i]['bone'] if kind == 'rot' else 'hips' if kind == 'hips' else 'ik_' + '+'.join(k['ik'][m]['effector'] for m in next(g for g in ik_groups(k) if g[0] == i))
                d[name] = round(sc, 3)
        if d:
            out[k['id']] = d
    return out


def _bracket_keys(corr, frame):
    """Indices of the keys whose correction is active at an output frame (the two bracketing keys; it is their interpolation)."""
    tf = [k.get('time_frame', k['src_frame']) for k in corr['keys']]
    lo = max([i for i, t in enumerate(tf) if t <= frame], default=0)
    hi = min([i for i, t in enumerate(tf) if t >= frame], default=len(tf) - 1)
    return sorted({lo, hi})


def build(d, corr):
    T = d['W'].shape[0]
    lt, lr, wins = build_offset_layer(d, corr)
    meta = dict(corrections=os.path.basename(corr.get('_path', 'corr.json')), keys=[dict(id=k['id'], src_frame=k['src_frame'],
                time_frame=k.get('time_frame', k['src_frame']), window=w) for k, w in zip(corr['keys'], wins)])
    off = make_dump(d, lt, lr, dict(meta, timeline='source'))
    n_out, src = time_map(corr, T)
    lt2, lr2 = retime(dict(lt=lt, lr=lr), src)
    timed = make_dump(d, lt2, lr2, dict(meta, timeline='output', frames=n_out))
    return off, timed, meta, n_out


def gate_search(d, corr, max_iter=45, verbose=False, step=0.7, extra=None):
    """Greedy per-component scale-down: while the timed clip fails the anatomical gate, the component (a rotation offset, the hips offset or
    one ik target of a key active at the worst violating frame) whose x`step` reduction lowers the total excess most is reduced.
    -> (corr_scaled, component scales, off, timed, meta, n_out, gate_result, iterations)."""
    import anatomy_gate as AG
    rg = AG.rig_of(d)
    comps = components(corr)
    cs = {c: 1.0 for c in comps}
    keys = [dict(src_frame=k['src_frame'], time_frame=k.get('time_frame', k['src_frame'])) for k in corr['keys']]
    Wv_t = AG.vanilla_on_timeline(d, keys)

    def evaluate(scales):
        c = scaled_corr(corr, scales)
        off, timed, meta, n_out = build(d, c)
        res = AG.gate(timed['W'], Wv_t, np.arange(n_out, dtype=float), rg)
        if res['passed'] and extra is not None:      # optional second check on the timed clip, e.g. the skinned-mesh stretch of demo_build.py
            ok, fr, ex = extra(timed, n_out)
            if not ok:
                res = dict(res, passed=False, n_violations=1, margin_min=-float(ex), violations=[dict(feature='skin_stretch', frame=int(fr), value=float(ex),
                           allowed=[0, 0], vanilla=[0, 0], excess=float(ex) * 10)], frames_bad=[int(fr)])
        return c, off, timed, meta, n_out, res

    def badness(res):
        return sum(e['excess'] ** 2 for e in res['violations']) + 100.0 * res['n_violations']

    state = evaluate(cs)
    for it in range(max_iter + 1):
        c, off, timed, meta, n_out, res = state
        if verbose:
            print('  gate it%d %s -> %s (%d violations, margin %.1f)' % (it, key_scale_summary(corr, cs) or 'authored', 'PASS' if res['passed'] else 'FAIL',
                                                                      res['n_violations'], res['margin_min']))
        if res['passed']:
            return c, cs, off, timed, meta, n_out, res, it
        kk = set(_bracket_keys(corr, res['violations'][0]['frame']))
        cand = [x for x in comps if x[0] in kk and cs[x] > 0]
        if not cand:
            cand = [x for x in comps if cs[x] > 0]
        if not cand:
            return c, cs, off, timed, meta, n_out, res, it
        best = None
        for x in cand:
            t = dict(cs)
            st_ = step if it < 14 else step * 0.5 if it < 24 else 0.0     # accelerate: after many reductions the component is dropped
            t[x] = 0.0 if cs[x] * st_ < 0.05 else cs[x] * st_
            st = evaluate(t)
            b = badness(st[5])
            if best is None or b < best[0]:
                best = (b, t, st)
        cs, state = best[1], best[2]
    c, off, timed, meta, n_out, res = state
    return c, cs, off, timed, meta, n_out, res, max_iter


def run(corr_path, out_dir=None, want_utppclip=False, base=None, gate=True, verbose=False, extra=None):
    corr = json.load(open(corr_path, encoding='utf-8'))
    corr['_path'] = corr_path
    d = MC.load_dump(MC.clip_path(corr['set'], corr['clip'], base))
    errs = validate(corr, d)
    if errs:
        raise SystemExit('INVALID correction file:\n  ' + '\n  '.join(errs))
    T = d['W'].shape[0]
    out_dir = out_dir or os.path.join(MC.WORK, 'corrected', corr['set'])
    gres, scales, iters = None, {}, 0
    if gate:
        c2, scales, off, timed, meta, n_out, gres, iters = gate_search(d, corr, verbose=verbose, extra=extra)
        if any(x < 1.0 for x in scales.values()):
            MC.jdump({k: v for k, v in c2.items() if k != '_path'}, os.path.join(out_dir, corr['clip'] + '.scaled.json'))
    else:
        off, timed, meta, n_out = build(d, corr)
    p_off = os.path.join(out_dir, corr['clip'] + '.offset.npz')
    save_dump(p_off, off)
    p_t = os.path.join(out_dir, corr['clip'] + '.timed.npz')
    save_dump(p_t, timed)
    info = dict(set=corr['set'], clip=corr['clip'], source_frames=T, output_frames=n_out, keys=meta['keys'], segments=segment_plan(corr),
                offset=p_off, timed=p_t)
    if gres is not None:
        status = 'pass' if gres['passed'] and all(x >= 1.0 for x in scales.values()) else 'scaled' if gres['passed'] else 'fail'
        info['gate'] = dict(status=status, key_scales=key_scale_summary(corr, scales), iterations=iters,
                            margin_min=gres['margin_min'], tightest=gres['worst_margins'], n_violations=gres['n_violations'],
                            violations=gres['violations'][:12])
    MC.jdump(info, os.path.join(out_dir, corr['clip'] + '.keys.json'))
    if want_utppclip:
        os.environ['UTPP_RIG'] = d['meta']['rig']
        sys.path.insert(0, os.path.join(MC.ROOT, 'Source', 'tools', 'ai_motion'))
        import game_skeleton_bridge as CP
        names, par, ref = CP.load_rig()
        clip = CP.to_utppclip(timed['W'], names, par, ref, d['rest'], corr['clip'] + '_keyposes')
        json.dump(clip, open(os.path.join(out_dir, corr['clip'] + '.timed.utppclip.json'), 'w'), separators=(',', ':'))
    return info, off, timed


def selftest():
    import tempfile
    tmp = tempfile.mkdtemp()
    d = MC.load_dump(MC.clip_path('pwa_katana', 'combo_attack_01'))
    ix = MC.idx(d)
    corr = dict(schema='utpp-keypose-correction/1', set='pwa_katana', clip='combo_attack_01', window=2, keys=[
        dict(id='a', src_frame=4), dict(id='b', src_frame=14), dict(id='c', src_frame=25)])
    p = os.path.join(tmp, 'zero.json')
    json.dump(corr, open(p, 'w'))
    info, off, timed = run(p, tmp, gate=False)
    assert np.abs(off['W'] - d['W']).max() < 1e-4, 'zero correction must reproduce the source'
    assert info['output_frames'] == d['W'].shape[0]
    # rotation offset in character space on the key frames, applied on the whole window, interpolated between keys
    corr['keys'][1]['rot'] = [dict(bone='RightArm', space='character', euler_deg=[0, 0, 20])]
    json.dump(corr, open(p, 'w'))
    info, off, timed = run(p, tmp, gate=False)
    i = ix['RightArm']
    Rs = d['W'][:, list(d['names']).index('RightArm'), :3, :3]
    Ro = off['W'][:, list(d['names']).index('RightArm'), :3, :3]
    ang = MC.rot_delta_deg(Rs, Ro)
    assert abs(ang[14] - 20) < 0.05 and abs(ang[13] - 20) < 0.05 and abs(ang[15] - 20) < 0.05, ang[10:19]
    assert ang[4] < 0.05 and ang[25] < 0.05 and 0.5 < ang[9] < 19.5
    # the rotation is about the character up axis: rotating RightArm by +20 about z moves the elbow along the ground plane only
    pe_s = d['W'][14, list(d['names']).index('RightForeArm'), :3, 3]
    pe_o = off['W'][14, list(d['names']).index('RightForeArm'), :3, 3]
    assert abs(pe_o[2] - pe_s[2]) < 0.05 * np.linalg.norm(pe_s - d['W'][14, list(d['names']).index('RightArm'), :3, 3]), 'elbow height changed'
    # retiming: key b moved from frame 14 to 24 -> out frame 24 equals the corrected frame 14
    corr['keys'][1]['time_frame'] = 24
    corr['keys'][2]['time_frame'] = 35
    json.dump(corr, open(p, 'w'))
    info, off, timed = run(p, tmp, gate=False)
    assert info['output_frames'] == 35 + (30 - 1 - 25) + 1
    assert np.abs(timed['W'][24] - off['W'][14]).max() < 1e-4
    assert [s['mode'] for s in info['segments']] == ['mb', 'offset'] and info['segments'][0]['N'] == 7, info['segments']
    # hips down + keep feet: feet stay put, hips drop
    corr['keys'][1].pop('rot')
    corr['keys'][1]['hips_offset_cm'] = [0, 0, -5]
    corr['keys'][1]['keep'] = ['LeftFoot', 'RightFoot']
    corr['keys'][1]['time_frame'] = 14
    corr['keys'][2]['time_frame'] = 25
    json.dump(corr, open(p, 'w'))
    info, off, timed = run(p, tmp, gate=False)
    names = list(d['names'])
    dh = off['W'][14, names.index('Hips'), 2, 3] - d['W'][14, names.index('Hips'), 2, 3]
    df = np.abs(off['W'][14, names.index('LeftFoot'), :3, 3] - d['W'][14, names.index('LeftFoot'), :3, 3]).max()
    assert abs(dh + 0.05) < 1e-4 and df < 2e-3, (dh, df)
    # validation
    bad = dict(corr, keys=[dict(id='a', src_frame=40)])
    assert validate(bad, d), 'frame out of range must be reported'
    bad = dict(corr, keys=[dict(id='a', src_frame=3, rot=[dict(bone='Nope', euler_deg=[0, 0, 0])])])
    assert any('unknown bone' in e for e in validate(bad, d))
    assert plan_gap(17) == ('mb', 6, 17) and plan_gap(30)[1] == 9 and plan_gap(8)[0] == 'offset' and plan_gap(60)[0] == 'offset'
    print('apply_keyposes selftest OK')


if __name__ == '__main__':
    a = sys.argv[1:]
    if not a or a[0] in ('-h', '--help'):
        print(__doc__)
    elif a[0] == '--selftest':
        selftest()
    else:
        info, _, _ = run(a[0], a[a.index('--out-dir') + 1] if '--out-dir' in a else None, '--utppclip' in a, gate='--no-gate' not in a, verbose='--verbose' in a)
        if '--plan' in a:
            print(json.dumps(info['segments'], indent=1))
        if 'gate' in info:
            g = info['gate']
            print('GATE %s scales %s margin %.1f (%d iterations)' % (g['status'].upper(), g['key_scales'] or 'authored', g['margin_min'], g['iterations']))
        print('APPLY_KEYPOSES %s/%s: %d keys, %d -> %d frames; segments %s' % (
            info['set'], info['clip'], len(info['keys']), info['source_frames'], info['output_frames'],
            [(s['gap'], s['mode'], s['N']) for s in info['segments']]))
