"""UTPP anti-deformation guard v2 - evaluation per state class, v1 vs v2, both bodies (offline, 2026-11-09).

  python -I guard_eval.py --rig woman|man [--part i --nparts n] [--noskin] [--only <key> ...]
        -> Build\\work\\anim-guard-v2-20261109\\eval_<rig>_p<i>.json (+ sheet\\<rig>\\*.npz worst frames vanilla / v1 / v2)
  python -I guard_eval.py --rig woman --summary
        -> eval_tables.txt + appended to report.txt (section 4): per-state verdict v1 vs v2 for both bodies

Selection: per (class, state) of classes A, B, E up to 5 visible clips (survey metrics, degenerate exports excluded): the v1 prototype
clip first, then the highest survey severity from distinct sets. Rules: report.txt section 2 (verdict() below).
"""
import glob
import json
import os
import sys
import time
from collections import defaultdict

ARGS = list(sys.argv)
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import guard_proto_revised as G2  # noqa: E402   (reads --rig through survey_metrics)
import survey_metrics as SM  # noqa: E402
import guard_proto as GP  # noqa: E402
import survey_common as SC  # noqa: E402

RIG = G2.RIG
OUT = G2.OUT
GP.SKIN_DIR = os.path.join(OUT, 'skin', RIG)          # own skin folder (the v1 helper writes there)
N_PER_STATE = 5
POP_GO = 10.0
HAND_GO = 2.0
SKIN_EPS = 0.01


def arg(name, default=None):
    return ARGS[ARGS.index(name) + 1] if name in ARGS else default


def select(met, proto):
    by = defaultdict(list)
    for k, m in met.items():
        if m['cls'] not in ('A', 'B', 'E') or m.get('degenerate_export') or m['mode'] not in ('full', 'melee', 'action'):
            continue
        by[(m['cls'], m['state'])].append(k)
    sel = {}
    for st, ks in sorted(by.items()):
        first = [k for k in ks if k in proto]
        rest = sorted([k for k in ks if k not in proto], key=lambda k: -(met[k].get('severity') or 0.0))
        out, sets = list(first), {met[k]['setkey'] for k in first}
        for k in rest:
            if len(out) >= N_PER_STATE:
                break
            if met[k]['setkey'] not in sets:
                out.append(k)
                sets.add(met[k]['setkey'])
        for k in rest:
            if len(out) >= N_PER_STATE:
                break
            if k not in out:
                out.append(k)
        sel['%s | %s' % st] = out
    return sel


def col(dg, T, name):
    return np.array([float(d.get(name, 0.0) or 0.0) for d in dg[:T]])


def eval_clip(k, m, ctx, skin=True):
    W0 = GP.load_shown(ctx, k, m)
    T = len(W0)
    gm = SM.glitch_mask(W0, ctx.par)
    legs = m['mode'] == 'full' and bool(m['legs'])
    t0 = time.time()
    W2, dg = G2.run_clip(W0, ctx, m['mode'], legs, ramp=False)
    W3, dg3 = G2.run_clip(W0, ctx, m['mode'], legs, ramp=True)
    t_v2 = (time.time() - t0) / (2 * T)
    _, W1, _ = GP.guard(W0, ctx, feet=m['cls'] in ('A', 'E') and bool(m['legs']) and m['mode'] == 'full')
    contact = np.array([bool(d.get('contact')) for d in dg[:T]])
    keep = ~gm if gm.any() and not gm.all() else np.ones(T, bool)
    _, e0 = ctx.measure(W0, legs=m['legs'])
    s0 = SM.summarize(e0, T, gm)
    lim0, _ = G2.arm_limits(W0, ctx)
    r = dict(cls=m['cls'], state=m['state'], mode=m['mode'], set=m['setkey'], frames=T, glitch=int(gm.sum()), fail_v=s0['fail_pct'],
             lim_v=round(float(lim0[keep].max()), 1), contact_frames=int(contact.sum()), ms_per_frame_py=round(1000 * t_v2, 1))
    r['v1'] = G2.clip_metrics(W0, W1, ctx, m, gm, contact, v1=True)
    r['v2'] = G2.clip_metrics(W0, W2, ctx, m, gm, contact, v1=False)
    tp = G2.pops(W0, W3, ctx, gm=gm)
    r['v2_trans'] = dict(entry=tp['entry'], exit=tp['exit'], pop_chain=tp['pop_chain'], pop_chain_at=tp['pop_chain_at'])
    st = {}
    for s in ('Left', 'Right'):
        a = s + 'Arm'
        b, br = col(dg, T, a + '.beta'), col(dg, T, a + '.beta_reach')
        st[s[0]] = dict(beta_max=round(float(b.max()), 2), beta_pct=round(100 * float((b > 0.02).mean()), 1),
                        beta_reach_pct=round(100 * float((br > 0.02).mean()), 1), kept_max_cm=round(float(col(dg, T, a + '.kept_cm').max()), 1),
                        delta_max=round(float(col(dg, T, a + '.delta_deg').max()), 1),
                        wrist_cap_pct=round(100 * float((col(dg, T, a + '.wrist_clamp') >= G2.P['HAND_ROT_TOL_DEG'] - 0.1).mean()), 1))
    gn = col(dg, T, 'gamma_need')
    st['gamma_need_max'] = round(float(gn.max()), 2)
    st['gamma_need_pct'] = round(100 * float((gn > 0.02).mean()), 1)
    r['state_v2'] = st
    if skin:
        top = list(m.get('top_frames') or [m['worst_frame']])[:2]
        hf = int(r['v2']['hand_at'])
        fr = GP.frames_around(top + ([hf] if r['v2']['hand_cm'] > HAND_GO else []), T)
        tag = k.replace('/', '__')
        sv, xv = GP.skin_strain(W0[fr], 'v__' + tag)
        s1, x1 = GP.skin_strain(W1[fr], 'g1__' + tag)
        s2, x2 = GP.skin_strain(W2[fr], 'g2__' + tag)
        regs = SM.AG.SKIN_REGIONS
        for name, x in (('skin_v', xv), ('skin_v1', x1), ('skin_v2', x2)):
            r[name] = round(float(x.max()), 3)
            r[name + '_at'] = regs[int(np.unravel_index(int(np.argmax(x)), x.shape)[1])]
        wf = fr[int(np.argmax(xv.max(1)))]
        os.makedirs(os.path.join(OUT, 'sheet', RIG), exist_ok=True)
        np.savez_compressed(os.path.join(OUT, 'sheet', RIG, tag + '.npz'), W0=W0[wf].astype(np.float32), W1=W1[wf].astype(np.float32),
                            W2=W2[wf].astype(np.float32), frame=wf, key=k)
        r['sheet_frame'] = int(wf)
    return r


def benefit(r, tag):
    """measurable benefit of the guard (report.txt 4c): gate fail% down >= 10 points, or the clip's skin excess removed, or the clip's
    arm excess (> 2 deg) brought within 2 deg or lowered by >= 2 deg. -> list of the criteria met."""
    c = r[tag]
    why = []
    if r['fail_v'] - c['fail'] >= 10:
        why.append('gate-%.0f' % (r['fail_v'] - c['fail']))
    if r.get('skin_v', 0.0) > 0 and r.get('skin_' + tag, 0.0) <= 0:
        why.append('skin')
    if r['lim_v'] > 2 and (c['lim_max'] <= 2 or c['lim_max'] <= r['lim_v'] - 2):
        why.append('lim%.0f->%.0f' % (r['lim_v'], c['lim_max']))
    return why


def verdict(r, tag):
    """-> (GO | HARMLESS | PARTIAL | NO-GO, reasons) - report.txt section 2 + 4c (GO requires a measurable benefit)."""
    c = r[tag]
    sk_v, sk = r.get('skin_v', 0.0), r.get('skin_' + tag, 0.0)
    why = []
    if c['hand_cm'] > HAND_GO:
        why.append('hand %.1fcm' % c['hand_cm'])
    if c['pop_chain'] > POP_GO:
        why.append('pop %.0f %s' % (c['pop_chain'], c['pop_chain_at']))
    if c['new_excess'] > 0:
        why.append('new arm excess %dfr %.0fdeg' % (c['new_excess'], c['new_excess_max']))
    if sk > max(sk_v, 0.0) + SKIN_EPS:
        why.append('skin worse %.2f>%.2f %s' % (sk, sk_v, r.get('skin_' + tag + '_at', '')))
    if c['fail'] > r['fail_v'] + 1.0:
        why.append('gate worse %.0f>%.0f%%' % (c['fail'], r['fail_v']))
    if tag == 'v2' and max(r['v2_trans']['entry'], r['v2_trans']['exit'], r['v2_trans']['pop_chain']) > POP_GO:
        why.append('ramp %.0f' % max(r['v2_trans']['entry'], r['v2_trans']['exit'], r['v2_trans']['pop_chain']))
    if why:
        return 'NO-GO', why
    if sk > 0:
        return 'PARTIAL', ['skin %.2f %s remains' % (sk, r.get('skin_' + tag + '_at', ''))]
    ben = benefit(r, tag)
    return ('GO', []) if ben else ('HARMLESS', ['no measurable benefit'])


def state_verdict(vs):
    """vs = [(verdict, reasons)] of the clips of one state."""
    if any(v == 'NO-GO' for v, _ in vs):
        return 'NO-GO'
    if any(v == 'PARTIAL' for v, _ in vs):
        return 'NO-GO(partial)'
    if any(v == 'GO' for v, _ in vs):
        return 'GO'
    return 'HARMLESS'


def run():
    met = json.load(open(os.path.join(SC.WORK, 'metrics_%s.json' % RIG), encoding='utf-8'))
    proto = json.load(open(os.path.join(SC.WORK, 'proto_%s.json' % RIG), encoding='utf-8'))
    sel = select(met, proto)
    keys = [(s, k) for s, ks in sel.items() for k in ks]
    only = [x for x in ARGS[ARGS.index('--only') + 1:] if not x.startswith('--')] if '--only' in ARGS else None
    if only:
        keys = [(s, k) for s, k in keys if k in only]
    part, nparts = int(arg('--part', 0)), int(arg('--nparts', 1))
    keys = sorted(keys, key=lambda sk: -met[sk[1]]['frames'])[part::nparts]
    ctx = SM.Ctx()
    out_p = os.path.join(OUT, 'eval_%s_p%d.json' % (RIG, part)) if not only else os.path.join(OUT, 'eval_%s_only.json' % RIG)
    res = json.load(open(out_p, encoding='utf-8')) if os.path.exists(out_p) else {}
    skin = '--noskin' not in ARGS
    for i, (s, k) in enumerate(keys):
        if k in res:
            continue
        t0 = time.time()
        try:
            r = eval_clip(k, met[k], ctx, skin=skin)
        except Exception as ex:  # noqa: BLE001  (one broken clip must not stop the batch; reported)
            r = dict(cls=met[k]['cls'], state=met[k]['state'], error='%s: %s' % (type(ex).__name__, ex))
        r['state_key'] = s
        res[k] = r
        SC.jdump(res, out_p)
        if 'error' in r:
            print('ERR %s %s' % (k, r['error']), flush=True)
            continue
        print('%3d/%d %-62s %s v1 %s | v2 %s | hand %.2f/%.2f pop %.0f/%.0f skin %.2f/%.2f/%.2f  %.0fs' % (
            i + 1, len(keys), k[:62], r['cls'], verdict(r, 'v1')[0], verdict(r, 'v2')[0], r['v1']['hand_cm'], r['v2']['hand_cm'],
            r['v1']['pop_chain'], r['v2']['pop_chain'], r.get('skin_v', 0), r.get('skin_v1', 0), r.get('skin_v2', 0), time.time() - t0), flush=True)
    print('WROTE', out_p)


def summary():
    lines = ['', '4. EVALUATION v1 vs v2 (guard_eval.py, %s)' % time.strftime('%Y-%m-%d %H:%M'), '-' * 60,
             'frozen P = ' + json.dumps(G2.P), '']
    allres = {}
    for rig in ('woman', 'man'):
        res = {}
        for f in sorted(glob.glob(os.path.join(OUT, 'eval_%s_p*.json' % rig))):
            res.update(json.load(open(f, encoding='utf-8')))
        allres[rig] = res
        by = defaultdict(list)
        for k, r in res.items():
            by[r.get('state_key', '%s | %s' % (r['cls'], r['state']))].append((k, r))
        n_err = sum(1 for r in res.values() if 'error' in r)
        lines.append('== %s: %d clips evaluated (%d errors)' % (rig.upper(), len(res), n_err))
        lines.append('%-46s %2s | %-15s | %-15s | %-11s %-11s %-17s %-11s %-9s %-9s | %s' % (
            'class | state', 'n', 'v1 verdict', 'v2 verdict', 'hand v1/v2', 'pop v1/v2', 'skin v/v1/v2', 'fail v/v2', 'beta% L/R',
            'gneed%', 'v2 reasons (clips)'))
        for s in sorted(by):
            rs = [(k, r) for k, r in by[s] if 'error' not in r]
            if not rs:
                lines.append('%-46s  0 | errors: %s' % (s, '; '.join(r['error'][:60] for _, r in by[s])))
                continue
            v1s, v2s = [verdict(r, 'v1') for _, r in rs], [verdict(r, 'v2') for _, r in rs]
            cnt = lambda vs: '%dG %dH %dP %dN' % tuple(sum(1 for v, _ in vs if v == x) for x in ('GO', 'HARMLESS', 'PARTIAL', 'NO-GO'))  # noqa: E731
            mx = lambda f: max(f(r) for _, r in rs)  # noqa: E731
            reasons = defaultdict(int)
            for v, why in v2s:
                for w in why:
                    reasons[w.split(' ')[0] + ' ' + w.split(' ')[1] if w.startswith(('hand', 'pop', 'new', 'skin', 'gate', 'ramp')) else w] += 1
            rtxt = ', '.join('%s x%d' % (w, n) for w, n in sorted(reasons.items(), key=lambda x: -x[1])[:3])
            lines.append('%-46s %2d | %-15s | %-15s | %4.1f/%-6.1f %3.0f/%-7.0f %5.2f/%5.2f/%-5.2f %3.0f/%-7.0f %3.0f/%-5.0f %-9.0f | %s' % (
                s[:46], len(rs), state_verdict(v1s)[:15], state_verdict(v2s)[:15], mx(lambda r: r['v1']['hand_cm']), mx(lambda r: r['v2']['hand_cm']),
                mx(lambda r: r['v1']['pop_chain']), mx(lambda r: r['v2']['pop_chain']), mx(lambda r: r.get('skin_v', 0)),
                mx(lambda r: r.get('skin_v1', 0)), mx(lambda r: r.get('skin_v2', 0)), mx(lambda r: r['fail_v']), mx(lambda r: r['v2']['fail']),
                mx(lambda r: r['state_v2']['L']['beta_pct']), mx(lambda r: r['state_v2']['R']['beta_pct']),
                mx(lambda r: r['state_v2']['gamma_need_pct']), rtxt))
            lines.append('%-46s    | v1 %s | v2 %s' % ('', cnt(v1s), cnt(v2s)))
        lines.append('')
    lines.append('per clip (rig, verdict v1 -> v2, hand v2 cm, pop_chain v2 + where, ramp entry/exit, skin v -> v2, beta max L/R, gamma_need max):')
    for rig, res in allres.items():
        for k, r in sorted(res.items(), key=lambda x: x[1].get('state_key', '')):
            if 'error' in r:
                lines.append('  %s %-64s ERROR %s' % (rig[0], k[:64], r['error'][:80]))
                continue
            v1, v2 = verdict(r, 'v1'), verdict(r, 'v2')
            lines.append('  %s %-64s %-8s -> %-8s hand %5.2f pop %4.0f %-14s ramp %3.0f/%-3.0f skin %5.2f -> %5.2f beta %.2f/%.2f gneed %.2f %s' % (
                rig[0], k[:64], v1[0], v2[0], r['v2']['hand_cm'], r['v2']['pop_chain'], r['v2']['pop_chain_at'], r['v2_trans']['entry'],
                r['v2_trans']['exit'], r.get('skin_v', 0), r.get('skin_v2', 0), r['state_v2']['L']['beta_max'], r['state_v2']['R']['beta_max'],
                r['state_v2']['gamma_need_max'], '; '.join(v2[1])[:90]))
    txt = '\n'.join(lines) + '\n'
    open(os.path.join(OUT, 'eval_tables.txt'), 'w', encoding='utf-8').write(txt)
    with open(os.path.join(OUT, 'report.txt'), 'a', encoding='utf-8') as fh:
        fh.write(txt)
    print(txt)


if __name__ == '__main__':
    if '--summary' in ARGS:
        summary()
    else:
        run()
