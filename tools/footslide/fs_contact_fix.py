"""Contact-curve fix for utppclip locomotion clips (game rig, 30 fps): removes contact (contact_l / contact_r >= 0.5) on frames where the foot
is in the air, i.e. where the runtime FootLocker (FootLocker.hpp: lock the toe at the contact rising edge > 0.5, release < 0.3, maxDrift
0.25 m) would hold a toe that is still travelling. Only the two contact curves change; bones, other curves, notifies are untouched.

  fix    python -I fs_contact_fix.py fix <woman|man> <src clip dir> <out dir> <patch.json> [--names REGEX] [--air 0.05] [--still 0.010]
                 [--min-run 3] [--min-gap 2] [--no-g58]
         out dir receives ONLY the changed clips (new files); patch.json = per clip old / new contact arrays, changed frames, provenance.
  apply  python -I fs_contact_fix.py apply <patch.json> <src clip dir> <out dir> [--changed-only]
         out dir (must not exist or be empty) = copy of every *.utppclip.json of src, the patched clips written as new files. Refuses
         (exit 2, nothing written) when a patched clip is missing, has another frame count or its contact arrays differ from the patch's
         'old' arrays (SHA-256 of the float list) - the patch only applies to the curves it was computed from.
  verify python -I fs_contact_fix.py verify <woman|man> <src clip dir> <fixed clip dir> <table.csv> [--names REGEX]
         for every clip of the family in BOTH dirs: identity check (everything but curves.contact_l/r and retarget.contact_fix identical,
         bones bit-identical), never-added contact, and a before / after table: height-rule false frames, residual flagged frames by class,
         missed contacts, and an independent FootLocker lock emulation (60 Hz linear interpolation, hysteresis 0.5 / 0.3, toe locked at the
         rising edge): per lock the max horizontal toe offset from the lock point and the offset at the release.

Rule of 'fix' per foot: contact is removed on a frame when the foot is in the air by the fs_audit / fs_contacts rule (heel AND toe joint
  > --air = 5 cm above their per-clip ground level, fs_lib.ground_levels) AND its toe moves (central-difference horizontal toe speed
  > --still = 1.0 cm per 30 fps frame = 30 cm/s). In-air frames with a still toe (ball-raised push-off, a foot hovering a few cm in the
  retarget, a vertical lift) keep their contact: the FootLocker lock is planar and such a frame does not move the locked toe. Edges move
  (early touchdown / late lift-off), a moving in-air run inside a contact run splits it (a step the curve missed), a run without a still
  frame disappears (a blip in the swing); a fragment of an edited run shorter than --min-run = 3 frames goes too (it would lock and
  release inside the 0.08 s FootLocker blend-in) and a split gap shorter than --min-gap = 2 frames widens by one frame towards the
  faster toe (FootLocker::SampleContact interpolates linearly: a 1-frame gap is below the 0.3 release for 0.2 frame only). Ramps: a new rising edge takes the old run's rising pair (e.g. 0.22 / 0.98), a new
  falling edge its falling pair; removed frames are 0 (< 0.3 release).
  source pack 5.8 restore (default on): a clip whose source pack meta says utpp_contacts.source = 'derived' (FootSpeed < 60 cm/s, binary,
  Source/tools/retarget/export_foot_contacts.py) and that has source pack 5.8 contact curves of the same frame count (Animations/sourcepack/SourcePack_export/meta)
  takes the source pack 5.8 curves when they have no more height-rule false frames, no more missed contacts and a smaller total lock offset;
  the edge rule is applied after that.
"""
import csv
import hashlib
import json
import os
import re
import shutil
import sys

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fs_lib as L  # noqa: E402

FAMILY = r'(_Turn_|_Pivot_|_Reface_|Idle_turn|Idle_Turn)'
G58_META = os.path.join(L.ROOT, 'Animations', 'sourcepack', 'SourcePack_export', 'meta')
ALS_META = os.path.join(L.ROOT, 'Animations', 'retarget', 'meta_sourcepack')
AIR_H = 0.05      # m, report rule of fs_audit.py / fs_contacts.py: heel AND toe joint > 5 cm above their ground level = foot in the air
ON, OFF = 0.5, 0.3  # FootLockConfig onThreshold / offThreshold
DRIFT = 0.25      # m, FootLockConfig maxDrift
SLIP = 0.05       # m, replay default slip start (FeedReplayMotion --slip)
MINRUN = 3        # frames (30 fps): shortest fragment an edited contact run may keep (FootLockConfig blend-in 0.08 s = 2.4 frames)
MINGAP = 2        # frames: shortest split gap (a 2-frame gap stays below the 0.3 release for 1.2 frames of the interpolated curve)
KEYS = ('contact_l', 'contact_r')
SIDES = (('Left', 'contact_l'), ('Right', 'contact_r'))
VERSION = 2


def arr_hash(a):
    return hashlib.sha256(json.dumps([float(x) for x in a]).encode()).hexdigest()


def file_sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest()


def opt(args, flag, default, cast=float):
    return cast(args[args.index(flag) + 1]) if flag in args else default


class Geo:
    """world heel / toe of both feet (one FK), heights above the per-clip ground levels."""

    def __init__(self, c):
        gp, gq, wp = c.world()
        gh, gt = L.ground_levels(c)
        self.N = c.N
        self.side = {}
        for side, key in SIDES:
            h = wp[:, c.bi[side + 'Heel']]
            t = wp[:, c.bi[side + 'ToeBase']]
            hr, tr = h[:, 2] - gh, t[:, 2] - gt
            d = np.linalg.norm(np.diff(t[:, :2], axis=0), axis=1)          # d[k] = |toe(k+1) - toe(k)|
            sp = np.empty(c.N)
            sp[0] = d[0]
            sp[-1] = d[-1]
            sp[1:-1] = 0.5 * (d[:-1] + d[1:])
            self.side[key] = dict(hr=hr, tr=tr, toe=t[:, :2].copy(), sp=sp, din=np.r_[0.0, d])


def false_frames(cv, g, air=AIR_H):
    return (cv >= ON) & (g['hr'] > air) & (g['tr'] > air)


def missed_frames(cv, g):
    """fs_audit.py rule: flag off but heel or toe within PLANT_H of the ground and the toe still (< 0.5 cm per frame)."""
    geo = (g['hr'] < L.PLANT_H) | (g['tr'] < L.PLANT_H)
    return (cv < ON) & geo & (g['din'] * 100 < 0.5)


def lock_emulation(cv, toe, sub=2):
    """60 Hz (sub = 2 per 30 fps frame) FootLocker-style lock: hysteresis on > 0.5 / off < 0.3, toe locked at the rising edge.
    -> list of (start_frame, end_frame, max_offset_m, release_offset_m or None if still locked at the clip end)."""
    N = len(cv)
    ts = np.arange((N - 1) * sub + 1) / float(sub)
    fr = np.arange(N)
    c = np.interp(ts, fr, cv)
    x = np.interp(ts, fr, toe[:, 0])
    y = np.interp(ts, fr, toe[:, 1])
    out, on, lx, ly, mx, s0 = [], False, 0.0, 0.0, 0.0, 0.0
    for i in range(len(ts)):
        if not on and c[i] > ON:
            on, lx, ly, mx, s0 = True, x[i], y[i], 0.0, ts[i]
        elif on and c[i] < OFF:
            out.append((s0, ts[i], mx, float(np.hypot(x[i] - lx, y[i] - ly))))
            on = False
            continue
        if on:
            mx = max(mx, float(np.hypot(x[i] - lx, y[i] - ly)))
    if on:
        out.append((s0, ts[-1], mx, None))
    return out


def lock_stats(locks):
    mxs = [l[2] for l in locks]
    rel = [l[3] for l in locks if l[3] is not None]
    return dict(locks=len(locks), max_cm=100 * max(mxs, default=0.0), sum_cm=100 * float(sum(mxs)),
                over_slip=sum(m > SLIP for m in mxs), over_drift=sum(m > DRIFT for m in mxs),
                rel_max_cm=100 * max(rel, default=0.0))


def _short_fragments(on, remove, minrun):
    """kept fragments (< minrun frames) of contact runs that lose frames; untouched runs keep their length."""
    short = np.zeros(len(on), dtype=bool)
    if minrun > 1:
        for (ob, oe) in L.segments(on):
            if not remove[ob:oe + 1].any():
                continue
            for (b, e) in L.segments(on & ~remove):
                if ob <= b and e <= oe and e - b + 1 < minrun:
                    short[b:e + 1] = True
    return short


def trim_side(cv, g, air, still, minrun=MINRUN, mingap=MINGAP):
    """returns (new curve, list of edits). Removes contact on frames that are in the air (heel AND toe joint > air above their ground
    level, the fs_audit rule) AND whose toe moves (central-difference horizontal toe speed > still per frame). A fragment of an edited
    contact run that would stay shorter than minrun frames goes too (a 1-2 frame lock would lock and release inside the FootLocker
    blend-in of 0.08 s = 2.4 frames: chatter); untouched runs keep their length. Kept runs get source-pack-style ramps: a new rising edge inside
    an old segment takes the old segment's rising pair (frame before, first frame), a new falling edge its falling pair; frames of an old
    ramp next to a removed edge go to 0."""
    N = len(cv)
    cv = np.asarray(cv, dtype=float)
    on = cv >= ON
    in_air = (g['hr'] > air) & (g['tr'] > air)
    remove = on & in_air & (g['sp'] > still)
    if not remove.any():
        return cv.copy(), []
    # a split gap (removed frames with kept frames on both sides) shorter than mingap frames widens by one frame towards the faster toe:
    # SampleContact interpolates linearly, a 1-frame gap at 0.22 is below the 0.3 release for 0.2 frame only and a 60 Hz sampler skips it
    widened = np.zeros(N, dtype=bool)
    for _ in range(8):
        rm = remove | widened | _short_fragments(on, remove | widened, minrun)
        grow = False
        for (ob, oe) in L.segments(on):
            for (b, e) in L.segments(rm):
                if ob < b and e < oe and e - b + 1 < mingap:
                    widened[b - 1 if g['sp'][b - 1] >= g['sp'][e + 1] else e + 1] = True
                    grow = True
        if not grow:
            break
    short = _short_fragments(on, remove | widened, minrun)
    remove = remove | widened | short
    keep = on & ~remove
    out = cv.copy()
    old = L.segments(on)
    for (ob, oe) in old:
        if remove[ob] and ob > 0 and not on[ob - 1] and not (ob >= 2 and on[ob - 2]):
            out[ob - 1] = 0.0
        if remove[oe] and oe < N - 1 and not on[oe + 1] and not (oe + 2 < N and on[oe + 2]):
            out[oe + 1] = 0.0
    out[remove] = 0.0
    edits = []
    for (ob, oe) in old:
        rise = (float(cv[ob - 1]), float(cv[ob])) if ob > 0 else (0.22, 0.98)
        fall = (float(cv[oe]), float(cv[oe + 1])) if oe < N - 1 else (0.98, 0.22)
        runs = [(b, e) for (b, e) in L.segments(keep) if ob <= b and e <= oe]
        rem = [int(i) for i in range(ob, oe + 1) if remove[i]]
        if not rem:
            continue
        for (b, e) in runs:
            if b > ob:
                out[b - 1] = min(out[b - 1], rise[0]) if out[b - 1] > 0 else rise[0]
                out[b] = rise[1]
            if e < oe:
                out[e + 1] = min(out[e + 1], fall[1]) if out[e + 1] > 0 else fall[1]
                out[e] = max(out[e], fall[0]) if (b > ob and e == b) else fall[0]
        edits.append(dict(seg=[int(ob), int(oe)], removed=rem, runs=[[int(b), int(e)] for (b, e) in runs],
                          short=[int(i) for i in range(ob, oe + 1) if short[i]],
                          widened=[int(i) for i in range(ob, oe + 1) if widened[i]]))
    # invariants: never adds contact; the on-runs of the result are exactly the kept runs
    assert not np.any((out >= ON) & ~on), 'contact added'
    assert L.segments(out >= ON) == L.segments(keep), 'ramp values changed the runs'
    assert not np.any((out >= OFF) & remove), 'removed frame above the release threshold'
    for (ob, oe) in old:
        rr = [(b, e) for (b, e) in L.segments(keep) if ob <= b and e <= oe]
        assert all(b1 - e0 - 1 >= mingap for (_, e0), (b1, _) in zip(rr, rr[1:])), 'split gap shorter than mingap'
        assert all(e - b + 1 >= minrun for (b, e) in rr) or not remove[ob:oe + 1].any(), 'short fragment kept'
    return out, edits


def load_meta_curves(name, N):
    """(source of the source pack contacts or None, source pack 5.8 curves (l, r) of the same length or None)."""
    src = None
    pa = os.path.join(ALS_META, name + '.json')
    if os.path.exists(pa):
        src = json.load(open(pa, encoding='utf-8')).get('utpp_contacts', {}).get('source')
    g58 = None
    pg = os.path.join(G58_META, name + '.json')
    if os.path.exists(pg):
        mc = json.load(open(pg, encoding='utf-8')).get('curves', {})
        if all(k in mc and len(mc[k]) == N for k in KEYS):
            g58 = tuple(np.asarray(mc[k], dtype=float) for k in KEYS)
    return src, g58


def clip_metrics(curves, geo):
    r = dict(false=0, missed=0)
    locks = []
    for key in KEYS:
        g = geo.side[key]
        r['false'] += int(false_frames(curves[key], g).sum())
        r['missed'] += int(missed_frames(curves[key], g).sum())
        locks += lock_emulation(curves[key], g['toe'])
    r.update(lock_stats(locks))
    return r


def cmd_fix(argv):
    g, src, out, patch_path = argv[:4]
    args = argv[4:]
    rx = re.compile(opt(args, '--names', FAMILY, str))
    air, still = opt(args, '--air', AIR_H), opt(args, '--still', 0.010)
    minrun, mingap = opt(args, '--min-run', MINRUN, int), opt(args, '--min-gap', MINGAP, int)
    use_g58 = '--no-g58' not in args
    os.makedirs(out, exist_ok=True)
    params = dict(air_m=air, still_m_per_frame=still, min_run_frames=minrun, min_gap_frames=mingap, g58_restore=use_g58, family=rx.pattern)
    note = dict(version=VERSION, tool='Source/tools/footslide/fs_contact_fix.py', params=params)
    patch = dict(format='utpp-contact-patch', version=VERSION, rig=g, src_dir=os.path.relpath(src, L.ROOT).replace('\\', '/'),
                 tool='Source/tools/footslide/fs_contact_fix.py', tool_sha256=file_sha(os.path.abspath(__file__)), params=params, clips=[])
    n_seen = 0
    for f in sorted(os.listdir(src)):
        if not f.endswith('.utppclip.json'):
            continue
        n = f[:-14]
        if not rx.search(n) or n.startswith('AO_'):
            continue
        c = L.Clip(os.path.join(src, f), g)
        if 'contact_l' not in c.c or 'contact_r' not in c.c:
            continue
        n_seen += 1
        geo = Geo(c)
        cur = {k: c.c[k].copy() for k in KEYS}
        base = cur
        prov = 'source'
        als_src, g58 = load_meta_curves(n, c.N)
        if use_g58 and als_src == 'derived' and g58 is not None:
            cand = dict(zip(KEYS, g58))
            m0, m1 = clip_metrics(cur, geo), clip_metrics(cand, geo)
            if m1['false'] <= m0['false'] and m1['missed'] <= m0['missed'] and m1['sum_cm'] < m0['sum_cm']:
                base, prov = cand, 'gasp58_restore'
            else:
                print('%s: source pack 5.8 restore rejected (false %d->%d missed %d->%d lock sum %.1f->%.1f cm)' % (
                    n, m0['false'], m1['false'], m0['missed'], m1['missed'], m0['sum_cm'], m1['sum_cm']))
        new, edits = {}, {}
        for key in KEYS:
            new[key], edits[key] = trim_side(base[key], geo.side[key], air, still, minrun, mingap)
        changed = any(np.abs(new[k] - cur[k]).max() > 0 for k in KEYS)
        if not changed:
            continue
        frames = {}
        for key in KEYS:
            idx = np.nonzero(np.abs(new[key] - cur[key]) > 0)[0]
            frames[key] = [[int(i), float(cur[key][i]), float(new[key][i])] for i in idx]
        m0, m1 = clip_metrics(cur, geo), clip_metrics(new, geo)
        patch['clips'].append(dict(name=n, N=c.N, provenance=prov, sourcepack_contact_source=als_src,
                                   old_sha256={k: arr_hash(cur[k]) for k in KEYS}, new_sha256={k: arr_hash(new[k]) for k in KEYS},
                                   old={k: [float(x) for x in cur[k]] for k in KEYS}, new={k: [float(x) for x in new[k]] for k in KEYS},
                                   changed_frames=frames, edits={k: edits[k] for k in KEYS}, before=m0, after=m1))
        d = json.load(open(os.path.join(src, f), encoding='utf-8'))
        for k in KEYS:
            d['curves'][k] = [float(x) for x in new[k]]
        d.setdefault('retarget', {})['contact_fix'] = dict(note, provenance=prov)
        tmp = os.path.join(out, f + '.tmp')
        with open(tmp, 'w') as fh:
            json.dump(d, fh, separators=(',', ':'))
        os.replace(tmp, os.path.join(out, f))
        print('%-44s %-15s false %3d->%3d missed %2d->%2d locks %2d->%2d max %5.1f->%5.1f cm  >5cm %d->%d  >25cm %d->%d  frames %s' % (
            n.replace('M_Neutral_', ''), prov, m0['false'], m1['false'], m0['missed'], m1['missed'], m0['locks'], m1['locks'],
            m0['max_cm'], m1['max_cm'], m0['over_slip'], m1['over_slip'], m0['over_drift'], m1['over_drift'],
            {k[-1].upper(): len(v) for k, v in frames.items()}))
    tmp = patch_path + '.tmp'
    with open(tmp, 'w') as fh:
        json.dump(patch, fh, indent=1)
    os.replace(tmp, patch_path)
    print('family clips', n_seen, 'changed', len(patch['clips']), '->', patch_path)


def cmd_apply(argv):
    patch_path, src, out = argv[:3]
    changed_only = '--changed-only' in argv[3:]
    patch = json.load(open(patch_path, encoding='utf-8'))
    if os.path.exists(out) and os.listdir(out):
        print('REFUSE: out dir %s exists and is not empty' % out)
        sys.exit(2)
    errs, todo = [], []
    for e in patch['clips']:
        p = os.path.join(src, e['name'] + '.utppclip.json')
        if not os.path.exists(p):
            errs.append('%s: missing in src' % e['name'])
            continue
        d = json.load(open(p, encoding='utf-8'))
        if len(d['frames']) != e['N']:
            errs.append('%s: frames %d != patch %d' % (e['name'], len(d['frames']), e['N']))
            continue
        for k in KEYS:
            if arr_hash(d['curves'].get(k, [])) != e['old_sha256'][k]:
                errs.append('%s: %s differs from the curve the patch was computed from' % (e['name'], k))
        todo.append((e, d))
    if errs:
        print('REFUSE (nothing written):\n  ' + '\n  '.join(errs))
        sys.exit(2)
    os.makedirs(out, exist_ok=True)
    n_copy = 0
    names = {e['name'] for e, _ in todo}
    if not changed_only:
        for f in sorted(os.listdir(src)):
            if f.endswith('.utppclip.json') and f[:-14] not in names:
                shutil.copyfile(os.path.join(src, f), os.path.join(out, f))
                n_copy += 1
    for e, d in todo:
        for k in KEYS:
            d['curves'][k] = e['new'][k]
            assert arr_hash(d['curves'][k]) == e['new_sha256'][k]
        d.setdefault('retarget', {})['contact_fix'] = dict(version=patch['version'], tool=patch['tool'], params=patch['params'],
                                                           provenance=e['provenance'], patch=os.path.basename(patch_path))
        tmp = os.path.join(out, e['name'] + '.utppclip.json.tmp')
        with open(tmp, 'w') as fh:
            json.dump(d, fh, separators=(',', ':'))
        os.replace(tmp, os.path.join(out, e['name'] + '.utppclip.json'))
    print('applied %d patched clips, copied %d unchanged -> %s' % (len(todo), n_copy, out))


def cmd_verify(argv):
    """table over every family clip that is changed OR has height-rule false frames before (unchanged ones: before == after)."""
    g, src, fx, table = argv[:4]
    rx = re.compile(opt(argv[4:], '--names', FAMILY, str))
    frames_out = opt(argv[4:], '--frames-json', None, str)
    rows, bad, fr_spec = [], 0, {}
    for f in sorted(os.listdir(src)):
        if not f.endswith('.utppclip.json') or not rx.search(f[:-14]) or f.startswith('AO_'):
            continue
        ps, pf = os.path.join(src, f), os.path.join(fx, f)
        ca = L.Clip(ps, g)
        if 'contact_l' not in ca.c or 'contact_r' not in ca.c:
            continue
        geo = Geo(ca)
        cur = {k: ca.c[k] for k in KEYS}
        status, prov = 'unchanged', ''
        if os.path.exists(pf):
            da, db = json.load(open(ps, encoding='utf-8')), json.load(open(pf, encoding='utf-8'))
            ok = all(da.get(k) == db.get(k) for k in set(da) | set(db) if k not in ('curves', 'retarget'))
            ok = ok and set(da['curves']) == set(db['curves']) and all(da['curves'][k] == db['curves'][k] for k in da['curves'] if k not in KEYS)
            ok = ok and ({k: v for k, v in da['retarget'].items() if k != 'contact_fix'} ==
                         {k: v for k, v in db['retarget'].items() if k != 'contact_fix'})
            prov = db['retarget'].get('contact_fix', {}).get('provenance', '?')
            cb = L.Clip(pf, g)
            ok = ok and np.array_equal(ca.B, cb.B)
            new = {k: cb.c[k] for k in KEYS}
            # never adds contact: against the source curve, or against the source pack 5.8 curve when that was restored
            ref = cur
            if prov == 'gasp58_restore':
                _, g58 = load_meta_curves(f[:-14], ca.N)
                ok = ok and g58 is not None
                ref = dict(zip(KEYS, g58)) if g58 is not None else cur
            ok = ok and not any(np.any((new[k] >= ON) & (ref[k] < ON)) for k in KEYS)
            status = 'ok' if ok else 'IDENTITY_FAIL'
            bad += not ok
        else:
            new = cur
        m0, m1 = clip_metrics(cur, geo), clip_metrics(new, geo)
        if status == 'unchanged' and m0['false'] == 0:
            continue
        # residual height-rule frames after the fix, by class: 'still' = toe within 3 cm of the lock point (first frame) of its contact run
        still_n = moving_n = 0
        det, spec = [], {}
        for key in KEYS:
            gk = geo.side[key]
            fl = false_frames(new[key], gk)
            res = []
            for (b, e) in L.segments(new[key] >= ON):
                for i in range(b, e + 1):
                    if fl[i]:
                        res.append(i)
                        if float(np.linalg.norm(gk['toe'][i] - gk['toe'][b])) <= 0.03:
                            still_n += 1
                        else:
                            moving_n += 1
            idx = np.nonzero(np.abs(new[key] - cur[key]) > 0)[0]
            rem = [int(i) for i in idx if cur[key][i] >= ON and new[key][i] < ON]
            if len(idx):
                det.append('%s:%s' % (key[-1].upper(), ' '.join(str(int(i)) for i in idx)))
            spec[key] = dict(removed=rem, residual=res, before=[int(i) for i in np.nonzero(false_frames(cur[key], gk))[0]])
        fr_spec[f[:-14]] = spec
        rows.append([f[:-14], status, prov, m0['false'], m1['false'], still_n, moving_n, m0['missed'], m1['missed'],
                     m0['locks'], m1['locks'], '%.1f' % m0['max_cm'], '%.1f' % m1['max_cm'], '%.1f' % m0['sum_cm'], '%.1f' % m1['sum_cm'],
                     m0['over_slip'], m1['over_slip'], m0['over_drift'], m1['over_drift'], '%.1f' % m0['rel_max_cm'], '%.1f' % m1['rel_max_cm'],
                     ' | '.join(det)])
    hdr = ['clip', 'status', 'provenance', 'false_before', 'false_after', 'residual_still', 'residual_moving', 'missed_before', 'missed_after',
           'locks_before', 'locks_after', 'lock_max_cm_before', 'lock_max_cm_after', 'lock_sum_cm_before', 'lock_sum_cm_after',
           'locks_over5cm_before', 'locks_over5cm_after', 'locks_over25cm_before', 'locks_over25cm_after', 'release_max_cm_before',
           'release_max_cm_after', 'changed_frames']
    with open(table, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(hdr)
        w.writerows(rows)
    if frames_out:
        with open(frames_out, 'w') as fh:
            json.dump(dict(clipdir=os.path.abspath(src), clips=fr_spec), fh, indent=1)
    tot = lambda i: sum(int(r[i]) for r in rows)  # noqa: E731
    chg = [r for r in rows if r[1] != 'unchanged']
    print('family clips in table %d (changed %d, unchanged with false frames %d), identity violations %d' % (
        len(rows), len(chg), len(rows) - len(chg), bad))
    print('clips with height-rule false frames %d -> %d; frames %d -> %d (residual: toe within 3 cm of its lock point %d, moving %d)' % (
        sum(r[3] > 0 for r in rows), sum(r[4] > 0 for r in rows), tot(3), tot(4), tot(5), tot(6)))
    print('missed %d -> %d; emulated locks > 5 cm %d -> %d, > 25 cm (drift unlock) %d -> %d; lock offset sum %.1f -> %.1f cm; max %.1f -> %.1f cm' % (
        tot(7), tot(8), tot(15), tot(16), tot(17), tot(18), sum(float(r[13]) for r in rows), sum(float(r[14]) for r in rows),
        max(float(r[11]) for r in rows), max(float(r[12]) for r in rows)))
    return bad


if __name__ == '__main__':
    cmd, rest = sys.argv[1], sys.argv[2:]
    if cmd == 'fix':
        cmd_fix(rest)
    elif cmd == 'apply':
        cmd_apply(rest)
    elif cmd == 'verify':
        sys.exit(1 if cmd_verify(rest) else 0)
    else:
        print(__doc__)
        sys.exit(2)
