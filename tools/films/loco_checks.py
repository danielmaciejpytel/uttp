"""Checks behind the report of the locomotion comparison films (2026-11-09).  Read only; writes Build\\work\\loco-films-20261109\\checks.json.

  python -I loco_checks.py [gate|tail|slides|spine|rawclip|dbentry ...]      (default: all six)
gate   UTPP / game frames with base-gate margin < 0 per feature: 'attributed' = the lowest-margin feature of the frame (what the worst
       sheet shows, loco_film.gate_margin), 'independent' = every feature outside its range; worst frame, value and range per feature.
tail   the DLL leg tail: sole height of the FootLocker INPUT (R lines of the replay dump = evaluator pose after overlay / style / grip IK)
       vs its OUTPUT (P lines = what the films show), both at the same visual-root placement; film 3 slide entry frame by frame.
slides slide episodes (PSM LocomotionDetailed 5) of the recorded in-game feeds (utpp-feed-*.txt): gait before the
       slide, entry speed, duration.
spine  spine / neck segment lengths: vanilla player_locomotion clips (left side) and the UTPP output (right side) vs the player rig
       reference (melee P.ref rest = REST_W), the player_locomotion glb bind pose and the gate reference (RG.seg_rest).
rawclip sole height of the retargeted source pack SOURCE clips (Animations\\retarget\\out_sourcepack_slide_woman / out_sourcepack_set3_woman,
       utppclip) at the clip times the replay played vs the evaluator pose: is a foot below the ground already in the source clip?
dbentry entries of the slide entry clip in the installed .utppmm (read only): sampling range and selectable rows (clip times).
"""
import collections
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import loco_common as LC  # noqa: E402
import loco_feed_stats as FS  # noqa: E402
import loco_film as LF  # noqa: E402
import loco_game as LG  # noqa: E402
import anatomy_gate as AG  # noqa: E402  (melee, unchanged: SEGS order of RG.seg_rest)

NB, NR, P = LC.NB, LC.NR, LC.P
IX = P.ix
SPINE = [('Hips', 'Spine'), ('Spine', 'Spine1'), ('Spine1', 'Spine2'), ('Spine2', 'Spine3'), ('Spine3', 'Neck'), ('Neck', 'Neck1'),
         ('Neck1', 'Head')]
_SIDES = {}


def sides(n):
    if n not in _SIDES:
        _SIDES[n] = LF.sides(n)
    return _SIDES[n]


def gate_feats(W):
    """per base-gate feature (same selection as loco_film.gate_margin): normalised margin [T], value [T], lo, hi."""
    g = NB.GB.frame_gate(W[:, :73], NB.RG, steps=True, al=NB.AL)
    T = len(W)
    out = {}
    for k, v in g['feat'].items():
        if k not in NB.AL or 'knee_dev' in k or 'elbow_dev' in k or 'shin_roll' in k:
            continue
        lo, hi = (float(x) for x in NB.AL[k])
        f = np.asarray(v, float).reshape(-1)
        if len(f) != T or not (np.isfinite(lo) and np.isfinite(hi)) or hi - lo <= 0:
            continue
        m = np.minimum(f - lo, hi - f) / (hi - lo)
        if np.median(np.abs(m)) < 1e-3:
            continue
        out[k] = (m, f, lo, hi)
    return out


def runs(mask):
    e = np.flatnonzero(np.diff(np.concatenate([[0], mask.astype(int), [0]])))
    return list(zip(e[::2].tolist(), (e[1::2] - 1).tolist()))


def check_gate():
    res = {}
    for n in ('1', '3'):
        W, lab, _, _ = sides(n)
        for s in ('utpp', 'game'):
            F = gate_feats(W[s])
            names = list(F)
            Mm = np.stack([F[k][0] for k in names], 1)
            best = Mm.min(1)
            ref, _ = LF.gate_margin(W[s])
            assert np.allclose(best, ref), 'gate margin differs from loco_film.gate_margin'
            arg = np.array(names)[Mm.argmin(1)]
            fail = best < 0
            r = dict(frames=len(best), fail_frames=int(fail.sum()),
                     attributed=dict(collections.Counter(arg[fail].tolist()).most_common()), features={})
            for k in names:
                m, f, lo, hi = F[k]
                bad = m < 0
                if not bad.any():
                    continue
                i = int(np.argmin(m))
                rr = runs(bad)
                r['features'][k] = dict(frames=int(bad.sum()), worst_frame=i, t=round(i / LC.FPS, 3), margin=round(float(m[i]), 3),
                                        value=round(float(f[i]), 2), lo=round(lo, 2), hi=round(hi, 2), clip=lab[s][i], runs=len(rr),
                                        longest_run_frames=max(b - a + 1 for a, b in rr),
                                        clips=collections.Counter(lab[s][j] for j in np.flatnonzero(bad)).most_common(5))
            res['film%s_%s' % (n, s)] = r
            print('\n[gate] film %s %s: margin < 0 in %d / %d frames; attributed %s' % (n, s, r['fail_frames'], r['frames'], r['attributed']))
            for k, v in sorted(r['features'].items(), key=lambda kv: -kv[1]['frames']):
                print('   %-24s %5d fr (%d runs, longest %d) worst %.2f at %.2f s value %.2f range [%.2f, %.2f] %s | %s' %
                      (k, v['frames'], v['runs'], v['longest_run_frames'], v['margin'], v['t'], v['value'], v['lo'], v['hi'], v['clip'],
                       ', '.join('%s %d' % c for c in v['clips'][:3])))
    return res


def check_tail():
    res = {}
    for n in ('1', '3'):
        d = LC.load_dump(os.path.join(LC.WORK, 'replay', LF.FILMS[n][0] + '.dump.txt'))
        assert 'pre_lt' in d, 'dump without R lines: rebuild / rerun loco_replay_build.py + loco_replay_run.py'
        Wo, Wi = LC.utpp_world(d), LC.utpp_world(d, pre=True)
        so, si = LC.sole_heights(Wo), LC.sole_heights(Wi)
        z = lambda W, j: W[:, IX[j], 2, 3]  # noqa: E731
        mo, mi = so.min(1), si.min(1)
        r = dict(min_sole_in=round(float(mi.min()), 4), frame_in=int(np.argmin(mi)), min_sole_out=round(float(mo.min()), 4),
                 frame_out=int(np.argmin(mo)), below2cm_in=int(np.sum(mi < -0.02)), below2cm_out=int(np.sum(mo < -0.02)),
                 tail_pushes_below=int(np.sum((mi >= -0.02) & (mo < -0.02))), tail_lifts_above=int(np.sum((mi < -0.02) & (mo >= -0.02))),
                 knee_below0_in=int(np.sum(np.minimum(z(Wi, 'LeftLeg'), z(Wi, 'RightLeg')) < 0)),
                 knee_below0_out=int(np.sum(np.minimum(z(Wo, 'LeftLeg'), z(Wo, 'RightLeg')) < 0)),
                 hips_dz_max_cm=round(float(np.abs(z(Wo, 'Hips') - z(Wi, 'Hips')).max() * 100), 2),
                 below5cm_out=int(np.sum(mo < -0.05)),
                 below2cm_by_clip=collections.Counter(d['meta'][i]['clip'] for i in np.flatnonzero(mo < -0.02)).most_common(8),
                 below5cm_by_clip=collections.Counter(d['meta'][i]['clip'] for i in np.flatnonzero(mo < -0.05)).most_common(8))
        f = r['frame_out']
        lo, hi = (300, 336) if n == '3' else (max(0, f - 8), min(len(mo), f + 9))
        rows = []
        for i in range(lo, hi):
            m = d['meta'][i]
            rows.append(dict(f=i, t=round(i / LC.FPS, 3), clip=m['clip'], clip_t=round(m['clip_t'], 3),
                             soleL_in=round(float(si[i, 0]) * 100, 1), soleL_out=round(float(so[i, 0]) * 100, 1),
                             soleR_in=round(float(si[i, 1]) * 100, 1), soleR_out=round(float(so[i, 1]) * 100, 1),
                             kneeL_in=round(float(z(Wi, 'LeftLeg')[i]) * 100, 1), kneeL_out=round(float(z(Wo, 'LeftLeg')[i]) * 100, 1),
                             toeL_in=round(float(z(Wi, 'LeftToeBase')[i]) * 100, 1), toeL_out=round(float(z(Wo, 'LeftToeBase')[i]) * 100, 1),
                             hips_in=round(float(z(Wi, 'Hips')[i]) * 100, 1), hips_out=round(float(z(Wo, 'Hips')[i]) * 100, 1),
                             contact=d['contact'][i].tolist()))
        r['rows'] = rows
        res['film' + n] = r
        print('\n[tail] film %s: min sole in %.1f cm (f%d) out %.1f cm (f%d); frames < -2 cm in %d out %d; tail pushes below %d, lifts above %d; '
              'knee z < 0 in %d out %d; max |hips dz| %.2f cm' % (n, r['min_sole_in'] * 100, r['frame_in'], r['min_sole_out'] * 100, f,
                                                                 r['below2cm_in'], r['below2cm_out'], r['tail_pushes_below'],
                                                                 r['tail_lifts_above'], r['knee_below0_in'], r['knee_below0_out'],
                                                                 r['hips_dz_max_cm']))
        print('   f     t     clip                                  clip_t | soleL in/out | soleR in/out | kneeL in/out | toeL in/out | hips in/out | cont')
        for w in rows:
            print('   %4d %6.3f %-38s %5.3f | %6.1f %6.1f | %6.1f %6.1f | %6.1f %6.1f | %6.1f %6.1f | %5.1f %5.1f | %s' %
                  (w['f'], w['t'], w['clip'][:38], w['clip_t'], w['soleL_in'], w['soleL_out'], w['soleR_in'], w['soleR_out'], w['kneeL_in'],
                   w['kneeL_out'], w['toeL_in'], w['toeL_out'], w['hips_in'], w['hips_out'], w['contact']))
    return res


def check_slides():
    paths = sorted(glob.glob(os.path.join(LC.ROOT, 'Research', 'logs', 'game', 'utpp-feed-*.txt')))
    eps = []
    for p in paths:
        a = FS.load(p)
        if len(a) < 10:
            continue
        t, dt = a[:, 0], a[:, 1]
        sp = np.hypot(a[:, 2], a[:, 3])
        loco = a[:, 6].astype(int)
        det = ((a[:, 7].astype(np.int64) >> 16) & 0xFF) - 1
        i = 0
        while i < len(a):
            if det[i] != 5:
                i += 1
                continue
            j = i
            while j + 1 < len(a) and det[j + 1] == 5:
                j += 1
            win = (t >= t[i] - 0.3) & (t < t[i])
            eps.append(dict(file=os.path.basename(p), t=float(t[i]), dur=float(t[j] - t[i] + dt[j]), before=float(sp[max(i - 1, 0)]),
                            entry=float(sp[i]), peak_before=float(sp[win].max()) if win.any() else float('nan'),
                            loco_before=int(loco[max(i - 1, 0)]), loco_at=int(loco[i]), loco3_win=bool(np.any(loco[win] == 3)),
                            cut=bool(i == 0 or j == len(a) - 1)))
            i = j + 1
    e = [x for x in eps if not x['cut']]
    entry = np.array([x['entry'] for x in e])
    dur = np.array([x['dur'] for x in e])
    # fragments: an episode starting < 0.25 s after the previous one ended (same file) continues it
    mg = []
    for x in e:
        if mg and mg[-1]['file'] == x['file'] and x['t'] - (mg[-1]['t'] + mg[-1]['dur']) < 0.25:
            mg[-1]['dur'] = x['t'] + x['dur'] - mg[-1]['t']
            mg[-1]['parts'] += 1
        else:
            mg.append(dict(x, parts=1))
    mfull = [x for x in mg if x['dur'] >= 3.0]
    merged = dict(episodes=len(mg), multi_part=int(sum(x['parts'] > 1 for x in mg)),
                  loco_before=dict(collections.Counter(x['loco_before'] for x in mg).most_common()),
                  loco3_within_0p3s=int(sum(x['loco3_win'] for x in mg)), entry_ge_7p5=int(sum(x['entry'] >= 7.5 for x in mg)),
                  sprint_and_ge_7p5=int(sum(x['loco_before'] == 3 and x['entry'] >= 7.5 for x in mg)),
                  shorter_0p1s=int(sum(x['dur'] < 0.1 for x in mg)),
                  full_ge_3s=len(mfull), full_sprint=int(sum(x['loco_before'] == 3 for x in mfull)),
                  full_entry_p=[round(float(v), 2) for v in np.percentile([x['entry'] for x in mfull], [0, 50, 100])] if mfull else [],
                  not_sprint=[dict(file=x['file'], t=round(x['t'], 2), entry=round(x['entry'], 2), loco_before=x['loco_before'],
                                   dur=round(x['dur'], 2), parts=x['parts']) for x in mg if x['loco_before'] != 3][:15])
    r = dict(files=len(paths), episodes=len(eps), cut_by_recording=len(eps) - len(e), counted=len(e), merged=merged,
             loco_before=dict(collections.Counter(x['loco_before'] for x in e).most_common()),
             loco_at_entry=dict(collections.Counter(x['loco_at'] for x in e).most_common()),
             loco3_within_0p3s=int(sum(x['loco3_win'] for x in e)),
             entry_ge_7p5=int(np.sum(entry >= 7.5)), loco3_before_and_entry_ge_7p5=int(sum(x['loco_before'] == 3 and x['entry'] >= 7.5 for x in e)),
             entry_speed_p=[round(float(v), 2) for v in np.percentile(entry, [0, 10, 50, 90, 100])],
             duration_p=[round(float(v), 2) for v in np.percentile(dur, [0, 10, 50, 90, 100])],
             full_ge_3s=int(np.sum(dur >= 3.0)),
             low_entry=[dict(file=x['file'], t=round(x['t'], 2), entry=round(x['entry'], 2), loco_before=x['loco_before'], dur=round(x['dur'], 2))
                        for x in e if x['entry'] < 7.5][:12])
    print('\n[slides]', json.dumps(r, indent=1))
    return r


def seglen(W, a, b):
    return np.linalg.norm(W[..., IX[a], :3, 3] - W[..., IX[b], :3, 3], axis=-1)


def check_spine():
    gate = dict(zip(AG.SEGS, NB.RG.seg_rest))
    sk = LG.skel()
    clips = list(NR.GA.clips(sk.gj))
    allW = {c: LG.clip(c)['W'] for c in clips}
    used = set()
    for n in ('1', '3'):
        used |= {c for lbl in sides(n)[1]['game'] for c in str(lbl).replace('>', ' ').replace('+', ' ').split() if c in allW}
    Wu = np.concatenate([sides(n)[0]['utpp'] for n in ('1', '3')])
    res = dict(clips_total=len(clips), clips_in_films=sorted(used), segments={})
    print('\n[spine] %d clips in the glb, %d used by the left side of the films' % (len(clips), len(used)))
    print('   segment            gate   playerRef  glbBind | vanilla all clips min / p50 / max (clip of max) | used clips max | UTPP min / max')
    for a, b in SPINE:
        g = gate.get((a, b), float('nan'))
        pr = float(seglen(NB.REST_W, a, b))
        gb = float(np.linalg.norm(sk.rest_t[IX[b]])) if P.par[IX[b]] == IX[a] else float('nan')
        per = {c: seglen(W, a, b) for c, W in allW.items()}
        v = np.concatenate(list(per.values()))
        cmax = max(per, key=lambda c: per[c].max())
        umax = max((per[c].max() for c in used), default=float('nan'))
        u = seglen(Wu, a, b)
        res['segments']['%s-%s' % (a, b)] = dict(gate=round(g, 4), player_ref=round(pr, 4), glb_bind=round(gb, 4),
                                                 vanilla_min=round(float(v.min()), 4), vanilla_p50=round(float(np.median(v)), 4),
                                                 vanilla_max=round(float(v.max()), 4), vanilla_max_clip=cmax, used_max=round(float(umax), 4),
                                                 utpp_min=round(float(u.min()), 4), utpp_max=round(float(u.max()), 4),
                                                 vanilla_max_vs_ref_pct=round((float(v.max()) / pr - 1) * 100, 1),
                                                 vanilla_max_vs_gate_pct=round((float(v.max()) / g - 1) * 100, 1),
                                                 used_max_vs_ref_pct=round((float(umax) / pr - 1) * 100, 1),
                                                 used_max_vs_gate_pct=round((float(umax) / g - 1) * 100, 1),
                                                 ref_vs_gate_pct=round((pr / g - 1) * 100, 1))
        print('   %-16s %6.4f  %6.4f  %6.4f | %6.4f / %6.4f / %6.4f (%s) | %6.4f | %6.4f / %6.4f' %
              (a + '-' + b, g, pr, gb, v.min(), np.median(v), v.max(), cmax, umax, u.min(), u.max()))
    return res


RAW_DIRS = [os.path.join(LC.ROOT, 'Animations', 'retarget', d) for d in ('out_sourcepack_slide_woman', 'out_sourcepack_set3_woman')]


def raw_clip(name):
    """the retargeted source pack source clip (utppclip, woman rig, root motion included) -> W [n,73,4,4], fps, path (None if not found)."""
    for d in RAW_DIRS:
        p = os.path.join(d, name + '.utppclip.json')
        if os.path.isfile(p):
            names, par = NB.BB.rig()[:2]
            assert list(names) == list(P.names[:73]), 'utppclip rig order differs from the player rig'
            W, clip = NB.BB.CP.world_from_utppclip(p, names, par)
            fps = float(clip.get('fps') or clip.get('frame_rate') or clip.get('sample_rate') or 30.0)
            return W, fps, p
    return None


def check_rawclip():
    """sole / hips height of the SOURCE clip at the clip times the replay played (no inertializer, overlay, style or leg tail)."""
    res = {}
    for n in ('1', '3'):
        d = LC.load_dump(os.path.join(LC.WORK, 'replay', LF.FILMS[n][0] + '.dump.txt'))
        Wi = LC.utpp_world(d, pre=True)
        si = LC.sole_heights(Wi)
        clips = sorted({m['clip'] for m in d['meta']})
        rows = []
        for c in clips:
            rc = raw_clip(c)
            fr = [i for i, m in enumerate(d['meta']) if m['clip'] == c]
            if rc is None:
                rows.append(dict(clip=c, frames=len(fr), raw='not found'))
                continue
            W, fps, p = rc
            sr = LC.sole_heights(W)
            k = np.clip(np.rint(np.array([d['meta'][i]['clip_t'] for i in fr]) * fps).astype(int), 0, len(W) - 1)
            played = sr[k].min(1)
            rows.append(dict(clip=c, frames=len(fr), fps=fps, src=os.path.relpath(p, LC.ROOT),
                             raw_min_sole_cm=round(float(sr.min()) * 100, 1), raw_min_sole_at_played_t_cm=round(float(played.min()) * 100, 1),
                             eval_min_sole_cm=round(float(si[fr].min()) * 100, 1),
                             raw_below2_played=int(np.sum(played < -0.02)), eval_below2=int(np.sum(si[fr].min(1) < -0.02))))
        res['film' + n] = rows
        print('\n[rawclip] film %s: source clip sole (at the played clip times) vs evaluator pose (FootLocker input), cm' % n)
        for r in rows:
            if 'raw' in r:
                print('   %-46s %4d fr  source not found' % (r['clip'], r['frames']))
            elif r['eval_below2'] or r['raw_below2_played']:
                print('   %-46s %4d fr  raw min %6.1f (played %6.1f, %3d fr < -2) | eval min %6.1f (%3d fr < -2)  %s' %
                      (r['clip'], r['frames'], r['raw_min_sole_cm'], r['raw_min_sole_at_played_t_cm'], r['raw_below2_played'],
                       r['eval_min_sole_cm'], r['eval_below2'], r['src']))
        if n == '3':
            rc = raw_clip('M_Neutral_Slide_FootOut_Into_Lfoot')
            if rc is not None:
                W, fps, p = rc
                sr = LC.sole_heights(W)
                hz = W[:, IX['Hips'], 2, 3]
                ent = []
                for i in range(306, 324):
                    m = d['meta'][i]
                    if m['clip'] != 'M_Neutral_Slide_FootOut_Into_Lfoot':
                        continue
                    k = int(np.clip(np.rint(m['clip_t'] * fps), 0, len(W) - 1))
                    ent.append(dict(f=i, clip_t=round(m['clip_t'], 3), raw_soleL=round(float(sr[k, 0]) * 100, 1), raw_hips=round(float(hz[k]) * 100, 1),
                                    eval_soleL=round(float(si[i, 0]) * 100, 1), eval_hips=round(float(Wi[i, IX['Hips'], 2, 3]) * 100, 1)))
                res['slide_entry'] = dict(fps=fps, src=os.path.relpath(p, LC.ROOT), rows=ent,
                                          raw_hips_t0=round(float(hz[0]) * 100, 1), raw_dur=round((len(W) - 1) / fps, 3))
                print('   slide entry, source %s (%.0f fps, %.2f s, hips at t0 %.1f cm):' % (res['slide_entry']['src'], fps,
                                                                                         res['slide_entry']['raw_dur'], hz[0] * 100))
                for e in ent:
                    print('     f%(f)d clip_t %(clip_t).3f | raw soleL %(raw_soleL)6.1f hips %(raw_hips)5.1f | eval soleL %(eval_soleL)6.1f hips %(eval_hips)5.1f' % e)
    return res


def check_dbentry(clip='M_Neutral_Slide_FootOut_Into_Lfoot'):
    """entries of one clip in the installed .utppmm v2 (read only): database, sampling range (rangeMin / rangeMax), rows and the clip
    times of the selectable (non-blocked) rows - can the matcher start the slide entry clip before t 0.93 s at all?"""
    import struct
    sys.path.insert(0, os.path.join(LC.ROOT, 'Source', 'tools', 'mmbake'))
    import export_style_weights as ESW  # noqa: E402  (unchanged: HEADER_FMT of the .utppmm v2 header)
    sys.path.insert(0, os.path.join(HERE, '..'))
    from utpp_paths import game_dir  # noqa: E402
    path = os.path.join(game_dir(), 'red4ext', 'plugins', 'UTPP', 'data', 'utpp_motion_woman.utppmm')
    b = open(path, 'rb').read()
    h = struct.unpack_from(ESW.HEADER_FMT, b, 0)
    assert h[0] == b'UTPPMMDB' and h[1] == 2
    num_clips, num_sections, table_off = h[8], h[18], h[19]
    secs = {}
    for i in range(num_sections):
        sid, cnt, esz, _, off, size = struct.unpack_from('<IIIIQQ', b, table_off + 32 * i)
        secs[struct.pack('<I', sid).decode('ascii')] = (cnt, esz, off, size)
    strs = secs['STRS'][2]

    def name_at(o):
        return b[strs + o:b.index(b'\0', strs + o)].decode('utf-8')
    clips = [struct.unpack_from('<IIIIIIQ', b, secs['CLIP'][2] + 32 * c) for c in range(num_clips)]
    ci = [i for i, c in enumerate(clips) if name_at(c[0]) == clip]
    assert len(ci) == 1, (clip, ci)
    ci = ci[0]
    _, first, nfr, flags, loop, nsel, mask = clips[ci]
    dbn = {d: name_at(struct.unpack_from('<I', b, secs['DBAS'][2] + secs['DBAS'][1] * d)[0]) for d in range(secs['DBAS'][0])}
    fps = 30.0  # source pack clips at 30 fps in the database: checked below against the clip length of the source meta (6.07 s)
    res = dict(db=path, sha_prefix=None, clip=clip, first_frame=first, num_frames=nfr, num_selectable_clip=nsel, entries=[])
    for e in range(secs['DENT'][0]):
        dbi, c, ef, rmin, rmax, fr0, nr, _ = struct.unpack_from('<IIIffIII', b, secs['DENT'][2] + 32 * e)
        if c != ci:
            continue
        rows = [struct.unpack_from('<IffII', b, secs['DROW'][2] + 20 * r) for r in range(fr0, fr0 + nr)]
        sel = sorted((rw[0] - first) / fps for rw in rows if not (rw[4] & 1))
        blk = sorted((rw[0] - first) / fps for rw in rows if rw[4] & 1)
        res['entries'].append(dict(database=dbn.get(dbi, dbi), flags=ef, range_min=round(rmin, 3), range_max=round(rmax, 3), rows=nr,
                                   selectable=len(sel), sel_t_min=round(sel[0], 3) if sel else None, sel_t_max=round(sel[-1], 3) if sel else None,
                                   blocked=len(blk), blocked_t=[round(blk[0], 3), round(blk[-1], 3)] if blk else None))
    res['clip_len_s_at_30fps'] = round((nfr - 1) / fps, 3)
    print('\n[dbentry]', json.dumps(res, indent=1))
    return res


def main(argv):
    todo = argv or ['gate', 'tail', 'slides', 'spine', 'rawclip', 'dbentry']
    out_path = os.path.join(LC.WORK, 'checks.json')
    res = json.load(open(out_path, encoding='utf-8')) if os.path.isfile(out_path) else {}
    fn = dict(gate=check_gate, tail=check_tail, slides=check_slides, spine=check_spine, rawclip=check_rawclip, dbentry=check_dbentry)
    for k in todo:
        res[k] = fn[k]()
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
    print('\nCHECKS', out_path)


if __name__ == '__main__':
    main(sys.argv[1:])
