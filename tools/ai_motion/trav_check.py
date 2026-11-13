"""Checks + packaging of the retargeted traversal gap clips (both V rigs, plain and rtz).

python -I trav_check.py select.json [--collect] [--sheets <png dir>]
Per clip and rig: foot slide over planted runs (contact curves >= 0.5, >= 4 frames; lowest of ankle / toe joint, cm per frame:
mean and max per run), floor penetration, obstacle clearance (low hurdle: min height over the footprint of ankle / toe / knee / hips above
the obstacle top), joint acceleration at the generated gap seams versus the clip's own motion, entry / exit speed, planted contact runs.
--collect copies the clips to Build/work/trav-mb-20261106/out/{man,woman}[/rtz] and writes <name>.meta.json (metadata for the bake).
"""
import json
import os
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'mmbake'))
import trav_lib as T  # noqa: E402
import mmbake_base as v1  # noqa: E402
import mmbake_motion as m2  # noqa: E402

W = T.WORK
RIGJ = {'woman': os.path.join(T.ROOT, 'Extracted', 'game', 'json', 'vanilla', 'player_woman_skeleton.rig.json'),
        'man': os.path.join(T.ROOT, 'Extracted', 'game', 'json', 'vanilla-man', 'player_man_skeleton.rig.json')}
CFG = json.load(open(os.path.join(HERE, '..', 'mmbake', 'config', 'default.json')))
KEY = ['Hips', 'LeftUpLeg', 'LeftLeg', 'LeftFoot', 'LeftToeBase', 'RightUpLeg', 'RightLeg', 'RightFoot', 'RightToeBase', 'LeftHand', 'RightHand',
       'Head', 'LeftArm', 'RightArm', 'LeftForeArm', 'RightForeArm', 'Spine3']


def world_pts(clip, rig, rtz):
    names, parents = v1.load_rig(RIGJ[rig], int(CFG['num_pose_bones']))
    cd = m2.ClipData(clip, CFG, names, parents)
    gp, _ = v1.fk_model(cd.P, cd.Q, parents)
    n = cd.N
    bi = {x: i for i, x in enumerate(names)}
    P = np.zeros((n, len(names), 3))
    for f in range(n):
        P[f, :, :2] = m2.rot2(np.full(len(names), cd.T[f, 2]), gp[f, :, :2]) + cd.T[f, :2]
        P[f, :, 2] = gp[f, :, 2] + (cd.root_z[f] if cd.root_z is not None else 0.0)
    return names, bi, P, cd


def runs(mask, minlen):
    out, f = [], 0
    while f < len(mask):
        if mask[f]:
            e = f
            while e + 1 < len(mask) and mask[e + 1]:
                e += 1
            if e - f + 1 >= minlen:
                out.append((f, e))
            f = e + 1
        else:
            f += 1
    return out



_BASE = {}


def plant_base(rig):
    """Planted level of ankle / toe of the rig (retargeted source pack Stand_Idle, min over the clip) - height-only contact reference."""
    if rig not in _BASE:
        d = os.path.join(T.ROOT, 'Animations', 'retarget', 'out_sourcepack_%s_knee' % rig)
        names, bi, P, cd = world_pts(os.path.join(d, 'M_Neutral_Stand_Idle_Loop.utppclip.json'), rig, False)
        _BASE[rig] = {s_ + k: float(P[:, bi[s_ + k], 2].min()) for s_ in ('Left', 'Right') for k in ('Foot', 'ToeBase')}
    return _BASE[rig]


def height_contacts(P, bi, rig, gen_mask, thr=0.03):
    """Contact = lowest of ankle / toe within thr of its planted level (no speed gate: a skating foot still counts). Runs >= 2 frames;
    per run the pivot joint's mean / max horizontal displacement (cm per frame); generated frames reported separately from source-pack-piece frames."""
    base = plant_base(rig)
    out = {'generated': [], 'sourcepack': []}
    for side in ('Left', 'Right'):
        a, t = bi[side + 'Foot'], bi[side + 'ToeBase']
        ha, ht = P[:, a, 2] - base[side + 'Foot'], P[:, t, 2] - base[side + 'ToeBase']
        low = np.minimum(ha, ht) < thr
        for s_, e_ in runs(low, 2):
            piv = [a if ha[f] <= ht[f] else t for f in range(s_, e_ + 1)]
            d_ = [np.linalg.norm(P[f, piv[f - s_], :2] - P[f - 1, piv[f - s_], :2]) * 100 for f in range(s_ + 1, e_ + 1) if piv[f - s_] == piv[f - 1 - s_]]
            if not d_:
                continue
            kind = 'generated' if gen_mask[s_:e_ + 1].any() else 'sourcepack'
            out[kind].append(dict(side=side, run=[int(s_), int(e_)], mean_cm_frame=round(float(np.mean(d_)), 2), max_cm_frame=round(float(np.max(d_)), 2)))
    return out


def ends_vs_neighbours(c_name, rig, clip, sel_meta):
    """First / last 3 frames of the clip vs the retargeted source frames they were cut from (local rotations in degrees, Hips z in cm)."""
    res = {}
    spans = [sp for sp in sel_meta['spans'] if not str(sp[3]).startswith('blend')]
    if not spans:
        return res
    R_ = os.path.join(T.ROOT, 'Animations', 'retarget')
    cands = ([os.path.join(T.ROOT, 'Build', 'test-data', 'LocomotionTwistFix-20261017', 'bake-inputs', 'clips-staging'),
              os.path.join(T.ROOT, 'Build', 'test-data', 'LocomotionTwistFix-20261017', 'bake-inputs', 'rtz')] if rig == 'woman' else
             [os.path.join(R_, 'out_sourcepack_man_knee_twistfix'), os.path.join(R_, 'out_sourcepack_man_rtz_knee_twistfix')]) + \
            [os.path.join(R_, 'out_sourcepack_%s_knee' % rig), os.path.join(R_, 'out_sourcepack_%s_rtz_knee' % rig)]
    names, parents = v1.load_rig(RIGJ[rig], int(CFG['num_pose_bones']))
    me = m2.ClipData(clip, CFG, names, parents)
    bi = {x: i for i, x in enumerate(names)}
    for label, sp, f_me, f_src in (('start', spans[0], 0, spans[0][4]), ('end', spans[-1], me.N - 3, spans[-1][5] - 2)):
        p = next((os.path.join(d_, sp[3] + '.utppclip.json') for d_ in cands if os.path.isfile(os.path.join(d_, sp[3] + '.utppclip.json'))), None)
        if p is None:
            continue
        try:
            src = m2.ClipData(p, CFG, names, parents)
        except SystemExit:
            continue
        a, b = me.Q[f_me:f_me + 3], src.Q[f_src:f_src + 3]
        if len(b) < 3:
            continue
        ang = np.degrees(2 * np.arccos(np.abs(np.sum(a * b, -1)).clip(0, 1)))
        res[label] = dict(src=sp[3], src_file=os.path.relpath(p, T.ROOT), src_frames=[int(f_src), int(f_src) + 2], max_local_rot_deg=round(float(ang[:, 2:73].max()), 2),
                          hips_local_z_diff_cm=round(float(np.abs(me.P[f_me:f_me + 3, bi['Hips'], 2] - src.P[f_src:f_src + 3, bi['Hips'], 2]).max() * 100), 2))
    return res


def check_clip(c, rig, rtz, sel_meta):
    d = os.path.join(W, 'rt', rig + ('_rtz' if rtz else ''))
    clip = os.path.join(d, c['name'] + '.utppclip.json')
    names, bi, P, cd = world_pts(clip, rig, rtz)
    n = len(P)
    J = json.load(open(clip))
    res = dict(frames=n, hip_ratio=cd.hip_ratio)
    # planted runs
    slide, cruns = [], {}
    for side, key in (('Left', 'contact_l'), ('Right', 'contact_r')):
        cv = np.asarray(J['curves'][key])
        rr = runs(cv >= 0.5, 4)
        cruns[side] = rr
        a, t = bi[side + 'Foot'], bi[side + 'ToeBase']
        for s, e in rr:
            piv = [a if P[f, a, 2] <= P[f, t, 2] else t for f in range(s, e + 1)]
            d_ = [np.linalg.norm(P[f, piv[f - s], :2] - P[f - 1, piv[f - s], :2]) * 100 for f in range(s + 1, e + 1) if piv[f - s] == piv[f - 1 - s]]
            if d_:
                slide.append(dict(side=side, run=[s, e], mean_cm_frame=round(float(np.mean(d_)), 2), max_cm_frame=round(float(np.max(d_)), 2)))
    res['planted_runs'] = {k: [list(map(int, r)) for r in v] for k, v in cruns.items()}
    res['contact_runs_ge2'] = {side: [list(map(int, r)) for r in runs(np.asarray(J['curves'][k]) >= 0.5, 2)] for side, k in (('Left', 'contact_l'), ('Right', 'contact_r'))}
    res['slide'] = slide
    res['slide_max_run_mean_cm_frame'] = max([x['mean_cm_frame'] for x in slide], default=0.0)
    res['slide_max_frame_cm'] = max([x['max_cm_frame'] for x in slide], default=0.0)
    # floor penetration: toe / ankle min height relative to their standing level at the first / last planted frames
    floor = min(P[:, bi['LeftToeBase'], 2].min(), P[:, bi['RightToeBase'], 2].min())
    res['toe_min_z_m'] = round(float(floor), 4)
    # speeds
    hp = P[:, bi['Hips'], :2]
    v = np.linalg.norm(np.diff(hp, axis=0), axis=1) * 30.0
    res['entry_speed_ms'] = round(float(v[:5].mean()), 2)
    res['exit_speed_ms'] = round(float(v[-5:].mean()), 2)
    # seam acceleration: key joints relative to the world, 2nd difference, gap seams vs rest
    kj = [bi[x] for x in KEY]
    acc = np.linalg.norm(P[2:, kj] - 2 * P[1:-1, kj] + P[:-2, kj], axis=-1).max(1) * 900
    gaps = sel_meta['gaps']
    keep = np.ones(len(acc), bool)
    seams = []
    for g in gaps:
        for s_ in (g['c'], g['g']):
            seams.append(float(acc[max(0, s_ - 2):s_ + 1].max()))
        keep[max(0, g['c'] - 3):g['g'] + 1] = False
    res['seam_acc_m_s2'] = [round(x, 1) for x in seams]
    res['own_acc_p99_m_s2'] = round(float(np.percentile(acc[keep], 99)), 1) if keep.any() else None
    res['max_acc_m_s2'] = round(float(acc.max()), 1)
    gen = np.zeros(n, bool)
    for g_ in sel_meta['gaps']:
        gen[g_['c'] + 1:g_['g']] = True
    for sp in sel_meta['spans']:
        if str(sp[3]).startswith('blend'):
            gen[sp[1]:sp[2] + 1] = True
    hc = height_contacts(P, bi, rig, gen)
    res['height_contacts'] = hc
    res['slide_height_generated_max_run_mean_cm_frame'] = max([x['mean_cm_frame'] for x in hc['generated']], default=0.0)
    res['slide_height_generated_max_frame_cm'] = max([x['max_cm_frame'] for x in hc['generated']], default=0.0)
    res['slide_height_generated_median_run_mean_cm_frame'] = round(float(np.median([x['mean_cm_frame'] for x in hc['generated']])), 2) if hc['generated'] else 0.0
    res['slide_height_generated_runs'] = len(hc['generated'])
    res['slide_height_sourcepack_max_run_mean_cm_frame'] = max([x['mean_cm_frame'] for x in hc['sourcepack']], default=0.0)
    bs = plant_base(rig)
    res['toe_below_baseline_cm'] = round(float(max(0.0, (bs['LeftToeBase'] - P[:, bi['LeftToeBase'], 2].min()) * 100, (bs['RightToeBase'] - P[:, bi['RightToeBase'], 2].min()) * 100)), 2)
    # obstacle clearance
    ob = sel_meta.get('obst')
    if ob and 'flight_frames' in sel_meta:
        f0 = sel_meta['flight_frames'][0]
        fa = f0 + sel_meta.get('obst_center_idx', 6)  # obstacle centre = middle of the airborne interval
        u = hp[min(fa + 3, n - 1)] - hp[max(fa - 3, 0)]
        u = u / np.linalg.norm(u)
        c0 = hp[fa]
        up4 = np.concatenate([np.linspace(P[i], P[i + 1], 4, endpoint=False) for i in range(n - 1)] + [P[-1:]])  # 4x linear upsampling: a foot passes the footprint between frames
        along = (up4[:, :, :2] - c0) @ u
        depth = ob['depth'] / 100.0 * cd.hip_ratio
        h = ob['h'] / 100.0
        res['obstacle'] = dict(h_m=h, depth_m=round(depth, 3), apex_frame=fa)
        for nm, js in (('ankle', ['LeftFoot', 'RightFoot']), ('toe', ['LeftToeBase', 'RightToeBase']), ('knee', ['LeftLeg', 'RightLeg']), ('hips', ['Hips'])):
            m = 1e9
            for j in js:
                ins = np.abs(along[:, bi[j]]) <= depth / 2
                if ins.any():
                    m = min(m, float((up4[ins, bi[j], 2] - h).min()))
            res['clear_%s_cm' % nm] = round(m * 100, 1) if m < 1e8 else None
        res['obstacle']['apex_hips_z_m'] = round(float(P[fa, bi['Hips'], 2]), 3)
        up = np.minimum(P[:, bi['LeftFoot'], 2] - P[:, bi['LeftFoot'], 2].min(), P[:, bi['RightFoot'], 2] - P[:, bi['RightFoot'], 2].min()) > 0.05
        a_, b_ = fa, fa
        while a_ > 0 and up[a_ - 1]:
            a_ -= 1
        while b_ < n - 1 and up[b_ + 1]:
            b_ += 1
        res['airborne_frames'] = [int(a_), int(b_)]
    if sel_meta.get('family') == 'top_out':
        Lg = sel_meta['ledge_height_m'] * cd.hip_ratio
        hs = {}
        for nm in ('LeftHand', 'RightHand'):
            z = P[:, bi[nm], 2]
            rr = runs(np.abs(z - Lg) < 0.08, 4)
            for s_, e_ in rr:
                d_ = [np.linalg.norm(P[f, bi[nm], :2] - P[f - 1, bi[nm], :2]) * 100 for f in range(s_ + 1, e_ + 1)]
                hs.setdefault(nm, []).append(dict(run=[int(s_), int(e_)], mean_cm_frame=round(float(np.mean(d_)), 2), max_cm_frame=round(float(np.max(d_)), 2),
                                                  z_over_ledge_cm=[round(float((z[s_:e_ + 1] - Lg).min() * 100), 1), round(float((z[s_:e_ + 1] - Lg).max() * 100), 1)]))
        res['ledge'] = dict(ledge_height_m_rig=round(Lg, 3), ledge_height_m_sourcepack=sel_meta['ledge_height_m'], hand_runs=hs,
                            end_hips_z_m=round(float(P[-1, bi['Hips'], 2]), 3), end_ankle_z_m=[round(float(P[-1, bi['LeftFoot'], 2]), 3), round(float(P[-1, bi['RightFoot'], 2]), 3)])
        res['hand_slide_max_run_mean_cm_frame'] = max([r_['mean_cm_frame'] for v_ in hs.values() for r_ in v_], default=0.0)
    return res


def main(a):
    sel = json.load(open(a[0]))
    allres = {}
    for c in sel:
        sm = json.load(open(os.path.join(W, 'asm', c['recipe'], c['var'] + '.sel.json')))
        allres[c['name']] = {}
        for rig in ('woman', 'man'):
            for rtz in (False, True):
                allres[c['name']]['%s%s' % (rig, '_rtz' if rtz else '')] = check_clip(c, rig, rtz, sm)
            allres[c['name']][rig]['ends_vs_source'] = ends_vs_neighbours(c['name'], rig, os.path.join(W, 'rt', rig, c['name'] + '.utppclip.json'), sm)
        r = allres[c['name']]
        print('%-34s W slide %.2f/%.2f  M slide %.2f/%.2f | seam %s own %s | clear W %s M %s' % (
            c['name'], r['woman']['slide_height_generated_max_run_mean_cm_frame'], r['woman']['slide_height_generated_max_frame_cm'], r['man']['slide_height_generated_max_run_mean_cm_frame'], r['man']['slide_height_generated_max_frame_cm'],
            r['woman']['seam_acc_m_s2'], r['woman']['own_acc_p99_m_s2'], r['woman'].get('clear_ankle_cm'), r['man'].get('clear_ankle_cm')) + (' | hand slide W %.2f M %.2f' % (r['woman']['hand_slide_max_run_mean_cm_frame'], r['man']['hand_slide_max_run_mean_cm_frame']) if 'ledge' in r['woman'] else ''))
    json.dump(allres, open(os.path.join(W, 'check.json'), 'w'), indent=1)
    if '--collect' in a:
        for rig in ('woman', 'man'):
            for sub in ('', 'rtz'):
                os.makedirs(os.path.join(W, 'out', rig, sub), exist_ok=True)
        for c in sel:
            sm = json.load(open(os.path.join(W, 'asm', c['recipe'], c['var'] + '.sel.json')))
            for rig in ('woman', 'man'):
                for rtz in (False, True):
                    src = os.path.join(W, 'rt', rig + ('_rtz' if rtz else ''), c['name'] + '.utppclip.json')
                    shutil.copyfile(src, os.path.join(W, 'out', rig, 'rtz' if rtz else '', c['name'] + '.utppclip.json'))
                meta = dict(name=c['name'], rig=rig, family=sm.get('family'), recipe=c['recipe'], variant=c['var'], generator=('GASP-ALS Climb_Start_2_5 pull-up rigidly raised (NOT generated; no MotionBricks)' if sm.get('family') == 'top_out' else 'NVIDIA MotionBricks (SOMA N1) in-betweening of the gap frames + GASP-ALS pieces (poses as seeds only)'),
                            fps=30, frames=allres[c['name']][rig]['frames'], gaps=sm['gaps'], spans=sm['spans'], chosen_candidates=sm['chosen'],
                            obstacle_height_m=(sm['obst']['h'] / 100.0 if sm.get('obst') else sm.get('ledge_height_m')), obstacle_depth_cm=(sm['obst']['depth'] if sm.get('obst') else None),
                            contact_frames=allres[c['name']][rig]['contact_runs_ge2'], contact_runs_ge4=allres[c['name']][rig]['planted_runs'], entry_speed_ms=allres[c['name']][rig]['entry_speed_ms'],
                            exit_speed_ms=allres[c['name']][rig]['exit_speed_ms'], metrics=allres[c['name']][rig], seeds_note='seed -1 = no jitter, n = jitter of the context history',
                            traversal_notes=('take_off_frame = first airborne frame, landing_frame = first frame after the airborne interval; obstacle centre = hips at obstacle_center_frame, edges +-depth/2 along the travel axis; no UE montage, so no M / W lines for travbake: the clip is meant for the MM database (--extra-assets)' if sm.get('family') == 'low_hurdle' else None),
                            take_off_frame=(allres[c['name']][rig].get('airborne_frames') or [None, None])[0],
                            landing_frame=((allres[c['name']][rig]['airborne_frames'][1] + 1) if allres[c['name']][rig].get('airborne_frames') else None),
                            retarget='retarget_batch.py --knee-fix --keep-all-curves (twistfix code); rtz/ = --rtz variant (root_z = ground height under the character, 0)',
                            extra={k: v for k, v in sm.items() if k in ('src_a', 'src_b', 'b_start', 'N', 'gait', 'src_apex', 'src_run', 'blend_smax', 'lift_cm', 'flight_frames', 'variant', 'dist_cm')})
                json.dump(meta, open(os.path.join(W, 'out', rig, c['name'] + '.meta.json'), 'w'), indent=1, default=float)
        print('COLLECTED', len(sel))


if __name__ == '__main__':
    main(sys.argv[1:])
