"""Batch of the MotionBricks stage (melee_regenerate.py) over the non-additive clips of one rig: resumable (skips existing .v4.npz), ONE MotionBricks run per chunk of clips.

  python -I melee_regenerate_batch.py [--rig man] [--sets pwa_fists,...] [--limit N] [--sample N] [--seeds 0,7,11] [--chunk 12] [--force] [--out DIR] [--report PATH] [--max-comp 1.7]
                             [--mag strict|unit_tol]      (round F: magnitude rule of melee_regenerate, default unit_tol; scratch work dirs go to <dirname(--out)>\_tmp)
Clip selection = redo_batch.py (non-additive, animation_type Normal, >= 6 frames, sorted per set); dumps whose joint translations are all at the origin (degenerate
export, e.g. pma_katana/finisher_katana_02) are skipped and listed.  --sample N takes N clips spread evenly over that list (the demo clips first), --limit N the first N.
Writes batch_report.json (rows of every clip that has a .v4.info.json in --out, restricted to the selection, + summary) NEXT TO the --out dir (F5, fix round E:
default --out = melee_regenerate.MB4\\clips -> MB4\\batch_report.json, MB4 = mb4 / mb4_man per rig; a scratch --out X\\clips writes X\\batch_report.json), or to --report PATH.
"""
import glob
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_regenerate as M  # noqa: E402
import melee_common as MC  # noqa: E402
from demo_metrics import CLIPS, CLIPS_MAN  # noqa: E402


def selection(sets):
    out, skipped = [], []
    for s in sets:
        for f in sorted(glob.glob(os.path.join(MC.WORK, 'npz', s, '*.npz'))):
            c = os.path.basename(f)[:-4]
            z = np.load(f, allow_pickle=False)
            meta = json.loads(str(z['meta']))
            if meta.get('additive') or meta.get('animation_type', 'Normal') != 'Normal' or len(z['W']) < 6:
                continue
            if float(np.abs(z['W'][..., :3, 3]).max()) < 1e-6:      # degenerate dump: every joint at the origin
                skipped.append((s, c))
                continue
            out.append((s, c))
    if skipped:
        print('SELECTION skipped degenerate dumps (every joint at the origin): %s' % skipped, flush=True)
    return out


def report(pairs, out_dir, path, wall):
    rows = []
    for s, c in pairs:
        p = os.path.join(out_dir, '%s__%s.v4.info.json' % (s, c))
        if not os.path.exists(p):
            continue
        i = json.load(open(p))
        reasons = {}
        for sg in i['segments']:
            if sg['status'] == 'fallback':
                for cd in sg.get('candidates', []):
                    for r in cd.get('reasons', []):
                        k = r.split(' ')[0] + ' ' + r.split(' ')[1] if ' ' in r else r
                        reasons[k] = reasons.get(k, 0) + 1
        jc = i.get('join_check', {})
        lg = i.get('legs', {})
        rows.append(dict(set=s, clip=c, frames=i['frames'], keys=i['keys'], keys_source=i['keys_source'], v3_pass=i['gate']['v3']['pass_frames'], v4_pass=i['gate']['v4']['pass_frames'],
                         segments_mb=i['segments_mb'], segments_fallback=i['segments_fallback'], segments_v3=i['segments_v3'], slide_v3_cm=i['foot']['v3']['slide_cm'],
                         slide_v4_cm=i['foot']['v4']['slide_cm'], foot_min_z_rel_cm=i['foot']['v4']['foot_min_z_rel_cm'], accel_v3=i['spikes']['accel_max_v3'],
                         accel_v4=i['spikes']['accel_max_v4'], hips_dev_max_cm=i['added_motion']['hips_dev_max_cm'], spine3_dev_max_deg=i['added_motion']['rot_dev_Spine3_max_deg'],
                         identical_to_v3=i['identical_to_v3'], seconds=round(i['timing_s'].get('prepare', 0) + i['timing_s'].get('finish', 0) + i['timing_s'].get('mb_share', 0), 2),
                         betas=[sg.get('beta') for sg in i['segments'] if sg['status'] == 'mb'], fallback_reasons=reasons,
                         join_converged=jc.get('converged'), join_iterations=jc.get('iterations'), join_demotions=i.get('join_demotions'),
                         fallback_why=[sg.get('why', '') for sg in i['segments'] if sg['status'] == 'fallback'],
                         lower_accel_v3=lg.get('v3', {}).get('lower_accel_max'), lower_accel_v4=lg.get('v4', {}).get('lower_accel_max'),
                         reach_max_v4=lg.get('v4', {}).get('reach_max'), knee_flex_min_v4=lg.get('v4', {}).get('knee_flex_min'),
                         knee_side_dev_max_v4=lg.get('v4', {}).get('knee_side_dev_max'), deeper_than_v3=i.get('deeper_than_v3', {}),
                         deeper_over_tol=i.get('deeper_over_tol', {}), new_failing_frames=len(i.get('new_failing_frames', [])),
                         mag_mode=i.get('settings', {}).get('mag_mode', 'strict')))
    if not rows:
        return
    sm = dict(clips=len(rows), frames=sum(r['frames'] for r in rows), v3_pass_frames=sum(r['v3_pass'] for r in rows), v4_pass_frames=sum(r['v4_pass'] for r in rows),
              clips_v4_pass_lt_v3=sum(1 for r in rows if r['v4_pass'] < r['v3_pass']), segments_mb=sum(r['segments_mb'] for r in rows),
              segments_fallback=sum(r['segments_fallback'] for r in rows), segments_v3=sum(r['segments_v3'] for r in rows),
              clips_with_mb=sum(1 for r in rows if r['segments_mb']), clips_identical_to_v3=sum(1 for r in rows if r['identical_to_v3']),
              slide_v4_max_cm=max(r['slide_v4_cm'] for r in rows), slide_v3_max_cm=max(r['slide_v3_cm'] for r in rows), foot_min_z_rel_min_cm=min(r['foot_min_z_rel_cm'] for r in rows),
              hips_dev_mean_cm=round(float(np.mean([r['hips_dev_max_cm'] for r in rows])), 2), spine3_dev_mean_deg=round(float(np.mean([r['spine3_dev_max_deg'] for r in rows])), 2),
              seconds_mean=round(float(np.mean([r['seconds'] for r in rows])), 2), wall_seconds_last_run=round(wall, 1),
              clips_join_not_converged=sum(1 for r in rows if r['join_converged'] is False), join_iterations_max=max((r['join_iterations'] or 0) for r in rows),
              join_demotions=sum((r['join_demotions'] or 0) for r in rows), fallback_join_check=sum(sum(1 for w in r['fallback_why'] if w.startswith('join check')) for r in rows),
              clips_deeper_than_v3=sum(1 for r in rows if r['deeper_than_v3']), pairs_deeper_than_v3=sum(len(r['deeper_than_v3']) for r in rows),
              clips_deeper_over_tol=sum(1 for r in rows if r['deeper_over_tol']), pairs_deeper_over_tol=sum(len(r['deeper_over_tol']) for r in rows),
              new_failing_frames=sum(r['new_failing_frames'] for r in rows), mag_modes=sorted(set(r['mag_mode'] for r in rows)),
              mag_tol=dict(M.MAG_TOL) if M.MAG_MODE == 'unit_tol' else 'strict')
    lv3 = [r['lower_accel_v3'] for r in rows if r['lower_accel_v3'] is not None]
    lv4 = [r['lower_accel_v4'] for r in rows if r['lower_accel_v4'] is not None]
    if lv4:
        sm.update(lower_accel_max_v3=round(max(lv3), 3), lower_accel_max_v4=round(max(lv4), 3), reach_max_v4=max(r['reach_max_v4'] for r in rows),
                  knee_flex_min_v4=min(r['knee_flex_min_v4'] for r in rows), knee_side_dev_max_v4=max(r['knee_side_dev_max_v4'] for r in rows))
    beta_hist, why = {}, {}
    for r in rows:
        for b in r['betas']:
            beta_hist[str(b)] = beta_hist.get(str(b), 0) + 1
        for k, v in r['fallback_reasons'].items():
            why[k] = why.get(k, 0) + v
    sm['beta_histogram'] = beta_hist
    sm['fallback_candidate_reasons'] = dict(sorted(why.items(), key=lambda kv: -kv[1]))
    MC.jdump(dict(summary=sm, rows=rows), path)
    print('BATCH_REPORT', path, json.dumps(sm))


def main(a):
    sets = a[a.index('--sets') + 1].split(',') if '--sets' in a else [s for s in MC.SETS if MC.RIG_OF_SET[s] == M.RIG]
    M.set_mag(a)
    seeds = [int(x) for x in a[a.index('--seeds') + 1].split(',')] if '--seeds' in a else [0, 7, 11]
    out = os.path.abspath(a[a.index('--out') + 1]) if '--out' in a else os.path.join(M.MB4, 'clips')
    chunk = int(a[a.index('--chunk') + 1]) if '--chunk' in a else 12
    mc = float(a[a.index('--max-comp') + 1]) if '--max-comp' in a else M.MAX_COMP
    allc = selection(sets)
    demo = CLIPS if M.RIG == 'woman' else CLIPS_MAN
    if '--sample' in a:
        n = int(a[a.index('--sample') + 1])
        ids = sorted(set(int(round(x)) for x in np.linspace(0, len(allc) - 1, n)))
        sel = [x for x in demo if x in allc] + [allc[i] for i in ids if allc[i] not in demo]
        pairs = sel
    elif '--limit' in a:
        pairs = allc[:int(a[a.index('--limit') + 1])]
    else:
        pairs = allc
    rpath = os.path.abspath(a[a.index('--report') + 1]) if '--report' in a else os.path.join(os.path.dirname(out), 'batch_report.json')
    todo = pairs if '--force' in a else [p for p in pairs if not os.path.exists(os.path.join(out, '%s__%s.v4.npz' % p))]
    print('BATCH %d clips selected, %d to do, chunk %d, seeds %s, rig %s, out %s, report %s, mag %s, solver intent %s' % (len(pairs), len(todo), chunk, seeds, M.RIG, out, rpath,
          M.MAG_MODE, getattr(M.RS, 'INTENT_MODE', 'v3b')), flush=True)
    t0 = time.time()
    B, al = M.RS.Body(), M.GB.allowed()
    for k in range(0, len(todo), chunk):
        part = todo[k:k + chunk]
        for attempt in (0, 1):
            try:
                M.run_clips(part, seeds, out, max_comp=mc, B=B, al=al)
                break
            except Exception as e:      # e.g. CUDA OOM while the renderer runs: wait and retry once, then skip the chunk (resumable)
                print('CHUNK FAILED (%s): %s' % (k, str(e)[-400:]), flush=True)
                time.sleep(90)
        report(pairs, out, rpath, time.time() - t0)
        print('PROGRESS %d / %d clips, %.0f s' % (min(k + chunk, len(todo)), len(todo), time.time() - t0), flush=True)
    report(pairs, out, rpath, time.time() - t0)
    print('BATCH DONE', flush=True)


if __name__ == '__main__':
    main(sys.argv[1:])
