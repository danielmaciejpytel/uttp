"""Batch run of the redo on every non-additive clip of the female sets: gate of the game clip, redo, gate of the redo, time per clip (no skin, no render).

  python -I redo_batch.py [--rig man] [--limit N] [--sets pwa_fists,...] [--out report.json]
Writes <REDO>\\batch_report.json (woman: redo\\, man: redo_man\\; --out overrides the path).  Default sets = every set of the selected rig
(woman: pwa_* + default_melee_weapon, man: pma_*).  The summary also holds a histogram of redo_fail_features over all clips.
Round F: --out-dir DIR (explicit output folder; the base references still come from BB.REDO = redo\\ / redo_man\\, never regenerated) writes DIR\\batch_report.json
and DIR\\clips\\<set>__<clip>.v3.npz = the v3b clip exactly as melee_regenerate.make_v3 / redo_demo.py build it (solve_refined + blend_keep), and adds per row the final
(blended) gate pass, the strike-intent deviation (redo_intent: worst hand mean / max cm, solved and final) and the binding constraint at the frame of the max;
the solver mode (redo_solve.INTENT_MODE, env UTPP_REDO_INTENT) is recorded in the summary.  Without --out-dir the tool behaves exactly as before.
"""
import glob
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import anatomy_gate_base as GB  # noqa: E402
import base_body as BB  # noqa: E402
import melee_common as MC  # noqa: E402
import redo_solve as RS  # noqa: E402
import redo_intent as RI  # noqa: E402
import redo_pen as RPN  # noqa: E402


def binding(tr, S, t, wg):
    """round F: which solver stage moved hand S most at frame t: clearance kinds (eye / torso / reach / min_reach), arm-torso push, IK miss, kept game frame."""
    m = {k: float(np.linalg.norm(tr['clear'][k][S][t]) * 100) for k in tr['clear']}
    m['arm_push'] = float(np.linalg.norm(tr['final'][S][t] - tr['adj'][S][t]) * 100)
    m['ik'] = float(np.linalg.norm(tr['hand'][S][t] - tr['final'][S][t]) * 100)
    m['extension'] = float(np.linalg.norm(tr['ext'][S][t] - tr['raw'][S][t]) * 100)
    m['keep_game'] = float(wg[t]) * 100.0
    k = max(m, key=m.get)
    return dict(kind=k if m[k] > 0.5 else 'none', cm={a: round(b, 2) for a, b in m.items() if b > 0.05})


def main(a):
    lim = int(a[a.index('--limit') + 1]) if '--limit' in a else 10 ** 9
    sets = a[a.index('--sets') + 1].split(',') if '--sets' in a else [s for s in MC.SETS if MC.RIG_OF_SET[s] == BB.RIG_NAME]
    outp = a[a.index('--out') + 1] if '--out' in a else os.path.join(BB.REDO, 'batch_report.json')
    odir = os.path.abspath(a[a.index('--out-dir') + 1]) if '--out-dir' in a else None
    if odir:
        assert os.path.abspath(odir) != os.path.abspath(BB.REDO), '--out-dir must not be the reference folder %s' % BB.REDO
        os.makedirs(os.path.join(odir, 'clips'), exist_ok=True)
        outp = a[a.index('--out') + 1] if '--out' in a else os.path.join(odir, 'batch_report.json')
    B = RS.Body()
    al = GB.allowed()
    rows = []
    n = 0
    for s in sets:
        for f in sorted(glob.glob(os.path.join(MC.WORK, 'npz', s, '*.npz'))):
            c = os.path.basename(f)[:-4]
            d = MC.load_dump(f)
            if d['meta'].get('additive') or d['meta'].get('animation_type', 'Normal') != 'Normal':
                continue
            if len(d['W']) < 6:
                continue
            n += 1
            if n > lim:
                break
            t0 = time.time()
            if not np.isfinite(d['W']).all() or float(np.abs(d['W'][:, 1:, :3, 3]).max()) < 1e-6:
                # degenerate dump (e.g. pma_katana/finisher_katana_02: every joint at the origin in all 81 frames): not a solver case
                rows.append(dict(set=s, clip=c, error='degenerate dump: all joints at the origin / non-finite', seconds=0.0))
                continue
            try:
                gv = GB.frame_gate(d['W'], B.rg, al=al)
                tr = {} if odir else None
                Wr, info, intent = RS.solve_refined(B, d, trace=tr)
                gr = GB.frame_gate(Wr, B.rg, al=al)
                row = dict(set=s, clip=c, frames=len(d['W']), game_pass=int(gv['ok'].sum()), redo_pass=int(gr['ok'].sum()),
                           redo_fail_features=GB.summary(gr)['fail_features'], seconds=round(time.time() - t0, 2))
                if odir:        # round F: the v3b clip (blend_keep), its gate, the strike intent and the binding constraint
                    Wk, wg = RS.blend_keep(B, d['W'], Wr, RS.keep_mask(B, d['W'], gv['ok']))
                    gk = GB.frame_gate(Wk, B.rg, al=al)
                    np.savez(os.path.join(odir, 'clips', '%s__%s.v3.npz' % (s, c)), W=Wk, rest=B.rest, names=np.array(B.names), parents=np.array(B.par), fps=30)
                    ik = RI.summary(RI.dev(B, d['W'], Wk))
                    isv = RI.summary(RI.dev(B, d['W'], Wr))
                    S = {'L': 'Left', 'R': 'Right'}[ik['worst']['hand']]
                    row.update(final_pass=int(gk['ok'].sum()), final_fail_features=GB.summary(gk)['fail_features'], game_frames_kept=int((wg >= 1 - 1e-6).sum()),
                               intent_final=ik, intent_solved=dict(worst=isv['worst']), binding=binding(tr, S, ik[ik['worst']['hand']]['f_max'], wg),
                               arm_ratio={k: round(v, 4) for k, v in intent['arm_ratio'].items()}, seconds=round(time.time() - t0, 2),
                               pen_frames=int(RPN.pen_frames(Wk)[1].sum()), gp_fallback_frames=info.get('gp_fallback_frames', 0), surface=info.get('surface'))
                rows.append(row)
            except Exception as e:     # keep going, report
                rows.append(dict(set=s, clip=c, error=str(e)[:200], seconds=round(time.time() - t0, 2)))
    ok = [r for r in rows if 'error' not in r]
    tot = sum(r['frames'] for r in ok)
    summ = dict(clips=len(rows), errors=len(rows) - len(ok), frames=tot, game_pass_frames=sum(r['game_pass'] for r in ok),
                redo_pass_frames=sum(r['redo_pass'] for r in ok), clips_all_frames_pass=sum(1 for r in ok if r['redo_pass'] == r['frames']),
                clips_redo_ge_90pct=sum(1 for r in ok if r['redo_pass'] >= 0.9 * r['frames']),
                seconds_total=round(sum(r['seconds'] for r in rows), 1), seconds_mean=round(float(np.mean([r['seconds'] for r in rows])), 2))
    hist = {}
    for r in ok:
        for k, v in r['redo_fail_features'].items():
            hist[k] = hist.get(k, 0) + v       # frames failing the feature (top 8 per clip)
    summ['redo_fail_frames_by_feature'] = dict(sorted(hist.items(), key=lambda kv: -kv[1]))
    summ['redo_fail_clips_by_feature'] = dict(sorted(
        {k: sum(1 for r in ok if k in r['redo_fail_features']) for k in hist}.items(), key=lambda kv: -kv[1]))
    if odir:        # round F summary: final (blended) pass = the v3b baseline of the MotionBricks stage, intent deviation, binding constraints
        m = np.array([r['intent_final']['worst']['mean'] for r in ok])
        x = np.array([r['intent_final']['worst']['max'] for r in ok])
        bk = {}
        for r in ok:
            if not r['intent_final']['ok']:
                bk[r['binding']['kind']] = bk.get(r['binding']['kind'], 0) + 1
        summ.update(intent_mode=RS.INTENT_MODE, pen_mode=RS.PEN_MODE, loop_mode=RS.LOOP_MODE, final_pass_frames=sum(r['final_pass'] for r in ok), clips_final_all_pass=sum(1 for r in ok if r['final_pass'] == r['frames']),
                    intent_mean_of_means=round(float(m.mean()), 2), intent_p90_mean=round(float(np.percentile(m, 90)), 2), intent_mean_of_max=round(float(x.mean()), 2),
                    intent_p90_max=round(float(np.percentile(x, 90)), 2), intent_max_max=round(float(x.max()), 2),
                    clips_intent_mean_le_5=int((m <= RI.MEAN_MAX_CM).sum()), clips_intent_max_le_9=int((x <= RI.MAX_MAX_CM).sum()),
                    clips_intent_ok=sum(1 for r in ok if r['intent_final']['ok']), binding_of_failing_clips=dict(sorted(bk.items(), key=lambda kv: -kv[1])),
                    references=BB.REDO, clips_dir=os.path.join(odir, 'clips'), pen_frames=sum(r['pen_frames'] for r in ok), clips_pen=sum(1 for r in ok if r['pen_frames']),
                    pen_thr=RPN.proxy().thr, gp_fallback_frames=sum(r['gp_fallback_frames'] for r in ok),
                    surface_applied=sum(1 for r in ok if isinstance(r['surface'], dict) and r['surface'].get('result') == 'applied'))
    MC.jdump(dict(summary=summ, rows=rows), outp)
    print(json.dumps(summ))


if __name__ == '__main__':
    main(sys.argv[1:])
