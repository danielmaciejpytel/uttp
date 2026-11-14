"""Offline proof of the reload contact correction on recorded reload episodes.

  python -I reload_contacts_proof.py --episodes EPISODES.json --eps-dir NPZ_DIR --logs LOG_DIR --index-map RIG_MAP.json
                                     --replay REPLAY_EXE --overlays WOMAN.utppov --out OUT_DIR [--owner fk|mapping] [--group REGEX]

EPISODES.json: the reload episode list ({'episodes': [{file, identify, ...}], 'groups': {name: {episodes: [...]}}}); NPZ_DIR holds the cut
episodes (<recording ticks>_<n>.npz with t0 / t1); LOG_DIR the text recordings utpp-pose-<ticks>.csv (read only); RIG_MAP.json maps game
rig index -> bone name ({'645': {name: index}}). REPLAY_EXE = replay_reload_contacts (tools/analysis/replay_reload_contacts.cpp).

For every episode of the selected groups the replay runs the contact stage of the pose hook (with the given owner) on the recorded stage 0
(game pose) / stage 1 (after the motion blend), and the survey metrics (tools/anim/survey/survey_metrics.py, thresholds unchanged) score
  stage 1 = the reload without the correction, and the replayed output = with the correction,
on the reload frames (action 2). Also reported: single-frame steps of the arm chain over the whole window (entry, reload, exit; pose meter
spike test, tools/gametest/pose_metrics.py), the left / right hand offset in the weapon frame against the game's pose (0 = the hand is where
the game's reload puts it on the weapon) and the cost of the stage per frame. Writes OUT_DIR/proof.json; reads every input only.
"""
import argparse
import concurrent.futures as cf
import json
import os
import re
import subprocess
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, os.path.join(TOOLS, 'anim', 'survey'), os.path.join(TOOLS, 'melee'), os.path.join(TOOLS, 'gametest')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

ARM_BONES = ['LeftShoulder', 'RightShoulder', 'LeftArm', 'RightArm', 'LeftForeArm', 'RightForeArm', 'LeftHand', 'RightHand']
BIG_DEG = 6.0  # pose meter 'big' step (pose_stage_pops.BIG_DEG)
BIG_CM = 3.0
MAX_MATCH_RESID_DEG = 5.0  # median arm residual of the clip match; the game-clip episodes match within ~1 deg


def episode_windows(episodes_json, eps_dir, group_rx):
    import numpy as np
    d = json.load(open(episodes_json))
    rx = re.compile(group_rx) if group_rx else None
    out = {}
    for ep in d['episodes']:
        # the matched game clip set is the weapon; a large residual = no game clip (a purpose-built reload): not a subject here
        if not ep.get('identify') or ep.get('resid') is None or ep['resid'] > MAX_MATCH_RESID_DEG:
            continue
        name = ep['identify'][0][0][0].split(':', 1)[1]
        if rx and not rx.search(name):
            continue
        if True:
            key = ep['file']
            z = np.load(os.path.join(eps_dir, key))
            csv = 'utpp-pose-%s.csv' % key.rsplit('_', 1)[0]
            # 0.5 s before the reload edge, 1.0 s after its end (the exit fade and the hand back to the motion pose)
            out.setdefault(csv, []).append(dict(key=key, group=name, a=int(z['t0']) - 5_000_000, b=int(z['t1']) + 10_000_000,
                                                t0=int(z['t0']), t1=int(z['t1'])))
    return out


def run_replay(exe, logs, overlays, csv, wins, out_dir, owner):
    base = os.path.join(out_dir, '%s_%s' % (csv[:-4], owner))
    wfile = base + '.windows.txt'
    with open(wfile, 'w') as f:
        for w in wins:
            f.write('%d %d %s\n' % (w['a'], w['b'], w['key']))
    r = subprocess.run([exe, os.path.join(logs, csv), overlays, wfile, base + '.csv', base + '.frames.csv', owner],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError('%s: %s' % (csv, r.stderr))
    return base


def arm_steps(W, ix, sec, windows=()):
    """Single-frame steps of the arm chain (pose meter spike test): rotations in model space (deg), hand positions (cm). Returns the largest
    excess over all frames, the count of 'big' steps, and the largest excess inside each (first, last) seconds window (entry / exit)."""
    import numpy as np
    import pose_metrics as pm
    from scipy.spatial.transform import Rotation as R
    dt = np.r_[np.nan, np.diff(sec)]
    n = len(sec)
    deg = np.zeros(n)
    cm = np.zeros(n)
    big = 0
    for b in ARM_BONES:
        q = R.from_matrix(W[:, ix[b], :3, :3]).as_quat()
        _, sp, _ = pm.angular_series(q, dt, False)
        m, ex = pm.spike_mask(sp, dt, pm.SPIKE_MIN_DEG)
        deg = np.where(m, np.maximum(deg, np.nan_to_num(ex)), deg)
        big += int((m & (np.nan_to_num(ex) >= BIG_DEG)).sum())
    for b in ('LeftHand', 'RightHand'):
        p = W[:, ix[b], :3, 3] * 100
        v = np.r_[np.nan, np.linalg.norm(np.diff(p, axis=0), axis=1)] / dt
        m, ex = pm.spike_mask(v, dt, pm.SPIKE_MIN_CM)
        cm = np.where(m, np.maximum(cm, np.nan_to_num(ex)), cm)
        big += int((m & (np.nan_to_num(ex) >= BIG_CM)).sum())
    out = dict(step_deg=round(float(deg.max()), 2), step_cm=round(float(cm.max()), 2), big_steps=big)
    for name, (a, b) in windows:
        w = (sec >= a) & (sec <= b)
        out[name + '_step_deg'] = round(float(deg[w].max()), 2) if w.any() else 0.0
        out[name + '_step_cm'] = round(float(cm[w].max()), 2) if w.any() else 0.0
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--episodes', required=True)
    ap.add_argument('--eps-dir', required=True)
    ap.add_argument('--logs', required=True)
    ap.add_argument('--index-map', required=True)
    ap.add_argument('--replay', required=True)
    ap.add_argument('--overlays', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--owner', default='fk', choices=('fk', 'mapping', 'off'))
    ap.add_argument('--group')
    a = ap.parse_args(argv)
    import numpy as np
    import pandas as pd
    import pose_clip_match as PM
    sys.argv = [sys.argv[0], '--rig', 'woman']  # the survey reads its rig at import; every recorded episode is Female V
    import survey_metrics as SM
    import melee_common as MC
    ctx = SM.Ctx()
    ix = ctx.ix
    os.makedirs(a.out, exist_ok=True)
    maps = json.load(open(a.index_map))
    idx2name = {int(i): n for n, i in maps['645'].items() if i >= 0}
    names_sorted = [n for _, n in sorted(idx2name.items())][:73]
    assert names_sorted == ctx.names, 'recording joint order differs from the survey order'
    wins = episode_windows(a.episodes, a.eps_dir, a.group)
    with cf.ThreadPoolExecutor(max_workers=min(9, len(wins))) as pool:
        futs = {pool.submit(run_replay, a.replay, a.logs, a.overlays, csv, w, a.out, a.owner): csv for csv, w in wins.items()}
        bases = {futs[f]: f.result() for f in cf.as_completed(futs)}
    from scipy.spatial.transform import Rotation as R
    rq = R.from_matrix(ctx.rg.Lrest).as_quat()

    def fill(lt, lr):
        lt = lt.copy(); lr = lr.copy()
        bad = ~np.isfinite(lt).all(-1) | ~np.isfinite(lr).all(-1)
        for j in range(lt.shape[1]):
            m = bad[:, j]
            if m.any():
                lt[m, j] = ctx.rg.Lrest_t[j]
                lr[m, j] = rq[j]
        return lt, lr

    def measure(lt, lr, sel, sec, windows):
        W = MC.fk_world(ctx.par, np.asarray(lt, float), np.asarray(lr, float))
        f, ex = ctx.measure(W[sel], legs=True)
        s = SM.summarize(ex, int(sel.sum()), SM.glitch_mask(W[sel], ctx.par))
        d = np.linalg.norm(W[sel][:, ix['LeftHand'], :3, 3] - W[sel][:, ix['WeaponRight'], :3, 3], axis=-1) * 100
        r = dict(severity=s['severity'], fail_pct=s['fail_pct'], worst_feature=s['worst_feature'], top_fail=s['fail_features'],
                 lh_weapon_cm=round(float(np.median(d)), 1))
        r.update(arm_steps(W, ix, sec, windows))
        return r

    rows = []
    for csv, w in wins.items():
        base = bases[csv]
        recs = PM.load_csv_windows(base + '.csv', [(x['a'], x['b']) for x in w], idx2name, names_sorted)
        fr = pd.read_csv(base + '.frames.csv')
        for x, rec in zip(w, recs):
            if rec is None:
                continue
            act = np.asarray(rec.info['action'])
            sel = act == 2
            ts, te = x['t0'] / 1e7, x['t1'] / 1e7
            windows = (('entry', (ts - 0.3, ts + 0.4)), ('exit', (te - 0.2, te + 0.8)))
            row = dict(key=x['key'], group=x['group'], frames=int(sel.sum()))
            for stage, name in ((1, 'without'), (2, 'with')):
                lt, lr = fill(rec.lt[:, stage], rec.lr[:, stage])
                row[name] = measure(lt, lr, sel, rec.sec, windows)
            f = fr[fr['label'] == x['key']]
            acc = f[f['captured'] == 1]
            row['frames_written'] = int(f['written'].sum())
            row['frames_admitted'] = int(f['admitted'].sum())
            row['rejects'] = {int(k): int(v) for k, v in acc['reject'].value_counts().items()}
            fa = f[f['action'] == 2]
            row['lh_in_weapon_cm'] = dict(with_p50=round(float(fa['lh_rel_cm'].median()), 2), with_max=round(float(fa['lh_rel_cm'].max()), 2),
                                          without_p50=round(float(fa['lh_rel_base_cm'].median()), 2),
                                          without_max=round(float(fa['lh_rel_base_cm'].max()), 2))
            row['rh_in_weapon_cm_max'] = round(float(fa['rh_rel_cm'].max()), 2)
            row['cost_us'] = dict(p50=round(float(acc['cost_us'].median()), 1) if len(acc) else None,
                                  p95=round(float(acc['cost_us'].quantile(.95)), 1) if len(acc) else None,
                                  max=round(float(acc['cost_us'].max()), 1) if len(acc) else None)
            rows.append(row)
            wo, wi = row['without'], row['with']
            print('%-22s %-34s n=%3d | without sev %5.2f entry %5.1f exit %5.1f deg | with sev %5.2f entry %5.1f exit %5.1f deg | lh-in-weapon p50 %4.2f max %5.2f cm | cost p95 %s us'
                  % (x['key'], x['group'][6:40], row['frames'], wo['severity'], wo['entry_step_deg'], wo['exit_step_deg'], wi['severity'],
                     wi['entry_step_deg'], wi['exit_step_deg'], row['lh_in_weapon_cm']['with_p50'], row['lh_in_weapon_cm']['with_max'],
                     row['cost_us']['p95']), flush=True)
    summary = {}
    for g in sorted({r['group'] for r in rows}):
        rs = [r for r in rows if r['group'] == g]
        summary[g] = dict(episodes=len(rs),
                          without_median=round(float(np.median([r['without']['severity'] for r in rs])), 3),
                          with_median=round(float(np.median([r['with']['severity'] for r in rs])), 3),
                          with_max=round(float(np.max([r['with']['severity'] for r in rs])), 3),
                          with_big_steps=int(sum(r['with']['big_steps'] for r in rs)),
                          entry_exit_step_deg_with_max=float(max(max(r['with']['entry_step_deg'], r['with']['exit_step_deg']) for r in rs)),
                          entry_exit_step_deg_without_max=float(max(max(r['without']['entry_step_deg'], r['without']['exit_step_deg']) for r in rs)),
                          without_big_steps=int(sum(r['without']['big_steps'] for r in rs)))
        print('GROUP %-40s %s' % (g, summary[g]))
    json.dump(dict(owner=a.owner, summary=summary, episodes=rows), open(os.path.join(a.out, 'proof_%s.json' % a.owner), 'w'), indent=1)


if __name__ == '__main__':
    main()
