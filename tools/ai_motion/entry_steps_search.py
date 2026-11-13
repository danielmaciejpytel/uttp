"""Candidate search for the car ENTRY stepping fix (2026-11-06): per shipped row the gliding SEGMENTS of the planted feet (clusters of frames with > 1 cm/frame ankle speed in a planted run,
entry_steps_gen.py) get a short MotionBricks span each (N = 4..8 tokens, c = first glide frame - 3, g covers the last glide frame; several seeds, entry_steps_run.sh); the candidates are scored on
the WINDOW c-2..g+2 with the clip's own foot labels, floor consistency and hips deviation. Kimodo-env python.

  python -I entry_steps_search.py <outdir> [--rows a,b,..] [--jobs 10] [--seeds 10,11,12,13,14,15] [--extra "--pin-glide 0.3"] [--vmin 1.0]
Writes <outdir>/<row>/s<k>_c<c>_N<N>_s<seed>/ (run dir of entry_steps_run.sh), <outdir>/results.json; prints the best candidate per segment.
Window score s = own max + 3 * own mean (cm / frame, planted feet) + 0.3 * max(0, floor dev - 3) + 0.4 * max(0, hips dev - 3); rejected: floor dev > 5 cm, hips dev > 6 cm (MotionBricks assumes a flat floor
and a free body: a long span over the sill / door opening moves the pelvis 15-35 cm, which the door / roof gates would not accept), window max not below 70 % of the before value."""
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import entry_steps_gen as E  # noqa: E402

ROOT = E.ROOT
BASE = ROOT + r'\Build\work\vehentry-mb-20261106\base'
BASH = os.environ.get('UTPP_BASH') or os.path.join(os.environ.get('ProgramFiles', ''), 'Git', 'bin', 'bash.exe')


def before_info(row):
    d = np.load(os.path.join(BASE, row, 'cur.dump.npz'), allow_pickle=True)
    pl = json.load(open(os.path.join(BASE, row, 'plants.json')))
    names = [str(x) for x in d['names']]
    return d, pl, names, E.plant_summary(d['W'], names, pl)


def win_stats(r, lo, hi):
    """max / mean ankle speed of the planted (extended) frames of both feet inside lo..hi"""
    v = []
    for s in E.SIDES:
        sp, em = r[s]['sp'], r[s]['ext']
        v += [sp[f] for f in range(max(lo, 1), min(hi, len(sp) - 1) + 1) if em[f] and em[f - 1]]
    return (float(max(v)) if v else 0.0), (float(np.mean(v)) if v else 0.0)


def segments(row, vmin, merge=5):
    d, pl, names, r = before_info(row)
    fr = sorted({f for s in E.SIDES for f in r[s]['over'] if f >= 4 and r[s]['sp'][f] > vmin})
    segs, cur = [], None
    for f in fr:
        if cur and f - cur[1] <= merge:
            cur[1] = f
        else:
            cur = [f, f]
            segs.append(cur)
    return [tuple(x) for x in segs], len(d['W'])


def metrics(row, steps, after):
    d0, pl, names, r0 = before_info(row)
    d1 = np.load(after, allow_pickle=True)
    st = np.load(steps, allow_pickle=True)
    ix = {n: i for i, n in enumerate(names)}
    r1 = E.plant_summary(d1['W'], names, E.after_plants(pl, st))
    c, g = int(st['c']), int(st['g'])
    n = g - c + 1
    from exit_steps_gen import planted_mask
    fz = 0.0
    for s in E.SIDES:
        pm = planted_mask(np.asarray(st['foot_contacts']), s, n)
        a0 = d0['W'][c:g + 1, ix[s + 'Foot'], 2, 3]
        a1 = d1['W'][c:g + 1, ix[s + 'Foot'], 2, 3]
        if pm.any():
            fz = max(fz, float(np.abs(a1 - a0)[pm].max()) * 100)
    h0, h1 = d0['W'][:, ix['Hips'], :3, 3], d1['W'][:, ix['Hips'], :3, 3]
    hd = float(np.linalg.norm((h1 - h0)[c:g + 1], axis=1).max() * 100)
    b0, m0 = win_stats(r0, c - 2, g + 2)
    b1, m1 = win_stats(r1, c - 2, g + 2)
    sc = b1 + 3 * m1 + 0.3 * max(0, fz - 3) + 0.4 * max(0, hd - 3)
    ok = fz <= 5 and hd <= 6 and b1 <= 0.7 * b0
    return dict(score=round(sc, 3), ok=bool(ok), before_max=round(b0, 2), before_mean=round(m0, 3), max=round(b1, 2), mean=round(m1, 3), floor_dev_cm=round(fz, 1), hips_cm=round(hd, 1), c=c, g=g)


def run_one(args):
    row, out, k, c, N, seed, extra = args
    rdir = os.path.join(out, row, 's%d_c%d_N%d_s%d' % (k, c, N, seed))
    rel = os.path.relpath(rdir, ROOT).replace('\\', '/')
    p = subprocess.run([BASH, HERE.replace('\\', '/') + '/entry_steps_run.sh', row, rel, str(c), str(N), '4', str(seed)] + extra, capture_output=True, text=True, timeout=600)
    steps = os.path.join(rdir, 'steps_c%d_N%d_s%d.steps.npz' % (c, N, seed))
    after = os.path.join(rdir, 'after_c%d_N%d_s%d.dump.npz' % (c, N, seed))
    if not (os.path.exists(steps) and os.path.exists(after)):
        return row, k, rdir, dict(error=(p.stdout + p.stderr)[-300:], ok=False, score=1e9)
    return row, k, rdir, metrics(row, steps, after)


def main(a):
    out = a[0]
    opt = lambda k, dflt: a[a.index(k) + 1] if k in a else dflt
    rows = opt('--rows', '').split(',') if '--rows' in a else [n for n, _ in E.rows()]
    jobs = int(opt('--jobs', '10'))
    seeds = [int(x) for x in opt('--seeds', '10,11,12,13,14,15').split(',')]
    extra = opt('--extra', '--pin-glide 0.3').split()
    vmin = float(opt('--vmin', '1.0'))
    todo, segmap = [], {}
    for row in rows:
        row = row if row.startswith('utpp_car_') else 'utpp_car_' + row
        segs, T = segments(row, vmin)
        segmap[row] = segs
        for k, (lo, hi) in enumerate(segs):
            for c in sorted({max(3, lo - 3), max(3, lo - 4)}):
                for N in range(6, 11):
                    g = c + 4 * N - 7
                    if g + 4 > T - 1:
                        break
                    if g >= hi + 1:
                        for NN in (N, N + 1):
                            if c + 4 * NN - 7 + 4 <= T - 1:
                                todo += [(row, out, k, c, NN, sd, extra) for sd in seeds]
                        break
    print('segments %s | candidates %d' % ({r[9:]: s for r, s in segmap.items()}, len(todo)), flush=True)
    res = {}
    with ThreadPoolExecutor(jobs) as ex:
        for row, k, rdir, m in ex.map(run_one, todo):
            res.setdefault(row, {}).setdefault(k, []).append(dict(dir=rdir, **m))
    json.dump(dict(segments={r: [list(x) for x in s] for r, s in segmap.items()}, results=res), open(os.path.join(out, 'results.json'), 'w'), indent=1)
    for row in sorted(res):
        for k in sorted(res[row]):
            good = sorted([x for x in res[row][k] if x.get('ok')], key=lambda x: x['score'])
            seg = segmap[row][k]
            print('%-30s seg %s | %d/%d ok | %s' % (row[9:], seg, len(good), len(res[row][k]), ('%s s=%.2f win max %.2f->%.2f mean %.3f->%.3f floor %.1f hips %.1f' % (
                os.path.basename(good[0]['dir']), good[0]['score'], good[0]['before_max'], good[0]['max'], good[0]['before_mean'], good[0]['mean'], good[0]['floor_dev_cm'], good[0]['hips_cm'])) if good else 'NONE'))


if __name__ == '__main__':
    main(sys.argv[1:])
