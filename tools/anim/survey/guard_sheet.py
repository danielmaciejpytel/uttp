"""UTPP anti-deformation guard v2 - worst-frame sheet: vanilla vs v1 vs v2, 3/4 front and side (offline software render).

  python -I guard_sheet.py --out <png> [--n_woman 10] [--n_man 4]
Rows = per state the clip with the largest vanilla skin excess (eval_<rig>_p*.json), its worst skinned frame; the skinned vertex
caches come from guard_eval.py (Build\\work\\anim-guard-v2-20261109\\skin\\<rig>\\{v,g1,g2}__<clip>.skin.*).
"""
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402
import survey_sheet as SS  # noqa: E402   (tile(); reads --out)

ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
WORK = os.path.join(ROOT, 'Build', 'work', 'anim-guard-v2-20261109')
SURVEY = os.path.join(ROOT, 'Build', 'work', 'anim-survey-20261109')
OUT = SS.OUT
TS = (460, 600) if '--big' in sys.argv else (230, 300)          # --big: detail sheet (larger tiles)
N = {'woman': int(sys.argv[sys.argv.index('--n_woman') + 1]) if '--n_woman' in sys.argv else 10,
     'man': int(sys.argv[sys.argv.index('--n_man') + 1]) if '--n_man' in sys.argv else 4}


def frames_around(top, T):
    fr = []
    for f in top:
        for g in (f - 1, f, f + 1):
            if 0 <= g < T and g not in fr:
                fr.append(g)
    return fr


def load(rig, tag, k):
    sk = os.path.join(WORK, 'skin', rig)
    z = np.load(os.path.join(sk, tag + '.skin.npz'))
    nv = len(z['rest'])
    pc = np.fromfile(os.path.join(sk, tag + '.skin.pc2'), dtype='<f4', offset=32).reshape(-1, nv, 3)
    return pc[k].astype(float), z['faces'], z['rest']


def verdict(r, tag):
    if '--rig' not in sys.argv:
        sys.argv += ['--rig', 'woman']           # the verdict rule is rig independent; the import chain needs a rig
    import guard_eval as GE  # noqa: E402
    return GE.verdict(r, tag)


def main():
    rows = []
    for rig in ('woman', 'man'):
        met = json.load(open(os.path.join(SURVEY, 'metrics_%s.json' % rig), encoding='utf-8'))
        res = {}
        for f in sorted(glob.glob(os.path.join(WORK, 'eval_%s_p*.json' % rig))):
            res.update(json.load(open(f, encoding='utf-8')))
        best = {}
        for k, r in res.items():
            if 'error' in r or 'sheet_frame' not in r:
                continue
            s = r['state_key']
            if s not in best or r['skin_v'] > best[s][1]['skin_v']:
                best[s] = (k, r)
        pick = sorted(best.values(), key=lambda kr: -kr[1]['skin_v'])[:N[rig]]
        for k, r in pick:
            m = met[k]
            T = r['frames']
            top = list(m.get('top_frames') or [m['worst_frame']])[:2]
            fr = frames_around(top + ([int(r['v2']['hand_at'])] if r['v2']['hand_cm'] > 2.0 else []), T)
            kk = fr.index(r['sheet_frame'])
            tag = k.replace('/', '__')
            P0, F, rest = load(rig, 'v__' + tag, kk)
            P1, _, _ = load(rig, 'g1__' + tag, kk)
            P2, _, _ = load(rig, 'g2__' + tag, kk)
            up = int(np.argmax(np.ptp(rest, axis=0)))
            cv, c1, c2 = (0.85, 0.55, 0.5), (0.85, 0.78, 0.45), (0.5, 0.7, 0.85)
            ims = [SS.tile(P0, F, up, 150, 'vanilla 3/4 front', cv, size=TS), SS.tile(P1, F, up, 150, 'v1 3/4 front', c1, size=TS),
                   SS.tile(P2, F, up, 150, 'v2 3/4 front', c2, size=TS), SS.tile(P0, F, up, 90, 'vanilla side', cv, size=TS),
                   SS.tile(P1, F, up, 90, 'v1 side', c1, size=TS), SS.tile(P2, F, up, 90, 'v2 side', c2, size=TS)]
            h = max(i.height for i in ims)
            w = sum(i.width for i in ims)
            row = Image.new('RGB', (w, h + 30), 'white')
            x = 0
            for i in ims:
                row.paste(i, (x, 30))
                x += i.width
            d = ImageDraw.Draw(row)
            g1, g2 = verdict(r, 'v1'), verdict(r, 'v2')
            d.text((4, 2), '%s | %s | %s  f%d' % (rig, r['state_key'], k[:60], r['sheet_frame']), fill='black')
            d.text((4, 15), 'v1 %s -> v2 %s   skin %.2f / %.2f / %.2f   hand v1 %.1f v2 %.1f cm   pop_chain v1 %.0f v2 %.0f   %s' % (
                g1[0], g2[0], r['skin_v'], r['skin_v1'], r['skin_v2'], r['v1']['hand_cm'], r['v2']['hand_cm'], r['v1']['pop_chain'],
                r['v2']['pop_chain'], '; '.join(g2[1])[:70]), fill='black')
            rows.append(row)
            print('row', rig, k, flush=True)
    W = max(r.width for r in rows)
    sheet = Image.new('RGB', (W * 2, (sum(r.height for r in rows) + 1) // 2 + max(r.height for r in rows) + 30), 'white')
    d = ImageDraw.Draw(sheet)
    d.text((6, 6), 'UTPP anti-deformation guard v2 (offline) 2026-11-09 - worst skinned frame per state: vanilla FPP clip shown in TPP (red), '
                   'v1 prototype (yellow), v2 prototype (blue). Software render of the skinned offline mesh, not a game capture.', fill='black')
    y = [30, 30]
    for i, r in enumerate(rows):
        c = 0 if y[0] <= y[1] else 1
        sheet.paste(r, (c * W, y[c]))
        y[c] += r.height
    sheet = sheet.crop((0, 0, W * 2, max(y)))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    sheet.save(OUT)
    print('WROTE', OUT, sheet.size)


if __name__ == '__main__':
    main()
