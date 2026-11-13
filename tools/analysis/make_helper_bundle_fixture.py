"""Builds the offline fixture for the player helper bundle test (BodyHelperBundleTest.cpp) from a recorded run.

Writes into --out-dir:
  frames.txt    merged-rig topology (hash, parent) and the recorded LOCAL records of chosen real frames (both stages)
  expected.txt  independent reference (Python evaluator + anchor transfer) of the committed helper LOCAL records per frame
  proxy.txt     one skin proxy point per helper (mean bone-local reference position of the vertices that helper dominates)
Inputs: the pose recording (.utpppose), the bundle's owned list, the deformer graph (through evaluate_body_deformer_graph).
Not shipped data: the fixture is derived from a local recording and stays in Build/test-data.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import evaluate_body_deformer_graph as E  # noqa: E402
import pose_helper_report as ph  # noqa: E402
from replay_body_skin import trs  # noqa: E402
from export_player_helper_bundle import fnv64  # noqa: E402

FRAMES = [  # label, local clock window start/end (middle frame is used)
    ('stand_unarmed', '16:14:48.0', '16:14:51.0'),
    ('stand_armed', '16:14:54.0', '16:14:55.0'),
    ('walk_armed', '16:15:04.0', '16:15:05.0'),
    ('aim_walk', '16:15:07.0', '16:15:08.0'),
    ('aim_stand', '16:15:10.0', '16:15:10.6'),
    ('walk_unarmed', '16:15:23.0', '16:15:24.0'),
]


def clock(txt):
    h, m, s = txt.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


def quat_from_matrix(R):
    a, b, c = R[0]
    d, e, f = R[1]
    g, hh, i = R[2]
    K = np.array([[a - e - i, b + d, c + g, hh - f], [b + d, e - a - i, f + hh, c - g],
                  [c + g, f + hh, i - a - e, d - b], [hh - f, c - g, d - b, a + e + i]]) / 3
    _, v = np.linalg.eigh(K)
    q = v[:, -1]
    return q / np.linalg.norm(q)


def decompose(m):
    t = m[:3, 3].copy()
    s = np.linalg.norm(m[:3, :3], axis=0)
    q = quat_from_matrix(m[:3, :3] / s)
    return t, q, s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose', required=True)
    ap.add_argument('--owned', required=True)
    ap.add_argument('--children', required=True, help='hex hashes of compensated children, one per line (from the bundle manifest)')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--skip-proxy', action='store_true')
    ap.add_argument('--body', default='man', choices=['man', 'woman'], help='deformer rig of the recording (default man)')
    ap.add_argument('--frames', help='text file "label start end" (local clock h:m:s) per line; default = the male windows above')
    ap.add_argument('--sweep', type=int, default=0, help='also write sweep.txt: N evenly spaced frames of the whole run (no expectations; scratch statistics only)')
    a = ap.parse_args()
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    owned = [x.strip() for x in Path(a.owned).read_text().splitlines() if x.strip() and not x.startswith('#')]
    children = [int(x, 16) for x in Path(a.children).read_text().split()]
    cfg = ph.Config()
    g = ph.load_groups([a.pose], cfg)[0]
    h = g['header']
    secs, vals = [], []
    for stream, ticks, sec, gidx, v in ph.iter_blocks(g):
        assert stream == 0
        secs.append(sec)
        vals.append(v.astype(np.float32))
    t = np.concatenate(secs)
    V = np.concatenate(vals)   # n, bones, 2, 12
    mapped = h.mapped()
    names = [h.names[i] for i in mapped]
    rig_idx = [int(h.game_index[i]) for i in mapped]
    hashes = [int(x) for x in h.hashes]
    parents = [int(p) for p in h.parents]
    ev = E.Evaluator(allow_rotation_bounce=True, body=a.body)
    windows = FRAMES
    if a.frames:
        windows = [tuple(l.split()) for l in Path(a.frames).read_text().splitlines() if l.strip() and not l.startswith('#')]
    core_names = [n for n in names if n in ev.coreNames and n in ev.names]
    name_hash = {n: fnv64(n) for n in names}
    for n in names:
        assert hashes[rig_idx[names.index(n)]] == name_hash[n], n
    sp3 = names.index('Spine3')
    with open(out / 'frames.txt', 'w') as f:
        f.write('UTPP_HELPER_FIXTURE 1\nRIG %d\n' % len(hashes))
        known = {fnv64(n): n for n in list(ev.names) + names}
        for i, (hh, p) in enumerate(zip(hashes, parents)):
            f.write('BONE %d %016X %d %s\n' % (i, hh, p, known.get(hh, '-')))
        chosen = []
        for label, a0, a1 in windows:
            lo, hi = clock(a0), clock(a1)
            ix = np.nonzero((t >= lo) & (t < hi))[0]
            if len(ix) == 0:
                raise SystemExit('empty window ' + label)
            k = int(ix[len(ix) // 2])
            chosen.append((label, k))
            for stage in (0, 1):
                f.write('FRAME %s.s%d %d\n' % (label, stage, len(names)))
                for b, n in enumerate(names):
                    f.write('R %d %s\n' % (rig_idx[b], ' '.join(repr(float(x)) for x in V[k, b, stage, :])))
    print('chosen', [(l, ph.clock(t[k])) for l, k in chosen])
    if a.sweep:
        with open(out / 'sweep.txt', 'w') as f:
            f.write('UTPP_HELPER_FIXTURE 1\nRIG %d\n' % len(hashes))
            for i, (hh, p) in enumerate(zip(hashes, parents)):
                f.write('BONE %d %016X %d %s\n' % (i, hh, p, known.get(hh, '-')))
            for k in np.linspace(0, len(t) - 1, a.sweep).astype(int):
                for stage in (0, 1):
                    f.write('FRAME f%d.s%d %d\n' % (k, stage, len(names)))
                    for b, n in enumerate(names):
                        f.write('R %d %s\n' % (rig_idx[b], ' '.join(repr(float(x)) for x in V[k, b, stage, :])))
    # reference
    merged_index = {hh: i for i, hh in enumerate(hashes)}
    owned_hash = {fnv64(n): n for n in owned}
    rec_pos = {rig_idx[b]: b for b in range(len(names))}
    with open(out / 'expected.txt', 'w') as f:
        for label, k in chosen:
            for stage in (0, 1):
                recs = {rig_idx[b]: V[k, b, stage, :] for b in range(len(names))}
                mats = {i: trs(r) for i, r in recs.items()}
                actual = {}
                for i in range(len(hashes)):
                    if i not in mats:
                        continue
                    p = parents[i]
                    if p >= 0 and p not in actual:
                        continue
                    actual[i] = (actual[p] if p >= 0 else np.eye(4)) @ mats[i]
                core = {n: mats[merged_index[name_hash[n]]] for n in core_names if n in ev.names}
                M = ev.run(core)
                sp3_i = merged_index[name_hash['Spine3']]
                xfer = actual[sp3_i] @ np.linalg.inv(M['Spine3'])
                desired = {}
                for hh, n in owned_hash.items():
                    desired[merged_index[hh]] = xfer @ M[n]
                out_idx = []
                for i in range(len(hashes)):  # patch plan order: parents first, owned + boundary children
                    p = parents[i]
                    if i in desired or (p in desired and hashes[i] in children):
                        out_idx.append(i)
                for i in out_idx:
                    if i not in desired:
                        desired[i] = actual[i]
                for i in out_idx:
                    p = parents[i]
                    par = desired[p] if p in desired else actual[p]
                    loc = np.linalg.inv(par) @ desired[i]
                    tt, qq, ss = decompose(loc)
                    f.write('E %s.s%d %016X %s %s %s\n' % (label, stage, hashes[i], ' '.join(repr(float(x)) for x in tt),
                                                          ' '.join(repr(float(x)) for x in qq), ' '.join(repr(float(x)) for x in ss)))
    # proxy points
    if not a.skip_proxy:
        import measure_body_deformation as MBD
        rig = MBD.Rig(a.body)
        SK = MBD.SK
        acc = {}
        for part, (jn, pre) in zip(rig.parts, rig.binds):
            js, ws, pos = part['js'], part['ws'], part['pos']
            dom = js[np.arange(len(js)), ws.argmax(1)]
            for vi in range(len(pos)):
                name = part['jn'][dom[vi]]
                if name not in owned_hash.values():
                    continue
                k = jn.index(name)
                q = (SK.CI @ (pre[k] @ np.r_[pos[vi], 1.0]))[:3]
                acc.setdefault(name, []).append(q)
        with open(out / 'proxy.txt', 'w') as f:
            for n in sorted(acc):
                q = np.mean(acc[n], axis=0)
                f.write('P %016X %s %d\n' % (fnv64(n), ' '.join(repr(float(x)) for x in q), len(acc[n])))
    print('fixture written to', out)


if __name__ == '__main__':
    main()
