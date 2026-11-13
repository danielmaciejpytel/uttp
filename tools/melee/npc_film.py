"""Offline preview of the NPC-based fists clips (npc1, npc_build.py) next to the current mb7 clips: a 1x side-by-side film and contact sheets.

python -I npc_film.py film  [--rig woman] [--res 640,720] [--out <mp4>]   left = current (mb7 v4; vanilla where mb7 has no clip), right = npc1
python -I npc_film.py sheet [--rig woman] [--out <png>]                   key frames, per clip one row current + one row new
python -I npc_film.py worst [--rig woman] [--out <png>]                   per new clip: lowest gate-margin frame + closest fist-to-head frame
Blender renders straight to video (npc_film_render.py, MKV intermediates in Build\\work\\melee-npc-20261109\\_tmp\\film); the mp4 / png
go to Media\\<ext>\\2026-11-09_melee-npc-preview.  Both variants get the same capsule path (npc_build.capsule_path, translation only) and a
world-fixed ground grid.  Chained attacks: the next clip starts at the recover start of the previous one with a 0.15 s crossfade.
"""
import json
import os
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_common as MC  # noqa: E402
import npc_build as NB  # noqa: E402
import npc_retarget as NR  # noqa: E402

OUT = NB.OUT
CLIPS = os.path.join(OUT, 'clips')
MB7 = os.path.join(MC.ROOT, 'Build', 'work', 'melee-export-20261109', 'input-mb7-frozen', 'clips')
MEDIA_MP4 = os.path.join(MC.ROOT, 'Media', 'mp4', '2026-11-09_melee-npc-preview')
MEDIA_PNG = os.path.join(MC.ROOT, 'Media', 'png', '2026-11-09_melee-npc-preview')
TMP = os.path.join(OUT, '_tmp', 'film')
XF = 5                     # crossfade 0.15 s at 30 fps (4.5 frames)
COL = {'cur': '0.30,0.50,0.85', 'new': '0.25,0.62,0.45'}
SHORT = lambda c: c.replace('combo_attack_', 'combo').replace('final_attack_', 'final').replace('heavy_attack_', 'heavy').replace('melee_', '')


def opt(a, k, d):
    return a[a.index(k) + 1] if k in a else d


_CACHE = {}


def load(variant, clip):
    """-> (W [T,73,4,4] capsule frame, names, parents, source tag)."""
    key = (variant, clip)
    if key not in _CACHE:
        if variant == 'new':
            z = np.load(os.path.join(CLIPS, '%s__%s.npc1.npz' % (NB.SET, clip)))
            _CACHE[key] = (z['W'].astype(float), list(z['names']), z['parents'], 'npc1')
        elif variant == 'van':          # the game's own first-person clip, no correction (same body / capsule / camera)
            d = MC.load_dump(MC.clip_path('pwa_fists', clip))
            z = np.load(os.path.join(CLIPS, 'pwa_fists__melee_idlepose.npc1.npz'))
            _CACHE[key] = (MC.world_all(d)[:, :73].astype(float), list(z['names']), z['parents'], 'vanilla')
        else:
            p = os.path.join(MB7, 'pwa_fists__%s.v4.npz' % clip)
            if os.path.exists(p):
                z = np.load(p)
                _CACHE[key] = (z['W'].astype(float), list(z['names']), z['parents'], 'mb7')
            else:
                d = MC.load_dump(MC.clip_path('pwa_fists', clip))
                z = np.load(os.path.join(CLIPS, 'pwa_fists__melee_idlepose.npc1.npz'))
                _CACHE[key] = (MC.world_all(d)[:, :73].astype(float), list(z['names']), z['parents'], 'vanilla')
    return _CACHE[key]


def to_local(W, par):
    L = W.copy()
    for j, p in enumerate(par):
        if p >= 0:
            L[:, j] = np.linalg.inv(W[:, p]) @ W[:, j]
    return L


def fk73(L, par):
    W = L.copy()
    for j, p in enumerate(par):
        assert p < j
        if p >= 0:
            W[:, j] = W[:, p] @ L[:, j]
    return W


def blend(La, Lb, w):
    """per-frame weight w [k]: slerp-like (normalised lerp) of the local rotations, lerp of the local translations."""
    sh = La.shape[:-2]
    wq = w.reshape((-1,) + (1,) * len(sh))            # [k,1,1] against [k,J,4] / [k,J,3]
    qa = R.from_matrix(La[..., :3, :3].reshape(-1, 3, 3)).as_quat().reshape(sh + (4,))
    qb = R.from_matrix(Lb[..., :3, :3].reshape(-1, 3, 3)).as_quat().reshape(sh + (4,))
    qb = np.where(np.sum(qa * qb, -1, keepdims=True) < 0, -qb, qb)
    q = (1 - wq) * qa + wq * qb
    q /= np.linalg.norm(q, axis=-1, keepdims=True)
    out = La.copy()
    out[..., :3, :3] = R.from_quat(q.reshape(-1, 4)).as_matrix().reshape(sh + (3, 3))
    out[..., :3, 3] = (1 - wq) * La[..., :3, 3] + wq * Lb[..., :3, 3]
    return out


def capsule(clip, frames):
    spec = NB.SPEC.get(clip, {})
    T = len(load('new', clip)[0]) if clip in NB.SPEC else 1
    c = NB.capsule_path(spec.get('kind', 'static'), T, spec.get('lunge'), 0)[0] if clip in NB.SPEC else np.zeros((1, 3))
    c = c[np.minimum(frames, len(c) - 1)]
    return c - c[0]


GUARD = {'new': 'guard_idle_loop', 'cur': 'melee_idlepose', 'van': 'melee_idlepose'}     # mb7 has no guard idle: the game's own idle pose plays


def plan():
    """(clip, n frames or None = all, overlap with the previous entry, mode)  mode: 'all' | 'chain' (cut at recover + XF) | 'cycle'."""
    g = lambda n: ('guard', n, XF, 'cycle')
    return [('melee_equip', None, 0, 'all'), g(60),
            ('combo_attack_01', None, XF, 'chain'), ('combo_attack_02', None, XF, 'chain'), ('combo_attack_03', None, XF, 'chain'),
            ('final_attack_03', None, XF, 'all'), g(20), ('combo_attack_04', None, XF, 'all'), g(12), ('combo_attack_05', None, XF, 'all'), g(15),
            ('hold_start_01', None, XF, 'all'), ('hold_loop_01', 30, 1, 'cycle'), ('heavy_attack_01', None, 1, 'all'), g(20),
            ('block_start', None, XF, 'all'), ('block', 30, 1, 'cycle'), ('block_end', None, 1, 'all'), g(20), ('melee_unequip', None, XF, 'all')]


def entry_frames(variant, clip, n, mode):
    W = load(variant, clip)[0]
    T = len(W)
    if mode == 'all':
        return np.arange(T)
    if mode == 'chain':
        ph = NB.vanilla_impact(clip)[3]
        rec = ph.get('recover', (14, T - 1))[0]
        return np.arange(min(T, rec + XF))
    per = T - 1 if T > 2 and np.abs(W[0] - W[-1]).max() < 1e-4 else T      # closed loops: last frame == first
    return np.arange(n) % max(per, 1)


def clean_name(clip):
    return 'guard' if clip in GUARD.values() else clip.replace('melee_', '').replace('_', ' ')


def timeline(variant, clean=False):
    Ls = C = None
    lab, par = [], None
    for clip, n, ov, mode in plan():
        cname = GUARD[variant] if clip == 'guard' else clip
        W, names, par_, src = load(variant, cname)
        par = par_
        fr = entry_frames(variant, cname, n, mode)
        L = to_local(W, par)[fr]
        cs = capsule(clip, fr) if mode != 'cycle' else np.zeros((len(fr), 3))
        tags = ['%s f%d%s' % (SHORT(cname), f, '' if src in ('mb7', 'npc1') else ' (%s)' % src) for f in fr]
        if clean:                     # presentation film: the clip name only
            tags = [clean_name(cname)] * len(fr)
        if Ls is None:
            Ls, C, lab = L, cs, tags
            continue
        k = max(1, min(ov, len(L), len(Ls)))
        w = NB.sstep((np.arange(k) + 1.0) / (k + 1.0))
        cin = cs + C[len(C) - k]
        Lo = blend(Ls[-k:], L[:k], w)
        Co = (1 - w)[:, None] * C[-k:] + w[:, None] * cin[:k]
        Ls = np.concatenate([Ls[:-k], Lo, L[k:]])
        C = np.concatenate([C[:-k], Co, cin[k:]])
        lab = lab[:-k] + [tags[i] if w[i] >= 0.5 else lab[len(lab) - k + i] for i in range(k)] + tags[k:]
    W = fk73(Ls, par)
    W[:, :, :3, 3] += C[:, None]
    return W, lab, C


def render(W, labels, title, color, name, res, follow, extra=()):
    os.makedirs(TMP, exist_ok=True)
    base = os.path.join(TMP, name)
    NR.skin_W(W, NB.P.ref['rest'], NB.RIG, base)
    lj = base + '.labels.json'
    json.dump(labels, open(lj, 'w', encoding='utf-8'))
    mkv = base + '.mkv'
    cmd = [NR.BLENDER, '-b', '--factory-startup', '-P', os.path.join(HERE, 'npc_film_render.py'), '--', base, mkv, lj, '--res', res,
           '--follow', str(follow), '--title', title, '--color', color] + list(extra)
    p = subprocess.run(cmd, capture_output=True, text=True)
    if 'RENDERED' not in p.stdout:
        raise RuntimeError('blender failed: ' + (p.stdout + p.stderr)[-1500:])
    return mkv


def ffmpeg(args):
    p = subprocess.run([NR.FFMPEG, '-y', '-loglevel', 'error'] + args, capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError('ffmpeg failed: ' + p.stderr[-800:])


def film(a):
    res = opt(a, '--res', '640,720')
    out = os.path.abspath(opt(a, '--out', os.path.join(MEDIA_MP4, 'fists_npc1_vs_mb7_1x.mp4')))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    tl = {v: timeline(v) for v in ('cur', 'new')}
    T = min(len(tl[v][0]) for v in tl)
    assert np.allclose(tl['cur'][2][:T], tl['new'][2][:T]), 'capsule paths differ'
    root = {v: round(float(np.abs(tl[v][0][:T, 0, :3, 3] - tl[v][2][:T]).max() * 100), 2) for v in tl}     # Root vs capsule (0 = no baked root motion)
    mk = {}
    for v, title in (('cur', 'CURRENT  mb7 (fists, Female V)'), ('new', 'NEW  NPC-based npc1 (fists, Female V)')):
        mk[v] = render(tl[v][0][:T], tl[v][1][:T], title, COL[v], 'film_' + v, res, 15)
    ffmpeg(['-i', mk['cur'], '-i', mk['new'], '-filter_complex', '[0:v][1:v]hstack=inputs=2,format=yuv420p[v]', '-map', '[v]', '-r', '30',
            '-c:v', 'libx264', '-crf', '20', '-preset', 'slow', '-movflags', '+faststart', out])
    info = dict(out=out, frames=T, seconds=round(T / 30.0, 2), bytes=os.path.getsize(out), root_offset_from_capsule_max_cm=root,
                capsule_end_m=np.round(tl['new'][2][T - 1], 3).tolist(), sequence=[p[0] for p in plan()])
    MC.jdump(info, os.path.join(OUT, 'film_info.json'))
    print('FILM', json.dumps(info))


def stills(items, name, title, out_png, cols=6, res='300,360'):
    Ws, labels = [], []
    for variant, clip, f in items:
        W = load(variant, clip)[0]
        f = min(int(f), len(W) - 1)
        Ws.append(W[f:f + 1])
        labels.append('%s %s f%d' % ('NEW' if variant == 'new' else 'CUR', SHORT(clip), f))
    mkv = render(np.concatenate(Ws), labels, title, '0.45,0.55,0.70', name, res, 1)
    rows = (len(items) + cols - 1) // cols
    ffmpeg(['-i', mkv, '-vf', 'tile=%dx%d:padding=2:color=white' % (cols, rows), '-frames:v', '1', out_png])
    print('SHEET', out_png, len(items), os.path.getsize(out_png))


def sheet(a):
    out = os.path.abspath(opt(a, '--out', os.path.join(MEDIA_PNG, 'fists_npc1_vs_mb7_keyframes.png')))
    items = []
    for clip in ('combo_attack_01', 'combo_attack_02', 'combo_attack_03', 'final_attack_03', 'heavy_attack_01', 'block_start', 'melee_equip'):
        T = len(load('new', clip)[0])
        if clip in NB.SPEC and NB.SPEC[clip]['kind'] in ('attack', 'heavy'):
            imp = NB.vanilla_impact(clip)[1]
            frs = [0, max(0, imp - 3), imp, imp + 4, min(T - 1, imp + 14), T - 1]
        else:
            frs = list(np.linspace(0, T - 1, 6).round().astype(int))
        for v in ('cur', 'new'):
            items += [(v, clip, f) for f in frs]
    stills(items, 'sheet_keyframes', 'rows: CUR = current mb7 / NEW = NPC-based npc1', out)


def gate_margin(W):
    """per frame: lowest normalised margin min(f-lo, hi-f)/(hi-lo) over the base-gate features that can move (the hinge-deviation
    features knee_dev_* / elbow_dev_* sit on their bound by construction of the hinge IK and are skipped) -> (margin [T], feature [T])."""
    g = NB.GB.frame_gate(W[:, :73], NB.RG, steps=True, al=NB.AL)
    T = len(W)
    best, name = np.full(T, np.inf), np.array([''] * T, dtype=object)
    for k, v in g['feat'].items():
        if k not in NB.AL or 'knee_dev' in k or 'elbow_dev' in k or 'shin_roll' in k:
            continue
        lo, hi = (float(x) for x in NB.AL[k])
        f = np.asarray(v, float).reshape(-1)
        if len(f) != T or not (np.isfinite(lo) and np.isfinite(hi)) or hi - lo <= 0:
            continue
        m = np.minimum(f - lo, hi - f) / (hi - lo)
        if np.median(np.abs(m)) < 1e-3:       # sits on its bound by construction (IK hinge) on most frames
            continue
        upd = m < best
        best = np.where(upd, m, best)
        name = np.where(upd, k, name)
    return best, name


def worst(a):
    """per new clip: the lowest moving-feature gate margin frame, the closest fist-to-head frame and, for attacks / heavies, frames 1-2
    (the fade-in of the anatomy fixes from the guard / hold pose)."""
    global CLIPS, OUT
    out = os.path.abspath(opt(a, '--out', os.path.join(MEDIA_PNG, 'fists_npc1_worst_frames.png')))
    if '--clips-root' in a:                # another build folder (e.g. the male build): <root>\clips, notes json written there
        OUT = os.path.abspath(opt(a, '--clips-root', OUT))
        CLIPS = os.path.join(OUT, 'clips')
    only = opt(a, '--only', '')            # 'stage2' = NB.ORDER2, or a comma list; default every clip of NB.ORDER
    names = NB.ORDER2 if only == 'stage2' else (only.split(',') if only else NB.ORDER)
    items, notes = [], {}
    for clip in names:
        z = np.load(os.path.join(CLIPS, '%s__%s.npc1.npz' % (NB.SET, clip)))
        W = NB.fk(z['lt'].astype(float), z['lr'].astype(float))
        m, feat = gate_margin(W)
        fm = int(np.argmin(m))
        fc, hc = NB.fist_centers(W), NB.head_center(W)
        dh = np.min([np.linalg.norm(fc[s] - hc, axis=1) for s in fc], axis=0)
        fh = int(np.argmin(dh))
        notes[clip] = dict(min_margin_frame=fm, min_margin=round(float(m[fm]), 3), feature=str(feat[fm]),
                           closest_fist_head_frame=fh, fist_head_cm=round(float(dh[fh] * 100), 1))
        frs = [fm, fh if fh != fm else len(W) // 2]
        if NB.SPEC[clip]['kind'] in ('attack', 'heavy'):
            frs += [1, 2]
        items += [('new', clip, f) for f in frs]
    stills(items, 'sheet_worst' + ('_' + only.replace(',', '_')[:40] if only else ''), 'npc1: lowest gate margin / closest fist-head / attacks f1-f2', out)
    MC.jdump(notes, os.path.join(OUT, 'worst_frames%s.json' % ('_' + only.replace(',', '_')[:40] if only else '')))
    print(json.dumps(notes))


def gamefilm(a):
    """presentation film: left = the game's own pwa_fists clips (vanilla for EVERY clip, no correction), right = npc1; same sequence,
    timing, capsule and camera as the v4 film; titles only ("Game Animations" / "Ultimate Third Person Project"), clip names small."""
    res = opt(a, '--res', '640,720')
    out = os.path.abspath(opt(a, '--out', os.path.join(MEDIA_MP4, 'fists_game_vs_utpp_1x.mp4')))
    tl = {v: timeline(v, clean=True) for v in ('van', 'new')}
    T = min(len(tl[v][0]) for v in tl)
    assert len(tl['van'][0]) == len(tl['new'][0]), 'timelines differ in length'
    assert np.allclose(tl['van'][2][:T], tl['new'][2][:T]), 'capsule paths differ'
    ex = ['--title-size', '0.075', '--title-align', 'center', '--label-size', '0.04']
    mk = {}
    for v, title in (('van', 'Game Animations'), ('new', 'Ultimate Third Person Project')):
        mk[v] = render(tl[v][0][:T], tl[v][1][:T], title, COL['cur' if v == 'van' else 'new'], 'gamefilm_' + v, res, 15, ex)
    ffmpeg(['-i', mk['van'], '-i', mk['new'], '-filter_complex', '[0:v][1:v]hstack=inputs=2,format=yuv420p[v]', '-map', '[v]', '-r', '30',
            '-c:v', 'libx264', '-crf', '20', '-preset', 'slow', '-movflags', '+faststart', out])
    srcs = sorted({load('van', GUARD['van'] if c == 'guard' else c)[3] for c, *_ in plan()})
    info = dict(out=out, frames=T, seconds=round(T / 30.0, 2), bytes=os.path.getsize(out), left_sources=srcs, sequence=[p[0] for p in plan()])
    MC.jdump(info, os.path.join(OUT, 'film_info_game_vs_utpp.json'))
    print('FILM', json.dumps(info))


if __name__ == '__main__':
    a = sys.argv[1:]
    {'film': film, 'sheet': sheet, 'worst': worst, 'gamefilm': gamefilm}[a[0]](a)
