"""Dual-wield pilot media (2026-11-09): ONE 1x comparison film and ONE worst-frames sheet of the dw1 clips (dw_build.py).

  python -I dw_film.py film   [--res 640,720]   -> Media\\mp4\\2026-11-09_dualwield-npc\\dualwield_game_vs_utpp_1x.mp4
  python -I dw_film.py worst  [--res 300,360]   -> Media\\png\\2026-11-09_dualwield-npc\\dualwield_dw1_worst_frames.png
Film: left = the game's own Female V dual-wield clips (Lexington set, no correction: equip, idlepose_shoulder, the 3-frame recoil additive
add_recoil_single_shot_shoulder on one arm chain per shot, reload), right = the dw1 clips; same sequence and timing on both sides:
equip -> aim 30 -> fire R / L / R / L -> aim 30 -> reload -> aim 30, crossfades of XF frames (local slerp); titles only ("Game Animations" /
"Ultimate Third Person Project"), no labels.  Guns = the gate proxy boxes (slide + grip) on the Weapon bones, dark.  Camera front-right 3/4
(both arms and guns visible; the in-game camera is behind-right).
Worst sheet: per moving dw1 clip the frames with the lowest base-gate margin, the smallest gun-gun / gun-body / gun-head clearance, the
smallest hand-head distance and the first / last frame; two rows of views (front-right, front-left).
Intermediates (skin caches, MKV) in Build\\work\\dualwield-npc-20261109\\_tmp\\film; no image files except the one sheet.
"""
import json
import os
import struct
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dw_build as B  # noqa: E402

DC, NB, MC, NR, IX, P = B.DC, B.NB, B.MC, B.NR, B.IX, B.P
TMP = os.path.join(DC.OUT, '_tmp', 'film')
XF = 5
COL = {'van': '0.30,0.50,0.85', 'new': '0.25,0.62,0.45'}
CAM = '1.6,1.9,1.4'
CAM2 = '-1.6,1.9,1.4'
SEQ = [('equip', 0), ('hold', 30), ('fire_right', 0), ('fire_left', 0), ('fire_right', 0), ('fire_left', 0), ('hold', 30), ('reload', 0),
       ('hold', 30)]
GUN_MESH = [((-0.014, -0.05, -0.015), (0.014, 0.16, 0.02)), B.GUN_BOXES[2]]     # slide (front + rear proxy boxes) and grip


def opt(a, k, d):
    return a[a.index(k) + 1] if k in a else d


def new_clip(name):
    z = np.load(os.path.join(B.CLIPS, '%s__%s.%s.npz' % (B.SET, name, B.SUFFIX)))
    return z['lt'].astype(float), z['lr'].astype(float)


def van_fire(side):
    """the game's per-hand shot: idlepose_shoulder + the 3-frame additive (delta to its frame 0, local post-multiply) on that arm chain."""
    idl = B.vanilla('idlepose_shoulder')
    add = B.vanilla('add_recoil_single_shot_shoulder')
    T = B.FIRE_FRAMES
    lt, lr = np.repeat(idl['lt'][:1], T, 0), np.repeat(idl['lr'][:1], T, 0).copy()
    n = min(T, len(add['lr']))
    d = B.qmul(B.qinv(np.repeat(add['lr'][:1], n, 0)), add['lr'][:n])
    for nm in B.ARM[side]:
        lr[:n, IX[nm]] = B.qmul(lr[:n, IX[nm]], d[:, IX[nm]])
    return lt, lr


def entries(variant):
    if variant == 'new':
        loop = new_clip('aim_idle_loop')
        hold = lambda n: (loop[0][:n], loop[1][:n])
        get = new_clip
    else:
        idl = B.vanilla('idlepose_shoulder')
        hold = lambda n: (np.repeat(idl['lt'][:1], n, 0), np.repeat(idl['lr'][:1], n, 0))
        get = lambda c: van_fire('Right') if c == 'fire_right' else van_fire('Left') if c == 'fire_left' else (B.vanilla(c)['lt'], B.vanilla(c)['lr'])
    return [hold(n) if c == 'hold' else get(c) for c, n in SEQ]


def timeline(variant):
    Ls = Rs = None
    for lt, lr in entries(variant):
        lr = NR.S.qcontinuous(lr)
        if Ls is None:
            Ls, Rs = lt.copy(), lr.copy()
            continue
        k = max(1, min(XF, len(lt), len(Ls)))
        w = NB.sstep((np.arange(k) + 1.0) / (k + 1.0))
        bt = (1 - w)[:, None, None] * Ls[-k:] + w[:, None, None] * lt[:k]
        br = NB.qslerp(Rs[-k:], lr[:k], np.repeat(w[:, None], lr.shape[1], 1))
        Ls = np.concatenate([Ls[:-k], bt, lt[k:]])
        Rs = np.concatenate([Rs[:-k], br, lr[k:]])
    return NB.fk(Ls, NR.S.qcontinuous(Rs))


def box(lo, hi):
    c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])], float)
    f = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return c, f


def gun_cache(W, base):
    """both guns' proxy boxes in world space per frame -> <base>.npz (rest, faces) + <base>.pc2."""
    Vs, F, off = [], [], 0
    for s in DC.SIDES:
        M = W[:, IX[B.WPN[s]]]
        for lo, hi in GUN_MESH:
            c, f = box(lo, hi)
            Vs.append(np.einsum('tij,nj->tni', M[:, :3, :3], c) + M[:, None, :3, 3])
            F += [tuple(i + off for i in q) for q in f]
            off += len(c)
    V = np.concatenate(Vs, 1).astype('<f4')
    np.savez(base + '.npz', rest=V[0], faces=np.array(F, np.int32))
    with open(base + '.pc2', 'wb') as fh:
        fh.write(b'POINTCACHE2\0' + struct.pack('<iiffi', 1, V.shape[1], 0.0, 1.0, V.shape[0]))
        fh.write(V.tobytes())
    return base


def render(W, labels, title, color, name, res, follow, cam, extra=()):
    os.makedirs(TMP, exist_ok=True)
    base = os.path.join(TMP, name)
    NR.skin_W(W[:, :73], P.ref['rest'], NB.RIG, base)
    sk = np.load(base + '.npz')
    j = sk['joints']
    dev = float(np.abs(j - W[:, [IX[str(n)] for n in sk['joint_names']]][:, :, :3, 3]).max())     # by name (Hips, Head, hands, feet)
    gb = gun_cache(W, base + '_guns')
    lj = base + '.labels.json'
    json.dump(labels, open(lj, 'w', encoding='utf-8'))
    mkv = base + '.mkv'
    cmd = [NR.BLENDER, '-b', '--factory-startup', '-P', os.path.join(HERE, 'dw_film_render.py'), '--', base, mkv, lj, '--res', res,
           '--follow', str(follow), '--title', title, '--color', color, '--cam', cam, '--guns', gb] + list(extra)
    p = subprocess.run(cmd, capture_output=True, text=True)
    if 'RENDERED' not in p.stdout:
        raise RuntimeError('blender failed: ' + (p.stdout + p.stderr)[-1500:])
    print('render', name, 'skin joints vs W max dev m', round(dev, 5), 'joints shape', j.shape)
    return mkv


def ffmpeg(args):
    p = subprocess.run([NR.FFMPEG, '-y', '-loglevel', 'error'] + args, capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError('ffmpeg failed: ' + p.stderr[-800:])


def film(a):
    res = opt(a, '--res', '640,720')
    out = os.path.abspath(opt(a, '--out', os.path.join(DC.MEDIA_MP4, 'dualwield_game_vs_utpp_1x.mp4')))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    W = {v: timeline(v) for v in ('van', 'new')}
    assert len(W['van']) == len(W['new']), (len(W['van']), len(W['new']))
    T = len(W['new'])
    ex = ['--title-size', '0.075', '--title-align', 'center', '--label-size', '0.04']
    mk = {}
    for v, title in (('van', 'Game Animations'), ('new', 'Ultimate Third Person Project')):
        mk[v] = render(W[v], [''] * T, title, COL[v], 'gamefilm_' + v, res, 15, CAM, ex)
    ffmpeg(['-i', mk['van'], '-i', mk['new'], '-filter_complex', '[0:v][1:v]hstack=inputs=2,format=yuv420p[v]', '-map', '[v]', '-r', '30',
            '-c:v', 'libx264', '-crf', '20', '-preset', 'slow', '-movflags', '+faststart', out])
    sole = {v: round(float(NR.sole_heights(P, W[v][:, :, :3, 3]).min() * 100), 1) for v in W}
    info = dict(out=out, frames=T, seconds=round(T / 30.0, 2), bytes=os.path.getsize(out), sequence=[c for c, _ in SEQ], crossfade=XF, camera=CAM,
                sole_min_cm=sole, left='Female V Lexington dual-wield player clips (no correction)', right='dw1 pilot clips')
    MC.jdump(info, os.path.join(DC.OUT, 'film_info.json'))
    print('FILM', json.dumps(info))


def gate_margin(W):
    """(copy of melee npc_film.gate_margin) per frame the lowest normalised margin over the base-gate features that can move."""
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
        if np.median(np.abs(m)) < 1e-3:
            continue
        upd = m < best
        best = np.where(upd, m, best)
        name = np.where(upd, k, name)
    return best, name


def worst(a):
    res = opt(a, '--res', '300,360')
    out = os.path.abspath(opt(a, '--out', os.path.join(DC.MEDIA_PNG, 'dualwield_dw1_worst_frames.png')))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    Ws, labels, notes = [], [], {}
    for clip in ('equip', 'fire_right', 'fire_left', 'reload', 'aim_idle_loop'):
        lt, lr = new_clip(clip)
        W = NB.fk(lt, lr)
        inf = json.load(open(os.path.join(B.CLIPS, '%s__%s.%s.info.json' % (B.SET, clip, B.SUFFIX)), encoding='utf-8'))
        gun = inf['per_frame']['gun']
        m, feat = gate_margin(W)
        hd = np.min([np.linalg.norm(W[:, IX[s + 'Hand'], :3, 3] - (W[:, IX['LeftEye'], :3, 3] + W[:, IX['RightEye'], :3, 3]) / 2, axis=1)
                     for s in DC.SIDES], 0)
        pick = [('margin %.2f %s' % (m[int(np.argmin(m))], feat[int(np.argmin(m))][:14]), int(np.argmin(m))),
                ('gun-gun %.1f cm' % min(gun['gun_gun']), int(np.argmin(gun['gun_gun']))),
                ('gun-body %.1f cm' % min(gun['body_clear']), int(np.argmin(gun['body_clear']))),
                ('hand-eyes %.1f cm' % (hd.min() * 100), int(np.argmin(hd)))]
        if clip in ('equip', 'reload'):
            pick += [('first', 0), ('last', len(W) - 1)]
        seen = set()
        for why, f in pick:
            if f in seen:
                continue
            seen.add(f)
            Ws.append(W[f:f + 1])
            labels.append('%s f%d %s' % (clip.replace('aim_idle_loop', 'aim loop').replace('_', ' '), f, why))
        notes[clip] = {why: f for why, f in pick}
    Wc = np.concatenate(Ws)
    n = len(Wc)
    cols = 6
    rows = (n + cols - 1) // cols
    mk = [render(Wc, labels, '', '0.25,0.62,0.45', 'sheet_worst_%d' % i, res, 1, cam) for i, cam in enumerate((CAM, CAM2))]
    tile = 'tile=%dx%d:padding=2:color=white' % (cols, rows)
    ffmpeg(['-i', mk[0], '-i', mk[1], '-filter_complex', '[0:v]%s[a];[1:v]%s[b];[a][b]vstack=inputs=2[v]' % (tile, tile), '-map', '[v]',
            '-frames:v', '1', out])
    MC.jdump(dict(out=out, stills=n, rows='top: front-right view, bottom: front-left view', picks=notes, labels=labels),
             os.path.join(DC.OUT, 'worst_frames.json'))
    print('SHEET', out, n, os.path.getsize(out))


if __name__ == '__main__':
    a = sys.argv[1:]
    {'film': film, 'worst': worst}[a[0]](a)
