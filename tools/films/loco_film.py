"""Locomotion comparison films (2026-11-09): left "Game Animations" (loco_game.py), right "Ultimate Third Person Project" (replay dump of
the installed v2 bundle, loco_replay_run.py), same feed, same capsule path, same camera, 1x, 30 fps, titles only.

  python -I loco_film.py film 1|3 [--res 640,720]   -> Media\\mp4\\2026-11-09_loco-films\\<name>_game_vs_utpp_1x.mp4
  python -I loco_film.py worst 1|3 [--side utpp|game] -> Media\\png\\2026-11-09_loco-films\\<name>_<side>_worst_frames.png
  python -I loco_film.py metrics 1|3                -> Build\\work\\loco-films-20261109\\metrics_<n>.json (no media)
Film 1 = loco_film1.feed (walk / run / sprint: starts, 90 / 180 turns, stops), Film 3 = loco_film3.feed (sprint -> slide -> exit;
run -> crouch, crouch walk, stand up).  60 Hz data, every 2nd frame rendered (30 fps = real time).
Camera: the smoothed capsule path of the feed (0.5 s moving average) + a fixed world offset, the same .npy for both sides; ground extent
the same for both.  Intermediates (skin caches, MKV, camera path) in Build\\work\\loco-films-20261109\\_tmp; image / video files only in
Media (the sheet = one PNG tiled by ffmpeg from an MKV of stills, no per-frame images).
Worst frames (60 Hz, side utpp by default): lowest base anatomy gate margin (melee anatomy_gate_base via npc_build, as dw_film.py),
largest world rotation step of a body joint per 1/60 s, largest planted-foot slide (toe / heel xy speed while the sole is within 2 cm of
the ground), lowest sole (below the ground), smallest knee-knee / ankle-ankle distance, smallest hand-to-torso distance.
"""
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import loco_common as LC  # noqa: E402
import loco_game as LG  # noqa: E402

NB, NR, MC, P = LC.NB, LC.NR, LC.MC, LC.P
IX = P.ix
FILMS = {'1': ('loco_film1', 'lokomocja'), '3': ('loco_film3', 'bieg_wslizg_kucanie')}
COL = {'game': '0.30,0.50,0.85', 'utpp': '0.25,0.62,0.45'}
CAM = '2.7,-3.0,1.45'
CAM2 = '-2.7,-3.0,1.45'
LENS = '42'
BODY = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head'] + [s + n for s in ('Left', 'Right') for n in (
    'Shoulder', 'Arm', 'ForeArm', 'Hand', 'UpLeg', 'Leg', 'Foot', 'ToeBase')]


def opt(a, k, d):
    return a[a.index(k) + 1] if k in a else d


def feed_path(n):
    return os.path.join(LC.WORK, 'feeds', FILMS[n][0] + '.feed')


def sides(n):
    """-> dict(utpp=W60, game=W60), labels (clip names per 60 Hz frame), capsule xy [T,2], dump."""
    d = LC.load_dump(os.path.join(LC.WORK, 'replay', FILMS[n][0] + '.dump.txt'))
    Wu = LC.utpp_world(d)
    dev = LC.placement_check(d, Wu)
    assert max(dev) < 0.002, ('placement check failed', dev)
    Wg, glab, fd = LG.timeline(feed_path(n))
    assert len(Wg) == len(Wu), (len(Wg), len(Wu))
    cap = np.stack([fd['x'], fd['y']], 1)
    capd = np.array([m['cap'] for m in d['meta']])
    assert np.abs(cap - capd).max() < 0.01, 'feed capsule path differs from the replay capsule path'
    return dict(utpp=Wu, game=Wg), dict(utpp=[m['clip'] for m in d['meta']], game=glab), cap, d


def ffmpeg(args):
    p = subprocess.run([NR.FFMPEG, '-y', '-loglevel', 'error'] + args, capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError('ffmpeg failed: ' + p.stderr[-800:])


def render(W73, labels, title, color, name, res, cam, campath=None, extent=None, extra=()):
    os.makedirs(LC.TMP, exist_ok=True)
    base = os.path.join(LC.TMP, name)
    NR.skin_W(W73, P.ref['rest'], NB.RIG, base)
    sk = np.load(base + '.npz')
    j = sk['joints']
    dev = float(np.abs(j - W73[:, [IX[str(nm)] for nm in sk['joint_names']]][:, :, :3, 3]).max())
    lj = base + '.labels.json'
    json.dump(labels, open(lj, 'w', encoding='utf-8'))
    mkv = base + '.mkv'
    cmd = [NR.BLENDER, '-b', '--factory-startup', '-P', os.path.join(HERE, 'loco_film_render.py'), '--', base, mkv, lj, '--res', res,
           '--title', title, '--color', color, '--cam', cam, '--lens', LENS] + list(extra)
    if campath is not None:
        cmd += ['--campath', campath]
    if extent is not None:
        cmd += ['--extent', extent]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if 'RENDERED' not in p.stdout:
        raise RuntimeError('blender failed: ' + (p.stdout + p.stderr)[-1500:])
    print('render', name, 'frames', len(W73), 'skin joints vs W max dev m', round(dev, 5))
    return mkv, dev


def film(a):
    n = a[1]
    res = opt(a, '--res', '640,720')
    feed, stem = FILMS[n]
    out = os.path.abspath(os.path.join(LC.MEDIA_MP4, stem + '_game_vs_utpp_1x.mp4'))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    W, lab, cap, d = sides(n)
    sel = np.arange(0, len(cap), 2)                       # 60 Hz -> 30 fps (real time)
    cp = LC.cam_path(cap[:, 0], cap[:, 1], 30)[sel]
    os.makedirs(LC.TMP, exist_ok=True)
    cpf = os.path.join(LC.TMP, feed + '_campath.npy')
    np.save(cpf, cp)
    lo, hi = cp.min(0) - 20.0, cp.max(0) + 20.0
    ext = '%.2f,%.2f,%.2f,%.2f' % (lo[0], lo[1], hi[0], hi[1])
    ex = ['--title-size', '0.075', '--title-align', 'center', '--label-size', '0.04']
    mk, devs = {}, {}
    for s, title in (('game', 'Game Animations'), ('utpp', 'Ultimate Third Person Project')):
        mk[s], devs[s] = render(W[s][sel], [''] * len(sel), title, COL[s], '%s_film_%s' % (feed, s), res, CAM, cpf, ext, ex)
    ffmpeg(['-i', mk['game'], '-i', mk['utpp'], '-filter_complex', '[0:v][1:v]hstack=inputs=2,format=yuv420p[v]', '-map', '[v]', '-r', '30',
            '-c:v', 'libx264', '-crf', '20', '-preset', 'slow', '-movflags', '+faststart', out])
    # clip sequences (run-length) of both sides for the report
    seq = {}
    for s in ('game', 'utpp'):
        runs, last = [], None
        for i, c in enumerate(lab[s]):
            if c != last:
                runs.append([round(i / LC.FPS, 2), c])
                last = c
        seq[s] = runs
    info = dict(out=out, frames=len(sel), seconds=round(len(sel) / 30.0, 2), bytes=os.path.getsize(out), feed=feed_path(n), camera=CAM,
                lens=LENS, campath=cpf, extent=ext, skin_dev_m=devs, placement_dev_m=LC.placement_check(d, W['utpp']), clips=seq)
    MC.jdump(info, os.path.join(LC.WORK, 'film_info_%s.json' % n))
    print('FILM', out, info['frames'], info['seconds'], info['bytes'])


def gate_margin(W):
    """(copy of dualwield dw_film.gate_margin = melee npc_film.gate_margin) per frame the lowest normalised margin over the base-gate
    features that can move, and the feature name."""
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


def seg_dist(p0, p1, q0, q1):
    """distance between segments p0-p1 and q0-q1 per frame [T] (clamped closest points)."""
    u, v, w0 = p1 - p0, q1 - q0, p0 - q0
    a, b, c = np.sum(u * u, 1), np.sum(u * v, 1), np.sum(v * v, 1)
    dd, e = np.sum(u * w0, 1), np.sum(v * w0, 1)
    den = np.maximum(a * c - b * b, 1e-9)
    s = np.clip((b * e - c * dd) / den, 0, 1)
    t = np.clip((b * s + e) / np.maximum(c, 1e-9), 0, 1)
    s = np.clip((b * t - dd) / np.maximum(a, 1e-9), 0, 1)
    return np.linalg.norm((p0 + u * s[:, None]) - (q0 + v * t[:, None]), axis=1)


def metrics(W):
    """per-frame metrics of a 60 Hz world clip [T,73,4,4] (dict of arrays)."""
    pos = lambda nm: W[:, IX[nm], :3, 3]
    m, feat = gate_margin(W)
    st = LC.world_rot_steps(W)
    bi = [IX[b] for b in BODY]
    step = st[:, bi].max(1)
    step_j = np.array(BODY)[np.argmax(st[:, bi], 1)]
    sole = LC.sole_heights(W)
    slide = np.zeros(len(W))
    for s, side in enumerate(('Left', 'Right')):
        for jn in (side + 'ToeBase', side + 'Foot'):
            v = np.zeros(len(W))
            v[1:] = np.linalg.norm(np.diff(pos(jn)[:, :2], axis=0), axis=1) * LC.FPS
            planted = sole[:, s] < 0.02
            planted[1:] &= planted[:-1]
            slide = np.maximum(slide, np.where(planted, v, 0.0))
    knee = seg_dist(pos('LeftUpLeg'), pos('LeftLeg'), pos('RightUpLeg'), pos('RightLeg'))
    shin = seg_dist(pos('LeftLeg'), pos('LeftFoot'), pos('RightLeg'), pos('RightFoot'))
    torso = [pos('Hips'), pos('Spine3')]
    hand = np.min([seg_dist(pos(s + 'ForeArm'), pos(s + 'Hand'), torso[0], torso[1]) for s in ('Left', 'Right')], 0)
    return dict(margin=m, margin_feat=feat, step=step, step_joint=step_j, sole=sole.min(1), slide=slide, legs=np.minimum(knee, shin), hand_torso=hand,
                hips_z=W[:, IX['Hips'], 2, 3])


def summary(M, labels):
    out = {}
    for k, worst in (('margin', np.argmin), ('step', np.argmax), ('slide', np.argmax), ('sole', np.argmin), ('legs', np.argmin),
                     ('hand_torso', np.argmin), ('hips_z', np.argmin)):
        f = int(worst(M[k]))
        out[k] = dict(frame=f, t=round(f / LC.FPS, 3), value=round(float(M[k][f]), 4), clip=labels[f],
                      p50=round(float(np.median(M[k])), 4), p99=round(float(np.percentile(M[k], 99)), 4), p1=round(float(np.percentile(M[k], 1)), 4))
    f = int(np.argmin(M['margin']))
    out['margin']['feature'] = str(M['margin_feat'][f])
    f = int(np.argmax(M['step']))
    out['step']['joint'] = str(M['step_joint'][f])
    out['gate_fail_frames'] = int(np.sum(M['margin'] < 0))
    out['step_over_20deg_frames'] = int(np.sum(M['step'] > 20.0))
    out['sole_below_minus2cm_frames'] = int(np.sum(M['sole'] < -0.02))
    out['planted_slide_over_0.5ms_frames'] = int(np.sum(M['slide'] > 0.5))
    return out


def metrics_cmd(a):
    n = a[1]
    W, lab, cap, d = sides(n)
    res = {}
    for s in ('utpp', 'game'):
        res[s] = summary(metrics(W[s]), lab[s])
    MC.jdump(res, os.path.join(LC.WORK, 'metrics_%s.json' % n))
    print(json.dumps(res, indent=1)[:6000])


def worst(a):
    n = a[1]
    side = opt(a, '--side', 'utpp')
    res = opt(a, '--res', '300,360')
    feed, stem = FILMS[n]
    out = os.path.abspath(os.path.join(LC.MEDIA_PNG, '%s_%s_worst_frames.png' % (stem, side)))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    W, lab, cap, d = sides(n)
    Wx, L = W[side], lab[side]
    M = metrics(Wx)
    picks = []

    def add(why, f):
        if all(abs(f - g) > 6 for _, g in picks):
            picks.append((why, int(f)))
    add('margin %.2f %s' % (M['margin'].min(), str(M['margin_feat'][int(np.argmin(M['margin']))])[:12]), np.argmin(M['margin']))
    order = np.argsort(-M['step'])
    for f in order[:40]:
        if len([1 for w, _ in picks if w.startswith('step')]) >= 3:
            break
        add('step %.0fdeg %s' % (M['step'][f], M['step_joint'][f]), f)
    order = np.argsort(-M['slide'])
    for f in order[:40]:
        if len([1 for w, _ in picks if w.startswith('slide')]) >= 2:
            break
        add('slide %.2fm/s' % M['slide'][f], f)
    add('sole %.1fcm' % (M['sole'].min() * 100), np.argmin(M['sole']))
    add('legs %.1fcm' % (M['legs'].min() * 100), np.argmin(M['legs']))
    add('hand-torso %.1fcm' % (M['hand_torso'].min() * 100), np.argmin(M['hand_torso']))
    add('hips low %.2fm' % M['hips_z'].min(), np.argmin(M['hips_z']))
    # clip / stance edges of the side (first frame of a new clip), the 3 with the largest step within 4 frames after the change
    edges = [i for i in range(1, len(L)) if L[i] != L[i - 1]]
    edges = sorted(edges, key=lambda i: -M['step'][i:i + 4].max())[:3]
    for e in edges:
        add('edge %s' % L[e][:16], e)
    picks.sort(key=lambda p: p[1])
    frames = [f for _, f in picks]
    labels = ['%.2fs %s | %s' % (f / LC.FPS, why, L[f][:22]) for why, f in picks]
    cols = 6
    rows = (len(frames) + cols - 1) // cols
    Ws = Wx[frames]
    mk = []
    for i, cam in enumerate((CAM, CAM2)):
        m_, _ = render(Ws, labels, '', COL[side], '%s_worst_%s_%d' % (feed, side, i), res, cam, None, None, ['--label-size', '0.045'])
        mk.append(m_)
    tile = 'tile=%dx%d:padding=2:color=white' % (cols, rows)
    ffmpeg(['-i', mk[0], '-i', mk[1], '-filter_complex', '[0:v]%s[a];[1:v]%s[b];[a][b]vstack=inputs=2[v]' % (tile, tile), '-map', '[v]',
            '-frames:v', '1', out])
    MC.jdump(dict(out=out, side=side, stills=len(frames), rows='top: front-right view, bottom: front-left view', picks=picks, labels=labels,
                  summary=summary(M, L)), os.path.join(LC.WORK, 'worst_%s_%s.json' % (n, side)))
    print('SHEET', out, len(frames))


def zoom(a):
    """close-up sheet of chosen 60 Hz frames of one side (two views): zoom 1|3 --frames f,f,... --name <png stem> [--side utpp|game]
    [--cam dx,dy,dz] [--cam2 dx,dy,dz] [--res 360,400] [--cols 6]."""
    n = a[1]
    side = opt(a, '--side', 'utpp')
    frames = [int(x) for x in opt(a, '--frames', '').split(',') if x]
    stem = opt(a, '--name', '%s_%s_zoom' % (FILMS[n][1], side))
    res = opt(a, '--res', '360,400')
    cols = int(opt(a, '--cols', '6'))
    cams = (opt(a, '--cam', '1.9,-2.1,0.9'), opt(a, '--cam2', '-1.9,-2.1,0.9'))
    out = os.path.abspath(os.path.join(LC.MEDIA_PNG, stem + '.png'))
    W, lab, cap, d = sides(n)
    M = metrics(W[side])
    labels = ['%.2fs f%d sole %.0fcm %s' % (f / LC.FPS, f, M['sole'][f] * 100, lab[side][f][10:30]) for f in frames]
    rows = (len(frames) + cols - 1) // cols
    mk = []
    for i, cam in enumerate(cams):
        m_, _ = render(W[side][frames], labels, '', COL[side], '%s_zoom_%s_%d' % (FILMS[n][0], side, i), res, cam, None, None,
                       ['--label-size', '0.04'])
        mk.append(m_)
    tile = 'tile=%dx%d:padding=2:color=white' % (cols, rows)
    ffmpeg(['-i', mk[0], '-i', mk[1], '-filter_complex', '[0:v]%s[a];[1:v]%s[b];[a][b]vstack=inputs=2[v]' % (tile, tile), '-map', '[v]',
            '-frames:v', '1', out])
    MC.jdump(dict(out=out, side=side, frames=frames, labels=labels, cams=cams), os.path.join(LC.WORK, '%s.json' % stem))
    print('ZOOM', out, len(frames))


if __name__ == '__main__':
    a = sys.argv[1:]
    {'film': film, 'worst': worst, 'metrics': metrics_cmd, 'zoom': zoom}[a[0]](a)
