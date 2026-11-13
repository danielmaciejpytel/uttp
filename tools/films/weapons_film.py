"""Weapon films 2026-11-09 (offline, Female V, skinned body, 1x, 30 fps).  Run with Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I.

  weapons_film.py k5      -> Media\\mp4\\2026-11-09_weapon-films\\barki_trzy_warianty_1x.mp4
  weapons_film.py film2   -> Media\\mp4\\2026-11-09_weapon-films\\celowanie_strzelanie_nue_karabinek_game_vs_utpp_1x.mp4
  weapons_sheet.py film2 | k5 | closeup -> Media\\png\\2026-11-09_weapon-films\\*_najgorsze_klatki.png (needs the caches of k5 / film2)

Right side / all K5 panels: the production pose evaluator (weapons_eval.exe, weapons_common.py) on the installed mod data.  Left side of
film2: the game's own Female V clips (weapons_vanilla.py).  Guns: PROXY boxes on the WeaponRight bone (same on both sides; pistol =
slide + grip, rifle = receiver, handguard, barrel, stock, grip, magazine sized from the hands of the vanilla Ajax ADS pose), not the
game meshes.  Titles only, no technical labels.  Intermediates (steps, evaluator output, skin caches, MKV) in
Build\\work\\weapon-films-20261109\\_tmp; no image file except the sheets.
"""
import json
import os
import struct
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import weapons_common as WC  # noqa: E402
import weapons_vanilla as V  # noqa: E402

NR, MC = WC.NR, WC.MC
TMP = WC.TMP
COL = {'van': '0.30,0.50,0.85', 'new': '0.25,0.62,0.45'}
PISTOL = [((-0.014, -0.05, -0.015), (0.014, 0.16, 0.02)),          # slide (dual-wield pilot boxes, weapon frame: +Y barrel, +Z up)
          ((-0.013, -0.055, -0.09), (0.013, -0.015, -0.015))]      # grip
RIFLE = [((-0.025, -0.12, -0.05), (0.025, 0.12, 0.035)),           # receiver
         ((-0.022, 0.12, -0.045), (0.022, 0.40, 0.025)),           # handguard (left hand at y 0.22, z -0.06 in the Ajax ADS pose)
         ((-0.009, 0.40, -0.019), (0.009, 0.58, -0.001)),          # barrel
         ((-0.02, -0.33, -0.075), (0.02, -0.12, 0.02)),            # stock (butt near the right clavicle, y -0.22 .. -0.31)
         ((-0.014, -0.10, -0.13), (0.014, -0.05, -0.05)),          # pistol grip (right hand y -0.07, z -0.06)
         ((-0.014, 0.0, -0.20), (0.014, 0.07, -0.05))]             # magazine
NUE_RECOIL = 'add_recoil_single_shot_ironsight.utpprecoil'          # recoil_profile kNue, aimed shot (ironsight)
AJAX_RECOIL = 'ajax_single_shot_ironsight.utpprecoil'               # recoil_profile 12 ajax, aimed shot
K5_VARIANTS = [('global', (0, 0.025), 'Obecnie (zwężenie wszędzie)'), ('none', (0, 0.0), 'Bez zwężenia'),
               ('low', (1, 0.020), 'Zwężenie tylko w niskiej pozie')]
# film2 schedule (s): (ready until aim_on, aim until aim_off, ready until end); shots; burst (automatic fire window)
NUE = dict(aim_on=1.2, aim_off=5.4, end=6.8, shots=[2.2, 2.7, 3.2, 3.7, 4.2], burst=None)
AJAX = dict(aim_on=1.2, aim_off=6.4, end=7.8, shots=[2.2, 2.7, 3.2], burst=(4.0, 5.2), cadence=0.12)
ADS_S = 0.2                                                         # left side shoulder <-> ironsight blend (the overlay machine ADS is 0.2 s)


def opt(a, k, d):
    return a[a.index(k) + 1] if k in a else d


def box(lo, hi):
    c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])], float)
    f = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return c, f


def gun_cache(W, base, boxes):
    """proxy boxes on WeaponRight in world space per frame -> <base>.npz (rest, faces) + <base>.pc2."""
    M = W[:, WC.player().ix['WeaponRight']]
    Vs, F, off = [], [], 0
    for lo, hi in boxes:
        c, f = box(lo, hi)
        Vs.append(np.einsum('tij,nj->tni', M[:, :3, :3], c) + M[:, None, :3, 3])
        F += [tuple(i + off for i in q) for q in f]
        off += len(c)
    Vv = np.concatenate(Vs, 1).astype('<f4')
    np.savez(base + '.npz', rest=Vv[0], faces=np.array(F, np.int32))
    with open(base + '.pc2', 'wb') as fh:
        fh.write(b'POINTCACHE2\0' + struct.pack('<iiffi', 1, Vv.shape[1], 0.0, 1.0, Vv.shape[0]))
        fh.write(Vv.tobytes())
    return base


def root_path(r, W):
    """world xy path of the evaluator output from the Trajectory bone (root motion of the playing clip): per-frame deltas, loop wraps /
    clip switches (jumps > 0.15 m per frame) continue the walk clip's previous step or stop."""
    tr = W[:, WC.player().ix['Trajectory'], :2, 3]
    clips = r['diag']['clip']
    d = np.zeros_like(tr)
    for f in range(1, len(tr)):
        s = tr[f] - tr[f - 1]
        if np.linalg.norm(s) > 0.15:
            s = d[f - 1] if clips[f] == clips[f - 1] else np.zeros(2)
        d[f] = s
    return np.cumsum(d, 0)


def utpp_world(r):
    lt, lr = WC.full152(r['lt'], r['lr'])
    W = WC.fk(lt, lr)
    xy = root_path(r, W)
    return WC.place_root(W, xy) if np.abs(xy).max() > 1e-6 else W


def render(W, title, color, name, res, cam, tz, lens, boxes, follow=15, labels=None, title_size='0.06', extra=()):
    os.makedirs(TMP, exist_ok=True)
    base = os.path.join(TMP, name)
    NR.skin_W(W[:, :73], WC.player().ref['rest'], 'woman', base)
    gb = gun_cache(W, base + '_gun', boxes)
    lj = base + '.labels.json'
    json.dump(labels if labels is not None else [''] * len(W), open(lj, 'w', encoding='utf-8'), ensure_ascii=False)
    mkv = base + '.mkv'
    cmd = [NR.BLENDER, '-b', '--factory-startup', '-P', os.path.join(HERE, 'weapons_film_render.py'), '--', base, mkv, lj, '--res', res,
           '--follow', str(follow), '--title', title, '--color', color, '--cam', cam, '--guns', gb, '--tz', str(tz), '--lens', str(lens),
           '--title-size', title_size, '--title-align', 'center', '--label-size', '0.035'] + list(extra)
    p = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace')
    if 'RENDERED' not in p.stdout:
        raise RuntimeError('blender failed: ' + (p.stdout + p.stderr)[-1500:])
    print('render', name, len(W))
    return mkv


def ffmpeg(args):
    p = subprocess.run([NR.FFMPEG, '-y', '-loglevel', 'error'] + args, capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError('ffmpeg failed: ' + p.stderr[-800:])


def save_cache(name, W, extra):
    np.savez_compressed(os.path.join(TMP, name + '.world.npz'), W=W.astype(np.float32), **extra)


# ----------------------------------------------------------------------------------------------------------------- K5 shoulders
def k5_timeline(fem):
    tl = WC.Timeline('Pistol2H', fem=fem)
    tl.aim = 'relaxed'
    tl.run(1.5)                       # low pose (Pistol2H Relaxed), standing
    tl.run(1.7, aim='aim')            # aim
    tl.run(3.6, aim='relaxed')        # release: the overlay machine holds Ready ~2.5 s, then back to the low pose
    tl.switch_clip(WC.WALK, 0.0, 0.3)
    tl.run(2.6, moving=1)             # short walk in the low pose
    tl.switch_clip(WC.IDLE, 0.0, 0.4)
    tl.run(1.4, moving=0)
    return tl


def k5_stats(d):
    narrow = (np.asarray(d['widthBefore'], float) - np.asarray(d['widthAfter'], float)) * 100
    states = {'low (Relaxed)': np.asarray(d['wRelaxedP2H'], float) > 0.99, 'aim': np.asarray(d['wAimP2H'], float) > 0.99,
              'ready': np.asarray(d['wReadyP2H'], float) > 0.99, 'walk': np.char.startswith(np.asarray(d['clip'], str), 'M_Neutral_Walk')}
    out = {}
    for k, m in states.items():
        if m.any():
            out[k] = dict(frames=int(m.sum()), narrow_cm_min=round(float(narrow[m].min()), 2), narrow_cm_max=round(float(narrow[m].max()), 2),
                          narrow_cm_mean=round(float(narrow[m].mean()), 2))
    out['all'] = dict(narrow_cm_min=round(float(narrow.min()), 2), narrow_cm_max=round(float(narrow.max()), 2),
                      fem_hand_err_mm_max=round(float(np.max(d['femHandErr']) * 1000), 4),
                      fem_weapon_err_mm_max=round(float(np.max(d['femWeaponErr']) * 1000), 4),
                      elbow_move_cm_max=round(float(np.max([d['elbowMoveL'], d['elbowMoveR']]) * 100), 2),
                      elbow_flex_deg_maxabs=round(float(np.max(np.abs([d['elbowFlexL'], d['elbowFlexR']]))), 2),
                      grip_left_err_mm_max=round(float(np.max(d['gripLeftErr']) * 1000), 3),
                      width_step_cm_max=round(float(np.abs(np.diff(narrow)).max()), 3))
    return out


def k5(a):
    res = opt(a, '--res', '520,720')
    out = os.path.abspath(opt(a, '--out', os.path.join(WC.MEDIA_MP4, 'barki_trzy_warianty_1x.mp4')))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    mk, info = [], {}
    for key, fem, title in K5_VARIANTS:
        r = WC.run_eval(k5_timeline(fem), 'k5_' + key)
        W = utpp_world(r)
        info[key] = dict(title=title, fem=fem, sanity=WC.sanity(W, 'k5_' + key), stats=k5_stats(r['diag']))
        save_cache('k5_' + key, W, dict(diag_width_before=np.asarray(r['diag']['widthBefore'], float),
                                        diag_width_after=np.asarray(r['diag']['widthAfter'], float)))
        mk.append(render(W, title, COL['new'], 'k5_' + key, res, '0.45,2.3,1.45', 1.22, 42, PISTOL, title_size='0.042'))
    ffmpeg(['-i', mk[0], '-i', mk[1], '-i', mk[2], '-filter_complex', '[0:v][1:v][2:v]hstack=inputs=3,format=yuv420p[v]', '-map', '[v]',
            '-r', '30', '-c:v', 'libx264', '-crf', '20', '-preset', 'slow', '-movflags', '+faststart', out])
    info['out'] = out
    info['bytes'] = os.path.getsize(out)
    WC.jdump(info, os.path.join(WC.WORK, 'k5_info.json'))
    print('K5', json.dumps(info)[:3000])


# ----------------------------------------------------------------------------------------------------------------- film 2
def utpp_segment(overlay, recoil, sch):
    tl = WC.Timeline(overlay, fem=(0, 0.025))
    tl.aim = 'ready'
    tl.run(sch['aim_on'])
    tl.run(sch['aim_off'] - sch['aim_on'], aim='aim')
    tl.run(sch['end'] - sch['aim_off'], aim='ready')
    shots = list(sch['shots'])
    if sch.get('burst'):
        t = sch['burst'][0]
        while t < sch['burst'][1] - 1e-6:
            shots.append(round(t, 4))
            t += sch['cadence']
    return tl, shots


def film2(a):
    res = opt(a, '--res', '640,720')
    out = os.path.abspath(opt(a, '--out', os.path.join(WC.MEDIA_MP4, 'celowanie_strzelanie_nue_karabinek_game_vs_utpp_1x.mp4')))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    cam, tz, lens = '1.45,1.85,1.5', 1.15, 36
    parts, info = [], {}
    for key, overlay, recoil, sch, boxes in (('nue', 'Pistol2H', NUE_RECOIL, NUE, PISTOL), ('ajax', 'Rifle', AJAX_RECOIL, AJAX, RIFLE)):
        tl, shots = utpp_segment(overlay, recoil, sch)
        r = WC.run_eval(tl, 'f2_' + key, shots_s=shots, recoil_file=recoil)
        Wn = utpp_world(r)
        T = len(Wn)
        vs = V.VSet(key)
        lt, lr = V.timeline(vs, T, sch['aim_on'], sch['aim_off'], ADS_S, sch['shots'], sch.get('burst'))
        Wv = WC.fk(lt, lr)
        d = r['diag']
        info[key] = dict(frames=T, seconds=round(T / 30.0, 2), shots_utpp=shots, shots_game=sch['shots'], burst=sch.get('burst'),
                         recoil_file=recoil, log=[ln for ln in r['log'].splitlines() if ln.startswith('RECOIL')],
                         sanity_utpp=WC.sanity(Wn, key + '_utpp'), sanity_game=WC.sanity(Wv, key + '_game'),
                         shoot_applied_frames=int(np.sum(d['shootApplied'])), shoot_scale_min=round(float(np.min(
                             np.where(np.asarray(d['shootApplied']) > 0, d['shootScale'], 1.0))), 3),
                         recoil_eff_T_max_cm=round(float(np.max(d['recoilT']) * 100), 2), recoil_eff_R_max_deg=round(float(np.degrees(np.max(d['recoilR']))), 2),
                         grip_left_err_mm_max=round(float(np.max(d['gripLeftErr']) * 1000), 3),
                         narrow_cm=[round(float(np.min(d['widthBefore'] - d['widthAfter']) * 100), 2), round(float(np.max(d['widthBefore'] - d['widthAfter']) * 100), 2)])
        save_cache('f2_' + key + '_utpp', Wn, dict(shootApplied=np.asarray(d['shootApplied']), gripLeftErr=np.asarray(d['gripLeftErr'], float),
                                                     recoilT=np.asarray(d['recoilT'], float)))
        save_cache('f2_' + key + '_game', Wv, {})
        mv = render(Wv, 'Game Animations', COL['van'], 'f2_%s_game' % key, res, cam, tz, lens, boxes, follow=15, title_size='0.06')
        mn = render(Wn, 'Ultimate Third Person Project', COL['new'], 'f2_%s_utpp' % key, res, cam, tz, lens, boxes, follow=15, title_size='0.06')
        part = os.path.join(TMP, 'f2_%s_pair.mkv' % key)
        ffmpeg(['-i', mv, '-i', mn, '-filter_complex', '[0:v][1:v]hstack=inputs=2,format=yuv420p[v]', '-map', '[v]', '-r', '30', '-c:v', 'libx264',
                '-crf', '16', '-preset', 'slow', part])
        parts.append(part)
    lst = os.path.join(TMP, 'f2_concat.txt')
    with open(lst, 'w') as fh:
        for p in parts:
            fh.write("file '%s'\n" % p.replace('\\', '/'))
    ffmpeg(['-f', 'concat', '-safe', '0', '-i', lst, '-r', '30', '-c:v', 'libx264', '-crf', '20', '-preset', 'slow', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', out])
    info['out'] = out
    info['bytes'] = os.path.getsize(out)
    info['camera'] = dict(offset=cam, target_z=tz, lens=lens)
    WC.jdump(info, os.path.join(WC.WORK, 'film2_info.json'))
    print('FILM2', json.dumps(info)[:3000])


if __name__ == '__main__':
    a = sys.argv[1:]
    {'k5': k5, 'film2': film2}[a[0]](a)
