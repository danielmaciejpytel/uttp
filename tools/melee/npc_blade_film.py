"""1x film and contact sheets of the NPC-based knife / katana melee clips (npc_blade_build.py output), Female V film; sheets per rig.

  python -I npc_blade_film.py film   [--res 640,720] [--out <mp4>]          left = the game's own clips ("Game Animations"),
                                                                             right = npc1 ("Ultimate Third Person Project"), knife then katana
  python -I npc_blade_film.py sheet  [--out <png>]                          keyframes of a few attacks, rows game / npc1
  python -I npc_blade_film.py worst  [--rig man] [--out <png>] [--top N]    worst frames of the npc1 clips of both weapons (lowest
                                                                             base-gate margin, smallest blade clearance, attack f1)
Local variant of npc_film.py (W1 tool, imported unchanged: to_local, fk73, blend, gate_margin, ffmpeg): the weapon is drawn (handle +
blade on WeaponRight, npc_blade_film_render.py); the film has titles only, no per-frame labels.  Every clip that is not in clips\\ (rejected
or not built) plays the game's own clip on the right as well.  Video path only: Blender renders straight into MKV files under
Build\\work\\melee-npc-blades-20261109\\_tmp, ffmpeg stacks / tiles them; no png intermediates.
"""
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import npc_blade_common as BC  # noqa: E402  (consumes --rig)
import npc_film as NF  # noqa: E402  (W1 tool: import only)

NB, NR, MC = BC.NB, BC.NR, BC.MC
OUT = BC.OUTW
TMP = os.path.join(OUT, '_tmp', 'film_' + BC.RIG)
MEDIA_MP4 = os.path.join(MC.ROOT, 'Media', 'mp4', '2026-11-09_melee-npc-blades')
MEDIA_PNG = os.path.join(MC.ROOT, 'Media', 'png', '2026-11-09_melee-npc-blades')
XF = 5
COL = {'van': '0.30,0.50,0.85', 'new': '0.25,0.62,0.45'}
HANDLE = {'knife': (-0.085, 0.03, 0.013), 'katana': (-0.25, 0.05, 0.016)}
BLADE_R = {'knife': 0.007, 'katana': 0.009}
GUARD = {'new': 'guard_idle_loop', 'van': 'melee_idlepose'}     # npc1 guard loop (missing -> load() falls back to melee_idlepose)
SHORT = lambda c: c.replace('combo_attack_', 'combo').replace('final_attack_', 'final').replace('heavy_attack_', 'heavy').replace('melee_', '')


def opt(a, k, d):
    return a[a.index(k) + 1] if k in a else d


_C = {}


def load(variant, weapon, clip):
    """-> (W [T,73,4,4], names, parents, source 'npc1' | 'vanilla', capsule y [T] in m (npc1 info; 0 for the game's clips))."""
    key = (variant, weapon, clip)
    if key not in _C:
        st = BC.player_set(weapon)
        ref = np.load(os.path.join(OUT, 'clips', '%s__melee_idlepose.npc1.npz' % st))
        p = os.path.join(OUT, 'clips', '%s__%s.npc1.npz' % (st, clip))
        cy = None
        if os.path.exists(p.replace('.npz', '.info.json')):
            cy = np.asarray(json.load(open(p.replace('.npz', '.info.json'))).get('capsule_path_cm') or [0.0], float) / 100.0
        if variant == 'new' and os.path.exists(p):
            W, src = np.load(p)['W'][:, :73].astype(float), 'npc1'
        elif os.path.exists(MC.clip_path(st, clip)):
            W, src = MC.world_all(BC.vanilla(st, clip)[0])[:, :73].astype(float), 'vanilla'
        else:                           # not a game clip (guard_idle_loop) and not built: the guard pose
            return load(variant, weapon, 'melee_idlepose')
        if cy is None or len(cy) != len(W):
            cy = np.zeros(len(W))
        _C[key] = (W, list(ref['names']), ref['parents'], src, cy)
    return _C[key]


def built(weapon, clip):
    return os.path.exists(os.path.join(OUT, 'clips', '%s__%s.npc1.npz' % (BC.player_set(weapon), clip)))


MAX_FR = 300            # per weapon (10 s): film 15-20 s for both weapons


def plan(weapon):
    """(clip, n frames or None, overlap, mode)  mode: 'all' | 'chain' (cut at the vanilla recover start + XF) | 'cycle'.  Groups of the
    candidate list are used only when every clip of the group is built (npc1); the guard opens and closes; about MAX_FR frames."""
    g = lambda n: ('guard', n, XF, 'cycle')
    c = lambda clip, mode='all', n=None, ov=XF: (clip, n, ov, mode)
    if weapon == 'knife':
        groups = [[c('combo_attack_01', 'chain'), c('combo_attack_02', 'chain'), c('combo_attack_03', 'chain'), c('combo_attack_04')], [g(12)],
                  [c('final_attack_03')], [g(12)], [c('hold_start_01'), c('hold_loop_01', 'cycle', 20, 1), c('heavy_attack_01', ov=1)], [g(12)],
                  [c('final_attack_04')], [g(12)], [c('jump_attack')], [g(12)]]
    else:
        groups = [[c('combo_attack_01', 'chain'), c('combo_attack_02', 'chain'), c('combo_attack_03', 'chain')], [c('combo_attack_04')], [g(12)],
                  [c('block_start'), c('block', 'cycle', 30, 1), c('block_end', ov=1)], [g(12)],
                  [c('hold_start_01'), c('hold_loop_01', 'cycle', 20, 1), c('heavy_attack_01', ov=1)], [g(12)],
                  [c('final_attack_03')], [g(12)], [c('safe_attack')], [g(12)], [c('crouch_attack')], [g(12)], [c('final_attack_05')], [g(12)]]
    out, n = [g(24)], 24
    for grp in groups:
        if any(e[0] != 'guard' and not built(weapon, e[0]) for e in grp):
            continue
        if out[-1][0] == 'guard' and grp[0][0] == 'guard':
            continue
        if n > MAX_FR and grp[0][0] != 'guard':
            break
        out += grp
        n += sum(e[1] or 25 for e in grp)
    if out[-1][0] != 'guard':
        out.append(g(16))
    return out


def entry_frames(variant, weapon, clip, n, mode):
    W = load(variant, weapon, clip)[0]
    T = len(W)
    if mode == 'all':
        return np.arange(T)
    if mode == 'chain':
        ph = BC.vanilla(BC.player_set(weapon), clip)[1]
        return np.arange(min(T, ph.get('recover', (14, T - 1))[0] + XF))
    per = T - 1 if T > 2 and np.abs(W[0] - W[-1]).max() < 1e-4 else T
    return np.arange(n) % max(per, 1)


def timeline(variant, weapon):
    """-> W [T,73,4,4] with the capsule path added, capsule [T,3], sources used."""
    Ls = C = None
    par, srcs = None, {}
    for clip, n, ov, mode in plan(weapon):
        cname = GUARD[variant] if clip == 'guard' else clip
        W, names, par_, src, cy = load(variant, weapon, cname)
        cy_new = load('new', weapon, cname)[4]            # one capsule path for both sides (npc1 model)
        par = par_
        srcs[cname] = src
        fr = entry_frames(variant, weapon, cname, n, mode)
        L = NF.to_local(W, par)[fr]
        cs = np.zeros((len(fr), 3))
        if mode != 'cycle':
            cs[:, 1] = cy_new[np.minimum(fr, len(cy_new) - 1)] - cy_new[min(fr[0], len(cy_new) - 1)]
        if Ls is None:
            Ls, C = L, cs
            continue
        k = max(1, min(ov, len(L), len(Ls)))
        w = NB.sstep((np.arange(k) + 1.0) / (k + 1.0))
        cin = cs + C[len(C) - k]
        Lo = NF.blend(Ls[-k:], L[:k], w)
        Co = (1 - w)[:, None] * C[-k:] + w[:, None] * cin[:k]
        Ls = np.concatenate([Ls[:-k], Lo, L[k:]])
        C = np.concatenate([C[:-k], Co, cin[k:]])
    W = NF.fk73(Ls, par)
    W[:, :, :3, 3] += C[:, None]
    return W, C, srcs


def render(W, labels, title, color, name, weapon, res, follow, extra=()):
    os.makedirs(TMP, exist_ok=True)
    base = os.path.join(TMP, name)
    NR.skin_W(W, NB.P.ref['rest'], NB.RIG, base)
    lj = base + '.labels.json'
    json.dump(list(labels), open(lj, 'w', encoding='utf-8'))
    wn = base + '.weapon.npy'
    np.save(wn, W[:, BC.WR].astype(np.float64))
    z0, z1, _ = BC.BLADE[weapon]
    mkv = base + '.mkv'
    cmd = [NR.BLENDER, '-b', '--factory-startup', '-P', os.path.join(HERE, 'npc_blade_film_render.py'), '--', base, mkv, lj,
           '--weapon', wn, '--blade', '%g,%g,%g' % (z0, z1, BLADE_R[weapon]), '--handle', '%g,%g,%g' % HANDLE[weapon],
           '--res', res, '--follow', str(follow), '--title', title, '--color', color] + list(extra)
    p = subprocess.run(cmd, capture_output=True, text=True)
    if 'RENDERED' not in p.stdout:
        raise RuntimeError('blender failed: ' + (p.stdout + p.stderr)[-1500:])
    return mkv


def film(a):
    res = opt(a, '--res', '640,720')
    out = os.path.abspath(opt(a, '--out', os.path.join(MEDIA_MP4, 'blades_game_vs_utpp_1x.mp4')))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    ex = ['--title-size', '0.075', '--title-align', 'center', '--label-size', '0.04']
    mk, info = [], dict(out=out, segments={})
    for weapon in ('knife', 'katana'):
        tl = {v: timeline(v, weapon) for v in ('van', 'new')}
        T = min(len(tl[v][0]) for v in tl)
        assert len(tl['van'][0]) == len(tl['new'][0]) and np.allclose(tl['van'][1], tl['new'][1]), 'timelines / capsule paths differ'
        for v, title in (('van', 'Game Animations'), ('new', 'Ultimate Third Person Project')):
            mk.append(render(tl[v][0][:T], [''] * T, title, COL[v], 'film_%s_%s' % (weapon, v), weapon, res, 15, ex))
        info['segments'][weapon] = dict(frames=T, right_sources=tl['new'][2], sequence=[p[0] for p in plan(weapon)])
    NF.ffmpeg(['-i', mk[0], '-i', mk[1], '-i', mk[2], '-i', mk[3], '-filter_complex',
               '[0:v][1:v]hstack=inputs=2[a];[2:v][3:v]hstack=inputs=2[b];[a][b]concat=n=2:v=1:a=0,format=yuv420p[v]', '-map', '[v]', '-r', '30',
               '-c:v', 'libx264', '-crf', '20', '-preset', 'slow', '-movflags', '+faststart', out])
    info.update(frames=sum(s['frames'] for s in info['segments'].values()), bytes=os.path.getsize(out))
    info['seconds'] = round(info['frames'] / 30.0, 2)
    MC.jdump(info, os.path.join(OUT, 'film_info_blades.json'))
    print('FILM', json.dumps(info))


def stills(groups, name, title, out_png, cols=6, res='240,300'):
    """groups: {weapon: [(variant, clip, frame, label)]} -> one MKV per weapon (camera per frame), ffmpeg concat + tile -> ONE png."""
    mk, n = [], 0
    for weapon, items in groups.items():
        if not items:
            continue
        Ws = [load(v, weapon, c)[0][min(int(f), len(load(v, weapon, c)[0]) - 1)][None] for v, c, f, _ in items]
        mk.append(render(np.concatenate(Ws), [it[3] for it in items], title, '0.45,0.55,0.70', '%s_%s' % (name, weapon), weapon, res, 1,
                         ['--label-size', '0.05']))
        n += len(items)
    rows = (n + cols - 1) // cols
    ins = sum([['-i', m] for m in mk], [])
    fc = ''.join('[%d:v]' % i for i in range(len(mk))) + 'concat=n=%d:v=1:a=0[c];[c]tile=%dx%d:padding=2:color=white[v]' % (len(mk), cols, rows)
    NF.ffmpeg(ins + ['-filter_complex', fc, '-map', '[v]', '-frames:v', '1', out_png])
    print('SHEET', out_png, n, os.path.getsize(out_png))


def impact(weapon, clip):
    st = BC.player_set(weapon)
    p = os.path.join(OUT, 'clips', '%s__%s.npc1.info.json' % (st, clip))
    return int(json.load(open(p))['impact_vanilla'])


def sheet(a):
    out = os.path.abspath(opt(a, '--out', os.path.join(MEDIA_PNG, 'blades_keyframes_game_vs_npc1.png')))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    groups = {}
    pref = {'knife': ('combo_attack_02', 'heavy_attack_01', 'final_attack_03', 'combo_attack_01'),
            'katana': ('combo_attack_01', 'combo_attack_02', 'combo_attack_03', 'safe_attack', 'final_attack_03')}
    for weapon in ('knife', 'katana'):
        clips = [c for c in pref[weapon] if built(weapon, c)][:2]          # only clips that passed (npc1 row = npc1)
        items = []
        for clip in clips:
            T = len(load('new', weapon, clip)[0])
            imp = impact(weapon, clip)
            frs = [0, max(0, imp - 3), imp, min(T - 1, imp + 4), min(T - 1, imp + 12), T - 1]
            for v in ('van', 'new'):
                items += [(v, clip, f, '%s %s %s f%d' % ('GAME' if v == 'van' else 'NPC1', weapon, SHORT(clip), f)) for f in frs]
        groups[weapon] = items
    stills(groups, 'sheet_keyframes', '', out)


def worst(a):
    rig = BC.RIG
    out = os.path.abspath(opt(a, '--out', os.path.join(MEDIA_PNG, 'blades_%s_worst_frames.png' % rig)))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    top = int(opt(a, '--top', '12'))
    groups, notes = {}, {}
    for weapon in ('knife', 'katana'):
        st = BC.player_set(weapon)
        rows = []
        for fn in sorted(os.listdir(os.path.join(OUT, 'clips'))):
            if not (fn.startswith(st + '__') and fn.endswith('.npc1.npz')):
                continue
            clip = fn[len(st) + 2:-len('.npc1.npz')]
            W = load('new', weapon, clip)[0]
            th = np.load(os.path.join(OUT, 'clips', fn.replace('.npz', '.twohand.npz')))
            m, feat = NF.gate_margin(W)
            fm = int(np.argmin(m))
            cnt, clr = BC.blade_pen(W, weapon, th['w'])
            fb = int(np.argmin(clr))
            inf = json.load(open(os.path.join(OUT, 'clips', fn.replace('.npz', '.info.json'))))
            fx = inf.get('fixes', {})
            rows.append(dict(clip=clip, min_margin_frame=fm, min_margin=round(float(m[fm]), 3), feature=str(feat[fm]),
                             blade_clear_frame=fb, blade_clear_cm=round(float(clr[fb]), 1), wrist_relax=fx.get('wrist_relax_max', 0.0),
                             hilt_roll=max([abs(x) for x in fx.get('hilt_roll_deg', [0.0])]), impact=inf.get('impact_vanilla')))
        notes[weapon] = rows
        pick = sorted(rows, key=lambda r: r['min_margin'])[:top // 2] + sorted(rows, key=lambda r: r['blade_clear_cm'])[:top // 2]
        if weapon == 'katana':        # the left hand turned on the hilt (wrist relax / roll): fingers may float off or cut the hilt
            for r in sorted([r for r in rows if r['impact'] is not None], key=lambda r: -(r['wrist_relax'] * 100 + r['hilt_roll']))[:2]:
                r = dict(r, min_margin_frame=int(r['impact']), feature='relax %.2f roll %.0f' % (r['wrist_relax'], r['hilt_roll']), min_margin=float('nan'))
                pick.append(r)
        seen, items = set(), []
        for r in pick:
            for f, why in ((r['min_margin_frame'], 'm%.2f %s' % (r['min_margin'], r['feature'][:14])), (r['blade_clear_frame'], 'blade %.1fcm' % r['blade_clear_cm'])):
                if (r['clip'], f) in seen:
                    continue
                seen.add((r['clip'], f))
                items.append(('new', r['clip'], f, '%s %s f%d %s' % (weapon, SHORT(r['clip'])[:16], f, why)))
        groups[weapon] = items
    stills(groups, 'sheet_worst', '', out)
    MC.jdump(notes, os.path.join(OUT, 'worst_frames_%s.json' % rig))
    print('WORST', json.dumps({w: sorted(r, key=lambda x: x['min_margin'])[:5] for w, r in notes.items()}))


if __name__ == '__main__':
    a = [x for x in sys.argv[1:]]
    {'film': film, 'sheet': sheet, 'worst': worst}[a[0]](a)
