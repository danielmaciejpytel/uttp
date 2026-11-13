"""1x film and contact sheets of the NPC-based ONE-HANDED BLUNT and BATON clips of V (npc_blunt_build.py output, 2026-11-09).

  python -I npc_blunt_film.py film  [--res 640,720] [--out <mp4>]          Female V: left = the game's own clips ("Game Animations"),
                                                                           right = npc1 ("Ultimate Third Person Project"), blunt then baton
  python -I npc_blunt_film.py sheet [--out <png>]                          keyframes of two attacks per weapon, rows game / npc1
  python -I npc_blunt_film.py worst [--rig man] [--out <png>] [--top N]    worst frames of the npc1 clips of both weapons: lowest base-gate
                                                                           margin (joint angles), smallest weapon clearance (penetration),
                                                                           largest palm-to-handle deviation (hand - weapon distance)
npc_blade_film.py imported UNCHANGED (load, timeline, render, stills, entry_frames; Blender renders straight into MKV, ffmpeg stacks / tiles,
no png intermediates); only its weapon tables, media folders and the per-weapon sequence (plan) are set here.  The weapon is drawn as the
proxy of the gates (npc_blunt_common: blunt capsule 0.05..0.75 m r 0.025, baton 0.03..0.50 m r 0.018 along +Z of WeaponRight; ASSUMPTION,
no weapon meshes in the project) plus a short handle in the fist.  Clips that are not in clips\\ (rejected / chain-reverted / not built)
play the game's own clip on the right as well.  Media: Media\\mp4|png\\2026-11-09_melee-npc-blunt\\ (flat; never deleted).
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import npc_blunt_common as UC  # noqa: E402  (weapons, OUTW, blade_pen_release; consumes --rig)
import npc_blade_film as F  # noqa: E402

BC, NB, MC, NF = F.BC, F.NB, F.MC, F.NF
WEAPONS = ('blunt', 'baton')
F.MEDIA_MP4 = os.path.join(MC.ROOT, 'Media', 'mp4', '2026-11-09_melee-npc-blunt')
F.MEDIA_PNG = os.path.join(MC.ROOT, 'Media', 'png', '2026-11-09_melee-npc-blunt')
F.HANDLE.update(blunt=(-0.06, 0.05, 0.018), baton=(-0.09, 0.03, 0.016))
F.BLADE_R.update(blunt=BC.BLADE['blunt'][2], baton=BC.BLADE['baton'][2])
if BC.RIG == 'man':                        # npc_blunt_build --rig man writes to <OUTW>\man
    F.OUT = os.path.join(UC.OUTW, 'man')
F.TMP = os.path.join(UC.OUTW, '_tmp', 'film_' + BC.RIG)
opt = F.opt
SHORT = F.SHORT


def plan(weapon):
    """as npc_blade_film.plan, the blunt / baton sequences (a group is used only when every clip of it is built)."""
    g = lambda n: ('guard', n, F.XF, 'cycle')
    c = lambda clip, mode='all', n=None, ov=F.XF: (clip, n, ov, mode)
    hold = [c('hold_start_01'), c('hold_loop_01', 'cycle', 20, 1), c('heavy_attack_01', ov=1)]
    block = [c('block_start'), c('block', 'cycle', 30, 1), c('block_end', ov=1)]
    if weapon == 'blunt':
        groups = [[c('combo_attack_01', 'chain'), c('combo_attack_02', 'chain'), c('combo_attack_03')], [g(12)], [c('final_attack_03')], [g(12)],
                  hold, [g(12)], block, [g(12)], [c('safe_attack')], [g(12)], [c('crouch_attack')], [g(12)]]
    else:
        groups = [[c('combo_attack_01', 'chain'), c('combo_attack_02', 'chain'), c('combo_attack_03', 'chain'), c('combo_attack_04')], [g(12)],
                  [c('final_attack_04')], [g(12)], hold, [g(12)], block, [g(12)], [c('final_attack_05')], [g(12)]]
    out, n = [g(24)], 24
    for grp in groups:
        if any(e[0] != 'guard' and not F.built(weapon, e[0]) for e in grp):
            continue
        if out[-1][0] == 'guard' and grp[0][0] == 'guard':
            continue
        if n > F.MAX_FR and grp[0][0] != 'guard':
            break
        out += grp
        n += sum(e[1] or 25 for e in grp)
    if out[-1][0] != 'guard':
        out.append(g(16))
    return out


F.plan = plan


def film(a):
    res = opt(a, '--res', '640,720')
    out = os.path.abspath(opt(a, '--out', os.path.join(F.MEDIA_MP4, 'blunt_baton_game_vs_utpp_1x.mp4')))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    ex = ['--title-size', '0.075', '--title-align', 'center', '--label-size', '0.04']
    mk, info = [], dict(out=out, segments={})
    for weapon in WEAPONS:
        tl = {v: F.timeline(v, weapon) for v in ('van', 'new')}
        T = min(len(tl[v][0]) for v in tl)
        assert len(tl['van'][0]) == len(tl['new'][0]) and np.allclose(tl['van'][1], tl['new'][1]), 'timelines / capsule paths differ'
        for v, title in (('van', 'Game Animations'), ('new', 'Ultimate Third Person Project')):
            mk.append(F.render(tl[v][0][:T], [''] * T, title, F.COL[v], 'film_%s_%s' % (weapon, v), weapon, res, 15, ex))
        info['segments'][weapon] = dict(frames=T, right_sources=tl['new'][2], sequence=[p[0] for p in plan(weapon)])
    NF.ffmpeg(['-i', mk[0], '-i', mk[1], '-i', mk[2], '-i', mk[3], '-filter_complex',
               '[0:v][1:v]hstack=inputs=2[a];[2:v][3:v]hstack=inputs=2[b];[a][b]concat=n=2:v=1:a=0,format=yuv420p[v]', '-map', '[v]', '-r', '30',
               '-c:v', 'libx264', '-crf', '20', '-preset', 'slow', '-movflags', '+faststart', out])
    info.update(frames=sum(s['frames'] for s in info['segments'].values()), bytes=os.path.getsize(out))
    info['seconds'] = round(info['frames'] / 30.0, 2)
    MC.jdump(info, os.path.join(F.OUT, 'film_info_blunt_baton.json'))
    print('FILM', json.dumps(info))


def sheet(a):
    out = os.path.abspath(opt(a, '--out', os.path.join(F.MEDIA_PNG, 'blunt_baton_keyframes_game_vs_npc1.png')))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    groups = {}
    pref = {'blunt': ('combo_attack_01', 'heavy_attack_01', 'final_attack_03', 'combo_attack_03'),
            'baton': ('combo_attack_02', 'final_attack_05', 'heavy_attack_01', 'combo_attack_01')}
    for weapon in WEAPONS:
        clips = [c for c in pref[weapon] if F.built(weapon, c)][:2]
        items = []
        for clip in clips:
            T = len(F.load('new', weapon, clip)[0])
            imp = F.impact(weapon, clip)
            frs = [0, max(0, imp - 3), imp, min(T - 1, imp + 4), min(T - 1, imp + 12), T - 1]
            for v in ('van', 'new'):
                items += [(v, clip, f, '%s %s %s f%d' % ('GAME' if v == 'van' else 'NPC1', weapon, SHORT(clip), f)) for f in frs]
        groups[weapon] = items
    F.stills(groups, 'sheet_keyframes', '', out)


def worst(a):
    rig = BC.RIG
    out = os.path.abspath(opt(a, '--out', os.path.join(F.MEDIA_PNG, 'blunt_baton_%s_worst_frames.png' % rig)))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    top = int(opt(a, '--top', '12'))
    groups, notes = {}, {}
    for weapon in WEAPONS:
        st = BC.player_set(weapon)
        _, p0 = UC.knuckle_in_wr(MC.world_all(BC.vanilla(st, 'melee_idlepose')[0])[:1])
        rad0 = float(np.linalg.norm(p0[0, :2]) * 100)
        rows = []
        for fn in sorted(os.listdir(os.path.join(F.OUT, 'clips'))):
            if not (fn.startswith(st + '__') and fn.endswith('.npc1.npz')):
                continue
            clip = fn[len(st) + 2:-len('.npc1.npz')]
            W = F.load('new', weapon, clip)[0]
            th = np.load(os.path.join(F.OUT, 'clips', fn.replace('.npz', '.twohand.npz')))
            m, feat = NF.gate_margin(W)
            fm = int(np.argmin(m))
            cnt, clr = BC.blade_pen(W, weapon, th['w'])
            fb = int(np.argmin(clr))
            _, p = UC.knuckle_in_wr(W)
            dev = np.abs(np.linalg.norm(p[:, :2], axis=1) * 100 - rad0)
            fg = int(np.argmax(dev))
            inf = json.load(open(os.path.join(F.OUT, 'clips', fn.replace('.npz', '.info.json'))))
            rows.append(dict(clip=clip, min_margin_frame=fm, min_margin=round(float(m[fm]), 3), feature=str(feat[fm]),
                             blade_clear_frame=fb, blade_clear_cm=round(float(clr[fb]), 1), blade_pen_frames=int((cnt > 0).sum()),
                             grip_dev_frame=fg, grip_dev_cm=round(float(dev[fg]), 2), two_hand_frames=int((th['w'] > 0.5).sum()),
                             impact=inf.get('impact_vanilla')))
        notes[weapon] = rows
        pick = (sorted(rows, key=lambda r: r['min_margin'])[:top // 3] + sorted(rows, key=lambda r: r['blade_clear_cm'])[:top // 3]
                + sorted(rows, key=lambda r: -r['grip_dev_cm'])[:top // 3])
        seen, items = set(), []
        for r in pick:
            for f, why in ((r['min_margin_frame'], 'm%.2f %s' % (r['min_margin'], r['feature'][:14])),
                           (r['blade_clear_frame'], 'wpn %.1fcm' % r['blade_clear_cm']), (r['grip_dev_frame'], 'grip %.1fcm' % r['grip_dev_cm'])):
                if (r['clip'], f) in seen:
                    continue
                seen.add((r['clip'], f))
                items.append(('new', r['clip'], f, '%s %s f%d %s' % (weapon, SHORT(r['clip'])[:16], f, why)))
        groups[weapon] = items
    F.stills(groups, 'sheet_worst', '', out)
    MC.jdump(notes, os.path.join(F.OUT, 'worst_frames_%s.json' % rig))
    print('WORST', json.dumps({w: sorted(r, key=lambda x: x['min_margin'])[:4] for w, r in notes.items()}))


if __name__ == '__main__':
    a = [x for x in sys.argv[1:]]
    {'film': film, 'sheet': sheet, 'worst': worst}[a[0]](a)
