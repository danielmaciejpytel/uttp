"""1x side-by-side film of one packed third-person melee set (one weapon, one body): left "Game Animations" (the game's own clips of
the set on the third-person body), right "Ultimate Third Person Project" (the clip arrays the set was exported from).

  python -I -B melee_installed_film.py check --rig woman|man --weapon <w> --input <dir> --manifest <json> [--sha <input-sha256.txt>]
                                               [--fists-pack] --work <dir>
  python -I -B melee_installed_film.py film  (same options) [--res 640,720] --out <mp4>
  python -I -B melee_installed_film.py seams (same options)      steps of the body joints at every clip boundary of the sequence, both sides

  weapons: knife, katana, blunt (one-handed blunt), baton, fists, bat (two-handed blunt)
  --input     folder of <set>__<clip>.npc1.npz (+ .npc1.info.json) the set was exported from
  --manifest  stage manifest (sets -> mod_clips); with --fists-pack every array of --input counts as packed
  --sha       input-sha256.txt of the export run: every array read for the right side must match it (the film is refused otherwise)
  --work      output folder: _tmp (skin caches, MKV), film_info_<weapon>_<body>.json

Sequence: guard, draw (melee_equip), light combo (combo_attack_01..m, each cut at the game clip's recover start + crossfade as the game
switches), the final attack that follows combo m (final_attack_0(m+1)), strong attack (hold_start_01, hold_loop_01, heavy_attack_01),
block start / hold / end (not for the knife), one hit reaction (parried_left), guard.  A clip that is not in the packed set plays the
game's own clip on the right as well (listed in the info file).  Before rendering, every packed clip used is checked with the base
anatomy gate (anatomy_gate_base.frame_gate, the builders' limits) on all of its frames; a failing clip refuses the film.  For two-hand
weapons (katana, bat) hand_cross_* is reported but does not refuse the film.  npc_blade_film.py is imported unchanged; its loader is
replaced in this process only.
"""
import hashlib
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_A = sys.argv[1:]
WEAPON = _A[_A.index('--weapon') + 1]
if WEAPON in ('blunt', 'baton'):
    import npc_blunt_common as UC  # noqa: E402,F401  (registers blunt / baton and the baton game-clip folder; consumes --rig)
import npc_blade_film as F  # noqa: E402

BC, NB, MC, NF, NR = F.BC, F.NB, F.MC, F.NF, F.NR
GB = NB.GB
RIG = BC.RIG
BODY = {'woman': 'female', 'man': 'male'}[RIG]
SETNAME = {'knife': 'knife', 'katana': 'katana', 'blunt': 'one_handed_blunt', 'baton': 'baton', 'fists': 'fists',
           'bat': 'two_handed_blunt'}[WEAPON]
ST = ('pwa_' if RIG == 'woman' else 'pma_') + SETNAME
if WEAPON in ('blunt', 'baton'):
    F.HANDLE.update(blunt=(-0.06, 0.05, 0.018), baton=(-0.09, 0.03, 0.016))
    F.BLADE_R.update(blunt=BC.BLADE['blunt'][2], baton=BC.BLADE['baton'][2])
if WEAPON == 'bat':
    BC.WEAPONS['bat'] = dict(player={'woman': 'pwa_two_handed_blunt', 'man': 'pma_two_handed_blunt'}, two_hand=True)
    F.HANDLE['bat'] = (-0.145, 0.21, 0.016)
    F.BLADE_R['bat'] = 0.030
    BC.BLADE['bat'] = (0.21, 0.718, 0.032)
if WEAPON == 'fists':
    BC.WEAPONS['fists'] = dict(player={'woman': 'pwa_fists', 'man': 'pma_fists'}, two_hand=False)
TWO_HAND = WEAPON in ('katana', 'bat')
opt = F.opt
INPUT = os.path.abspath(opt(_A, '--input', ''))
WORK = os.path.abspath(opt(_A, '--work', ''))
F.TMP = os.path.join(WORK, '_tmp', 'film_%s_%s' % (WEAPON, RIG))
F.GUARD = {'new': 'melee_idlepose', 'van': 'melee_idlepose'}      # the packed guard pose (guard_idle_loop is not a game clip name)


def mod_clips():
    if '--fists-pack' in _A:
        return sorted(f[len(ST) + 2:-len('.npc1.npz')] for f in os.listdir(INPUT) if f.startswith(ST + '__') and f.endswith('.npc1.npz'))
    m = json.load(open(opt(_A, '--manifest', ''), encoding='utf-8-sig'))
    return sorted(m['sets'][ST]['mod_clips'])


MOD = set(mod_clips())
OPENED = {}
_C = {}


def arr_path(clip):
    return os.path.join(INPUT, '%s__%s.npc1.npz' % (ST, clip))


def load(variant, weapon, clip):
    """-> (W [T,73,4,4], names, parents, source 'mod' | 'game', capsule y [T] in m (mod info; 0 for the game's clips))."""
    key = (variant, clip)
    if key not in _C:
        ref = np.load(arr_path('melee_idlepose'))
        p = arr_path(clip)
        cy = None
        if clip in MOD and os.path.exists(p.replace('.npz', '.info.json')):
            cy = np.asarray(json.load(open(p.replace('.npz', '.info.json'))).get('capsule_path_cm') or [0.0], float) / 100.0
        if variant == 'new' and clip in MOD:
            W, src = np.load(p)['W'][:, :73].astype(float), 'mod'
            OPENED[os.path.basename(p)] = p
        elif os.path.exists(MC.clip_path(ST, clip)):
            W, src = MC.world_all(BC.vanilla(ST, clip)[0])[:, :73].astype(float), 'game'
        else:
            return load(variant, weapon, 'melee_idlepose')
        if cy is None or len(cy) != len(W):
            cy = np.zeros(len(W))
        _C[key] = (W, list(ref['names']), ref['parents'], src, cy)
    return _C[key]


F.load = load
F.built = lambda weapon, clip: clip in MOD


def game_has(clip):
    return os.path.exists(MC.clip_path(ST, clip))


def combo_len():
    """largest m with combo_attack_01..m and final_attack_0(m+1) present in the game set (the final takes the slot after combo m)."""
    best = None
    for m in range(1, 6):
        if all(game_has('combo_attack_%02d' % k) for k in range(1, m + 1)) and game_has('final_attack_%02d' % (m + 1)):
            best = m
    return best


def plan(weapon):
    g = lambda n: ('guard', n, F.XF, 'cycle')
    c = lambda clip, mode='all', n=None, ov=F.XF: (clip, n, ov, mode)
    m = combo_len()
    out = [g(24), c('melee_equip'), g(12)]
    out += [c('combo_attack_%02d' % k, 'chain') for k in range(1, m + 1)] + [c('final_attack_%02d' % (m + 1)), g(12)]
    out += [c('hold_start_01'), c('hold_loop_01', 'cycle', 20, 1), c('heavy_attack_01', ov=1), g(12)]
    if weapon != 'knife':
        out += [c('block_start'), c('block', 'cycle', 30, 1), c('block_end', ov=1), g(12)]
    out += [c('parried_left'), g(16)]
    return [e for e in out if e[0] == 'guard' or game_has(e[0])]


F.plan = plan


def sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest().upper()


def sha_table():
    p = opt(_A, '--sha', '')
    if not p:
        return None
    t = {}
    for ln in open(p, encoding='utf-8-sig'):
        parts = ln.split()
        if len(parts) >= 3:
            t[os.path.basename(parts[-1].replace('\\', '/'))] = parts[0].upper()
    return t


def gate_clip(W):
    g = GB.frame_gate(W[:, :73], NB.RG, steps=True, al=NB.AL)
    bad = {k: int((v > 1e-9).sum()) for k, v in g['viol'].items() if (v > 1e-9).any()}
    blocking = {k: n for k, n in bad.items() if not (TWO_HAND and k.startswith('hand_cross'))}
    return dict(frames=len(W), fail_features=bad, blocking=blocking, margin_min=round(float(g['margin'].min()), 4))


def pops(W, top=8):
    """largest per-frame world-position step of any joint on the timeline (cm) with the frame / time."""
    P = W[:, :73, :3, 3] * 100.0
    st = np.linalg.norm(np.diff(P, axis=0), axis=2)          # [T-1, 73]
    mx, jx = st.max(1), st.argmax(1)
    order = np.argsort(-mx)[:top]
    return [dict(frame=int(i + 1), t_s=round((i + 1) / 30.0, 2), step_cm=round(float(mx[i]), 2), joint=str(NB.P.names[jx[i]])) for i in order]


def check():
    """-> info dict (sequence, sources, sha check, gate per packed clip); raises when a packed clip shown fails."""
    seq = plan(WEAPON)
    tl = {v: F.timeline(v, WEAPON) for v in ('van', 'new')}
    assert len(tl['van'][0]) == len(tl['new'][0]) and np.allclose(tl['van'][1], tl['new'][1]), 'timelines / capsule paths differ'
    names = [e[0] for e in seq]
    shown = sorted({F.GUARD['new'] if n == 'guard' else n for n in names})
    tab = sha_table()
    shas, mism = {}, []
    for b, p in sorted(OPENED.items()):
        shas[b] = sha(p)
        if tab is not None and tab.get(b) != shas[b]:
            mism.append(b)
    gates, refused = {}, []
    for clip in shown:
        if clip in MOD:
            gates[clip] = gate_clip(load('new', WEAPON, clip)[0])
            if gates[clip]['blocking']:
                refused.append(clip)
    # starts of each sequence entry on the timeline (frames) for the report
    starts, n = [], 0
    for clip, k, ov, mode in seq:
        cname = F.GUARD['new'] if clip == 'guard' else clip
        L = len(F.entry_frames('new', WEAPON, cname, k, mode))
        n0 = n if not starts else n - max(1, min(ov, L))
        starts.append(dict(clip=clip, start_s=round(n0 / 30.0, 2), frames=L,
                           right=('mod' if cname in MOD else 'game')))
        n = n0 + L
    tg = GB.frame_gate(tl['new'][0][:, :73], NB.RG, steps=True, al=NB.AL)
    tl_bad = [round(i / 30.0, 2) for i in np.nonzero(~tg['ok'])[0]]
    info = dict(weapon=WEAPON, set=ST, rig=RIG, frames=len(tl['new'][0]), seconds=round(len(tl['new'][0]) / 30.0, 2), sequence=starts,
                right_game_clips=[c for c in shown if c not in MOD], input=INPUT, sha_checked=tab is not None, sha_mismatch=mism,
                input_sha256=shas, gate=gates, refused=refused, timeline_gate_fail_s=tl_bad[:40], timeline_gate_fail_frames=len(tl_bad),
                pops_right=pops(tl['new'][0]), pops_left=pops(tl['van'][0], 4))
    return info, tl


def render(W, title, color, name, res):
    os.makedirs(F.TMP, exist_ok=True)
    base = os.path.join(F.TMP, name)
    NR.skin_W(W, NB.P.ref['rest'], NB.RIG, base)
    lj = base + '.labels.json'
    json.dump([''] * len(W), open(lj, 'w', encoding='utf-8'))
    mkv = base + '.mkv'
    ex = ['--title-size', '0.075', '--title-align', 'center', '--label-size', '0.04']
    if WEAPON == 'fists':
        cmd = [NR.BLENDER, '-b', '--factory-startup', '-P', os.path.join(HERE, 'npc_film_render.py'), '--', base, mkv, lj]
    else:
        wn = base + '.weapon.npy'
        np.save(wn, W[:, BC.WR].astype(np.float64))
        z0, z1, _ = BC.BLADE[WEAPON]
        cmd = [NR.BLENDER, '-b', '--factory-startup', '-P', os.path.join(HERE, 'npc_blade_film_render.py'), '--', base, mkv, lj,
               '--weapon', wn, '--blade', '%g,%g,%g' % (z0, z1, F.BLADE_R[WEAPON]), '--handle', '%g,%g,%g' % F.HANDLE[WEAPON]]
    cmd += ['--res', res, '--follow', '15', '--title', title, '--color', color] + ex
    p = subprocess.run(cmd, capture_output=True, text=True)
    if 'RENDERED' not in p.stdout:
        raise RuntimeError('blender failed: ' + (p.stdout + p.stderr)[-1500:])
    return mkv


FINGERS = ('Thumb', 'Index', 'Middle', 'Ring', 'Pinky')


def seams(info, tl):
    """per sequence boundary: largest world-position step (cm / frame) of a body joint (no fingers) in the crossfade window, right and
    left, and the largest step of that side inside the 10 frames before the window (what the clip itself moves)."""
    body = [i for i, n in enumerate(NB.P.names[:73]) if not any(f in str(n) for f in FINGERS)]
    rows = []
    for v in ('new', 'van'):
        P = tl[v][0][:, body][:, :, :3, 3] * 100.0
        st = np.linalg.norm(np.diff(P, axis=0), axis=2)
        for k, e in enumerate(info['sequence'][1:], 1):
            b = int(round(e['start_s'] * 30))
            lo, hi = max(0, b - 1), min(len(st), b + F.XF + 2)
            w = st[lo:hi]
            j = np.unravel_index(int(w.argmax()), w.shape)
            pre = st[max(0, lo - 10):lo]
            rows.append(dict(side='right' if v == 'new' else 'left', into=e['clip'], t_s=round((lo + j[0] + 1) / 30.0, 2),
                             step_cm=round(float(w.max()), 1), joint=str(NB.P.names[body[j[1]]]),
                             before_cm=round(float(pre.max()), 1) if len(pre) else None))
    return rows


def gaps(variant):
    """per sequence boundary, before the crossfade: largest body-joint distance (cm, root space, no fingers) and hand rotation (deg)
    between the last frame of the outgoing entry and the first frame of the incoming one."""
    from scipy.spatial.transform import Rotation as Rot
    names = [str(n) for n in NB.P.names[:73]]
    body = [i for i, n in enumerate(names) if not any(f in n for f in FINGERS)]
    hands = [names.index('LeftHand'), names.index('RightHand')]
    prev, rows = None, []
    for clip, n, ov, mode in plan(WEAPON):
        cname = F.GUARD[variant] if clip == 'guard' else clip
        W, _, par, _, _ = load(variant, WEAPON, cname)
        fr = F.entry_frames(variant, WEAPON, cname, n, mode)
        Wk = NF.fk73(NF.to_local(W, par)[[fr[0], fr[-1]]], par)
        if prev is not None:
            d = np.linalg.norm(prev[body, :3, 3] - Wk[0][body, :3, 3], axis=1) * 100.0
            ang = [float(np.degrees((Rot.from_matrix(prev[h, :3, :3]).inv() * Rot.from_matrix(Wk[0][h, :3, :3])).magnitude())) for h in hands]
            rows.append(dict(into=clip, gap_cm=round(float(d.max()), 1), joint=names[body[int(d.argmax())]],
                             hand_deg=[round(a, 1) for a in ang]))
        prev = Wk[1]
    return rows


def extras(info, tl):
    """right timeline: weapon clearance, foot slide on planted runs, failing gate features; gaps before the crossfade, both sides."""
    out = dict(gaps_right=gaps('new'), gaps_left=gaps('van'))
    for v, key in (('new', 'weapon_clearance'), ('van', 'weapon_clearance_left')):
        out[key] = clearance(tl[v][0])
    return foot_gate(tl['new'][0], [str(n) for n in NB.P.names[:73]], out)


def clearance(W):
    try:
        if WEAPON in ('blunt', 'baton'):
            cnt, clr = UC.blade_pen_release(W, WEAPON)[:2]
        elif WEAPON != 'fists':
            cnt, clr = BC.blade_pen(W, WEAPON)[:2]
        else:
            return None
        clr = np.asarray(clr, float)
        f = int(np.argmin(clr))
        return dict(min_cm=round(float(clr[f]), 1), t_s=round(f / 30.0, 2), frames_inside=int((clr < 0).sum()),
                    t_inside_s=[round(i / 30.0, 2) for i in np.nonzero(clr < 0)[0]][:20],
                    proxy='cylinder along WeaponRight (drawing proxy)')
    except Exception as e:                              # noqa: BLE001  (clearance is a report extra)
        return dict(error=repr(e)[:200])


def foot_gate(W, names, out):
    feet = {}
    for side in ('Left', 'Right'):
        P = W[:, names.index(side + 'Foot'), :3, 3] * 100.0
        up = 2                                                               # Z is up
        h = P[:, up]
        hor = np.delete(P, up, axis=1)
        planted = h <= h.min() + 2.0
        best, run, t0, bt = 0.0, 0.0, 0, 0
        for i in range(1, len(P)):
            if planted[i] and planted[i - 1]:
                run += float(np.linalg.norm(hor[i] - hor[i - 1]))
                if run > best:
                    best, bt = run, t0
            else:
                run, t0 = 0.0, i
        feet[side] = dict(max_slide_cm=round(best, 1), run_start_s=round(bt / 30.0, 2))
    out['foot_slide'] = feet
    tg = GB.frame_gate(W[:, :73], NB.RG, steps=True, al=NB.AL)
    bad = {}
    for k, v in tg['viol'].items():
        idx = np.nonzero(v > 1e-9)[0]
        if len(idx):
            bad[k] = [round(i / 30.0, 2) for i in idx][:12]
    out['timeline_gate_features'] = bad
    return out


def main():
    cmd = _A[0]
    info, tl = check()
    if cmd == 'seams':
        p = os.path.join(WORK, 'seams_%s_%s.json' % (WEAPON, BODY))
        MC.jdump(dict(window=seams(info, tl), **extras(info, tl)), p)
        print('SEAMS', p)
        return
    jp = os.path.join(WORK, 'film_info_%s_%s.json' % (WEAPON, BODY))
    os.makedirs(WORK, exist_ok=True)
    if info['sha_mismatch'] or info['refused'] or cmd == 'check':
        MC.jdump(info, jp)
        print('CHECK', json.dumps(dict(refused=info['refused'], sha_mismatch=info['sha_mismatch'], seconds=info['seconds'])))
        if info['sha_mismatch'] or info['refused']:
            sys.exit(2)
        return
    out = os.path.abspath(opt(_A, '--out', ''))
    assert out.endswith('.mp4') and not os.path.exists(out), 'out missing or exists: ' + out
    res = opt(_A, '--res', '640,720')
    T = len(tl['new'][0])
    mk = [render(tl[v][0][:T], title, F.COL[v], 'film_%s' % v, res)
          for v, title in (('van', 'Game Animations'), ('new', 'Ultimate Third Person Project'))]
    NF.ffmpeg(['-i', mk[0], '-i', mk[1], '-filter_complex', '[0:v][1:v]hstack=inputs=2,format=yuv420p[v]', '-map', '[v]', '-r', '30',
               '-c:v', 'libx264', '-crf', '20', '-preset', 'slow', '-movflags', '+faststart', out])
    for m in mk:
        os.remove(m)
    info.update(out=out, bytes=os.path.getsize(out))
    MC.jdump(info, jp)
    print('FILM', json.dumps(dict(out=out, seconds=info['seconds'], bytes=info['bytes'])))


if __name__ == '__main__':
    main()
