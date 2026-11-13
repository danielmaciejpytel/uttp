"""Weapon films 2026-11-09 - worst-frame sheets of the UTPP output (skinned Female V), checked BEFORE the films are shown.

  weapons_sheet.py film2  -> Media\\png\\2026-11-09_weapon-films\\celowanie_strzelanie_najgorsze_klatki.png
  weapons_sheet.py k5     -> Media\\png\\2026-11-09_weapon-films\\barki_najgorsze_klatki.png
Needs the caches of weapons_film.py (world matrices <name>.world.npz, skin caches <name>.npz/.pc2, gun caches <name>_gun.*).
Per part, frames picked by: lowest anatomy-gate margin (melee anatomy_gate_base via npc_build, unchanged), smallest gun-to-body clearance
(proxy gun surface points vs skinned body vertices farther than 12 cm from both hands = no grip contact), sharpest elbow (interior angle),
largest recoil offset (film2) / largest elbow change of the shoulder pass (K5), the middle of the aim enter / exit transitions.
Two rows of views per part: front-right and behind-right (the in-game camera side).  Writes per-pick numbers to
Build\\work\\weapon-films-20261109\\worst_<what>.json.
"""
import json
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import weapons_common as WC  # noqa: E402
import weapons_film as F  # noqa: E402

sys.path.insert(0, os.path.join(WC.ROOT, 'Source', 'tools', 'melee'))
import npc_build as NB  # noqa: E402

TMP = WC.TMP
CAMS = ('1.45,1.85,1.5', '1.2,-1.9,1.65')


def gate_margin(W):
    """(as dualwield/dw_film.gate_margin) per frame the lowest normalised margin over the base-gate features that can move."""
    g = NB.GB.frame_gate(W[:, :73], NB.RG, steps=True, al=NB.AL)
    T = len(W)
    best, name = np.full(T, np.inf), np.array([''] * T, dtype=object)
    for k, v in g['feat'].items():
        # seg_len_dev*: bone length vs the reference rig - the feminine shoulder pass shortens the clavicle -> upper-arm offset ON PURPOSE
        # (2.5 cm per side) and the source pack retarget keeps its own lengths, so it fails on every UTPP frame and says nothing about a pose
        if k not in NB.AL or 'knee_dev' in k or 'elbow_dev' in k or 'shin_roll' in k or 'seg_len' in k:
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


def pc2(path, nv):
    return np.fromfile(path, dtype='<f4', offset=32).reshape(-1, nv, 3)


def gun_points(name, n=6):
    """dense points on the proxy gun boxes per frame from the gun cache (box corners -> face grids)."""
    g = np.load(os.path.join(TMP, name + '_gun.npz'))
    V = pc2(os.path.join(TMP, name + '_gun.pc2'), len(g['rest'])).astype(float)
    u = np.linspace(0, 1, n)
    pts = []
    for b in range(V.shape[1] // 8):
        c = V[:, b * 8:(b + 1) * 8]          # corner order of weapons_film.box: x major, then y, then z
        o, ex, ey, ez = c[:, 0], c[:, 4] - c[:, 0], c[:, 2] - c[:, 0], c[:, 1] - c[:, 0]
        for a in u:
            for bb in u:
                for e1, e2, e3 in ((ex, ey, ez), (ey, ez, ex), (ez, ex, ey)):
                    for s in (0.0, 1.0):
                        pts.append(o + a * e1 + bb * e2 + s * e3)
    return np.stack(pts, 1)


SKIP_SEG = ('Root', 'Trajectory', 'reference_joint', 'WeaponLeft', 'WeaponRight', 'LeftEye', 'RightEye')


def gun_clearance(name, W, part=False):
    """per frame: min distance (m) from the proxy gun surface to skinned body vertices that do NOT belong to a forearm, hand or finger
    (each vertex is owned by its nearest bone segment of the 73-joint skeleton); part=True also returns the owner of the closest vertex."""
    sk = np.load(os.path.join(TMP, name + '.npz'))
    B = pc2(os.path.join(TMP, name + '.pc2'), len(sk['rest'])).astype(float)
    G = gun_points(name)
    P = WC.player()
    segs = [(int(P.par[j]), j) for j in range(73) if P.par[j] >= 0 and P.names[j] not in SKIP_SEG and P.names[int(P.par[j])] not in SKIP_SEG]
    # segment (parent, child): excluded = elbow-to-wrist (child *Hand), palm/fingers and anything below the elbow (parent *ForeArm* / *Hand*);
    # the upper arm (RightArm>RightForeArm) and the clavicle/shoulder segment (RightShoulder>RightArm) stay in the check.
    grip = np.array(['Hand' in P.names[j] or 'ForeArm' in P.names[i] or 'Hand' in P.names[i] for i, j in segs])
    u = np.linspace(0, 1, 9)
    out, who = np.zeros(len(W)), [''] * len(W)
    for f in range(len(W)):
        pts = np.concatenate([W[f, a, :3, 3][None] + u[:, None] * (W[f, b, :3, 3] - W[f, a, :3, 3])[None] for a, b in segs])
        owner = np.repeat(np.arange(len(segs)), len(u))
        _, k = cKDTree(pts).query(B[f])
        keep = ~grip[owner[k]]
        d, kk = cKDTree(B[f][keep]).query(G[f])
        out[f] = d.min()
        if part:
            si = segs[owner[k[keep]][kk[int(np.argmin(d))]]]
            who[f] = P.names[si[0]] + '>' + P.names[si[1]]
    return (out, who) if part else out


def violations(W):
    """frames outside the gate range per feature (seg_len excluded, see gate_margin)."""
    g = NB.GB.frame_gate(W[:, :73], NB.RG, steps=True, al=NB.AL)
    res = {}
    for k, v in g['feat'].items():
        if k not in NB.AL or 'seg_len' in k:
            continue
        lo, hi = (float(x) for x in NB.AL[k])
        f = np.asarray(v, float).reshape(-1)
        bad = (f < lo) | (f > hi)
        if bad.any():
            res[k] = dict(frames=int(bad.sum()), min=round(float(f.min()), 1), max=round(float(f.max()), 1), range=[round(lo, 1), round(hi, 1)])
    return res


def elbow_angle(W):
    ix = WC.player().ix
    res = []
    for s in ('Left', 'Right'):
        u, l, h = (W[:, ix[s + n], :3, 3] for n in ('Arm', 'ForeArm', 'Hand'))
        a, b = u - l, h - l
        c = np.einsum('ti,ti->t', a, b) / (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1))
        res.append(np.degrees(np.arccos(np.clip(c, -1, 1))))
    return np.min(res, 0)


def picks_for(name, W, extra):
    m, feat = gate_margin(W)
    clr, who = gun_clearance(name, W, part=True)
    el = elbow_angle(W)
    viol = violations(W)
    p = [('gate %.2f %s' % (m.min(), str(feat[int(np.argmin(m))])[:16]), int(np.argmin(m))),
         ('gun-body %.1f cm' % (clr.min() * 100), int(np.argmin(clr))),
         ('elbow %.0f deg' % el.min(), int(np.argmin(el)))] + extra
    seen, out = set(), []
    for why, f in p:
        if f in seen:
            continue
        seen.add(f)
        out.append((why, f))
    stats = dict(gate_margin_min=round(float(m.min()), 3), gate_feature=str(feat[int(np.argmin(m))]), gate_violated_frames=int(np.sum(m < 0)),
                 gun_body_clearance_cm_min=round(float(clr.min() * 100), 2), gun_body_under_1cm_frames=int(np.sum(clr < 0.01)),
                 gun_body_closest_part=who[int(np.argmin(clr))],
                 gun_body_under_1cm_parts=sorted(set(who[i] for i in np.where(clr < 0.01)[0])),
                 elbow_interior_deg_min=round(float(el.min()), 1), gate_violations=viol)
    return out, stats


def sheet(parts, out_png, what, res='360,420', cols=8):
    """parts: [(tag, name, W, extra picks)] -> one png, rows of views per part (front-right / behind-right)."""
    Ws, labels, notes, rows = [], [], {}, []
    for tag, name, W, extra in parts:
        pk, st = picks_for(name, W, extra)
        notes[tag] = dict(stats=st, picks={why: f for why, f in pk})
        rows.append(len(pk))
        for why, f in pk[:cols]:
            Ws.append(W[f:f + 1])
            labels.append('%s f%d %s' % (tag, f, why))
        for _ in range(cols - min(cols, len(pk))):     # pad the row with the last pick (keeps one part per row)
            Ws.append(W[pk[-1][1]:pk[-1][1] + 1])
            labels.append('')
    Wc = np.concatenate(Ws)
    mk = []
    boxes = F.RIFLE if 'ajax' in what else F.PISTOL
    for i, cam in enumerate(CAMS):
        mk.append(F.render(Wc, '', '0.25,0.62,0.45', 'sheet_%s_%d' % (what, i), res, cam, 1.15, 36, boxes, follow=1, labels=labels,
                           title_size='0.05'))
    return Wc, mk, notes


def tile(mk, n_rows, cols, out_png):
    t = 'tile=%dx%d:padding=3:color=white' % (cols, n_rows)
    F.ffmpeg(['-i', mk[0], '-i', mk[1], '-filter_complex', '[0:v]%s[a];[1:v]%s[b];[a][b]vstack=inputs=2[v]' % (t, t), '-map', '[v]',
              '-frames:v', '1', out_png])


def film2(a):
    out = os.path.join(WC.MEDIA_PNG, 'celowanie_strzelanie_najgorsze_klatki.png')
    info = {}
    for key, sch in (('nue', F.NUE), ('ajax', F.AJAX)):
        name = 'f2_%s_utpp' % key
        z = np.load(os.path.join(TMP, name + '.world.npz'))
        W = z['W'].astype(float)
        rec = int(np.argmax(z['recoilT']))
        extra = [('recoil max %.1f cm' % (z['recoilT'][rec] * 100), rec), ('aim enter', int(round((sch['aim_on'] + 0.1) * 30))),
                 ('aim exit', int(round((sch['aim_off'] + 0.15) * 30))), ('last', len(W) - 1)]
        Wc, mk, notes = sheet([(key, name, W, extra)], out, 'f2_' + key + ('_ajax' if key == 'ajax' else ''))
        info[key] = dict(notes=notes, mkv=mk)
    # two parts -> one png: nue (2 view rows) above ajax (2 view rows)
    t = 'tile=8x1:padding=3:color=white'
    F.ffmpeg(['-i', info['nue']['mkv'][0], '-i', info['nue']['mkv'][1], '-i', info['ajax']['mkv'][0], '-i', info['ajax']['mkv'][1],
              '-filter_complex', '[0:v]%s[a];[1:v]%s[b];[2:v]%s[c];[3:v]%s[d];[a][b][c][d]vstack=inputs=4[v]' % (t, t, t, t), '-map', '[v]',
              '-frames:v', '1', out])
    WC.jdump(dict(out=out, rows='nue front-right, nue behind-right, ajax front-right, ajax behind-right',
                  parts={k: v['notes'] for k, v in info.items()}), os.path.join(WC.WORK, 'worst_film2.json'))
    print('SHEET', out, json.dumps({k: v['notes'] for k, v in info.items()})[:2500])


def k5(a):
    out = os.path.join(WC.MEDIA_PNG, 'barki_najgorsze_klatki.png')
    parts = []
    for key, fem, title in F.K5_VARIANTS:
        name = 'k5_' + key
        z = np.load(os.path.join(TMP, name + '.world.npz'))
        W = z['W'].astype(float)
        d = np.genfromtxt(os.path.join(TMP, name + '.diag.csv'), delimiter=',', names=True, dtype=None, encoding='utf-8')
        ef = np.max(np.abs([d['elbowFlexL'], d['elbowFlexR']]), 0)
        em = np.max([d['elbowMoveL'], d['elbowMoveR']], 0)
        extra = [('elbow change %.1f deg' % ef.max(), int(np.argmax(ef))), ('elbow move %.1f cm' % (em.max() * 100), int(np.argmax(em))),
                 ('low pose', 30), ('aim', 85), ('walk', 250)]
        parts.append((key, name, W, extra))
    Ws, labels, notes = [], [], {}
    cols = 8
    for tag, name, W, extra in parts:
        pk, st = picks_for(name, W, extra)
        notes[tag] = dict(stats=st, picks={why: f for why, f in pk})
        for why, f in (pk + [pk[-1]] * cols)[:cols]:
            Ws.append(W[f:f + 1])
            labels.append('%s f%d %s' % (tag, f, why))
    Wc = np.concatenate(Ws)
    mk = [F.render(Wc, '', '0.25,0.62,0.45', 'sheet_k5_%d' % i, '360,420', cam, 1.2, 40, F.PISTOL, follow=1, labels=labels, title_size='0.05')
          for i, cam in enumerate(('0.45,2.3,1.45', '0.9,-2.2,1.6'))]
    tile(mk, len(parts), cols, out)
    WC.jdump(dict(out=out, rows='top block: front view (global, none, low); bottom block: behind-right view', parts=notes),
             os.path.join(WC.WORK, 'worst_k5.json'))
    print('SHEET', out, json.dumps(notes)[:2500])


CLOSE = [('k5_global', 30, 'low pose, 2.5 cm/side'), ('k5_none', 30, 'low pose, none'), ('k5_low', 30, 'low pose, 2 cm/side'),
         ('k5_global', 150, 'ready'), ('k5_global', 250, 'walk'), ('f2_nue_utpp', 80, 'nue aim'), ('f2_nue_utpp', 71, 'nue recoil max'),
         ('f2_ajax_utpp', 0, 'ajax ready'), ('f2_ajax_utpp', 90, 'ajax aim'), ('f2_ajax_utpp', 150, 'ajax burst')]
CLOSE_CAMS = ('-0.45,0.7,0.2', '-0.65,-0.35,0.25')   # left-hand close-ups: front-left-above, outside-behind (forearm / wrist twist)


def closeup(a):
    """left forearm / wrist / grip close-ups (gate: wrist_twist_L above the melee range in every UTPP frame) -> one png."""
    out = os.path.join(WC.MEDIA_PNG, 'rece_zblizenia_najgorsze_klatki.png')
    stacks = {'pistol': ([], []), 'rifle': ([], [])}
    for name, f, why in CLOSE:
        W = np.load(os.path.join(TMP, name + '.world.npz'))['W'].astype(float)
        k = 'rifle' if 'ajax' in name else 'pistol'
        stacks[k][0].append(W[f:f + 1])
        stacks[k][1].append('%s f%d %s' % (name.replace('_utpp', ''), f, why))
    mk = []
    for i, cam in enumerate(CLOSE_CAMS):
        pair = []
        for k, boxes in (('pistol', F.PISTOL), ('rifle', F.RIFLE)):
            Ws, labels = stacks[k]
            pair.append(F.render(np.concatenate(Ws), '', '0.25,0.62,0.45', 'sheet_close_%s_%d' % (k, i), '400,400', cam, 1.1, 50, boxes,
                                 follow=1, labels=labels, title_size='0.05', extra=['--focus', '2']))
        row = os.path.join(TMP, 'sheet_close_row_%d.mkv' % i)
        F.ffmpeg(['-i', pair[0], '-i', pair[1], '-filter_complex', '[0:v][1:v]concat=n=2:v=1:a=0[v]', '-map', '[v]', '-c:v', 'libx264', '-crf', '12',
                  row])
        mk.append(row)
    n = len(CLOSE)
    tile(mk, 1, n, out)
    print('SHEET', out)


if __name__ == '__main__':
    a = sys.argv[1:]
    {'film2': film2, 'k5': k5, 'closeup': closeup}[a[0]](a)
