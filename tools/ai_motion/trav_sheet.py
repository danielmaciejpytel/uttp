"""Contact sheet (PNG) of a generated clip: side view stick figure (PIL), floor line, optional obstacle box, contact marks.

python -I trav_sheet.py soma <soma.npz> <t0> <t1> <out.png> [--obstacle h_cm,y_from_cm,y_to_cm] [--title T] [--cols 8] [--frames a,b,c]
  soma: a scene soma npz (SOMA77, metres, Y up, travel +Z or any heading) - drawn in the plane (travel axis, height).
python -I trav_sheet.py cp <utppclip.json> <rig.json> <out.png> [...same options, obstacle in metres via --obstacle-m h,s0,s1]
  cp: a retargeted game-rig clip (FK of the rig, root Trajectory applied).
The horizontal axis of every cell is the travel direction of the whole clip, cells are centred on the pelvis; frames with a
foot contact (contact_l / contact_r curve or the height rule) carry a red (L) / blue (R) dot under the foot.
"""
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

SOMA_BONES = [('Hips', 'Spine1'), ('Spine1', 'Spine2'), ('Spine2', 'Chest'), ('Chest', 'Neck1'), ('Neck1', 'Head'),
              ('Chest', 'LeftShoulder'), ('LeftShoulder', 'LeftArm'), ('LeftArm', 'LeftForeArm'), ('LeftForeArm', 'LeftHand'),
              ('Chest', 'RightShoulder'), ('RightShoulder', 'RightArm'), ('RightArm', 'RightForeArm'), ('RightForeArm', 'RightHand'),
              ('Hips', 'LeftLeg'), ('LeftLeg', 'LeftShin'), ('LeftShin', 'LeftFoot'), ('LeftFoot', 'LeftToeBase'),
              ('Hips', 'RightLeg'), ('RightLeg', 'RightShin'), ('RightShin', 'RightFoot'), ('RightFoot', 'RightToeBase')]
CP_BONES = [('Hips', 'Spine'), ('Spine', 'Spine1'), ('Spine1', 'Spine2'), ('Spine2', 'Spine3'), ('Spine3', 'Neck'), ('Neck', 'Head'),
            ('Spine3', 'LeftShoulder'), ('LeftShoulder', 'LeftArm'), ('LeftArm', 'LeftForeArm'), ('LeftForeArm', 'LeftHand'),
            ('Spine3', 'RightShoulder'), ('RightShoulder', 'RightArm'), ('RightArm', 'RightForeArm'), ('RightForeArm', 'RightHand'),
            ('Hips', 'LeftUpLeg'), ('LeftUpLeg', 'LeftLeg'), ('LeftLeg', 'LeftFoot'), ('LeftFoot', 'LeftToeBase'),
            ('Hips', 'RightUpLeg'), ('RightUpLeg', 'RightLeg'), ('RightLeg', 'RightFoot'), ('RightFoot', 'RightToeBase')]


def soma_points(path, t0, t1):
    s = np.load(path)
    names = [str(x) for x in s['names']]
    P = s['pos'][t0:t1 + 1] if 'pos' in s.files and len(s['pos']) == len(s['G']) else None
    if P is None:
        from foot_pin import Skel
        import trav_lib as T
        sk = Skel(T.TP)
        P = np.array([sk.fk(s['G'][t], s['hips'][t]) for t in range(t0, t1 + 1)])
    # soma: Y up; horizontal plane XZ -> (u, v) with u along the clip's travel
    return names, P[:, :, [0, 2]], P[:, :, 1]


def cp_points(clip, rigp, t0, t1):
    from mmbake_base import load_rig, fk_model
    import mmbake_motion as m2
    c = json.load(open(clip))
    cfg = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'mmbake', 'config', 'default.json')))
    names, parents = load_rig(rigp, int(cfg['num_pose_bones']))
    cd = m2.ClipData(clip, cfg, names, parents)
    gp, _ = fk_model(cd.P, cd.Q, cd.parents)  # [N, B, 3] model space (z up)
    xy = m2.rot2(cd.T[:, 2], gp[:, :, :2].reshape(-1, 2).reshape(len(gp), -1, 2)[0]) if False else None
    P = np.zeros((len(gp), len(names), 3))
    for f in range(len(gp)):
        P[f, :, :2] = m2.rot2(np.full(len(names), cd.T[f, 2]), gp[f, :, :2]) + cd.T[f, :2]
        P[f, :, 2] = gp[f, :, 2] + (cd.RZ[f] if getattr(cd, 'RZ', None) is not None else 0.0)
    return names, P[t0:t1 + 1, :, :2], P[t0:t1 + 1, :, 2], c


def draw_sheet(names, UV, Z, bones, out, frames, cols, title, obstacle, contacts=None, scale=105.0):
    """UV [T,J,2] horizontal, Z [T,J] height (m). obstacle = (h, v0, v1) in the same (u-axis) frame, metres."""
    idx = {n: i for i, n in enumerate(names)}
    T = len(Z)
    hips = idx['Hips']
    # travel axis: pelvis displacement of the whole clip (fallback: +u)
    d = UV[-1, hips] - UV[0, hips]
    if np.linalg.norm(d) < 0.2:
        d = np.array([0.0, 1.0])
    d = d / np.linalg.norm(d)
    U = UV @ d  # [T,J]
    cw, ch = 240, int(opt_ch) if False else 330
    rows = (len(frames) + cols - 1) // cols
    img = Image.new('RGB', (cols * cw, rows * ch + 28), (250, 250, 250))
    dr = ImageDraw.Draw(img)
    dr.text((6, 6), title, fill=(0, 0, 0))
    z0 = float(Z[:, [idx.get('LeftToeBase', 0), idx.get('RightToeBase', 0)]].min()) if 'LeftToeBase' in idx else 0.0
    for k, f in enumerate(frames):
        cx, cy = (k % cols) * cw, (k // cols) * ch + 28
        ox = U[f, hips]
        gy = cy + ch - 40
        dr.line([cx, gy, cx + cw, gy], fill=(160, 160, 160))
        if obstacle:
            h, a, b = obstacle[:3]
            if len(obstacle) == 4:  # (h, apex frame, depth, 'rel'): footprint centred on the hips at the apex frame
                h, fa, dep, _ = obstacle
                a, b = U[fa, hips] - dep / 2, U[fa, hips] + dep / 2
            x0, x1 = cx + cw / 2 + (a - ox) * scale, cx + cw / 2 + (b - ox) * scale
            dr.rectangle([x0, gy - h * scale, x1, gy], fill=(225, 190, 150), outline=(150, 100, 60))
        for a, b in bones:
            if a in idx and b in idx:
                ia, ib = idx[a], idx[b]
                p = (cx + cw / 2 + (U[f, ia] - ox) * scale, gy - (Z[f, ia] - z0) * scale)
                q = (cx + cw / 2 + (U[f, ib] - ox) * scale, gy - (Z[f, ib] - z0) * scale)
                col = (200, 40, 40) if 'Left' in a + b else (40, 60, 200) if 'Right' in a + b else (30, 30, 30)
                dr.line([p, q], fill=col, width=3)
        if contacts is not None:
            for side, col, off in (('Left', (200, 40, 40), -6), ('Right', (40, 60, 200), 6)):
                if contacts[side][f] > 0.5:
                    fi = idx[side + 'Foot']
                    x = cx + cw / 2 + (U[f, fi] - ox) * scale + off
                    dr.ellipse([x - 4, gy + 8, x + 4, gy + 16], fill=col)
        dr.text((cx + 4, cy + 2), 'f%d' % f, fill=(0, 0, 0))
    img.save(out)


def main(a):
    mode = a[0]
    opt = lambda k, d=None: a[a.index(k) + 1] if k in a else d
    title = opt('--title', os.path.basename(a[1]))
    cols = int(opt('--cols', '8'))
    obstacle = None
    if '--obstacle' in a:
        h, v0, v1 = (float(x) for x in opt('--obstacle').split(','))
        obstacle = (h / 100.0, v0 / 100.0, v1 / 100.0)
    if mode == 'cp':
        import trav_check as C
        clip, rig, out = a[1], a[2], a[3]
        names, bi, P, cd = C.world_pts(clip, rig, False)
        J = json.load(open(clip))
        ct = {'Left': J['curves']['contact_l'], 'Right': J['curves']['contact_r']}
        n = len(P)
        ob = None
        if '--meta' in a:
            mt = json.load(open(opt('--meta')))
            om = mt['metrics'].get('obstacle')
            if om:
                ob = (om['h_m'], om['apex_frame'], om['depth_m'], 'rel')
        frames = [int(x) for x in opt('--frames').split(',')] if '--frames' in a else list(np.unique(np.linspace(0, n - 1, int(opt('--n', '16'))).astype(int)))
        draw_sheet(names, P[:, :, :2], P[:, :, 2], CP_BONES, out, frames, cols, title, ob, contacts=ct, scale=float(opt('--scale', '105')))
        print('SHEET', out)
        return
    if mode == 'soma':
        t0, t1, out = int(a[2]), int(a[3]), a[4]
        names, UV, Z = soma_points(a[1], t0, t1)
        n = len(Z)
        frames = [int(x) for x in opt('--frames').split(',')] if '--frames' in a else list(np.unique(np.linspace(0, n - 1, int(opt('--n', '16'))).astype(int)))
        draw_sheet(names, UV, Z, SOMA_BONES, out, frames, cols, title, obstacle)
    print('SHEET', out)


if __name__ == '__main__':
    main(sys.argv[1:])
