"""Per-frame gates of traversal data on the V rig (offline, read-only).  One process per rig (base_body reads UTPP_RIG once).

  python -I trav_gate_offline.py --rig woman|man --trav FILE.utpptrav --out OUT.json
      gates every montage of a format-2 traversal file exactly as the DLL plays it: P lines (pose) + R lines (Trajectory x y, root_z, yaw)
      over the baked montage segment.
  python -I trav_gate_offline.py --rig woman|man --clips DIR --list LIST.txt --out OUT.json
      gates retargeted --rtz clips (.utppclip.json, whole clip; no montage, no warp windows) - for clips without a montage.

Gates (per frame; summary per montage):
  anatomy   anatomy_gate_base.frame_gate (V base body reference + human limits, steps on).  NOTE: base_reference.json was built from the
            M_Neutral_Traversal_* / M_Neutral_Jump_* clips of out_sourcepack_set3_<rig> (base_body.RANGE_PATTERNS), so for the Neutral traversal
            clips the gate is partly circular (their own 0.5..99.5 percentiles + tolerance); not circular for the Relaxed style.
  spikes    npc_build.spikes_cm definition re-implemented: per body joint the frame step v; pop = v[t] - 2 max(v[t-1], v[t+1]); first / last step
            against their only neighbour (v0 - 2 v1, vN - 2 vN-1); world space AND root-relative (in the Trajectory + root_z frame). Limit 3 cm.
  obstacle  box from the data (rig scale, metres, travel axis u = facing of frame 0, s = distance along u):
              top H: authored height from the name (_1_0_ = 1.0 m, _2_5_ = 2.5 m in the source) x hip_ratio (like root_z); names without
                     a height (Catch_*): climb family = root_z at the hand-back frame, others = max root_z after the FrontLedge window end;
              front edge s_f: median over frames of s_root - Distance_From_Ledge * hip_ratio / 100 (UE cm, scaled like the Trajectory;
                     gives s_f = 0.000 for every clip that has the curve: the source pack clips are authored with the front edge at the origin);
                     spread reported; without the curve: hand rest on the top (wrist |v| < 0.5 m/s, H - 0.08 < z < H + 0.12 m, min s - 0.03 m);
                     last fallback = FrontLedge contact hand s - 0.10 m (ASSUMED);
              back edge: BackLedge contact hand + 0.03 m; Hurdle / Vault family without it = last root on the top (>= 0.9 max root_z),
                     min depth 0.20 m (ASSUMED); else open (platform);
              floor z = 0 before the obstacle (none for Catch_*); behind a hurdle = the root_z at the hand-back frame, none for the
                     vault-and-fall family, H for the climb family.
            contacts: FrontLedge / BackLedge contact hand (W line) wrist z - H at the window end (wrist joint above the palm, expected +0.01..+0.07);
                      planted-hand xy slide over the window end +-1 frame; Mantle family: ankles at the hand-back frame vs H (ankle rest
                      height above the sole subtracted).
            penetration: joints with radii (FORMAT_TRAVERSAL get-up clamp set: Hips 0.10, knees / ankles 0.05, toes 0.02, wrists 0.03, Spine3 0.10,
                      Head 0.09) inside [s_f, s_back] below H (depth = H - (z - r)); floor penetration (z - r < floor - 0.01);
                      body_max_cm = the same without the wrists (a gripping wrist sits at / just below the edge by design).
  root      entry: root speed of the first step, pop of the first root step; exit at the planner's hand-back frame (MotionTraversal.hpp:
            climb = ArrivalFrame(frontEnd, 0.03 m), hurdle = last warp window end, Vault_1_0 / Catch_Vault = VaultLandingFrame): planar and
            vertical speed in / out, pop of the root step at the hand-back frame.
"""
import argparse
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
_a = sys.argv
if '--rig' in _a:
    os.environ['UTPP_RIG'] = _a[_a.index('--rig') + 1]
sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'melee'))
import anatomy_gate_base as GB  # noqa: E402
import base_body as BB  # noqa: E402

CP = BB.CP
HIP_RATIO = {'woman': 1.092927, 'man': 1.031735}
RADII = {'Hips': 0.10, 'Spine3': 0.10, 'Head': 0.09, 'LeftLeg': 0.05, 'RightLeg': 0.05, 'LeftFoot': 0.05, 'RightFoot': 0.05,
         'LeftToeBase': 0.02, 'RightToeBase': 0.02, 'LeftHand': 0.03, 'RightHand': 0.03}
SPIKE_LIMIT_CM = 3.0
CLIMB = ('Mantle', 'ClimbStart')


def family(mname, action):
    if '_Catch_Mantle' in mname or '_Catch_Cliff' in mname:
        return 'climb'
    if '_Catch_Vault' in mname or '_Vault_' in mname:
        return 'vault_fall'
    if '_Catch_Hurdle' in mname or '_Hurdle_' in mname:
        return 'hurdle'
    if action in CLIMB or '_Mantle_' in mname or '_Climb_' in mname:
        return 'climb'
    return 'other'


def parse_trav(path):
    """-> (posebones, [montage dict(name, seq, action, fps, frames, rtz, windows [..], R [n,4], dfl [n] | None, P [n,B,7])])"""
    mons, cur, pb = [], None, None
    with open(path, 'r', encoding='ascii') as f:
        for ln in f:
            t = ln.split()
            if not t:
                continue
            k = t[0]
            if k == 'POSEBONES':
                pb = t[2:]
            elif k == 'M':
                cur = dict(name=t[1], seq=t[2], action=t[11], fps=float(t[12]), frames=int(t[13]), rtz=int(t[14]), windows=[], R=[], P=[], dfl=None)
            elif k == 'W':
                cur['windows'].append(dict(target=t[1], start_s=float(t[2]), end_s=float(t[3]), sf=int(t[4]), ef=int(t[5]), bone=t[15],
                                           off=(float(t[16]), float(t[17]), float(t[18]))))
            elif k == 'R':
                cur['R'].append([float(x) for x in t[1:5]])
            elif k == 'C':
                cur['dfl'] = None if t[2] == '-' else np.array([float(x) for x in t[2:]])
            elif k == 'P':
                cur['P'].append(np.array([float(x) for x in t[1:]]).reshape(-1, 7))
            elif k == 'E':
                cur['R'] = np.array(cur['R'])
                cur['P'] = np.array(cur['P'])
                mons.append(cur)
                cur = None
    return pb, mons


def clip_as_montage(path):
    c = json.load(open(path, encoding='utf-8'))
    names = c['bone_names']
    ti = names.index('Trajectory')
    P = np.array([fr['bones'] for fr in c['frames']], dtype=np.float64)
    rz = c.get('root_z') or [0.0] * c['num_frames']
    R = np.array([[fr['bones'][ti][0], fr['bones'][ti][1], rz[f], _yaw(fr['bones'][ti][3:7])] for f, fr in enumerate(c['frames'])])
    dfl = c.get('curves', {}).get('distance_from_ledge')
    m = dict(name=c['source_clip'], seq=c['source_clip'], action=_action(c['source_clip']), fps=float(c['fps']), frames=int(c['num_frames']),
             rtz=int('root_z' in c), windows=[], R=R, P=P, dfl=(np.array(dfl, float) if dfl else None))
    return names, m


def _yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _action(n):
    for key, act in (('_Catch_', 'Catch'), ('_Climb_Start', 'ClimbStart'), ('_Hurdle_', 'Hurdle'), ('_Vault_', 'Vault'), ('_Mantle_', 'Mantle')):
        if key in n:
            return act
    return 'Other'


def world(m, names):
    """W [n,73,4,4] world (game axes): Trajectory(x, y, root_z, yaw) * in-place FK (Root = identity), like world_from_utppclip + root_z."""
    rn, par = BB.rig()[0], BB.rig()[1]
    assert list(names[:73]) == list(rn), 'pose bone order differs from the rig'
    n = len(m['R'])
    L = np.zeros((n, 73, 4, 4))
    for f in range(n):
        for b in range(73):
            L[f, b] = CP.mat(m['P'][f, b, 0:3], m['P'][f, b, 3:7])
    Wm = CP.fk(par, L)
    T = np.tile(np.eye(4), (n, 1, 1))
    for f, (x, y, z, yaw) in enumerate(m['R']):
        c, s = math.cos(yaw), math.sin(yaw)
        T[f, :2, :2] = [[c, -s], [s, c]]
        T[f, :3, 3] = (x, y, z)
    W = T[:, None] @ Wm
    W[:, 0] = np.eye(4)
    W[:, 1] = T
    return W, T


def pops(P):
    """P [n,J,3] -> (max pop cm, frame, joint index) with the npc_build.spikes_cm rule (incl. first / last step)."""
    if len(P) < 3:
        return 0.0, -1, -1
    v = np.linalg.norm(np.diff(P, axis=0), axis=-1)
    sp = v[1:-1] - 2 * np.maximum(v[:-2], v[2:])
    best = (0.0, -1, -1)
    if sp.size:
        i = np.unravel_index(int(np.argmax(sp)), sp.shape)
        best = (float(sp[i]) * 100, int(i[0]) + 1, int(i[1]))
    e0 = v[0] - 2 * v[1]
    e1 = v[-1] - 2 * v[-2]
    for e, fr in ((e0, 0), (e1, len(P) - 1)):
        j = int(np.argmax(e))
        if e[j] * 100 > best[0]:
            best = (float(e[j]) * 100, fr, j)
    return max(best[0], 0.0), best[1], best[2]


def handback(m, fam):
    R = m['R']
    n = len(R)
    wins = sorted(m['windows'], key=lambda w: w['start_s'])
    front = [w for w in wins if w['target'] == 'FrontLedge']
    fe = front[0]['ef'] if front else 0
    if fam == 'climb':
        zmax = R[fe:, 2].max()
        f = next(i for i in range(fe, n) if R[i, 2] >= zmax - 0.03)
        return max(f, fe), 'ArrivalFrame'
    if fam == 'vault_fall':
        apex = fe
        for i in range(fe, n):
            if R[i, 2] > R[apex, 2]:
                apex = i
        for i in range(apex, n):
            if R[i, 2] <= R[0, 2] + 0.03:
                return i, 'VaultLandingFrame'
        return n - 1, 'VaultLandingFrame(none: last frame)'
    if wins:
        return wins[-1]['ef'], 'last window end'
    return n - 1, 'last frame (no windows)'


def gate_montage(m, names, rig, al, rg):
    W, T = world(m, names)
    n = len(W)
    bi = {x: i for i, x in enumerate(BB.rig()[0])}
    fam = family(m['name'], m['action'])
    res = dict(name=m['name'], seq=m['seq'], action=m['action'], family=fam, frames=n, rtz=m['rtz'])
    # anatomy
    g = GB.frame_gate(W, rg, steps=True, al=al)
    sm = GB.summary(g)
    bad = np.where(~g['ok'])[0]
    res['anatomy'] = dict(pass_frames=sm['pass_frames'], frames=n, fail_features=sm['fail_features'], margin_min=round(sm['margin_min'], 3),
                          fail_runs=_runs(bad), worst_frame=int(np.argmin(g['margin'])), worst_feature=str(g['worst'][int(np.argmin(g['margin']))]))
    # the base gate's shoulder_width_cm is a melee extra (+-3 cm around the melee base body, anatomy_gate_base.BASE_SHOULDER_TOL_CM);
    # traversal raises the arms overhead, so it is also reported without that one feature
    other = {k: v for k, v in g['viol'].items() if k != 'shoulder_width_cm'}
    ok2 = np.ones(n, bool)
    exc = np.zeros(n)
    for k, v in other.items():
        ok2 &= v <= 1e-9
        lo, hi = al.get(k, (0.0, 1.0)) if k in al else (0.0, 1.0)
        sc = max(abs(hi - lo) if hi - lo < 1e8 else 20.0, 5.0)
        exc = np.maximum(exc, v / sc)
    cnt2 = {k: int((v > 1e-9).sum()) for k, v in other.items() if (v > 1e-9).any()}
    res['anatomy']['pass_frames_wo_shoulder_width'] = int(ok2.sum())
    res['anatomy']['fail_features_wo_shoulder_width'] = dict(sorted(cnt2.items(), key=lambda kv: -kv[1])[:8])
    res['anatomy']['worst_frame_wo_shoulder_width'] = int(np.argmax(exc)) if (~ok2).any() else None
    res['anatomy']['worst_excess_wo_shoulder_width'] = round(float(exc.max()), 3)
    res['anatomy']['shoulder_width_cm_range'] = [round(float(np.min(g['feat']['shoulder_width_cm'])), 1), round(float(np.max(g['feat']['shoulder_width_cm'])), 1)] if 'shoulder_width_cm' in g['feat'] else None
    # spikes
    body = [i for i, x in enumerate(BB.rig()[0]) if x not in ('Root', 'Trajectory', 'reference_joint')]
    Pw = W[:, body][..., :3, 3]          # (not W[:, body, :3, 3]: numpy moves the advanced index first)
    Prel = (np.linalg.inv(T)[:, None] @ W)[:, body][..., :3, 3]
    sw, swf, swj = pops(Pw)
    sr, srf, srj = pops(Prel)
    res['spikes'] = dict(world_cm=round(sw, 2), world_frame=swf, world_joint=BB.rig()[0][body[swj]] if swj >= 0 else None,
                         rel_cm=round(sr, 2), rel_frame=srf, rel_joint=BB.rig()[0][body[srj]] if srj >= 0 else None,
                         ok=bool(max(sw, sr) <= SPIKE_LIMIT_CM))
    # root continuity
    R = m['R']
    fps = m['fps']
    st = np.linalg.norm(np.diff(R[:, :2], axis=0), axis=1)
    vz = np.diff(R[:, 2])
    hb, hb_rule = handback(m, fam)
    hb = int(min(max(hb, 1), n - 1))

    def pop1(i):
        if 0 < i < len(st) - 1:
            return float(st[i] - 2 * max(st[i - 1], st[i + 1])) * 100
        return None
    res['root'] = dict(entry_speed_ms=round(float(st[0] * fps), 2), entry_vz_ms=round(float(vz[0] * fps), 2),
                       entry_step_pop_cm=round(float(st[0] - 2 * st[1]) * 100, 2) if len(st) > 1 else None,
                       handback_frame=hb, handback_rule=hb_rule, handback_t_s=round(hb / fps, 3),
                       handback_speed_in_ms=round(float(st[hb - 1] * fps), 2), handback_speed_out_ms=(round(float(st[hb] * fps), 2) if hb < len(st) else None),
                       handback_vz_in_ms=round(float(vz[hb - 1] * fps), 2), handback_step_pop_cm=(round(pop1(hb - 1), 2) if pop1(hb - 1) is not None else None),
                       root_z_start=round(float(R[0, 2]), 3), root_z_max=round(float(R[:, 2].max()), 3), root_z_end=round(float(R[-1, 2]), 3),
                       root_z_handback=round(float(R[hb, 2]), 3), path_len_m=round(float(st.sum()), 3))
    # obstacle
    yaw0 = R[0, 3]
    u = np.array([-math.sin(yaw0), math.cos(yaw0)])
    s_root = R[:, :2] @ u
    wins = sorted(m['windows'], key=lambda w: w['start_s'])
    front = [w for w in wins if w['target'] == 'FrontLedge']
    backl = [w for w in wins if w['target'] == 'BackLedge']

    def hand_world(w):
        f = min(w['ef'], n - 1)
        return W[f, bi[w['bone']], :3, 3] if w['bone'] in bi else None
    fe = front[0]['ef'] if front else None
    # top height: authored (name "_1_0" / "_2_5" = metres in the source, x hip_ratio like root_z) else from the root track
    mh = re.search(r'_(\d)_(\d)_', m['name'])
    if fam == 'climb':
        H_root = float(R[hb, 2])
    else:
        a0 = fe if fe is not None else 0
        H_root = float(R[a0:, 2].max())
    H = float('%s.%s' % mh.groups()) * HIP_RATIO[rig] if mh else H_root
    H_src = 'name x hip_ratio' if mh else ('root_z at hand-back' if fam == 'climb' else 'max root_z after FrontLedge end')
    sf, sf_spread, sf_src = None, None, None
    if m['dfl'] is not None and len(m['dfl']) == n:
        k = HIP_RATIO[rig] / 100.0
        cand = s_root - m['dfl'] * k
        lim = fe if fe is not None else n
        cs = cand[:max(lim, 2)]
        sf, sf_spread, sf_src = float(np.median(cs)), float(np.percentile(cs, 95) - np.percentile(cs, 5)), 'Distance_From_Ledge'
    if sf is None:                      # no Distance_From_Ledge: where the hands rest on the top (probe_edge.py, 2026-11-09)
        rest_s = []
        for hand in ('LeftHand', 'RightHand'):
            Ph = W[:, bi[hand]][..., :3, 3]
            vh = np.r_[np.inf, np.linalg.norm(np.diff(Ph, axis=0), axis=1) * fps]
            # band H - 0.08 .. H + 0.12: a wrist on the top sits at / above it; hands hanging at the sides (man ~0.90 m) stay outside
            idx = np.where((vh < 0.5) & (Ph[:, 2] > H - 0.08) & (Ph[:, 2] < H + 0.12))[0]
            rest_s += [float(x) for x in Ph[idx, :2] @ u]
        if rest_s:
            sf, sf_src = min(rest_s) - 0.03, 'hand rest on top (|v| < 0.5 m/s, H - 0.08 < z < H + 0.12): min s - 0.03'
    if sf is None and front:
        hw = hand_world(front[0])
        sf, sf_src = float(hw[:2] @ u) - 0.10, 'ASSUMED FrontLedge hand - 0.10'
    sback, sb_src = None, None
    if fam in ('hurdle', 'vault_fall'):
        if backl:
            hw = hand_world(backl[0])
            sback, sb_src = float(hw[:2] @ u) + 0.03, 'BackLedge hand + 0.03'
        elif sf is not None:
            top = [f for f in range(fe or 0, n) if R[f, 2] >= 0.9 * H_root]
            sback = max(sf + 0.20, float(s_root[top[-1]]) if top else sf + 0.20)
            sb_src = 'ASSUMED last root on top (>= 0.9 max root_z), min depth 0.20'
    catch = '_Catch_' in m['name']
    floor_before = -1e9 if catch else 0.0                       # catch clips start airborne: no floor in front
    floor_after = -1e9 if fam == 'vault_fall' else (float(R[hb, 2]) if fam == 'hurdle' else H)  # Vault_1_0 / Catch_Vault fall on
    ob = dict(top_H_m=round(H, 3), top_src=H_src, top_root_m=round(H_root, 3), front_s_m=(round(sf, 3) if sf is not None else None),
              front_src=sf_src, front_spread_m=(round(sf_spread, 3) if sf_spread is not None else None),
              back_s_m=(round(sback, 3) if sback is not None else None), back_src=sb_src,
              floor_before_m=(None if floor_before < -1e8 else floor_before), floor_after_m=(None if floor_after < -1e8 else round(floor_after, 3)))
    # contacts
    con = {}
    for w in front[:1] + backl[:1]:
        if w['bone'] not in bi:
            continue
        f = min(w['ef'], n - 1)
        j = bi[w['bone']]
        a, b = max(0, f - 1), min(n - 1, f + 1)
        con[w['target']] = dict(hand=w['bone'], frame=f, wrist_minus_top_cm=round((float(W[f, j, 2, 3]) - H) * 100, 1),
                                hand_s_minus_front_cm=(round((float(W[f, j, :2, 3] @ u) - sf) * 100, 1) if sf is not None else None),
                                plant_slide_cm=round(float(np.linalg.norm(W[b, j, :3, 3] - W[a, j, :3, 3])) * 100, 1),
                                offset_z_m=round(w['off'][2], 3))
        # best hand-on-top frame of the contact hand inside the window (does the hand ever reach the top?)
        ws = max(0, w['sf'])
        zz = W[ws:f + 1, j, 2, 3] - H
        con[w['target']]['window_wrist_minus_top_cm_closest'] = round(float(zz[np.argmin(np.abs(zz))]) * 100, 1) if len(zz) else None
    if fam == 'climb':
        rest = BB.rig()[3]
        ank = {}
        for s_ in ('Left', 'Right'):
            j = bi[s_ + 'Foot']
            sole = float(rest[j, 2, 3]) - float(min(rest[bi['LeftToeBase'], 2, 3], rest[bi['RightToeBase'], 2, 3]))
            ank[s_] = round((float(W[hb, j, 2, 3]) - sole - H) * 100, 1)
        con['feet_on_top_at_handback_cm'] = ank
    ob['contacts'] = con
    # penetration: depth = shortest way out of the box (top, front face, back face); floor: feet below the floor of their side
    pen_max, pen_frames, pen_joint, floor_pen = 0.0, 0, None, 0.0
    pen_body, pen_body_joint = 0.0, None
    floor_pen_frames = 0
    pen_series = np.zeros(n)
    if sf is not None:
        for f in range(n):
            for nm, r in RADII.items():
                p_ = W[f, bi[nm], :3, 3]
                s = float(p_[:2] @ u)
                z = float(p_[2])
                inside = (s + r > sf) and (z - r < H) and (sback is None or s - r < sback)
                if inside:
                    d = min(H - (z - r), (s + r) - sf, (sback - (s - r)) if sback is not None else 1e9)
                    if d > 0.01:
                        pen_series[f] = max(pen_series[f], d)
                        if d > pen_max:
                            pen_max, pen_joint = d, '%s@%d' % (nm, f)
                        if nm not in ('LeftHand', 'RightHand') and d > pen_body:     # wrists grip the edge: reported separately
                            pen_body, pen_body_joint = d, '%s@%d' % (nm, f)
                elif nm in ('LeftFoot', 'RightFoot', 'LeftToeBase', 'RightToeBase'):
                    fl = floor_before if s + r <= sf else floor_after
                    d = fl - (z - r)
                    if d > 0.01:
                        floor_pen = max(floor_pen, d)
                        floor_pen_frames += 1
        pen_frames = int((pen_series > 0.02).sum())
    ob['penetration'] = dict(max_cm=round(pen_max * 100, 1), frames_gt2cm=pen_frames, worst=pen_joint, floor_max_cm=round(floor_pen * 100, 1),
                             floor_joint_frames=floor_pen_frames, body_max_cm=round(pen_body * 100, 1), body_worst=pen_body_joint)
    res['obstacle'] = ob
    return res, W


def _runs(idx):
    out = []
    for i in idx:
        if out and i == out[-1][1] + 1:
            out[-1][1] = int(i)
        else:
            out.append([int(i), int(i)])
    return out[:12]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rig', choices=('woman', 'man'))  # consumed by base_body at import (removed from sys.argv)
    ap.add_argument('--trav')
    ap.add_argument('--clips')
    ap.add_argument('--list')
    ap.add_argument('--out', required=True)
    ap.add_argument('--worst-npz', help='optional: save the world matrices of every montage (npz) for the media step')
    a = ap.parse_args()
    a.rig = a.rig or BB.RIG_NAME
    al = GB.allowed()
    rg = BB.rig_for_gate()
    items = []
    if a.trav:
        pb, mons = parse_trav(a.trav)
        items = [(pb, m) for m in mons]
    else:
        names = [x.strip() for x in open(a.list, encoding='utf-8') if x.strip() and not x.startswith('#')]
        for nm in names:
            items.append(clip_as_montage(os.path.join(a.clips, nm + '.utppclip.json')))
    out, keep = [], {}
    for names, m in items:
        r, W = gate_montage(m, names, a.rig, al, rg)
        out.append(r)
        if a.worst_npz:
            keep[m['name']] = W.astype(np.float32)
        an, sp, rt, ob = r['anatomy'], r['spikes'], r['root'], r['obstacle']
        print('%-56s %-10s an %3d/%3d %-22s spk w%5.1f r%5.1f | in %4.1f m/s hb %3d %-14s out %4.1f vz %5.1f | H %.2f pen %4.1f cm fl %4.1f | %s' % (
            r['name'][:56], r['family'], an['pass_frames'], an['frames'], (an['worst_feature'] if an['pass_frames'] < an['frames'] else '-')[:22],
            sp['world_cm'], sp['rel_cm'], rt['entry_speed_ms'], rt['handback_frame'], rt['handback_rule'][:14],
            rt['handback_speed_out_ms'] if rt['handback_speed_out_ms'] is not None else -1, rt['handback_vz_in_ms'], ob['top_H_m'],
            ob['penetration']['max_cm'], ob['penetration']['floor_max_cm'],
            ' '.join('%s:%s' % (k, v.get('wrist_minus_top_cm')) for k, v in ob['contacts'].items() if isinstance(v, dict) and 'wrist_minus_top_cm' in v)))
    summ = dict(rig=a.rig, source=a.trav or a.clips, items=len(out),
                anatomy_all_pass=sum(1 for r in out if r['anatomy']['pass_frames'] == r['anatomy']['frames']),
                anatomy_frames=[sum(r['anatomy']['pass_frames'] for r in out), sum(r['anatomy']['frames'] for r in out)],
                anatomy_frames_wo_shoulder_width=sum(r['anatomy']['pass_frames_wo_shoulder_width'] for r in out),
                anatomy_all_pass_wo_shoulder_width=sum(1 for r in out if r['anatomy']['pass_frames_wo_shoulder_width'] == r['anatomy']['frames']),
                spikes_ok=sum(1 for r in out if r['spikes']['ok']),
                penetration_gt2cm=sum(1 for r in out if r['obstacle']['penetration']['max_cm'] > 2.0),
                floor_pen_gt2cm=sum(1 for r in out if r['obstacle']['penetration']['floor_max_cm'] > 2.0))
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(dict(summary=summ, items=out), f, indent=1)
    if a.worst_npz:
        np.savez_compressed(a.worst_npz, **keep)
    print('SUMMARY', json.dumps(summ))


if __name__ == '__main__':
    main()
