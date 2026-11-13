"""Shared code of the NPC-based knife / katana third-person clips of V (2026-11-09): weapon config, NPC donors with the NPC grip, blade
geometry, blade penetration gate, vanilla clip info, survey / grip report.

  python -I npc_blade_common.py survey --weapon knife|katana [--rig woman|man]     tip paths of the vanilla attacks and of the NPC attack donors
  python -I npc_blade_common.py grip   --weapon knife|katana [--rig woman|man]     weapon bone vs hand: vanilla player vs NPC, two-hand frames
Run with Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I.  Reads only (Extracted\\game\\npc-melee, Build\\work\\melee-anim-20261107);
writes Build\\work\\melee-npc-blades-20261109\\survey\\*.json.  Imports npc_build / npc_retarget (W1's tools) unchanged.

Weapon bone (measured 2026-11-09, both rigs share the 152-joint player rig; the NPC rigs (71 joints) hold WeaponRight / WeaponLeft and the
fingers): the weapon hangs from WeaponRight (child of RightHand).  Blade axis = +Z of WeaponRight: the right-hand knuckle line Pinky1 -> Index1
runs along +Z in the vanilla and the NPC grips of both weapons, and the left hand of the two-handed katana grip sits on -Z (pommel side).
Blade proxy = capsule along +Z from BLADE[w][0] to BLADE[w][1] (m) with radius BLADE[w][2]; lengths are an assumption (no weapon mesh in the
project): knife 3..21 cm, katana 5..77 cm (blade + guard of a ~1 m katana).
Grip: the NPC grip of each clip is kept exactly (NPC WeaponRight local rotation AND translation, NPC fingers); the vanilla player grips differ
by 12-16 deg (idle) and the vanilla knife flips to a reverse grip in heavy / final / jump / sprint attacks (see grip report).
"""
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import npc_build as NB  # noqa: E402  (imports base_body: --rig is consumed there)
import npc_retarget as NR  # noqa: E402
import melee_common as MC  # noqa: E402

RIG = NB.RIG
P = NB.P
IX = P.ix
OUTW = os.path.join(MC.ROOT, 'Build', 'work', 'melee-npc-blades-20261109')
PREF = NR.NPC_PREFIX[RIG]
WEAPONS = {
    'knife': dict(player={'woman': 'pwa_knife', 'man': 'pma_knife'}, action='gang_knife_action_combat',
                  loco=('gang_knife_locomotion_combat',), two_hand=False),
    'katana': dict(player={'woman': 'pwa_katana', 'man': 'pma_katana'}, action='corpo_katana_action_combat',
                   loco=('corpo_katana_locomotion_combat', 'corpo_katana_locomotion_combat_blocking'), two_hand=True),
}
BLADE = {'knife': (0.03, 0.21, 0.015), 'katana': (0.05, 0.77, 0.020)}     # (start, tip, radius) m along +Z of WeaponRight
WR, WL = IX['WeaponRight'], IX['WeaponLeft']
HAND_EXCL = {s: [n for n in P.names if n.startswith(s + 'Hand')] for s in ('Left', 'Right')}


def npc_set(weapon, kind='action'):
    w = WEAPONS[weapon]
    return PREF + (w['action'] if kind == 'action' else kind)


def player_set(weapon):
    return WEAPONS[weapon]['player'][RIG]


# ------------------------------------------------------------------ raw NPC skeleton (exact NPC grip relations)
_RAW = {}


def raw_npc(set_name, clip):
    """NPC clip on the NPC's OWN skeleton (game axes, Root = identity): world W [T,N,4,4], locals lt / lr (game), name index."""
    key = (set_name, clip)
    if key not in _RAW:
        nsk = NR.skeleton(os.path.join(NR.NPC_DIR, set_name + '.anims.glb'))
        nlt, nlr, ex, dur = NR.sample_clip(nsk, clip)
        lt, lr = NR.locals_to_game(nlt, nlr)
        ri = nsk.index['Root']
        lt[:, ri] = 0
        lr[:, ri] = [0, 0, 0, 1.0]
        W = MC.fk_world(np.array(nsk.parent), lt, lr)
        _RAW[key] = dict(W=W, lt=lt, lr=lr, ix=dict(nsk.index))
    return _RAW[key]


def left_rel(raw):
    """left hand pose in the WeaponRight frame [T,4,4] (NPC skeleton) and the two-hand weight [T] (left palm on the hilt line)."""
    W, ix = raw['W'], raw['ix']
    rel = np.linalg.inv(W[:, ix['WeaponRight']]) @ W[:, ix['LeftHand']]
    palm = np.linalg.inv(W[:, ix['WeaponRight']]) @ np.concatenate([(W[:, ix['LeftHand'], :3, 3] + W[:, ix['LeftHandMiddle1'], :3, 3]) / 2,
                                                                    np.ones((len(W), 1))], 1)[..., None]
    p = palm[:, :3, 0]
    radial = np.linalg.norm(p[:, :2], axis=1)
    on = (radial < 0.075) & (p[:, 2] < -0.02) & (p[:, 2] > -0.24)
    return rel, on, p


# ------------------------------------------------------------------ donors with the NPC grip
SEAM_FADE = 6.0


def _rel_fade(rel, rel0, w):
    """4x4 relations [T] offset so that frame 0 equals rel0, the offset fading with w (1 at frame 0)."""
    dR = R.from_matrix(rel0[:3, :3]) * R.from_matrix(rel[0, :3, :3]).inv()
    rv = dR.as_rotvec()
    out = rel.copy()
    out[:, :3, :3] = (R.from_rotvec(rv[None] * w[:, None]) * R.from_matrix(rel[:, :3, :3])).as_matrix()
    out[:, :3, 3] = rel[:, :3, 3] + (rel0[:3, 3] - rel[0, :3, 3])[None] * w[:, None]
    return out


class BSeq:
    """concatenated NPC donor clips on V's rig (npc_retarget.Donor: NPC locals by name) + the NPC WeaponRight local TRANSLATION (exact grip),
    NPC root path continued across the parts, left-hand-in-weapon relation and two-hand flags of the NPC skeleton (katana grip IK)."""

    def __init__(self, donors, despike=True):
        lts, lrs, rts, rRs, rels, ons, self.parts = [], [], [], [], [], [], []
        off_t, off_R = np.zeros(3), np.eye(3)
        n0 = 0
        self.seam_raw_deg = []
        for k, d in enumerate(donors):
            set_name, clip = d[0], d[1]
            dn = NR.Donor(P, clip, set_name, False)
            raw = raw_npc(set_name, clip)
            lt = dn.lt.copy()
            lr = dn.lr.copy()
            lt[:, WR] = raw['lt'][:, raw['ix']['WeaponRight']]
            lt[:, WL] = raw['lt'][:, raw['ix']['WeaponLeft']]
            rel, on, _ = left_rel(raw)
            rR = np.einsum('ij,tjk->tik', dn.root_R[0].T, dn.root_R)
            rt = np.einsum('ij,tj->ti', dn.root_R[0].T, dn.root_t - dn.root_t[0])
            s = 0 if k == 0 else 1
            if k:       # NPC split clips (pN -> pN_miss, *_to_attack -> *_to_idle) do not always continue: the follow-up part starts on the
                # previous part's last pose (locals, hips, left-hand relation) and the offset fades out over SEAM_FADE frames
                self.seam_raw_deg.append(round(float(NB.rot_step_max(NB.qcont(np.stack([lrs[-1][-1], lr[0]])))), 1))
                w = 1.0 - NB.sstep(np.arange(len(lr)) / float(SEAM_FADE))
                lr = NB.offset_start(lr, lrs[-1][-1], w)
                lt = lt + (lts[-1][-1] - lt[0])[None] * w[:, None, None]
                rel = _rel_fade(rel, rels[-1][-1], w)
            lts.append(lt[s:])
            lrs.append(lr[s:])
            rels.append(rel[s:])
            ons.append(on[s:])
            rts.append((np.einsum('ij,tj->ti', off_R, rt) + off_t)[s:])
            rRs.append(np.einsum('ij,tjk->tik', off_R, rR)[s:])
            self.parts.append(dict(set=set_name, clip=clip, start=n0, frames=dn.n, phases=dn.phases, root_end_cm=np.round(rt[-1] * 100, 1).tolist(),
                                   two_hand_frames=int(on.sum())))
            off_t, off_R = rts[-1][-1].copy(), rRs[-1][-1].copy()
            n0 += dn.n - s
        self.lt = np.concatenate(lts)
        self.lr = NB.qcont(np.concatenate(lrs))
        self.rel_L = np.concatenate(rels)
        self.on2h = np.concatenate(ons)
        self.root_t, self.root_R = np.concatenate(rts), np.concatenate(rRs)
        self.n = len(self.lt)
        self.W = NB.fk(self.lt, self.lr)
        self.despiked = []
        while despike:      # one-frame out-and-back donor outliers -> halfway pose of the neighbours (local variant of npc_build.Seq despike)
            Pp = self.W[:, NB.BODY_IX][:, :, :3, 3]
            sag = (np.linalg.norm(Pp[1:-1] - (Pp[:-2] + Pp[2:]) / 2, axis=-1) - np.linalg.norm(Pp[2:] - Pp[:-2], axis=-1) / 2).max(1)
            t = int(np.argmax(sag)) + 1
            if sag[t - 1] <= NB.DESPIKE_CM / 100.0 or len(self.despiked) > 8:
                break
            self.lt[t] = (self.lt[t - 1] + self.lt[t + 1]) / 2
            self.lr[t] = NB.qslerp(self.lr[t - 1], self.lr[t + 1], np.full(self.lr.shape[1], 0.5))
            self.lr = NB.qcont(self.lr)
            self.W = NB.fk(self.lt, self.lr)
            self.despiked.append(dict(frame=t, sagitta_cm=round(float(sag[t - 1]) * 100, 1)))
        self.seam_deg = [round(float(NB.rot_step_max(self.lr[p['start'] - 1:p['start'] + 1])), 1) for p in self.parts[1:]]
        self.loop = self.parts[0]['phases'].get('loop')

    def hips_world(self, fr):
        h = self.W[:, IX['Hips'], :3, 3]
        hw = np.einsum('tij,tj->ti', self.root_R, h) + self.root_t
        return NB.interp_rows(hw, fr)

    def rel_at(self, tau):
        """left-hand-in-weapon relation (4x4) and the two-hand weight at fractional frames (nearest / linear)."""
        i0 = np.clip(np.floor(tau).astype(int), 0, self.n - 1)
        i1 = np.minimum(i0 + 1, self.n - 1)
        u = tau - i0
        Ra = R.from_matrix(self.rel_L[i0, :3, :3])
        Rb = R.from_matrix(self.rel_L[i1, :3, :3])
        qa, qb = Ra.as_quat(), Rb.as_quat()
        q = NB.qslerp(qa, qb, u)
        M = np.repeat(np.eye(4)[None], len(tau), 0)
        M[:, :3, :3] = R.from_quat(q).as_matrix()
        M[:, :3, 3] = self.rel_L[i0, :3, 3] * (1 - u[:, None]) + self.rel_L[i1, :3, 3] * u[:, None]
        w = self.on2h[i0].astype(float) * (1 - u) + self.on2h[i1].astype(float) * u
        return M, w


# ------------------------------------------------------------------ blade geometry
def blade_pts(W, weapon, n=9):
    """points along the blade proxy [T,n,3] (world of W) and the tip [T,3]."""
    z0, z1, _ = BLADE[weapon]
    zs = np.linspace(z0, z1, n)
    M = W[:, WR]
    pts = M[:, None, :3, 3] + zs[None, :, None] * M[:, None, :3, 2]
    return pts, pts[:, -1]


def tip(W, weapon):
    return blade_pts(W, weapon, 2)[1]


_PX = None


def proxy():
    global _PX
    if _PX is None:
        import redo_pen as RPN
        _PX = RPN.proxy()
    return _PX


def blade_pen(W, weapon, two_hand=None, detail=False):
    """blade proxy (capsule) vs V's body proxy vertices (redo_pen rigid proxy of the reference skin) on every frame.
    Excluded: the right hand (+ fingers) that holds the hilt, and the left hand on frames where it is on the hilt (two_hand [T] > 0.5).
    -> (vertices inside per frame [T], clearance per frame [T] in cm (min vertex distance to the blade segment - radius))
    detail: + (closest body point [T,3], closest blade point [T,3], label of the closest vertex [T])."""
    px = proxy()
    z0, z1, r = BLADE[weapon]
    names = np.array(px.names)
    lab = px.lab
    excl_R = np.isin(names[lab], HAND_EXCL['Right'])
    excl_L = np.isin(names[lab], HAND_EXCL['Left'])
    T = len(W)
    cnt, clr = np.zeros(T, int), np.zeros(T)
    bp, qp, lb = np.zeros((T, 3)), np.zeros((T, 3)), np.array([''] * T, dtype=object)
    for t in range(T):
        V = px.verts(W[t, :73])
        keep = ~excl_R
        if two_hand is not None and two_hand[t] > 0.5:
            keep &= ~excl_L
        V, L = V[keep], lab[keep]
        M = W[t, WR]
        a = M[:3, 3] + z0 * M[:3, 2]
        b = M[:3, 3] + z1 * M[:3, 2]
        ab = b - a
        u = np.clip(((V - a) @ ab) / float(ab @ ab), 0, 1)
        Q = a + u[:, None] * ab
        d = np.linalg.norm(V - Q, axis=1)
        k = int(np.argmin(d))
        cnt[t] = int((d < r).sum())
        clr[t] = (float(d[k]) - r) * 100
        bp[t], qp[t], lb[t] = V[k], Q[k], names[L[k]]
    if detail:
        return cnt, clr, bp, qp, lb
    return cnt, clr


# ------------------------------------------------------------------ vanilla clips
def vanilla(set_name, clip):
    d = MC.load_dump(MC.clip_path(set_name, clip))
    ph = {e['eventName']: (e['startFrame'], e['startFrame'] + e['durationInFrames']) for e in d['meta']['events'] if e.get('type') == 'Phase'}
    return d, ph


def vanilla_index(set_name):
    return json.load(open(os.path.join(MC.WORK, 'npz', set_name, '_index.json')))


def tip_speed(W, weapon):
    t = tip(W, weapon)
    v = np.r_[0.0, np.linalg.norm(np.diff(t, axis=0), axis=1)] * 30.0
    return v


def strike_frame(W, weapon, lo, hi):
    """frame of the fastest blade tip in [lo, hi] (the swing peak)."""
    v = tip_speed(W, weapon)
    lo, hi = max(0, int(lo)), min(len(W) - 1, int(hi))
    return lo + int(np.argmax(v[lo:hi + 1])), v


def tip_dir(W, weapon, f, k=3):
    """blade tip displacement over [f-k, f+k] in the hips heading frame (x right, y forward, z up), unit + length (cm)."""
    t = tip(W, weapon)
    H = MC.heading_frame(W[:, IX['LeftUpLeg'], :3, 3], W[:, IX['RightUpLeg'], :3, 3])
    a, b = max(0, f - k), min(len(W) - 1, f + k)
    d = H[f].T @ (t[b] - t[a])
    n = float(np.linalg.norm(d))
    return d / max(n, 1e-9), n * 100


def tip_rel(W, weapon, f):
    t = tip(W, weapon)
    H = MC.heading_frame(W[:, IX['LeftUpLeg'], :3, 3], W[:, IX['RightUpLeg'], :3, 3])
    return H[f].T @ (t[f] - W[f, IX['Spine3'], :3, 3])


def survey(a):
    weapon = a[a.index('--weapon') + 1]
    st = player_set(weapon)
    out = {'vanilla': {}, 'npc': {}}
    idx = vanilla_index(st)
    print('VANILLA', st)
    for clip in sorted(idx):
        m = idx[clip]
        if m['additive'] or not any(k in clip for k in ('attack', 'hold', 'block', 'parried')):
            continue
        d, ph = vanilla(st, clip)
        W = MC.world_all(d)
        lo, hi = ph.get('active', (0, len(W) - 1))
        f, v = strike_frame(W, weapon, lo - 1, hi + 1)
        u, n = tip_dir(W, weapon, f)
        tr = tip_rel(W, weapon, f)
        out['vanilla'][clip] = dict(frames=len(W), phases=ph, strike=f, tip_speed=round(float(v[f]), 1), dir=np.round(u, 2).tolist(), disp_cm=round(n, 1),
                                    tip_rel_cm=np.round(tr * 100, 1).tolist())
        print('  %-24s T=%3d ph=%-40s strike=%2d v=%5.1f dir=%s tip@Spine3=%s' % (clip, len(W), ph, f, v[f], np.round(u, 2).tolist(),
                                                                              np.round(tr * 100, 0).tolist()))
    for kind in ('action',):
        sn = npc_set(weapon, kind)
        sk = NR.skeleton(os.path.join(NR.NPC_DIR, sn + '.anims.glb'))
        print('NPC', sn)
        import glbanim as GA
        for clip in GA.clips(sk.gj):
            if not any(k in clip for k in ('attack', 'combo', 'heavy', 'charge', 'parry', 'deflect', 'block', 'push')):
                continue
            sq = BSeq([(sn, clip)])
            lo, hi = sq.loop if sq.loop else (0, sq.n - 1)
            f, v = strike_frame(sq.W, weapon, lo - 1, hi + 1)
            u, n = tip_dir(sq.W, weapon, f)
            tr = tip_rel(sq.W, weapon, f)
            out['npc'][clip] = dict(frames=sq.n, loop=sq.loop, strike=f, tip_speed=round(float(v[f]), 1), dir=np.round(u, 2).tolist(), disp_cm=round(n, 1),
                                    tip_rel_cm=np.round(tr * 100, 1).tolist(), two_hand=int(sq.on2h.sum()), root_end_cm=sq.parts[0]['root_end_cm'])
            print('  %-36s T=%3d loop=%-9s strike=%2d v=%5.1f dir=%s tip@Spine3=%s 2H=%d root=%s' % (
                clip, sq.n, sq.loop, f, v[f], np.round(u, 2).tolist(), np.round(tr * 100, 0).tolist(), int(sq.on2h.sum()), sq.parts[0]['root_end_cm']))
    MC.jdump(out, os.path.join(OUTW, 'survey', 'tips_%s_%s.json' % (weapon, RIG)))


def grip(a):
    """weapon bone in the hand: vanilla player clips vs NPC clips (angle to the vanilla idle grip, blade direction in the hand frame)."""
    weapon = a[a.index('--weapon') + 1]
    st = player_set(weapon)
    d0, _ = vanilla(st, 'melee_idlepose')
    W0 = MC.world_all(d0)
    G0 = np.linalg.inv(W0[0, IX['RightHand']]) @ W0[0, WR]
    rep = dict(weapon=weapon, player_set=st, vanilla_idle_grip=dict(t_cm=np.round(G0[:3, 3] * 100, 2).tolist(),
                                                                     blade_dir_in_hand=np.round(G0[:3, 2], 3).tolist()), vanilla={}, npc={})
    for clip, m in sorted(vanilla_index(st).items()):
        if m['additive']:
            continue
        d, ph = vanilla(st, clip)
        W = MC.world_all(d)
        Gm = np.linalg.inv(W[:, IX['RightHand']]) @ W[:, WR]
        ang = MC.rot_delta_deg(G0[None, :3, :3], Gm[:, :3, :3])
        bz = np.einsum('ij,tj->ti', G0[:3, :3].T, Gm[:, :3, 2])[:, 2]       # blade direction vs the idle blade direction (cos)
        rep['vanilla'][clip] = dict(frames=len(W), ang_min=round(float(ang.min()), 1), ang_max=round(float(ang.max()), 1),
                                    reverse_frames=int((bz < 0).sum()))
    sn = npc_set(weapon)
    sk = NR.skeleton(os.path.join(NR.NPC_DIR, sn + '.anims.glb'))
    import glbanim as GA
    for clip in GA.clips(sk.gj):
        raw = raw_npc(sn, clip)
        Wn, ix = raw['W'], raw['ix']
        Gn = np.linalg.inv(Wn[:, ix['RightHand']]) @ Wn[:, ix['WeaponRight']]
        ang = MC.rot_delta_deg(G0[None, :3, :3], Gn[:, :3, :3])
        bz = np.einsum('ij,tj->ti', G0[:3, :3].T, Gn[:, :3, 2])[:, 2]
        tdev = np.linalg.norm(Gn[:, :3, 3] - Gn[:1, :3, 3], axis=1).max() * 100
        _, on, _ = left_rel(raw)
        rep['npc'][clip] = dict(frames=len(Wn), ang_min=round(float(ang.min()), 1), ang_max=round(float(ang.max()), 1), reverse_frames=int((bz < 0).sum()),
                                t_f0_cm=np.round(Gn[0, :3, 3] * 100, 2).tolist(), t_dev_cm=round(float(tdev), 1), two_hand_frames=int(on.sum()))
    MC.jdump(rep, os.path.join(OUTW, 'survey', 'grip_%s_%s.json' % (weapon, RIG)))
    rv = [k for k, v in rep['vanilla'].items() if v['reverse_frames']]
    rn = [k for k, v in rep['npc'].items() if v['reverse_frames']]
    print('GRIP', weapon, RIG, 'vanilla idle grip t_cm', rep['vanilla_idle_grip']['t_cm'])
    print('  vanilla clips with reverse-grip frames:', rv)
    print('  NPC clips with reverse-grip frames:', rn)
    print('  NPC grip angle to the vanilla idle grip: median %.1f deg' % float(np.median([v['ang_min'] for v in rep['npc'].values()])))


if __name__ == '__main__':
    cmd = sys.argv[1]
    {'survey': survey, 'grip': grip}[cmd](sys.argv[1:])
