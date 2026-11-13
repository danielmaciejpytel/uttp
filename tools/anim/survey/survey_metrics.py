"""Stage 4: anatomy metrics of every dumped clip as the game SHOWS it in UTPP third person (one process per rig).

  python -I survey_metrics.py --rig woman|man [--only <setkey substring>]
Shown pose: A (and E / measured C) = the clip's full body; B = HYBRID: MM-owned joints from the MM base pose (base_body.compose_base,
read from <REDO>\\base_pose.npz), the game-owned joints from the clip (locals, FK):
  melee mask / release (arm cyberware, baton, hammer, grenade, melee equip, consumables if shown): game = everything except
      GameIntegration.hpp IsMaskLowerBone (Hips, Spine, legs) -> Spine1 and above from the game;
  weapon action (reload, quick melee; MotionDriver.cpp:5305-5313 BuildWeaponActionMask): game = clavicles, arms, hands (fingers), weapons
      (GameIntegration.hpp:838-855 IsWeaponActionBone); spine / neck / head / legs stay on MM.
Features: Source\\tools\\melee\\anatomy_gate.features (rest-relative, the rig's own joint frames) against the base-body reference
(anatomy_gate_base.allowed(): source pack ranges + tolerance), with these survey changes:
  - knee_flex, hip_flex, hip_twist: human limits only (stance caps dropped); torso_lean, hips_drop, foot_dev, hand_cross: not counted;
  - shoulder_width / clavicle gap recomputed in the Spine3 frame (the ground-projected heading frame degenerates on ladders, seats, death);
  - leg features ignored for *_sync_upperbody clips (the game masks them; the legs shown come from locomotion);
  - translation stretch (seg_len_dev_pct, clav_shift_*) and continuity (rot_step) reported in their own columns, not in the fail %;
  - plausibility (arm through the torso, hand at the eyes) reported separately (a joint clamp cannot fix it);
  - wrist_twist not counted: the game rig keeps the forearm a pure hinge and the pronation roll in the HAND (ForearmTwist.hpp), so a large
    hand twist is the game convention, not a deformation; the combined pronation (forearm + hand roll) is counted.
Output: Build\\work\\anim-survey-20261109\\metrics_<rig>.json (per clip: fail %, worst feature / exceedance / frame, top frames for
the skin check, translation / continuity / plausibility maxima).
"""
import csv
import glob
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import survey_common as SC  # noqa: E402

RIG = sys.argv[sys.argv.index('--rig') + 1] if '--rig' in sys.argv else 'woman'
os.environ['UTPP_RIG'] = RIG
SC.add_tool_paths()
import anatomy_gate as AG  # noqa: E402
import anatomy_gate_base as ABG  # noqa: E402
import base_body as BB  # noqa: E402
import melee_common as MC  # noqa: E402

assert BB.RIG_NAME == RIG

STRUCT_ANG = ['clav_swing_L', 'clav_swing_R', 'clav_twist_L', 'clav_twist_R', 'elbow_dev_L', 'elbow_dev_R', 'knee_dev_L', 'knee_dev_R',
              'shin_roll_L', 'shin_roll_R', 'neck_bend', 'neck_twist', 'spine_bend_total', 'spine_twist_total'] + \
             ['spine_%s_%s' % (a, b) for a in ('bend', 'twist') for b in ('Spine', 'Spine1', 'Spine2', 'Spine3')]
HUMAN = ['elbow_flex_L', 'elbow_flex_R', 'forearm_roll_L', 'forearm_roll_R', 'wrist_flex_L', 'wrist_flex_R', 'wrist_dev_L', 'wrist_dev_R',
         'pronation_L', 'pronation_R', 'uparm_twist_L', 'uparm_twist_R', 'uparm_swing_L', 'uparm_swing_R',
         'knee_flex_L', 'knee_flex_R', 'hip_flex_L', 'hip_flex_R', 'hip_twist_L', 'hip_twist_R']
DEFORM = STRUCT_ANG + HUMAN + ['shoulder_width_s3', 'clav_gap_s3']
TRANS = ['seg_len_dev_pct', 'clav_shift_L_cm', 'clav_shift_R_cm']
PLAUS = ['elbow_torso_cm_L', 'elbow_torso_cm_R', 'hand_torso_cm_L', 'hand_torso_cm_R', 'upperarm_torso_cm_L', 'upperarm_torso_cm_R',
         'forearm_torso_cm_L', 'forearm_torso_cm_R', 'hand_eye_cm']
LEG_FEATS = {'knee_dev_L', 'knee_dev_R', 'shin_roll_L', 'shin_roll_R', 'knee_flex_L', 'knee_flex_R', 'hip_flex_L', 'hip_flex_R', 'hip_twist_L',
             'hip_twist_R'}
HUMAN_LEGS = {'knee_flex_L': (-5.0, 150.0), 'knee_flex_R': (-5.0, 150.0), 'hip_flex_L': (0.0, 130.0), 'hip_flex_R': (0.0, 130.0),
              'hip_twist_L': (0.0, 60.0), 'hip_twist_R': (0.0, 60.0)}
SCALE = {'ang': 10.0, 'cm': 2.0, 'pct': 2.5}        # exceedance normalisation for ranking (10 deg ~ 2 cm ~ 2.5 %)
SHOULDER_TOL_CM, MIN_GAP_CM = 3.0, 2.5
WEAPON_ACTION_STATES = ('firearm action',)
MELEE_MASK_STATES = ('arm cyberware', 'melee (not covered', 'melee equip', 'grenade', 'consumable')


def is_mask_lower(n):          # GameIntegration.hpp IsMaskLowerBone
    return n in ('Hips', 'Spine') or n.startswith(('LeftUpLeg', 'LeftLeg', 'LeftFoot', 'LeftHeel', 'LeftToe', 'RightUpLeg', 'RightLeg',
                                                   'RightFoot', 'RightHeel', 'RightToe'))


def is_weapon_action(n):       # GameIntegration.hpp IsWeaponActionBone
    return n.startswith(('LeftShoulder', 'RightShoulder', 'LeftArm', 'RightArm', 'LeftForeArm', 'RightForeArm', 'LeftHand', 'RightHand',
                         'LeftInHand', 'RightInHand')) or n in ('WeaponLeft', 'WeaponRight')


def kind_of(f):
    return 'cm' if f.endswith('_cm') or f.endswith('_s3') else ('pct' if f.endswith('_pct') else 'ang')


class Ctx:
    def __init__(self):
        names, par, ref, rest = BB.rig()
        self.names, self.par, self.rest = list(names), np.asarray(par), np.asarray(rest, float)
        self.rg = BB.rig_for_gate()
        self.ix = self.rg.ix
        z = np.load(os.path.join(BB.REDO, 'base_pose.npz'))
        self.baseL = z['L'].astype(float)                 # [73,4,4] MM base pose locals (game axes)
        assert [str(x) for x in z['names']] == self.names
        al = ABG.allowed()
        self.allow = {}
        for k in STRUCT_ANG + HUMAN + TRANS + PLAUS + ['rot_step']:
            lo, hi = al.get(k, AG.LIM.get(k, (None, None, None))[:2])
            self.allow[k] = (lo, hi)
        for k, v in HUMAN_LEGS.items():
            lo, hi = al.get(k, v)
            self.allow[k] = (min(v[0], lo), max(v[1], hi)) if k.startswith(('hip',)) else v
        Wb = MC.fk_world(self.par, self.baseL[None, :, :3, 3], self._quat(self.baseL[None]))
        sw, gp = self.s3_width_gap(Wb)
        self.base_width, self.base_gap = float(sw[0]), float(gp[0])
        self.allow['shoulder_width_s3'] = (self.base_width - SHOULDER_TOL_CM, self.base_width + SHOULDER_TOL_CM)
        self.allow['clav_gap_s3'] = (MIN_GAP_CM, 1e9)
        gl = os.path.join(SC.WORK, 'guard_limits_%s.json' % RIG)       # source pack corpus band in the Spine3 frame (guard_limits.py), if built
        if os.path.exists(gl):
            g = json.load(open(gl, encoding='utf-8'))
            if 'shoulder_width_s3' in g:
                self.allow['shoulder_width_s3'] = tuple(g['shoulder_width_s3'])
                self.allow['clav_gap_s3'] = (min(MIN_GAP_CM, g['clav_gap_s3'][0]), 1e9)

    @staticmethod
    def _quat(L):
        from scipy.spatial.transform import Rotation as R
        sh = L.shape[:-2]
        return R.from_matrix(L[..., :3, :3].reshape(-1, 3, 3)).as_quat().reshape(sh + (4,))

    def s3_width_gap(self, W):
        ix = self.ix
        R3 = W[:, ix['Spine3'], :3, :3]
        rest3 = self.rest[ix['Spine3'], :3, :3]
        d0 = rest3.T @ (self.rest[ix['RightArm'], :3, 3] - self.rest[ix['LeftArm'], :3, 3])
        u = d0 / np.linalg.norm(d0)
        d = np.einsum('tji,tj->ti', R3, W[:, ix['RightArm'], :3, 3] - W[:, ix['LeftArm'], :3, 3])
        g = np.einsum('tji,tj->ti', R3, W[:, ix['RightShoulder'], :3, 3] - W[:, ix['LeftShoulder'], :3, 3])
        return d @ u * 100, g @ u * 100

    def shown(self, d, mode):
        """W [T,73,4,4] of the pose shown in game. mode 'full' | 'melee' | 'action'."""
        W = d['W'].astype(float)
        if mode == 'full':
            return W
        ix152 = {n: i for i, n in enumerate(d['names152'])}
        sel = [ix152[n] for n in self.names]
        lt = d['lt'][:, sel].astype(float)
        lr = d['lr'][:, sel].astype(float)
        bt, bq = self.baseL[:, :3, 3], self._quat(self.baseL)
        for j, n in enumerate(self.names):
            if n == 'Root':
                continue
            game = (not is_mask_lower(n)) if mode == 'melee' else is_weapon_action(n)
            if not game:
                lt[:, j] = bt[j]
                lr[:, j] = bq[j]
        return MC.fk_world(self.par, lt, lr)

    def measure(self, W, legs=True):
        f = AG.features(W, self.rg)
        f['shoulder_width_s3'], f['clav_gap_s3'] = self.s3_width_gap(W)
        T = len(W)
        ex = {}
        for k in DEFORM + TRANS + PLAUS + ['rot_step']:
            if k not in f or (not legs and k in LEG_FEATS):
                continue
            lo, hi = self.allow[k]
            v = np.asarray(f[k], float)
            e = np.maximum(lo - v, 0) + np.maximum(v - hi, 0) if lo is not None else np.zeros(T)
            ex[k] = e
        return f, ex


def norm_ex(k, e):
    return e / SCALE['step' if k == 'rot_step' else kind_of(k)] if k != 'rot_step' else e / 10.0


def glitch_mask(W, par, thr=120.0, span=10):
    """Frames inside a one-frame flip of a body bone (local step > thr deg, paired with the flip back within span frames): suspected
    decompression / export glitches (a 180 deg spike between consecutive keys; glbanim.slerp already takes the shortest path)."""
    L = AG._local(W, par)
    T = len(W)
    M = np.swapaxes(L[:-1], -1, -2) @ L[1:]
    ang = np.degrees(np.arccos(np.clip((np.trace(M, axis1=-2, axis2=-1) - 1) / 2, -1, 1)))     # [T-1, J]
    g = np.zeros(T, bool)
    for j in range(ang.shape[1]):
        ev = list(np.nonzero(ang[:, j] > thr)[0])
        i = 0
        while i < len(ev):
            a = ev[i]
            if i + 1 < len(ev) and ev[i + 1] - a <= span:
                g[a + 1:ev[i + 1] + 1] = True
                i += 2
            else:
                g[a + 1] = True
                i += 1
    return g


def summarize(ex, T, mask=None):
    dk = [k for k in DEFORM if k in ex]
    keep = np.ones(T, bool) if mask is None else ~mask
    if not keep.any():
        keep = np.ones(T, bool)
    ex = {k: np.where(keep, v, 0.0) for k, v in ex.items()}
    M = np.stack([norm_ex(k, ex[k]) for k in dk], 1)          # [T, F] normalised exceedance (glitch frames zeroed)
    fail = (M.max(1) > 1e-6)[keep]
    t = int(M.max(1).argmax())
    k = dk[int(M[t].argmax())]
    order = np.argsort(-M.max(1))
    top = []
    for f in order:
        if M[f].max() <= 1e-6 or len(top) >= 3:
            break
        if all(abs(int(f) - g) > 3 for g in top):
            top.append(int(f))
    cnt = {k2: int((ex[k2] > 1e-6).sum()) for k2 in dk if (ex[k2] > 1e-6).any()}
    tr = {k2: round(float(ex[k2].max()), 2) for k2 in TRANS if k2 in ex}
    pl = {k2: int((ex[k2] > 1e-6).sum()) for k2 in PLAUS if k2 in ex and (ex[k2] > 1e-6).any()}
    return dict(frames=T, fail_pct=round(100.0 * fail.mean(), 1), worst_feature=k, worst_excess=round(float(ex[k][t]), 2),
                worst_norm=round(float(M[t].max()), 2), worst_frame=t, top_frames=top,
                severity=round(float(np.percentile(M.max(1)[keep], 90)), 3), fail_features=dict(sorted(cnt.items(), key=lambda kv: -kv[1])[:6]),
                trans_excess=tr, trans_fail_pct=round(100.0 * np.mean((np.max(np.stack([ex[k2] for k2 in TRANS if k2 in ex], 1), 1) > 1e-6)[keep]), 1),
                glitch_frames=int(T - keep.sum()),
                rot_step_max=round(float(ex['rot_step'].max()), 1) if 'rot_step' in ex else 0.0, plaus_frames=pl)


def clip_class(row, clip):
    cls = row['cls']
    if 'exploration' in row['state']:
        for rx, c in ((r'^(ladder|knockdown|vehicle_knockdown|death|fpp_swim|fpp_idle_swim)', 'A'), (r'^(vault|climb)', 'E')):
            if re.search(rx, clip):
                return c
        return 'C'
    return cls


def mode_of(row):
    st = row['state']
    if row['cls'] == 'B' and st.startswith(WEAPON_ACTION_STATES):
        return 'action'
    if st.startswith(MELEE_MASK_STATES):
        return 'melee'
    return 'full'


def main():
    only = sys.argv[sys.argv.index('--only') + 1] if '--only' in sys.argv else ''
    tri = {r['setkey']: r for r in csv.DictReader(open(os.path.join(SC.WORK, 'triage_sets.csv'), encoding='utf-8'))}
    ctx = Ctx()
    out_p = os.path.join(SC.WORK, 'metrics_%s.json' % RIG)
    res = json.load(open(out_p, encoding='utf-8')) if (only and os.path.exists(out_p)) else {}
    checked_rest = False
    for idx in sorted(glob.glob(os.path.join(SC.DUMPS, RIG, '*', '_index.json'))):
        sk = os.path.basename(os.path.dirname(idx))
        if only and only not in sk:
            continue
        row = tri.get(sk)
        if row is None:
            continue
        info = json.load(open(idx, encoding='utf-8'))
        fpp_set = any(v.get('fpp_cam_animated') for v in info.get('clips', {}).values())
        mode = mode_of(row)
        for clip, ci in info.get('clips', {}).items():
            if not ci.get('dumped'):
                continue
            d = MC.load_dump(os.path.join(os.path.dirname(idx), clip + '.npz'))
            if not checked_rest:          # dump rest locals == rig rest (translation reset / segment lengths rely on it)
                ix152 = {n: i for i, n in enumerate(d['names152'])}
                sel = [ix152[n] for n in ctx.names]
                from scipy.spatial.transform import Rotation as R
                Rr = R.from_quat(d['rest_lr'][sel]).as_matrix()
                dr = max(float(np.degrees(np.arccos(np.clip((np.trace(Rr[j].T @ ctx.rg.Lrest[j]) - 1) / 2, -1, 1)))) for j in range(1, 73))
                dt = float(np.abs(d['rest_lt'][sel][1:] - ctx.rg.Lrest_t[1:]).max())
                print('REST CHECK max rot %.4f deg, max trans %.5f m' % (dr, dt))
                assert dr < 0.05 and dt < 1e-3, 'dump rest differs from the rig rest'
                checked_rest = True
            W = ctx.shown(d, mode)
            legs = not clip.endswith('sync_upperbody') and 'sync_upperbody' not in clip
            f, ex = ctx.measure(W, legs=legs)
            gm = glitch_mask(W, ctx.par)
            degenerate = bool(np.median(f['seg_len_dev_pct']) >= 90.0)     # all bone translations zero in the export (collapsed skeleton)
            s = summarize(ex, len(W), gm)
            s['degenerate_export'] = degenerate
            s.update(set=row['set'], setkey=sk, clip=clip, cls=clip_class(row, clip), state=row['state'], mode=mode, legs=legs,
                     fpp_cam_animated=bool(ci.get('fpp_cam_animated')), fpp_set=fpp_set,
                     tpp_camera_look=row['state'].startswith('vehicle'))
            res[sk + '/' + clip] = s
        print('METRICS %s %s (%s)' % (RIG, sk, mode))
    SC.jdump(res, out_p)
    print('WROTE', out_p, len(res), 'clips')


if __name__ == '__main__':
    main()
