"""Anatomy gate against the BASE BODY of V (no vanilla melee clip is a reference).  Per frame.

  python -I anatomy_gate_base.py selftest [--out FILE]     (base + source pack clips must pass, vanilla melee clips must fail on the shoulders;
                                                            default output BB.REDO\\gate_selftest.json, --out writes elsewhere so a re-run keeps the stored result)
Reference = base_reference.json (base_body.py): ranges of anatomy_gate.features over source pack locomotion / idle / crouch / traversal / aim clips
(0.5 / 99.5 percentile) united with the shipped overlay poses (Pistol1H / 2H / Rifle / Feminine / Neutral / Crouch ...), plus human limits (anatomy_gate.LIM).
  structure features (clavicle / shoulder / spine / neck / knee deviation / bone length):  allowed = [range lo - tol, range hi + tol]
  human features (elbow / wrist / forearm / arm swing / torso distance / reach ...):        allowed = human limit united with the range
Extras that no existing feature tests: clavicle layout (left clavicle joint left of the right one in the chest frame, gap >= 2.5 cm) and the shoulder
width within BASE_SHOULDER_TOL_CM of the base.  Returns per-frame pass flags, so a clip can keep its good game frames and redo only the bad ones.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import anatomy_gate as AG  # noqa: E402
import base_body as BB  # noqa: E402   (resolves --rig / UTPP_RIG; REDO = reference folder of the selected body)
import melee_common as MC  # noqa: E402

REF = os.path.join(BB.REDO, 'base_reference.json')       # woman: ...\redo (unchanged), man: ...\redo_man
STRUCT = ['clav_swing_L', 'clav_swing_R', 'clav_twist_L', 'clav_twist_R', 'clav_shift_L_cm', 'clav_shift_R_cm', 'shoulder_width_cm', 'elbow_dev_L',
          'elbow_dev_R', 'knee_dev_L', 'knee_dev_R', 'shin_roll_L', 'shin_roll_R', 'seg_len_dev_pct', 'neck_bend', 'neck_twist', 'spine_bend_total',
          'spine_twist_total', 'knee_flex_L', 'knee_flex_R', 'torso_lean_deg'] + ['spine_%s_%s' % (a, b) for a in ('bend', 'twist')
                                                                                  for b in ('Spine', 'Spine1', 'Spine2', 'Spine3')]
TOL = {'ang': 5.0, 'cm': 1.5, 'pct': 1.0}
SKIP = ['hips_drop_cm', 'hand_cross_cm_L', 'hand_cross_cm_R']      # hips_drop depends on the stance (reference = T pose); hand cross handled below
BASE_SHOULDER_TOL_CM = 3.0
MIN_CLAV_GAP_CM = 2.5
HAND_CROSS_MAX_CM = 22.0
_REF = None


def ref():
    global _REF
    if _REF is None:
        _REF = json.load(open(REF, encoding='utf-8'))
    return _REF


def allowed():
    """feature -> (lo, hi) over all frames."""
    d = ref()
    out = {}
    for k, (hl, hh, kind) in AG.LIM.items():
        if k.startswith('delta_') or k in SKIP:
            continue
        r = d['ranges'].get(k)
        o = d['overlay_ranges'].get(k)
        los, his = [], []
        for x in (r and (r['p01'], r['p99']), o and (o['min'], o['max'])):
            if x:
                los.append(x[0])
                his.append(x[1])
        t = TOL.get(kind, 6.0 if kind == 'step' else 0.0)
        if k in STRUCT and los:
            lo, hi = min(los) - t, max(his) + t
            if k.startswith(('clav_shift', 'seg_len', 'knee_dev', 'elbow_dev', 'shin_roll')):
                lo = 0.0
        elif los:
            lo, hi = min(hl, min(los) - t), max(hh, max(his) + t)
        else:
            lo, hi = hl, hh
        if kind == 'step':
            hi = max(hh, (max(his) if his else hh) * 1.6 + 6.0)
        out[k] = (lo, hi)
    out['shoulder_width_cm'] = (ref()['base']['shoulder_width_cm'] - BASE_SHOULDER_TOL_CM, ref()['base']['shoulder_width_cm'] + BASE_SHOULDER_TOL_CM)
    out['hand_cross_cm_L'] = out['hand_cross_cm_R'] = (-1e9, HAND_CROSS_MAX_CM)
    return out


def clav_gap_cm(W, rg):
    P = W[:, :, :3, 3]
    H = MC.heading_frame(P[:, rg.ix['LeftArm']], P[:, rg.ix['RightArm']])
    d = P[:, rg.ix['RightShoulder']] - P[:, rg.ix['LeftShoulder']]
    return np.einsum('tji,tj->ti', H, d)[:, 0] * 100


def frame_gate(W, rg, steps=True, al=None):
    """-> dict(ok [T] bool, viol {feature: [T] excess (0 = inside)}, margin [T] (min over features of normalised margin, < 0 = violated), worst [T] feature name)."""
    al = al or allowed()
    f = AG.features(W, rg, step_frames=steps)
    f['clav_gap_cm'] = clav_gap_cm(W, rg)
    al = dict(al)
    al['clav_gap_cm'] = (MIN_CLAV_GAP_CM, 1e9)
    T = len(W)
    viol, margin, worst = {}, np.full(T, 1e9), np.array([''] * T, dtype=object)
    for k, (lo, hi) in al.items():
        if k not in f:
            continue
        v = np.asarray(f[k], float)
        ex = np.maximum(lo - v, 0) + np.maximum(v - hi, 0)
        sc = max(abs(hi - lo) if hi - lo < 1e8 else 20.0, 5.0)
        m = np.minimum(v - lo, hi - v) / sc
        viol[k] = ex
        bad = m < margin
        margin = np.where(bad, m, margin)
        worst = np.where(bad, k, worst)
    ok = np.ones(T, bool)
    for k, ex in viol.items():
        ok &= ex <= 1e-9
    return dict(ok=ok, viol=viol, margin=margin, worst=worst, feat=f)


def summary(g):
    n = len(g['ok'])
    cnt = {k: int((v > 1e-9).sum()) for k, v in g['viol'].items() if (v > 1e-9).any()}
    return dict(frames=n, pass_frames=int(g['ok'].sum()), fail_features=dict(sorted(cnt.items(), key=lambda kv: -kv[1])[:8]),
                margin_min=float(g['margin'].min()))


def selftest(out=None):
    """out: result json (default BB.REDO\\gate_selftest.json, the stored result; pass another path to re-run without overwriting it)."""
    rg = BB.rig_for_gate()
    al = allowed()
    names = BB.rig()[0]
    L, Wb, Li = BB.compose_base()
    res = {}
    g = frame_gate(np.concatenate([Wb, BB.fkL(Li)[None]]), rg, steps=False, al=al)
    res['base+idle'] = summary(g)
    print('base+idle', res['base+idle'])
    for c in ['M_Neutral_Stand_Idle_Loop', 'M_Neutral_Walk_Loop_F', 'M_Neutral_Run_Loop_F', 'M_Neutral_Sprint_Loop_F', 'M_Neutral_Crouch_Idle_Loop',
              'M_Neutral_Jump_Start_Stand', 'AO_Pistol2H_Stand_Sweep', 'M_Neutral_Traversal_Catch_Hurdle_high_stand']:
        try:
            W = BB.sourcepack_W(c)
        except Exception:
            continue
        g = frame_gate(W, rg, al=al)
        res[c] = summary(g)
        print(c, res[c])
    pre = 'pwa_' if BB.RIG_NAME == 'woman' else 'pma_'      # vanilla melee clips of the selected body (female / male FPP rig)
    for s, c in [(pre + 'fists', 'combo_attack_01'), (pre + 'katana', 'combo_attack_01'), (pre + 'two_handed_blunt', 'heavy_attack_01'),
                 (pre + 'katana', 'combo_attack_04_hit'), (pre + 'fists', 'block_deflect_right'), (pre + 'two_handed_blunt', 'melee_equip')]:
        d = MC.load_dump(MC.clip_path(s, c))
        g = frame_gate(d['W'], rg, al=al)
        res[s + '/' + c] = summary(g)
        print(s, c, res[s + '/' + c])
    out = out or os.path.join(BB.REDO, 'gate_selftest.json')
    MC.jdump(res, out)
    print('WROTE', out)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'selftest':
        selftest(os.path.abspath(sys.argv[sys.argv.index('--out') + 1]) if '--out' in sys.argv else None)
