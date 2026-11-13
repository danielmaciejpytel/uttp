"""IK helper chain check of a built melee GLB (build_melee_set.py output, before any import / packing) against the vanilla set, on EVERY
frame including the last one (2026-11-09, knife / katana NPC-based clips).

  python -I npc_blade_helpers.py --set pwa_katana --built-glb <utpp_<set>.anims.glb> --vanilla-glb-nort <set>.anims.glb
                                 --vanilla-json <set>.anims.json --report <json> [--twohand-dir <clips dir>]

Per built clip and frame: distance of arm_{left,right}_ik / _ik_offset / _ik_weapon to the hand of the same side, built vs vanilla on the
same frame (the game's hand IK follows these helpers: a helper far from the hand pulls the hand away in game).  --twohand-dir: the
<set>__<clip>.npc1.twohand.npz files (katana): the left-hand helpers are also reported on the two-hand frames only (w > 0.5).
Pass rule: every frame <= 1.5 cm, or <= the vanilla distance of that frame + 0.1 cm (inherited from vanilla).
Frames moved by the exporter (2026-11-09 final2; build_melee_set.snap_hand_helpers puts a hand helper that is farther than 1.5 cm from the
hand ON the hand: arm_*_ik -> the hand's world pose, arm_*_ik_offset -> identity local, arm_*_ik_weapon keeps its vanilla local and follows):
on such a frame the vanilla world distance of arm_*_ik_weapon is no reference (its parent was moved on purpose), so there it is judged
INSTEAD against the vanilla grip carried by the built parent: <= 1.5 cm, or <= the distance to the hand of (built arm_*_ik_offset world o
vanilla arm_*_ik_weapon local) + 0.1 cm (the vanilla-distance clause does not apply on those frames).  Moved frames are recognised from the
data (no extra input): arm_*_ik_offset built local = identity while the vanilla local is not, or arm_*_ik built exactly on its hand
(<= MOVED_ON_HAND_CM) while the vanilla one is not.  Every other frame and helper: the rule above, unchanged.
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ME = os.path.join(HERE, '..', 'anim', 'melee-export')
sys.path.insert(0, ME)
sys.path.insert(0, os.path.join(HERE, '..', 'anim', 'vehicle-entry'))
import melee_export_lib as L  # noqa: E402
import skel as S  # noqa: E402

HELPERS = [('arm_%s_ik%s' % (s, x), s.capitalize() + 'Hand') for s in ('left', 'right') for x in ('', '_offset', '_weapon')]
LIMIT_CM = 1.5
TOL_CM = 0.1                 # allowed excess over the reference distance (vanilla, or the carried vanilla grip on moved frames)
IDENT_T_M, IDENT_Q = 1e-6, 1e-5      # identity local (built arm_*_ik_offset on a moved frame: exactly 0 / (0, 0, 0, 1) in the GLB)
MOVED_ON_HAND_CM = 0.01      # built arm_*_ik this close to its hand while the vanilla one is not = put on the hand by the exporter (the
                             # re-anchored helpers keep the vanilla per-frame distance, so only the snap lands exactly on the hand)


def is_identity(lt, lr):
    return (np.linalg.norm(lt, axis=-1) <= IDENT_T_M) & (np.minimum(np.linalg.norm(lr - [0, 0, 0, 1], axis=-1),
                                                                    np.linalg.norm(lr + [0, 0, 0, 1], axis=-1)) <= IDENT_Q)


def moved_frames(sk, skn, lt, lr, vlt, vlr, nWt, vWt, side):
    """frames where the exporter put the hand IK parent chain of arm_<side>_ik_weapon on the hand (bool [T])."""
    T = len(lt)
    m = np.zeros(T, bool)
    off, ik, hand = 'arm_%s_ik_offset' % side, 'arm_%s_ik' % side, side.capitalize() + 'Hand'
    if off in sk.index and off in skn.index:
        j, jv = sk.index[off], skn.index[off]
        m |= is_identity(lt[:, j], lr[:, j]) & ~is_identity(vlt[:, jv], vlr[:, jv])
    if ik in sk.index and ik in skn.index:
        dn = np.linalg.norm(nWt[:, sk.index[ik]] - nWt[:, sk.index[hand]], axis=-1) * 100
        dv = np.linalg.norm(vWt[:, skn.index[ik]] - vWt[:, skn.index[hand]], axis=-1) * 100
        m |= (dn <= MOVED_ON_HAND_CM) & (dv > MOVED_ON_HAND_CM)
    return m


def judge(h, b, sk, skn, lt, lr, vlt, vlr, nWt, nWr, vWt, w2h=None):
    """per-frame rule of one helper -> (report row, failing frames)."""
    T = len(lt)
    dn = np.linalg.norm(nWt[:, sk.index[h]] - nWt[:, sk.index[b]], axis=-1) * 100
    dv = np.linalg.norm(vWt[:, skn.index[h]] - vWt[:, skn.index[b]], axis=-1) * 100
    ok = (dn <= LIMIT_CM) | (dn <= dv + TOL_CM)
    extra = {}
    if h.endswith('_ik_weapon'):
        side = h.split('_')[1]
        mv = moved_frames(sk, skn, lt, lr, vlt, vlr, nWt, vWt, side)
        if mv.any():
            j, p = sk.index[h], sk.parent[sk.index[h]]
            carried = nWt[:, p] + np.einsum('tij,tj->ti', nWr[:, p], vlt[:, skn.index[h]])     # built parent o vanilla grip (local translation)
            dref = np.linalg.norm(carried - nWt[:, sk.index[b]], axis=-1) * 100
            ok = np.where(mv, (dn <= LIMIT_CM) | (dn <= dref + TOL_CM), ok)       # moved frames: the carried grip replaces the vanilla clause
            extra = dict(frames_parent_on_hand=np.nonzero(mv)[0].tolist(), carried_grip_ref_max_cm=round(float(dref[mv].max()), 3),
                         carried_grip_dev_max_cm=round(float(np.linalg.norm(nWt[mv, j] - carried[mv], axis=-1).max() * 100), 4))
    over = np.nonzero(~ok)[0]
    r = dict(max_cm=round(float(dn.max()), 3), last_cm=round(float(dn[-1]), 3), vanilla_max_cm=round(float(dv.max()), 3),
             vanilla_last_cm=round(float(dv[-1]), 3), max_minus_vanilla_cm=round(float((dn - dv).max()), 3), frames_over=over.tolist(), **extra)
    if w2h is not None and 'left' in h and len(w2h) == T and (w2h > 0.5).any():
        r['two_hand_frames'] = int((w2h > 0.5).sum())
        r['two_hand_max_cm'] = round(float(dn[w2h > 0.5].max()), 3)
    return r, over


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--set', required=True)
    ap.add_argument('--built-glb', required=True)
    ap.add_argument('--vanilla-glb-nort', required=True)
    ap.add_argument('--vanilla-json', required=True)
    ap.add_argument('--report', required=True)
    ap.add_argument('--twohand-dir')
    a = ap.parse_args()
    vclips, _ = L.anims_json_clips(a.vanilla_json)
    sk = S.Skeleton.from_glb(a.built_glb)
    skn = S.Skeleton.from_glb(a.vanilla_glb_nort)
    built = list(L.G.clips(sk.gj).keys()) if hasattr(L, 'G') else []
    rep, bad = {}, []
    for clip in built:
        vc = vclips[clip]
        T, dur = int(vc['numFrames']), float(vc['duration'])
        lt, lr, _, _ = L.sample_clip(sk, clip, T, dur, root_seconds=True)
        vlt, vlr, _, _ = L.sample_clip(skn, clip, T, dur)
        nWt, nWr = sk.world(lt, lr)
        vWt, _ = skn.world(vlt, vlr)
        w2h = None
        if a.twohand_dir:
            p = os.path.join(a.twohand_dir, '%s__%s.npc1.twohand.npz' % (a.set, clip))
            if os.path.exists(p):
                w2h = np.load(p)['w']
        row = {}
        for h, b in HELPERS:
            if h not in sk.index:
                continue
            r, over = judge(h, b, sk, skn, lt, lr, vlt, vlr, nWt, nWr, vWt, w2h)
            row[h] = r
            if len(over):
                bad.append((clip, h, over.tolist()[:6]))
        rep[clip] = row
    allmax = max((r['max_cm'] for row in rep.values() for r in row.values()), default=0.0)
    lastmax = max((r['last_cm'] for row in rep.values() for r in row.values()), default=0.0)
    devmax = max((r['max_minus_vanilla_cm'] for row in rep.values() for r in row.values()), default=0.0)
    tw = [r['two_hand_max_cm'] for row in rep.values() for h, r in row.items() if 'two_hand_max_cm' in r]
    mv = [(c, h, len(r['frames_parent_on_hand'])) for c, row in rep.items() for h, r in row.items() if r.get('frames_parent_on_hand')]
    summ = dict(set=a.set, clips=len(rep), helper_max_cm=round(allmax, 3), helper_last_frame_max_cm=round(lastmax, 3),
                helper_max_minus_vanilla_cm=round(devmax, 3), left_helpers_two_hand_max_cm=round(max(tw), 3) if tw else None,
                weapon_frames_parent_on_hand=sum(n for _, _, n in mv), weapon_tracks_parent_on_hand=[[c, h, n] for c, h, n in mv],
                failing=bad, PASS=not bad)
    L.jdump(dict(summary=summ, clips=rep), a.report)
    print('HELPERS', json.dumps(summ))
    return 0 if not bad else 1


if __name__ == '__main__':
    sys.exit(main())
