"""Closed loop of the geometric roof head cap (build_entry_clip.py --roof-json): the builder measures the ceiling at the head position of its INTERMEDIATE pose; the later stages
(end blend, root smoothing, limb IK) move the head a few cm sideways and on a sloping roof that is several cm of height. This tool measures the cap again at the head positions of
the FINISHED clip (builder GLB of the previous pass) and writes it (min with the previous cap) as the --roof-cap-file of the next pass.

  python -I roof_cap_iter.py --clip <pass-N.anims.glb> --roof-json <body-data/class.json> --roof-seat left|right --end-pose <vanilla.anims.glb>:<seated clip>
         --roof-margin M [--roof-relax-seated] [--prev <cap json of pass N-1>] --out <cap json>

Prints the lowest cap and the largest drop against the previous cap.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import skel as S  # noqa: E402
import build_entry_clip as B  # noqa: E402
import check_car_entry as K  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clip', required=True)
    ap.add_argument('--clip-name', default='enter')
    ap.add_argument('--roof-json', required=True)
    ap.add_argument('--roof-seat', choices=['left', 'right'], default='left')
    ap.add_argument('--end-pose', required=True)
    ap.add_argument('--roof-margin', type=float, default=0.01)
    ap.add_argument('--roof-top', type=float, default=0.24)
    ap.add_argument('--roof-floor-below-seated', type=float, default=0.25)
    ap.add_argument('--roof-relax-seated', action='store_true')
    ap.add_argument('--prev')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    a.roof_lookahead = 0  # the finished clip is evaluated frame by frame (the builder's own +-2 frame filter covers its intermediate pose)
    a.roof_cap_file = a.prev
    sk = S.Skeleton.from_glb(a.clip)
    _, _, lt, lr = K.sample(sk, a.clip_name)
    Wt, _ = sk.world(lt, lr)
    hi, ri = sk.index['Head'], sk.index['Root']
    cap, floor_y, _, relax, _ = B.roof_cap(a, Wt, lt, lr, sk, hi, ri)
    drop = 0.0
    if a.prev:
        prev = np.asarray(json.load(open(a.prev, encoding='utf-8'))['cap_y'], float)
        m = (cap < 8.0) & (prev < 8.0)
        drop = float((prev - cap)[m].max()) if m.any() else 0.0
    json.dump(dict(cap_y=[round(float(x), 4) for x in cap], floor_y=floor_y, relax_seated_m=relax), open(a.out, 'w'))
    print('roof cap pass: frames capped %d, lowest %.3f m, largest drop vs previous pass %.1f cm, seated relax %.1f cm' % (int((cap < 8.0).sum()), float(cap.min()), drop * 100, relax * 100))
    return 0


if __name__ == '__main__':
    sys.exit(main())
