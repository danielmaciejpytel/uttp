"""Hips sink plan for head_duck.py --duck-sink-profile (roofpack3): the sink that keeps the pelvis path MONOTONE (it never rises again after it has sunk).

  python -I sink_plan.py <shipped clip.anims.glb> --class sport1 --seat driver --max-sink-cm 9 --floor-cm 3 [--start 27] [--end 58]

s(f) = the shipped Hips height above the seated hips (cm, vehicle frame z; the shipped pelvis hovers ~10 cm above the seat from frame 36 to 50 and sits down in the last 10 frames).
The new height h(f) = min(h(f-1), max(s(f) - max_sink, floor)), clipped to s(f) (never above the shipped pelvis), so the pelvis goes down early, stays there and follows the shipped sit-down
to the seated frame; sink(f) = s(f) - h(f), 0 before --start and from --end on. The printed knots (one per frame, cm) are the --duck-sink-profile argument.
A profile that eases back (e.g. 9 cm -> 5 cm while the shipped pelvis stays at +10) makes the pelvis sit, rise 4 cm and sit again (reviewed 2026-11-04).
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import check_door_clearance as C  # noqa: E402


ONSET = 4   # frames of cosine onset from sink 0 at --start (the pelvis headroom to the open door is 0.9 cm at frame 27)


def hips_above_seat_cm(clip, cls, seat):
    sk, t, Wt, Wr = C.load_clip(clip)
    ri = sk.index['Root']
    Re, pe = Wr[-1, ri], Wt[-1, ri]
    dj = C.load_door_data(C.DATA, cls)
    side = 'left' if seat == 'driver' else 'right'
    st, sR = np.array(dj['seat'][side]['t']), np.array(dj['seat'][side]['R']).reshape(3, 3)
    A = sR @ Re.T
    z = np.array([((A @ (Wt[k, sk.index['Hips']] - pe) + st) @ C.GLTF_TO_VEH.T)[2] for k in range(len(t))])
    return (z - z[-1]) * 100.0


def plan(s, max_sink, floor, start, end):
    h = np.array(s, float)
    out = np.zeros(len(s))
    prev = float('inf')
    for f in range(len(s)):
        if f < start + ONSET or f >= end:
            continue
        hf = min(prev, max(s[f] - max_sink, floor), s[f])
        prev = hf
        out[f] = max(0.0, s[f] - hf)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clip')
    ap.add_argument('--class', dest='cls', default='sport1')
    ap.add_argument('--seat', default='driver')
    ap.add_argument('--max-sink-cm', type=float, required=True)
    ap.add_argument('--floor-cm', type=float, required=True)
    ap.add_argument('--start', type=int, default=27)
    ap.add_argument('--end', type=int, default=58)
    a = ap.parse_args()
    s = hips_above_seat_cm(a.clip, a.cls, a.seat)
    sk = plan(s, a.max_sink_cm, a.floor_cm, a.start, a.end)
    knots = [(a.start, 0.0)] + [(f, sk[f]) for f in range(a.start + ONSET, a.end + 1)]   # the onset frames are eased in between the first two knots
    print(','.join('%d:%.1f' % (f, v) for f, v in knots))
    print('# shipped hips above seated (cm):', ' '.join('%d:%.1f' % (f, s[f]) for f in range(a.start, a.end + 1, 3)), file=sys.stderr)
    print('# new hips above seated (cm):    ', ' '.join('%d:%.1f' % (f, s[f] - sk[f]) for f in range(a.start, a.end + 1, 3)), file=sys.stderr)


if __name__ == '__main__':
    main()
