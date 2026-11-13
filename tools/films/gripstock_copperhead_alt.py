"""Rifle grip + stock 2026-11-10 (round 3) - Copperhead decision numbers from gripstock_bodycheck.py (stock part only, --tag stockonly):
(A) hand priority (--nostock): grip, reach shift, stock contact against the shoulder pocket along the barrel ON THE OUTPUT POSE (pocket rides
on RightArm), skin of the right upper arm inside the stock (face sign, winding-checked worst frame) per frame;
(B) stock at contact + full grip: forward offset of the left shoulder / torso along the barrel = reach need after the stock push + the push
clamp residual (estimate: the reach need is the DLL's own ReachShiftNeeded on the pushed pose).
Steady part = the last 2/3 of the Aim phases.
  python -I gripstock_copperhead_alt.py  -> stdout (copy in Build\\work\\rifle-grip-stock-20261110\\copperhead_alternatives.txt)
"""
import json
import os

import numpy as np

W = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'Build', 'work', 'rifle-grip-stock-20261110')


def col(d, k):
    return np.array([np.nan if x is None else x for x in d[k]], float)


def rng(x):
    return '%.2f..%.2f (median %.2f)' % (np.nanmin(x), np.nanmax(x), np.nanmedian(x))


for body in ('woman', 'man'):
    cur = json.load(open(os.path.join(W, 'bodycheck_copperhead_stockonly_%s.json' % body)))
    ns = json.load(open(os.path.join(W, 'bodycheck_copperhead_nostock_stockonly_%s.json' % body)))
    c, n, b = cur[body + ':after']['frames'], ns[body + ':after']['frames'], ns[body + ':before']['frames']
    lab = cur[body + ':after']['phase']
    gap, sshift, need = col(c, 'stockGap'), col(c, 'stockShift'), col(c, 'need')
    for ph in ('aim + shots', 'walk aim'):
        idx = np.nonzero(np.array([x == ph for x in lab]))[0]
        idx = idx[len(idx) // 3:]
        resid = np.maximum(0, -(gap[idx] + sshift[idx]))
        print('%s %s, %d frames (steady)' % (body, ph, len(idx)))
        print('   overlay without the fix: contact vs pocket %s cm; right upper arm skin in the stock %s cm (%s vertices)' % (
            rng(col(b, 'pocketGap')[idx]), rng(col(b, 'rarm_cm')[idx]), rng(col(b, 'rarm_in')[idx])))
        print('   current build: grip %s cm; contact vs pocket %s cm; skin in the stock %s cm' % (
            rng(col(c, 'grip')[idx]), rng(col(c, 'pocketGap')[idx]), rng(col(c, 'rarm_cm')[idx])))
        print('   (A) hand priority: grip %s cm, reach shift %s cm; contact vs pocket %s cm; right upper arm skin in the stock %s cm '
              '(%s vertices); torso / left arm 0' % (rng(col(n, 'grip')[idx]), rng(col(n, 'shift')[idx]), rng(col(n, 'pocketGap')[idx]),
                                                       rng(col(n, 'rarm_cm')[idx]), rng(col(n, 'rarm_in')[idx])))
        print('       per frame (every 3rd) contact vs pocket cm:', ' '.join('%.2f' % x for x in col(n, 'pocketGap')[idx][::3]))
        print('       per frame (every 3rd) skin depth cm:      ', ' '.join('%.2f' % x for x in col(n, 'rarm_cm')[idx][::3]))
        print('   (B) contact + full grip (estimate): left shoulder / torso forward along the barrel %s cm (need after the push %s + clamp '
              'residual %s)' % (rng(need[idx] + resid), rng(need[idx]), rng(resid)))
