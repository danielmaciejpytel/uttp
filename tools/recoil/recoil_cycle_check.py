"""Step 4 of recoil-missing-20261109: does a single-shot clip carry a hand cycle (pump, bolt, hammer) besides the weapon kick?

utpp_motion.reds FallbackRecoilProfile keeps shotguns, precision/sniper rifles and HMGs on 0 because "their native shot animation carries a
bolt/pump cycle that an Ajax kick would replace". An authored profile only carries the WeaponRight motion (the hands follow the weapon
through the grip IK), so a cycle of a HAND relative to the weapon (forend pump, bolt, cocking) is not in the file. This measures, for
every clip of the plan and, as the baseline, for every shipped family, the motion of LeftHand and RightHand in WeaponRight space
against their place at t = 0 (cm, max over the clip). Read only; writes <work>\\cycle-check.json.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import recoil_common as rc  # noqa: E402
import export_missing_recoil as em  # noqa: E402

WORK = rc.ROOT / 'Build/work/recoil-missing-20261109'


def hand_motion(path: Path, clip: str):
    anim = rc.animation(path)
    duration = anim.clips[clip][0]
    first = None
    worst = {'LeftHand': 0.0, 'RightHand': 0.0}
    for t in np.linspace(0, duration, 49):
        m = anim.model(clip, float(t))
        w = np.linalg.inv(m['WeaponRight'])
        rel = {h: (w @ m[h])[:3, 3] for h in worst}
        first = first or rel
        for h in worst:
            worst[h] = max(worst[h], float(np.linalg.norm(rel[h] - first[h])))
    return worst


def main():
    index = rc.index_glbs([rc.GLB_25], 'pwa')
    jobs = [(f, 'shipped', src, 'add_recoil_single_shot_shoulder', 'add_recoil_single_shot_ironsight')
            for f, src in rc.core.PROFILE_SOURCES.items()]
    jobs += [(f, 'excluded' if f in em.EXCLUDED else 'new', p[0], p[1], p[2]) for f, p in em.PLAN.items()]
    out = []
    for fam, group, src, hip, ads in jobs:
        path = index.get(src)
        if path is None:
            out.append(dict(family=fam, group=group, error='GLB missing: ' + src))
            continue
        row = dict(family=fam, group=group, source=src)
        for kind, clip in (('hip', hip), ('ads', ads)):
            m = hand_motion(path, clip)
            row[kind] = dict(clip=clip, leftHandCm=m['LeftHand'] * 100, rightHandCm=m['RightHand'] * 100)
        out.append(row)
        print(f"{group:<9}{fam:<14} hip L {row['hip']['leftHandCm']:6.2f} R {row['hip']['rightHandCm']:6.2f} cm | "
              f"ads L {row['ads']['leftHandCm']:6.2f} R {row['ads']['rightHandCm']:6.2f} cm")
    (WORK / 'cycle-check.json').write_text(json.dumps(dict(generator='Source/tools/recoil/recoil_cycle_check.py', rows=out), indent=2)
                                           + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
