"""Build Build\\work\\melee-anim-20261107\\manifest.json from npz\\<set>\\_index.json + the dumps' meta.

  python -I melee_manifest.py [--per-group 12.5]
clips[]: set, clip, rig, frames, duration, additive, animation_type, root_motion, family, group, npz, glb, phases.
groups[]: id (<set>_gNN), set, rig, family list, clips (ordered). Groups: per set, clips ordered by family (attacks, heavy/final,
specials, hit/deflect, block/parry, equip, locomotion, additives) then name, split into round(n / per_group) chunks of equal size
(+-1), so a group is one or two neighbouring families of ONE set. Proposed partition only: clips outside a group are never needed for that group.
"""
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import melee_common as MC  # noqa: E402

FAMILY_ORDER = ['attack_combo', 'attack_heavy_final', 'attack_special', 'react_hit_deflect', 'block_parry', 'equip_throw',
                'locomotion', 'pose_misc', 'additive']


def family(clip, additive):
    if additive:
        return 'additive'
    if re.search(r'_(hit|deflect)$', clip):
        return 'react_hit_deflect'
    if re.match(r'combo_attack_\d+$', clip):
        return 'attack_combo'
    if re.match(r'(heavy|final)_attack_\d+$', clip):
        return 'attack_heavy_final'
    if re.match(r'(sprint|crouch|jump|fall|slide|safe|equip|block|groundslam)_attack|melee_groundslam_attack|melee_bodyslam', clip):
        return 'attack_special'
    if clip.startswith(('block', 'hold_', 'parried')):
        return 'block_parry'
    if re.search(r'equip|throw', clip):
        return 'equip_throw'
    if re.search(r'sprint|slide|jump|crouch|air|idle_to|to_idle|to_safe|to_sprint', clip):
        return 'locomotion'
    return 'pose_misc'


def main(a):
    per = float(a[a.index('--per-group') + 1]) if '--per-group' in a else 12.5
    clips, groups = [], []
    for s in MC.SETS:
        base = os.path.join(MC.WORK, 'npz', s)
        index = json.load(open(os.path.join(base, '_index.json')))
        rows = []
        for clip, v in index.items():
            z = np.load(os.path.join(base, clip + '.npz'))
            m = json.loads(str(z['meta']))
            phases = {e['eventName']: [e['startFrame'], e['startFrame'] + e['durationInFrames']]
                      for e in m['events'] if e.get('type') == 'Phase'}
            rows.append(dict(set=s, clip=clip, rig=m['rig'], frames=v['frames'], duration=round(v['duration'], 4),
                             additive=v['additive'], animation_type=v['animation_type'], root_motion=v['root_motion'],
                             family=family(clip, v['additive']), npz='npz\\%s\\%s.npz' % (s, clip), glb='glb\\%s.anims.glb' % s,
                             phases=phases, event_names=sorted({e['eventName'] for e in m['events'] if e.get('type') != 'Phase'})))
        rows.sort(key=lambda r: (FAMILY_ORDER.index(r['family']), r['clip']))
        g = max(1, round(len(rows) / per))
        chunks = np.array_split(np.arange(len(rows)), g)
        for gi, ch in enumerate(chunks, 1):
            gid = '%s_g%02d' % (s, gi)
            members = [rows[i] for i in ch]
            for r in members:
                r['group'] = gid
            groups.append(dict(id=gid, set=s, rig=MC.RIG_OF_SET[s], n=len(members), n_additive=sum(r['additive'] for r in members),
                               families=sorted({r['family'] for r in members}, key=FAMILY_ORDER.index),
                               clips=[r['clip'] for r in members]))
        clips += rows
    man = dict(created='2026-11-07', fps=30, n_clips=len(clips), n_groups=len(groups), n_additive=sum(r['additive'] for r in clips),
               sets=MC.SETS, groups=groups, clips=clips)
    MC.jdump(man, os.path.join(MC.WORK, 'manifest.json'))
    print('MANIFEST clips %d groups %d sizes %s' % (len(clips), len(groups), sorted({g['n'] for g in groups})))


if __name__ == '__main__':
    main(sys.argv[1:])
