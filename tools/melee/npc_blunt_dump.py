"""Vanilla dumps of the BATON player sets (pwa_baton / pma_baton) for the NPC-based blunt / baton builder (2026-11-09).

  python -I npc_blunt_dump.py pwa_baton|pma_baton

The twelve sets of melee_dump.py live in Build\\work\\melee-anim-20261107 (a separate work folder); the baton sets are not among them.  This
wrapper runs melee_dump.dump_set UNCHANGED with melee_common.WORK redirected (this process only) to
Build\\work\\melee-npc-blunt-20261109\\vanilla-dump, which holds glb\\<set>.anims.glb (WolvenKit export WITH root motion of the retail 2.5
.anims, Build\\work\\melee-export-20261109\\evidence\\retail\\raw) and json\\<set>.anims.json (the retail serialize of the same file).
Output: vanilla-dump\\npz\\<set>\\<clip>.npz + _index.json (melee_common dump layout).  One process per set (UTPP_RIG is read at import).
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_common as MC  # noqa: E402

BATON = {'pwa_baton': 'woman', 'pma_baton': 'man'}
s = sys.argv[1]
assert s in BATON, s
MC.WORK = os.path.join(MC.ROOT, 'Build', 'work', 'melee-npc-blunt-20261109', 'vanilla-dump')
MC.SETS = list(MC.SETS) + [k for k in BATON if k not in MC.SETS]
MC.RIG_OF_SET.update(BATON)
import melee_dump  # noqa: E402  (imports melee_common: the patched module object)

melee_dump.dump_set(s)
