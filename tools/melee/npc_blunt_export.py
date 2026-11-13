"""Run a script of Source\\tools\\anim\\melee-export UNCHANGED with the two BATON player sets added to its set lists (2026-11-09).

  python -I npc_blunt_export.py <melee-export script.py> [its arguments ...]
      e.g. registration_evidence.py --retail <evidence\\retail> --out <dir>; build_melee_set.py / patch_melee_json.py / verify_melee_set.py
      (called by npc_blunt_export.ps1); make_melee_xl.py --registration <json> --manifest <run\\manifest.json> --out <run>\\utpp_melee_tpp.archive.xl

melee_export_lib.SETS / VANILLA_DEPOT / RIG_OF_SET know the twelve sets of the first export (no baton).  This shim imports melee_export_lib
first, appends pwa_baton / pma_baton (retail depot base\\animations\\weapon\\baton\\p?a_baton.anims, rigs woman / man) and runs the script with
runpy as __main__; the script's own `import melee_export_lib` gets the same (extended) module object.  Nothing else changes.
"""
import os
import runpy
import sys

ME = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'anim', 'melee-export'))
sys.path.insert(0, ME)
import melee_export_lib as L  # noqa: E402

BATON = {'pwa_baton': ('woman', r'base\animations\weapon\baton\pwa_baton.anims'),
         'pma_baton': ('man', r'base\animations\weapon\baton\pma_baton.anims')}
for s, (rig, depot) in BATON.items():
    if s not in L.SETS:
        L.SETS.append(s)
    L.VANILLA_DEPOT.setdefault(s, depot)
    L.RIG_OF_SET.setdefault(s, rig)

script = sys.argv[1]
if not os.path.isabs(script):
    script = os.path.join(ME, script)
assert os.path.dirname(os.path.abspath(script)).lower() == ME.lower(), script
sys.argv = [script] + sys.argv[2:]
if os.path.basename(script).lower() == 'registration_evidence.py':
    # its weapon-variable list PRIMARY (mirrored entries, exclusive weapon variables) has no baton variable: imported as a module, the
    # tuple extended by w_melee_baton (the variable of the vanilla p?a_baton entries, priority 128), then main() as the script would
    import importlib
    RE = importlib.import_module('registration_evidence')
    if 'w_melee_baton' not in RE.PRIMARY:
        RE.PRIMARY = tuple(RE.PRIMARY) + ('w_melee_baton',)
    RE.main()
else:
    runpy.run_path(script, run_name='__main__')
