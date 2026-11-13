"""Male V alias of the MotionBricks stage (melee_regenerate.py, melee_regenerate_demo.py, melee_regenerate_batch.py, melee_regenerate_sheet.py, test_melee_regenerate.py).

  python -I melee_regenerate_man.py clip <set> <clip> [melee_regenerate.py options: --seeds --out --no-mb --demo-dir --max-comp --keep-tmp]
  python -I melee_regenerate_man.py demo  [melee_regenerate_demo.py options: --only <clip> --seeds --no-render --no-run]
  python -I melee_regenerate_man.py batch [melee_regenerate_batch.py options: --sets --limit --sample --seeds --chunk --force --out --report --max-comp]
  python -I melee_regenerate_man.py sheet <set> <clip> [melee_regenerate_sheet.py options: --frames --n --out]
  python -I melee_regenerate_man.py test  [--no-gpu]          (test_melee_regenerate.run_all, temp files under mb4_man\\_tmp)

Since the fix round E (2026-11-09) the stage itself is rig-aware, so this wrapper is a thin alias kept for the existing commands:
  * rig man: UTPP_RIG=man is set before melee_regenerate / base_body / game_skeleton_bridge are imported (one rig per process); the gate reference is then
    redo_man\\base_reference.json (anatomy_gate_base.REF follows base_body.REDO), logged at start;
  * output root: melee_regenerate.MB4 is WORK\\mb4_man for the man rig (F5; asserted here, no longer overridden);
  * demo clips = demo_metrics.CLIPS_MAN (melee_regenerate_demo's default for man as well); before the demo the rendered {3q,side}_{base,orig,redo} frame folders
    of redo_man\\demo3\\<set>__<clip> are COPIED (never moved) when missing - melee_regenerate_demo.make copies the skins / dumps itself and its paired redo + v4
    render (same camera, camvar redo) then replaces {3q,side}_redo;
  * batch: melee_regenerate_batch.selection skips dumps with every joint translation at the origin (e.g. pma_katana/finisher_katana_02) itself; batch_report.json
    goes next to the --out dir (default mb4_man\\batch_report.json).
"""
import hashlib
import os
import shutil
import sys

if '--rig' in sys.argv:
    _i = sys.argv.index('--rig')
    if sys.argv[_i + 1] != 'man':
        raise SystemExit('melee_regenerate_man.py is the Male V wrapper (rig man only)')
    del sys.argv[_i:_i + 2]
os.environ['UTPP_RIG'] = 'man'

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402

import melee_regenerate as M  # noqa: E402
import melee_common as MC  # noqa: E402

assert M.RIG == 'man' and M.BB.RIG_NAME == 'man' and M.CP.RIG_NAME == 'man', (M.RIG, M.BB.RIG_NAME, M.CP.RIG_NAME)
OUT = os.path.join(MC.WORK, 'mb4_man')
assert os.path.normcase(M.MB4) == os.path.normcase(OUT), M.MB4
SRC_DEMO = os.path.join(MC.WORK, 'redo_man', 'demo3')
B_FILES = ('melee_regenerate.py', 'melee_regenerate_batch.py', 'melee_regenerate_demo.py', 'melee_regenerate_sheet.py', 'test_melee_regenerate.py')


def provenance():
    print('RIG %s | BB.REDO %s | GB.REF %s | CP.RIG_NAME %s | MB4 %s' % (M.RIG, M.BB.REDO, M.GB.REF, M.CP.RIG_NAME, M.MB4), flush=True)
    for f in B_FILES:
        print('SHA256 %s %s' % (hashlib.sha256(open(os.path.join(HERE, f), 'rb').read()).hexdigest(), f), flush=True)
    assert os.path.normcase(os.path.dirname(M.GB.REF)) == os.path.normcase(os.path.join(MC.WORK, 'redo_man')), M.GB.REF


def degenerate(s, c):
    z = np.load(MC.clip_path(s, c), allow_pickle=False)
    return float(np.abs(z['W'][..., :3, 3]).max()) < 1e-6


def copy_demo_renders(pairs, demo_root):
    for s, c in pairs:
        src, dst = os.path.join(SRC_DEMO, '%s__%s' % (s, c)), os.path.join(demo_root, '%s__%s' % (s, c))
        os.makedirs(dst, exist_ok=True)
        for view in ('3q', 'side'):
            for var in ('base', 'orig', 'redo'):
                a, b = os.path.join(src, '%s_%s' % (view, var)), os.path.join(dst, '%s_%s' % (view, var))
                if os.path.isdir(a) and not os.path.exists(b):
                    shutil.copytree(a, b)
        print('COPIED renders', s, c, flush=True)


def main(a):
    if not a or a[0] in ('-h', '--help'):
        print(__doc__)
        return
    provenance()
    cmd, rest = a[0], a[1:]
    if cmd == 'clip':
        M.main(rest)
    elif cmd == 'demo':
        import melee_regenerate_demo as DM
        from demo_metrics import CLIPS_MAN
        assert os.path.normcase(DM.SRC_DEMO) == os.path.normcase(SRC_DEMO), DM.SRC_DEMO
        DM.CLIPS = CLIPS_MAN
        only = rest[rest.index('--only') + 1] if '--only' in rest else None
        copy_demo_renders([(s, c) for s, c in CLIPS_MAN if not only or c == only], os.path.join(OUT, 'demo3'))
        DM.main(rest)
    elif cmd == 'batch':
        import melee_regenerate_batch as RB
        RB.main(rest)
    elif cmd == 'sheet':
        import melee_regenerate_sheet as SH
        SH.main(rest)
    elif cmd == 'test':
        import test_melee_regenerate as TT
        TT.run_all('--no-gpu' in rest)
    else:
        raise SystemExit('unknown command %r (clip | demo | batch | sheet | test)' % cmd)


if __name__ == '__main__':
    main(sys.argv[1:])
