"""Stage the test-only CET command module (utpp_testcmd) into the test-probes stage and write its install manifest.

    python Source\\tools\\install\\stage_test_probes.py [--dry-run] [--stage test-probes] [--manifest NAME.json]

Copies Source/plugin/UTPP/cet/utpp_testcmd/init.lua to Build/stages/<stage>/bin/x64/plugins/cyber_engine_tweaks/mods/
utpp_testcmd/init.lua (a real copy, hash verified) and writes Research/inventory/<manifest> listing EVERY file of the stage
(files[].path / sha256 / bytes, the format install_verified_stage.ps1 reads), so files already staged there (other probes)
keep their entries unchanged. Install afterwards, with the game closed:

    powershell -NoProfile -ExecutionPolicy Bypass -File Source\\tools\\install\\install_verified_stage.ps1 -StageName <stage>
        -ManifestName <manifest> [-Apply]

The module is never part of a release (tools/release/make_release.py refuses it). Writes nothing in the game folder.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
from datetime import date

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))   # the tools folder
from utpp_paths import project_root  # noqa: E402

MODULE_REL = os.path.join('bin', 'x64', 'plugins', 'cyber_engine_tweaks', 'mods', 'utpp_testcmd', 'init.lua')
SOURCE_REL = os.path.join('Source', 'plugin', 'UTPP', 'cet', 'utpp_testcmd', 'init.lua')


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest().upper()


def stage_files(stage: str) -> list[dict]:
    out = []
    for d, _, names in os.walk(stage):
        for n in sorted(names):
            p = os.path.join(d, n)
            out.append({'path': os.path.relpath(p, stage), 'sha256': sha256(p), 'bytes': os.path.getsize(p)})
    return sorted(out, key=lambda e: e['path'].lower())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stage', default='test-probes')
    ap.add_argument('--manifest', default=f'utpp-test-probes-{date.today():%Y%m%d}.json')
    ap.add_argument('--root', default=project_root())
    ap.add_argument('--dry-run', action='store_true', help='print the plan, write nothing')
    a = ap.parse_args(argv)
    if not a.stage.replace('-', '').replace('_', '').isalnum() or not a.manifest.endswith('.json') \
            or os.path.basename(a.manifest) != a.manifest:
        raise SystemExit('unsafe stage or manifest name')
    src = os.path.join(a.root, SOURCE_REL)
    stage = os.path.join(a.root, 'Build', 'stages', a.stage)
    dst = os.path.join(stage, MODULE_REL)
    manifest = os.path.join(a.root, 'Research', 'inventory', a.manifest)
    if not os.path.isfile(src):
        raise SystemExit(f'missing {src}')
    print(f'module  {SOURCE_REL} -> {os.path.relpath(dst, a.root)} ({sha256(src)[:12]})')
    print(f'manifest {os.path.relpath(manifest, a.root)}')
    if a.dry_run:
        return 0
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    if os.path.exists(dst):
        os.remove(dst)              # never write through a hard link of a frozen copy
    shutil.copyfile(src, dst)
    if sha256(dst) != sha256(src):
        raise SystemExit('staged copy differs from the source')
    files = stage_files(stage)
    os.makedirs(os.path.dirname(manifest), exist_ok=True)
    with open(manifest, 'w', encoding='utf-8') as f:
        json.dump({'files': files}, f, indent=1)
    for e in files:
        print(f'  {e["sha256"][:12]} {e["bytes"]:>8} {e["path"]}')
    print(f'STAGE_OK files={len(files)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
