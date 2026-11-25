"""Build an installable stage (archive + .xl + install manifest) from a box layout.

    python -I build_playground_stage.py [--layout spike_layout.json] [--anchor X Y Z [YAW]] [--stage-name utpp_playground_spike]

Work files go to Build/work/playground-<date>/_tmp, the stage to Build/stages/<stage-name>, the manifest the
installer reads to Research/inventory/<stage-name>.json.  The game folder is never touched.
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
from utpp_paths import project_root  # noqa: E402
import playground_sector as ps  # noqa: E402

ARCHIVE_NAME = 'utpp_playground'                      # WolvenKit names the archive after the folder it packs
BLOCK_PATH = r'utpp_playground\world\playground.streamingblock'
SECTOR_PATH = r'utpp_playground\world\playground.streamingsector'
WORK_DIR = 'playground-20261124'

LOG = []


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace')
    LOG.append({'cmd': cmd, 'exit': p.returncode})
    out = (p.stdout + p.stderr).replace('\r', '\n')
    tail = [l for l in out.split('\n') if l.strip() and not l.strip().endswith('%')][-4:]
    print('exit=%d  %s' % (p.returncode, ' '.join(os.path.basename(c) if os.path.exists(c) else c for c in cmd[:3])))
    for l in tail:
        print('   ', l)
    if p.returncode != 0 or '[ 0: Error' in out or 'Exception' in out:
        raise SystemExit('command failed: %s' % cmd)
    return out


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest().upper()


def write_json(path, doc):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, indent=1)


def main():
    root = project_root()
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--layout', default=os.path.join(HERE, 'spike_layout.json'))
    ap.add_argument('--anchor', nargs='+', type=float, metavar=('X', 'Y'), help='X Y Z [YAW_DEG] in the game world')
    ap.add_argument('--stage-name', default='utpp_playground_spike')
    ap.add_argument('--wolvenkit', default=os.path.join(root, 'Tools', 'WolvenKit.CLI', 'WolvenKit.CLI.exe'))
    args = ap.parse_args()

    with open(args.layout, encoding='utf-8') as f:
        layout = json.load(f)
    if args.anchor:
        if len(args.anchor) not in (3, 4):
            raise SystemExit('--anchor needs X Y Z [YAW]')
        layout['anchor'] = {'x': args.anchor[0], 'y': args.anchor[1], 'z': args.anchor[2],
                            'yaw_deg': args.anchor[3] if len(args.anchor) == 4 else 0.0}
    elif 'todo' in layout['anchor']:
        print('WARNING: the anchor is not set (TODO in the layout); the layout is built at the world origin.'
              ' Pass --anchor X Y Z [YAW].')

    boxes = ps.place_boxes(layout)
    sector, bbox = ps.build_sector(boxes, SECTOR_PATH, layout.get('mesh', ps.MESH_PATH))
    block = ps.build_block(SECTOR_PATH, bbox)

    work = os.path.join(root, 'Build', 'work', WORK_DIR, '_tmp', args.stage_name)
    if os.path.isdir(work):
        shutil.rmtree(work)
    json_dir = os.path.join(work, 'json')
    cr2w_dir = os.path.join(work, 'cr2w')
    pack_dir = os.path.join(work, 'pack', ARCHIVE_NAME)
    sector_json = os.path.join(json_dir, 'playground.streamingsector.json')
    block_json = os.path.join(json_dir, 'playground.streamingblock.json')
    write_json(sector_json, sector)
    write_json(block_json, block)

    wk = args.wolvenkit
    os.makedirs(cr2w_dir, exist_ok=True)
    for j in (sector_json, block_json):
        run([wk, 'convert', 'deserialize', j, '-o', cr2w_dir])
    depot_dir = os.path.join(pack_dir, os.path.dirname(BLOCK_PATH))
    os.makedirs(depot_dir, exist_ok=True)
    for name in ('playground.streamingsector', 'playground.streamingblock'):
        src = os.path.join(cr2w_dir, name)
        if not os.path.isfile(src):
            raise SystemExit('deserialize did not produce %s' % src)
        shutil.copy2(src, os.path.join(depot_dir, name))
    pack_out = os.path.join(work, 'packed')
    run([wk, 'pack', pack_dir, '-o', pack_out])
    archive = os.path.join(pack_out, ARCHIVE_NAME + '.archive')
    if not os.path.isfile(archive):
        raise SystemExit('pack did not produce %s' % archive)

    stage = os.path.join(root, 'Build', 'stages', args.stage_name)
    if os.path.isdir(stage):
        shutil.rmtree(stage)
    mod_dir = os.path.join(stage, 'archive', 'pc', 'mod')
    os.makedirs(mod_dir)
    shutil.copy2(archive, os.path.join(mod_dir, ARCHIVE_NAME + '.archive'))
    with open(os.path.join(mod_dir, ARCHIVE_NAME + '.archive.xl'), 'w', encoding='utf-8', newline='\n') as f:
        f.write(ps.build_xl(BLOCK_PATH))

    files = []
    for rel in ('archive\\pc\\mod\\%s.archive' % ARCHIVE_NAME, 'archive\\pc\\mod\\%s.archive.xl' % ARCHIVE_NAME):
        files.append({'path': rel, 'sha256': sha256(os.path.join(stage, rel))})
    manifest_path = os.path.join(root, 'Research', 'inventory', args.stage_name + '.json')
    write_json(manifest_path, {'stage': args.stage_name, 'anchor': layout['anchor'], 'boxes': len(boxes), 'files': files})
    write_json(os.path.join(work, 'resolved_boxes.json'), boxes)
    write_json(os.path.join(work, 'commands.json'), LOG)
    print('stage    ', stage)
    print('manifest ', manifest_path)
    for f in files:
        print('  %s  %s' % (f['sha256'][:16], f['path']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
