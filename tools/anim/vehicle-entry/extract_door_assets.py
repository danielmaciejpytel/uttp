"""Read-only extraction of the vehicle door assets for check_door_clearance.py (called by 01_extract_car.ps1 -Doors).

  python -I extract_door_assets.py --wkit <WolvenKit.CLI.exe> --game <Cyberpunk 2077 dir> --root <Extracted\\game\\vehicle-entry-car>

Unbundles (game archives are only read; archive\\pc\\mod is not touched) into <root>\\raw-veh: the vehicle clip sets
base\\animations\\vehicle\\{standard,sport,utility}\\vehicle\\v_<vehicle>.anims, the default car animgraph, the front door entities and their door meshes of
the six test vehicles (Archer Hella, Villefort Columbus, Chevalier Emperor, Quadra Turbo-R, Herrera Outlaw = v_sport2_porsche_911turbo, Kaukaz Bratsk);
serializes the .anims / .animgraph / .ent to <root>\\json-veh, exports the vehicle .anims to <root>\\glb-veh and the door meshes to <root>\\glb-doors
(MeshExportArgs ExportMaterials=false: WolvenKit needs no material depot for a plain mesh export), runs extract_door_data.py and writes
<root>\\door-provenance.txt (sha256 of every raw file) next to the existing provenance.txt.
"""
import argparse
import glob
import hashlib
import os
import re
import subprocess
import sys

VEH = ['standard.v_standard2_archer_hella', 'standard.v_standard25_villefort_columbus', 'standard.v_standard3_chevalier_emperor',
       'sport.v_sport1_quadra_turbo', 'sport.v_sport2_porsche_911turbo', 'utility.v_utility4_kaukaz_bratsk']
ARCH = ['content\\basegame_1_engine', 'content\\basegame_4_animation', 'content\\basegame_4_appearance']
# door mesh stems: every part (skin, window glass, sill) of extract_door_data.CLASSES, left and right
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import extract_door_data as EDD  # noqa: E402
MESH = sorted({stem for _, (_, pf) in EDD.CLASSES.items() for sd, s_ in (('fl', 'l'), ('fr', 'r')) for stem, _k in pf(s_, sd)})


def run(cmd, env=None):
    r = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace', env=env)
    return r.returncode, r.stdout + r.stderr


# REAR doors (4-door classes), ADDITIVE: new folders raw-rear / json-rear / glb-rear only, nothing of the front-door extraction is touched or wiped.
# Door entity regex = door_(f[lr]|b[lr]) widened to the rear doors: the Emperor ents are ..._door_b{l,r}.ent, Hella / Columbus ..._ext01_door_b{l,r}_01.ent.
# Mesh names were read from those ents (json-rear): Emperor ext01_door_b?_01 + ext01_door_window_b?_01, Hella ext01_door_b?_01 + ext01_window_b?_01,
# Columbus ext01_door_b?_01 (no window component). The meshes of the police / sticker / bulletholes / shadow variants and base\\vehicles\\decoration are not taken.
REAR_ENT_RX = ('(v_standard3_chevalier_emperor_door_b[lr]|v_standard2_archer_hella__ext01_door_b[lr]_01|v_standard25_villefort_columbus__ext01_door_b[lr]_01)\\.ent$')
REAR_MESH_RX = ('(vehicles\\\\standard\\\\(v_standard3_chevalier_emperor|v_standard2_archer_hella|v_standard25_villefort_columbus)\\\\entities\\\\meshes\\\\'
                '(v_standard3_chevalier_emperor__ext01_door_(b[lr]_01|window_b[lr]_01)|v_standard2_archer_hella__ext01_(door|window)_b[lr]_01|'
                'v_standard25_villefort_columbus__ext01_door_b[lr]_01)\\.mesh$)')


def rear_assets(a):
    arch = os.path.join(a.game, 'archive', 'pc', 'content', 'basegame_4_appearance.archive')
    raw, jsn, glb = (os.path.join(a.root, d) for d in ('raw-rear', 'json-rear', 'glb-rear'))
    for d in (raw, jsn, glb):
        if os.path.exists(d) and os.listdir(d):
            sys.exit('%s exists and is not empty: the rear extraction is additive and never overwrites (remove it by hand to redo)' % d)
        os.makedirs(d, exist_ok=True)
    for rx in (REAR_ENT_RX, REAR_MESH_RX):
        rc, out = run([a.wkit, 'unbundle', arch, '-o', raw, '-r', rx])
        if rc != 0:
            sys.exit('rear unbundle failed (%d): %s' % (rc, out[-400:]))
    ents = [p for p in glob.glob(os.path.join(raw, '**', '*.ent'), recursive=True)]
    meshes = [p for p in glob.glob(os.path.join(raw, '**', '*.mesh'), recursive=True) if '\\decoration\\' not in p and '/decoration/' not in p]
    if len(ents) != 6 or len(meshes) != 10:
        sys.exit('expected 6 rear ents and 10 rear meshes, got %d / %d' % (len(ents), len(meshes)))
    for p in ents:
        rc, out = run([a.wkit, 'convert', 'serialize', p, '-o', jsn])
        if rc != 0:
            sys.exit('serialize %s failed (%d)' % (p, rc))
    env = dict(os.environ, MeshExportArgs__ExportMaterials='false', MeshExportArgs__MaterialRepo=os.path.join(a.root, 'matrepo'))
    rc, out = run([a.wkit, 'export'] + meshes + ['-o', glb, '-gp', a.game], env)
    if rc != 0 or len(glob.glob(os.path.join(glb, '*.glb'))) != len(meshes):
        sys.exit('export of the rear door meshes failed (%d): %s' % (rc, out[-400:]))
    here = os.path.dirname(os.path.abspath(__file__))
    sys.exit(subprocess.call([sys.executable, '-I', os.path.join(here, 'extract_door_data.py'), '--root', a.root, '--rear']))


# BODY SHELL (additive, new folders raw-body / glb-body / body-data only): the car body mesh (exterior shell: roof, roof rails, B/C-pillars, quarter panels, door frame, sills)
# and the interior mesh (roof lining) of the six classes; the base is the entity mesh frame (no component offset: ent json ext01_body_01 components have identity transforms and no
# parent bone), related to the rig frame by the 'Base' bone (extract_door_data.py --body documents and checks the offset).
BODY_MESH_RX = (r'(vehicles\\(standard|sport|utility)\\v_(standard2_archer_hella|standard25_villefort_columbus|standard3_chevalier_emperor|sport1_quadra_turbo|sport2_porsche_911turbo|utility4_kaukaz_bratsk)\\entities\\meshes\\'
                r'v_[a-z0-9_]+__(ext01_body_01|int01_interior_01|ext01_window_(a|b|c|f|bl|br)_01|ext01_window_(bl|br)_02_01|ext01_windshield_01|ext01_window_f_[abcd]_01)\.mesh$)')
# static GLASS of the body entity (ext01_body_01 ent components window_*: windshield, rear / quarter windows; the door glass hangs on the door bones and is in door-data). Names read from the body ents.


def body_assets(a):
    arch = os.path.join(a.game, 'archive', 'pc', 'content', 'basegame_4_appearance.archive')
    raw, glb = (os.path.join(a.root, d) for d in ('raw-body', 'glb-body'))
    for d in (raw, glb):
        if os.path.exists(d) and os.listdir(d):
            sys.exit('%s exists and is not empty: the body extraction is additive and never overwrites (remove it by hand to redo)' % d)
        os.makedirs(d, exist_ok=True)
    rc, out = run([a.wkit, 'unbundle', arch, '-o', raw, '-r', BODY_MESH_RX])
    if rc != 0:
        sys.exit('body unbundle failed (%d): %s' % (rc, out[-400:]))
    meshes = [p for p in glob.glob(os.path.join(raw, '**', '*.mesh'), recursive=True) if 'police' not in p and 'deathrace' not in p and 'missile' not in p]
    if len(meshes) < 12 + 18:
        sys.exit('expected 12 body / interior meshes (6 classes x 2) + the body-entity glass, got %d' % len(meshes))
    env = dict(os.environ, MeshExportArgs__ExportMaterials='false', MeshExportArgs__MaterialRepo=os.path.join(a.root, 'matrepo'))
    # one call for all meshes (the archive manager loads once); the shell meshes are big, the call takes several minutes
    rc, out = run([a.wkit, 'export'] + meshes + ['-o', glb, '-gp', a.game], env)
    if rc != 0 or len(glob.glob(os.path.join(glb, '*.glb'))) != len(meshes):
        sys.exit('export of the body meshes failed (%d): %s' % (rc, out[-400:]))
    here = os.path.dirname(os.path.abspath(__file__))
    sys.exit(subprocess.call([sys.executable, '-I', os.path.join(here, 'extract_door_data.py'), '--root', a.root, '--body']))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wkit', required=True)
    ap.add_argument('--game', required=True)
    ap.add_argument('--root', required=True)
    ap.add_argument('--body', action='store_true', help='ADDITIVE body shell / roof lining extraction (raw-body, glb-body, body-data) instead of the front-door extraction')
    ap.add_argument('--rear', action='store_true', help='ADDITIVE rear door / glass extraction (raw-rear, json-rear, glb-rear, rear-data) instead of the front-door extraction')
    a = ap.parse_args()
    if a.body:
        body_assets(a)
    if a.rear:
        rear_assets(a)
    raw = os.path.join(a.root, 'raw-veh')
    jsn = os.path.join(a.root, 'json-veh')
    import shutil
    # fresh output folders: nothing stale is exported, hashed or read by extract_door_data.py (only these door-specific folders are touched)
    for d in (raw, jsn, os.path.join(a.root, 'glb-veh'), os.path.join(a.root, 'glb-doors'), os.path.join(a.root, 'door-data')):
        if os.path.isdir(d):
            shutil.rmtree(d)
    for d in (raw, jsn, os.path.join(a.root, 'glb-veh'), os.path.join(a.root, 'glb-doors')):
        os.makedirs(d, exist_ok=True)
    names = '|'.join(v.split('.')[1] for v in VEH)
    rx = ('(animations\\\\vehicle\\\\(standard|sport|utility)\\\\vehicle\\\\(%s)\\.anims$)' % names +
          '|(gameplay\\\\default_car_anim_graph\\.animgraph$)' +
          '|(vehicles\\\\(standard|sport|utility)\\\\(%s)\\\\entities\\\\[^\\\\]*door_(f[lr]|[lr])(_01)?\\.ent$)' % names +
          '|(vehicles\\\\(standard|sport|utility)\\\\(%s)\\\\entities\\\\meshes\\\\(%s)\\.mesh$)' % (names, '|'.join(re.escape(m) for m in MESH)))
    for arc in ARCH:
        p = os.path.join(a.game, 'archive', 'pc', arc + '.archive')
        rc, out = run([a.wkit, 'unbundle', p, '-o', raw, '-r', rx])
        print('unbundle', arc, 'rc', rc)
        if rc != 0:
            sys.exit('unbundle %s failed (%d): %s' % (arc, rc, out[-400:]))
    files = [f for f in glob.glob(os.path.join(raw, '**', '*'), recursive=True) if os.path.isfile(f)]
    with open(os.path.join(a.root, 'door-provenance.txt'), 'w', encoding='ascii') as f:
        for p in sorted(files):
            h = hashlib.sha256(open(p, 'rb').read()).hexdigest().upper()
            f.write('%s  %d  %s\n' % (h, os.path.getsize(p), os.path.relpath(p, raw)))
    for p in files:
        if p.endswith(('.anims', '.animgraph', '.ent')):
            rc, out = run([a.wkit, 'convert', 'serialize', p, '-o', jsn])
            if rc != 0:
                sys.exit('serialize %s failed (%d): %s' % (p, rc, out[-400:]))
    env = dict(os.environ, AnimationExportArgs__incRootMotion='false', MeshExportArgs__ExportMaterials='false',
               MeshExportArgs__MaterialRepo=os.path.join(a.root, 'matrepo'))
    os.makedirs(env['MeshExportArgs__MaterialRepo'], exist_ok=True)
    anims = [p for p in files if p.endswith('.anims') and 'bratsk_extended' not in p]
    rc, out = run([a.wkit, 'export'] + anims + ['-o', os.path.join(a.root, 'glb-veh'), '-gp', a.game], env)
    print('export anims rc', rc)
    if rc != 0 or len(glob.glob(os.path.join(a.root, 'glb-veh', '*.glb'))) != len(anims):
        sys.exit('export of the vehicle clip sets failed (%d): %s' % (rc, out[-400:]))
    meshes = [p for p in files if p.endswith('.mesh')]
    rc, out = run([a.wkit, 'export'] + meshes + ['-o', os.path.join(a.root, 'glb-doors'), '-gp', a.game], env)
    print('export meshes rc', rc, 'files', len(meshes))
    if rc != 0 or len(glob.glob(os.path.join(a.root, 'glb-doors', '**', '*.glb'), recursive=True)) != len(meshes):
        sys.exit('export of the door meshes failed (%d): %s' % (rc, out[-400:]))
    here = os.path.dirname(os.path.abspath(__file__))
    rc = subprocess.call([sys.executable, '-I', os.path.join(here, 'extract_door_data.py'), '--root', a.root])
    sys.exit(rc)


if __name__ == '__main__':
    main()
