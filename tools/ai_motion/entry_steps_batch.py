"""Batch of the glide pin for the car ENTRY rows (2026-11-06): entry_pin_run.sh per row (parallel), then build_entry_steps.py splices the steps into the shipped BUILDER GLB and writes
<out>/glb/<clip>.anims.glb + <clip>.build-report.json (the input of build_car.ps1 -FromGlbDir). Kimodo-env python.

  python -I entry_steps_batch.py <out dir> [--rows a,b,..] [--vmin 2.0] [--pin "--vthr 0.5 --margin 4"] [--jobs 12]
Rows (default): every shipped row whose planted-foot glide max is >= --vmin cm/frame (entry_steps_gen.py plant_summary); the others stay the shipped clip (no real glide)."""
import json
import os
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import entry_steps_gen as E  # noqa: E402

ROOT = E.ROOT
BASE = ROOT + r'\Build\work\vehentry-mb-20261106\base'
BASH = os.environ.get('UTPP_BASH') or os.path.join(os.environ.get('ProgramFiles', ''), 'Git', 'bin', 'bash.exe')
VE = ROOT + r'\Source\tools\anim\vehicle-entry'


def before_max(row):
    d = np.load(os.path.join(BASE, row, 'cur.dump.npz'), allow_pickle=True)
    pl = json.load(open(os.path.join(BASE, row, 'plants.json')))
    r = E.plant_summary(d['W'], [str(x) for x in d['names']], pl)
    return max(r[s]['max'] for s in E.SIDES)


def one(args):
    row, out, pin = args
    rdir = os.path.join(out, row)
    rel = os.path.relpath(rdir, ROOT).replace('\\', '/')
    p = subprocess.run([BASH, HERE.replace('\\', '/') + '/entry_pin_run.sh', row, rel] + pin.split(), capture_output=True, text=True, timeout=600)
    open(os.path.join(out, row + '.log'), 'w').write(p.stdout + p.stderr)
    steps = os.path.join(rdir, 'steps_pin.steps.npz')
    if not os.path.exists(steps):
        return row, None, p.stdout[-300:]
    d0 = np.load(os.path.join(rdir, 'cur.dump.npz'), allow_pickle=True)
    d1 = np.load(os.path.join(rdir, 'after_pin.dump.npz'), allow_pickle=True)
    names = [str(x) for x in d0['names']]
    ix = {n: i for i, n in enumerate(names)}
    f0, f1 = E.feet_motion(d0['W'], names), E.feet_motion(d1['W'], names)
    shift = max(float(np.linalg.norm(d1['W'][:, ix[sd + 'Foot'], :3, 3] - d0['W'][:, ix[sd + 'Foot'], :3, 3], axis=1).max()) * 100 for sd in E.SIDES)
    dv = max(f1[sd][1] - f0[sd][1] for sd in E.SIDES)
    if dv > 1.5 or shift > 10.5:   # a pop (speed change) added by the pin, or a foot moved far from its clip position: that row stays the shipped clip
        return row, False, 'REJECTED dv +%.1f (limit 1.5) shift %.1f cm (limit 10.5)' % (dv, shift)
    src = E.rows()
    run = dict(src)[row]
    glb = os.path.join(out, 'glb')
    os.makedirs(glb, exist_ok=True)
    r = subprocess.run([sys.executable, '-I', VE + r'\build_entry_steps.py', '--glb', os.path.join(run, 'glb', row + '.anims.glb'), '--steps', steps, '--out', os.path.join(glb, row + '.anims.glb'),
                        '--report', os.path.join(rdir, 'splice-report.json')], capture_output=True, text=True, timeout=600)
    shutil.copy(os.path.join(run, 'glb', row + '.build-report.json'), os.path.join(glb, row + '.build-report.json'))
    return row, True, (r.stdout + r.stderr)[-400:]


def main(a):
    out = a[0]
    opt = lambda k, dflt: a[a.index(k) + 1] if k in a else dflt
    vmin = float(opt('--vmin', '2.0'))
    pin = opt('--pin', '--vthr 0.5 --margin 8 --maxshift 10')
    rows = [('utpp_car_' + x if not x.startswith('utpp_car_') else x) for x in opt('--rows', '').split(',')] if '--rows' in a else [n for n, _ in E.rows() if before_max(n) >= vmin]
    os.makedirs(out, exist_ok=True)
    with ThreadPoolExecutor(int(opt('--jobs', '12'))) as ex:
        for row, ok, msg in ex.map(one, [(r, out, pin) for r in rows]):
            print(row[9:], 'OK' if ok else 'FAIL', '|', msg.strip().splitlines()[-1][:230] if msg.strip() else '')


if __name__ == '__main__':
    main(sys.argv[1:])
