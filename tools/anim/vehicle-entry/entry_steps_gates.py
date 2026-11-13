"""Gate runs for the entry rows with spliced steps (2026-11-06): one build_car.ps1 -FromGlbDir process per row (round trip + check_car_entry + door / roof / jamb + approach + arrival ring against the
shipped R13j clips), all rows in parallel. Writes <out>\\<row>\\ (run dir of build_car.ps1) and <out>\\<row>.log.

  python -I entry_steps_gates.py <glb dir (builder GLBs + build reports)> <out dir> [--rows a,b,..] [--approach-jobs 3] [--selftest] [--no-approach]
Prints at the end per row the verdict lines of the run (check_car_entry RESULT, door / approach exit codes, close_after).
Default off in build_car.ps1: -FromGlbDir; the approach baseline is the SHIPPED set (r13\\sources-final-R13j.txt) so the gate is relative to what ships now."""
import os
import re
import subprocess
import sys
import time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

ROOT = project_root()
BUILD = ROOT + r'\Source\tools\anim\vehicle-entry\build_car.ps1'
BASELINE = ROOT + r'\Build\work\vehentry-mb-20261106\sources-baseline-R13j.txt'   # = r13\sources-final-R13j.txt with the run dirs under Build\work (the original paths no longer exist)


def main(a):
    glb, out = a[0], a[1]
    opt = lambda k, d: a[a.index(k) + 1] if k in a else d
    rows = [r for r in opt('--rows', '').split(',') if r] or sorted(f[:-len('.anims.glb')] for f in os.listdir(glb) if f.endswith('.anims.glb'))
    os.makedirs(out, exist_ok=True)
    procs = {}
    for row in rows:
        row = row if row.startswith('utpp_car_') else 'utpp_car_' + row
        m = re.match(r'utpp_car_p([wm])a_(\w+?)_(driver|passenger_r)$', row)
        g, cls, seat = m.group(1), m.group(2), m.group(3)
        cmd = ['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', BUILD, '-Out', os.path.join(out, row), '-Classes', cls, '-Genders', g, '-Seats', seat,
               '-FromGlbDir', glb, '-ApproachBaselineSources', BASELINE, '-ArrivalRing', '-ApproachJobs', opt('--approach-jobs', '3')]
        if '--selftest' not in a:
            cmd.append('-SkipSelftest')
        if '--no-approach' in a:
            cmd.append('-SkipApproach')
        procs[row] = (subprocess.Popen(cmd, stdout=open(os.path.join(out, row + '.log'), 'w'), stderr=subprocess.STDOUT), time.time())
    for row, (p, t0) in procs.items():
        p.wait()
        print('%-34s exit %d  %.0f s' % (row[9:], p.returncode, time.time() - t0), flush=True)
    for row in procs:
        txt = open(os.path.join(out, row + '.log'), errors='replace').read().splitlines()
        keep = [l for l in txt if re.search(r'^RESULT|exit=|door=|approach=|FAIL|CRASH|throw|offline check|door_close_solve at|waiver|START |JOINT|WAIVERS|REARWARD', l)]
        print('--', row[9:])
        for l in keep[-10:]:
            print('   ', l[:230])


if __name__ == '__main__':
    main(sys.argv[1:])
