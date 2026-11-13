"""Offline pre-screen of the start-spot levers (no WolvenKit round trip): builds the clip with build_entry_clip.py for candidate
(--start-out m, --start-back m) pairs into a scratch dir and runs the door model (check_door_clearance.py) on the builder GLB.

  python -I door_prescreen.py --class standard2 --gender w --seat driver --out 0 0.2 --back 0 0.3 0.6 [--scratch <dir>] [--jobs 4]

Prints per candidate: door phases (clearance cm, '!' = over gate), solved close_after_s, start pose error (joint max, m) from the build report.
The argument list of the builder is the one of build_car.ps1 (keep both in sync: COMMON / CLASS below).
"""
import argparse
import json
import os
import subprocess
import sys
import itertools
from concurrent.futures import ThreadPoolExecutor
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
EX = os.path.join(project_root(), r'Extracted\game\vehicle-entry-car')
COMMON = ['--target-duration', '--end-blend', '0.4', '--end-blend-legs', '0.6', '--hand-ik', '--corr-start', '0.25', '--corr-end', '0.8',
          '--smooth-rot-limit', '11', '--warp-interp', 'cubic', '--limit-pad', '3', '--plant-hold', '--ik-max-extend', '0.999', '--ik-bend-sigma', '2',
          '--root-smooth', '--ease-in', '0.2', '--end-ease', '0.2', '--end-ease-helpers', '0.6', '--ref-joint-fixed']
# class -> (all-clip args, passenger args)
CLASS = {
    'standard2': ([], []),
    'sport2': (['--cap-margin', '-0.035', '--smooth-rot-limit', '8'], ['--hand-follow-min', '0.8']),
    'standard25': ([], []),
    'standard3': ([], []),
    'sport1': ([], ['--smooth-rot-limit', '4']),
    'utility': (['--smooth-rot-limit', '6'], []),
}


def build(cls, g, seat, out, back, scratch, extra=()):
    tgt = '%s\\glb\\p%sa_%s_%s.anims.glb' % (EX, g, cls, seat)
    npc = '%s\\glb\\%sa_%s_passenger_r.anims.glb' % (EX, g, cls)
    end_clip = 'tpp_idle' if seat == 'driver' else 'idle'
    name = 'ps_p%sa_%s_%s_o%03d_b%s%03d' % (g, cls, seat, round(out * 100), 'n' if back < 0 else '', abs(round(back * 100)))
    glb = os.path.join(scratch, name + '.anims.glb')
    rep = os.path.join(scratch, name + '.json')
    allx, prx = CLASS[cls]
    extra_l = list(allx)
    if seat == 'driver':
        extra_l.append('--mirror')
    else:
        extra_l += prx
        if cls == 'sport2':
            extra_l += ['--cap-clip', '%s\\glb\\p%sa_sport2_driver.anims.glb:enter' % (EX, g)]
    if out > 0:
        extra_l += ['--start-out', str(out), '--out-axis', '-1' if seat == 'driver' else '1']
    if back != 0:
        extra_l += ['--start-back', str(back)]
    cmd = [sys.executable, '-I', os.path.join(HERE, 'build_entry_clip.py'), '--src', npc, '--src-clip', 'enter', '--target', tgt, '--target-clip', 'enter',
           '--name', 'enter', '--end-pose', '%s:%s' % (tgt, end_clip), '--cap-margin', '-0.02'] + COMMON + extra_l + list(extra) + ['--limb-ik-final', '--out', glb, '--report', rep]
    r = subprocess.run(cmd, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    if r.returncode != 0:
        raise RuntimeError('build failed: ' + r.stderr[-800:])
    return glb, rep


def screen(job):
    cls, g, seat, out, back, scratch = job
    sys.path.insert(0, HERE)
    import check_door_clearance as C
    glb, rep = build(cls, g, seat, out, back, scratch)
    m = C.Model(glb, cls, seat)
    r = C.analyse(m, None)
    sv = C.solve(m)
    ca = sv.get('close_after_s')
    r2 = C.analyse(m, ca) if ca else None
    br = json.load(open(rep))
    sp = max(br['start_pose_joint_err_m'].values())
    ph = (r2 or r)['phases']
    s = ' '.join('%s %.1f%s' % (p[:6], v['clearance_cm'], '' if v['ok'] else '!') for p, v in ph.items())
    return (cls, g, seat, out, back, s, ca, sp, bool(r2 and r2['door_clearance_ok']), br['start_in_seat_frame']['new']['pos_m'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--class', dest='cls', required=True)
    ap.add_argument('--gender', default='w')
    ap.add_argument('--seat', default='driver')
    ap.add_argument('--out', type=float, nargs='+', default=[0.0])
    ap.add_argument('--back', type=float, nargs='+', default=[0.0])
    ap.add_argument('--scratch', default=os.path.join(project_root(), r'Build\work\vehicle-entry-car\door3-prescreen'))
    ap.add_argument('--jobs', type=int, default=4)
    a = ap.parse_args()
    os.makedirs(a.scratch, exist_ok=True)
    jobs = [(a.cls, a.gender, a.seat, o, b, a.scratch) for o, b in itertools.product(a.out, a.back)]
    with ThreadPoolExecutor(a.jobs) as ex:
        for res in ex.map(screen, jobs):
            print('%s p%sa %s out %.2f back %.2f | %s | close_after %s | start err %.2f m | ok=%s | start %s' % res, flush=True)


if __name__ == '__main__':
    main()
