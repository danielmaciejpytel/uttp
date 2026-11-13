"""Fast screen of a head-duck setting on the SHIPPED round-trip clip of a sport1 DRIVER row (roofpack3; no builder, no WolvenKit round trip, ~5 s): head_duck.py -> candidate GLB ->
roof penetration / gate, the body shell, the per-capsule penetration of the FULLY OPEN door, the hips height above the seated hips.

  python -I duck_screen.py pwa_sport1_driver TAG [--checks] -- <head_duck options>

Why the open door only: the Turbo-R door has been fully open since clip frame 7-10 (event 0.056 s + open_dur 1.05 s minus the V lag 0.76-0.88 s), the head passes the roof edge at frames 28-34 and
the arm meets the door's sill plate (bone door_front_left_c) at frames 24-33. gen_door_timing / the redscript delay only the CLOSE, so no door timing changes the numbers printed here.
Candidates go to Build\\work\\vehicle-entry-car\\roof-pack3\\proto\\<ROW>.<TAG>.glb. The screen runs the duck AFTER the limb IK helpers (check k fails there by construction);
the verdict is build_car.ps1 (builder checks a..n, door solve, approach gate).
"""
import os
import subprocess
import sys
import time

HE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HE)
import numpy as np  # noqa: E402
import check_door_clearance as C  # noqa: E402
import roof_table as RT  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

OUT = os.environ.get('UTPP_DUCK_OUT', os.path.join(project_root(), r'Build\work\vehicle-entry-car\roof-pack3\proto'))
BODY = os.path.join(project_root(), r'Extracted\game\vehicle-entry-car\body-data')
RUN = os.path.join(project_root(), r'Build\work\vehicle-entry-car\run-sport1-door7b')   # the shipped sport1 driver clips


def open_scan(clip, g):
    """per capsule: worst penetration (cm) of the fully open door over the clip (120 Hz interpolated, lag independent), its frame, the gate."""
    m = C.Model(clip, 'sport1', 'driver', rear_ref=os.path.join(RT.GLB, 'p%sa_sport1_driver.anims.glb' % g), lag=0.0)
    su = float(m.su)
    gate = m.gate_cm('open')
    worst = np.full(len(m.names), -99.0)
    wf = np.zeros(len(m.names), int)
    for k in range(0, int(round(m.clip_end * 30)) + 1):
        pv, _ = m.pen_caps_V(m.body(int(round(k / 30.0 * C.HZ))), su)
        pv = pv * 100
        for j in np.flatnonzero(pv > worst):
            worst[j], wf[j] = pv[j], k
    return m.names, gate, worst, wf


def hips_height(clip):
    """Hips height (m) above the seat frame per clip frame (vehicle frame z)."""
    sk, t, Wt, Wr = C.load_clip(clip)
    ri = sk.index['Root']
    Re, pe = Wr[-1, ri], Wt[-1, ri]
    dj = C.load_door_data(C.DATA, 'sport1')
    st, sR = np.array(dj['seat']['left']['t']), np.array(dj['seat']['left']['R']).reshape(3, 3)
    A = sR @ Re.T
    return np.array([((A @ (Wt[k, sk.index['Hips']] - pe) + st) @ C.GLTF_TO_VEH.T)[2] for k in range(len(t))])


def summary(clip, g, label=''):
    st = RT.static(clip, 'sport1', 'driver')
    names, gate, worst, wf = open_scan(clip, g)
    bad = [(n, round(float(worst[j]), 2), int(wf[j])) for j, n in enumerate(names) if worst[j] > gate[j] - 1e-9]
    top = sorted(((round(float(worst[j]), 2), n, int(wf[j])) for j, n in enumerate(names)), reverse=True)[:4]
    hz = hips_height(clip)
    d = hz - hz[-1]
    rise = max(d[f] - d[31:f + 1].min() for f in range(31, len(d))) * 100   # largest rise of the pelvis after its running minimum from frame 31 on (sit - rise - sit = a bob)
    print('  hips above seated (cm) f26..58 step 4:', ' '.join('%d:%.0f' % (k, d[k] * 100) for k in range(26, 59, 4)), '| min f30-58 %.1f | largest re-rise after f31 %.1f' % (d[30:59].min() * 100, rise))
    print('%s roof %.2f/%.2f | body worst %s | open door: over-gate %s | top %s' % (
        label, st['roof_pen'], st['roof_gate'], sorted(((round(v, 1), k) for k, v in st['body'].items()), reverse=True)[:2], bad, top), flush=True)
    return st, bad


def main():
    argv = sys.argv[1:]
    k = argv.index('--') if '--' in argv else len(argv)
    row, tag = argv[0], argv[1]
    flags = argv[2:k]
    duck = argv[k + 1:]
    name = 'utpp_car_' + row
    g, cls, seat = RT.parse(name)
    assert (cls, seat) == ('sport1', 'driver'), row
    clip = os.path.join(RUN, 'roundtrip', name + '.anims.glb')
    os.makedirs(OUT, exist_ok=True)
    out = os.path.join(OUT, row + '.' + tag + '.glb')
    t0 = time.time()
    cmd = [sys.executable, '-I', os.path.join(HE, 'head_duck.py'), '--clip', clip, '--out', out, '--duck-roof-json', os.path.join(BODY, cls + '.json'), '--duck-roof-seat', 'left'] + duck
    r = subprocess.run(cmd, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    if r.returncode:
        print(r.stderr[-1500:])
        return 2
    for ln in r.stdout.splitlines():
        if ln.startswith('target'):
            print(ln[:300])
    summary(out, g, row + '.' + tag)
    if '--checks' in flags:
        ck = RT.checks(out, name, RUN)
        print('  checks failing:', [kk for kk, v in ck.items() if kk.endswith('_ok') and not v])
    print('  (%.0f s) %s' % (time.time() - t0, out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
