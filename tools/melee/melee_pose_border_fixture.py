"""Cuts the recorded fixture of the melee full pose borders (feet and Hips height at the start and end of the layer, feet while the
capsule stands) and of the layer's clip choice in held states, from a pose recording and the mod log of the same session.
Test: plugin/UTPP/tests/MeleePoseTest.cpp (extra arguments; dispatched by the "MPB1" magic).

  python -I melee_pose_border_fixture.py --pose SEG [SEG ...] --log utpp-....log --body F|M --out FILE
         --window LABEL WEAPON HH:MM:SS.fff HH:MM:SS.fff [--window ...] [--hold LABEL ...]

--pose    pose recording segments (utpp-pose*.utpppose) of one session, in time order (both stages are used)
--log     the mod log of the same session: 'MMMELEEFULL on / off' lines give the layer's timing, 'POS p=' lines the capsule
--window  a time span (local clock) and the melee weapon anim variable of the drawn weapon (w_melee_fists, w_melee_knife, ...); every
          full-body episode whose start lies in the span becomes one fixture window, from 0.1 s before its start to 0.7 s after its end
--hold    labels whose windows are held states (block): the test passes the held state to the layer
Layer timing model (the game's full-body fade): the fade rises over 0.2 s after an 'on' line and falls over 0.35 s after an 'off' line
with why=state (0.15 s otherwise), smoothstep; the layer writes while the fade is above 0; the log line is 16 ms ahead of the recording.
Output "MPB1" (little endian):
  "MPB1" | int32 nb | nb x { int16 parent, char name[32] } (parents before children) | int32 body (0 Female, 1 Male) | int32 nwin |
  nwin x { char label[32] | char weapon[32] | int32 hold | float capsuleMove (m, largest capsule distance from its position at the start
           of the episode, log POS lines) | float on | float off (s from the window start) | int32 nframes |
           nframes x { float dt (s since the previous frame; first = 1/60) | float weight (layer fade 0..1) | int32 writing |
                       int16 P0[nb*3], int16 Q0[nb*4] (stage 0: the engine pose before the mod) |
                       int16 P1[nb*3], int16 Q1[nb*4] (stage 1: the pose shown) } }   (P in 0.1 mm, Q xyzw x 32767)
Read-only on the recording and the log.
"""
import argparse
import os
import re
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'gametest'))
sys.path.insert(0, os.path.join(HERE, '..', 'analysis'))
import pose_helper_report as ph  # noqa: E402
import pose_metrics as pm  # noqa: E402

SHIFT = 0.016
FADE_IN = 0.2
FADE_OUT_STATE = 0.35
FADE_OUT = 0.15
BEFORE = 0.1
AFTER = 0.7
KEY_NAMES = ['Neck', 'Head', 'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand', 'LeftShoulder', 'LeftArm', 'LeftForeArm',
             'LeftHand', 'Spine1', 'Spine2', 'Spine3']
LOWER_NAMES = ['Hips', 'Spine', 'LeftUpLeg', 'LeftLeg', 'LeftFoot', 'RightUpLeg', 'RightLeg', 'RightFoot']
TS = re.compile(r'^\[\d{4}-\d\d-\d\d (\d\d):(\d\d):(\d\d\.\d+)\]')


def clock(text):
    h, m, s = text.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


def smooth(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def parse_log(path):
    full, pos = [], []
    for ln in open(path, encoding='utf-8', errors='replace'):
        m = TS.match(ln)
        if not m:
            continue
        t = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3))
        if 'UTPP MMMELEEFULL on ' in ln:
            full.append((t, True, ''))
        elif 'UTPP MMMELEEFULL off ' in ln:
            w = re.search(r'why=(\w+)', ln)
            full.append((t, False, w.group(1) if w else ''))
        elif 'POS p=' in ln:
            p = ln.split('POS p=')[1].split()[0].split(',')
            pos.append((t, float(p[0]), float(p[1]), float(p[2])))
    return full, np.array(pos)


def episodes(full):
    out, on = [], None
    for t, is_on, why in full:
        if is_on and on is None:
            on = t
        elif not is_on and on is not None:
            out.append((on, t, why))
            on = None
    return out


def fade_curve(full, times):
    """layer fade at the given times (seconds of the day): linear progress at the in / out rates, smoothstep"""
    ev = [(t + SHIFT, on, why) for t, on, why in full]
    prog, want, out_rate, k = 0.0, False, 1.0 / FADE_OUT, 0
    res = np.zeros(len(times))
    last = times[0]
    for i, t in enumerate(times):
        while k < len(ev) and ev[k][0] <= t:
            want = ev[k][1]
            if not want:
                out_rate = 1.0 / (FADE_OUT_STATE if ev[k][2] == 'state' else FADE_OUT)
            k += 1
        dt = max(0.0, t - last)
        last = t
        prog = min(1.0, prog + dt / FADE_IN) if want else max(0.0, prog - dt * out_rate)
        res[i] = smooth(prog)
    return res


def load(segs):
    g = ph.load_groups([segs[0]], ph.Config())[0]
    h = g['header']
    names = [h.names[i] for i in h.mapped()]
    rec, data = pm.load_recording(segs, names)
    keep = data['keep']
    nm = [rec.names[k] for k in keep]
    pos = {k: i for i, k in enumerate(keep)}
    par = [pos.get(rec.parent[k], -1) for k in keep]
    o = np.argsort(data['sec'], kind='stable')
    return nm, par, data['sec'][o], data['q'][o], data['t'][o]


def subset(names, par):
    """the needed bones and every ancestor, parents before children"""
    idx = {n: i for i, n in enumerate(names)}
    want = set()
    for n in KEY_NAMES + LOWER_NAMES:
        if n not in idx:
            raise SystemExit('bone missing in the recording: ' + n)
        b = idx[n]
        while b >= 0 and b not in want:
            want.add(b)
            b = par[b]
    order, done = [], set()

    def visit(b):
        if b in done:
            return
        if par[b] >= 0:
            visit(par[b])
        done.add(b)
        order.append(b)
    for b in sorted(want):
        visit(b)
    return order


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--pose', nargs='+', required=True)
    ap.add_argument('--log', required=True)
    ap.add_argument('--body', choices=('F', 'M'), required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--window', nargs=4, action='append', required=True, metavar=('LABEL', 'WEAPON', 'FROM', 'TO'))
    ap.add_argument('--hold', nargs='*', default=[])
    a = ap.parse_args(argv)
    names, par, sec, q, t = load(a.pose)
    keep = subset(names, par)
    newi = {b: k for k, b in enumerate(keep)}
    full, pos = parse_log(a.log)
    eps = episodes(full)
    fade = fade_curve(full, sec)
    wins = []
    for label, weapon, t0, t1 in a.window:
        f0, f1 = clock(t0), clock(t1)
        for k, (on, off, why) in enumerate(e for e in eps if f0 <= e[0] < f1):
            i = np.nonzero((sec >= on - BEFORE) & (sec <= off + AFTER))[0]
            if len(i) < 10:
                continue
            cap = 0.0
            if len(pos):
                sel = pos[(pos[:, 0] >= on - 0.5) & (pos[:, 0] <= off + 0.1)]
                if len(sel):
                    cap = float(np.max(np.linalg.norm(sel[:, 1:3] - sel[0, 1:3], axis=1)))
            wins.append(('%s%d' % (label, k), weapon, label in a.hold, cap, on, off, i))
    with open(a.out, 'wb') as fh:
        fh.write(b'MPB1' + struct.pack('<i', len(keep)))
        for b in keep:
            fh.write(struct.pack('<h', newi[par[b]] if par[b] >= 0 else -1) + names[b].encode()[:31].ljust(32, b'\0'))
        fh.write(struct.pack('<ii', 0 if a.body == 'F' else 1, len(wins)))
        for label, weapon, hold, cap, on, off, idx in wins:
            st = sec[idx[0]]
            fh.write(label.encode()[:31].ljust(32, b'\0') + weapon.encode()[:31].ljust(32, b'\0'))
            fh.write(struct.pack('<ifffi', 1 if hold else 0, cap, on - st, off - st, len(idx)))
            prev = None
            for i in idx:
                dt = 1.0 / 60.0 if prev is None else float(sec[i] - sec[prev])
                prev = i
                fh.write(struct.pack('<ffi', dt, float(fade[i]), 1 if fade[i] > 0.0 else 0))
                for stage in (0, 1):
                    P = np.nan_to_num(t[i, keep, stage].astype(np.float64))
                    Q = np.nan_to_num(q[i, keep, stage].astype(np.float64))
                    Q = Q / np.maximum(np.linalg.norm(Q, axis=-1, keepdims=True), 1e-9)
                    fh.write(np.clip(np.round(P * 1e4), -32767, 32767).astype('<i2').tobytes())
                    fh.write(np.clip(np.round(Q * 32767), -32767, 32767).astype('<i2').tobytes())
            print('%-16s %-24s hold=%d capsule %.2f m  %d frames  on %.2f s off %.2f s' % (label, weapon, hold, cap, len(idx), on - st, off - st))
    print('fixture', a.out, os.path.getsize(a.out), 'bytes,', len(wins), 'windows,', len(keep), 'bones')


if __name__ == '__main__':
    main()
