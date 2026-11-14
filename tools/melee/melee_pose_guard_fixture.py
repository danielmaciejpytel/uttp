"""Cuts the recorded fixture of the melee reference pose guard (src/mm/MeleePoseGuard.hpp, test: plugin/UTPP/tests/MeleePoseTest.cpp)
from a pose recording: windows of the ENGINE pose (stage 0 = before the mod writes) around chosen times.

  python -I melee_pose_guard_fixture.py --pose SEG [SEG ...] --body F|M --out FILE
         [--at HH:MM:SS.fff[=label] ...] [--signature] [--before 6] [--after 6] [--all]

--pose       compact recording segments (utpp-pose*.utpppose) of one session, in time order
--at         window centres (local clock of the recording); the frame nearest to each is the centre
--signature  also add a window around every frame that shows the reference pose signature (LeftArm local == RightArm local within 0.5 deg)
--all        one window with every frame of the recording (for local checks; large)
Output "MPG1" (little endian):
  "MPG1" | int32 nb | nb x { int16 parent, char name[32] } | int32 body (0 Female, 1 Male) | int32 nwin |
  nwin x { char label[32] | int32 nframes | nframes x { float dt (s since the previous frame of the window; first = 1/60),
           int16 P[nb*3] (local position, 0.1 mm), int16 Q[nb*4] (local rotation xyzw x 32767) } }
Read-only on the recording.
"""
import argparse
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'gametest'))
sys.path.insert(0, os.path.join(HERE, '..', 'analysis'))
import pose_helper_report as ph  # noqa: E402
import pose_metrics as pm  # noqa: E402

SIGNATURE_DEG = 0.5


def qang(a, b):
    a = a / np.linalg.norm(a, axis=-1, keepdims=True)
    b = b / np.linalg.norm(b, axis=-1, keepdims=True)
    return np.degrees(2 * np.arccos(np.clip(np.abs((a * b).sum(-1)), 0, 1)))


def parse_clock(text):
    h, m, s = text.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


def load(segs):
    g = ph.load_groups([segs[0]], ph.Config())[0]
    h = g['header']
    names = [h.names[i] for i in h.mapped()]
    rec, data = pm.load_recording(segs, names)
    keep = data['keep']
    pos = {k: i for i, k in enumerate(keep)}
    par = np.array([pos.get(rec.parent[k], -1) for k in keep], np.int16)
    o = np.argsort(data['sec'], kind='stable')
    return [rec.names[k] for k in keep], par, data['sec'][o], data['q'][o][:, :, 0], data['t'][o][:, :, 0]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--pose', nargs='+', required=True)
    ap.add_argument('--body', choices=('F', 'M'), required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--at', nargs='*', default=[])
    ap.add_argument('--signature', action='store_true')
    ap.add_argument('--before', type=int, default=6)
    ap.add_argument('--after', type=int, default=6)
    ap.add_argument('--all', action='store_true')
    a = ap.parse_args(argv)
    names, par, sec, q, t = load(a.pose)
    n = len(sec)
    wins = []
    if a.all:
        wins.append(('all', 0, n - 1))
    for item in a.at:
        clock, _, label = item.partition('=')
        c = int(np.argmin(np.abs(sec - parse_clock(clock))))
        wins.append((label or clock, max(0, c - a.before), min(n - 1, c + a.after)))
    if a.signature:
        la, ra = names.index('LeftArm'), names.index('RightArm')
        sig = np.nonzero(qang(q[:, la].astype(np.float64), q[:, ra].astype(np.float64)) < SIGNATURE_DEG)[0]
        for c in sig:
            if any(lo <= c <= hi for _, lo, hi in wins):
                continue
            h = int(sec[c] // 3600)
            m = int((sec[c] - h * 3600) // 60)
            wins.append(('ref %02d:%02d:%06.3f' % (h, m, sec[c] - h * 3600 - m * 60), max(0, c - a.before), min(n - 1, c + a.after)))
    if not wins:
        print('no windows')
        return 2
    P = np.clip(np.round(t * 10000.0), -32767, 32767).astype('<i2')
    Q = np.clip(np.round(q * 32767.0), -32767, 32767).astype('<i2')
    out = bytearray(b'MPG1')
    out += struct.pack('<i', len(names))
    for nm, p in zip(names, par):
        out += struct.pack('<h', int(p)) + nm.encode('ascii')[:31].ljust(32, b'\0')
    out += struct.pack('<ii', 0 if a.body == 'F' else 1, len(wins))
    for label, lo, hi in wins:
        out += label.encode('ascii')[:31].ljust(32, b'\0') + struct.pack('<i', hi - lo + 1)
        for k in range(lo, hi + 1):
            dt = float(sec[k] - sec[k - 1]) if k > lo else 1.0 / 60.0
            out += struct.pack('<f', dt) + P[k].tobytes() + Q[k].tobytes()
    tmp = a.out + '.part'
    with open(tmp, 'wb') as f:
        f.write(out)
    os.replace(tmp, a.out)
    print('%s: %d bones, %d windows, %d frames, %d bytes' % (a.out, len(names), len(wins), sum(hi - lo + 1 for _, lo, hi in wins), len(out)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
