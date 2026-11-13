"""Cuts a glue-test fixture ("MBO3", see plugin/UTPP/tests/MeleeBaseReplay.hpp) from a recorded fists run: the first frames after each melee
upper-body mask start, i.e. the 0.15 s blend-in and the first punches, with the engine records RAW and the MM pose of the blended upper bones,
so that tests/MeleeBaseGlueTest.cpp replays the real blend through the production path (BlendIntoRecords with upperBits, CaptureEngineKeys, Apply).

  python -I fist_base_blend_fixture.py --pose utpp-pose-....utpppose --night RUN_DIR --body F|M --windows 0,2 --frames 50
         --fixture OUT.bin [--exe utpp_melee_base_offline.exe]

Inputs are read in place (the pose recording in <game>\\red4ext\\logs, the run folder with logs\\utpp-*.log); the fixture is the only output.
Model of the blend (calibrated on the 2026-11-13 runs, replay vs the recorded stage 1: rms 0.2..2.3 deg of arm elevation in the blend): the MM
pose of the upper bones is the pose shown right before the mask (stage 1, median of the last 3 frames); upperScale = 1 - smoothstep(t / 0.15);
the log's mask-on line is 16 ms ahead of the recording's first blended frame (SHIFT); the lower body fade (melee full body) follows the
MMMELEEFULL on line, smoothstep over 0.2 s. refElev (clip elevation of the striking arm at the matched time) is set for the frames whose blend
is nearly over (>= REF_FROM s after the mask start) and whose arm is stretched (>= 45 cm), from the replay of the current code and the shipped clip table inputs.
The aim tilt layer is not part of the replay (the tests measure this layer alone).
"""
import argparse
import os
import re
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fist_base_offline as fbo  # noqa: E402

SHIFT = 0.016
REF_FROM = 0.12
UPPER_BLEND = ['Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head', 'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand', 'LeftShoulder',
               'LeftArm', 'LeftForeArm', 'LeftHand']


def smooth(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def nlerp(a, b, w):
    s = 1.0 if np.dot(a, b) >= 0 else -1.0
    q = (1 - w) * a + w * s * b
    return q / np.linalg.norm(q)


def melee_windows(log):
    """(mask on, mask off) of the melee mask reason (melee=1) and the melee full body (on, off) windows, seconds of the day"""
    mask, full, cur, curf = [], [], None, None
    for ln in open(log, encoding='utf-8', errors='replace'):
        m = re.match(r'\[2026-\d\d-\d\d (\d\d:\d\d:\d\d\.\d+)\].*UTPP (MMMASK|MMMELEEFULL) (on|off)\b(.*)', ln)
        if not m:
            continue
        t = fbo.sec_of(m.group(1))
        kind, onoff, rest = m.group(2), m.group(3), m.group(4)
        if kind == 'MMMASK':
            me = re.search(r'melee=(\d)', rest)
            if onoff == 'on' and me and me.group(1) == '1' and cur is None:
                cur = t
            elif onoff == 'off' and cur is not None:
                mask.append((cur, t))
                cur = None
        elif onoff == 'on' and curf is None:
            curf = t
        elif onoff == 'off' and curf is not None:
            full.append((curf, t))
            curf = None
    return mask, full


def build_frames(rec, log, windows, nframes):
    mask, full = melee_windows(log)
    lower = [n for n in rec.names if fbo.is_lower(n)]
    out = []
    for wi in windows:
        wa, wb = mask[wi]
        pre = rec.idx(wa - 0.12, wa - 0.005)[-3:]
        mm_p = np.nanmedian(rec.P(pre, 1), 0)
        mm_q = np.stack([fbo.qmean(rec.Q(pre, 1)[:, k]) for k in range(len(rec.names))])
        fon = [w for w in full if abs(w[0] - wa) < 0.5]
        fon = fon[0][0] if fon else None
        n_win = 0
        for i in rec.idx(wa, wb):
            if n_win >= nframes:
                break
            tt = float(rec.t[i])
            P0, Q0 = rec.P([i], 0)[0].copy(), rec.Q([i], 0)[0].copy()
            if not (np.isfinite(P0).all() and np.isfinite(Q0).all()):
                continue
            m = smooth((tt - wa + SHIFT) / 0.15)
            ls = 1.0 - (smooth((tt - fon + SHIFT) / 0.2) if fon is not None and tt >= fon - 0.1 else 0.0)
            out.append(dict(i=int(i), t=tt, since=tt - wa, w='punches', P=P0, Q=Q0, Peng=P0.copy(), Qeng=Q0.copy(),
                            P1=rec.P([i], 1)[0], Q1=rec.Q([i], 1)[0],
                            mmP=np.array([mm_p[rec.cat[x]] for x in lower]), mmQ=np.array([mm_q[rec.cat[x]] for x in lower]),
                            mmUP=np.array([mm_p[rec.cat[x]] for x in UPPER_BLEND]), mmUQ=np.array([mm_q[rec.cat[x]] for x in UPPER_BLEND]),
                            meleeW=m, lowerScale=ls, upperScale=1.0 - m, base=1, dt=-1.0 if n_win == 0 else tt - out[-1]['t']))
            n_win += 1
    # the blended upper bones as the game shows them (for the elevation / extension measurements of analyse)
    for f in out:
        us = f['upperScale']
        if us > 0:
            for k, bn in enumerate(UPPER_BLEND):
                b = rec.cat[bn]
                f['P'][b] = (1 - us) * f['Peng'][b] + us * f['mmUP'][k]
                f['Q'][b] = nlerp(f['Qeng'][b], f['mmUQ'][k], us)
    return out, lower


def write_mbo3(path, rec, body, frames, lower, upper_names, subset=True):
    names = list(rec.names)
    keep = list(range(len(names)))
    if subset:
        want = set(rec.cat[n] for n in fbo.NEEDED + list(lower) + list(upper_names))
        closure = set()
        for b in want:
            while b >= 0 and b not in closure:
                closure.add(b)
                b = rec.parent[b]
        keep = sorted(closure)
    newi = {b: k for k, b in enumerate(keep)}
    with open(path, 'wb') as fh:
        fh.write(b'MBO3' + struct.pack('<i', len(keep)))
        for b in keep:
            fh.write(struct.pack('<h', newi[rec.parent[b]] if rec.parent[b] >= 0 else -1) + names[b].encode()[:31].ljust(32, b'\0'))
        fh.write(struct.pack('<ii', 0 if body == 'F' else 1, len(lower)))
        fh.write(b''.join(struct.pack('<i', newi[rec.cat[n]]) for n in lower))
        fh.write(struct.pack('<i', len(upper_names)))
        fh.write(b''.join(struct.pack('<i', newi[rec.cat[n]]) for n in upper_names))
        fh.write(struct.pack('<i', len(frames)))
        for f in frames:
            fh.write(f['Peng'][keep].astype('<f4').tobytes() + f['Qeng'][keep].astype('<f4').tobytes() + f['mmP'].astype('<f4').tobytes() +
                     f['mmQ'].astype('<f4').tobytes() + f['mmUP'].astype('<f4').tobytes() + f['mmUQ'].astype('<f4').tobytes())
            fh.write(struct.pack('<fffI', f['meleeW'], f['lowerScale'], f['upperScale'], f['base']))
            fh.write(struct.pack('<ffi', f['dt'], f.get('refElev', float('nan')), f.get('side', 0)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose', required=True)
    ap.add_argument('--night', required=True)
    ap.add_argument('--body', required=True, choices=['F', 'M'])
    ap.add_argument('--windows', default='0')
    ap.add_argument('--frames', type=int, default=50)
    ap.add_argument('--fixture', required=True)
    ap.add_argument('--exe', default=os.path.join(fbo.ROOT, 'Build', 'UTPP', 'utpp_melee_base_offline.exe'),
                    help='the replay exe built from the CURRENT sources (ninja -C Build/UTPP utpp_melee_base_offline); it must read MBO3')
    a = ap.parse_args()
    import glob
    rec = fbo.Recording(a.pose, a.night)
    log = glob.glob(os.path.join(a.night, 'logs', 'utpp-*.log'))[0]
    wins = [int(x) for x in a.windows.split(',')]
    frames, lower = build_frames(rec, log, wins, a.frames)
    tmpd = os.path.join(os.path.dirname(os.path.abspath(a.fixture)), '_tmp')
    os.makedirs(tmpd, exist_ok=True)
    inp, outp = os.path.join(tmpd, 'fixture_in.bin'), os.path.join(tmpd, 'fixture_out.bin')
    write_mbo3(inp, rec, a.body, frames, lower, UPPER_BLEND, subset=False)
    res, clipnames = fbo.run_exe(a.exe, inp, outp)
    rows = fbo.analyse(rec, a.body, frames, res, clipnames, fbo.load_clip_table(a.body))
    n_ref = 0
    for fr, row in zip(frames, rows):
        fr['side'] = 0 if row['side'] == 'Right' else 1
        ok = row['ec'] is not None and row['ext'] >= 45.0 and fr['since'] >= REF_FROM
        fr['refElev'] = row['ec'] if ok else float('nan')
        n_ref += 1 if ok else 0
    write_mbo3(a.fixture, rec, a.body, frames, lower, UPPER_BLEND, subset=True)
    for p in (inp, outp, outp + '.clips.txt'):
        if os.path.exists(p):
            os.remove(p)
    try:
        os.rmdir(tmpd)
    except OSError:
        pass
    print('fixture', a.fixture, os.path.getsize(a.fixture), 'bytes,', len(frames), 'frames,', n_ref, 'with a reference')


if __name__ == '__main__':
    main()
