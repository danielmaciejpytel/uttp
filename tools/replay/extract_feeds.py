"""Extract compact per-frame motion-matching feeds from recorded UTPP game logs (port replay, report 7 Etap 10.1 p.4).

Source data (read-only): Research\\logs\\night\\<session>\\logs\\utpp-*.log (MMTRACE lines) and
Research\\logs\\night\\<session>\\gametest\\<scenario>_r<N>.txt (step windows "=== step <name>  <t0> -> <t1>").

MMTRACE is NOT per frame (MotionHost.cpp): a "tick" line about once per second plus a "TRANS" line on every v1
transition. Per-frame feeds are therefore RECONSTRUCTED on a fixed grid (default 60 Hz):
  - velChar (character-frame capsule velocity) and pl (player yaw) are linearly interpolated between trace lines and
    rotated to world: vx = c*cx - s*cy, vy = s*cx + c*cy (inverse of integ::WorldToCharacter);
  - mv (stick), cam, loco (1 stand / 2 crouch / 3 sprint) are sample-and-hold from the last trace line;
  - v1 pick columns (clip, frame, cost, sUs) are sample-and-hold; trans = 1 on the frame a TRANS line falls into.
  - psmBlock is not in MMTRACE -> written as 0 (MotionAdapter DetailFromFeed then falls back to locoState).
  - vz is not in MMTRACE -> 0 (no jumps in the traced scenarios).

Output (text, one header + one line per frame, space separated; clip names have no spaces):
  # feed v1 session=<s> scenario=<n> log=<path> hz=<hz> t0=<iso>
  # step <name> <tStart> <tEnd>          (seconds relative to t0)
  # cols t dt vx vy vz mvx mvy yawDeg camDeg loco psm trans baseClip baseFlags baseCost baseMicros step
Stdlib only.  Usage:
  python extract_feeds.py --session <dir> --scenario <name> [--scenario ...] --out <dir> [--hz 60]
"""
from __future__ import annotations

import argparse
import bisect
import glob
import math
import os
import re
import sys
from datetime import datetime

TS_RE = re.compile(r'^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\]')
FIELD_RE = re.compile(r'(?:^|\s)([A-Za-z|]+)=(\S+)')
PAIR_RE = re.compile(r'\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)')
STEP_RE = re.compile(r'^=== step (\S+)\s+(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+) -> (\d\d:\d\d:\d\d\.\d+)')


def parse_ts(s: str) -> datetime:
    return datetime.strptime(s[:23], '%Y-%m-%d %H:%M:%S.%f')


def read_trace(path: str):
    out = []
    with open(path, 'rb') as f:
        text = f.read().decode('utf-8', errors='replace')
    for line in text.splitlines():
        m = TS_RE.match(line)
        if not m or 'MMTRACE' not in line:
            continue
        body = line.split('MMTRACE', 1)[1].strip()
        kind = body.split(' ', 1)[0]
        if kind not in ('TRANS', 'tick'):
            continue
        fl = dict(FIELD_RE.findall(' ' + body))
        try:
            vc = PAIR_RE.search(fl.get('velChar', ''))
            mv = PAIR_RE.search(fl.get('mv', ''))
            rec = {
                'ts': parse_ts(m.group(1)), 'kind': kind, 'clip': fl.get('clip', '-') or '-',
                'f': int(fl.get('f', '-1')), 'cost': float(fl.get('cost', 'nan')), 'sUs': float(fl.get('sUs', 'nan')),
                'cx': float(vc.group(1)) if vc else 0.0, 'cy': float(vc.group(2)) if vc else 0.0,
                'mvx': float(mv.group(1)) if mv else 0.0, 'mvy': float(mv.group(2)) if mv else 0.0,
                'cam': float(fl.get('cam', '0')), 'pl': float(fl.get('pl', '0')), 'loco': int(fl.get('loco', '0')),
                'gate': fl.get('gate', '0x0'), 'w': float(fl.get('w', '0')),
            }
        except (ValueError, AttributeError):
            continue
        out.append(rec)
    out.sort(key=lambda r: r['ts'])  # several writer threads
    return out


def read_steps(path: str):
    steps = []
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            m = STEP_RE.match(line)
            if m:
                t0 = parse_ts(m.group(2))
                t1 = parse_ts(m.group(2)[:11] + m.group(3))
                steps.append((m.group(1), t0, t1))
    return steps


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def extract(trace, steps, hz: float):
    t0 = steps[0][1]
    t1 = steps[-1][2]
    secs = [(r['ts'] - t0).total_seconds() for r in trace]
    dt = 1.0 / hz
    n = int(math.floor((t1 - t0).total_seconds() * hz))
    frames = []
    stepw = [(name, (a - t0).total_seconds(), (b - t0).total_seconds()) for name, a, b in steps]
    for k in range(1, n + 1):
        t = k * dt
        j = bisect.bisect_right(secs, t) - 1  # last line <= t
        if j < 0:
            continue  # no trace yet
        a = trace[j]
        b = trace[j + 1] if j + 1 < len(trace) else a
        ta, tb = secs[j], secs[j + 1] if j + 1 < len(trace) else secs[j]
        u = 0.0 if tb <= ta else min(1.0, max(0.0, (t - ta) / (tb - ta)))
        cx = a['cx'] + (b['cx'] - a['cx']) * u
        cy = a['cy'] + (b['cy'] - a['cy']) * u
        yaw = a['pl'] + wrap180(b['pl'] - a['pl']) * u
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        vx, vy = c * cx - s * cy, s * cx + c * cy
        trans = 0
        i = j
        while i >= 0 and secs[i] > t - dt:
            if trace[i]['kind'] == 'TRANS':
                trans = 1
                break
            i -= 1
        step = '-'
        for name, sa, sb in stepw:
            if sa <= t < sb:
                step = name
                break
        frames.append((t, dt, vx, vy, 0.0, a['mvx'], a['mvy'], wrap180(yaw), a['cam'], a['loco'], 0, trans,
                       a['clip'], a['f'], a['cost'], a['sUs'], step))
    return t0, stepw, frames


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--session', required=True, help='Research\\logs\\night\\<session> directory')
    ap.add_argument('--scenario', action='append', required=True, help='gametest scenario name (run 1)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--hz', type=float, default=60.0)
    a = ap.parse_args(argv)
    logs = sorted(glob.glob(os.path.join(a.session, 'logs', 'utpp-*.log')))
    if not logs:
        print('no utpp-*.log in', a.session)
        return 2
    trace = []
    for p in logs:
        trace += read_trace(p)
    trace.sort(key=lambda r: r['ts'])
    os.makedirs(a.out, exist_ok=True)
    sess = os.path.basename(os.path.normpath(a.session))
    rc = 0
    for sc in a.scenario:
        gt = os.path.join(a.session, 'gametest', sc + '_r1.txt')
        if not os.path.isfile(gt):
            print('MISSING', gt)
            rc = 1
            continue
        steps = read_steps(gt)
        if not steps:
            print('NO STEPS', gt)
            rc = 1
            continue
        t0, stepw, frames = extract(trace, steps, a.hz)
        lo, hi = steps[0][1], steps[-1][2]
        lines_in = [r for r in trace if lo <= r['ts'] <= hi]
        ntr = sum(1 for r in lines_in if r['kind'] == 'TRANS')
        path = os.path.join(a.out, f'{sess}_{sc}.feed')
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'# feed base session={sess} scenario={sc} log={os.path.basename(logs[0])} hz={a.hz:g} '
                    f't0={t0.isoformat(sep=" ")} trace_lines={len(lines_in)} trans_lines={ntr}\n')
            for name, sa, sb in stepw:
                f.write(f'# step {name} {sa:.3f} {sb:.3f}\n')
            f.write('# cols t dt vx vy vz mvx mvy yawDeg camDeg loco psm trans baseClip baseFlags baseCost baseMicros step\n')
            for fr in frames:
                f.write('%.4f %.5f %.3f %.3f %.3f %.2f %.2f %.1f %.1f %d %d %d %s %d %.3f %.0f %s\n' % fr)
        print(f'{path}: frames={len(frames)} trace_lines={len(lines_in)} trans_lines={ntr} steps={len(stepw)}')
    return rc


if __name__ == '__main__':
    sys.exit(main())
