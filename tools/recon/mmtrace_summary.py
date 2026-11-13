"""Summarize UTPP MMTRACE / MMGATE lines of a red4ext utpp log for a time window (read-only).

Usage: python mmtrace_summary.py <utpp.log> [HH:MM:SS start] [HH:MM:SS end]
Prints one compact line per trace entry: time, TRANS/tick, clip, frame, cost, |v|, rate, loco, move input, cam/pl yaw,
offset root, foot lock L/R, plus MMGATE transitions; MMTRACE feet / footstep / costcap lines keep their own label, as do the
MMAIR air phase changes (label 'air  ').
"""
import re
import sys

KEYS = ['clip', 'f', 'cost', '|v|', 'rate', 'loco', 'mv', 'cam', 'pl', 'off', 'L', 'R', 'bt', 'tgt']
SIDE_LINES = (('MMTRACE feet ', 'feet '), ('MMTRACE footstep ', 'step '), ('MMTRACE costcap ', 'ccap '))


def main():
    path = sys.argv[1]
    t0 = sys.argv[2] if len(sys.argv) > 2 else '00:00:00'
    t1 = sys.argv[3] if len(sys.argv) > 3 else '99:99:99'
    for line in open(path, encoding='utf-8', errors='replace'):
        m = re.match(r'\[\d{4}-\d\d-\d\d (\d\d:\d\d:\d\d\.\d+)\]', line)
        if not m or not (t0 <= m.group(1)[:8] <= t1):
            continue
        ts = m.group(1)[:11]
        if 'MMGATE' in line:
            g = re.search(r'MMGATE (.*)', line)
            print(f'{ts} GATE  {g.group(1)[:90]}')
            continue
        if 'UTPP MMAIR ' in line:  # AIR HOLD stage: air phase changes
            print(f'{ts} air   {line.split("UTPP MMAIR ", 1)[1].strip()[:110]}')
            continue
        if 'MMTRACE' not in line:
            continue
        # not trace samples: loco C0 feet / footstep lines, 0.5.8 cost cap events (own label, body as written)
        side = next(((tok, lab) for tok, lab in SIDE_LINES if tok in line), None)
        if side:
            print(f'{ts} {side[1]} {line.split(side[0], 1)[1].strip()[:110]}')
            continue
        kind = 'TRANS' if 'MMTRACE TRANS' in line else 'tick '
        parts = []
        for k in KEYS:
            mm = re.search(r'(?:^|\s)' + re.escape(k) + r'=(\S+)', line)
            if mm:
                v = mm.group(1)
                if k == 'clip':
                    v = v.replace('M_Neutral_', '')
                parts.append(f'{k}={v}')
        print(f'{ts} {kind} ' + ' '.join(parts))


if __name__ == '__main__':
    main()
