"""Reads the 'UTPP MELEEPOSE' lines of a mod log (melee full pose layer, its borders and the reference pose guard; cumulative counters,
about once per second while a melee weapon is out and diagnostic logs are on) and reports the counter increase per time window.

  python -I melee_pose_log.py LOG [--window 10] [--from HH:MM:SS] [--to HH:MM:SS] [--json OUT]
                              [--min-written N] [--max-guard-passed N] [--max-busy-share F]

--window            window length in seconds (default 10); a window holds the increase between its first and last line
--from / --to       only lines in this span (local clock of the log)
--json              also write the windows as JSON
Checks (exit code 1 when one fails; every check is reported):
  counters          no counter decreases inside the log (a decrease = the plugin was reloaded: reported, the windows restart there)
  --min-written     at least N written calls over the whole span
  --max-guard-passed  at most N reference-pose frames that the guard could not replace over the whole span
  --max-busy-share  busy calls (another animation job owned the layer) at most this share of the calls over the whole span
"""
import argparse
import json
import re
import sys

LINE = re.compile(r'^\[\d{4}-\d\d-\d\d (\d\d):(\d\d):(\d\d(?:\.\d+)?)\].*\bUTPP MELEEPOSE (.*)$')
PAIR = re.compile(r'(\w+)=(-?[0-9.]+)')
COUNTERS = ('calls', 'written', 'held', 'relocks', 'frozen', 'toBase', 'busy', 'noTable', 'noWeapon', 'border', 'tail', 'planted', 'steps',
            'hipsLimited', 'guardCalls', 'guardReplaced', 'guardPassed', 'guardBusy')


def clock(text):
    h, m, s = text.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


def hms(sec):
    h = int(sec // 3600)
    m = int((sec - h * 3600) // 60)
    return '%02d:%02d:%06.3f' % (h, m, sec - h * 3600 - m * 60)


def parse(lines):
    """[(seconds of the day, {key: value})] of every MELEEPOSE line"""
    out = []
    for ln in lines:
        m = LINE.match(ln)
        if not m:
            continue
        t = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3))
        vals = {k: float(v) for k, v in PAIR.findall(m.group(4))}
        out.append((t, vals))
    return out


def windows(samples, length):
    """per window: start, end, increase of every counter (a decrease restarts the base: reload), the resets seen"""
    res, resets = [], []
    i = 0
    while i < len(samples):
        t0 = samples[i][0]
        j = i
        while j + 1 < len(samples) and samples[j + 1][0] < t0 + length:
            j += 1
        inc = {k: 0.0 for k in COUNTERS}
        base = samples[i][1]
        for k in range(i + 1, j + 1):
            cur = samples[k][1]
            dropped = any(cur.get(c, 0.0) < base.get(c, 0.0) for c in COUNTERS)
            if dropped:
                resets.append(samples[k][0])
                for c in COUNTERS:
                    inc[c] += cur.get(c, 0.0)
            else:
                for c in COUNTERS:
                    inc[c] += cur.get(c, 0.0) - base.get(c, 0.0)
            base = cur
        # the first line of the window carries the increase since the previous window's last line
        if i > 0:
            prev, cur = samples[i - 1][1], samples[i][1]
            if any(cur.get(c, 0.0) < prev.get(c, 0.0) for c in COUNTERS):
                resets.append(samples[i][0])
                for c in COUNTERS:
                    inc[c] += cur.get(c, 0.0)
            else:
                for c in COUNTERS:
                    inc[c] += cur.get(c, 0.0) - prev.get(c, 0.0)
        res.append({'from': samples[i][0], 'to': samples[j][0], 'lines': j - i + 1, 'inc': inc,
                    'w': samples[j][1].get('w', 0.0), 'state': int(samples[j][1].get('state', -1))})
        i = j + 1
    return res, sorted(set(resets))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('log')
    ap.add_argument('--window', type=float, default=10.0)
    ap.add_argument('--from', dest='t_from')
    ap.add_argument('--to', dest='t_to')
    ap.add_argument('--json')
    ap.add_argument('--min-written', type=int)
    ap.add_argument('--max-guard-passed', type=int)
    ap.add_argument('--max-busy-share', type=float)
    a = ap.parse_args(argv)
    with open(a.log, encoding='utf-8', errors='replace') as fh:
        samples = parse(fh)
    lo = clock(a.t_from) if a.t_from else -1.0
    hi = clock(a.t_to) if a.t_to else 1e9
    samples = [s for s in samples if lo <= s[0] <= hi]
    if not samples:
        print('no UTPP MELEEPOSE lines (diagnostic logs off, no melee weapon drawn, or an older plugin)')
        return 1 if (a.min_written or 0) > 0 else 0
    wins, resets = windows(samples, a.window)
    cols = ('calls', 'written', 'held', 'relocks', 'frozen', 'toBase', 'planted', 'steps', 'hipsLimited', 'busy', 'guardReplaced',
            'guardPassed')
    print('%-12s %-12s %5s ' % ('from', 'to', 'lines') + ' '.join('%9s' % c[:9] for c in cols))
    total = {c: 0.0 for c in COUNTERS}
    for w in wins:
        for c in COUNTERS:
            total[c] += w['inc'][c]
        print('%-12s %-12s %5d ' % (hms(w['from']), hms(w['to']), w['lines']) + ' '.join('%9d' % w['inc'][c] for c in cols))
    print('%-12s %-12s %5d ' % ('total', '', len(samples)) + ' '.join('%9d' % total[c] for c in cols))
    ok = True
    if resets:
        print('counters restarted at ' + ', '.join(hms(t) for t in resets) + ' (plugin reloaded): windows count from there')
    if a.min_written is not None:
        good = total['written'] >= a.min_written
        ok &= good
        print('[%s] written %d >= %d' % ('PASS' if good else 'FAIL', total['written'], a.min_written))
    if a.max_guard_passed is not None:
        good = total['guardPassed'] <= a.max_guard_passed
        ok &= good
        print('[%s] guard passed %d <= %d' % ('PASS' if good else 'FAIL', total['guardPassed'], a.max_guard_passed))
    if a.max_busy_share is not None:
        share = total['busy'] / total['calls'] if total['calls'] > 0 else 0.0
        good = share <= a.max_busy_share
        ok &= good
        print('[%s] busy share %.3f <= %.3f' % ('PASS' if good else 'FAIL', share, a.max_busy_share))
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as fh:
            json.dump({'windows': wins, 'resets': resets, 'total': total}, fh, indent=1)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
