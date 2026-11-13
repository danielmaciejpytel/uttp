"""Harness helper of car_cases.ps1 for the ragdoll get-up metric (carhit cases): bodytrace CSV pick + trim, then getup_metric.py.

  python -I bt_getup.py snap    --dir <game>\\red4ext\\logs [--wait 2]
        JSON on stdout: {"time", "files": {name: size}, "active": bool, "growing": [names]}. 'active' = a body-trace CSV grew
        during --wait (the continuous capture is already on, e.g. "Turn all logs on"): the caller must not switch it off afterwards.
  python -I bt_getup.py collect --dir <game>\\red4ext\\logs --snap snap.json --out <folder> --log-window <log lines file>
        [--digest <harness digest.txt> ...] [--metric <...\\tools\\ragdoll\\getup_metric.py>] [--seq N] [--margin 0.5]
        Picks the CSV of this run (a file that is new or grew since the snapshot, posthelper files excluded, whose arm time lies
        before the release of the last episode in the logs), writes a TRIMMED copy (all header lines + the rows of the episode
        window +- margin; the files are GB sized, the window is the part getup_metric.py reads anyway) into --out and runs
        getup_metric.py on it (getup_metric.json + getup_metric.txt in --out, the M1..M5 summary is printed).
        Exit code: 0 = metric ran (PASS or FAIL verdict, see the printed summary), 2 = no usable CSV / log / metric error.

The trimmed copy has the same header as the source plus a first '# trimmed window ...' comment line (the same convention as the
r6a window copy), so getup_metric.py reads it exactly like the whole file. Read-only on the game folder.
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import time

sys.dont_write_bytecode = True

DEFAULT_METRIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'Build', 'wt-integ-r6', 'tools', 'ragdoll', 'getup_metric.py')


def trace_files(d):
    out = []
    for p in glob.glob(os.path.join(d, 'utpp-bodytrace-*.csv')):
        n = os.path.basename(p)
        if n.startswith('utpp-bodytrace-posthelper-'):
            continue
        out.append(p)
    return out


def sizes(d):
    r = {}
    for p in trace_files(d):
        try:
            r[os.path.basename(p)] = os.path.getsize(p)
        except OSError:
            pass
    return r


def cmd_snap(a):
    s1 = sizes(a.dir)
    time.sleep(max(0.0, a.wait))
    s2 = sizes(a.dir)
    growing = sorted(n for n, v in s2.items() if v > s1.get(n, -1))
    print(json.dumps({'time': time.time(), 'files': s2, 'active': bool(growing), 'growing': growing}))
    return 0


def load_metric(path):
    path = os.path.abspath(path)
    if not os.path.isfile(path):
        raise ValueError('getup_metric.py not found: %s' % path)
    sys.path.insert(0, os.path.dirname(path))
    import getup_metric as gm  # noqa: E402
    return gm


def header_end(f):
    """Byte offset after the column header line ('ticks,...'); header lines are '#' lines and that one."""
    f.seek(0)
    while True:
        line = f.readline()
        if not line:
            return f.tell()
        if line.startswith(b'ticks'):
            return f.tell()
        if not line.startswith(b'#'):
            return f.tell() - len(line)


def tick_of(line):
    c = line.find(b',')
    if c <= 0:
        return None
    try:
        return int(line[:c])
    except ValueError:
        return None


def seek_start(f, begin, size, tk_lo):
    """Offset of a line boundary at or before the first row with ticks >= tk_lo (ticks grow along the file)."""
    lo, hi = begin, size
    while hi - lo > (1 << 20):
        mid = (lo + hi) // 2
        f.seek(mid)
        f.readline()                      # partial line
        tk = tick_of(f.readline())
        if tk is None or tk >= tk_lo:
            hi = mid
        else:
            lo = mid
    return lo


def trim(src, dst, header, t_lo, t_hi):
    """Header + rows with arm-relative time in [t_lo, t_hi] -> dst. Returns (rows, bytes)."""
    arm, hz = header.arm, header.hz
    tk_lo = int(arm + t_lo * hz)
    tk_hi = int(arm + t_hi * hz)
    size = os.path.getsize(src)
    rows = 0
    with open(src, 'rb') as f, open(dst, 'wb') as o:
        he = header_end(f)
        f.seek(0)
        head = f.read(he)
        note = '# trimmed window %.3f..%.3f s after arm_ticks; source %s %d bytes (bt_getup.py)\n' % (
            t_lo, t_hi, os.path.basename(src), size)
        o.write(note.encode('ascii'))
        o.write(head)
        start = seek_start(f, he, size, tk_lo)
        f.seek(start)
        if start > he:
            f.readline()
        for line in f:
            tk = tick_of(line)
            if tk is None:
                continue
            if tk < tk_lo:
                continue
            if tk > tk_hi:
                break
            o.write(line)
            rows += 1
    return rows, os.path.getsize(dst)


def cmd_collect(a):
    gm = load_metric(a.metric)
    logs = [a.log_window] + list(a.digest or [])
    logs = [p for p in logs if p and os.path.isfile(p)]
    if not logs:
        print('bt_getup: no log input (window file missing)', file=sys.stderr)
        return 2
    events = gm.parse_log(logs)
    try:
        seq, ep = gm.pick_episode(events, a.seq)
    except ValueError as e:
        print('bt_getup: %s (the ragdoll episode is not in the log window)' % e, file=sys.stderr)
        return 2
    if 'release' not in ep:
        print('bt_getup: episode seq=%s has no release line' % seq, file=sys.stderr)
        return 2
    snap = {'time': 0.0, 'files': {}}
    if a.snap and os.path.isfile(a.snap):
        with open(a.snap, 'r', encoding='utf-8') as f:
            snap = json.load(f)
    cands = []
    for p in trace_files(a.dir):
        n = os.path.basename(p)
        try:
            sz, mt = os.path.getsize(p), os.path.getmtime(p)
        except OSError:
            continue
        if n not in snap['files'] or sz > snap['files'][n] or mt >= snap['time'] - 5.0:
            cands.append((mt, p))
    cands.sort(reverse=True)
    if not cands:
        print('bt_getup: no body-trace CSV is new or grew since the snapshot in %s (continuous capture not running?)' % a.dir, file=sys.stderr)
        return 2
    chosen = None
    notes = []
    for mt, p in cands:
        try:
            h = gm.parse_header(p)
        except (ValueError, OSError) as e:
            notes.append('%s: header unusable (%s)' % (os.path.basename(p), e))
            continue
        day0, _ = gm.arm_local_seconds_of_day(h, a.utc_offset_hours)
        d = ep['release'] - day0
        if d < -43200:
            d += 86400
        if d > 43200:
            d -= 86400
        dur = mt - h.utc.timestamp()
        if d < -1.0 or d > dur + 5.0:
            notes.append('%s: release at %.1f s after its arm time, file covers %.1f s' % (os.path.basename(p), d, dur))
            continue
        chosen = (p, h, day0)
        break
    if chosen is None:
        print('bt_getup: no candidate CSV covers the episode: ' + '; '.join(notes), file=sys.stderr)
        return 2
    src, h, day0 = chosen

    def rel(s):
        d = s - day0
        if d < -43200:
            d += 86400
        if d > 43200:
            d -= 86400
        return d
    t_rel = rel(ep['release'])
    lo = t_rel - 1.0
    if 'start' in ep:
        lo = min(lo, rel(ep['start']) - 1.3)
    hi = (rel(ep['end']) if 'end' in ep else t_rel + 6.0) + gm.WINDOW_POST_S + 0.1
    os.makedirs(a.out, exist_ok=True)
    dst = os.path.join(a.out, os.path.basename(src)[:-4] + '.window.csv')
    rows, nbytes = trim(src, dst, h, lo - a.margin, hi + a.margin)
    print('bt_getup: episode seq=%s release %.3f s after arm; source %s (%d bytes) -> %s (%d rows, %d bytes)'
          % (seq, t_rel, os.path.basename(src), os.path.getsize(src), dst, rows, nbytes))
    if rows == 0:
        print('bt_getup: the window holds no rows', file=sys.stderr)
        return 2
    out_json = os.path.join(a.out, 'getup_metric.json')
    cmd = [sys.executable, '-I', os.path.abspath(a.metric), '--csv', dst, '--out', out_json]
    for lg in logs:
        cmd += ['--log', lg]
    if a.seq is not None:
        cmd += ['--seq', str(a.seq)]
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    # M4 reads the 'UTPP MOTION ... db= clip=' lines (mm.trace) and the ground estimate the '[night] POS' lines: without them the metric
    # still prints PASS ("no Jumps / Lands pick") - say plainly that the verdict is not evaluable
    notes = []
    if not events['mm']:
        notes.append('M4 NOT EVALUABLE: no MOTION line in the logs (mm.trace off?) - the M4 PASS above means nothing')
    if not events['pos']:
        notes.append('GROUND NOT EVALUABLE FROM POS: no [night] POS line in the logs (NightAuto off?) - M1 / M3 / M5 use the fallback ground')
    with open(os.path.join(a.out, 'getup_metric.txt'), 'w', encoding='utf-8') as f:
        f.write(r.stdout + (('\n[stderr]\n' + r.stderr) if r.stderr.strip() else '') + ''.join('\n' + n for n in notes) + '\n')
    print(r.stdout.rstrip())
    for n in notes:
        print('bt_getup: ' + n)
    if r.stderr.strip():
        print(r.stderr.rstrip(), file=sys.stderr)
    print('bt_getup: getup_metric exit %d (0 = every metric passes, 1 = at least one metric fails, 2 = input error) -> %s' % (r.returncode, out_json))
    return 2 if r.returncode == 2 else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('snap')
    s.add_argument('--dir', required=True)
    s.add_argument('--wait', type=float, default=2.0)
    s = sub.add_parser('collect')
    s.add_argument('--dir', required=True)
    s.add_argument('--snap', default='')
    s.add_argument('--out', required=True)
    s.add_argument('--log-window', required=True)
    s.add_argument('--digest', action='append')
    s.add_argument('--metric', default=DEFAULT_METRIC)
    s.add_argument('--seq', type=int)
    s.add_argument('--margin', type=float, default=0.5)
    s.add_argument('--utc-offset-hours', type=float)
    a = ap.parse_args(argv)
    try:
        return cmd_snap(a) if a.cmd == 'snap' else cmd_collect(a)
    except (OSError, ValueError) as e:
        print('bt_getup: %s' % e, file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
