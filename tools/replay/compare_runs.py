"""Before / after comparison of two builds on recorded game sessions, both bodies, one command.

Runs, for every session, the per-frame game feed dump (<night logs>/<run>/logs/utpp-feed-*.txt) through
  - utpp_motion_replay (matcher / evaluator / foot locker lab): one call per body with all its sessions, --footlock --brief;
    read: FLTAB ALL (slide while planted, correction), FLTOT maxCorr, FLPEL pelvis offset, SUMMARY clip changes per second;
  - utpp_replay_pose_stages (the real runtime path: driver, blend stack, root frame, host leg tail) with the session's settings
    log and the recorded yaw ahead, summary only; read: RUNTIME (clip changes, foot correction, procedural yaw per turn, Feed cost).
and prints one table per body (before, after, after - before) plus the per-session runtime lines.

    python compare_runs.py --before-dir <build A> --after-dir <build B> [--runs woman:<run> man:<run> ...]
                           [--after-replay-args "--opt pelvis=1"] [--after-stages-args "..."] [--no-stages] [--repeat N]

--before-dir / --after-dir: folders that hold utpp_motion_replay.exe and utpp_replay_pose_stages.exe (a build folder or a copy).
--repeat N runs the runtime path N times and reports the median Feed p50 / p95 (cost comparisons need repeats).
Exit code 1 when a tool fails or a run reports REPLAY_FAILS != 0.
"""
import argparse
import glob
import os
import re
import shlex
import statistics
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))   # the tools folder
from utpp_paths import project_root  # noqa: E402

# Reference sessions: Female (all moves, keyboard, crouch) and Male (the same three).
DEFAULT_RUNS = ['woman:20261118-121140', 'woman:20261118-122228', 'woman:20261118-122643',
                'man:20261118-130756', 'man:20261118-131922', 'man:20261118-132319']


def one(pattern):
    hits = sorted(glob.glob(pattern))
    if not hits:
        raise SystemExit('missing: ' + pattern)
    return hits[0]


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True, errors='replace')
    return p.returncode, p.stdout + p.stderr


def kv(line):
    return {k: v for k, v in re.findall(r'(\w+)=(\S+)', line)}


def parse_replay(text):
    out = {}
    secs = changes = 0.0
    for line in text.splitlines():
        f = line.split()
        if line.startswith('FLTAB ALL'):
            out.update(slide_post_cms=float(f[4]), slide_post_p95=float(f[5]), corr_p95=float(f[11]), corr_max=float(f[12]))
        elif line.startswith('FLTOT'):
            out['fl_max_corr'] = float(f[4])
        elif line.startswith('FLPEL'):
            m = re.search(r'\|z\| cm p50 \S+ p95 (\S+)', line)
            if m:
                out['pelvis_z_p95'] = float(m.group(1))
        elif line.startswith('SUMMARY'):
            d = kv(line)
            secs += float(d['secs'])
            changes += float(d['clipChangesPerS']) * float(d['secs'])
        elif line.startswith('REPLAY_FAILS='):
            out['fails'] = int(line.split('=')[1])
    if secs > 0:
        out['clip_changes_s'] = changes / secs
    return out


def parse_runtime(text):
    out = None
    for line in text.splitlines():
        if line.startswith('RUNTIME') or line.startswith('PICKS'):
            d = kv(line)
            out = out or {}
            out.update({k: (float(v) if k != 'run' else v) for k, v in d.items()})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--before-dir', required=True)
    ap.add_argument('--after-dir', required=True)
    ap.add_argument('--runs', nargs='+', default=DEFAULT_RUNS, help='body:run (body = woman | man)')
    ap.add_argument('--root', default=project_root())
    ap.add_argument('--before-replay-args', default='')
    ap.add_argument('--after-replay-args', default='')
    ap.add_argument('--before-stages-args', default='')
    ap.add_argument('--after-stages-args', default='')
    ap.add_argument('--no-stages', action='store_true')
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--jobs', type=int, default=8)
    a = ap.parse_args()

    night = os.path.join(a.root, 'Research', 'logs', 'night')
    data = os.path.join(a.root, 'Data')
    sessions = {}
    for spec in a.runs:
        body, rid = spec.split(':', 1)
        logs = os.path.join(night, rid, 'logs')
        sessions.setdefault(body, []).append((rid, one(os.path.join(logs, 'utpp-feed-*.txt')), one(os.path.join(logs, 'utpp-2*.log'))))

    builds = {'before': (a.before_dir, a.before_replay_args, a.before_stages_args),
              'after': (a.after_dir, a.after_replay_args, a.after_stages_args)}
    jobs = {}
    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        for side, (d, rargs, sargs) in builds.items():
            for body, lst in sessions.items():
                cmd = [os.path.join(d, 'utpp_motion_replay.exe'), '--db', os.path.join(data, 'utpp_motion_%s.utppmm' % body),
                       '--footlock', '--brief'] + shlex.split(rargs, posix=False) + [feed for _, feed, _ in lst]
                jobs[(side, body, 'replay', None, 0)] = pool.submit(run, cmd)
                if a.no_stages:
                    continue
                for rid, feed, log in lst:
                    for k in range(a.repeat):
                        cmd = [os.path.join(d, 'utpp_replay_pose_stages.exe'), data, os.path.join(data, 'utpp_base.utppmm'), feed, '-',
                               '--settings-log', log, '--ahead', 'recorded', '--body', body] + shlex.split(sargs, posix=False)
                        jobs[(side, body, 'stages', rid, k)] = pool.submit(run, cmd)
        res = {k: f.result() for k, f in jobs.items()}

    bad = False
    for k, (rc, text) in res.items():
        if rc != 0 or (k[2] == 'replay' and parse_replay(text).get('fails', 0) != 0):
            bad = True
            print('FAIL %s rc=%d' % (' '.join(str(x) for x in k), rc))
            print('\n'.join(text.splitlines()[-5:]))

    rows = [('slide_post_cms', 'replay: planted slide after locker, cm/s (FLTAB ALL)'),
            ('slide_post_p95', 'replay: planted slide p95, cm/s'),
            ('corr_p95', 'replay: foot correction p95, cm'),
            ('fl_max_corr', 'replay: foot correction max, cm (FLTOT)'),
            ('pelvis_z_p95', 'replay: pelvis |z| p95, cm (FLPEL)'),
            ('clip_changes_s', 'replay: clip changes / s'),
            ('rt_clip_changes_s', 'runtime: clip changes / s'),
            ('rt_corr_max', 'runtime: foot correction max, cm'),
            ('rt_corr_p95', 'runtime: foot correction p95, cm'),
            ('rt_proc_turn', 'runtime: procedural yaw per turn, deg'),
            ('rt_proc_share', 'runtime: procedural yaw share'),
            ('rt_feed_p50', 'runtime: Feed us p50 (median of repeats)'),
            ('rt_feed_p95', 'runtime: Feed us p95 (median of repeats)'),
            ('rt_history', 'runtime: searches with the live pose history'),
            ('stop_picks', 'runtime: stop picks'),
            ('stop_wrong', 'runtime: stop picks, first-frame foot offset > 10 cm'),
            ('stop_off_p50', 'runtime: stop pick foot offset p50, cm'),
            ('stop_drift_mean', 'runtime: stop planted drift first 0.3 s mean, cm'),
            ('stop_drift_p95', 'runtime: stop planted drift first 0.3 s p95 (max of runs), cm'),
            ('start_picks', 'runtime: start picks'),
            ('start_wrong', 'runtime: start picks, first-frame foot offset > 10 cm'),
            ('start_off_p50', 'runtime: start pick foot offset p50, cm'),
            ('start_drift_mean', 'runtime: start planted drift first 0.3 s mean, cm'),
            ('start_drift_p95', 'runtime: start planted drift first 0.3 s p95 (max of runs), cm')]
    for body, lst in sessions.items():
        agg = {}
        for side in builds:
            m = parse_replay(res[(side, body, 'replay', None, 0)][1])
            if not a.no_stages:
                rts = {rid: [parse_runtime(res[(side, body, 'stages', rid, k)][1]) for k in range(a.repeat)] for rid, _, _ in lst}
                firsts = [v[0] for v in rts.values() if v[0]]
                secs = sum(r['secs'] for r in firsts) or 1.0
                w = lambda key: sum(r[key] * r['secs'] for r in firsts) / secs  # noqa: E731
                turns = sum(r['turns'] for r in firsts) or 1.0
                m.update(rt_clip_changes_s=w('clipChangesPerS'), rt_corr_max=max(r['ikCorrMaxCm'] for r in firsts),
                         rt_corr_p95=w('ikCorrP95Cm'),
                         rt_proc_turn=sum(r['procYawPerTurnDeg'] * r['turns'] for r in firsts) / turns,
                         rt_proc_share=w('procYawShare'),
                         rt_feed_p50=statistics.median(r['feedUsP50'] for v in rts.values() for r in v if r),
                         rt_feed_p95=statistics.median(r['feedUsP95'] for v in rts.values() for r in v if r))
                m['rt_history'] = sum(r.get('historySearches', 0) for r in firsts)
                for c in ('stop', 'start'):
                    n = sum(r.get(c + 'Picks', 0) for r in firsts) or 1.0
                    nd = sum(r.get(c + 'DriftN', 0) for r in firsts) or 1.0
                    m[c + '_picks'] = sum(r.get(c + 'Picks', 0) for r in firsts)
                    m[c + '_wrong'] = sum(r.get(c + 'WrongFoot', 0) for r in firsts)
                    m[c + '_off_p50'] = sum(r.get(c + 'OffP50Cm', 0) * r.get(c + 'Picks', 0) for r in firsts) / n
                    m[c + '_drift_mean'] = sum(r.get(c + 'DriftMeanCm', 0) * r.get(c + 'DriftN', 0) for r in firsts) / nd
                    m[c + '_drift_p95'] = max(r.get(c + 'DriftP95Cm', 0) for r in firsts)
                for rid, v in rts.items():
                    for k, r in enumerate(v):
                        if r:
                            print('%-6s %-6s %s #%d clip/s %.3f corrMax %.2f corrP95 %.2f turns %d procYaw/turn %.2f share %.4f '
                                  'feed p50 %.1f p95 %.1f' % (side, body, rid, k, r['clipChangesPerS'], r['ikCorrMaxCm'], r['ikCorrP95Cm'],
                                                              r['turns'], r['procYawPerTurnDeg'], r['procYawShare'], r['feedUsP50'],
                                                              r['feedUsP95']))
            agg[side] = m
        print('\n== %s (%s)' % (body, ', '.join(rid for rid, _, _ in lst)))
        print('%-48s %10s %10s %10s' % ('metric', 'before', 'after', 'delta'))
        for key, label in rows:
            b, f = agg['before'].get(key), agg['after'].get(key)
            if b is None and f is None:
                continue
            fmt = lambda x: '%10.3f' % x if x is not None else '%10s' % '-'  # noqa: E731
            print('%-48s %s %s %s' % (label, fmt(b), fmt(f), fmt(f - b if b is not None and f is not None else None)))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
