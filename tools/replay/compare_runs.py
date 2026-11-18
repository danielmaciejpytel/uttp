"""Before / after comparison of two builds on recorded game sessions, both bodies, one command.

Runs, for every session, the per-frame game feed dump (<night logs>/<run>/logs/utpp-feed-*.txt) through
  - utpp_motion_replay (matcher / evaluator / foot locker lab): one call per body with all its sessions, --footlock --brief;
    read: FLTAB ALL (slide while planted, correction), FLTOT maxCorr, FLPEL pelvis offset, SUMMARY clip changes per second;
  - utpp_replay_pose_stages (the real runtime path: driver, blend stack, root frame, host leg tail) with the session's settings
    log and the recorded yaw ahead, summary only; read: RUNTIME (clip changes, foot correction, procedural yaw per turn, Feed cost).
and prints one table per body (before, after, after - before) plus the per-session runtime lines.

    python compare_runs.py --before-dir <build A> --after-dir <build B> [--runs woman:<run> man:<run> ...]
                           [--after-replay-args "--opt pelvis=1"] [--after-stages-args "..."] [--no-stages] [--repeat N]
                           [--world-slide] [--aa N] [--margin-out M.json] [--margin M.json]

--before-dir / --after-dir: folders that hold utpp_motion_replay.exe and utpp_replay_pose_stages.exe (a build folder or a copy).
--repeat N runs the runtime path N times and reports the median Feed p50 / p95 (cost comparisons need repeats).
--world-slide adds the world-space planted-toe slide per state class (start, stop, pivot, loop, landing, idle, other; p50 / p95 cm/s,
locks, lock path p95 cm) of the runtime path: the final pose of every feed measured with the in-game metric code
(runtime_world_slide.py, footslide/world_slide.py; all feed rows of the sessions). The replay output goes to --work and is deleted.
--aa N runs the before build N more times (same commands, same --jobs) and takes, per metric and body, the spread (max - min) of its
runs as the noise margin (--margin-out writes it); --margin M.json reads a margin written earlier. With a margin the table gets a
margin column and a mark: '~' = |after - before| within the margin (noise), '!' = outside it.
Exit code 1 when a tool fails or a run reports REPLAY_FAILS != 0.
"""
import argparse
import glob
import json
import os
import re
import shlex
import statistics
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))   # the tools folder
from utpp_paths import project_root  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import runtime_world_slide as rws  # noqa: E402

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
    ap.add_argument('--world-slide', action='store_true')
    ap.add_argument('--work', help='folder for the --world-slide replay output (default <root>/Build/work/_tmp/compare_runs)')
    ap.add_argument('--aa', type=int, default=0)
    ap.add_argument('--margin')
    ap.add_argument('--margin-out')
    a = ap.parse_args()
    work = a.work or os.path.join(a.root, 'Build', 'work', '_tmp', 'compare_runs')
    if a.world_slide:
        os.makedirs(work, exist_ok=True)

    night = os.path.join(a.root, 'Research', 'logs', 'night')
    data = os.path.join(a.root, 'Data')
    sessions = {}
    for spec in a.runs:
        body, rid = spec.split(':', 1)
        logs = os.path.join(night, rid, 'logs')
        sessions.setdefault(body, []).append((rid, one(os.path.join(logs, 'utpp-feed-*.txt')), one(os.path.join(logs, 'utpp-2*.log'))))

    builds = {'before': (a.before_dir, a.before_replay_args, a.before_stages_args),
              'after': (a.after_dir, a.after_replay_args, a.after_stages_args)}
    for i in range(a.aa):
        builds['aa%d' % (i + 1)] = builds['before']
    jobs = {}
    ws_jobs = {}
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
                        out = os.path.join(work, '%s_%s_%s' % (side, body, rid)) if a.world_slide and k == 0 else '-'
                        cmd = [os.path.join(d, 'utpp_replay_pose_stages.exe'), data, os.path.join(data, 'utpp_base.utppmm'), feed, out,
                               '--settings-log', log, '--ahead', 'recorded', '--body', body] + shlex.split(sargs, posix=False)
                        jobs[(side, body, 'stages', rid, k)] = pool.submit(run, cmd)
                        if out != '-':
                            ws_jobs[(side, body, rid)] = (jobs[(side, body, 'stages', rid, k)], out, feed, body)
        res = {k: f.result() for k, f in jobs.items()}
    # world slide of the replay output: separate processes (the measurement is numpy heavy), files deleted after reading
    ws_parts = {}
    if ws_jobs:
        with ProcessPoolExecutor(max_workers=max(1, min(a.jobs, len(ws_jobs)))) as procs:
            futs = {k: procs.submit(world_slide_parts, out, feed, body) for k, (_, out, feed, body) in ws_jobs.items()
                    if res[(k[0], k[1], 'stages', k[2], 0)][0] == 0}
            for k, f in futs.items():
                try:
                    ws_parts[k] = f.result()
                except (OSError, ValueError) as e:
                    print('FAIL world slide %s: %s' % (' '.join(k), e))
                    ws_parts[k] = None
        for _, out, _, _ in ws_jobs.values():
            for ext in ('.bin', '.txt'):
                try:
                    os.remove(out + ext)
                except OSError:
                    pass

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
    for cls in rws.ws.CLASSES:
        rows += [('ws_%s_p50' % cls, 'world slide %s p50, cm/s' % cls), ('ws_%s_p95' % cls, 'world slide %s p95, cm/s' % cls),
                 ('ws_%s_locks' % cls, 'world slide %s locks' % cls), ('ws_%s_path_p95' % cls, 'world slide %s lock path p95, cm' % cls)]
    margin = {}
    if a.margin:
        with open(a.margin, encoding='utf-8') as f:
            margin = json.load(f)
    margin_out = {}
    for body, lst in sessions.items():
        agg = {}
        for side in builds:
            m = parse_replay(res[(side, body, 'replay', None, 0)][1])
            if a.world_slide:
                parts = [ws_parts.get((side, body, rid)) for rid, _, _ in lst]
                if all(p is not None for p in parts):
                    tab = rws.class_table(parts)
                    for cls in rws.ws.CLASSES:
                        t = tab[cls]
                        m.update({'ws_%s_p50' % cls: t['p50'], 'ws_%s_p95' % cls: t['p95'], 'ws_%s_locks' % cls: float(t['locks']),
                                  'ws_%s_path_p95' % cls: t['path_p95']})
                else:
                    bad = True
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
        aa_sides = ['before'] + ['aa%d' % (i + 1) for i in range(a.aa)]
        if a.aa:
            mb = margin_out.setdefault(body, {})
            for key, _ in rows:
                v = [agg[s].get(key) for s in aa_sides]
                if all(x is not None for x in v):
                    mb[key] = round(max(v) - min(v), 4)
        mrg = margin_out.get(body, {}) if a.aa else margin.get(body, {})
        print('\n== %s (%s)' % (body, ', '.join(rid for rid, _, _ in lst)))
        print('%-48s %10s %10s %10s %10s' % ('metric', 'before', 'after', 'delta', 'margin' if mrg else ''))
        for key, label in rows:
            b, f = agg['before'].get(key), agg['after'].get(key)
            if b is None and f is None:
                continue
            fmt = lambda x: '%10.3f' % x if x is not None else '%10s' % '-'  # noqa: E731
            d = f - b if b is not None and f is not None else None
            mk = ''
            if key in mrg and d is not None:
                mk = '%10.3f %s' % (mrg[key], '~' if abs(d) <= mrg[key] + 1e-9 else '!')
            print('%-48s %s %s %s %s' % (label, fmt(b), fmt(f), fmt(d), mk))
    if a.margin_out and margin_out:
        with open(a.margin_out, 'w', encoding='utf-8') as fo:
            json.dump(margin_out, fo, indent=1, sort_keys=True)
    return 1 if bad else 0


def world_slide_parts(prefix, feed, body):
    """Pooled world-slide samples of one runtime-path output (all feed rows)."""
    import numpy as np
    ses, _ = rws.offline_session(prefix, feed, body)
    return rws.pool(ses, np.ones(len(ses.fsec), dtype=bool))


if __name__ == '__main__':
    sys.exit(main())
