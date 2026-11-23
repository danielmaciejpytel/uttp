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
--before-stages-args / --after-stages-args "--capsule-stats 1" replay the capsule Acceleration / Deceleration stats the game reports
after every feed (stop and start forecasts as in the game); a build that does not print the CAPSTAT line for it counts as a failure
(an older replay tool ignores the option), so both sides must be built with a replay tool that knows it.
Other replay options (utpp_replay_pose_stages header): "--air-feed 1" feeds the air fields as the game does; "--motion-log 1" makes the
--world-slide contact track the reconstruction from the replay's MOTION trace (the game side's model) instead of the tail contact.
--root <variant root>: replay and world slide read <root>/Data (motion databases included).
With --world-slide every runtime-path output is also read per feed row (session_windows) and one WINDOW line per session is printed:
  stepMax = the most driver clip changes per second over the harness steps the in-game gate scored in that session
  (<run>/gametest/summary.json, PASS / FAIL steps), each step window put on the feed clock through the MOTION trace (needs
  --motion-log 1: the replay trace and the game log share the driver time t=); the same windows and ratio as the in-game limit
  (the session mean hides a short burst such as a fast running arc; table row 'clip changes / s, busiest scored step', stepGame =
  the in-game value of that step, 'sessions without scored steps' counts sessions it could not measure);
  clipWinMax = the most driver clip changes inside any --clip-window (default 3 s) of feed time / the window, anywhere in the session
  (a report: it also counts the time between steps and unscored steps; table row 'clip changes / s, busiest window');
  corrMax = the largest horizontal foot lock correction (cm) with its time, class and clip; corrVMax = the largest vertical (ground)
  correction when the output carries one ('-' = not measured: the build has no ground stage, the table row reads 0 and 'vertical foot
  correction measured' 0; a ground stage reports it as the RUNTIME key ikCorrVMaxCm and / or the meta fields corrVL_cm / corrVR_cm);
  budgetTrips = cost budget trip lines in the --motion-log 1 trace (a trip pauses the motion pose: the pose rows then depend on the
  machine load, not only on the code; table row 'cost budget trips').
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
# Window of the busiest-window clip change measure (s of feed time; the in-game steps that the 3.0 / s limit judges last about 3 s)
CLIP_WINDOW_S = 3.0
CLOCK_TOLERANCE_S = 0.05   # spread allowed between the matched trace lines that place the harness steps on the feed clock
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
    ap.add_argument('--clip-window', type=float, default=CLIP_WINDOW_S,
                    help='window (s of feed time) of the busiest-window clip change measure (--world-slide only)')
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
            logs_of = {rid: log for lst in sessions.values() for rid, _, log in lst}
            futs = {k: procs.submit(world_slide_parts, out, feed, body, rws.db_path_of(a.root, body),
                                    'trace' if rws.motion_log_requested(builds[k[0]][2]) else 'tail', a.clip_window,
                                    os.path.join(night, k[2]), logs_of[k[2]])
                    for k, (_, out, feed, body) in ws_jobs.items()
                    if res[(k[0], k[1], 'stages', k[2], 0)][0] == 0}
            for k, f in futs.items():
                try:
                    ws_parts[k] = f.result()
                except (OSError, ValueError) as e:
                    print('FAIL world slide %s: %s' % (' '.join(k), e))
                    ws_parts[k] = None
        for _, out, _, _ in ws_jobs.values():
            for ext in ('.bin', '.txt', '.motion.log'):
                try:
                    os.remove(out + ext)
                except OSError:
                    pass

    bad = False
    for k, (rc, text) in res.items():
        stats_ignored = k[2] == 'stages' and rc == 0 and rws.capsule_stats_requested(builds[k[0]][2]) and 'CAPSTAT ' not in text
        if stats_ignored:
            text += '\n--capsule-stats 1 requested but the %s replay tool printed no CAPSTAT line (it does not know the option)' % k[0]
        air_ignored = k[2] == 'stages' and rc == 0 and rws.air_feed_requested(builds[k[0]][2]) and 'AIRFEED ' not in text
        if air_ignored:
            text += '\n--air-feed 1 requested but the %s replay tool printed no AIRFEED line (it does not know the option)' % k[0]
            stats_ignored = True
        if rc != 0 or stats_ignored or (k[2] == 'replay' and parse_replay(text).get('fails', 0) != 0):
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
            ('rt_clip_step_max', 'runtime: clip changes / s, busiest scored step'),
            ('rt_clip_step_missing', 'runtime: sessions without scored steps'),
            ('rt_clip_win_max', 'runtime: clip changes / s, busiest window'),
            ('rt_corr_max', 'runtime: foot correction max, cm'),
            ('rt_corr_v_max', 'runtime: foot correction vertical max, cm'),
            ('rt_corr_v_measured', 'runtime: vertical foot correction measured'),
            ('rt_budget_trips', 'runtime: cost budget trips'),
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
                    wins = [(rid, p['win']) for (rid, _, _), p in zip(lst, parts)]
                    for rid, w in wins:
                        print(window_line(side, body, rid, w))
                    top = max(wins, key=lambda x: x[1]['clip_rate'])[1]
                    m['rt_clip_win_max'] = top['clip_rate']
                    sr = [w['step_rate'] for _, w in wins if w.get('step_rate') is not None]
                    if sr:
                        m['rt_clip_step_max'] = max(sr)
                    m['rt_clip_step_missing'] = float(sum(1 for _, w in wins if w.get('step_rate') is None))
                    trips = [w['budget_trips'] for _, w in wins if w['budget_trips'] is not None]
                    if trips:
                        m['rt_budget_trips'] = float(sum(trips))
                    vs = [w['corr_v_max'] for _, w in wins if w['corr_v_max'] is not None]
                    if vs:
                        m['rt_corr_v_max'] = max(vs)
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
                # vertical (ground) foot correction: RUNTIME key ikCorrVMaxCm of a build with a ground stage; without it the
                # value reads 0 and rt_corr_v_measured = 0 (not measured)
                vrt = [r['ikCorrVMaxCm'] for r in firsts if 'ikCorrVMaxCm' in r]
                if vrt:
                    m['rt_corr_v_max'] = max(vrt + ([m['rt_corr_v_max']] if 'rt_corr_v_max' in m else []))
                m['rt_corr_v_measured'] = 1.0 if 'rt_corr_v_max' in m else 0.0
                m.setdefault('rt_corr_v_max', 0.0)
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


def world_slide_parts(prefix, feed, body, db_path=None, contact='tail', window_s=CLIP_WINDOW_S, run_dir=None, game_log=None):
    """Pooled world-slide samples of one runtime-path output (all feed rows); db_path = the database the replay read; contact = tail /
    trace (runtime_world_slide.offline_session). Key 'win' = session_windows() of the same output (clip changes per scored harness
    step and in the busiest window, foot correction maxima with their place, budget trips of the trace file); run_dir / game_log =
    the recorded session (its harness steps and the game log that places them on the feed clock)."""
    import numpy as np
    ses, m = rws.offline_session(prefix, feed, body, db_path=db_path, contact=contact)
    parts = rws.pool(ses, np.ones(len(ses.fsec), dtype=bool))
    steps = harness_steps(run_dir) if run_dir else None
    parts['win'] = session_windows(m, ses.cls, ses.clip_names, window_s, prefix + '.motion.log', steps, game_log)
    return parts


def harness_steps(run_dir):
    """Scored steps of a recorded harness session: [dict(label, t0, t1 (wall datetimes), game_rate)] from gametest/summary.json, the
    steps the in-game gate judged (verdict PASS / FAIL; SKIP steps and the runner's probe walks are left out); game_rate = the
    in-game motion_transitions_per_s of the step. None when the session has no harness summary."""
    import datetime
    p = os.path.join(run_dir, 'gametest', 'summary.json')
    if not os.path.isfile(p):
        return None
    with open(p, encoding='utf-8') as f:
        summ = json.load(f)
    out = []
    for st in summ.get('steps') or []:
        if st.get('verdict') not in ('PASS', 'FAIL') or not st.get('t0') or not st.get('t1'):
            continue
        t0, t1 = (datetime.datetime.fromisoformat(st[k]) for k in ('t0', 't1'))
        if t1 <= t0:
            continue
        out.append(dict(label='%s#%s/%s' % (st.get('scenario'), st.get('run', 1), st.get('step')), t0=t0, t1=t1,
                        game_rate=(st.get('metrics') or {}).get('motion_transitions_per_s')))
    return out


def feed_clock_origin(replay_motion_log, game_log):
    """(wall datetime of feed time 0, matched lines, spread s) from the MOTION lines both logs share: the replay driver runs the
    recorded feeds, so its trace carries the same driver time t= as the game's, stamped with the feed time (2000-01-01 wall time)
    instead of the wall time. When the game driver started at another feed row (its t= then differs from the replay by a
    constant and fewer than 3 lines pair up within CLOCK_TOLERANCE_S), the pick changes (' chg') of the same clip in both traces
    vote on the offset in CLOCK_TOLERANCE_S bins; the winning bin needs at least 3 votes and 30 % of the shorter change list.
    A driver time offset can also leave a few lines whose t= values coincide by chance (2 Hz lines of both traces on the same
    driver times): they agree with each other but sit at the wrong origin. So when both ways give an origin and they differ by more
    than CLOCK_TOLERANCE_S, the vote wins. None when neither way agrees."""
    import datetime
    import statistics as st
    rep = [e for e in rws.ws.read_motion_log(replay_motion_log) if e['kind'] == 'motion']
    game = [e for e in rws.ws.read_motion_log(game_log) if e['kind'] == 'motion']
    day, epoch = datetime.datetime(2000, 1, 1), datetime.datetime(1970, 1, 1)
    feed_of = {}
    for e in rep:
        feed_of.setdefault(e['t'], (e['wall'] - day).total_seconds())
    offs, seen = [], set()
    for e in game:
        if e['t'] in feed_of and e['t'] not in seen:
            seen.add(e['t'])
            offs.append((e['wall'] - epoch).total_seconds() - feed_of[e['t']])
    direct = None
    if len(offs) >= 3 and max(offs) - min(offs) <= CLOCK_TOLERANCE_S:
        direct = (st.median(offs), len(offs), max(offs) - min(offs))
    # the driver started at another feed in the game (its t= differs by a constant): vote on the offsets of the pick changes of the
    # same clip in both traces; the true offset collects the most votes
    vote = _change_vote(rep, game, day, epoch)
    pick = vote if vote and (not direct or abs(direct[0] - vote[0]) > CLOCK_TOLERANCE_S) else direct
    if not pick:
        return None
    return epoch + datetime.timedelta(seconds=pick[0]), pick[1], pick[2]


def _change_vote(rep, game, day, epoch):
    """(offset s, votes, spread s) of the same-clip pick change vote of feed_clock_origin, None without a clear winner."""
    import statistics as st
    rc = [(e['clip_idx'], (e['wall'] - day).total_seconds()) for e in rep if e['chg']]
    gc = [(e['clip_idx'], (e['wall'] - epoch).total_seconds()) for e in game if e['chg']]
    votes = {}
    for ci, gw in gc:
        for cj, fr in rc:
            if ci == cj:
                b = round((gw - fr) / CLOCK_TOLERANCE_S)
                votes.setdefault(b, []).append(gw - fr)
    if not votes:
        return None
    best = max(votes, key=lambda b: len(votes[b]) + len(votes.get(b - 1, [])) + len(votes.get(b + 1, [])))
    near = votes[best] + votes.get(best - 1, []) + votes.get(best + 1, [])
    if len(near) < max(3, 0.3 * min(len(rc), len(gc))):
        return None
    return st.median(near), len(near), max(near) - min(near)


def step_rates(t, rows, steps, origin):
    """[(rate, changes, duration s, label, game rate)] of the scored steps: clip changes of the replay (rows = clip_change_rows) inside
    each step window put on the feed clock / the step duration, as the in-game gate counts a step."""
    import numpy as np
    tc = np.asarray(t, dtype=np.float64)[rows] if rows else np.zeros(0)
    out = []
    for s in steps:
        s0, s1 = (s['t0'] - origin).total_seconds(), (s['t1'] - origin).total_seconds()
        n = int(np.count_nonzero((tc >= s0) & (tc < s1)))
        out.append((n / (s1 - s0), n, s1 - s0, s['label'], s['game_rate']))
    return out


# ---------------------------------------------------------------- per-window measures of one runtime-path output
def clip_change_rows(clip):
    """Row indices where the driver pick clip changes, counted as the RUNTIME line counts them: a row without a pick (clip < 0)
    neither counts nor resets the last pick (A, -, A is no change)."""
    out, last = [], -1
    for i, c in enumerate(clip):
        c = int(c)
        if c < 0:
            continue
        if last >= 0 and c != last:
            out.append(i)
        last = c
    return out


def busiest_window(t, rows, window_s):
    """(changes, first row, last row) of the window of window_s seconds of feed time [t0, t0 + window_s) that holds the most clip
    changes (rows = clip_change_rows); (0, None, None) without changes."""
    best = (0, None, None)
    j = 0
    for i in range(len(rows)):
        while t[rows[i]] - t[rows[j]] >= window_s:
            j += 1
        if i - j + 1 > best[0]:
            best = (i - j + 1, rows[j], rows[i])
    return best


def _name(names, c):
    c = int(c)
    return names[c] if 0 <= c < len(names) else '-'


def session_windows(m, cls, names, window_s=None, motion_log=None, steps=None, game_log=None):
    """Window measures of one runtime-path output (m = meta arrays of runtime_world_slide.load_stages, cls = class per feed row,
    names = clip names of the database):
      step_rate / step_label / step_n / step_dur / step_game = the most clip changes per second over the scored harness steps of the
                  session (harness_steps; the same windows and ratio as the in-game limit; the session mean hides a short burst
                  such as a fast running arc), placed on the feed clock with feed_clock_origin (needs the --motion-log 1 trace and
                  the game log); step_game = the in-game value of that step. None without steps or without a clock match;
      clip_rate = most clip changes inside any window of window_s s of feed time / window_s, anywhere in the session (between steps
                  and in unscored steps too: a report, it runs well above the in-game step values);
      corr_max / corr_t / corr_cls / corr_clip = the largest horizontal foot lock correction (cm, either foot) and where it is;
      corr_v_max (cm) = the largest vertical (ground) correction when the output carries one (meta corrVL_cm / corrVR_cm), else None;
      budget_trips = cost budget trip lines in the trace file (--motion-log 1; None without the file)."""
    import numpy as np
    window_s = window_s or CLIP_WINDOW_S
    t = np.asarray(m['t'], dtype=np.float64)
    clip = np.nan_to_num(m['clip'], nan=-1).astype(np.int64)
    rows = clip_change_rows(clip)
    n, r0, r1 = busiest_window(t, rows, window_s)
    out = dict(window_s=window_s, changes=len(rows), secs=float(t[-1] - t[0]) if len(t) > 1 else 0.0, clip_rate=n / window_s,
               clip_n=n, clip_t0=None, clip_t1=None, clip_cls='-', clip_names=[])
    if n:
        sel = [r for r in rows if r0 <= r <= r1]
        k = [str(cls[r]) for r in sel]
        prev = next((int(c) for c in clip[r0 - 1::-1] if c >= 0), -1) if r0 > 0 else -1   # the pick the window starts from
        out.update(clip_t0=float(t[r0]), clip_t1=float(t[r1]), clip_cls=max(set(k), key=k.count),
                   clip_names=[_name(names, prev)] + [_name(names, clip[r]) for r in sel])
    corr = np.fmax(np.nan_to_num(m.get('corrL_cm', np.full(len(t), np.nan)), nan=0.0),
                   np.nan_to_num(m.get('corrR_cm', np.full(len(t), np.nan)), nan=0.0))
    i = int(np.argmax(corr)) if len(corr) else 0
    out.update(corr_max=float(corr[i]) if len(corr) else 0.0, corr_t=float(t[i]) if len(t) else None,
               corr_cls=str(cls[i]) if len(t) else '-', corr_clip=_name(names, clip[i]) if len(t) else '-')
    if 'corrVL_cm' in m or 'corrVR_cm' in m:
        v = np.fmax(np.nan_to_num(np.abs(m.get('corrVL_cm', np.zeros(len(t)))), nan=0.0),
                    np.nan_to_num(np.abs(m.get('corrVR_cm', np.zeros(len(t)))), nan=0.0))
        j = int(np.argmax(v))
        out.update(corr_v_max=float(v[j]), corr_v_t=float(t[j]), corr_v_cls=str(cls[j]))
    else:
        out['corr_v_max'] = None
    out['budget_trips'] = None
    if motion_log and os.path.exists(motion_log):
        with open(motion_log, encoding='utf-8', errors='replace') as f:
            out['budget_trips'] = sum(1 for ln in f if 'budget trip' in ln)
    out.update(step_rate=None, step_n=None, step_dur=None, step_label='-', step_game=None, steps=0, clock_lines=0, clock_spread=None)
    if steps and motion_log and game_log and os.path.exists(motion_log):
        org = feed_clock_origin(motion_log, game_log)
        if org:
            origin, out['clock_lines'], out['clock_spread'] = org
            rates = step_rates(t, rows, steps, origin)
            if rates:
                r = max(rates, key=lambda x: x[0])
                out.update(step_rate=r[0], step_n=r[1], step_dur=r[2], step_label=r[3], step_game=r[4], steps=len(rates))
    return out


def window_line(side, body, rid, w):
    """One WINDOW line of the per-session window measures (gate_eval.py reads it back)."""
    clips = '>'.join(w['clip_names'][:6]) or '-'
    st = w.get('step_rate')
    return ('WINDOW %s %s %s stepMax=%s stepN=%s stepDur=%s stepLabel=%s stepGame=%s steps=%d clockLines=%d clockSpread=%s '
            'window=%.1f changes=%d secs=%.1f clipWinMax=%.3f clipWinN=%d clipWinAt=%s clipWinCls=%s clipWinClips=%s '
            'corrMax=%.2f corrAt=%s corrCls=%s corrClip=%s corrVMax=%s budgetTrips=%s'
            % (side, body, rid, '%.3f' % st if st is not None else '-', w.get('step_n') if st is not None else '-',
               '%.2f' % w['step_dur'] if st is not None else '-', w.get('step_label', '-'),
               '%.3f' % w['step_game'] if w.get('step_game') is not None else '-', w.get('steps', 0), w.get('clock_lines', 0),
               '%.3f' % w['clock_spread'] if w.get('clock_spread') is not None else '-',
               w['window_s'], w['changes'], w['secs'], w['clip_rate'], w['clip_n'],
               '%.2f-%.2fs' % (w['clip_t0'], w['clip_t1']) if w['clip_t0'] is not None else '-', w['clip_cls'], clips,
               w['corr_max'], '%.2fs' % w['corr_t'] if w['corr_t'] is not None else '-', w['corr_cls'], w['corr_clip'],
               '%.2f' % w['corr_v_max'] if w['corr_v_max'] is not None else '-',
               w['budget_trips'] if w['budget_trips'] is not None else '-'))


if __name__ == '__main__':
    sys.exit(main())
