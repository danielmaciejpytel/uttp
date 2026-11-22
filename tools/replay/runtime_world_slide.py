"""World-space planted-foot slide of the offline runtime replay, measured with the code of the in-game metric.

  python -I runtime_world_slide.py --exe-dir <build> [--runs woman:<run> man:<run> ...] [--stages-args "..."] [--game]
                                   [--json OUT.json] [--md OUT.md] [--work DIR] [--keep-bin] [--jobs N]

For every session the recorded feed dump goes through utpp_replay_pose_stages (with the session's settings log and the recorded yaw
ahead, as tools/replay/compare_runs.py runs it) and the final-stage pose of every feed is read back. The record of feed row r becomes
frame r of a footslide/world_slide.py Session:
  - toe = LeftToeBase / RightToeBase by forward kinematics from the Root bone (Root local included: the visual root offset is part of
    the published pose, as in the recorded game pose);
  - capsule / yaw / feed step = the feed dump row (read as text: float32 world coordinates would add jitter);
  - contact = the leg tail contact of that feed (contactL / contactR): the contact the foot locker received in the replay;
    with --stages-args "... --motion-log 1" instead the contact track world_slide.py rebuilds from the replay's own MOTION trace
    lines (contact_track: the clip / clip time / blend of each line, layers crossfaded), the same reconstruction the game side gets
    from the game log, so both sides measure planted frames with one model (the tail contact is crisp where the reconstruction
    crossfades a loop re-pick). Trace mode measures the contact the in-game metric uses (rebuilt from clip, clip time and blend),
    not what the foot locker received: a variant that changes only the leg tail contact signal (e.g. --signals 2) does not move it,
    so measure such variants in tail mode;
  - clip = the driver's pick of that feed.
The pose of row r is drawn at the capsule of row r + FEED_SHIFT, as in the game metric; the per-shift scores are reported.
Samples, locks, trimming and the per-class pooling are the world_slide.py code.

--game also measures the recorded game session of the same run (world_slide.py: pose recording + feed + log) and reports both on the
common feed rows (the rows the game recording is aligned to, outside its rejected step / scenario windows), plus per-row agreement of
the clip and of the planted state. Without --game only the replay is measured (all rows).
--stages-args "--capsule-stats 1" replays the capsule Acceleration / Deceleration stats the game reports after every feed (the stop
and start forecasts use them in the game; without them the replay picks other stop clips). A replay tool that ignores the option is
refused (no CAPSTAT line).
--stages-args "--air-feed 1" hands each feed's air fields to the driver before the feed, as the game does (landing / idle picks agree
with the game; without it the replay driver never gets an air feed). A replay tool that ignores it is refused (no AIRFEED line), and
--motion-log 1 without a written trace file is refused as well (an older replay tool).
--root <variant root>: the replay reads <root>/Data and the world slide uses the same root's motion database (clip names, contacts).
Read-only on every input; the .bin files of the replay are written to --work and deleted afterwards unless --keep-bin.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
import functools
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (TOOLS, os.path.join(TOOLS, 'footslide'), os.path.join(TOOLS, 'gametest')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import world_slide as ws  # noqa: E402
from utpp_paths import project_root  # noqa: E402

DEFAULT_RUNS = ['woman:20261118-121140', 'woman:20261118-122228', 'woman:20261118-122643',
                'man:20261118-130756', 'man:20261118-131922', 'man:20261118-132319']
BODY_NAME = {'woman': 'female', 'man': 'male'}


# ------------------------------------------------------------------------------------------------------------------------ replay output
def run_stages(exe_dir, root, feed, log, body, prefix, extra=''):
    data = os.path.join(root, 'Data')
    cmd = [os.path.join(exe_dir, 'utpp_replay_pose_stages.exe'), data, os.path.join(data, 'utpp_base.utppmm'), feed, prefix,
           '--settings-log', log, '--ahead', 'recorded', '--body', body] + shlex.split(extra, posix=False)
    p = subprocess.run(cmd, capture_output=True, text=True, errors='replace')
    if p.returncode != 0:
        raise RuntimeError('replay failed rc=%d: %s' % (p.returncode, (p.stdout + p.stderr)[-400:]))
    if capsule_stats_requested(extra) and 'CAPSTAT ' not in p.stdout:
        raise RuntimeError('--capsule-stats 1 requested but %s printed no CAPSTAT line (the tool does not know the option)' % cmd[0])
    if air_feed_requested(extra) and 'AIRFEED ' not in p.stdout:
        raise RuntimeError('--air-feed 1 requested but %s printed no AIRFEED line (the tool does not know the option)' % cmd[0])
    if lean_pack_requested(extra) and not re.search(r'^LEANPACK .*\bloaded=1\b', p.stdout, re.M):
        raise RuntimeError('--lean-pack 1 requested but %s printed no LEANPACK line with loaded=1 (old tool or no lean pack in %s)'
                           % (cmd[0], data))
    return p.stdout


def _flag_on(extra, flag):
    a = shlex.split(extra or '', posix=False)
    return any(x == flag and i + 1 < len(a) and a[i + 1] not in ('0', '') for i, x in enumerate(a))


def air_feed_requested(extra):
    """True when the replay arguments ask for the air feed (--air-feed with a non-zero value)."""
    return _flag_on(extra, '--air-feed')


def lean_pack_requested(extra):
    """True when the replay arguments ask for the motion lean pack (--lean-pack with a non-zero value)."""
    return _flag_on(extra, '--lean-pack')


def motion_log_requested(extra):
    """True when the replay arguments ask for the MOTION trace file (--motion-log with a non-zero value)."""
    a = shlex.split(extra or '', posix=False)
    return any(x == '--motion-log' and i + 1 < len(a) and a[i + 1] not in ('0', '') for i, x in enumerate(a))


def trace_contact_track(prefix, db, fsec):
    """world_slide contact Track rebuilt from <prefix>.motion.log (lines stamped with the feed time as 2000-01-01 wall time)."""
    import datetime
    base = datetime.datetime(2000, 1, 1)
    if not os.path.exists(prefix + '.motion.log'):
        raise ValueError('--motion-log 1 requested but %s.motion.log was not written (the replay tool does not know the option)' % prefix)
    with open(prefix + '.motion.log', encoding='utf-8', errors='replace') as f:
        events = ws.parse_motion_lines(f)
    if not any(e.get('kind', 'motion') == 'motion' for e in events):
        raise ValueError('%s.motion.log: no MOTION lines' % prefix)
    return ws.contact_track(db, events, np.asarray(fsec, dtype=np.float64), lambda w: (w - base).total_seconds())


def capsule_stats_requested(extra):
    """True when the replay arguments ask for the capsule stat replay (--capsule-stats with a non-zero value)."""
    a = shlex.split(extra or '', posix=False)
    return any(x == '--capsule-stats' and i + 1 < len(a) and a[i + 1] not in ('0', '') for i, x in enumerate(a))


def _qrot(q, v):
    """Rotate vectors v [N,3] by quaternions q [N,4] (x y z w)."""
    u, w = q[:, :3], q[:, 3:4]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def _qmul(a, b):
    ax, ay, az, aw = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    bx, by, bz, bw = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    return np.stack([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz], axis=1)


def load_stages(prefix, stage='final'):
    """(meta dict of arrays, toe [2][N,3] model space, heel, clip name of id dict) of a pose-stages output."""
    with open(prefix + '.txt', encoding='utf-8') as f:
        lines = f.read().splitlines()
    nb = int(lines[1].split()[1])
    meta = lines[2].split()[2:]
    names, parent = [], []
    for ln in lines:
        p = ln.split()
        if p and p[0] == 'bone':
            names.append(p[2])
            parent.append(int(p[3]))
    clips = {int(ln.split()[1]): ln.split()[2] for ln in lines if ln.startswith('clip ')}
    rec = len(meta) + nb * 14
    a = np.fromfile(prefix + '.bin', dtype=np.float32).reshape(-1, rec)
    m = {k: a[:, i].astype(np.float64) for i, k in enumerate(meta)}
    base = len(meta) + (nb * 7 if stage == 'final' else 0)
    pose = a[:, base:base + nb * 7].reshape(-1, nb, 7).astype(np.float64)

    def chain(i):
        out = []
        while i >= 0:
            out.append(i)
            i = parent[i]
        return out[::-1]
    out = {}
    for bn in ws.TOE + ws.HEEL:
        if bn not in names:
            continue
        q = np.tile([0.0, 0.0, 0.0, 1.0], (len(a), 1))
        t = np.zeros((len(a), 3))
        for i in chain(names.index(bn)):
            tl, ql = pose[:, i, 0:3], pose[:, i, 3:7]
            t = t + _qrot(q, tl)
            q = _qmul(q, ql)
        out[bn] = t
    toe = [out[ws.TOE[0]], out[ws.TOE[1]]]
    heel = [out[h] for h in ws.HEEL] if all(h in out for h in ws.HEEL) else None
    return m, toe, heel, clips


def db_path_of(root, body):
    """Motion database of a body under a project root (the one the replay read: <root>/Data/utpp_motion_<body>.utppmm)."""
    return os.path.join(root, 'Data', 'utpp_motion_%s.utppmm' % body)


def offline_session(prefix, feed_path, body, fixed_shift=ws.FEED_SHIFT, db_path=None, contact='tail'):
    """world_slide Session of a replay output: frame r = feed row r.

    db_path = the database the replay ran with (clip names / classes and contact curves); default the live project database. Pass
    db_path_of(<root>, body) for a --root variant: its clip indices only name the right clips in that variant's database.
    contact = 'tail' (the leg tail contact of each feed) or 'trace' (rebuilt from <prefix>.motion.log, see the module doc)."""
    import pose_metrics as pm
    m, toe, heel, _ = load_stages(prefix)
    feed = pm.read_feed(feed_path)
    n = len(m['t'])
    if len(feed['t']) != n or np.nanmax(np.abs(feed['t'] - m['t'])) > 2e-3:
        raise ValueError('%s: replay records do not match the feed rows (%d vs %d)' % (prefix, n, len(feed['t'])))
    db = ws.load_db(db_path or ws.default_db_path(body))
    clip = np.nan_to_num(m['clip'], nan=-1).astype(np.int64)
    published = (m['published'] > 0.5) & (m['tailed'] > 0.5)
    state = np.where(published & np.isfinite(m['contactL']) & np.isfinite(m['contactR']), ws.ST_OK, ws.ST_UNKNOWN)
    cont = np.stack([m['contactL'], m['contactR']], axis=1)
    cont_pose = cont_cut = None
    if contact == 'trace':
        tr = trace_contact_track(prefix, db, feed['t'])
        cont, cont_pose, cont_cut = tr.cont.copy(), tr.cont_pose.copy(), tr.cont_cut.copy()
        state = np.where(published, tr.state, ws.ST_UNKNOWN).astype(np.int8)
        for c in (cont_pose, cont_cut):
            c[~published] = np.nan
    elif contact != 'tail':
        raise ValueError('contact must be tail or trace')
    cont[~published] = np.nan
    toe = [np.where(published[:, None], t, np.nan) for t in toe]
    ses = ws.Session(feed['t'], toe, feed, np.arange(n, dtype=np.int64), cont, state, clip, db.names, heel, cont_pose=cont_pose,
                     cont_cut=cont_cut)
    ses.choose_shift(fixed_shift)
    return ses, m


# ------------------------------------------------------------------------------------------------------------------------ game side
def game_session(run_dir):
    """(Session, keep-frame mask) of a recorded game run, as world_slide.analyze_run pools it."""
    import gt_posemeter as gp
    poses, log, feed, windows, steps = ws.run_inputs(run_dir)
    ses, info, rec = ws.build_session(poses, log, feed)
    variants = ses.variants()
    wins, _, _, spans = ws.analyze_windows(ses, rec, windows, variants)
    keep = np.ones(len(ses.fsec), dtype=bool)
    for label, w in wins.items():
        if w['rejected'] and not label.endswith(gp.MACRO_SUFFIX):
            i0, i1 = spans[label]
            keep[i0:i1] = False
    return ses, keep, variants, info


# ------------------------------------------------------------------------------------------------------------------------ pooling
def pool(ses, keep_frame, variant='main', variants=None):
    """Per-class arrays (speed, lock path cm) of one session over the frames in keep_frame."""
    smp, locks = (variants or {}).get(variant) or (ses.samples() if variant == 'main' else ses.variants()[variant])
    k = keep_frame[smp['frame']] if len(smp['frame']) else np.zeros(0, dtype=bool)
    lk = [L for L in locks if keep_frame[L['mid']]]
    return dict(speed=smp['speed'][k], cls=smp['cls'][k], lock_cm=np.array([L['cm'] for L in lk]),
                lock_cls=np.array([L['cls'] for L in lk], dtype=str))


def class_table(parts):
    """{class: stats} of pooled parts (list of pool() dicts)."""
    sp = np.concatenate([p['speed'] for p in parts]) if parts else np.array([])
    cl = np.concatenate([p['cls'] for p in parts]) if parts else np.array([], dtype=str)
    lc = np.concatenate([p['lock_cm'] for p in parts]) if parts else np.array([])
    ll = np.concatenate([p['lock_cls'] for p in parts]) if parts else np.array([], dtype=str)
    out = {}
    for c in ws.CLASSES + ('all',):
        v = sp if c == 'all' else sp[cl == c]
        L = lc if c == 'all' else lc[ll == c]
        out[c] = dict(n=int(len(v)), p50=_pct(v, 50), p95=_pct(v, 95), locks=int(len(L)), path_p50=_pct(L, 50), path_p95=_pct(L, 95))
    return out


def _pct(v, q):
    v = np.asarray(v, dtype=np.float64)
    v = v[np.isfinite(v)]
    return round(float(np.percentile(v, q)), 2) if len(v) else None


_GAITS = ('Walk', 'Run', 'Sprint', 'Crouch')


def _gait(name):
    return next((g for g in _GAITS if '_%s_' % g in name), '-')


def stop_pair_kind(game_clip, replay_clip):
    """Kind of a game -> replay clip disagreement on a stop row: foot (the same stop on the other foot), gait (a stop of another
    gait), direction (a stop of the same gait and foot, other direction), timing (one side not in a stop clip: the stop starts
    earlier or later), other."""
    gs, rs = '_Stop' in game_clip, '_Stop' in replay_clip
    if gs != rs:
        return 'timing'
    if not gs:
        return 'other'
    if _gait(game_clip) != _gait(replay_clip):
        return 'gait'
    swap = game_clip.replace('Lfoot', '#').replace('Rfoot', 'Lfoot').replace('#', 'Rfoot')
    if swap == replay_clip:
        return 'foot'
    gf, rf = 'Lfoot' in game_clip, 'Lfoot' in replay_clip
    return 'foot' if gf != rf else 'direction'


def stop_pairs(gses, gkeep, oses):
    """{kind: rows} and the most frequent game -> replay pairs of the stop rows (game class) whose clips disagree."""
    import collections
    f = np.nonzero(gkeep & (gses.rows >= 0) & (gses.rows < len(oses.fsec)) & (gses.cls == 'stop'))[0]
    r = gses.rows[f]
    names = gses.clip_names
    known = np.isin(gses.state[f], (ws.ST_OK, ws.ST_BLEND)) & (oses.state[r] == ws.ST_OK)
    kinds, pairs = collections.Counter(), collections.Counter()
    for gi, oi in zip(gses.clip_of[f][known], oses.clip_of[r][known]):
        g = names[gi] if 0 <= gi < len(names) else '-'
        o = names[oi] if 0 <= oi < len(names) else '-'
        if g != o:
            kinds[stop_pair_kind(g, o)] += 1
            pairs[(g, o)] += 1
    return dict(rows=int(known.sum()), kinds=dict(kinds), top=[[g, o, n] for (g, o), n in pairs.most_common(5)])


def agreement(gses, gkeep, oses):
    """Per-row agreement on the game frames kept: same clip, same planted state (contact >= 0.5) per foot, by game class."""
    f = np.nonzero(gkeep & (gses.rows >= 0) & (gses.rows < len(oses.fsec)))[0]
    r = gses.rows[f]
    names = gses.clip_names
    gclip = np.array([names[c] if 0 <= c < len(names) else '-' for c in gses.clip_of[f]])
    oclip = np.array([names[c] if 0 <= c < len(names) else '-' for c in oses.clip_of[r]])
    known = np.isin(gses.state[f], (ws.ST_OK, ws.ST_BLEND)) & (oses.state[r] == ws.ST_OK)
    out = {}
    for c in ws.CLASSES + ('all',):
        m = known & ((gses.cls[f] == c) if c != 'all' else True)
        if not m.any():
            continue
        same_clip = float(np.mean(gclip[m] == oclip[m]))
        gp = np.nan_to_num(gses.cont[f][m], nan=0.0) >= ws.CONTACT_MIN
        op = np.nan_to_num(oses.cont[r][m], nan=0.0) >= ws.CONTACT_MIN
        out[c] = dict(rows=int(m.sum()), same_clip=round(same_clip, 3), same_planted=round(float(np.mean(gp == op)), 3),
                      game_planted=round(float(np.mean(gp)), 3), replay_planted=round(float(np.mean(op)), 3))
    return out


# ------------------------------------------------------------------------------------------------------------------------ driver
def measure(spec, a):
    body, rid = spec.split(':', 1)
    night = os.path.join(a.root, 'Research', 'logs', 'night')
    run_dir = os.path.join(night, rid)
    logs = os.path.join(run_dir, 'logs')
    feed = sorted(f for f in os.listdir(logs) if f.startswith('utpp-feed-') and f.endswith('.txt'))[-1]
    log = sorted(f for f in os.listdir(logs) if f.startswith('utpp-2') and f.endswith('.log'))[-1]
    feed, log = os.path.join(logs, feed), os.path.join(logs, log)
    prefix = os.path.join(a.work, '%s_%s' % (body, rid))
    os.makedirs(a.work, exist_ok=True)
    stdout = run_stages(a.exe_dir, a.root, feed, log, body, prefix, a.stages_args)
    try:
        oses, _ = offline_session(prefix, feed, body, db_path=db_path_of(a.root, body),
                                  contact='trace' if motion_log_requested(a.stages_args) else 'tail')
    finally:
        if not a.keep_bin:
            for ext in ('.bin', '.txt', '.motion.log'):
                try:
                    os.remove(prefix + ext)
                except OSError:
                    pass
    res = dict(run=rid, body=BODY_NAME.get(body, body), shift_scores=oses.shift_scores, shift_flag=oses.shift_flag,
               runtime=[ln for ln in stdout.splitlines() if ln.startswith(('RUNTIME', 'PICKS', 'CAPSTAT'))])
    allrows = np.ones(len(oses.fsec), dtype=bool)
    parts = dict(replay_all=pool(oses, allrows))
    if a.game:
        gses, gkeep, gvar, _ = game_session(run_dir)
        common = np.zeros(len(oses.fsec), dtype=bool)
        ok = gkeep & (gses.rows >= 0) & (gses.rows < len(oses.fsec))
        common[gses.rows[ok]] = True
        gframe = ok.copy()
        parts.update(game=pool(gses, gframe, variants=gvar), replay_common=pool(oses, common))
        # the replay pose with the game's contact track: separates the contact reconstruction from the pose
        mixed = ws.Session(oses.fsec, oses.toe, oses.feed, oses.rows, _game_cont_on_rows(gses, len(oses.fsec)),
                           _game_state_on_rows(gses, len(oses.fsec)), _game_clip_on_rows(gses, len(oses.fsec)), oses.clip_names)
        mixed.choose_shift(ws.FEED_SHIFT)
        parts['replay_pose_game_contact'] = pool(mixed, common)
        res['agreement'] = agreement(gses, gframe, oses)
        res['stop_pairs'] = stop_pairs(gses, gframe, oses)
    return res, parts


def _rows_of(gses, n, values, fill):
    out = np.full((n,) + values.shape[1:], fill, dtype=values.dtype)
    ok = (gses.rows >= 0) & (gses.rows < n)
    out[gses.rows[ok]] = values[ok]
    return out


def _game_cont_on_rows(gses, n):
    return _rows_of(gses, n, np.asarray(gses.cont, dtype=np.float64), np.nan)


def _game_state_on_rows(gses, n):
    return _rows_of(gses, n, np.asarray(gses.state), ws.ST_UNKNOWN)


def _game_clip_on_rows(gses, n):
    return _rows_of(gses, n, np.asarray(gses.clip_of, dtype=np.int64), -1)


def summarize(results):
    """{body: {part: class table}} pooled over the sessions of each body."""
    by = {}
    for res, parts in results:
        for name, p in parts.items():
            by.setdefault(res['body'], {}).setdefault(name, []).append(p)
    return {b: {n: class_table(v) for n, v in d.items()} for b, d in by.items()}


def write_md(doc, path):
    lines = ['# World slide: offline runtime replay against the game', '',
             'Speeds cm/s (p50 / p95 of planted toe speed, runtime contact gate, 3-frame trim), lock path cm p50 / p95, locks.', '']
    for body, parts in doc['summary'].items():
        lines += ['## %s' % body, '', '| class | ' + ' | '.join(parts) + ' |', '|---|' + '---|' * len(parts)]
        for c in ws.CLASSES + ('all',):
            cells = []
            for name in parts:
                s = parts[name][c]
                cells.append('%s / %s; %d locks, path %s / %s' % (s['p50'], s['p95'], s['locks'], s['path_p50'], s['path_p95']))
            lines.append('| %s | %s |' % (c, ' | '.join(cells)))
        lines.append('')
    if any('agreement' in r for r in doc['runs']):
        lines += ['## Per-row agreement (game class; same clip share, same planted-state share, planted share game / replay)', '']
        for r in doc['runs']:
            ag = r.get('agreement') or {}
            lines.append('- %s %s: ' % (r['body'], r['run']) + '; '.join(
                '%s %d rows clip %.2f planted %.2f (%.2f / %.2f)' % (c, v['rows'], v['same_clip'], v['same_planted'], v['game_planted'],
                                                                    v['replay_planted']) for c, v in ag.items()))
        lines.append('')
        lines += ['## Stop rows: game -> replay clip disagreements by kind (foot / gait / direction / timing / other)', '']
        for r in doc['runs']:
            sp = r.get('stop_pairs') or {}
            lines.append('- %s %s: %s of %s rows; top %s' % (r['body'], r['run'], sp.get('kinds'), sp.get('rows'), sp.get('top')))
        lines.append('')
    lines += ['## Shift scores of the replay (mean planted cm/s of moving clips, samples)', '']
    for r in doc['runs']:
        lines.append('- %s %s: %s %s' % (r['body'], r['run'], r['shift_scores'], r['shift_flag'] or ''))
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--exe-dir', required=True)
    ap.add_argument('--runs', nargs='+', default=DEFAULT_RUNS)
    ap.add_argument('--root', default=project_root())
    ap.add_argument('--stages-args', default='')
    ap.add_argument('--game', action='store_true')
    ap.add_argument('--work', default=None, help='folder for the replay output (default <root>/Build/work/_tmp/world_slide)')
    ap.add_argument('--keep-bin', action='store_true')
    ap.add_argument('--jobs', type=int, default=3)
    ap.add_argument('--json')
    ap.add_argument('--md')
    a = ap.parse_args(argv)
    a.work = a.work or os.path.join(a.root, 'Build', 'work', '_tmp', 'world_slide')
    with ProcessPoolExecutor(max_workers=a.jobs) as ex:  # the capture decompressor is not thread safe
        results = list(ex.map(functools.partial(measure, a=a), a.runs))
    doc = dict(runs=[r for r, _ in results], summary=summarize(results))
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1)
    if a.md:
        write_md(doc, a.md)
    for body, parts in doc['summary'].items():
        print('==', body)
        for c in ws.CLASSES + ('all',):
            print('%-8s ' % c + '  '.join('%s %s/%s L%d' % (n[:14], parts[n][c]['p50'], parts[n][c]['p95'], parts[n][c]['locks'])
                                         for n in parts))
    return 0


if __name__ == '__main__':
    sys.exit(main())
