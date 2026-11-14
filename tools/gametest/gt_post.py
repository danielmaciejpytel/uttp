"""Post-processing of a night run after the game is closed: contact sheets and failure clips in a pool of processes.

  python gt_post.py sheets <run folder> [--only NAME] [--workers N]     build the contact sheets of a run (the on-demand form
                                                                         after a --fast run); the recording may already be in Media
  python gt_post.py cores                                               idle cores and the pool size this machine would get

The pool size follows the cores that are idle when the work starts (idle_cores / pool_size), never more than MAX_POOL; the
workers run at below-normal priority so a pose meter or a game that is still closing keeps its time. One job = one process:
  sheet <annotated_main args>      video_sheets.annotated_main in its own interpreter (the log is parsed once per job)
  clip <spec.json>                 one failure clip (failure_clips.cut)
The long "full" sheet is cut into parts of one sheet each (24 frames), so the biggest job does not decide the end time; the
parts are merged into one folder again (sheets\\full\\sheet_NN.png, index.txt) with the frame and sheet numbers of the
uncut sheet (video_sheets --frame-offset).
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_load  # noqa: E402
import gt_night as gn  # noqa: E402

MAX_POOL = 24                      # project rule: heavy scripts use up to about 24 threads / processes
RESERVE_CORES = 1                  # left to the other post-processing (pose meter workers) and to the system
MIN_POOL = 2                       # a loaded machine still gets some parallelism
BELOW_NORMAL = 0x00004000          # BELOW_NORMAL_PRIORITY_CLASS (CreateProcess flag)
PART_FRAMES = 24                   # frames of one part of the full sheet = one sheet of 6 x 4
FULL_EVERY_S = 0.5
JOB_TIMEOUT_S = 1800.0


# ---------------------------------------------------------------------------------------------- pool size
def idle_cores(sample_s: float = 0.5, probe=None) -> float:
    """Cores that are idle right now (cores x (1 - CPU load)): psutil over `sample_s`, else typeperf, else half of them."""
    cores = os.cpu_count() or 4
    pct = None
    if probe is not None:
        pct = probe()
    else:
        try:
            import psutil
            pct = float(psutil.cpu_percent(interval=sample_s))
        except Exception:  # noqa: BLE001
            pct = gt_load.cpu_percent_typeperf()
    if pct is None:
        pct = 50.0
    return max(0.0, cores * (1.0 - pct / 100.0))


def pool_size(njobs: int, idle: float | None = None, reserve: int = RESERVE_CORES, cap: int = MAX_POOL,
              floor: int = MIN_POOL) -> int:
    """Workers for `njobs` jobs: the idle cores minus `reserve`, at least `floor`, at most `cap`, never more than the jobs."""
    if njobs <= 0:
        return 1
    idle = idle_cores() if idle is None else idle
    return max(1, min(njobs, cap, max(floor, int(idle) - reserve)))


def child_command(kind: str, *args: str) -> list[str]:
    return [sys.executable, '-I', os.path.abspath(__file__), kind, *args]


def run_pool(jobs, workers: int, runner=subprocess.run, timeout: float = JOB_TIMEOUT_S) -> dict:
    """Runs the jobs [(name, argv)] `workers` at a time, each in its own process (below-normal priority). Returns {name:
    {'ok', 'rc', 'seconds', 'error'}} in the order of the jobs once every job has ended; a job that cannot start or runs
    out of time is a failed job, never an exception."""
    jobs = list(jobs)

    def one(job):
        name, argv = job
        t0 = time.monotonic()
        try:
            p = runner(argv, capture_output=True, text=True, errors='replace', timeout=timeout,
                       creationflags=BELOW_NORMAL if os.name == 'nt' else 0)
            ok = p.returncode == 0
            err = '' if ok else ((p.stderr or '').strip().splitlines() or [f'rc {p.returncode}'])[-1][:300]
            return name, {'ok': ok, 'rc': p.returncode, 'seconds': round(time.monotonic() - t0, 2), 'error': err}
        except Exception as e:  # noqa: BLE001
            return name, {'ok': False, 'rc': None, 'seconds': round(time.monotonic() - t0, 2), 'error': repr(e)[:300]}
    if not jobs:
        return {}
    workers = max(1, min(int(workers), len(jobs)))
    if workers == 1:
        done = [one(j) for j in jobs]
    else:
        with ThreadPoolExecutor(max_workers=workers) as ex:
            done = list(ex.map(one, jobs))
    return dict(done)


# ---------------------------------------------------------------------------------------------- contact sheets
def plan_sheets(video: str, log: str, video_end: str, clock_offset: float, ffmpeg: str, video_start: datetime,
                duration: float, scenarios: list[dict], pad: float, sheet_dir: str, keep_frames: bool = False,
                only: str = '', part_frames: int = PART_FRAMES, phases: list[dict] | None = None) -> list[dict]:
    """Jobs of the contact sheets: [{'sheet' (logical name), 'part' (index or None), 'out' (folder), 'args'}]. The "full"
    sheet (a frame every FULL_EVERY_S s, 6 x 4 tiles) is cut into parts of `part_frames` frames; every scenario start and
    end gets its own sheet (gt_night.sheet_windows, 6 x 5 tiles, a frame every 0.1 s). `phases` ([{'name', 'start', 'end'}
    datetimes, gt_night.phase_windows_of]) adds one sheet per labelled phase (6 x 5 tiles over the whole phase). `only` keeps one
    logical sheet."""
    base = ['--video', video, '--log', log, '--video-end', video_end, '--clock-offset', str(clock_offset), '--ffmpeg', ffmpeg]
    if not keep_frames:
        base.append('--no-keep-sheet-frames')
    jobs: list[dict] = []
    end = max(0.0, duration - 0.05)
    n = int((end + 1e-9) / FULL_EVERY_S) + 1
    for j in range((n + part_frames - 1) // part_frames):
        first = j * part_frames
        count = min(part_frames, n - first)
        start = first * FULL_EVERY_S
        jobs.append({'sheet': 'full', 'part': j, 'out': os.path.join(sheet_dir, f'full.part{j + 1:03d}'),
                     'args': base + ['--every', str(FULL_EVERY_S), '--start', str(start),
                                     '--end', str(start + (count - 1) * FULL_EVERY_S), '--cols', '6', '--rows', '4',
                                     '--width', '320', '--frame-offset', str(first),
                                     '--out', os.path.join(sheet_dir, f'full.part{j + 1:03d}')]})
    for w in gn.sheet_windows(video_start, duration, scenarios, pad):
        jobs.append({'sheet': w['name'], 'part': None, 'out': os.path.join(sheet_dir, w['name']),
                     'args': base + ['--every', '0.1', '--start', str(w['start']), '--end', str(w['end']), '--cols', '6',
                                     '--rows', '5', '--width', '320', '--out', os.path.join(sheet_dir, w['name'])]})
    for w in gn.phase_sheet_windows(video_start, duration, phases or []):
        jobs.append({'sheet': w['name'], 'part': None, 'out': os.path.join(sheet_dir, w['name']),
                     'args': base + ['--every', str(w['every']), '--start', str(w['start']), '--end', str(w['end']), '--cols', '6',
                                     '--rows', '5', '--width', '320', '--out', os.path.join(sheet_dir, w['name'])]})
    if only:
        jobs = [j for j in jobs if j['sheet'] == only]
    return jobs


def merge_parts(sheet_dir: str, name: str = 'full') -> list[str]:
    """Moves the sheets of <name>.partNNN folders into <sheet_dir>\\<name> and joins their index.txt (the header block of the
    first part, then the frame entries of every part in order); the part folders are removed. Returns the problems."""
    problems: list[str] = []
    parts = sorted(d for d in os.listdir(sheet_dir) if d.startswith(f'{name}.part')
                   and os.path.isdir(os.path.join(sheet_dir, d))) if os.path.isdir(sheet_dir) else []
    if not parts:
        return problems
    dest = os.path.join(sheet_dir, name)
    os.makedirs(dest, exist_ok=True)
    index: list[str] = []
    for k, d in enumerate(parts):
        src = os.path.join(sheet_dir, d)
        for fn in sorted(os.listdir(src)):
            p = os.path.join(src, fn)
            if fn.startswith('sheet_') and fn.endswith('.png'):
                shutil.move(p, os.path.join(dest, fn))
        ip = os.path.join(src, 'index.txt')
        if os.path.isfile(ip):
            with open(ip, encoding='utf-8') as f:
                text = f.read()
            if k:
                cut = text.find('\n#')
                text = text[cut + 1:] if cut >= 0 else ''
            index.append(text.rstrip('\n'))
        else:
            problems.append(f'{d}: no index.txt')
        shutil.rmtree(src, ignore_errors=True)
    with open(os.path.join(dest, 'index.txt'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(x for x in index if x) + '\n')
    return problems


def run_sheets(jobs: list[dict], sheet_dir: str, workers: int = 0, runner=subprocess.run) -> tuple[dict, dict]:
    """Runs the jobs of plan_sheets in the pool, merges the parts of "full". Returns ({sheet: 'ok' | 'error: ...'}, timing)
    with timing = {'workers', 'seconds' (wall), 'jobs': n, 'busy_s' (sum of the job times), 'slowest': [(job, s)]}."""
    t0 = time.monotonic()
    if not jobs:
        return {}, {'workers': 0, 'seconds': 0.0, 'jobs': 0, 'busy_s': 0.0, 'slowest': []}
    workers = workers or pool_size(len(jobs))
    named = [(f'{j["sheet"]}#{j["part"]}' if j['part'] is not None else j['sheet'],
              child_command('sheet', *j['args'])) for j in jobs]
    res = run_pool(named, workers, runner)
    out: dict = {}
    for j, (name, _) in zip(jobs, named):
        r = res[name]
        cur = out.get(j['sheet'])
        if not r['ok']:
            out[j['sheet']] = f'error: {r["error"]}' if cur in (None, 'ok') else cur
        elif cur is None:
            out[j['sheet']] = 'ok'
    problems = merge_parts(sheet_dir) if any(j['part'] is not None for j in jobs) else []
    if problems and out.get('full') == 'ok':
        out['full'] = 'error: ' + '; '.join(problems)
    slow = sorted(((n, r['seconds']) for n, r in res.items()), key=lambda x: -x[1])[:3]
    return out, {'workers': workers, 'seconds': round(time.monotonic() - t0, 1), 'jobs': len(jobs),
                 'busy_s': round(sum(r['seconds'] for r in res.values()), 1), 'slowest': slow}


# ---------------------------------------------------------------------------------------------- failure clips
def cut_pool(video: str, jobs, ffmpeg: str, small: bool, workers: int = 0, scratch: str = '', runner=subprocess.run) -> dict:
    """Cuts the clips of failure_clips.make_clips ([(item, out path)]) in the pool. Returns {out path: None | error text}."""
    jobs = list(jobs)
    if not jobs:
        return {}
    workers = workers or pool_size(len(jobs))
    threads = max(1, min(4, (os.cpu_count() or 4) // max(1, workers)))
    scratch = scratch or os.path.join(os.path.dirname(jobs[0][1]), '_clip_specs')
    os.makedirs(scratch, exist_ok=True)
    named = []
    for i, (it, path) in enumerate(jobs):
        spec = {'video': video, 'out': path, 'ffmpeg': ffmpeg, 'small': small, 'threads': threads,
                'item': {k: it[k] for k in ('verdict', 'name', 'why', 'start', 'end')}}
        sp = os.path.join(scratch, f'clip{i:03d}.json')
        with open(sp, 'w', encoding='utf-8') as f:
            json.dump(spec, f)
        named.append((path, child_command('clip', sp)))
    res = run_pool(named, workers, runner)
    shutil.rmtree(scratch, ignore_errors=True)
    return {path: (None if res[path]['ok'] else res[path]['error'] or 'failed') for path, _ in named}


# ---------------------------------------------------------------------------------------------- on-demand: sheets of a run
def _parse_time(s) -> datetime:
    if isinstance(s, datetime):
        return s
    return datetime.strptime(s, '%Y-%m-%d %H:%M:%S.%f' if '.' in s else '%Y-%m-%d %H:%M:%S')


def sheets_for_run(run_dir: str, only: str = '', workers: int = 0, ffmpeg: str = '', keep_frames: bool = False,
                   pad: float = 3.0, runner=subprocess.run, media_root: str | None = None, move: bool = True) -> tuple[dict, dict]:
    """The contact sheets of a finished run folder (summary.json: recording and clock, cycle.json: scenarios). They are built in
    <run>\\sheets\\<name> and, like the sheets of a run that made them itself, the images move to Media\\png\\<date>_night-<run>
    (move=False keeps them in the run folder); summary.json gets the results ("sheets") and the images ("sheet_images"). The
    recording may live in Media already."""
    import video_sheets
    run_dir = os.path.abspath(run_dir)
    with open(os.path.join(run_dir, 'summary.json'), encoding='utf-8') as f:
        summ = json.load(f)
    with open(os.path.join(run_dir, 'cycle.json'), encoding='utf-8') as f:
        cyc = json.load(f)
    rec, ck = summ.get('recording') or {}, summ.get('clock') or {}
    video = next((p for p in (rec.get('path'), rec.get('recording_original')) if p and os.path.isfile(p)), None)
    log = summ.get('utpp_log')
    if not video or not ck.get('video_start') or not log or not os.path.isfile(log):
        raise SystemExit(f'no usable recording, clock or log in {run_dir}')
    scenarios = [{'name': f'{s["name"]}_r{s["run"]}', 'start': _parse_time(s['start']),
                  'end': _parse_time(s['end']) if s.get('end') else None} for s in cyc.get('scenarios', [])]
    sd = os.path.join(run_dir, 'sheets')
    jobs = plan_sheets(video, log, ck['video_end'], ck['clock_offset_s'], ffmpeg or video_sheets.FFMPEG,
                       _parse_time(ck['video_start']), float(ck['duration_s']), scenarios, pad, sd, keep_frames, only,
                       phases=gn.phase_windows_of(cyc.get('scenarios', [])))
    if only and not jobs:
        raise SystemExit(f'no sheet named {only!r} (names: full, <scenario>_r<run>_start / _end, <phase label>_r<run>)')
    res, timing = run_sheets(jobs, sd, workers, runner)
    if move and res:
        import failure_clips
        old = summ.get('sheets')
        summ['sheets'] = dict(old if isinstance(old, dict) else {}, **res)
        notes = failure_clips.move_to_media(run_dir, summ, media_root or failure_clips.MEDIA, log=print)
        timing['media'] = notes
        with open(os.path.join(run_dir, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump(summ, f, indent=1, default=str)
    return res, timing


# ---------------------------------------------------------------------------------------------- command line
def _child_sheet(args: list[str]) -> int:
    import video_sheets
    try:
        return int(video_sheets.annotated_main(args) or 0)
    except SystemExit as e:
        print(str(e.code), file=sys.stderr)
        return 1


def _child_clip(spec_path: str) -> int:
    import failure_clips
    with open(spec_path, encoding='utf-8') as f:
        spec = json.load(f)
    try:
        failure_clips.cut(spec['video'], spec['item'], spec['out'], spec['ffmpeg'], spec['small'], spec.get('threads', 0))
    except Exception as e:  # noqa: BLE001
        print(repr(e), file=sys.stderr)
        return 1
    return 0


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == 'sheet':
        return _child_sheet(argv[1:])
    if argv and argv[0] == 'clip':
        return _child_clip(argv[1])
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sh = sub.add_parser('sheets', help='contact sheets of a finished run folder')
    sh.add_argument('run_dir')
    sh.add_argument('--only', default='', help='one sheet: full or <scenario>_r<run>_start / _end')
    sh.add_argument('--workers', type=int, default=0, help='processes (0 = from the idle cores)')
    sh.add_argument('--ffmpeg', default='')
    sh.add_argument('--keep-frames', action='store_true')
    sh.add_argument('--no-media-move', action='store_true', help='leave the sheet images in the run folder')
    sub.add_parser('cores', help='idle cores and the pool size they give')
    a = ap.parse_args(argv)
    if a.cmd == 'cores':
        idle = idle_cores()
        print(f'cores {os.cpu_count()}, idle {idle:.1f}, pool for 40 jobs {pool_size(40, idle)}')
        return 0
    res, timing = sheets_for_run(a.run_dir, a.only, a.workers, a.ffmpeg, a.keep_frames, move=not a.no_media_move)
    bad = {k: v for k, v in res.items() if v != 'ok'}
    print(f'{len(res)} sheets, {len(bad)} failed, {timing["seconds"]} s with {timing["workers"]} processes '
          f'({timing["jobs"]} jobs, {timing["busy_s"]} s of work)')
    for k, v in bad.items():
        print(f'  {k}: {v}')
    media = timing.get('media') or {}
    if media:
        print(f'{media.get("sheet_images", 0)} sheet images moved to Media' + (f'; problems: {media["errors"]}' if media.get('errors') else ''))
    return 1 if bad or media.get('errors') else 0


if __name__ == '__main__':
    sys.exit(main())
