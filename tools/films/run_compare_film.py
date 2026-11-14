"""Side-by-side films of the same scenario steps from two night runs, at real-time speed.

  python run_compare_film.py --left <run folder> --right <run folder> --left-title "Game Animations"
      --right-title "Ultimate Third Person Project" --out <folder> [--scenario a,b | --steps a/step,b/step --film NAME]
      [--run N] [--pre S] [--post S] [--dry-run] [--overwrite] [--ffmpeg PATH]

Both folders are run folders of night.py that played the same scenarios (each holds summary.json with the clock and the
recording of the game window, cycle.json, and gametest/summary.json with the start and end of every step). For every
selected step (or whole scenario) the same stretch is cut from both recordings, starting at the step start of each run, and
put left | right in one frame at 1x speed: 30 fps, each side at most 960 x 1080 (scaled to fit, aspect kept: two 16:9
recordings give a 1920 x 540 film), the titles in the top left corner of each side. Both recordings must have the same shape. A side that is shorter than the other is held on its last frame, so both halves always
run for the same time. One mp4 per scenario (default), or one film of all the listed steps in order (--steps with --film).
A step whose time is missing in one of the runs, a recording that does not cover the step, or a missing recording stops the
run before anything is written. Nothing is written outside --out.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
GAMETEST = os.path.normpath(os.path.join(HERE, '..', 'gametest'))
if GAMETEST not in sys.path:
    sys.path.insert(0, GAMETEST)

FPS = 30
SIDE_W = 960
SIDE_H = 1080
CRF = 20
MAX_TITLE = 60


class FilmError(ValueError):
    """The films cannot be planned (missing step, recording or clock); nothing has been written."""


# ---------------------------------------------------------------------------------------------- run folders
def _t(s) -> datetime:
    if isinstance(s, datetime):
        return s
    return datetime.strptime(s, '%Y-%m-%d %H:%M:%S.%f' if '.' in s else '%Y-%m-%d %H:%M:%S')


def _load(path: str) -> dict:
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def load_run(run_dir: str) -> dict:
    """What a run folder tells about the recording and the steps: {'dir', 'video', 'video_start', 'duration', 'steps':
    {(scenario, run): [(step, t0, t1)] in play order}, 'scenarios': {(scenario, run): (start, end)}}. The steps come from
    gametest/summary.json (every step of every pad scenario, the parts of macros under their own names) and from the phase
    windows of the macros in cycle.json. Raises FilmError when the folder has no usable recording / clock."""
    run_dir = os.path.abspath(run_dir)
    try:
        summ = _load(os.path.join(run_dir, 'summary.json'))
    except (OSError, ValueError) as e:
        raise FilmError(f'{run_dir}: summary.json not readable ({e})') from None
    rec, ck = summ.get('recording') or {}, summ.get('clock') or {}
    # the recording may already live in Media (the new path is 'path', the old one 'recording_original')
    video = next((p for p in (rec.get('path'), rec.get('recording_original')) if p and os.path.isfile(p)), None)
    if not video:
        raise FilmError(f'{run_dir}: no recording file (summary.json recording.path / recording_original)')
    if not ck.get('video_start') or not ck.get('duration_s'):
        raise FilmError(f'{run_dir}: summary.json has no clock (video_start / duration_s)')
    steps: dict = {}
    gpath = os.path.join(run_dir, 'gametest', 'summary.json')
    if os.path.isfile(gpath):
        for s in _load(gpath).get('steps') or []:
            if not s.get('t0') or not s.get('t1') or not s.get('scenario') or not s.get('step'):
                continue
            steps.setdefault((s['scenario'], int(s.get('run', 1))), []).append((s['step'], _t(s['t0']), _t(s['t1'])))
    scenarios: dict = {}
    cpath = os.path.join(run_dir, 'cycle.json')
    if os.path.isfile(cpath):
        for c in _load(cpath).get('scenarios') or []:
            key = (c['name'], int(c.get('run', 1)))
            if c.get('start') and c.get('end'):
                scenarios[key] = (_t(c['start']), _t(c['end']))
            # the macros that play several things (melee weapons, fists) record their phases with wall-clock windows: a step
            # of the macro named '<weapon>.<phase>' (or '<phase>'), so a phase is paired by name and not by play order
            for w in c.get('phase_windows') or []:
                if w.get('start') and w.get('end') and w.get('phase'):
                    name = f'{w["weapon"]}.{w["phase"]}' if w.get('weapon') else w['phase']
                    steps.setdefault(key, []).append((name, _t(w['start']), _t(w['end'])))
    return {'dir': run_dir, 'video': video, 'video_start': _t(ck['video_start']), 'duration': float(ck['duration_s']),
            'steps': steps, 'scenarios': scenarios}


def scenario_span(run: dict, name: str, number: int):
    """(start, end) wall times of a whole scenario in a run: first step start to last step end, else the cycle record."""
    st = run['steps'].get((name, number))
    if st:
        return min(a for _, a, _ in st), max(b for _, _, b in st)
    return run['scenarios'].get((name, number))


def step_span(run: dict, name: str, step: str, number: int):
    return next(((a, b) for s, a, b in run['steps'].get((name, number), []) if s == step), None)


def common_scenarios(left: dict, right: dict, number: int) -> list[str]:
    """The scenarios of the cycle record (what the set asked for; not the internal probes and macro parts that
    gametest/summary.json also lists) that both runs have for this run number, in the play order of the left run."""
    starts = {name: a for (name, r), (a, _) in left['scenarios'].items() if r == number}
    if not starts:
        starts = {name: min(a for _, a, _ in st) for (name, r), st in left['steps'].items() if r == number}
    return [n for n in sorted(starts, key=starts.get) if scenario_span(right, n, number) is not None]


# ---------------------------------------------------------------------------------------------- planning
def _side(run: dict, span, pre: float, post: float, label: str, which: str) -> dict:
    a, b = span
    start = (a - run['video_start']).total_seconds() - pre
    end = (b - run['video_start']).total_seconds() + post
    if start < 0:
        raise FilmError(f'{label}: the {which} recording starts {-start:.2f} s after the step start')
    if start >= run['duration']:
        raise FilmError(f'{label}: the {which} recording ends ({run["duration"]:.1f} s) before the step start ({start:.1f} s)')
    return {'start': round(start, 3), 'end': round(min(end, run['duration']), 3), 'cut_at_end': end > run['duration']}


def _pair(left: dict, right: dict, label: str, lspan, rspan, pre: float, post: float) -> dict:
    ls = _side(left, lspan, pre, post, label, 'left')
    rs = _side(right, rspan, pre, post, label, 'right')
    duration = round(max(ls['end'] - ls['start'], rs['end'] - rs['start']), 3)
    if duration <= 0:
        raise FilmError(f'{label}: zero length')
    return {'label': label, 'left': ls, 'right': rs, 'duration': duration}


def plan_films(left: dict, right: dict, scenarios: list[str] | None = None, steps: list[str] | None = None,
               film: str = '', number: int = 1, pre: float = 0.0, post: float = 0.0) -> list[dict]:
    """The films to make: [{'name', 'pairs': [{'label', 'left': {start, end}, 'right': {...}, 'duration'}]}] (video seconds).
    steps = ['scenario/step', ...] -> ONE film (`film` is its name) with a pair per step; otherwise one film per scenario
    (`scenarios`, default: every scenario both runs have) with one pair spanning the whole scenario. Raises FilmError for a
    step / scenario that has no time in one of the runs (the message names the run)."""
    if steps:
        if not film:
            raise FilmError('--steps needs --film NAME (the name of the one film they go into)')
        pairs, missing = [], []
        for item in steps:
            if item.count('/') != 1 or not all(item.split('/')):
                raise FilmError(f'step {item!r}: expected scenario/step')
            name, step = item.split('/')
            ls, rs = step_span(left, name, step, number), step_span(right, name, step, number)
            for which, sp in (('left', ls), ('right', rs)):
                if sp is None:
                    missing.append(f'{item} (run {number}) has no time in the {which} run')
            if ls is not None and rs is not None:
                pairs.append(_pair(left, right, f'{name}/{step}', ls, rs, pre, post))
        if missing:
            raise FilmError('; '.join(missing))
        return [{'name': film, 'pairs': pairs}]
    names = scenarios or common_scenarios(left, right, number)
    if not names:
        raise FilmError(f'no scenario with run {number} in both runs')
    films, missing = [], []
    for name in names:
        ls, rs = scenario_span(left, name, number), scenario_span(right, name, number)
        for which, sp in (('left', ls), ('right', rs)):
            if sp is None:
                missing.append(f'{name} (run {number}) has no time in the {which} run')
        if ls is not None and rs is not None:
            films.append({'name': f'{name}_r{number}', 'pairs': [_pair(left, right, name, ls, rs, pre, post)]})
    if missing:
        raise FilmError('; '.join(missing))
    return films


# ---------------------------------------------------------------------------------------------- ffmpeg
def clean_title(text: str) -> str:
    """A title is plain words: no filter syntax characters, at most MAX_TITLE characters."""
    return re.sub(r'[^\w .\-+&()/]', ' ', text, flags=re.UNICODE).strip()[:MAX_TITLE] or ' '


def _drawtext(text: str) -> str:
    import failure_clips                       # the caption style of the failure clips (font path, box, escaping)
    return failure_clips._drawtext(clean_title(text))


def default_ffmpeg() -> str:
    import video_sheets                        # the ffmpeg gt_post.py uses for the sheets
    return video_sheets.FFMPEG


def filter_script(film: dict, left_title: str, right_title: str, fps: int = FPS, width: int = SIDE_W,
                  height: int = SIDE_H) -> str:
    """The filter graph of one film: every input is set to fps, scaled to fit width x height (aspect kept), titled, held on
    its last frame up to the pair length and cut to it; the pairs are joined left | right and then one after the other."""
    parts = []
    n = len(film['pairs'])
    for i, p in enumerate(film['pairs']):
        for k, (tag, title) in enumerate((('l', left_title), ('r', right_title))):
            idx = 2 * i + k
            parts.append(f'[{idx}:v]fps={fps},scale={width}:{height}:force_original_aspect_ratio=decrease:force_divisible_by=2,'
                         f'setsar=1,{_drawtext(title)},'
                         f'tpad=stop_mode=clone:stop_duration={p["duration"]:.3f},trim=duration={p["duration"]:.3f},'
                         f'setpts=PTS-STARTPTS[{tag}{i}]')
        parts.append(f'[l{i}][r{i}]hstack=inputs=2[p{i}]')
    if n == 1:
        parts.append('[p0]null[v]')
    else:
        parts.append(''.join(f'[p{i}]' for i in range(n)) + f'concat=n={n}:v=1:a=0[v]')
    return ';\n'.join(parts)


def build_command(ffmpeg: str, left_video: str, right_video: str, film: dict, script_path: str, out: str,
                  fps: int = FPS, crf: int = CRF) -> list[str]:
    """One ffmpeg call for a film: per pair a seek + length on both recordings, the graph from the script file, mp4 h264."""
    cmd = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-y']
    for p in film['pairs']:
        for video, side in ((left_video, p['left']), (right_video, p['right'])):
            length = min(p['duration'], side['end'] - side['start'])
            cmd += ['-ss', f'{side["start"]:.3f}', '-t', f'{length:.3f}', '-i', video]
    return cmd + ['-filter_complex_script', script_path, '-map', '[v]', '-an', '-r', str(fps), '-c:v', 'libx264',
                  '-preset', 'veryfast', '-crf', str(crf), '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out]


def _inside(path: str, folder: str) -> bool:
    a, b = os.path.normcase(os.path.abspath(path)), os.path.normcase(os.path.abspath(folder))
    return a == b or a.startswith(b + os.sep)


def render(films: list[dict], left: dict, right: dict, out_dir: str, left_title: str, right_title: str, ffmpeg: str,
           overwrite: bool = False, runner=subprocess.run) -> list[str]:
    """Writes the films into out_dir (nothing elsewhere; a film that exists is refused unless overwrite). Returns the files."""
    out_dir = os.path.abspath(out_dir)
    targets = [os.path.join(out_dir, re.sub(r'[^A-Za-z0-9_.-]+', '_', f['name']) + '.mp4') for f in films]
    for t in targets:
        if not _inside(t, out_dir):
            raise FilmError(f'{t} is outside --out')
        if os.path.exists(t) and not overwrite:
            raise FilmError(f'{t} exists (use --overwrite)')
    os.makedirs(out_dir, exist_ok=True)
    done = []
    for f, target in zip(films, targets):
        script = os.path.join(out_dir, f'_filter_{os.getpid()}.txt')
        with open(script, 'w', encoding='utf-8', newline='\n') as fh:
            fh.write(filter_script(f, left_title, right_title))
        try:
            p = runner(build_command(ffmpeg, left['video'], right['video'], f, script, target), capture_output=True,
                       text=True, errors='replace')
        finally:
            try:
                os.remove(script)
            except OSError:
                pass
        if p.returncode != 0:
            if os.path.isfile(target):
                os.remove(target)
            raise FilmError(f'ffmpeg failed for {f["name"]} (both recordings must have the same shape): '
                            f'{(p.stderr or "").strip().splitlines()[-1:] or p.returncode}')
        done.append(target)
    return done


# ---------------------------------------------------------------------------------------------- command line
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--left', required=True, help='run folder shown on the left')
    ap.add_argument('--right', required=True, help='run folder shown on the right')
    ap.add_argument('--left-title', required=True)
    ap.add_argument('--right-title', required=True)
    ap.add_argument('--out', required=True, help='folder the mp4 files are written to (the only place written)')
    ap.add_argument('--scenario', default='', help='comma list of scenarios (default: every scenario both runs have)')
    ap.add_argument('--steps', default='', help='comma list of scenario/step: all of them go into ONE film (--film)')
    ap.add_argument('--film', default='', help='name of the film made from --steps')
    ap.add_argument('--run', type=int, default=1, help='run number of the scenarios (default 1)')
    ap.add_argument('--pre', type=float, default=0.0, help='seconds before the step start (both sides)')
    ap.add_argument('--post', type=float, default=0.0, help='seconds after the step end (both sides)')
    ap.add_argument('--ffmpeg', default='', help='ffmpeg executable (default: the one gt_post.py uses)')
    ap.add_argument('--overwrite', action='store_true')
    ap.add_argument('--dry-run', action='store_true', help='print the plan and the ffmpeg command, write nothing')
    a = ap.parse_args(argv)
    try:
        left, right = load_run(a.left), load_run(a.right)
        films = plan_films(left, right, [s.strip() for s in a.scenario.split(',') if s.strip()] or None,
                           [s.strip() for s in a.steps.split(',') if s.strip()] or None, a.film, a.run, a.pre, a.post)
        ffmpeg = a.ffmpeg or default_ffmpeg()
        for f in films:
            total = sum(p['duration'] for p in f['pairs'])
            print(f'{f["name"]}: {len(f["pairs"])} pair(s), {total:.1f} s')
            for p in f['pairs']:
                print(f'   {p["label"]:40} left {p["left"]["start"]:.2f}-{p["left"]["end"]:.2f}  '
                      f'right {p["right"]["start"]:.2f}-{p["right"]["end"]:.2f}  {p["duration"]:.2f} s')
        if a.dry_run:
            f = films[0]
            print('ffmpeg: ' + ' '.join(build_command(ffmpeg, left['video'], right['video'], f, '<filter script>',
                                                      os.path.join(os.path.abspath(a.out), f['name'] + '.mp4'))))
            return 0
        for path in render(films, left, right, a.out, a.left_title, a.right_title, ffmpeg, a.overwrite):
            print('wrote ' + path)
        return 0
    except FilmError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
