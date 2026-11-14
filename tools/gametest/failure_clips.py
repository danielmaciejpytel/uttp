"""Cut 1x clips of every non-passing step from a night run recording.

Usage: python -I failure_clips.py <night run dir> [--out DIR] [--pad 2.0] [--full-size] [--per-cause N] [--name TEXT] [--list]
night.py runs it after every cycle (off: --no-failure-clips; --fast leaves it for later and lists the clips in summary.json).
At most N (default 6) clips of each cause are cut: a cause is the verdict and the first failure text with its numbers removed;
the first failing item of each scenario comes first, then the others in time order. --per-cause 0 cuts every failing item,
--name keeps the items whose name contains TEXT, --list prints what would be cut (video seconds) and cuts nothing. The cuts run
in a pool of processes (gt_post.py).

Reads <run>/gametest/summary.json (steps with wall t0/t1 and verdict), <run>/summary.json
(recording clock, scenario verdicts) and <run>/cycle.json (scenario start/end).  A step is cut when its
verdict is FAIL, INVALID or ABORTED; a scenario is cut whole when its night verdict is not PASS and no
step of it was cut.  Clips keep real time (1x) and carry a caption burned in at the top.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
MEDIA = os.path.join(PROJECT, 'Media')
FFMPEG = os.path.join(PROJECT, 'Tools', 'pylibs', 'imageio_ffmpeg', 'binaries', 'ffmpeg-win-x86_64-v7.1.exe')
BAD_STEP = ('FAIL', 'INVALID', 'ABORTED')
MAX_SCENARIO_S = 80.0   # a 190 s whole-scenario clip was 86 MB, over the 30 MB a remote viewer can receive
PER_CAUSE = 6           # clips per cause: the first failing item of each scenario first, then the others in time order
_NUMBER = re.compile(r'-?\d+(?:\.\d+)?')


def _t(s):
    if isinstance(s, datetime):
        return s
    return datetime.strptime(s, '%Y-%m-%d %H:%M:%S.%f' if '.' in s else '%Y-%m-%d %H:%M:%S')


def _load(p):
    with open(p, encoding='utf-8') as f:
        return json.load(f)


def _safe(s: str) -> str:
    return re.sub(r'[^A-Za-z0-9_-]+', '_', s).strip('_')[:60]


def collect(run_dir: str, pad: float) -> tuple[str, list[dict]]:
    night = _load(os.path.join(run_dir, 'summary.json'))
    rec = night.get('recording') or {}
    ck = night.get('clock') or {}
    # the recording may already live in Media\mp4 (move_to_media stores the new path in 'path', the old one in
    # 'recording_original'): the first existing one is used
    video = next((p for p in (rec.get('path'), rec.get('recording_original')) if p and os.path.isfile(p)), None)
    if not video or not ck.get('video_start'):
        raise SystemExit(f'no usable recording in {run_dir}')
    v0, dur = _t(ck['video_start']), float(ck['duration_s'])
    items = []
    gsum = os.path.join(run_dir, 'gametest', 'summary.json')
    cut_scen = set()
    if os.path.isfile(gsum):
        for s in _load(gsum).get('steps', []):
            if s.get('verdict') not in BAD_STEP or not s.get('t0') or not s.get('t1'):
                continue
            cut_scen.add(f'{s["scenario"]}#{s.get("run", 1)}')
            fails = s.get('fails') or []
            items.append({'name': f'{s["scenario"]}_r{s.get("run", 1)}_{s["step"]}', 'verdict': s['verdict'],
                          'scenario': f'{s["scenario"]}#{s.get("run", 1)}',
                          'why': '; '.join(str(x) for x in fails)[:160], 'a': _t(s['t0']), 'b': _t(s['t1'])})
    spans = {f'{c["name"]}#{c.get("run", 1)}': c for c in _load(os.path.join(run_dir, 'cycle.json')).get('scenarios', [])}
    for key, v in (night.get('scenario_verdicts') or {}).items():
        if v.get('verdict') == 'PASS' or key in cut_scen:
            continue
        c = spans.get(key)
        if not c or not c.get('start'):
            continue
        name, run = key.split('#')
        items.append({'name': f'{name}_r{run}', 'verdict': v.get('verdict', '?'), 'scenario': key,
                      'why': '; '.join(str(x) for x in v.get('fails') or [])[:160],
                      'a': _t(c['start']), 'b': _t(c.get('end') or c['start']), 'whole': True})
    for it in items:
        it['start'] = max(0.0, (it['a'] - v0).total_seconds() - pad)
        it['end'] = min(dur - 0.05, (it['b'] - v0).total_seconds() + pad)
    return video, [it for it in items if it['end'] > it['start']]


def _drawtext(text: str) -> str:
    t = text.replace('\\', '/').replace(':', r'\:').replace("'", '').replace('%', r'\%')
    return (f"drawtext=fontfile='C\\:/Windows/Fonts/arial.ttf':text='{t}':x=20:y=20:fontsize=26:fontcolor=white:"
            "box=1:boxcolor=black@0.6:boxborderw=8")


def cause_of(it: dict) -> str:
    """The kind of a failure: the verdict and its first failure text with the numbers removed, so that
    'blocked: V did not move (|v| <= 0.12 m/s ...)' and '... 0.31 m/s ...' are one cause."""
    first = str(it.get('why') or '').split('; ')[0]
    return f'{it.get("verdict", "?")}: ' + _NUMBER.sub('#', first).strip()[:110]


def select_per_cause(items: list[dict], per_cause: int) -> tuple[list[dict], list[dict]]:
    """At most `per_cause` clips of each cause (0 = no limit): the first failing item of each scenario first, then the
    others in time order. Returns (kept items in their original order, [{'cause', 'dropped': [names]}])."""
    if per_cause <= 0:
        return list(items), []
    groups: dict[str, list[dict]] = {}
    for it in sorted(items, key=lambda x: x['a']):
        groups.setdefault(cause_of(it), []).append(it)
    keep_ids, dropped = set(), []
    for cause, its in groups.items():
        seen, firsts, rest = set(), [], []
        for it in its:
            (rest if it.get('scenario') in seen else firsts).append(it)
            seen.add(it.get('scenario'))
        order = firsts + rest
        keep_ids.update(id(it) for it in order[:per_cause])
        if len(order) > per_cause:
            dropped.append({'cause': cause, 'dropped': [it['name'] for it in order[per_cause:]]})
    return [it for it in items if id(it) in keep_ids], dropped


def cut_command(video: str, it: dict, out: str, ffmpeg: str, small: bool, threads: int = 0) -> list[str]:
    caption = f'{it["verdict"]}  {it["name"]}  {it["why"]}'[:150]
    long = it['end'] - it['start'] > 30.0   # 80 s at 1280 px / crf 26 was 34 MB; 960 px / crf 30 was 9 MB
    width, crf = ((960, 30) if long else (1280, 26)) if small else (0, 21)
    vf = _drawtext(caption) + (f',scale={width}:-2' if width else '')
    return [ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-ss', f'{it["start"]:.2f}', '-i', video,
            '-t', f'{it["end"] - it["start"]:.2f}', '-vf', vf, '-an', '-c:v', 'libx264', '-preset', 'veryfast']\
        + (['-threads', str(threads)] if threads else []) \
        + ['-crf', str(crf), '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out]


def cut(video: str, it: dict, out: str, ffmpeg: str, small: bool, threads: int = 0) -> str:
    subprocess.run(cut_command(video, it, out, ffmpeg, small, threads), check=True)
    return out


def add_readme_row(out: str) -> None:
    """Media\\<ext>\\README.md table row '| folder | files | MB |' (Media rule: every folder listed once). The extension
    is the name of the parent folder (mp4, png, ...): the files of that type in `out` are counted."""
    readme = os.path.join(os.path.dirname(out), 'README.md')
    if not os.path.isfile(readme):
        return
    name = os.path.basename(out)
    ext = '.' + os.path.basename(os.path.dirname(out)).lower()
    files = [os.path.join(out, f) for f in os.listdir(out) if f.lower().endswith(ext)]
    row = f'| {name} | {len(files)} | {sum(os.path.getsize(f) for f in files) / 1e6:.1f} |'
    with open(readme, 'rb') as f:
        lines = f.read().decode('utf-8').split('\n')
    lines = [row if ln.startswith(f'| {name} |') else ln for ln in lines]
    if row not in lines:
        while lines and lines[-1] == '':
            lines.pop()
        lines += [row, '']
    with open(readme, 'wb') as f:
        f.write('\n'.join(lines).encode('utf-8'))


def plan_clips(run_dir: str, pad: float = 2.0, per_cause: int = PER_CAUSE, name_filter: str = '',
               max_scenario_s: float = MAX_SCENARIO_S) -> tuple[str, list[dict], list[dict]]:
    """(recording, the items that get a clip, [{'cause', 'dropped'}] of the items the per-cause limit left out). Nothing is
    cut. Every item has its video seconds ('start', 'end'), its 'cause' and the failing step or scenario ('name').
    A failing step and its whole scenario never both get a clip: a scenario is an item only when none of its steps failed.
    name_filter keeps the items whose name contains it (applied before the limit)."""
    video, items = collect(os.path.abspath(run_dir), pad)
    if name_filter:
        items = [it for it in items if name_filter in it['name']]
    kept, dropped = select_per_cause(items, per_cause)
    for it in kept:
        it['cause'] = cause_of(it)
        if it.get('whole') and it['end'] - it['start'] > max_scenario_s:
            it['start'] = it['end'] - max_scenario_s
    return video, kept, dropped


def default_out(run_dir: str) -> str:
    stamp = os.path.basename(os.path.abspath(run_dir))
    return os.path.join(PROJECT, 'Media', 'mp4', f'{stamp[:4]}-{stamp[4:6]}-{stamp[6:8]}_niezaliczone_{stamp}')


def make_clips(run_dir: str, out: str = '', pad: float = 2.0, small: bool = True, ffmpeg: str = FFMPEG,
               max_scenario_s: float = MAX_SCENARIO_S, per_cause: int = PER_CAUSE, name_filter: str = '',
               cutter=None, dropped_out: list | None = None) -> list[dict]:
    """Cut the failing items of a night run into out (default Media\\mp4\\<date>_niezaliczone_<run>) and list the
    folder in Media\\mp4\\README.md: at most per_cause clips of each cause (plan_clips). Whole-scenario items keep only
    their last max_scenario_s s (where it failed). cutter(video, jobs) cuts the jobs [(item, out path)] (default: one
    after the other; night.py passes a process pool) and returns {out path: None | error text}; a clip that did not
    come out is left out of the result and logged to stderr. The causes that were cut short are appended to
    dropped_out."""
    run_dir = os.path.abspath(run_dir)
    video, items, dropped = plan_clips(run_dir, pad, per_cause, name_filter, max_scenario_s)
    if dropped_out is not None:
        dropped_out.extend(dropped)
    if not items:
        return []
    out = out or default_out(run_dir)
    os.makedirs(out, exist_ok=True)
    jobs = []
    for i, it in enumerate(items, 1):
        it['path'] = os.path.join(out, f'{i:02d}_{_safe(it["name"])}.mp4')
        jobs.append((it, it['path']))
    if cutter is None:
        def cutter(video_, jobs_):
            res = {}
            for it_, path in jobs_:
                try:
                    cut(video_, it_, path, ffmpeg, small)
                    res[path] = None
                except (OSError, subprocess.SubprocessError) as e:
                    res[path] = repr(e)
            return res
    results = cutter(video, jobs)
    done = []
    for it in items:
        err = results.get(it['path'])
        if err:
            print(f'clip {it["path"]} failed: {err}', file=sys.stderr)
            continue
        del it['a'], it['b']
        done.append(it)
    if os.path.normcase(os.path.dirname(out)) == os.path.normcase(os.path.join(PROJECT, 'Media', 'mp4')):
        add_readme_row(out)
    return done


def media_folder(run_dir: str, ext: str, media_root: str = MEDIA) -> str:
    """Media\\<ext>\\<YYYY-MM-DD>_night-<stamp>: the date comes from the run's stamp (YYYYMMDD-HHMMSS), not from today."""
    stamp = os.path.basename(os.path.abspath(run_dir))
    day = f'{stamp[:4]}-{stamp[4:6]}-{stamp[6:8]}' if re.match(r'\d{8}-\d{6}', stamp) else f'{datetime.now():%Y-%m-%d}'
    return os.path.join(media_root, ext, f'{day}_night-{stamp}')


def _unique(dest: str) -> str:
    """Never overwrite in Media: a taken name gets '~2', '~3' (the convention of the Media READMEs)."""
    if not os.path.exists(dest):
        return dest
    stem, ext = os.path.splitext(dest)
    n = 2
    while os.path.exists(f'{stem}~{n}{ext}'):
        n += 1
    return f'{stem}~{n}{ext}'


def _inside(path: str, folder: str) -> bool:
    return os.path.normcase(os.path.abspath(path)).startswith(os.path.normcase(os.path.abspath(folder)) + os.sep)


def move_to_media(run_dir: str, summary: dict, media_root: str = MEDIA, log=print) -> dict:
    """Media rule: the recording and the sheet images of a night run go to Media\\mp4 / Media\\png.

    - rec_*.mp4 (summary['recording']['path'], else any rec_*.mp4 of the run folder) -> Media\\mp4\\<date>_night-<stamp>\\;
      summary['recording'] gets the new 'path' and the old one as 'recording_original';
    - sheets\\<name>\\sheet_NN.png -> Media\\png\\<date>_night-<stamp>\\<name>__sheet_NN.png (summary['sheet_images'] =
      [{'name', 'path', 'original'}]); the per-frame images (sheets\\<name>\\frames\\), index.txt and the other files
      stay where they are;
    - each folder gets / updates its row in the README of Media\\<ext>;
    - a failed move is logged, listed in the returned 'errors' and never raised; what is already moved is not moved again.
    Returns {'recording': new path | None, 'sheet_images': n, 'errors': [...]}."""
    run_dir = os.path.abspath(run_dir)
    notes: dict = {'recording': None, 'sheet_images': 0, 'errors': []}

    def fail(what: str, e: Exception) -> None:
        msg = f'{what}: {e!r}'
        notes['errors'].append(msg)
        log(f'[media] move failed - {msg}')

    rec = summary.get('recording') if isinstance(summary.get('recording'), dict) else None
    src = rec.get('path') if rec else None
    if not (src and os.path.isfile(src)):
        found = sorted(glob.glob(os.path.join(run_dir, 'rec_*.mp4')))
        src = found[0] if found else None
    if src and os.path.isfile(src) and _inside(src, run_dir):
        try:
            out = media_folder(run_dir, 'mp4', media_root)
            os.makedirs(out, exist_ok=True)
            dest = _unique(os.path.join(out, os.path.basename(src)))
            shutil.move(src, dest)
            if rec is None:
                rec = summary['recording'] = {}
            rec['recording_original'], rec['path'] = src, dest
            notes['recording'] = dest
            log(f'[media] recording -> {dest}')
            try:
                add_readme_row(out)
            except OSError as e:
                fail('README row (mp4)', e)
        except OSError as e:
            fail(f'recording {src}', e)
    images = []
    for sheet in sorted(glob.glob(os.path.join(run_dir, 'sheets', '*', 'sheet_*.png'))):
        name = os.path.basename(os.path.dirname(sheet))
        try:
            out = media_folder(run_dir, 'png', media_root)
            os.makedirs(out, exist_ok=True)
            dest = _unique(os.path.join(out, f'{name}__{os.path.basename(sheet)}'))
            shutil.move(sheet, dest)
            images.append({'name': name, 'path': dest, 'original': sheet})
        except OSError as e:
            fail(f'sheet {sheet}', e)
    if images:
        summary['sheet_images'] = list(summary.get('sheet_images') or []) + images
        notes['sheet_images'] = len(images)
        log(f'[media] {len(images)} sheet images -> {os.path.dirname(images[0]["path"])}')
        try:
            add_readme_row(os.path.dirname(images[0]['path']))
        except OSError as e:
            fail('README row (png)', e)
    return notes


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--out', default='')
    ap.add_argument('--pad', type=float, default=2.0)
    ap.add_argument('--full-size', action='store_true', help='1920 px, crf 21 (default: 1280 px, crf 26 for sending)')
    ap.add_argument('--max-scenario-s', type=float, default=MAX_SCENARIO_S)
    ap.add_argument('--ffmpeg', default=FFMPEG)
    ap.add_argument('--per-cause', type=int, default=PER_CAUSE,
                    help='clips per cause (the failure kind with its numbers removed): the first failing item of each scenario '
                         'first; 0 = every failing item (default %(default)s)')
    ap.add_argument('--name', default='', help='only the failing steps / scenarios whose name contains this text')
    ap.add_argument('--list', action='store_true',
                    help='cut nothing: list what would be cut with its video seconds (the on-demand form after a --fast run)')
    ap.add_argument('--workers', type=int, default=0, help='clips cut in parallel (0 = from the idle cores)')
    a = ap.parse_args(argv)
    if a.list:
        video, items, dropped = plan_clips(a.run_dir, a.pad, a.per_cause, a.name, a.max_scenario_s)
        print(f'recording {video}')
        for it in items:
            print(f'{it["name"]}\t{it["verdict"]}\t{it["start"]:.1f}-{it["end"]:.1f} s\t{it["cause"]}')
        for d in dropped:
            print(f'(not listed: {len(d["dropped"])} more of the cause {d["cause"]!r}; --per-cause 0 lists them)')
        if not items:
            print('no failing steps')
        return 0
    import gt_post
    dropped: list = []
    items = make_clips(a.run_dir, a.out, a.pad, not a.full_size, a.ffmpeg, a.max_scenario_s, a.per_cause, a.name,
                       cutter=lambda video, jobs: gt_post.cut_pool(video, jobs, a.ffmpeg, not a.full_size, a.workers,
                                                                   scratch=os.path.join(os.path.abspath(a.run_dir), '_tmp', 'clips')),
                       dropped_out=dropped)
    if not items:
        print('no failing steps')
    for it in items:
        print(f'{it["path"]}\t{it["verdict"]}\t{it["end"] - it["start"]:.1f}s\t{it["why"]}')
    for d in dropped:
        print(f'not cut: {len(d["dropped"])} more of the cause {d["cause"]!r} (--per-cause 0 cuts them)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
