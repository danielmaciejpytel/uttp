"""Cut 1x clips of every non-passing step from a night run recording.

Usage: python -I failure_clips.py <night run dir> [--out DIR] [--pad 2.0] [--full-size]
night.py runs it after every cycle (off: --no-failure-clips).

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
                          'why': '; '.join(str(x) for x in fails)[:160], 'a': _t(s['t0']), 'b': _t(s['t1'])})
    spans = {f'{c["name"]}#{c.get("run", 1)}': c for c in _load(os.path.join(run_dir, 'cycle.json')).get('scenarios', [])}
    for key, v in (night.get('scenario_verdicts') or {}).items():
        if v.get('verdict') == 'PASS' or key in cut_scen:
            continue
        c = spans.get(key)
        if not c or not c.get('start'):
            continue
        name, run = key.split('#')
        items.append({'name': f'{name}_r{run}', 'verdict': v.get('verdict', '?'),
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


def cut(video: str, it: dict, out: str, ffmpeg: str, small: bool) -> str:
    caption = f'{it["verdict"]}  {it["name"]}  {it["why"]}'[:150]
    long = it['end'] - it['start'] > 30.0   # 80 s at 1280 px / crf 26 was 34 MB; 960 px / crf 30 was 9 MB
    width, crf = ((960, 30) if long else (1280, 26)) if small else (0, 21)
    vf = _drawtext(caption) + (f',scale={width}:-2' if width else '')
    cmd = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-ss', f'{it["start"]:.2f}', '-i', video,
           '-t', f'{it["end"] - it["start"]:.2f}', '-vf', vf, '-an', '-c:v', 'libx264', '-preset', 'veryfast',
           '-crf', str(crf), '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out]
    subprocess.run(cmd, check=True)
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


def make_clips(run_dir: str, out: str = '', pad: float = 2.0, small: bool = True, ffmpeg: str = FFMPEG,
               max_scenario_s: float = MAX_SCENARIO_S) -> list[dict]:
    """Cut every failing item of a night run into out (default Media\\mp4\\<date>_niezaliczone_<run>) and list the
    folder in Media\\mp4\\README.md. Whole-scenario items keep only their last max_scenario_s s (where it failed)."""
    run_dir = os.path.abspath(run_dir)
    video, items = collect(run_dir, pad)
    if not items:
        return []
    stamp = os.path.basename(run_dir)
    out = out or os.path.join(PROJECT, 'Media', 'mp4', f'{stamp[:4]}-{stamp[4:6]}-{stamp[6:8]}_niezaliczone_{stamp}')
    os.makedirs(out, exist_ok=True)
    for i, it in enumerate(items, 1):
        if it.get('whole') and it['end'] - it['start'] > max_scenario_s:
            it['start'] = it['end'] - max_scenario_s
        it['path'] = cut(video, it, os.path.join(out, f'{i:02d}_{_safe(it["name"])}.mp4'), ffmpeg, small)
        del it['a'], it['b']
    if os.path.normcase(os.path.dirname(out)) == os.path.normcase(os.path.join(PROJECT, 'Media', 'mp4')):
        add_readme_row(out)
    return items


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
    a = ap.parse_args(argv)
    items = make_clips(a.run_dir, a.out, a.pad, not a.full_size, a.ffmpeg, a.max_scenario_s)
    if not items:
        print('no failing steps')
    for it in items:
        print(f'{it["path"]}\t{it["verdict"]}\t{it["end"] - it["start"]:.1f}s\t{it["why"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
