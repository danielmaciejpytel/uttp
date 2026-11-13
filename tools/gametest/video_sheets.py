"""Contact sheets of a V-centred crop of a long recording, with the video time burnt into every tile.

  python video_sheets.py <video> <outdir> <name> <t0> <t1> <fps> [--cols 3 --rows 2 --crop W:H:X:Y --tile-w 640]

Tiles carry "t=<seconds into the video>" (drawtext, offset t0) so every finding can be mapped to the log time base.
ffmpeg path and font are the project's (Tools\\pylibs, Windows Arial).

Library helpers (restored 2026-11-09 verbatim from commit 8426179; the 2026-10-25 rewrite 2430cb7 dropped them although
test_gametest.py and night.py still use them): parse_duration / probe_duration (container duration from `ffmpeg -i`)
and frame_annotation (UTPP trace text for one video frame, lines from gt_log.parse_text / parse_file). The old annotated
CLI is restored as the library function annotated_main(argv) for night.py (--video/--log flags); the positional main()
below stays the only command line.
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import textwrap
from datetime import datetime, timedelta

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from utpp_paths import project_root  # noqa: E402

FFMPEG = os.path.join(project_root(), "Tools", "pylibs", "imageio_ffmpeg", "binaries", "ffmpeg-win-x86_64-v7.1.exe")
FONT = "C\\:/Windows/Fonts/arial.ttf"
DUR_RE = re.compile(r'Duration:\s*(\d+):(\d\d):(\d\d(?:\.\d+)?)')


def parse_duration(ffmpeg_stderr: str) -> float | None:
    m = DUR_RE.search(ffmpeg_stderr)
    if not m:
        return None
    return int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3))


def probe_duration(video: str, ffmpeg: str = FFMPEG) -> float:
    p = subprocess.run([ffmpeg, '-hide_banner', '-i', video], capture_output=True, text=True, errors='replace')
    d = parse_duration(p.stderr)
    if d is None:
        raise SystemExit(f'could not parse duration of {video}')
    return d


def frame_annotation(lines: list, wall: datetime, half: float, wrap: int = 70, max_lines: int = 7) -> list[str]:
    """Text lines for one frame: last trace sample at/before `wall` + events within +-half seconds."""
    last = None
    last_motion = None
    for l in lines:
        if l.ts > wall:
            break
        if l.is_trace:
            last = l
        if 'UTPP MOTION ' in l.raw:
            last_motion = l
    lo, hi = wall - timedelta(seconds=half), wall + timedelta(seconds=half)
    events = [l for l in lines if lo <= l.ts <= hi and l.kind in ('TRANS', 'GATE') and l is not last]
    out = []
    motion_fresh = last_motion is not None and (wall - last_motion.ts).total_seconds() <= 2.0
    if motion_fresh:
        body = last_motion.raw.split('UTPP MOTION ', 1)[1]
        fields = dict(re.findall(r'(?:^|\s)(db|clip|ov|lay|aimW)=([^\s]+)', body))
        age = (wall - last_motion.ts).total_seconds()
        summary = ' '.join(f'{key}={fields[key]}' for key in ('clip', 'db', 'ov', 'lay', 'aimW') if key in fields)
        out += textwrap.wrap(f'[-{age:.2f}s] motion {summary}', wrap)
    if last is not None:
        age = (wall - last.ts).total_seconds()
        prefix = f'[-{age:.2f}s] base ' if motion_fresh else f'[-{age:.2f}s] '
        out += textwrap.wrap(prefix + last.compact()[13:], wrap)
    else:
        if not motion_fresh:
            out.append('(no trace sample before this time)')
    for e in events:
        out += textwrap.wrap(e.compact(), wrap)
    return out[:max_lines]


def annotated_main(argv=None) -> int:
    """Annotated sheets for night.py (restored 2026-11-09 from main(argv) of commit 8426179, unchanged apart from the name
    and the lazy gt_log import). Frames every --every s with the matching UTPP trace lines; frames\\fNNNN.png, sheet_NN.png,
    index.txt in --out. Wall clock: recording end = mtime or --video-end, start = end - duration (+ --clock-offset)."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gt_log  # noqa: E402
    project = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
    ap = argparse.ArgumentParser(description=annotated_main.__doc__)
    ap.add_argument('--video', required=True)
    ap.add_argument('--log', default='', help='UTPP log (default newest in the red4ext logs dir)')
    ap.add_argument('--every', type=float, default=0.5)
    ap.add_argument('--start', type=float, default=0.0)
    ap.add_argument('--end', type=float, default=None)
    ap.add_argument('--cols', type=int, default=4)
    ap.add_argument('--rows', type=int, default=3)
    ap.add_argument('--width', type=int, default=480)
    ap.add_argument('--text-height', type=int, default=120)
    ap.add_argument('--video-end', default='', help='wall-clock end of the recording "YYYY-mm-dd HH:MM:SS[.fff]"')
    ap.add_argument('--clock-offset', type=float, default=0.0)
    ap.add_argument('--out', default='')
    ap.add_argument('--ffmpeg', default=FFMPEG)
    ap.add_argument('--keep-sheet-frames', action=argparse.BooleanOptionalAction, default=False,
                    help='keep the per-frame images (frames\\fNNNN.png / .txt, intermediate files of the sheets) after '
                         'the sheets are built; default: they are deleted once the sheets exist')
    a = ap.parse_args(argv)

    log = a.log or gt_log.newest_log()
    if not log or not os.path.isfile(log):
        raise SystemExit(f'UTPP log not found: {log}')
    dur = probe_duration(a.video, a.ffmpeg)
    if a.video_end:
        fmt = '%Y-%m-%d %H:%M:%S.%f' if '.' in a.video_end else '%Y-%m-%d %H:%M:%S'
        vend = datetime.strptime(a.video_end, fmt)
    else:
        vend = datetime.fromtimestamp(os.path.getmtime(a.video))
    vstart = vend - timedelta(seconds=dur) + timedelta(seconds=a.clock_offset)
    lines = gt_log.parse_file(log)
    end = min(a.end if a.end is not None else dur, dur - 0.05)
    times = []
    t = a.start
    while t <= end + 1e-9:
        times.append(round(t, 3))
        t += a.every
    if not times:
        raise SystemExit('no frame times in range')

    stem = os.path.splitext(os.path.basename(a.video))[0]
    out = a.out or os.path.join(project, 'Research', 'logs', 'gametest', f'video_{stem}_{datetime.now():%Y%m%d-%H%M%S}')
    fdir = os.path.join(out, 'frames')
    os.makedirs(fdir, exist_ok=True)
    print(f'video {a.video}: {dur:.2f} s, wall {vstart:%H:%M:%S.%f}'[:-3] + f' -> {vend:%H:%M:%S}; log {log}')

    index = [f'video {a.video}\nlog {log}\nvideo start (wall) {vstart}  duration {dur:.3f} s  offset {a.clock_offset}\n']
    drawtext_ok = True
    per_sheet = a.cols * a.rows
    font = 'C\\:/Windows/Fonts/consola.ttf'
    for i, vt in enumerate(times, start=1):
        wall = vstart + timedelta(seconds=vt)
        ann = frame_annotation(lines, wall, a.every / 2, max_lines=max(1, (a.text_height - 6) // 15 - 1))
        header = f'v={vt:7.2f}s  {wall:%H:%M:%S.%f}'[:-3] + f'  #{i}'
        txt = os.path.join(fdir, f'f{i:04d}.txt')
        with open(txt, 'w', encoding='utf-8', newline='\n') as f:  # CR would render as an empty line
            f.write('\n'.join([header] + ann).replace('%', '%%'))
        index.append(f'#{i:04d} sheet {(i - 1) // per_sheet + 1:02d}  {header}\n    ' + '\n    '.join(ann))
        vf = f'scale={a.width}:-2,pad=iw:ih+{a.text_height}:0:0:black'
        if drawtext_ok:
            vf += (f",drawtext=fontfile='{font}':textfile='f{i:04d}.txt':x=4:y=h-{a.text_height - 4}:"
                   f'fontsize=11:fontcolor=white:line_spacing=2')
        cmd = [a.ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-ss', f'{vt:.3f}', '-i',
               os.path.abspath(a.video), '-frames:v', '1', '-vf', vf, f'f{i:04d}.png']
        p = subprocess.run(cmd, cwd=fdir, capture_output=True, text=True, errors='replace')
        if p.returncode != 0 and drawtext_ok:
            print(f'drawtext failed ({p.stderr.strip()[:200]}); continuing without burned-in text', file=sys.stderr)
            drawtext_ok = False
            cmd[cmd.index('-vf') + 1] = f'scale={a.width}:-2,pad=iw:ih+{a.text_height}:0:0:black'
            p = subprocess.run(cmd, cwd=fdir, capture_output=True, text=True, errors='replace')
        if p.returncode != 0:
            raise SystemExit(f'ffmpeg frame {i} failed: {p.stderr.strip()[:300]}')

    sheets = 0
    for s, first in enumerate(range(1, len(times) + 1, per_sheet), start=1):
        p = subprocess.run([a.ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-start_number', str(first),
                            '-i', 'f%04d.png', '-frames:v', '1', '-vf',
                            f'tile={a.cols}x{a.rows}:padding=4:margin=4', os.path.join('..', f'sheet_{s:02d}.png')],
                           cwd=fdir, capture_output=True, text=True, errors='replace')
        if p.returncode != 0:
            raise SystemExit(f'ffmpeg sheet {s} failed: {p.stderr.strip()[:300]}')
        sheets += 1
    with open(os.path.join(out, 'index.txt'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(index) + '\n')
    if not a.keep_sheet_frames:
        shutil.rmtree(fdir, ignore_errors=True)
    print(f'{len(times)} frames, {sheets} sheets -> {out}' + ('' if a.keep_sheet_frames else ' (frames deleted)'))
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("video")
    p.add_argument("outdir")
    p.add_argument("name")
    p.add_argument("t0", type=float)
    p.add_argument("t1", type=float)
    p.add_argument("fps", type=float)
    p.add_argument("--cols", type=int, default=3)
    p.add_argument("--rows", type=int, default=2)
    p.add_argument("--crop", default="1100:980:410:100")
    p.add_argument("--tile-w", type=int, default=640)
    a = p.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    vf = (f"fps={a.fps},crop={a.crop},scale={a.tile_w}:-2,"
          f"drawtext=fontfile='{FONT}':text='t=%{{pts\\:flt\\:{a.t0}}}':x=8:y=8:fontsize=26:fontcolor=yellow:box=1:boxcolor=black@0.6,"
          f"tile={a.cols}x{a.rows}")
    out = os.path.join(a.outdir, a.name + "_%02d.png")
    cmd = [FFMPEG, "-y", "-loglevel", "error", "-ss", f"{a.t0:.3f}", "-t", f"{a.t1 - a.t0:.3f}", "-i", a.video,
           "-an", "-vf", vf, "-vsync", "0", out]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stderr[-600:])
        return 1
    n = len([f for f in os.listdir(a.outdir) if f.startswith(a.name + "_")])
    print(a.name, "sheets:", n)
    return 0


if __name__ == "__main__":
    sys.exit(main())
