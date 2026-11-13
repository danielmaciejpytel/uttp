"""Compose the melee redo demo (v3): three panels BASE (V idle, static) | ORIGINAL (game) | REDONE, 1272x720 H.264, per clip 3/4 + side, real time + 0.5x.

  python -I redo_compose.py <out_dir> --demo-dir DIR --name melee_redo_demo_v3 [--crf 24] [--tmp DIR]
Needs <demo-dir>\\report.json (redo_demo.py: frames kept from the game / redone per clip) and the rendered {3q,side}_{base,orig,redo}\\f_####.png.
Refuses to overwrite existing media files.  Writes <out_dir>\\<set>__<clip>_redo_v3.mp4 per clip and <name>.mp4 (concat).
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_common as MC  # noqa: E402
from demo_metrics import CLIPS  # noqa: E402
from demo_compose import FF, FONT, FONTB, concat  # noqa: E402

VIEWS = {'3q': '3/4 z tyłu-prawej', 'side': 'z boku'}


def tfile(d, name, text):
    os.makedirs(d, exist_ok=True)
    p = os.path.join(d, name + '.txt')
    open(p, 'w', encoding='utf-8').write(text)
    return p.replace('\\', '/').replace(':', '\\:')


def seg(d, view, slow, out, s, c, crf, txt, rep):
    fr = 30 if slow else 60
    sel = '' if slow else "select='not(mod(n\\,2))',setpts=N/(30*TB),"
    inputs = []
    for v in ('base', 'orig', 'redo'):
        inputs += ['-framerate', str(fr), '-i', os.path.join(d, '%s_%s' % (view, v), 'f_%04d.png')]
    speed = '0,5x (zwolnione)' if slow else 'czas rzeczywisty 30 kl./s'
    t1 = tfile(txt, 'p1', 'baza: V idle (statyczna)')
    t2 = tfile(txt, 'p2', 'oryginał (gra)')
    kept = rep['game_frames_kept'] + rep['game_frames_blended']
    t3 = tfile(txt, 'p3', 'przerobione na ciało bazowe V')
    t4 = tfile(txt, 'c_%s_%s_%s_%d' % (s, c, view, slow), '%s / %s | %s | %s | klatki: z gry %d, przejścia %d, przerobione %d z %d' % (
        s, c, VIEWS[view], speed, rep['game_frames_kept'], rep['game_frames_blended'], rep['redone_frames'], rep['frames']))
    dt = "drawtext=fontfile='%s':textfile='%s':fontcolor=black:fontsize=%d:x=%s:y=%s:box=1:boxcolor=white@0.7:boxborderw=5"
    vf = ('[0:v]%sfps=30[a];[1:v]%sfps=30[b];[2:v]%sfps=30[c];[a][b][c]hstack=inputs=3,tpad=stop_mode=clone:stop_duration=0.4,' % (sel, sel, sel)
          + dt % (FONTB, t1, 22, '10', '10') + ',' + dt % (FONTB, t2, 22, '434', '10') + ',' + dt % (FONTB, t3, 22, '858', '10') + ','
          + dt % (FONT, t4, 17, '10', 'h-34'))
    cmd = [FF, '-y', '-loglevel', 'error'] + inputs + ['-filter_complex', vf, '-c:v', 'libx264', '-crf', str(crf), '-preset', 'slow',
           '-pix_fmt', 'yuv420p', '-r', '30', '-movflags', '+faststart', out]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError(p.stderr[-800:])


def main(a):
    out_dir = os.path.abspath(a[0])
    demo = os.path.abspath(a[a.index('--demo-dir') + 1])
    name = a[a.index('--name') + 1]
    suffix = a[a.index('--suffix') + 1] if '--suffix' in a else '_redo_v3'
    crf = int(a[a.index('--crf') + 1]) if '--crf' in a else 24
    tmp = os.path.abspath(a[a.index('--tmp') + 1]) if '--tmp' in a else os.path.join(demo, '_segs')
    os.makedirs(tmp, exist_ok=True)
    rep = json.load(open(os.path.join(demo, 'report.json'), encoding='utf-8'))
    final = os.path.join(out_dir, name + '.mp4')
    if os.path.exists(final):
        raise SystemExit('refusing to overwrite ' + final)
    files = []
    for s, c in CLIPS:
        d = os.path.join(demo, s + '__' + c)
        parts = []
        for i, (view, slow) in enumerate((('3q', 0), ('side', 0), ('3q', 1), ('side', 1))):
            o = os.path.join(tmp, '%s__%s_%d.mp4' % (s, c, i))
            seg(d, view, slow, o, s, c, crf, os.path.join(tmp, '_txt'), rep['%s/%s' % (s, c)])
            parts.append(o)
        f = os.path.join(out_dir, '%s__%s%s.mp4' % (s, c, suffix))
        if os.path.exists(f):
            raise SystemExit('refusing to overwrite ' + f)
        concat(parts, f)
        files.append(f)
        print('CLIP', f, round(os.path.getsize(f) / 1e6, 2), 'MB')
    concat(files, final)
    print('FINAL', final, round(os.path.getsize(final) / 1e6, 2), 'MB')


if __name__ == '__main__':
    main(sys.argv[1:])
