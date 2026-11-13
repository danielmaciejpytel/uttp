"""Compose the melee demo mp4s from demo_render.py frames: ORIGINAL (game) | CORRECTED (MotionBricks) side by side, 1280x720 H.264.

  python -I demo_compose.py <out_dir> [--crf 24] [--demo-dir DIR] [--name melee_mb_demo_v1] [--suffix _orig_vs_mb] [--tmp DIR]
  --demo-dir: where demo_render.py wrote <set>__<clip>\{3q,side}_{orig,mb} (default Build\...\demo); v2 uses demo2. NEVER point --name / --suffix at an
  existing media file: v1 stays as it is (the script refuses to overwrite an existing output).
Per clip (demo\\<set>__<clip>): 3/4 real time, side real time, 3/4 0.5x, side 0.5x (each with 0.4 s end hold), labels via drawtext
(Arial, UTF-8 text files). Writes <out_dir>\\<clip>_orig_vs_mb.mp4 per clip and melee_mb_demo_v1.mp4 (concat of all clips).
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_common as MC  # noqa: E402
from demo_metrics import CLIPS  # noqa: E402

FF = os.path.join(MC.ROOT, 'Tools', 'pylibs', 'imageio_ffmpeg', 'binaries', 'ffmpeg-win-x86_64-v7.1.exe')
FONT = 'C\\:/Windows/Fonts/arial.ttf'
FONTB = 'C\\:/Windows/Fonts/arialbd.ttf'
TXT = os.path.join(MC.WORK, 'demo', '_txt')
# combo_attack_01 (katana) has no MotionBricks segment: only the corrected + retimed source (offset layer)
MBLABEL = {('pwa_katana', 'combo_attack_01'): 'poprawione (korekta póz, bez MB)'}
VIEWS = {'3q': '3/4 z tyłu-prawej', 'side': 'z boku'}


def tfile(name, text):
    os.makedirs(TXT, exist_ok=True)
    p = os.path.join(TXT, name + '.txt')
    open(p, 'w', encoding='utf-8').write(text)
    return p.replace('\\', '/').replace(':', '\\:')


def seg(d, view, slow, out, clip, crf, setname, mb_label='poprawione (MotionBricks)'):
    fr = 30 if slow else 60
    sel = '' if slow else "select='not(mod(n\\,2))',setpts=N/(30*TB),"
    inputs = []
    for v in ('orig', 'mb'):
        inputs += ['-framerate', str(fr), '-i', os.path.join(d, '%s_%s' % (view, v), 'f_%04d.png')]
    speed = '0,5x (zwolnione)' if slow else 'czas rzeczywisty 30 kl./s'
    t_o, t_m = tfile('orig', 'oryginał (gra)'), tfile('mb_%s_%s' % (setname, clip), mb_label)
    t_c = tfile('c_%s_%s_%d' % (clip, view, slow), '%s / %s   |   widok: %s   |   %s' % (setname, clip, VIEWS[view], speed))
    dt = "drawtext=fontfile='%s':textfile='%s':fontcolor=black:fontsize=%d:x=%s:y=%s:box=1:boxcolor=white@0.7:boxborderw=6"
    vf = ('[0:v]%sfps=30[a];[1:v]%sfps=30[b];[a][b]hstack,tpad=stop_mode=clone:stop_duration=0.4,' % (sel, sel)
          + dt % (FONTB, t_o, 30, '20', '14') + ',' + dt % (FONTB, t_m, 30, '660', '14') + ',' + dt % (FONT, t_c, 24, '20', 'h-44'))
    cmd = [FF, '-y', '-loglevel', 'error'] + inputs + ['-filter_complex', vf, '-c:v', 'libx264', '-crf', str(crf), '-preset', 'slow',
           '-pix_fmt', 'yuv420p', '-r', '30', '-movflags', '+faststart', out]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError(p.stderr[-800:])


def concat(parts, out):
    lst = out + '.list.txt'
    open(lst, 'w', encoding='utf-8').write(''.join("file '%s'\n" % p.replace('\\', '/') for p in parts))
    p = subprocess.run([FF, '-y', '-loglevel', 'error', '-f', 'concat', '-safe', '0', '-i', lst, '-c', 'copy', '-movflags', '+faststart', out],
                       capture_output=True, text=True)
    os.remove(lst)
    if p.returncode:
        raise RuntimeError(p.stderr[-800:])


def label_for(demo_dir, s, c):
    """'poprawione (MotionBricks)' only when a generated span is really in the clip (<demo_dir>\corrected\<set>\<clip>.mb.info.json, gate mode mb)."""
    p = os.path.join(demo_dir, 'corrected', s, c + '.mb.info.json')
    if os.path.exists(p):
        import json
        i = json.load(open(p, encoding='utf-8'))
        if i.get('mb_segments') and i.get('gate', {}).get('mode') != 'offset_fallback':
            return 'poprawione (MB, ramiona z korekty)'
    return 'poprawione (korekta póz, bez MB)'


def main(a):
    out_dir = os.path.abspath(a[0])
    crf = int(a[a.index('--crf') + 1]) if '--crf' in a else 24
    os.makedirs(out_dir, exist_ok=True)
    demo_dir = os.path.abspath(a[a.index('--demo-dir') + 1]) if '--demo-dir' in a else os.path.join(MC.WORK, 'demo')
    name = a[a.index('--name') + 1] if '--name' in a else 'melee_mb_demo_v1'
    suffix = a[a.index('--suffix') + 1] if '--suffix' in a else '_orig_vs_mb'
    final = os.path.join(out_dir, name + '.mp4')
    if os.path.exists(final):
        raise SystemExit('refusing to overwrite existing media file ' + final)
    tmp = os.path.abspath(a[a.index('--tmp') + 1]) if '--tmp' in a else os.path.join(demo_dir, '_segs')
    os.makedirs(tmp, exist_ok=True)
    clip_files = []
    for s, c in CLIPS:
        d = os.path.join(demo_dir, s + '__' + c)
        parts = []
        for i, (view, slow) in enumerate((('3q', 0), ('side', 0), ('3q', 1), ('side', 1))):
            o = os.path.join(tmp, '%s__%s_%d.mp4' % (s, c, i))
            seg(d, view, slow, o, c, crf, s, label_for(demo_dir, s, c) if '--demo-dir' in a else MBLABEL.get((s, c), 'poprawione (MotionBricks)'))
            parts.append(o)
        f = os.path.join(out_dir, '%s__%s%s.mp4' % (s, c, suffix))
        if os.path.exists(f):
            raise SystemExit('refusing to overwrite existing media file ' + f)
        concat(parts, f)
        clip_files.append(f)
        print('CLIP', f, round(os.path.getsize(f) / 1e6, 2), 'MB')
    concat(clip_files, final)
    print('FINAL', final, round(os.path.getsize(final) / 1e6, 2), 'MB')


if __name__ == '__main__':
    main(sys.argv[1:])
