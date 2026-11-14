"""Cuts the recorded fixtures of the fists chest base correction for ONE body from a pose recording and its log.

Output (plugin/UTPP/tests/data, read by plugin/UTPP/tests/MeleeBaseGlueTest.cpp through tests/MeleeBaseReplay.hpp):
  melee_base_<female|male>.bin        "MBO2": one standing combo window + one walking window (the two windows of the glue test), the
                                      engine records of every frame, the MM pose of the lower bones, the mask / lower-body weights, and per
                                      frame the clip elevation of the striking arm at the matched time (refElev, NaN = no reference)
  melee_base_blend_<body>.bin         "MBO3": the first frames after the melee upper-body mask starts in two fresh windows (the 0.15 s blend-in
                                      and the first punches), see fist_base_blend_fixture.py for the blend model

  python -I fist_base_fixture.py --pose utpp-pose-YYYY-MM-DD-HH-MM-SS.utpppose --utpp-log utpp-YYYY-MM-DD-HH-MM-SS.log --body F|M
         --settle HH:MM:SS.s-HH:MM:SS.s --stand-at HH:MM:SS.s --walk-at HH:MM:SS.s --blend-at HH:MM:SS.s,HH:MM:SS.s
         [--stand-frames 110] [--walk-frames 60] [--blend-frames 36] [--data-dir DIR] [--exe EXE] [--scratch DIR] [--dry-run]

--pose / --utpp-log   file names in <game>/red4ext/logs, or full paths. The log of the SAME session as the recording.
--settle              a stretch of the recording with the character standing still on the MM idle and no punch (the MM pose of the lower
                      bones for the standing window; the recording shows the engine's pose there)
--stand-at            the mask-on time (log clock, local time) of the standing window; the window starts 0.1 s before it and holds
                      --stand-frames frames. It must start with a full-body mask (a standing combo); the third punch is a late part of it
--walk-at             the mask-on time of the walking window (mask without the full body), --walk-frames frames
--blend-at            the mask-on times of the two blend windows (fresh melee masks, a full-body start each)
--data-dir            where the four files are written (default: the tests/data folder of this source tree); nothing is written before
                      every file is computed (a failure leaves the old files)
--exe                 utpp_melee_base_offline built from the CURRENT sources: ninja -C <build> utpp_melee_base_offline
--dry-run             compute and print the per-clip table, write nothing
The mask-on times are the "MMMASK on" lines of the log: any mask reason for stand / walk, melee=1 for the blend windows; list them with --list-windows.
A blend window must be a fresh melee mask: do not pick a start that is in the melee=1 list but not in the any-reason list (the mask was already up
for another reason, there is no blend-in to replay).
Both bodies are cut the same way; run it once per body with that body's recording (--body F writes the female files, --body M the male files).

The reference elevation comes from the clip files the compiled table was generated from (fist_base_offline.CLIPS_IN); the run stops when a
clip file does not match the SHA-256 prefix recorded in the header of src/mm/MeleeBaseData.cpp, so a reference can never come from a different
clip than the one the table matched. The printed table gives, per clip name, the frames matched, the frames with a reference and the elevation
error against the clip (p95 / max, degrees) of the engine pose (before) and of the corrected pose (after).
"""
import argparse
import glob
import hashlib
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import fist_base_blend_fixture as fbf  # noqa: E402
import fist_base_offline as fbo  # noqa: E402
from utpp_paths import game_dir  # noqa: E402

SOURCE = os.path.abspath(os.path.join(HERE, '..', '..'))
TABLE_CPP = os.path.join(SOURCE, 'plugin', 'UTPP', 'src', 'mm', 'MeleeBaseData.cpp')
DATA_DIR = os.path.join(SOURCE, 'plugin', 'UTPP', 'tests', 'data')
BODY_WORD = {'F': 'female', 'M': 'male'}
CLIP_PREFIX = {'F': 'pwa_fists', 'M': 'pma_fists'}
WINDOW_TOLERANCE = 0.05


def parse_clock(text):
    """HH:MM:SS[.fff] -> seconds of the day"""
    m = re.fullmatch(r'(\d{1,2}):(\d{2}):(\d{2}(?:\.\d+)?)', text.strip())
    if not m:
        raise ValueError('bad time %r (expected HH:MM:SS[.fff])' % text)
    return int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3))


def parse_range(text):
    a, b = text.split('-')
    return parse_clock(a), parse_clock(b)


def find_window(windows, at, tol=WINDOW_TOLERANCE):
    """index of the (on, off) window that starts at the time 'at' (+- tol); raises when there is none or more than one"""
    hits = [i for i, w in enumerate(windows) if abs(w[0] - at) <= tol]
    if len(hits) != 1:
        raise ValueError('%d windows start at %.3f s (+-%.0f ms); the log has: %s' %
                         (len(hits), at, tol * 1000, ', '.join('%.3f' % w[0] for w in windows) or 'none'))
    return hits[0]


def table_inputs(cpp_path):
    """[(sha-256 prefix, file name)] of the header of the generated clip table"""
    out = []
    with open(cpp_path, encoding='utf-8') as fh:
        for ln in fh:
            m = re.match(r'//\s+([0-9a-f]{16})\s+(\S+\.npz)\s*$', ln)
            if m:
                out.append((m.group(1), m.group(2)))
            elif not ln.startswith('//'):
                break
    return out


def check_clip_inputs(cpp_path, clips_in):
    """every clip file the table was generated from must be in clips_in with the recorded SHA-256 prefix"""
    listed = table_inputs(cpp_path)
    if not listed:
        raise RuntimeError('no clip digests in the header of ' + cpp_path)
    for digest, name in listed:
        path = os.path.join(clips_in, name)
        if not os.path.exists(path):
            raise RuntimeError('clip file %s of the table is missing in %s' % (name, clips_in))
        with open(path, 'rb') as fh:
            got = hashlib.sha256(fh.read()).hexdigest()
        if not got.startswith(digest):
            raise RuntimeError('clip file %s differs from the one the table was generated from (%s vs %s): the references would not match the '
                               'table; point fist_base_offline.CLIPS_IN at the right clip set' % (name, got[:16], digest))
    return len(listed)


def pct(values, p):
    """the percentile of the glue test (MeleeBaseGlueTest.cpp Pct): sorted[floor(p * (n - 1))], no interpolation"""
    v = sorted(values)
    return v[int(p * (len(v) - 1))] if v else 0.0


def clip_table(frames, rows):
    """per clip name: frames matched, frames with a reference, p95 / max elevation error before and after (degrees)"""
    per = {}
    for fr, row in zip(frames, rows):
        if row['cname'] == '-':
            continue
        d = per.setdefault(row['cname'], dict(n=0, ref=[], after=[], before=[]))
        d['n'] += 1
        if not np.isnan(fr['refElev']):
            d['ref'].append(fr['refElev'])
            d['after'].append(abs(row['e1'] - fr['refElev']))
            d['before'].append(abs(row['e0'] - fr['refElev']))
    return per


def format_clip_table(frames, rows, title):
    per = clip_table(frames, rows)
    lines = ['%s: %d frames, %d with a reference' % (title, len(frames), sum(1 for f in frames if not np.isnan(f['refElev']))),
             '  %-34s %6s %5s %18s %18s' % ('clip', 'frames', 'refs', 'after p95 / max', 'before p95 / max')]
    for name, d in sorted(per.items(), key=lambda kv: -kv[1]['n']):
        if d['ref']:
            a, b = np.array(d['after']), np.array(d['before'])
            tail = '%8.1f / %6.1f %8.1f / %6.1f' % (pct(a, 0.95), a.max(), pct(b, 0.95), b.max())
        else:
            tail = '%18s %18s' % ('-', '-')
        lines.append('  %-34s %6d %5d %s' % (name, d['n'], len(d['ref']), tail))
    allr = [(fr, row, i) for i, (fr, row) in enumerate(zip(frames, rows)) if not np.isnan(fr['refElev'])]
    if allr:
        a = np.array([abs(row['e1'] - fr['refElev']) for fr, row, _ in allr])
        b = np.array([abs(row['e0'] - fr['refElev']) for fr, row, _ in allr])
        lines.append('  %-34s %6s %5d %8.1f / %6.1f %8.1f / %6.1f' % ('(all)', '', len(allr), pct(a, 0.95), a.max(), pct(b, 0.95), b.max()))
        worst = sorted(zip(a, range(len(allr))), reverse=True)[:4]
        lines.append('  largest errors after (deg; frame of the window, seconds since the mask start, clip, limit flags of the frame, bit 0 = forward bend limit):')
        for err, k in worst:
            fr, row, idx = allr[k]
            lines.append('    %5.1f  frame %3d  +%.2f s  %-22s flags=%d' % (err, idx, fr['since'], row['cname'], int(row['so'][18])))
    return '\n'.join(lines)


def set_references(frames, rows, since_min=None):
    """side and refElev of every frame from the replay of the current code: the clip elevation of the striking arm at the matched time, for the
    frames whose arm is stretched (>= 45 cm); since_min: only frames at least that many seconds after the mask start (the blend is over)"""
    for fr, row in zip(frames, rows):
        fr['side'] = 0 if row['side'] == 'Right' else 1
        ok = row['ec'] is not None and row['ext'] >= 45.0 and (since_min is None or fr['since'] >= since_min)
        fr['refElev'] = row['ec'] if ok else float('nan')


def cut_standing_walking(rec, log, body, stand_at, walk_at, stand_frames, walk_frames, exe, scratch):
    """-> (frames, lower, rows, window starts): the two windows of the glue test with their references"""
    # the replay frames are cut by fbo.build_frames from the steps of the recording; these steps just bracket the picked windows
    rec.steps['punches'] = (stand_at, stand_at + 120.0)
    rec.steps['punch_walk'] = (walk_at, walk_at + 120.0)
    frames, lower = fbo.build_frames(rec, log, ('punches', 'punch_walk'), stand_frames, walk_frames,
                                     pick={'punches': stand_at, 'punch_walk': walk_at})
    starts = [f['t'] - f['since'] for f in frames if f['dt'] < 0]
    if len(starts) != 2:
        raise RuntimeError('expected 2 windows (standing + walking), cut %d' % len(starts))
    os.makedirs(scratch, exist_ok=True)
    inp, outp = os.path.join(scratch, 'stand_walk_in.bin'), os.path.join(scratch, 'stand_walk_out.bin')
    fbo.write_mbo2(inp, rec, body, frames, lower)
    res, clipnames = fbo.run_exe(exe, inp, outp)
    rows = fbo.analyse(rec, body, frames, res, clipnames, fbo.load_clip_table(body))
    set_references(frames, rows)
    return frames, lower, rows


def cut_blend(rec, log, body, blend_at, nframes, exe, scratch):
    mask, _ = fbf.melee_windows(log)
    wins = [find_window(mask, t) for t in blend_at]
    frames, lower = fbf.build_frames(rec, log, wins, nframes)
    os.makedirs(scratch, exist_ok=True)
    inp, outp = os.path.join(scratch, 'blend_in.bin'), os.path.join(scratch, 'blend_out.bin')
    fbf.write_mbo3(inp, rec, body, frames, lower, fbf.UPPER_BLEND, subset=False)
    res, clipnames = fbo.run_exe(exe, inp, outp)
    rows = fbo.analyse(rec, body, frames, res, clipnames, fbo.load_clip_table(body))
    set_references(frames, rows, since_min=fbf.REF_FROM)
    return frames, lower, rows


def resolve(name):
    if os.path.isabs(name) or os.path.exists(name):
        return os.path.abspath(name)
    return os.path.join(game_dir(), 'red4ext', 'logs', name)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pose', required=True)
    ap.add_argument('--utpp-log', required=True)
    ap.add_argument('--body', required=True, choices=['F', 'M'])
    ap.add_argument('--settle', help='HH:MM:SS.s-HH:MM:SS.s: standing still on the MM idle, no punch')
    ap.add_argument('--stand-at', help='mask-on time of the standing combo window')
    ap.add_argument('--walk-at', help='mask-on time of the walking window')
    ap.add_argument('--blend-at', help='mask-on times (melee=1) of the two blend windows, comma separated')
    ap.add_argument('--stand-frames', type=int, default=110)
    ap.add_argument('--walk-frames', type=int, default=60)
    ap.add_argument('--blend-frames', type=int, default=36)
    ap.add_argument('--data-dir', default=DATA_DIR)
    ap.add_argument('--exe', default=fbo.EXE)
    ap.add_argument('--scratch', default=os.path.join(os.getcwd(), '_tmp'))
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--list-windows', action='store_true', help='print the mask / full-body windows of the log and stop')
    a = ap.parse_args(argv)

    log = resolve(a.utpp_log)
    if a.list_windows:
        for title, ws in (('MMMASK (any reason)', fbo.windows(log, 'MMMASK')), ('MMMASK melee=1', fbf.melee_windows(log)[0]),
                          ('MMMELEEFULL', fbo.windows(log, 'MMMELEEFULL'))):
            print(title)
            for w in ws:
                print('  %s - %s' % (fmt_clock(w[0]), fmt_clock(w[1])))
        return 0
    for opt in ('settle', 'stand_at', 'walk_at', 'blend_at'):
        if getattr(a, opt) is None:
            ap.error('--%s is required' % opt.replace('_', '-'))
    if not os.path.exists(a.exe):
        ap.error('replay exe not found: %s (build utpp_melee_base_offline from the current sources)' % a.exe)
    n_clips = check_clip_inputs(TABLE_CPP, fbo.CLIPS_IN)
    print('clip inputs: %d files match the table header' % n_clips)
    os.makedirs(a.scratch, exist_ok=True)
    settle = parse_range(a.settle)
    stand_at, walk_at = parse_clock(a.stand_at), parse_clock(a.walk_at)
    blend_at = [parse_clock(x) for x in a.blend_at.split(',')]
    if len(blend_at) != 2:
        ap.error('--blend-at takes two times')

    rec = fbo.Recording(resolve(a.pose), None, {'settle': settle})
    body = a.body
    sw_frames, sw_lower, sw_rows = cut_standing_walking(rec, log, body, stand_at, walk_at, a.stand_frames, a.walk_frames, a.exe, a.scratch)
    bl_frames, bl_lower, bl_rows = cut_blend(rec, log, body, blend_at, a.blend_frames, a.exe, a.scratch)
    print(format_clip_table(sw_frames, sw_rows, 'standing + walking window'))
    print(format_clip_table(bl_frames, bl_rows, 'blend windows'))
    if not a.dry_run:
        os.makedirs(a.data_dir, exist_ok=True)
        word = BODY_WORD[body]
        p_sw = os.path.join(a.data_dir, 'melee_base_%s.bin' % word)
        p_bl = os.path.join(a.data_dir, 'melee_base_blend_%s.bin' % word)
        fbo.write_mbo2(p_sw, rec, body, sw_frames, sw_lower, subset=True)
        fbf.write_mbo3(p_bl, rec, body, bl_frames, bl_lower, fbf.UPPER_BLEND, subset=True)
        for p in (p_sw, p_bl):
            print('wrote', p, os.path.getsize(p), 'bytes')
    for p in glob.glob(os.path.join(a.scratch, '*')):
        if os.path.basename(p).startswith(('stand_walk_', 'blend_')):
            os.remove(p)
    try:
        os.rmdir(a.scratch)
    except OSError:
        pass
    return 0


def fmt_clock(sec):
    s = sec % 86400.0
    return '%02d:%02d:%06.3f' % (int(s // 3600), int(s % 3600 // 60), s % 60)


if __name__ == '__main__':
    sys.exit(main())
