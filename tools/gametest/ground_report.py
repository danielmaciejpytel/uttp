"""Moving platform (elevator) rides and curb / step crossings of one in-game run, from the mod log and the feed dump.

    python ground_report.py <run folder | folder with utpp-*.log and utpp-feed-*.txt> [--json out.json]

Elevator: every "UTPP PLATFORM on" / "UTPP PLATFORM off" pair of the mod log (motionhost::Feed), with the feed rows of the ride
(the feed dump file name is its wall start time; window = the ride +- WINDOW_PAD_S): the clips the motion driver played, the air
details (LocomotionDetailed 14..22) and the ground contact. A ride passes when every platform feed was conditioned (no air detail
while the flag was on), no base clip of the window is a jump / fall / landing clip and no feed row of the window has an air detail.

Curbs: isolated one-frame capsule z steps (a run of 1-3 frames of >= STEP_MIN_M each, calm frames before and after, V moving, on the
ground: a curb or a stair edge, not a slope or a jump), each with the camera evidence the mod logs: the camTrace pivot offset
(pivotZ - V z, 10 Hz) before / after and any "pivot teleport" / "pivot clamp" line within +- 0.5 s (a camera snap on a curb = fail).
The camera's fast mode itself is not logged; the per-frame camera step is not in any log (camTrace is 10 Hz).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from datetime import datetime, timedelta

WINDOW_PAD_S = 1.0
STEP_MIN_M = 0.04
AIR_DETAILS = set(range(14, 23))
AIR_CLIP_RE = re.compile(r'jump|fall|land|air', re.I)
TS_RE = re.compile(r'^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\]')
ON_RE = re.compile(r'UTPP PLATFORM on: onGround=(-?\d+) vz=(-?[\d.]+) detail=(-?\d+)')
OFF_RE = re.compile(r'UTPP PLATFORM off: ([\d.]+) s, feeds=(\d+) airFeeds=(\d+) contactLost=(\d+) vz=(-?[\d.]+)\.\.(-?[\d.]+) conditioned=(\d+)')
CAM_RE = re.compile(r'camTrace t=[\d.]+ .*? pivotZ=(-?[\d.]+)\(([+-][\d.]+)\)')
SNAP_RE = re.compile(r'\[utpp_camera\] pivot (teleport|clamp)')
FEED_NAME_RE = re.compile(r'utpp-feed-(\d{4})-(\d\d)-(\d\d)-(\d\d)-(\d\d)-(\d\d)\.txt$')


def find_files(folder: str) -> tuple[list[str], list[str]]:
    """(mod logs, feed dumps) in folder or folder/logs, sorted by name (= time)."""
    logs, feeds = [], []
    for d in (folder, os.path.join(folder, 'logs')):
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            p = os.path.join(d, f)
            if re.match(r'utpp-\d{4}-.*\.log$', f):
                logs.append(p)
            elif FEED_NAME_RE.search(f):
                feeds.append(p)
    return logs, feeds


def read_log(paths: list[str]) -> dict:
    """PLATFORM rides, camTrace samples (wall time, pivot offset) and camera snap lines of the mod logs."""
    rides, cams, snaps, open_ride = [], [], [], None
    for path in paths:
        with open(path, encoding='utf-8', errors='replace') as f:
            for ln in f:
                m = TS_RE.match(ln)
                if not m:
                    continue
                if 'UTPP PLATFORM' not in ln and 'camTrace' not in ln and 'pivot ' not in ln:
                    continue
                ts = datetime.strptime(m.group(1)[:26], '%Y-%m-%d %H:%M:%S.%f')
                on, off, cam = ON_RE.search(ln), OFF_RE.search(ln), CAM_RE.search(ln)
                if on:
                    open_ride = {'on': ts, 'on_ground': int(on.group(1)), 'on_vz': float(on.group(2)), 'on_detail': int(on.group(3))}
                elif off:
                    r = open_ride or {'on': ts - timedelta(seconds=float(off.group(1)))}
                    r.update(off=ts, seconds=float(off.group(1)), feeds=int(off.group(2)), air_feeds=int(off.group(3)),
                             contact_lost=int(off.group(4)), vz=[float(off.group(5)), float(off.group(6))], conditioned=int(off.group(7)))
                    rides.append(r)
                    open_ride = None
                elif cam:
                    cams.append((ts, float(cam.group(2))))
                elif SNAP_RE.search(ln):
                    snaps.append((ts, ln.split('[utpp_camera] ', 1)[-1].strip()))
    if open_ride:
        open_ride['off'] = None
        rides.append(open_ride)
    return {'rides': rides, 'cams': cams, 'snaps': snaps}


def feed_start(path: str) -> datetime | None:
    m = FEED_NAME_RE.search(os.path.basename(path))
    return datetime(*(int(v) for v in m.groups())) if m else None


def read_feed(path: str) -> list[dict]:
    rows, cols = [], None
    with open(path, encoding='utf-8', errors='replace') as f:
        for ln in f:
            if ln.startswith('# cols'):
                cols = ln.split()[2:]
                continue
            if ln.startswith('#') or cols is None:
                continue
            p = ln.split()
            if len(p) < len(cols):
                continue
            r = dict(zip(cols, p))
            try:
                rows.append({'t': float(r['t']), 'x': float(r['capX']), 'y': float(r['capY']), 'z': float(r['capZ']),
                             'v': math.hypot(float(r['vx']), float(r['vy'])), 'clip': r.get('baseClip', '-'),
                             'det': int(r['detailed']) if r.get('detailed', '-').lstrip('-').isdigit() else None,
                             'ground': r.get('onGround', '-')})
            except (KeyError, ValueError):
                continue
    return rows


def ride_verdicts(rides: list[dict], feeds: list[tuple[datetime, list[dict]]]) -> list[dict]:
    out = []
    for r in rides:
        rec = {k: (v.strftime('%H:%M:%S.%f')[:-3] if isinstance(v, datetime) else v) for k, v in r.items()}
        fails = []
        if r.get('off') is None:
            fails.append('no PLATFORM off line (ride not finished in the log)')
        elif r['conditioned'] < r['feeds']:
            fails.append(f"{r['feeds'] - r['conditioned']} platform feed(s) not conditioned (an air detail while the flag was on)")
        lo = r['on'] - timedelta(seconds=WINDOW_PAD_S)
        hi = (r.get('off') or r['on']) + timedelta(seconds=WINDOW_PAD_S)
        clips, air_rows, lost, n = set(), 0, 0, 0
        for start, rows in feeds:
            for row in rows:
                ts = start + timedelta(seconds=row['t'])
                if lo <= ts <= hi:
                    n += 1
                    clips.add(row['clip'])
                    air_rows += 1 if row['det'] in AIR_DETAILS else 0
                    lost += 1 if row['ground'] == '0' else 0
        air_clips = sorted(c for c in clips if AIR_CLIP_RE.search(c))
        if n == 0:
            fails.append('no feed rows in the ride window (feed dump off or not copied)')
        if air_clips:
            fails.append('jump / fall / landing clip(s): ' + ', '.join(air_clips))
        if air_rows:
            fails.append(f'{air_rows} feed row(s) with an air detail (14..22)')
        rec.update(window_rows=n, clips=sorted(clips), feed_ground_lost_rows=lost, verdict='PASS' if not fails else 'FAIL', fails=fails)
        out.append(rec)
    return out


def curb_steps(rows: list[dict]) -> list[dict]:
    """Isolated capsule z steps on the ground while V moves (curb / stair edge)."""
    out, n, i = [], len(rows), 1
    dz = [0.0] + [rows[k]['z'] - rows[k - 1]['z'] for k in range(1, n)]
    dxy = [0.0] + [math.hypot(rows[k]['x'] - rows[k - 1]['x'], rows[k]['y'] - rows[k - 1]['y']) for k in range(1, n)]
    while i < n - 4:
        if abs(dz[i]) >= STEP_MIN_M and dxy[i] < 0.5:
            j = i
            while j < n and abs(dz[j]) >= STEP_MIN_M and j - i < 6:
                j += 1
            calm = all(abs(dz[k]) < 0.02 for k in range(max(1, i - 3), i)) and all(abs(dz[k]) < 0.02 for k in range(j, min(n, j + 3)))
            air = any(rows[k]['det'] in AIR_DETAILS for k in range(max(0, i - 5), min(n, j + 5)))
            if j - i <= 3 and calm and not air and rows[i]['v'] > 0.5:
                out.append({'t': rows[i]['t'], 'frames': j - i, 'dz_m': round(sum(dz[i:j]), 3), 'max_frame_dz_m': round(max(dz[i:j], key=abs), 3),
                            'pos': [round(rows[i]['x'], 2), round(rows[i]['y'], 2), round(rows[i]['z'], 2)], 'speed': round(rows[i]['v'], 2)})
            i = j
        else:
            i += 1
    return out


def camera_around(step_ts: datetime, cams, snaps) -> dict:
    before = [o for ts, o in cams if step_ts - timedelta(seconds=0.6) <= ts < step_ts]
    after = [o for ts, o in cams if step_ts <= ts <= step_ts + timedelta(seconds=0.4)]
    near = [s for ts, s in snaps if abs((ts - step_ts).total_seconds()) <= 0.5]
    return {'offset_before': before[-1] if before else None, 'offsets_after': after, 'camera_snaps': near}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('folder')
    ap.add_argument('--json', default='')
    args = ap.parse_args(argv)
    logs, feed_paths = find_files(args.folder)
    if not logs:
        print(f'no utpp-*.log in {args.folder}')
        return 2
    lg = read_log(logs)
    feeds = [(feed_start(p), read_feed(p)) for p in feed_paths]
    feeds = [(s, r) for s, r in feeds if s is not None]
    rides = ride_verdicts(lg['rides'], feeds)
    steps = []
    for start, rows in feeds:
        for st in curb_steps(rows):
            ts = start + timedelta(seconds=st['t'])
            st['wall'] = ts.strftime('%H:%M:%S.%f')[:-3]
            st.update(camera_around(ts, lg['cams'], lg['snaps']))
            st['verdict'] = 'FAIL' if st['camera_snaps'] else 'ok'
            steps.append(st)
    print(f'logs {len(logs)}, feed dumps {len(feeds)}, camTrace samples {len(lg["cams"])}')
    print(f'PLATFORM rides: {len(rides)}')
    for r in rides:
        print(f"  {r['verdict']} on {r.get('on')} off {r.get('off')} {r.get('seconds', '-')} s feeds={r.get('feeds', '-')} "
              f"airFeeds={r.get('air_feeds', '-')} contactLost={r.get('contact_lost', '-')} conditioned={r.get('conditioned', '-')} "
              f"window rows={r['window_rows']} clips={','.join(r['clips'][:6])}" + (' | ' + '; '.join(r['fails']) if r['fails'] else ''))
    print(f'curb / step crossings: {len(steps)}')
    for s in steps:
        after = ','.join('%+.2f' % o for o in s['offsets_after']) or '-'
        before = '-' if s['offset_before'] is None else '%+.2f' % s['offset_before']
        print(f"  {s['verdict']} {s['wall']} dz={s['dz_m']:+.3f} m in {s['frames']} frame(s) at {s['pos']} v={s['speed']} m/s; "
              f"camera pivot offset {before} -> {after}" + (' | ' + '; '.join(s['camera_snaps']) if s['camera_snaps'] else ''))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump({'rides': rides, 'steps': steps}, f, indent=1, default=str)
    bad = any(r['verdict'] == 'FAIL' for r in rides) or any(s['verdict'] == 'FAIL' for s in steps)
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
