"""Offline replay of the head look layer on recorded in-game poses.

  export  POSE.utpppose RUN.log --window HH:MM:SS HH:MM:SS --out fixture.txt [--stride N] [--sweep] [--weapon]
          Reads the compact pose recording (stage 1 of stream 0: the local pose after the motion blend, the aim offset and the melee
          tilt, i.e. the pose the layer receives), the head look decisions ("head look apply|clear (mode)") and the camera trace
          ("camTrace ...") of the run log, and writes the fixture of tests/HeadLookReplay.cpp.
  figure  replay.csv --out figure.png [--title TEXT]
          Draws the CSV the replay executable writes: the camera / gaze yaw, the head yaw before and after the layer, the chest,
          the eyes.
  run     POSE RUN.log --window A B --exe utpp_headlook_replay.exe --png figure.png [--csv out.csv] [--fixture out.txt]
          export + replay + figure in one go.

The target of every frame is the direction the camera defines, seen from the head of the recorded pose: camera mode = the camera
look direction relative to the body (camTrace body / out / pitch, the pitch through the dead band of the resting framing), face mode = towards the camera (the opposite bearing, 3 m);
frames outside a look (state off) carry weight 0. --sweep replaces the camera of the run by a scripted orbit (the recorded runs hold
the camera behind V almost all the time): off 0.8 s, camera yaw sweep +-95 deg with a pitch wobble 8 s, hold 1.5 s, face mode 2 s with the
camera in front, released for the rest of the window. --weapon: no chest share (a weapon is in the hands). Frames use the recording's own timestamps, so the replay sees the real frame
timing. The recording was made with the head look off in the DLL, so stage 1 is the pose without the layer.
"""
import argparse
import bisect
import datetime
import os
import re
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pose_helper_report as ph  # noqa: E402

# Player rig bones 0..32 (names and parents of the player skeleton; the head chain and the eyes are what the layer needs).
RIG_NAMES = ["Root", "Trajectory", "Hips", "reference_joint", "Spine", "LeftUpLeg", "RightUpLeg", "Spine1", "LeftLeg", "RightLeg",
             "Spine2", "LeftFoot", "RightFoot", "Spine3", "LeftShoulder", "RightShoulder", "Neck", "LeftArm", "RightArm", "Neck1",
             "LeftForeArm", "RightForeArm", "Head", "LeftHand", "RightHand", "LeftHandEnd", "WeaponLeft", "RightHandEnd", "WeaponRight",
             "LeftHeel", "RightHeel", "LeftEye", "RightEye"]
RIG_PARENTS = [-1, 0, 0, 0, 2, 2, 2, 4, 5, 6, 7, 8, 9, 10, 13, 13, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 23, 24, 24, 11, 12, 22, 22]
CHAIN = ["Hips", "Spine", "Spine1", "Spine2", "Spine3", "Neck", "Neck1", "Head", "LeftEye", "RightEye"]


def sec_of(text):
    t = datetime.datetime.strptime(text, '%H:%M:%S.%f' if '.' in text else '%H:%M:%S')
    return t.hour * 3600 + t.minute * 60 + t.second + t.microsecond / 1e6


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def read_decisions(log):
    """-> [(wall seconds, 'apply'|'clear', mode)] from the run log."""
    out = []
    rx = re.compile(r'\[\d{4}-\d\d-\d\d (\d\d:\d\d:\d\d\.\d+)\].*\[utpp_core\] t=([\d.]+) head look (apply|clear) \((\w+)\)')
    for line in open(log, encoding='utf-8', errors='replace'):
        m = rx.match(line)
        if m:
            out.append((sec_of(m.group(1)), m.group(3), m.group(4)))
    return out


def read_camera(log):
    """-> arrays (wall seconds, body yaw, camera yaw, camera pitch) from the camTrace lines (10 Hz)."""
    rx = re.compile(r'\[\d{4}-\d\d-\d\d (\d\d:\d\d:\d\d\.\d+)\].*camTrace t=([\d.]+) body=(-?[\d.]+) out=(-?[\d.]+) pitch=(-?[\d.]+)')
    rows = []
    for line in open(log, encoding='utf-8', errors='replace'):
        if 'camTrace' in line:
            m = rx.match(line)
            if m:
                rows.append((sec_of(m.group(1)), float(m.group(3)), float(m.group(4)), float(m.group(5))))
    a = np.array(rows)
    return a[:, 0], a[:, 1], a[:, 2], a[:, 3]


def state_at(events, t):
    state = 'off'
    for ts, op, mode in events:
        if ts <= t:
            state = mode if op == 'apply' else 'off'
    return state


NEUTRAL_PITCH = 15.0  # deg, utpp_core HL.DEFAULTS.neutralPitch: the dead band [-15, 0] of the camera pitch (camera mode only)


def framing_pitch(camera_pitch):
    """The pitch of the camera mode target after the dead band of utpp_core (mirror of HL.Target)."""
    return camera_pitch - min(max(camera_pitch, -NEUTRAL_PITCH), 0.0)


def sweep_target(t):
    """Scripted camera for --sweep, t = seconds from the start of the window -> (weight, mode, yaw, pitch, dist)."""
    if t < 0.8:
        return 0.0, 0, 0.0, 0.0, 5.0
    if t < 8.8:
        u = t - 0.8
        return 1.0, 0, 95.0 * np.sin(2.0 * np.pi * u / 8.0), -8.0 + 28.0 * np.sin(2.0 * np.pi * u / 5.0), 5.0 # camera pitch -36 .. +20
    if t < 10.3:
        return 1.0, 0, 0.0, -10.0, 5.0
    if t < 12.3:
        u = t - 10.3
        return 1.0, 1, 25.0 * np.sin(2.0 * np.pi * u / 2.0), 6.0, 3.0
    return 0.0, 0, 0.0, 0.0, 5.0


def export(pose, log, window, out, stride=1, sweep=False, weapon=False):
    cfg = ph.Config()
    group = ph.load_groups([pose], cfg)[0]
    header = group['header']
    mapped = header.mapped()
    names = [header.names[i] for i in mapped]
    column = {n: k for k, n in enumerate(names)}
    gindex = {header.names[i]: header.game_index[i] for i in mapped}
    for need in CHAIN:
        if need not in column:
            raise SystemExit('bone %s is not in the recording' % need)
        if RIG_NAMES[gindex[need]] != need:
            raise SystemExit('bone %s has game index %d, expected %s' % (need, gindex[need], RIG_NAMES[gindex[need]]))
    w0, w1 = sec_of(window[0]), sec_of(window[1])
    events = read_decisions(log) if not sweep else []
    ct, body, cam, pitch = read_camera(log) if not sweep else (np.array([0.0, 1.0]), None, None, np.array([0.0, 0.0]))
    camrel = np.degrees(np.unwrap(np.radians(np.array([wrap(c - b) for c, b in zip(cam, body)])))) if not sweep else np.array([0.0, 0.0])
    frames = []
    kept = 0
    for stream, ticks, sec, gidx, values in ph.iter_blocks(group):
        if stream != 0:
            continue
        for k in range(len(sec)):
            if not (w0 <= sec[k] <= w1):
                continue
            kept += 1
            if (kept - 1) % stride:
                continue
            t = float(sec[k])
            state = state_at(events, t)
            yaw = float(np.interp(t, ct, camrel))
            pit = float(np.interp(t, ct, pitch))
            if sweep:
                weight, mode, ty, tp, dist = sweep_target(t - w0)
                if mode == 0 and weight > 0:
                    tp = framing_pitch(tp)
            elif state == 'face':
                # the camera is in front of V and looks at her: the camera lies in the opposite bearing, a little above the head
                weight, mode, ty, tp, dist = 1.0, 1, wrap(yaw + 180.0), -pit, 3.0
            elif state in ('camera', 'loot'):
                weight, mode, ty, tp, dist = 1.0, 0 if state == 'camera' else 2, wrap(yaw), framing_pitch(pit) if state == 'camera' else pit, 5.0
            else:
                weight, mode, ty, tp, dist = 0.0, 0, 0.0, 0.0, 5.0
            rec = [values[k, column[n], 1, [0, 1, 2, 4, 5, 6, 7]] for n in CHAIN]
            frames.append((t, weight, mode, ty, tp, dist, rec))
    if not frames:
        raise SystemExit('no frames in the window')
    t0 = frames[0][0]
    with open(out, 'w', newline='\n') as f:
        f.write('rig %d\n' % len(RIG_NAMES))
        for i, name in enumerate(RIG_NAMES):
            f.write('bone %d %s %d %d\n' % (i, name, RIG_PARENTS[i], 1 if name in CHAIN else 0))
        order = sorted(CHAIN, key=lambda n: RIG_NAMES.index(n))
        f.write('frames %d\n' % len(frames))
        for t, weight, mode, ty, tp, dist, rec in frames:
            recd = {n: r for n, r in zip(CHAIN, rec)}
            nums = []
            for n in order:
                nums.extend('%.6g' % float(x) for x in recd[n])
            f.write('frame %.4f %g %d %d %.3f %.3f %g %s\n' % (t - t0, weight, mode, 0 if weapon else 1, ty, tp, dist, ' '.join(nums)))
    on = sum(1 for fr in frames if fr[1] > 0)
    print('exported %d frames (%d with a look) to %s' % (len(frames), on, out))


def figure(csv, out, title=''):
    from PIL import Image, ImageDraw, ImageFont
    rows = [ln.strip().split(',') for ln in open(csv) if ln.strip()]
    head, data = rows[0], np.array([[float(x) for x in r] for r in rows[1:]])
    col = {n: data[:, i] for i, n in enumerate(head)}
    t = col['t']
    W, H = 1500, 1120
    img = Image.new('RGB', (W, H), 'white')
    d = ImageDraw.Draw(img)
    font = ImageFont.truetype(os.path.join(os.environ.get('WINDIR', ''), 'Fonts', 'arial.ttf'), 15)
    fsm = ImageFont.truetype(os.path.join(os.environ.get('WINDIR', ''), 'Fonts', 'arial.ttf'), 13)
    X0, X1 = 150, W - 20
    span = max(t[-1] - t[0], 1e-6)

    def px(v):
        return X0 + (X1 - X0) * (v - t[0]) / span

    shade = {0: (242, 242, 242), 1: (207, 232, 255)}
    blue, red, green, gray = (30, 90, 200), (210, 40, 40), (0, 140, 60), (110, 110, 110)
    panels = [
        ('target direction (gaze) yaw (deg)', [(col['gaze_yaw'], (0, 0, 0), 'gaze (clamped)')]),
        ('head yaw in the model space (deg)', [(col['head_yaw_in'], blue, 'recorded pose (no layer)'), (col['head_yaw_out'], red, 'with the layer'),
                                              (col['gaze_yaw'], gray, 'gaze')]),
        ('chest (Spine3) yaw (deg)', [(col['chest_yaw_in'], blue, ''), (col['chest_yaw_out'], red, '')]),
        ('eyes in the head: yaw (deg)', [(col['eye_yaw'], green, '')]),
        ('head pitch (deg)', [(col['head_pitch_in'], blue, ''), (col['head_pitch_out'], red, ''), (col['gaze_pitch'], gray, '')]),
    ]
    ph_h = (H - 90) // len(panels)
    for k, (label, lines) in enumerate(panels):
        y0, y1 = 36 + k * ph_h, 36 + (k + 1) * ph_h - 16
        s = 0
        for j in range(1, len(t) + 1):
            if j == len(t) or (col['weight'][j] > 0) != (col['weight'][s] > 0):
                d.rectangle([px(t[s]), y0, px(t[min(j, len(t) - 1)]), y1], fill=shade[1 if col['weight'][s] > 0 else 0])
                s = j
        allv = np.concatenate([np.asarray(v) for v, _, _ in lines])
        lo, hi = float(np.nanmin(allv)), float(np.nanmax(allv))
        if hi - lo < 1e-6:
            lo, hi = lo - 1, hi + 1
        pad = 0.08 * (hi - lo)
        lo, hi = lo - pad, hi + pad

        def py(v):
            return y1 - (y1 - y0) * (v - lo) / (hi - lo)

        if lo < 0 < hi:
            d.line([X0, py(0), X1, py(0)], fill=(170, 170, 170))
        for arr, colr, _ in lines:
            pts = [(px(t[j]), py(float(arr[j]))) for j in range(len(t))]
            d.line(pts, fill=colr, width=2)
        d.rectangle([X0, y0, X1, y1], outline=(80, 80, 80))
        d.text((6, y0 + 4), label, fill='black', font=fsm)
        for v in (lo + pad, hi - pad):
            d.text((X0 - 48, py(v) - 7), '%.0f' % v, fill='black', font=fsm)
    d.text((X0, 8), title, fill='black', font=font)
    lx = X0 + 700
    d.rectangle([lx, 10, lx + 14, 22], fill=shade[1], outline=(80, 80, 80))
    d.text((lx + 18, 8), 'look on', fill='black', font=fsm)
    d.rectangle([lx + 90, 10, lx + 104, 22], fill=shade[0], outline=(80, 80, 80))
    d.text((lx + 108, 8), 'released', fill='black', font=fsm)
    d.line([lx + 190, 16, lx + 215, 16], fill=blue, width=3)
    d.text((lx + 220, 8), 'recorded pose (no layer)', fill='black', font=fsm)
    d.line([lx + 400, 16, lx + 425, 16], fill=red, width=3)
    d.text((lx + 430, 8), 'with the layer', fill='black', font=fsm)
    d.text((X0, H - 30), 'time (s) from %.1f to %.1f' % (t[0], t[-1]), fill='black', font=fsm)
    img.save(out)
    print('figure written: %s' % out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    e = sub.add_parser('export')
    e.add_argument('pose')
    e.add_argument('log')
    e.add_argument('--window', nargs=2, required=True)
    e.add_argument('--out', required=True)
    e.add_argument('--stride', type=int, default=1)
    e.add_argument('--sweep', action='store_true')
    e.add_argument('--weapon', action='store_true')
    f = sub.add_parser('figure')
    f.add_argument('csv')
    f.add_argument('--out', required=True)
    f.add_argument('--title', default='')
    r = sub.add_parser('run')
    r.add_argument('pose')
    r.add_argument('log')
    r.add_argument('--window', nargs=2, required=True)
    r.add_argument('--exe', required=True)
    r.add_argument('--png', required=True)
    r.add_argument('--csv', required=True)
    r.add_argument('--fixture', required=True)
    r.add_argument('--stride', type=int, default=1)
    r.add_argument('--sweep', action='store_true')
    r.add_argument('--weapon', action='store_true')
    r.add_argument('--title', default='')
    a = ap.parse_args()
    if a.cmd == 'export':
        export(a.pose, a.log, a.window, a.out, a.stride, a.sweep, a.weapon)
    elif a.cmd == 'figure':
        figure(a.csv, a.out, a.title)
    else:
        export(a.pose, a.log, a.window, a.fixture, a.stride, a.sweep, a.weapon)
        res = subprocess.run([a.exe, a.fixture, a.csv], capture_output=True, text=True)
        sys.stdout.write(res.stdout)
        figure(a.csv, a.png, a.title)
        sys.exit(res.returncode)


if __name__ == '__main__':
    main()
