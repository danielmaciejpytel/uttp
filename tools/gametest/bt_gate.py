"""Vehicle mount / exit regression gate for UTPP: body-trace CSV + UTPP log (read-only, stdlib only).

  python Source\\tools\\gametest\\bt_gate.py --log utpp-2026-10-28-19-54-19.log --trace utpp-bodytrace-2026-10-28-19-54-46.csv
        [--feed utpp-feed-*.txt] [--json report.json|-] [--tz-offset HOURS] [--stage s0|s1] [--allow-uncovered] [--allow-empty]
        [threshold options]

Exit code: 0 = every window passed, 1 = at least one violation, 2 = input / usage error (also: no vehicle window found,
unless --allow-empty).

Windows come from the log ([utpp_body] and [utpp_vehicle_anim] lines only; the utpp_vehiclecam 'keepTpp ...' lines carry
similar text but are not used):
  mount : first 'ride N: utpp_vehicle_tpp=1' (or 'vehicle idle -> mount' when there is no ride line, e.g. cars)
          .. 'vehicle mount -> ride' + mount_tail (0.6 s)
  exit  : 'vehicle ride -> exit' .. 'vehicle exit -> after' (the unmount) + exit_tail (0.6 s)

Trace checks (main skeleton only; no *_JNT helpers):
  mount : leg bones (UpLeg/Leg/Foot/ToeBase, both sides) <= 15 deg per frame and a per-bone rate change <= 10.5 deg per frame
          between two frames (leg_acc: a pop, a one-frame spike or an onset at full speed); Hips <= 7.5 cm per frame; no run
          of >= 5 identical leg frames after the cut (the cut = first leg step > 1 deg inside the window; the scan ends at
          'mount -> ride'); feet (model space) <= 1 cm per frame from the clip-end proxy to the window end
          (proxy = 'mount -> ride' + clip_end_offset, default 0.3 s; the log has no clip-end marker, the parameter is
          printed in the report).
          The trace is model space: during a standing (bike) entry the vehtrans root offset (written on the Hips) holds the
          body still in the world while the entity slides, so the model-space Hips moves by the slide. With --feed the Hips
          step is measured in the world (trace composed with the feed entity transform) and foot_skate checks that the feet
          are not carried together: the planar common mode of the two world foot steps (one step projected on the other's
          direction) <= 4 cm per frame, from mount start + 0.45 s (before that the hold and the leg decay of the transition
          move both feet in place) to 'mount -> ride'. Without --feed a note says so.
  exit  : Hips <= 4.5 cm per frame; arm bones (Arm / ForeArm / Hand) <= 30 deg per frame, every other main-skeleton bone
          <= 15 deg per frame; Hips height (model space z) >= 0.90 m from unmount - 0.1 s to unmount + 0.6 s.
  ride edge : around the log line 'vehicle mount -> ride' (edge +-0.5 s, the selected stage): local rotation step of
          Neck / Neck1 / Head <= 8 deg per frame; Head position relative to Hips (model space) <= 5 cm per frame; Head position
          in the Spine3 frame within 5 cm of the straight line between the frames nearest to edge - 50 ms and edge + 300 ms
          (ride_line_cm). A one-frame upper-body pop at the edge measured 23-35 deg (neck chain), 25-29 cm (Head vs Hips) and
          10 cm (line) on the first bike mount of four sessions; the second mount of each session stayed at <= 4 deg, <= 3.7 cm
          and <= 1.3 cm. The step checks use the same stall handling and dt scaling as the other mount checks.
Limits (calibrated on the 2026-10-28 23:05 session against the vanilla clips sampled at 60 Hz, local rotations / Hips
translation; Extracted\\game\\vehicle-bike\\glb):
  exit Hips 4.5 cm: vanilla tpp_pwa_sportbike exit_l / exit_r Hips max 3.08 / 3.49 cm per 1/60 s (pwa exit_l 3.47); the 23:05
          stand-ups ran at 3.1-3.5 cm; the known defects: 4.9 (exit-start step), 5.3 (dip), 22.7 (flash), 69 (squat).
  exit arms 30 deg: vanilla exit_l RightForeArm 19.4 at clip t 1.483 (one 30 fps key segment), LeftHand 16.2 at t 1.117; the
          game showed that key at exit + 1.485 s as 14.3 + 25.6 deg (frame phase). Spine / head / legs keep 15 (female tpp exit
          clips <= 13.2). Scope: calibrated on female V, sportbike1 (exit_l traced; exit_r from the clip only). The male tpp_pma
          exits have arm keys of 44.9-57.0 deg and legs up to 14.0 deg per 1/60 s in the clip data: a male V exit is a known
          false-positive risk for the 30 / 15 deg limits until a traced male session calibrates them.
  mount Hips 7.5 cm: the 12 c9 enter clips (Build\\work\\vehicle-bike-archive\\c9-clips-d) move the Hips up to 6.22 cm per 1/60 s
          (6.40 with root motion; male sportbike2 enter_l), vanilla enter <= 4.38; the c8b pops were 27-39 deg leg steps.
  foot_skate 4 cm: planar common mode of the feet in the c9 enter clips <= 3.32 cm per 1/60 s (vanilla <= 1.53); in game
          (ride 1, 0.23 m root offset) <= 2.5; ride 2 (0.68 m offset, old decay) 4.1-6.7 for 8 frames.
  mount legs 15 deg / leg_acc 10.5: in game the female enter_r ran at <= 9.84 deg per frame, rate change <= 8.05 (smooth, at the
          clip's own largest rate change, clip t 0.18 s); the male clips are up to 1.3x faster in the clip data. The in-game
          local rotation rates are ~1.8x the rates read from the exported GLB (positions agree: LeftFoot path 2.95 m in game,
          2.82 m in the clip), so the rotation limits come from the game traces, not from the GLB numbers. Known pops: c8b 27-39
          deg; c9 ride 2 leg spike 17.5 deg (rate change 13.5); ride 1 post-clip leg swap onset 0 -> 11.2.
Frames: one or two frames identical to the frame before (a game stall: the animation did not advance while it was moving)
are skipped by the step checks, so the catch-up step is measured over the stall (dt-scaled). leg_acc compares true rates and
merges a sample closer than 10 ms to the previous one into the next step (a hitch pair 30 ms + 6 ms is one step). The
frozen-leg check still sees every frame.
Log checks:
  exit  : EXITDIAG weight ramp (a 'w<1' line followed by an 'end' line with w=1 at least 0.1 s later); EXITDIAG end line
          fields exitsHanded / exits / noCapture when present; 'head slot tpp -> empty' and 'deactivated by the game'
          inside the window.
  mount : 'vehtrans mount' legsDroppedByCap == 0 when the field is present.
Fields that are not in the log yet are reported as n/a in the window notes, never as a pass.

Per-frame steps: a sample closer than 1 ms to the previous one is a duplicate (the member nearer to the frame before the
pair is kept). A step over a gap longer than 1/60 s is scaled by (1/60)/dt (never amplified for shorter dt); use
--no-dt-scale for raw per-sample steps. The trace is matched to the log through the local time of the log line
'body trace continuous: started file=<trace name>' against the 'utc=' header of the CSV (rounded to whole hours);
--tz-offset overrides it.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import re
import sys
from datetime import datetime, timedelta, timezone

EPOCH = datetime(1970, 1, 1)
FRAME = 1.0 / 60.0

DEFAULTS = {
    'mount_tail': 0.6,
    'exit_tail': 0.6,
    'exit_height_pre': 0.1,
    'mount_leg_deg': 15.0,
    'mount_leg_acc_deg': 10.5,
    'mount_hips_cm': 7.5,
    'mount_frozen_frames': 5,
    'mount_frozen_eps_deg': 0.1,
    'mount_cut_deg': 1.0,
    'mount_feet_cm': 1.0,
    'clip_end_offset': 0.3,
    'exit_hips_cm': 4.5,
    'exit_bone_deg': 15.0,
    'exit_arm_deg': 30.0,
    'exit_hips_min_m': 0.90,
    'dup_ms': 1.0,
    'coverage_slack': 0.25,
    'ramp_min_s': 0.1,
    'min_dt_ms': 10.0,
    'stall_max_frames': 2,
    'stall_eps_deg': 0.05,
    'skate_cm': 4.0,
    'skate_start_s': 0.45,
    'feed_search_ms': 60.0,
    'ride_edge_s': 0.5,
    'ride_neck_deg': 8.0,
    'ride_head_hips_cm': 5.0,
    'ride_head_line_cm': 5.0,
    'ride_line_pre_s': 0.05,
    'ride_line_post_s': 0.3,
    'ride_line_slack_s': 0.05,
}

LEG_BONES = ['LeftUpLeg', 'LeftLeg', 'LeftFoot', 'LeftToeBase', 'RightUpLeg', 'RightLeg', 'RightFoot', 'RightToeBase']
FEET = ['LeftFoot', 'RightFoot']
MAIN_BONES = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head',
              'LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand', 'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand'] + LEG_BONES
NECK_BONES = ['Neck', 'Neck1', 'Head']
ARM_BONES = {'LeftArm', 'LeftForeArm', 'LeftHand', 'RightArm', 'RightForeArm', 'RightHand'}
TRACE_BONES = set(MAIN_BONES) | {'Root'}

LOG_TS = re.compile(r'^\[(\d{4})-(\d\d)-(\d\d) (\d\d):(\d\d):(\d\d)\.(\d+)\]')
RE_ANIM_START = re.compile(r'\[utpp_vehicle_anim\] ride (\d+): utpp_vehicle_tpp=1')
RE_IDLE_MOUNT = re.compile(r'\[utpp_body\].*vehicle idle -> mount')
RE_MOUNT_RIDE = re.compile(r'\[utpp_body\].*vehicle mount -> ride')
RE_RIDE_EXIT = re.compile(r'\[utpp_body\].*vehicle ride -> exit')
RE_EXIT_AFTER = re.compile(r'\[utpp_body\].*vehicle exit -> after')
RE_KV = re.compile(r'([A-Za-z_][A-Za-z0-9_]*)=(\S+)')


# ----------------------------------------------------------------------------------------------------------- helpers

def fmt_t(t):
    if t is None:
        return '-'
    return (EPOCH + timedelta(seconds=t)).strftime('%H:%M:%S.%f')[:-3]


def epoch_of(dt):
    return (dt - EPOCH).total_seconds()


def to_float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


class Violation(dict):
    pass


def viol(check, t, value, limit, unit, bone=None, detail=None):
    v = {'check': check, 't': t, 'time': fmt_t(t), 'value': value, 'limit': limit, 'unit': unit}
    if bone:
        v['bone'] = bone
    if detail:
        v['detail'] = detail
    return v


# ----------------------------------------------------------------------------------------------------------- log

class LogLine:
    __slots__ = ('t', 'text')

    def __init__(self, t, text):
        self.t = t
        self.text = text


def parse_log(path):
    """Timestamped lines sorted by time (the log is written from several threads; sort is stable)."""
    lines = []
    # the UTPP log rotates at 10 MB (utpp-<start>.N.log = older parts): a current file stands for its whole session
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import gt_log
        files = gt_log.session_files(path) or [path]   # missing file: open() raises the usual OSError
    except ImportError:
        files = [path]
    for fp in files:
        with open(fp, 'r', encoding='utf-8', errors='replace') as f:
            for raw in f:
                m = LOG_TS.match(raw)
                if not m:
                    continue
                y, mo, d, h, mi, s, frac = m.groups()
                t = epoch_of(datetime(int(y), int(mo), int(d), int(h), int(mi), int(s))) + float('0.' + frac)
                lines.append(LogLine(t, raw.rstrip('\n')))
    lines.sort(key=lambda l: l.t)
    return lines


def find_windows(lines, cfg):
    """Mount and exit windows from the log. Returns (windows, notes)."""
    notes = []
    wins = []
    starts = []          # (t, ride_no or None, source)
    for l in lines:
        m = RE_ANIM_START.search(l.text)
        if m:
            starts.append((l.t, int(m.group(1)), 'anim'))
        elif RE_IDLE_MOUNT.search(l.text):
            starts.append((l.t, None, 'idle'))
    # merge starts closer than 1 s (a bike mount writes both lines): keep the earliest, take the ride number from any
    merged = []
    for s in starts:
        if merged and s[0] - merged[-1][0] < 1.0:
            if merged[-1][1] is None and s[1] is not None:
                merged[-1] = (merged[-1][0], s[1], merged[-1][2])
            continue
        merged.append(s)
    mount_ride = [l.t for l in lines if RE_MOUNT_RIDE.search(l.text)]
    for i, (t0, ride, src) in enumerate(merged):
        nxt = merged[i + 1][0] if i + 1 < len(merged) else float('inf')
        ends = [t for t in mount_ride if t0 <= t < nxt]
        w = {'kind': 'mount', 'id': ('ride %d' % ride) if ride is not None else 'mount %d' % (i + 1),
             'ride': ride, 'start_source': src, 't0': t0}
        if ends:
            w['t_ride'] = ends[0]
            w['t1'] = ends[0] + cfg['mount_tail']
        else:
            w['t_ride'] = None
            w['t1'] = None
        wins.append(w)
    exits = [l.t for l in lines if RE_RIDE_EXIT.search(l.text)]
    afters = [l.t for l in lines if RE_EXIT_AFTER.search(l.text)]
    for i, t0 in enumerate(exits):
        nxt = exits[i + 1] if i + 1 < len(exits) else float('inf')
        a = [t for t in afters if t0 <= t < nxt]
        w = {'kind': 'exit', 'id': 'exit %d' % (i + 1), 'ride': None, 'start_source': 'ride->exit', 't0': t0}
        if a:
            w['t_unmount'] = a[0]
            w['t1'] = a[0] + cfg['exit_tail']
        else:
            w['t_unmount'] = None
            w['t1'] = None
        wins.append(w)
    wins.sort(key=lambda w: w['t0'])
    return wins, notes


def parse_kv(text):
    return {k: v for k, v in RE_KV.findall(text)}


def check_log_mount(w, lines, cfg):
    out, notes = [], []
    if w['t1'] is None:
        return out, notes
    seen = False
    for l in lines:
        if l.t < w['t0'] or l.t > w['t1'] + 1.0:
            continue
        if 'vehtrans mount:' not in l.text:
            continue
        kv = parse_kv(l.text)
        if w['ride'] is not None and 'ride' in kv and to_float(kv['ride']) != w['ride']:
            continue
        if 'legsDroppedByCap' in kv:
            seen = True
            n = to_float(kv['legsDroppedByCap'])
            if n is not None and n != 0:
                out.append(viol('legs_dropped_by_cap', l.t, n, 0, 'bones', detail=l.text.split('] ', 3)[-1][:160]))
    if not seen:
        notes.append("'vehtrans mount' legsDroppedByCap: n/a (field not in the log)")
    return out, notes


def _w_below_one(kv):
    w = kv.get('w')
    if w is None:
        return None
    if w.startswith('<'):
        return True
    f = to_float(w)
    if f is None:
        return None
    return f < 0.999


def check_log_exit(w, lines, cfg):
    out, notes = [], []
    if w['t1'] is None:
        return out, notes
    diag = [l for l in lines if w['t0'] <= l.t <= w['t1'] and 'UTPP EXITDIAG' in l.text]
    first_low = None
    end_line = None
    for l in diag:
        kv = parse_kv(l.text)
        low = _w_below_one(kv)
        if low and first_low is None:
            first_low = l
        if re.search(r'\bend\s*$', l.text):
            end_line = l
    if not diag:
        notes.append('EXITDIAG: no line inside the window (n/a)')
    if first_low is not None and end_line is not None and end_line.t - first_low.t >= cfg['ramp_min_s']:
        out.append(viol('exit_weight_ramp', first_low.t, round(end_line.t - first_low.t, 3), cfg['ramp_min_s'], 's',
                        detail='EXITDIAG w<1 at hand-in, w=1 only at the end line (%s)' % fmt_t(end_line.t)))
    elif first_low is not None and end_line is None:
        notes.append('EXITDIAG: w<1 at hand-in but no end line to time the ramp')
    if end_line is not None:
        kv = parse_kv(end_line.text)
        handed = to_float(kv.get('exitsHanded'))
        exits = to_float(kv.get('exits'))
        nocap = to_float(kv.get('noCapture', kv.get('exitsNoCapture')))
        if handed is None and nocap is None:
            notes.append('EXITDIAG end line: exitsHanded / noCapture n/a (fields not in the log)')
        else:
            if handed is not None:
                want = exits if exits is not None else 1
                if handed != want:
                    out.append(viol('exitdiag_exits_handed', end_line.t, handed, want, 'exits',
                                    detail='exitsHanded must equal exits'))
            if nocap is not None and nocap != 0:
                out.append(viol('exitdiag_no_capture', end_line.t, nocap, 0, 'exits', detail='exit without a capture'))
    for l in lines:
        if l.t < w['t0'] or l.t > w['t1']:
            continue
        if 'head slot tpp -> empty' in l.text:
            out.append(viol('head_slot_tpp_empty', l.t, 1, 0, 'events', detail='head slot tpp -> empty inside the exit window'))
        if 'deactivated by the game' in l.text:
            out.append(viol('rep_deactivated_by_game', l.t, 1, 0, 'events',
                            detail='representation deactivated by the game inside the exit window'))
    return out, notes


def log_tz_anchor(lines, trace_name):
    """Local time of the log line that announces the trace file, or None."""
    base = os.path.basename(trace_name)
    for l in lines:
        if 'body trace continuous: started' in l.text and base in l.text.replace('\\', '/').split('/')[-1]:
            return l.t
    return None


# ----------------------------------------------------------------------------------------------------------- trace

class Frame:
    __slots__ = ('t', 'loc', '_model')

    def __init__(self, t):
        self.t = t
        self.loc = {}            # bone name -> (px, py, pz, qx, qy, qz, qw)
        self._model = {}


class Trace:
    def __init__(self):
        self.parents = {}        # index -> parent index
        self.index = {}          # name -> index
        self.name_of = {}        # index -> name
        self.utc = None          # epoch seconds of the header utc (naive UTC)
        self.arm_ticks = None
        self.qpc_hz = 1e7
        self.frames = []


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qrot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


def qangle_deg(a, b):
    """Angle between two unit quaternions (xyzw), degrees."""
    ax, ay, az, aw = a
    conj = (-ax, -ay, -az, aw)
    x, y, z, w = qmul(conj, b)
    return math.degrees(2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), abs(w)))


def model_pos(tr, fr, name):
    """Model-space position of a bone (FK along the header parent chain, over the bones present in the frame)."""
    key = tr.index.get(name)
    if key is None:
        return None
    return _model(tr, fr, key)[0]


def _model(tr, fr, idx):
    hit = fr._model.get(idx)
    if hit is not None:
        return hit
    name = tr.name_of.get(idx)
    loc = fr.loc.get(name) if name is not None else None
    if loc is None:
        res = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        if name is not None and tr.parents.get(idx, -1) >= 0:
            res = _model(tr, fr, tr.parents[idx])
        fr._model[idx] = res
        return res
    p = tr.parents.get(idx, -1)
    if p < 0:
        res = ((loc[0], loc[1], loc[2]), (loc[3], loc[4], loc[5], loc[6]))
    else:
        pp, pq = _model(tr, fr, p)
        r = qrot(pq, (loc[0], loc[1], loc[2]))
        res = ((pp[0] + r[0], pp[1] + r[1], pp[2] + r[2]), qmul(pq, (loc[3], loc[4], loc[5], loc[6])))
    fr._model[idx] = res
    return res


def read_trace(path, ranges, tz_offset_s, stage='s1', anchor_log_t=None):
    """Stream the CSV once. ranges = [(t0, t1)] in log (local) time; returns Trace with the frames inside them.
    tz_offset_s None = derive it (anchor log line, else file name, else system zone)."""
    tr = Trace()
    col = 20 if stage == 's1' else 8       # first of the 12 stage columns: pos xyz at +0..2, quaternion xyzw at +4..7
    cur = None
    cur_tk = None
    last_hi = max((r[1] for r in ranges), default=None)
    tz_note = None
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            if line.startswith('#'):
                if line.startswith('# rig_bone'):
                    kv = dict(RE_KV.findall(line))
                    if 'index' in kv and 'parent' in kv:
                        tr.parents[int(kv['index'])] = int(kv['parent'])
                elif 'arm_ticks=' in line:
                    kv = dict(RE_KV.findall(line))
                    tr.arm_ticks = int(kv['arm_ticks'])
                    tr.qpc_hz = float(kv.get('qpc_hz', 1e7))
                    u = datetime.strptime(kv['utc'], '%Y-%m-%dT%H:%M:%S.%fZ')
                    tr.utc = epoch_of(u)
                    if tz_offset_s is None:
                        tz_offset_s, tz_note = derive_tz(tr.utc, anchor_log_t, path)
                    tr.tz_offset_s = tz_offset_s
                    tr.tz_note = tz_note
                continue
            if line.startswith('ticks'):
                continue
            if tr.arm_ticks is None:
                raise ValueError('trace has no arm_ticks/utc header line')
            i = line.find(',')
            tk = int(line[:i])
            if tk == cur_tk and cur is None:
                continue
            if tk != cur_tk:
                cur_tk = tk
                t = tr.utc + tr.tz_offset_s + (tk - tr.arm_ticks) / tr.qpc_hz
                if last_hi is not None and t > last_hi:
                    break
                if any(a <= t <= b for a, b in ranges):
                    cur = Frame(t)
                    tr.frames.append(cur)
                else:
                    cur = None
            if cur is None:
                continue
            p = line.split(',', col + 8)
            name = p[6]
            if name not in TRACE_BONES:
                continue
            if name not in tr.index:
                gi = int(p[7])
                tr.index[name] = gi
                tr.name_of[gi] = name
            try:
                cur.loc[name] = (float(p[col]), float(p[col + 1]), float(p[col + 2]),
                                 float(p[col + 4]), float(p[col + 5]), float(p[col + 6]), float(p[col + 7]))
            except (ValueError, IndexError):
                continue
    if tr.arm_ticks is None:
        raise ValueError('trace has no arm_ticks/utc header line')
    return tr


def derive_tz(utc_epoch, anchor_log_t, path):
    if anchor_log_t is not None:
        off = round((anchor_log_t - utc_epoch) / 3600.0) * 3600
        return off, 'log anchor (body trace continuous: started): %+d h' % (off // 3600)
    m = re.search(r'(\d{4})-(\d\d)-(\d\d)-(\d\d)-(\d\d)-(\d\d)', os.path.basename(path))
    if m:
        local = epoch_of(datetime(*[int(x) for x in m.groups()]))
        off = round((local - utc_epoch) / 3600.0) * 3600
        return off, 'trace file name: %+d h' % (off // 3600)
    off = datetime.fromtimestamp(utc_epoch + 0.0, tz=timezone.utc).astimezone().utcoffset().total_seconds()
    return off, 'system zone: %+g h' % (off / 3600)


def dedupe(tr, dup_s):
    """Drop samples closer than dup_s to the previous kept one; of a pair keep the member nearer to the frame before it."""
    out = []
    dropped = 0

    def dist(a, b):
        ha, hb = a.loc.get('Hips'), b.loc.get('Hips')
        if ha is None or hb is None:
            return 0.0
        return math.dist(ha[:3], hb[:3]) + math.radians(qangle_deg(ha[3:], hb[3:]))

    for f in tr.frames:
        if out and f.t - out[-1].t < dup_s:
            dropped += 1
            if len(out) >= 2 and dist(f, out[-2]) < dist(out[-1], out[-2]):
                out[-1] = f
            continue
        out.append(f)
    tr.frames = out
    return dropped


def dt_scale(dt, enabled):
    if not enabled or dt <= FRAME:
        return 1.0
    return FRAME / dt


def window_frames(tr, t0, t1):
    return [f for f in tr.frames if t0 <= f.t <= t1]


def leg_step(a, b):
    worst, bone = 0.0, None
    for n in LEG_BONES:
        la, lb = a.loc.get(n), b.loc.get(n)
        if la is None or lb is None:
            continue
        d = qangle_deg(la[3:], lb[3:])
        if d > worst:
            worst, bone = d, n
    return worst, bone


def hips_step_cm(tr, a, b):
    pa, pb = model_pos(tr, a, 'Hips'), model_pos(tr, b, 'Hips')
    if pa is None or pb is None:
        return None
    return math.dist(pa, pb) * 100.0


def feet_step_cm(tr, a, b):
    worst, bone = 0.0, None
    for n in FEET:
        pa, pb = model_pos(tr, a, n), model_pos(tr, b, n)
        if pa is None or pb is None:
            continue
        d = math.dist(pa, pb) * 100.0
        if d > worst:
            worst, bone = d, n
    return worst, bone


def coverage(frames, t0, t1, cfg):
    if len(frames) < 3:
        return viol('trace_coverage', t0, len(frames), 3, 'frames', detail='fewer than 3 trace frames inside the window')
    if frames[0].t > t0 + cfg['coverage_slack'] or frames[-1].t < t1 - cfg['coverage_slack']:
        return viol('trace_coverage', t0, round(frames[-1].t - frames[0].t, 3), round(t1 - t0, 3), 's',
                    detail='trace covers %s .. %s of %s .. %s' % (fmt_t(frames[0].t), fmt_t(frames[-1].t), fmt_t(t0), fmt_t(t1)))
    return None


def same_pose(a, b, eps_deg):
    """Every main bone within eps_deg and the Hips translation within 0.2 mm: the animation did not advance."""
    for n in MAIN_BONES:
        la, lb = a.loc.get(n), b.loc.get(n)
        if la is None or lb is None:
            continue
        if qangle_deg(la[3:], lb[3:]) >= eps_deg:
            return False
    ha, hb = a.loc.get('Hips'), b.loc.get('Hips')
    return ha is None or hb is None or math.dist(ha[:3], hb[:3]) < 0.0002


def moving(a, b):
    """The pose changed visibly between two frames (some main bone > 1 deg or the Hips > 3 mm)."""
    for n in MAIN_BONES:
        la, lb = a.loc.get(n), b.loc.get(n)
        if la is not None and lb is not None and qangle_deg(la[3:], lb[3:]) > 1.0:
            return True
    ha, hb = a.loc.get('Hips'), b.loc.get('Hips')
    return ha is not None and hb is not None and math.dist(ha[:3], hb[:3]) > 0.003


def step_frames(frames, cfg, merge_short=False):
    """Frames for the per-frame step checks. merge_short (leg_acc rates only): a sample closer than min_dt_ms to the previous
    kept one is merged into the next step (a 6 ms sample would amplify its step into a false rate change; the step checks keep
    it, unscaled, so a one-frame pop is never spread over a longer gap); a run of 1..stall_max_frames frames identical to the
    frame before (a game stall) is skipped when the pose was moving into it and moves on after it, so the catch-up step spans
    the stall (a held pose that starts moving again, e.g. a clip end followed by the next clip, is not a stall). Returns (frames, merged count, stall frames skipped)."""
    min_dt = cfg['min_dt_ms'] / 1000.0
    out, merged = [], 0
    for f in frames:
        if merge_short and out and f.t - out[-1].t < min_dt:
            merged += 1
            continue
        out.append(f)
    if len(out) < 3:
        return out, merged, 0
    res, j, stalls = [out[0]], 1, 0
    while j < len(out):
        k = j
        while k < len(out) and same_pose(out[k - 1], out[k], cfg['stall_eps_deg']):
            k += 1
        run = k - j
        if 0 < run <= cfg['stall_max_frames'] and k < len(out) and j >= 2 and moving(out[j - 2], out[j - 1]):
            stalls += run
            j = k
            continue
        res.extend(out[j:max(k, j + 1)])
        j = max(k, j + 1)
    return res, merged, stalls


def leg_rates(a, b):
    """Per-bone leg rotation rate, degrees per 1/60 s (true rate: also short frames are normalised)."""
    dt = b.t - a.t
    r = {}
    if dt <= 0:
        return r
    for n in LEG_BONES:
        la, lb = a.loc.get(n), b.loc.get(n)
        if la is not None and lb is not None:
            r[n] = qangle_deg(la[3:], lb[3:]) * FRAME / dt
    return r


# ----------------------------------------------------------------------------------------------------------- feed (world space)

class Feed:
    """utpp-feed-*.txt: per game-feed sample t (CET / game clock) with the entity position (capX, capY, capZ) and yawDeg."""

    def __init__(self, path):
        self.path = path
        self.rows = []
        cols = None
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            for line in f:
                if line.startswith('#'):
                    p = line.split()
                    if len(p) > 2 and p[1] == 'cols':
                        cols = {n: i for i, n in enumerate(p[2:])}
                    continue
                if cols is None:
                    raise ValueError('feed has no "# cols" header: %s' % path)
                p = line.split()
                try:
                    self.rows.append((float(p[0]), float(p[cols['capX']]), float(p[cols['capY']]), float(p[cols['yawDeg']])))
                except (ValueError, IndexError, KeyError):
                    continue
        self.rows.sort()
        self.ts = [r[0] for r in self.rows]
        if not self.rows:
            raise ValueError('feed has no samples: %s' % path)

    def entity(self, t):
        i = bisect.bisect_left(self.ts, t)
        if i <= 0:
            return self.rows[0][1:]
        if i >= len(self.rows):
            return self.rows[-1][1:]
        p, q = self.rows[i - 1], self.rows[i]
        u = (t - p[0]) / (q[0] - p[0]) if q[0] > p[0] else 0.0
        dyaw = (q[3] - p[3] + 180.0) % 360.0 - 180.0
        return (p[1] + (q[1] - p[1]) * u, p[2] + (q[2] - p[2]) * u, p[3] + dyaw * u)


def world_pos(tr, feed, f, name, off):
    """World position of a bone: the feed entity transform at the trace frame (feed t = log t - off) applied to model space."""
    m = model_pos(tr, f, name)
    if m is None:
        return None
    x, y, yaw = feed.entity(f.t - off)
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return (x + c * m[0] - s * m[1], y + s * m[0] + c * m[1], m[2])


def feed_offset(tr, feed, w, lines, frames, cfg):
    """Log time - feed time for a mount window: the CET 't=' of 'vehicle idle -> mount' (10 ms resolution), refined within
    +-feed_search_ms to the smoothest world Hips (the feed and the animation thread sample the entity at different moments).
    Returns (offset, note) or (None, note)."""
    anchor = None
    for l in lines:
        if w['t0'] - 0.5 <= l.t <= (w['t1'] or w['t0'] + 3.0) and 'vehicle idle -> mount' in l.text:
            m = re.search(r'\bt=([0-9.]+)', l.text)
            if m:
                anchor = (l.t, float(m.group(1)))
                break
    if anchor is None:
        return None, "feed: no 'vehicle idle -> mount' line with t= in the window (world-space checks n/a)"
    off0 = anchor[0] - anchor[1]
    if feed.ts[0] > frames[0].t - off0 or feed.ts[-1] < frames[-1].t - off0:
        return None, 'feed: does not cover the window (world-space checks n/a)'

    def jerk(off):
        p = [world_pos(tr, feed, f, 'Hips', off) for f in frames]
        tot = 0.0
        for i in range(1, len(p) - 1):
            tot += ((p[i + 1][0] - 2 * p[i][0] + p[i - 1][0]) ** 2 + (p[i + 1][1] - 2 * p[i][1] + p[i - 1][1]) ** 2)
        return tot

    span = int(cfg['feed_search_ms'])
    best = min((jerk(off0 + k / 1000.0), off0 + k / 1000.0) for k in range(-span, span + 1, 2))
    return best[1], 'feed: world space, entity clock = log - %.3f s (CET anchor %+.0f ms, refined to the smoothest world Hips)' % (
        best[1] % 60.0, (best[1] - off0) * 1000.0)


def head_in_spine3(tr, fr):
    """Head position in the Spine3 frame (m), or None when either bone is missing from the frame."""
    ih, i3 = tr.index.get('Head'), tr.index.get('Spine3')
    if ih is None or i3 is None or 'Head' not in fr.loc or 'Spine3' not in fr.loc:
        return None
    ph, _ = _model(tr, fr, ih)
    p3, q3 = _model(tr, fr, i3)
    return qrot((-q3[0], -q3[1], -q3[2], q3[3]), (ph[0] - p3[0], ph[1] - p3[1], ph[2] - p3[2]))


def head_vs_hips(tr, fr):
    """Model-space Head position minus Hips position (m), or None."""
    ph, pi = model_pos(tr, fr, 'Head'), model_pos(tr, fr, 'Hips')
    if ph is None or pi is None or 'Head' not in fr.loc or 'Hips' not in fr.loc:
        return None
    return (ph[0] - pi[0], ph[1] - pi[1], ph[2] - pi[2])


def check_ride_edge(tr, w, cfg, scale_on):
    """Neck / Head pose pop around 'vehicle mount -> ride': neck-chain local step, Head vs Hips step, Head vs Spine3 line."""
    out, notes, stats = [], [], {}
    edge = w['t_ride']
    fr = [f for f in tr.frames if edge - cfg['ride_edge_s'] <= f.t <= edge + cfg['ride_edge_s']]
    stats['edge_frames'] = len(fr)
    if len(fr) < 3:
        out.append(viol('trace_coverage', edge, len(fr), 3, 'frames',
                        detail="fewer than 3 trace frames within %.2f s of 'mount -> ride'" % cfg['ride_edge_s']))
        return out, notes, stats
    sf, _, _ = step_frames(fr, cfg)
    neck_max = hh_max = 0.0
    for a, b in zip(sf, sf[1:]):
        sc = dt_scale(b.t - a.t, scale_on)
        for n in NECK_BONES:
            la, lb = a.loc.get(n), b.loc.get(n)
            if la is None or lb is None:
                continue
            d = qangle_deg(la[3:], lb[3:]) * sc
            neck_max = max(neck_max, d)
            if d > cfg['ride_neck_deg']:
                out.append(viol('ride_neck_rot', b.t, round(d, 2), cfg['ride_neck_deg'], 'deg/frame', n,
                                "%+.0f ms from 'mount -> ride'" % ((b.t - edge) * 1000.0)))
        ra, rb = head_vs_hips(tr, a), head_vs_hips(tr, b)
        if ra is not None and rb is not None:
            hs = math.dist(ra, rb) * 100.0 * sc
            hh_max = max(hh_max, hs)
            if hs > cfg['ride_head_hips_cm']:
                out.append(viol('ride_head_hips', b.t, round(hs, 2), cfg['ride_head_hips_cm'], 'cm/frame', 'Head',
                                "Head relative to Hips, %+.0f ms from 'mount -> ride'" % ((b.t - edge) * 1000.0)))
    stats['ride_neck_max_deg'] = round(neck_max, 2)
    stats['ride_head_hips_max_cm'] = round(hh_max, 2)
    t_a, t_b = edge - cfg['ride_line_pre_s'], edge + cfg['ride_line_post_s']
    fa = min(fr, key=lambda f: abs(f.t - t_a))
    fb = min(fr, key=lambda f: abs(f.t - t_b))
    slack = cfg['ride_line_slack_s']
    if abs(fa.t - t_a) > slack or abs(fb.t - t_b) > slack or fb.t <= fa.t:
        out.append(viol('trace_coverage', edge, round(fb.t - fa.t, 3), round(t_b - t_a, 3), 's',
                        detail="no trace frame within %.0f ms of edge - %.0f ms / edge + %.0f ms (head line n/a)" % (
                            slack * 1000.0, cfg['ride_line_pre_s'] * 1000.0, cfg['ride_line_post_s'] * 1000.0)))
        return out, notes, stats
    pa, pb = head_in_spine3(tr, fa), head_in_spine3(tr, fb)
    if pa is None or pb is None:
        notes.append('ride edge head line: Head / Spine3 not in the trace (n/a)')
        return out, notes, stats
    dev_max = 0.0
    for f in fr:
        if not fa.t <= f.t <= fb.t:
            continue
        p = head_in_spine3(tr, f)
        if p is None:
            continue
        u = (f.t - fa.t) / (fb.t - fa.t)
        d = math.dist(p, tuple(pa[i] + (pb[i] - pa[i]) * u for i in range(3))) * 100.0
        if d > dev_max:
            dev_max, dev_t = d, f.t
        if d > cfg['ride_head_line_cm']:
            out.append(viol('ride_head_line', f.t, round(d, 2), cfg['ride_head_line_cm'], 'cm', 'Head',
                            "Head in the Spine3 frame off the straight line edge-%.0f ms .. edge+%.0f ms, %+.0f ms from 'mount -> ride'" % (
                                cfg['ride_line_pre_s'] * 1000.0, cfg['ride_line_post_s'] * 1000.0, (f.t - edge) * 1000.0)))
    stats['ride_head_line_max_cm'] = round(dev_max, 2)
    return out, notes, stats


def check_trace_mount(tr, w, cfg, scale_on, feed=None, lines=None):
    out, notes, stats = [], [], {}
    t0, t1, tr_ride = w['t0'], w['t1'], w['t_ride']
    frames = window_frames(tr, t0, t1)
    cov = coverage(frames, t0, t1, cfg)
    stats['frames'] = len(frames)
    if cov is not None:
        out.append(cov)
        if len(frames) < 3:
            return out, notes, stats
    clip_end = tr_ride + cfg['clip_end_offset']
    stats['clip_end_proxy'] = fmt_t(clip_end)
    sf, _, stalls = step_frames(frames, cfg)
    af, merged, _ = step_frames(frames, cfg, merge_short=True)
    if merged or stalls:
        notes.append('step checks: %d stalled frame(s) skipped; leg_acc: %d short sample(s) merged' % (stalls, merged))
    off = None
    if feed is not None:
        off, n = feed_offset(tr, feed, w, lines or [], sf, cfg)
        notes.append(n)
    else:
        notes.append('mount Hips step in model space (includes the vehtrans root offset of a standing entry); --feed: world space')
    stats['space'] = 'world' if off is not None else 'model'
    leg_max = hips_max = feet_max = acc_max = skate_max = 0.0
    prev_rates = None
    for a, b in zip(sf, sf[1:]):
        dt = b.t - a.t
        sc = dt_scale(dt, scale_on)
        ld, lbone = leg_step(a, b)
        ld *= sc
        leg_max = max(leg_max, ld)
        if ld > cfg['mount_leg_deg']:
            out.append(viol('leg_rot', b.t, round(ld, 2), cfg['mount_leg_deg'], 'deg/frame', lbone))
        if off is not None:
            pa, pb = world_pos(tr, feed, a, 'Hips', off), world_pos(tr, feed, b, 'Hips', off)
            hs = math.dist(pa, pb) * 100.0
        else:
            hs = hips_step_cm(tr, a, b)
        if hs is not None:
            hs *= sc
            hips_max = max(hips_max, hs)
            if hs > cfg['mount_hips_cm']:
                out.append(viol('hips_step', b.t, round(hs, 2), cfg['mount_hips_cm'], 'cm/frame', 'Hips',
                                'world space' if off is not None else 'model space'))
        if off is not None and t0 + cfg['skate_start_s'] <= b.t <= tr_ride:
            d = []
            for n in FEET:
                pa, pb = world_pos(tr, feed, a, n, off), world_pos(tr, feed, b, n, off)
                if pa is not None and pb is not None:
                    d.append(((pb[0] - pa[0]) * 100.0 * sc, (pb[1] - pa[1]) * 100.0 * sc))
            if len(d) == 2:
                # planar common mode: both feet carried the same way (a body slide under the feet); a step or a hop moves the
                # feet apart (the projection of one foot's step on the other's direction is then small or negative)
                n1, n2 = math.hypot(*d[0]), math.hypot(*d[1])
                common = max(0.0, d[0][0] * d[1][0] + d[0][1] * d[1][1]) / max(n1, n2, 1e-9)
                skate_max = max(skate_max, common)
                if common > cfg['skate_cm']:
                    out.append(viol('foot_skate', b.t, round(common, 2), cfg['skate_cm'], 'cm/frame', 'feet',
                                    'both feet carried the same way in the world (LeftFoot %.1f, RightFoot %.1f cm)' % (n1, n2)))
        if b.t >= clip_end:
            fs, fbone = feet_step_cm(tr, a, b)
            fs *= sc
            feet_max = max(feet_max, fs)
            if fs > cfg['mount_feet_cm']:
                out.append(viol('feet_settle', b.t, round(fs, 2), cfg['mount_feet_cm'], 'cm/frame', fbone,
                                'after the clip-end proxy %s' % fmt_t(clip_end)))
    for a, b in zip(af, af[1:]):
        rates = leg_rates(a, b)
        if prev_rates is not None:
            ab, abone = 0.0, None
            for n, r in rates.items():
                if n in prev_rates and abs(r - prev_rates[n]) > ab:
                    ab, abone = abs(r - prev_rates[n]), n
            acc_max = max(acc_max, ab)
            if ab > cfg['mount_leg_acc_deg']:
                out.append(viol('leg_acc', b.t, round(ab, 2), cfg['mount_leg_acc_deg'], 'deg/frame per frame', abone,
                                'rate %.1f -> %.1f deg/frame' % (prev_rates[abone], rates[abone])))
        prev_rates = rates
    leg_steps = [leg_step(a, b)[0] * dt_scale(b.t - a.t, scale_on) for a, b in zip(frames, frames[1:])]
    # frozen leg frames after the cut, until 'mount -> ride'
    cut = None
    for i, ld in enumerate(leg_steps):
        if ld > cfg['mount_cut_deg']:
            cut = i + 1        # frame index of the first frame with the new pose
            break
    stats['cut'] = fmt_t(frames[cut].t) if cut is not None else None
    if cut is None:
        notes.append('frozen-leg check: no cut (leg step > %g deg) found in the window' % cfg['mount_cut_deg'])
    else:
        runs = []
        last = len(frames)
        while last > cut + 1 and frames[last - 1].t > tr_ride:
            last -= 1                 # exclusive end: frames up to 'mount -> ride'
        run_start = cut
        for j in range(cut + 1, last + 1):
            same = False
            if j < last:
                d, _ = leg_step(frames[j - 1], frames[j])
                same = d < cfg['mount_frozen_eps_deg']
            if not same:
                n = j - run_start
                if n >= cfg['mount_frozen_frames']:
                    runs.append((run_start, j - 1, n))
                run_start = j
        for (s, e, n) in runs:
            out.append(viol('frozen_legs', frames[s].t, n, cfg['mount_frozen_frames'] - 1, 'identical frames',
                            detail='%s .. %s (%.0f ms)' % (fmt_t(frames[s].t), fmt_t(frames[e].t), 1000 * (frames[e].t - frames[s].t))))
        stats['frozen_runs'] = [n for _, _, n in runs]
    stats.update({'leg_max_deg': round(leg_max, 2), 'leg_acc_max': round(acc_max, 2), 'hips_max_cm': round(hips_max, 2),
                  'feet_max_cm_after_clip': round(feet_max, 2)})
    if off is not None:
        stats['skate_max_cm'] = round(skate_max, 2)
    ev, en, es = check_ride_edge(tr, w, cfg, scale_on)
    out += ev
    notes += en
    stats.update(es)
    return out, notes, stats


def check_trace_exit(tr, w, cfg, scale_on):
    out, notes, stats = [], [], {}
    t0, t1, tu = w['t0'], w['t1'], w['t_unmount']
    frames = window_frames(tr, t0, t1)
    cov = coverage(frames, t0, t1, cfg)
    stats['frames'] = len(frames)
    if cov is not None:
        out.append(cov)
        if len(frames) < 3:
            return out, notes, stats
    sf, _, stalls = step_frames(frames, cfg)
    if stalls:
        notes.append('step checks: %d stalled frame(s) skipped' % stalls)
    hips_max = bone_max = 0.0
    for a, b in zip(sf, sf[1:]):
        sc = dt_scale(b.t - a.t, scale_on)
        hs = hips_step_cm(tr, a, b)
        if hs is not None:
            hs *= sc
            hips_max = max(hips_max, hs)
            if hs > cfg['exit_hips_cm']:
                out.append(viol('hips_step', b.t, round(hs, 2), cfg['exit_hips_cm'], 'cm/frame', 'Hips'))
        for n in MAIN_BONES:
            la, lb = a.loc.get(n), b.loc.get(n)
            if la is None or lb is None:
                continue
            d = qangle_deg(la[3:], lb[3:]) * sc
            bone_max = max(bone_max, d)
            lim = cfg['exit_arm_deg'] if n in ARM_BONES else cfg['exit_bone_deg']
            if d > lim:
                out.append(viol('bone_rot', b.t, round(d, 2), lim, 'deg/frame', n))
    hmin = None
    for f in frames:
        if not (tu - cfg['exit_height_pre'] <= f.t <= tu + cfg['exit_tail']):
            continue
        p = model_pos(tr, f, 'Hips')
        if p is None:
            continue
        hmin = p[2] if hmin is None else min(hmin, p[2])
        if p[2] < cfg['exit_hips_min_m']:
            out.append(viol('hips_height', f.t, round(p[2], 3), cfg['exit_hips_min_m'], 'm', 'Hips'))
    stats.update({'hips_max_cm': round(hips_max, 2), 'bone_max_deg': round(bone_max, 2),
                  'hips_min_height_m': None if hmin is None else round(hmin, 3)})
    return out, notes, stats


# ----------------------------------------------------------------------------------------------------------- driver

def summarize(violations):
    by = {}
    for v in violations:
        s = by.setdefault(v['check'], {'check': v['check'], 'count': 0, 'worst': None, 'first': v['time']})
        s['count'] += 1
        if s['worst'] is None or (isinstance(v['value'], (int, float)) and abs(v['value']) > abs(s['worst']['value'])):
            s['worst'] = v
    return list(by.values())


def run_gate(log_path, trace_path=None, cfg=None, tz_offset_h=None, stage='s1', scale_on=True, allow_uncovered=False,
             feed_path=None):
    cfg = dict(DEFAULTS, **(cfg or {}))
    lines = parse_log(log_path)
    windows, gnotes = find_windows(lines, cfg)
    report = {'log': log_path, 'trace': trace_path, 'feed': feed_path, 'stage': stage, 'thresholds': cfg, 'dt_scale': scale_on,
              'notes': gnotes, 'windows': []}
    feed = Feed(feed_path) if feed_path else None
    trace = None
    if trace_path:
        ranges = [(w['t0'] - 1.0, w['t1'] + 1.0) for w in windows if w['t1'] is not None]
        anchor = log_tz_anchor(lines, trace_path)
        trace = read_trace(trace_path, ranges, None if tz_offset_h is None else tz_offset_h * 3600.0, stage, anchor)
        dropped = dedupe(trace, cfg['dup_ms'] / 1000.0)
        report['trace_info'] = {'tz': trace.tz_note if tz_offset_h is None else 'override: %+g h' % tz_offset_h,
                                'frames_loaded': len(trace.frames), 'duplicates_dropped': dropped}
    else:
        gnotes.append('no trace given: trace checks skipped')
    for w in windows:
        rec = {'kind': w['kind'], 'id': w['id'], 't0': w['t0'], 'start': fmt_t(w['t0']), 'end': fmt_t(w['t1']),
               'violations': [], 'notes': [], 'stats': {}}
        if w['t1'] is None:
            rec['violations'].append(viol('window_incomplete', w['t0'], 1, 0, 'windows',
                                          detail="no 'vehicle %s' line after the start" %
                                                 ('mount -> ride' if w['kind'] == 'mount' else 'exit -> after')))
            report['windows'].append(rec)
            continue
        if w['kind'] == 'mount':
            rec['key_times'] = {'mount_to_ride': fmt_t(w['t_ride'])}
            lv, ln = check_log_mount(w, lines, cfg)
        else:
            rec['key_times'] = {'unmount': fmt_t(w['t_unmount'])}
            lv, ln = check_log_exit(w, lines, cfg)
        rec['violations'] += lv
        rec['notes'] += ln
        if trace is not None:
            if w['kind'] == 'mount':
                tv, tn, st = check_trace_mount(trace, w, cfg, scale_on, feed, lines)
            else:
                tv, tn, st = check_trace_exit(trace, w, cfg, scale_on)
            if allow_uncovered:
                tv = [v for v in tv if v['check'] != 'trace_coverage']
            rec['violations'] += tv
            rec['notes'] += tn
            rec['stats'] = st
        rec['violations'].sort(key=lambda v: v['t'])
        rec['summary'] = summarize(rec['violations'])
        rec['pass'] = not rec['violations']
        report['windows'].append(rec)
    report['violation_count'] = sum(len(w['violations']) for w in report['windows'])
    report['pass'] = report['violation_count'] == 0
    return report


def format_report(report, max_listed=3):
    out = []
    out.append('bt_gate: log=%s' % report['log'])
    out.append('         trace=%s stage=%s dt_scale=%s' % (report['trace'], report['stage'], report['dt_scale']))
    if report.get('feed'):
        out.append('         feed=%s' % report['feed'])
    ti = report.get('trace_info')
    if ti:
        out.append('         time anchor: %s; %d frames loaded, %d duplicate sample(s) dropped' %
                   (ti['tz'], ti['frames_loaded'], ti['duplicates_dropped']))
    for n in report['notes']:
        out.append('  note: ' + n)
    for w in report['windows']:
        status = 'PASS' if w['pass'] else 'FAIL'
        kt = ', '.join('%s %s' % (k, v) for k, v in w.get('key_times', {}).items())
        out.append('')
        out.append('[%s] %s %s  %s .. %s  (%s)' % (status, w['kind'], w['id'], w['start'], w['end'], kt))
        if w['stats']:
            out.append('  stats: ' + ', '.join('%s=%s' % (k, v) for k, v in w['stats'].items()))
        for s in w.get('summary', []):
            worst = s['worst']
            unit = worst['unit']
            bone = (' ' + worst['bone']) if worst.get('bone') else ''
            line = '  FAIL %-22s x%-3d first %s, worst %s %s%s @ %s (limit %s)' % (
                s['check'], s['count'], s['first'], worst['value'], unit, bone, worst['time'], worst['limit'])
            out.append(line)
            if worst.get('detail'):
                out.append('       ' + worst['detail'])
            if s['count'] > 1:
                times = list(dict.fromkeys(v['time'] for v in w['violations'] if v['check'] == s['check']))
                out.append('       at ' + ', '.join(times[:max_listed + 3]) + (' ...' if len(times) > max_listed + 3 else ''))
        for n in w['notes']:
            out.append('  note: ' + n)
    out.append('')
    out.append('RESULT: %s (%d violation(s) in %d window(s))' % ('PASS' if report['pass'] else 'FAIL',
                                                                  report['violation_count'], len(report['windows'])))
    return '\n'.join(out)


def build_parser():
    ap = argparse.ArgumentParser(description='UTPP vehicle mount / exit body-trace regression gate')
    ap.add_argument('--log', required=True, help='utpp-*.log')
    ap.add_argument('--trace', help='utpp-bodytrace-*.csv (omit for log-only checks)')
    ap.add_argument('--feed', help='utpp-feed-*.txt of the session: mount Hips step and foot_skate in world space')
    ap.add_argument('--json', help="write the JSON report here ('-' = stdout instead of the text report)")
    ap.add_argument('--tz-offset', type=float, help='local-time offset from UTC in hours (default: derived from the log)')
    ap.add_argument('--stage', choices=('s0', 's1'), default='s1', help='trace stage: s0 before MM, s1 after MM and aim (shown pose)')
    ap.add_argument('--no-dt-scale', action='store_true', help='raw per-sample steps (no scaling of long frames)')
    ap.add_argument('--allow-uncovered', action='store_true', help='do not fail windows the trace does not cover')
    ap.add_argument('--allow-empty', action='store_true', help='exit 0 when the log has no mount / exit window')
    for k, v in DEFAULTS.items():
        ap.add_argument('--' + k.replace('_', '-'), dest='cfg_' + k, type=type(v), default=None, help='default %s' % v)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    cfg = {k: getattr(args, 'cfg_' + k) for k in DEFAULTS if getattr(args, 'cfg_' + k) is not None}
    for p in (args.log, args.trace, args.feed):
        if p and not os.path.isfile(p):
            print('bt_gate: file not found: %s' % p, file=sys.stderr)
            return 2
    try:
        report = run_gate(args.log, args.trace, cfg, args.tz_offset, args.stage, not args.no_dt_scale, args.allow_uncovered,
                          args.feed)
    except (OSError, ValueError, KeyError) as e:
        print('bt_gate: %s' % e, file=sys.stderr)
        return 2
    if not report['windows']:
        report['notes'].append('no mount / exit window found in the log')
        if not args.allow_empty:
            report['pass'] = False
    if args.json == '-':
        print(json.dumps(report, indent=1))
    else:
        print(format_report(report))
        if args.json:
            with open(args.json, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=1)
    if not report['windows'] and not args.allow_empty:
        print('bt_gate: no vehicle window in the log', file=sys.stderr)
        return 2
    return 0 if report['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
