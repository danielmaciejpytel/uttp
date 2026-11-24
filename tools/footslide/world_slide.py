"""World-space planted-foot slide of a recorded game session (pose recording + feed dump + mod log).

  python -I world_slide.py RUN_DIR [RUN_DIR ...] [--json OUT.json] [--md OUT.md] [--jobs N]
  python -I world_slide.py --baseline LIST.json [--night DIR] --json OUT.json --md OUT.md [--md-notes NOTES.md] [--jobs N]

  LIST.json is a JSON list of {"run": <night folder name>, "set": ..., "body": ...}; --night is the folder that holds the runs
  (default Research/logs/night). --json writes the per-run results and the summary pooled per body, --md the report made from the
  same summary (with --md-notes the text of NOTES.md is appended to it as its notes section). --jobs runs the sessions in parallel.

What is measured
  The toe (LeftToeBase / RightToeBase, the lock point of the mod's foot locker) of the final recorded pose is put into the world with the
  capsule of the same frame:  world = capsule position + Rz(actor yaw) * model-space toe  (model +Y = forward, +Z = up; feed dump capX/Y/Z
  and yawDeg). Its horizontal speed is taken on every pair of consecutive frames while the foot is in contact; the distance of a pair is
  divided by the game step (feed dt) of the feed row the later frame is drawn at.
  contact = the contact the mod's foot locker receives. The 'UTPP MOTION' lines of the log (written on every change of the trace key and
            every 0.5 s of feed time, MotionDriver kTracePeriod) give the selected clip and clip time; between two lines the clip time
            advances at the rate the two lines give (loops unwrapped). The value is rebuilt from the database contact curves (CONT section
            of the motion database) as the motion driver makes it on the frames it owns (MotionHost MotionLegContextLocked: the leg context
            takes MotionSetFrame.contact): the live crossfade stack (MotionBlendStack) mixes the contact of every layer, newest last, with
            the layer's whole-layer fade weight smoothstep(clock / blend time) (MotionBlendStack::Alpha with bone -1, factor 1). A clip
            change pushes a layer whose blend time is the logged 'blend'; older layers keep fading (their clocks run 1.1^(i+1) times
            faster), a pick within 0.03 s of the head replaces it. Air frames (air mode != 0), slide frames (slide=1) and dodge clips
            (MOTIONDODGE start .. end + its blend) have no contact; traversal frames are left out.
            Two other gates are reported next to it: 'pose_blend' = the same stack with the leg bones' fade (blend time x 1 / 1.5, the
            per-bone factor of the toe in MotionPose: when the toe of the drawn pose has arrived at the new clip), and 'no_crossfade' =
            the contact of the selected clip alone (the v1 matcher path, MotionHost ComputeLegContextLocked: SampleContact of the played
            frame; its alpha is the fraction towards the next frame, not a crossfade).
  Contact segments are the runs of frames with contact >= 0.5; as in the replay table (FeedReplayMotion FLTAB) 3 frames are trimmed at
  both ends of a segment and segments with fewer than 4 frames left are ignored. A short-lock variant (1 frame trimmed, >= 2 left) is
  reported as well: a run / sprint loop contact lasts 4-9 frames at 60 Hz and never passes the main rule. Per window (a test step or a
  whole scenario) and per frame class (start, stop, pivot, loop, landing, idle, other: from the clip name) the result holds the speed
  mean / p50 / p95 / max (cm/s) and, per lock (one trimmed segment of one foot), the distance the toe travelled while locked (cm).

What is rejected
  Frames without a feed row matched to them, frame pairs whose pose step and feed step differ by more than 3 ms, pairs whose shifted feed
  rows (matched rows + FEED_SHIFT) are not one game step apart (a feed gap: the feed clock t moves by more than dt), intervals whose clip
  time does not advance at a plausible rate (0.2 .. 1.8: a continuing-pose time jump that no log line shows). When the next MOTION line
  is more than 0.8 s away the trace stopped or the mod is not driving: only the 0.5 s trace period after the last line keeps its clip.
  Frame pairs in which the capsule moves faster than 15 m/s (the game placed V: vehicle exit, teleport) are left out. Between two lines
  of the same clip the clip time follows the rate the two lines give (both clip times are logged). The rate a clip keeps after that
  (the next line is another clip, or the clip plays on as an older layer of the stack) comes only from line intervals of at least
  0.1 s and is clamped to the runtime bounds 0.5 .. 1.15 (ChooserMatcher walk / run / crouch loop minimum .. maxPlayRate).
  A window is marked rejected (reason in 'rejected') when less than 90 % of its frames are aligned with the feed or the contact is
  known on less than 80 % of them. The pooled summary leaves out the samples of rejected step and scenario windows; a rejected macro
  window ('~macro', report only: it spans the runner's own walks) is listed but keeps its samples.
  The pose of frame f is put at the capsule of feed row (matched row + 1) (FEED_SHIFT); the mean planted speed of every shift -2 .. 2 is
  reported per run, and the run is flagged when another shift is lower.

The IK correction of the foot locker (MMTRACE corrL / corrR, gt_log footlock_corr_max_cm) is a different quantity: how far the locker
moved the foot. World slide is what remains visible.

Read-only on every input.
"""
from __future__ import annotations

import argparse
import collections
import datetime
import json
import os
import re
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
GAMETEST = os.path.join(TOOLS, 'gametest')
for _p in (TOOLS, GAMETEST):
    if _p not in sys.path:
        sys.path.insert(0, _p)

TOE = ('LeftToeBase', 'RightToeBase')
HEEL = ('LeftHeel', 'RightHeel')
BONES = ('Hips', 'LeftFoot', 'RightFoot') + TOE + HEEL
CONTACT_MIN = 0.5
TRIM = 3                    # frames dropped at both ends of a contact segment (FeedReplayMotion FlFinalize kTrim)
MIN_SEG = 4                 # frames that must remain after the trim (kMinSeg)
SHORT_TRIM, SHORT_MIN_SEG = 1, 2   # short-lock variant: the 4-9 frame run / sprint loop contacts at 60 Hz
RATE_MIN, RATE_MAX = 0.2, 1.8       # plausible clip time advance per second between two lines (outside = a time jump)
PLAY_RATE_MIN, PLAY_RATE_MAX = 0.5, 1.15   # runtime play rate bounds (ChooserMatcher walk / run / crouch loop minimum .. maxPlayRate)
SHORT_INTERVAL_S = 0.1      # line intervals shorter than this give no play rate worth keeping
LINE_GAP_S = 0.8            # no MOTION line for longer = the trace stopped or the mod is not driving
HEARTBEAT_S = 0.5           # MotionDriver kTracePeriod: a line at least every 0.5 s of feed time while the driver runs
LEG_FADE_FACTOR = 1.0 / 1.5     # MotionPose live crossfade profile: UpLeg / Leg / Foot / ToeBase fade in blend time x 1 / 1.5
OLDER_CLOCK = 1.1           # MotionBlendStack::Step: the clock of layer i >= 1 runs 1.1^(i+1) times faster
REPLACE_S = 0.03            # a pick while the head is younger than this replaces the head (no new layer)
STACK_LAYERS = 6            # MotionBlendStack kCapacity
DT_MATCH_S = 0.003          # pose frame step vs feed step of the same frame; feed clock step vs feed dt of a row
TELEPORT_M_S = 15.0         # capsule step speed above which a frame pair is a placement by the game (vehicle exit, teleport), not a step
ALIGNED_MIN = 0.9           # share of aligned frames below which a window is rejected
KNOWN_MIN = 0.8             # share of frames with a known contact below which a window is rejected
LOCK_VALID_MIN = 0.75       # share of valid frame pairs a lock needs
SHIFTS = (-2, -1, 0, 1, 2)
FEED_SHIFT = 1              # pose frame f is drawn at the capsule of feed row (matched row + 1): the feed row holds the capsule before the
                            # move of its tick, and the driver draws the pose with the capsule turned by the coming tick's request
                            # (MotionHost.cpp:1613 raw.capsuleYawAhead); lowest planted speed at +1 in 18 of the 22 baseline runs of
                            # 2026-11-18, ties in the rest
CLASSES = ('start', 'stop', 'pivot', 'loop', 'landing', 'idle', 'other')
GATES = ('runtime', 'pose_blend', 'no_crossfade')
ST_UNKNOWN, ST_OK, ST_BLEND, ST_AIR, ST_TRAV, ST_JUMP, ST_NOCONT = 0, 1, 2, 3, 4, 5, 6
KNOWN_STATES = (ST_OK, ST_BLEND, ST_AIR, ST_NOCONT)


# ------------------------------------------------------------------------------------------------------------------------ database
class ContactDB:
    """Clip table and contact curves of a .utppmm v2 motion database (only the header, the section table and the STRS / CLIP / CONT
    sections are read)."""

    CLIP_DTYPE = np.dtype([('name', '<u4'), ('first', '<u4'), ('num', '<u4'), ('flags', '<u4'), ('loop_period', '<u4'),
                           ('selectable', '<u4'), ('db_mask', '<u8')])

    def __init__(self, path):
        with open(path, 'rb') as f:
            head = f.read(128)
            magic, ver, _ = struct.unpack_from('<8sII', head, 0)
            if magic != b'UTPPMMDB' or ver != 2:
                raise ValueError(f'{path}: not a motion database (v2)')
            self.fps, nsec, table, _ = struct.unpack_from('<fIII', head, 80)
            f.seek(table)
            tab = f.read(32 * nsec)
            secs = {}
            for i in range(nsec):
                sid, cnt, esz, _, off, size = struct.unpack_from('<IIIIQQ', tab, 32 * i)
                secs[struct.pack('<I', sid).decode('latin-1')] = (cnt, esz, off, size)

            def read(name):
                cnt, esz, off, size = secs[name]
                f.seek(off)
                return f.read(size), cnt, esz

            strs, _, _ = read('STRS')
            clip, ncl, cesz = read('CLIP')
            cont, ncont, kesz = read('CONT')
        if cesz != 32 or kesz != 8:
            raise ValueError(f'{path}: unexpected CLIP / CONT record size {cesz} / {kesz}')
        rec = np.frombuffer(clip, dtype=self.CLIP_DTYPE, count=ncl)
        self.names = [strs[o:strs.index(b'\0', o)].decode('utf-8', 'replace') for o in rec['name']]
        self.index = {n: i for i, n in enumerate(self.names)}
        self.first = rec['first'].astype(np.int64)
        self.num = rec['num'].astype(np.int64)
        self.loop = (rec['flags'] & 1) != 0
        self.cont = np.frombuffer(cont, dtype='<f4', count=ncont * 2).reshape(ncont, 2).astype(np.float64)
        self.path = path

    @classmethod
    def from_arrays(cls, names, num, loop, cont, fps=30.0):
        """A small database for tests: cont = [clip][frame][2]."""
        self = cls.__new__(cls)
        self.fps = float(fps)
        self.names = list(names)
        self.index = {n: i for i, n in enumerate(self.names)}
        self.num = np.asarray(num, dtype=np.int64)
        self.first = np.concatenate([[0], np.cumsum(self.num)[:-1]]).astype(np.int64)
        self.loop = np.asarray(loop, dtype=bool)
        self.cont = np.concatenate([np.asarray(c, dtype=np.float64).reshape(-1, 2) for c in cont])
        self.path = None
        return self

    def length(self, clip):
        return max(0.0, (self.num[clip] - 1) / self.fps)

    def resolve(self, idx, name):
        """Clip index of a log line: by name (the log index is cross-checked; a mismatch takes the name)."""
        if 0 <= idx < len(self.names) and self.names[idx] == name:
            return idx
        return self.index.get(name, -1)

    def contact(self, clip, ct):
        """Contact (l, r) of clip at clip times ct (array, seconds): linear between the 30 Hz frames, loops wrapped, others clamped."""
        ct = np.asarray(ct, dtype=np.float64)
        n = int(self.num[clip])
        local = np.nan_to_num(ct, nan=0.0) * self.fps
        if self.loop[clip] and n > 1:
            local = np.mod(local, n - 1)
        local = np.clip(local, 0.0, n - 1)
        i0 = np.floor(local).astype(np.int64)
        i0 = np.minimum(i0, n - 1)
        i1 = np.minimum(i0 + 1, n - 1)
        u = (local - i0)[:, None]
        a = self.cont[self.first[clip] + i0]
        b = self.cont[self.first[clip] + i1]
        return a + (b - a) * u


def default_db_path(body):
    from utpp_paths import project_root
    return os.path.join(project_root(), 'Data', f'utpp_motion_{body}.utppmm')


_DB_CACHE = {}


def load_db(path):
    if path not in _DB_CACHE:
        _DB_CACHE[path] = ContactDB(path)
    return _DB_CACHE[path]


# ------------------------------------------------------------------------------------------------------------------------ clip classes
def clip_class(name: str) -> str:
    n = (name or '').lower()
    if not n or n == '-':
        return 'other'
    if 'land' in n:
        return 'landing'
    if 'pivot' in n or 'reface' in n or '_turn_' in n or 'turninplace' in n or 'turn_in_place' in n:
        return 'pivot'
    if '_start' in n:
        return 'start'
    if '_stop' in n:
        return 'stop'
    if 'idle' in n:
        return 'idle'
    if 'loop' in n:
        return 'loop'
    return 'other'


def fl_key(name: str) -> str:
    """The replay table key of a clip (FeedReplayMotion FlKey): <gait>/<type>."""
    has = lambda s: s in name  # noqa: E731
    gait = ('Sprint' if has('Sprint') else 'Crouch' if has('Crouch') else 'Run' if has('Run_') else 'Walk' if has('Walk')
            else 'Idle' if has('Idle') else 'Trans' if has('Transition') else 'Other')
    typ = ('Reface' if has('Reface') else 'Pivot' if has('Pivot') else 'TIP' if has('TurnInPlace') else 'Start' if has('_Start')
           else 'Stop' if has('_Stop') else 'Turn' if has('_Turn_') else 'Loop' if has('Loop') else 'Idle' if has('Idle')
           else 'Trans' if has('Transition') else 'Other')
    return gait + '/' + typ


# ------------------------------------------------------------------------------------------------------------------------ log
WALL_RE = re.compile(r'^\[(\d{4})-(\d\d)-(\d\d) (\d\d):(\d\d):(\d\d)\.(\d+)\]')
MOTION_LINE_RE = re.compile(r'^\[(\d{4})-(\d\d)-(\d\d) (\d\d):(\d\d):(\d\d)\.(\d+)\].*?UTPP MOTION t=([\d.]+) db=(-?\d+):(\S+) '
                            r'clip=(-?\d+):(\S+) ct=(-?[\d.]+) cost=\S+ blend=([\d.]+)(\(live\))?')
AIR_RE = re.compile(r' air=mode(\d+),ttl[-\d.]+,land(\d)(\d)')
TRAV_RE = re.compile(r' trav=(-?\d+):')
SLIDE_RE = re.compile(r' slide=(\d)/')
DODGE_RE = re.compile(r'UTPP MOTIONDODGE (start|end) t=([\d.]+)(?:.*? blend=([\d.]+))?')


def _wall(m):
    y, mo, d, hh, mi, ss, frac = m.groups()[:7]
    return datetime.datetime(int(y), int(mo), int(d), int(hh), int(mi), int(ss), int((frac + '000000')[:6]))


def parse_motion_lines(lines):
    """MOTION lines -> [dict(kind 'motion', wall, t, clip_idx, clip, ct, blend, live, air, trav, slide, chg)] and the dodge clip lines
    -> dict(kind 'dodge_start' / 'dodge_end', wall, t, blend), in log order."""
    out = []
    for ln in lines:
        if 'UTPP MOTION' not in ln:
            continue
        if 'UTPP MOTION t=' in ln:
            m = MOTION_LINE_RE.match(ln)
            if not m:
                continue
            a = AIR_RE.search(ln)
            tr = TRAV_RE.search(ln)
            sl = SLIDE_RE.search(ln)
            out.append(dict(kind='motion', wall=_wall(m), t=float(m.group(8)), clip_idx=int(m.group(11)), clip=m.group(12),
                            ct=float(m.group(13)), blend=float(m.group(14)), live=m.group(15) is not None,
                            air=int(a.group(1)) if a else 0, trav=int(tr.group(1)) if tr else 0, slide=int(sl.group(1)) if sl else 0,
                            chg=' chg' in ln))
        elif 'UTPP MOTIONDODGE ' in ln:
            d = DODGE_RE.search(ln)
            w = WALL_RE.match(ln)
            if d and w and d.group(1) in ('start', 'end'):
                out.append(dict(kind='dodge_' + d.group(1), wall=_wall(w), t=float(d.group(2)),
                                blend=float(d.group(3)) if d.group(3) else 0.0))
    return out


def read_motion_log(path):
    import pose_metrics as pm
    with pm.open_log_parts(path) as f:
        return parse_motion_lines(f)


# ------------------------------------------------------------------------------------------------------------------------ contact track
Track = collections.namedtuple('Track', 'cont cont_pose cont_cut state clip_of notes')


def _smooth(x):
    t = np.clip(x, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _clamp_rate(r):
    return float(min(max(r, PLAY_RATE_MIN), PLAY_RATE_MAX))


def _interval_rates(db, ev):
    """Play rate of each interval between two lines of the same clip: a float (kept), ('short', r) (interval under SHORT_INTERVAL_S:
    used for itself, not kept), 'jump' (a continuing-pose time jump) or None (no estimate)."""
    rate_of = [None] * len(ev)
    for i in range(len(ev) - 1):
        a, b = ev[i], ev[i + 1]
        if a['c'] < 0 or a['c'] != b['c']:
            continue
        dtr = b['t'] - a['t']
        if dtr <= 1e-4 or b['s'] - a['s'] > LINE_GAP_S:
            continue
        d = b['ct'] - a['ct']
        if db.loop[a['c']] and db.length(a['c']) > 0:
            L = db.length(a['c'])
            k = 0
            while d < RATE_MIN * dtr - 0.02 and k < 8:
                d += L
                k += 1
        r = d / dtr
        if dtr < SHORT_INTERVAL_S:               # two lines a few frames apart: only a gross jump can be told
            rate_of[i] = ('short', r) if -0.05 <= d <= RATE_MAX * dtr + 0.05 else 'jump'
        else:
            rate_of[i] = r if RATE_MIN <= r <= RATE_MAX else 'jump'
    return rate_of


def contact_track(db: ContactDB, events, fsec, sec_of) -> Track:
    """Per frame: contact [N, 2] of the three gates (runtime, pose_blend, no_crossfade; nan = unknown), state [N] (ST_*), clip index
    [N] (-1 = unknown) and a note dict. fsec = frame times (s, sorted); sec_of(datetime) -> the same time base."""
    N = len(fsec)
    fsec = np.asarray(fsec, dtype=np.float64)
    if N > 1 and np.any(np.diff(fsec) < 0):
        raise ValueError('frame times must be sorted')
    cont = np.full((N, 2), np.nan)
    cont_pose = np.full((N, 2), np.nan)
    cont_cut = np.full((N, 2), np.nan)
    state = np.zeros(N, dtype=np.int8)
    clip_of = np.full(N, -1, dtype=np.int64)
    motion = [e for e in events if e.get('kind', 'motion') == 'motion']
    dodges = [e for e in events if e.get('kind') in ('dodge_start', 'dodge_end')]
    notes = dict(lines=len(motion), unknown_clip=0, ct_jumps=0, blends=0, replaced=0, max_layers=0, not_live=0,
                 short_rates=0, rates_clamped=0, dodge_windows=0, slide_lines=0)
    ev = []
    for e in motion:
        c = db.resolve(e['clip_idx'], e['clip'])
        if c < 0:
            notes['unknown_clip'] += 1
        if not e.get('live', True):
            notes['not_live'] += 1
        ev.append(dict(e, c=c, s=sec_of(e['wall'])))
    rate_of = _interval_rates(db, ev)
    last_rate = {}
    layers = []                                  # newest first: dict(c, ct, rate, dur, clock) at the time of the current line
    for i, e in enumerate(ev):
        t_a = e['s']
        nxt = ev[i + 1] if i + 1 < len(ev) else None
        t_b = nxt['s'] if nxt is not None and nxt['s'] - t_a <= LINE_GAP_S else t_a + HEARTBEAT_S
        contiguous = i > 0 and t_a - ev[i - 1]['s'] <= LINE_GAP_S
        f0, f1 = int(np.searchsorted(fsec, t_a)), int(np.searchsorted(fsec, t_b))
        c = e['c']
        r = rate_of[i]
        if r == 'jump':
            notes['ct_jumps'] += 1
            if c >= 0 and f1 > f0:
                state[f0:f1] = ST_JUMP
                clip_of[f0:f1] = c
            layers = []                          # the runtime pushed a layer of the same clip at a time no line shows
            continue
        # r: the rate of this interval (both of its clip times are logged, so the measured rate reproduces the next line's ct);
        # keep: the rate the clip plays at after it (clamped to the runtime bounds; a short interval's rate is never kept)
        if isinstance(r, tuple):
            notes['short_rates'] += 1
            r = r[1]
            keep = last_rate.get(c, 1.0)
        elif r is None:
            r = keep = last_rate.get(c, 1.0)
        else:
            keep = _clamp_rate(r)
            if keep != r:
                notes['rates_clamped'] += 1
            last_rate[c] = keep
        if c < 0:
            layers = []
            continue
        if not contiguous or not layers:
            layers = [dict(c=c, ct=e['ct'], rate=r, dur=0.0, clock=0.0)]
        elif layers[0]['c'] != c:                # clip change: a new layer over the stack
            new = dict(c=c, ct=e['ct'], rate=r, dur=e['blend'] if e['blend'] > 1e-3 else 0.0, clock=0.0)
            head = layers[0]
            if head['clock'] < head['dur'] and head['clock'] < REPLACE_S:
                layers[0] = new
                notes['replaced'] += 1
            else:
                layers.insert(0, new)
                if new['dur'] > 0:
                    notes['blends'] += 1
            del layers[STACK_LAYERS:]
        else:
            layers[0]['ct'] = e['ct']             # the selected clip time is authoritative for the head
            layers[0]['rate'] = r
        for k in range(len(layers) - 1):         # relevance pruning: a fully faded-in layer hides everything below it
            if layers[k]['dur'] <= 1e-8 or layers[k]['clock'] >= layers[k]['dur']:
                del layers[k + 1:]
                break
        notes['max_layers'] = max(notes['max_layers'], len(layers))
        if f1 > f0:
            ts = fsec[f0:f1] - t_a
            samp = [db.contact(L['c'], L['ct'] + ts * L['rate']) for L in layers]
            fac = [1.0] + [OLDER_CLOCK ** (k + 1) for k in range(1, len(layers))]
            sig, sig_pose = samp[-1].copy(), samp[-1].copy()
            a0 = np.ones(len(ts))
            for k in range(len(layers) - 2, -1, -1):
                L = layers[k]
                clk = L['clock'] + ts * fac[k]
                a = _smooth(clk / L['dur']) if L['dur'] > 1e-8 else np.ones(len(ts))
                ap = _smooth(clk / (L['dur'] * LEG_FADE_FACTOR)) if L['dur'] > 1e-8 else np.ones(len(ts))
                sig = sig + (samp[k] - sig) * a[:, None]
                sig_pose = sig_pose + (samp[k] - sig_pose) * ap[:, None]
                if k == 0:
                    a0 = a
            st = np.where(a0 < 1.0, ST_BLEND, ST_OK).astype(np.int8)
            cut = samp[0]
            if e['air'] or e.get('slide'):
                sig = np.zeros_like(sig)
                sig_pose = np.zeros_like(sig_pose)
                cut = np.zeros_like(cut)
                st[:] = ST_AIR if e['air'] else ST_NOCONT
                if e.get('slide'):
                    notes['slide_lines'] += 1
            if e['trav']:
                st[:] = ST_TRAV
            cont[f0:f1] = sig
            cont_pose[f0:f1] = sig_pose
            cont_cut[f0:f1] = cut
            state[f0:f1] = st
            clip_of[f0:f1] = c
        dt_line = t_b - t_a                      # advance the stack to the next line
        for k, L in enumerate(layers):
            L['ct'] += dt_line * L['rate']
            L['clock'] += dt_line * (1.0 if k == 0 else OLDER_CLOCK ** (k + 1))
        if layers:
            layers[0]['rate'] = keep             # under a new pick the head plays on at the kept rate
    start = None                                 # dodge clips and their hand-back blend: no contact
    for d in dodges:
        s = sec_of(d['wall'])
        if d['kind'] == 'dodge_start':
            start = s
        elif start is not None:
            f0, f1 = int(np.searchsorted(fsec, start)), int(np.searchsorted(fsec, s + d['blend']))
            known = np.isin(state[f0:f1], (ST_OK, ST_BLEND))
            for arr in (cont, cont_pose, cont_cut):
                sub = arr[f0:f1]
                sub[known] = 0.0
                arr[f0:f1] = sub
            stt = state[f0:f1]
            stt[known] = ST_NOCONT
            state[f0:f1] = stt
            notes['dodge_windows'] += 1
            start = None
    return Track(cont, cont_pose, cont_cut, state, clip_of, notes)


# ------------------------------------------------------------------------------------------------------------------------ world points
def world_xy(model_xyz, cap, yaw_deg):
    """Horizontal world position (m) of model-space points: capsule xy + Rz(yaw) * model xy."""
    a = np.radians(yaw_deg)
    c, s = np.cos(a), np.sin(a)
    x = c * model_xyz[:, 0] - s * model_xyz[:, 1] + cap[:, 0]
    y = s * model_xyz[:, 0] + c * model_xyz[:, 1] + cap[:, 1]
    return np.stack([x, y], axis=1)


def frame_rows(rows_to_frame, n_frames):
    """Inverse of the alignment: feed row of every pose frame (-1 = none)."""
    out = np.full(n_frames, -1, dtype=np.int64)
    if rows_to_frame is None:
        return out
    j = np.nonzero((rows_to_frame >= 0) & (rows_to_frame < n_frames))[0]
    out[rows_to_frame[j]] = j
    return out


class Session:
    """Per-frame inputs of one recording: frame times, model-space toe positions [2][N, 3], feed rows, capsule / yaw, contact of the
    gates (cont = runtime; cont_pose / cont_cut default to it)."""

    def __init__(self, fsec, toe, feed, rows, cont, state, clip_of, clip_names, heel=None, cont_pose=None, cont_cut=None):
        self.fsec = np.asarray(fsec, dtype=np.float64)
        self.toe = toe
        self.heel = heel
        self.feed = feed
        self.rows = rows
        self.cont = cont
        self.gate_cont = dict(runtime=cont, pose_blend=cont if cont_pose is None else cont_pose,
                              no_crossfade=cont if cont_cut is None else cont_cut)
        self.state = state
        self.clip_of = clip_of
        self.clip_names = clip_names
        n = len(self.fsec)
        self.active = np.ones(n, dtype=bool)      # frames a window's shares are taken over
        cls_of = {i: clip_class(nm) for i, nm in enumerate(clip_names)}
        key_of = {i: fl_key(nm) for i, nm in enumerate(clip_names)}
        self.cls = np.array([cls_of.get(int(c), 'other') if c >= 0 else 'other' for c in clip_of]) if n else np.array([], dtype=str)
        self.key = np.array([key_of.get(int(c), 'Other/Other') if c >= 0 else 'Other/Other' for c in clip_of]) if n else np.array([], dtype=str)
        # frame pair validity (pair k = frames k-1 -> k): both frames matched to consecutive feed rows, steps agree
        pdt = np.diff(self.fsec, prepend=np.nan)
        fdt = np.full(n, np.nan)
        ok = rows >= 0
        fdt[ok] = feed['dt'][rows[ok]]
        prev_ok = np.zeros(n, dtype=bool)
        prev_ok[1:] = ok[1:] & ok[:-1] & (rows[1:] == rows[:-1] + 1)
        nrow = len(feed['dt'])
        # feed rows one game step after the previous row (the feed clock t moves by dt): a row after a feed gap is not
        row_bad = np.ones(nrow, dtype=bool)
        if nrow > 1:
            t = feed.get('t')
            if t is not None and len(t) == nrow:
                with np.errstate(invalid='ignore'):
                    row_bad[1:] = ~(np.abs(np.diff(t) - np.asarray(feed['dt'][1:], dtype=np.float64)) <= DT_MATCH_S)
            else:
                row_bad[1:] = False
        self.row_bad_cum = np.cumsum(row_bad)
        # placement by the game: a feed row step faster than TELEPORT_M_S anywhere within the rows a shift of SHIFTS can draw on
        jump = np.zeros(nrow + 1, dtype=bool)
        if nrow > 1:
            with np.errstate(invalid='ignore', divide='ignore'):
                step = np.hypot(np.diff(feed['capX']), np.diff(feed['capY'])) / np.maximum(np.asarray(feed['dt'][1:], dtype=np.float64), 1e-3)
            jump[1:nrow] = step > TELEPORT_M_S
        near = np.zeros(n, dtype=bool)
        lo, hi = min(SHIFTS), max(SHIFTS)
        for s in range(lo, hi + 1):
            q = rows + s
            v = ok & (q >= 1) & (q < nrow)
            near[v] |= jump[q[v]]
        self.teleport = prev_ok & near
        with np.errstate(invalid='ignore'):
            self.pair_ok = prev_ok & (np.abs(pdt - fdt) <= DT_MATCH_S) & ~self.teleport
        self.pdt = pdt
        self.aligned = ok
        self.shift = 0
        self.shift_scores = None
        self.shift_flag = None

    def world(self, side, shift=None):
        shift = self.shift if shift is None else shift
        n = len(self.fsec)
        out = np.full((n, 2), np.nan)
        r = self.rows + shift
        ok = (self.rows >= 0) & (r >= 0) & (r < len(self.feed['dt']))
        cap = np.stack([self.feed['capX'][r[ok]], self.feed['capY'][r[ok]]], axis=1)
        out[ok] = world_xy(self.toe[side][ok], cap, self.feed['yawDeg'][r[ok]])
        return out

    def step_dt(self, shift):
        """Game step of pair k: feed dt of the row frame k is drawn at (matched row + shift); nan without one."""
        n = len(self.fsec)
        out = np.full(n, np.nan)
        q = self.rows + shift
        ok = (self.rows >= 0) & (q >= 0) & (q < len(self.feed['dt']))
        out[ok] = self.feed['dt'][q[ok]]
        return out

    def shift_ok(self, shift):
        """Pair k is drawn on feed rows (rows[k-1] + shift, rows[k] + shift): every row between the matched and the shifted rows of the
        pair must follow its predecessor by one game step (no feed gap in between)."""
        nrow = len(self.feed['dt'])
        r = self.rows
        lo = r + min(0, shift)                   # first row whose step into it the pair spans (the step from row lo - 1)
        hi = r + max(0, shift)
        out = np.zeros(len(r), dtype=bool)
        v = (r >= 1) & (lo >= 1) & (hi < nrow)
        out[v] = (self.row_bad_cum[hi[v]] - self.row_bad_cum[lo[v] - 1]) == 0
        return out

    def speeds(self, side, shift=None):
        """cm/s of frame pair k (frames k-1 -> k); nan where the pair is not valid."""
        shift = self.shift if shift is None else shift
        w = self.world(side, shift)
        sp = np.full(len(self.fsec), np.nan)
        d = np.linalg.norm(np.diff(w, axis=0), axis=1) * 100.0
        dt = self.step_dt(shift)
        with np.errstate(invalid='ignore', divide='ignore'):
            sp[1:] = d / dt[1:]
        bad = ~(self.pair_ok & self.shift_ok(shift) & (dt > 1e-4))
        sp[bad] = np.nan
        dist = np.concatenate([[np.nan], d])
        dist[bad] = np.nan
        return sp, dist

    def locks(self, side, gate='runtime'):
        """Contact segments of one foot: [(i, j)] frame ranges with contact >= CONTACT_MIN and a known contact state."""
        c = self.gate_cont[gate][:, side]
        known = np.isin(self.state, KNOWN_STATES)
        on = known & (np.nan_to_num(c, nan=0.0) >= CONTACT_MIN)
        out = []
        i, n = 0, len(on)
        while i < n:
            if not on[i]:
                i += 1
                continue
            j = i
            while j < n and on[j]:
                j += 1
            out.append((i, j))
            i = j
        return out

    def choose_shift(self, fixed=None):
        """Integer pose / feed row shift with the lowest mean planted speed on the trimmed contact frames of the moving clips (every
        class but idle: a standing pose does not tell the shifts apart). fixed = use that shift, the scores are still reported."""
        scores = {}
        for s in SHIFTS:
            vals = []
            for side in (0, 1):
                sp, _ = self.speeds(side, s)
                for i, j in self.locks(side):
                    if j - i < 2 * TRIM + MIN_SEG:
                        continue
                    ks = np.arange(i + TRIM + 1, j - TRIM)
                    ks = ks[self.cls[ks] != 'idle']
                    v = sp[ks]
                    vals.append(v[np.isfinite(v)])
            v = np.concatenate(vals) if vals else np.array([])
            scores[s] = (float(np.mean(v)) if len(v) else None, int(len(v)))
        good = {s: m for s, (m, n) in scores.items() if m is not None and n >= 200}
        self.shift_scores = {str(s): [None if m is None else round(m, 2), n] for s, (m, n) in scores.items()}
        if fixed is not None:
            self.shift = fixed
            best = min(good, key=good.get) if good else None
            self.shift_flag = None if best in (None, fixed) else f'lowest mean at shift {best} ({good[best]:.1f} vs {good.get(fixed, float("nan")):.1f} cm/s)'
            return self.shift
        if not good:
            self.shift, self.shift_flag = 0, 'no steady loop contact: shift 0 assumed'
            return self.shift
        order = sorted(good, key=good.get)
        self.shift = order[0]
        if len(order) > 1 and good[order[1]] < 1.05 * good[order[0]]:
            self.shift_flag = f'ambiguous: shift {order[0]} {good[order[0]]:.1f} vs {order[1]} {good[order[1]]:.1f} cm/s'
        return self.shift

    # ---- samples
    def samples(self, gate='runtime', trim=TRIM, min_seg=MIN_SEG):
        """All trimmed contact samples and locks of both feet with the current shift (gate: runtime / pose_blend / no_crossfade;
        trim / min_seg: frames dropped at each end of a segment / frames that must remain).
        Returns (samples dict of arrays: frame, side, speed, cls, key, blend), locks list of dicts."""
        fr, sd, spd, bl = [], [], [], []
        locks = []
        for side in (0, 1):
            sp, dist = self.speeds(side)
            wpos = self.world(side)
            for i, j in self.locks(side, gate):
                if j - i < 2 * trim + min_seg:
                    continue
                ks = np.arange(i + trim + 1, j - trim)
                v = sp[ks]
                okk = np.isfinite(v)
                if okk.sum() < LOCK_VALID_MIN * len(ks):
                    continue
                fr.append(ks[okk])
                sd.append(np.full(int(okk.sum()), side))
                spd.append(v[okk])
                bl.append(self.state[ks[okk]] == ST_BLEND)
                cls_vals, counts = np.unique(self.cls[ks], return_counts=True)
                net = float(np.linalg.norm(wpos[ks[-1]] - wpos[ks[0] - 1]) * 100.0) if okk.all() else None
                locks.append(dict(side=side, first=int(ks[0]) - 1, last=int(ks[-1]), mid=int((ks[0] + ks[-1]) // 2),
                                  cm=float(np.nansum(dist[ks])), net_cm=net, frames=int(len(ks)),
                                  cls=str(cls_vals[int(np.argmax(counts))]), key=str(self.key[ks[len(ks) // 2]])))
        if fr:
            f = np.concatenate(fr)
            smp = dict(frame=f, side=np.concatenate(sd), speed=np.concatenate(spd), blend=np.concatenate(bl), cls=self.cls[f], key=self.key[f])
        else:
            smp = dict(frame=np.array([], dtype=np.int64), side=np.array([], dtype=np.int64), speed=np.array([]), blend=np.array([], dtype=bool),
                       cls=np.array([], dtype=str), key=np.array([], dtype=str))
        return smp, locks

    def variants(self):
        """The reported sample sets: {name: (samples, locks)}; 'main' = runtime gate with the replay table's lock rule."""
        return dict(main=self.samples(), pose_blend=self.samples('pose_blend'), no_crossfade=self.samples('no_crossfade'),
                    short_locks=self.samples('runtime', SHORT_TRIM, SHORT_MIN_SEG))


def _stats(v):
    v = np.asarray(v, dtype=np.float64)
    if not len(v):
        return dict(n=0, mean=None, p50=None, p95=None, max=None)
    return dict(n=int(len(v)), mean=round(float(np.mean(v)), 2), p50=round(float(np.percentile(v, 50)), 2),
                p95=round(float(np.percentile(v, 95)), 2), max=round(float(np.max(v)), 2))


def _lock_stats(locks):
    """cm: toe path while locked; net_cm: start-to-end displacement (locks with every pair valid)."""
    cms = np.asarray([L['cm'] for L in locks], dtype=np.float64)
    nets = np.asarray([L['net_cm'] for L in locks if L.get('net_cm') is not None], dtype=np.float64)
    out = dict(locks=int(len(cms)), cm_mean=None, cm_p95=None, cm_max=None, net_cm_mean=None, net_cm_p95=None)
    if len(cms):
        out.update(cm_mean=round(float(np.mean(cms)), 2), cm_p95=round(float(np.percentile(cms, 95)), 2), cm_max=round(float(np.max(cms)), 2))
    if len(nets):
        out.update(net_cm_mean=round(float(np.mean(nets)), 2), net_cm_p95=round(float(np.percentile(nets, 95)), 2))
    return out


def window_rejection(ses: Session, i0, i1):
    """(aligned share, known share, reason or None) of frames [i0, i1) (shares over the session's active frames)."""
    act = ses.active[i0:i1]
    n = int(act.sum())
    if not n:
        return 0.0, 0.0, 'no active frames'
    aligned = float(np.mean(ses.aligned[i0:i1][act]))
    known = float(np.mean(np.isin(ses.state[i0:i1][act], KNOWN_STATES)))
    why = []
    if aligned < ALIGNED_MIN:
        why.append(f'only {aligned:.0%} of the frames aligned with the feed')
    if known < KNOWN_MIN:
        why.append(f'contact known on only {known:.0%} of the frames')
    return aligned, known, '; '.join(why) or None


def window_stats(ses: Session, smp, locks, i0, i1, variants=None):
    """World slide of frames [i0, i1): overall, per class and per replay key; variants = {name: (samples, locks)} adds a compact block
    per name (overall and per class)."""
    n = max(0, i1 - i0)
    inw = (smp['frame'] >= i0) & (smp['frame'] < i1)
    lk = [L for L in locks if i0 <= L['mid'] < i1]
    aligned, known, why = window_rejection(ses, i0, i1) if n else (0.0, 0.0, 'empty window')
    res = dict(frames=n, aligned_share=round(aligned, 3), contact_known_share=round(known, 3), rejected=why, shift=ses.shift)
    sp = smp['speed'][inw]
    res.update({'speed_' + k: v for k, v in _stats(sp).items()})
    bl = smp['blend'][inw]
    res['blend_share'] = round(float(np.mean(bl)), 3) if inw.any() else None
    res['speed_steady_p95'] = _stats(sp[~bl])['p95']      # outside a clip crossfade: the contact curve is exact there
    res.update(_lock_stats(lk))
    by = {}
    for c in CLASSES:
        m = smp['cls'][inw] == c
        lc = [L for L in lk if L['cls'] == c]
        if m.any() or lc:
            d = {'speed_' + k: v for k, v in _stats(sp[m]).items()}
            d['speed_steady_p95'] = _stats(sp[m & ~bl])['p95']
            d.update(_lock_stats(lc))
            by[c] = d
    res['by_class'] = by
    keys = {}
    for k in sorted(set(smp['key'][inw].tolist())):
        m = smp['key'][inw] == k
        keys[k] = {'speed_' + a: b for a, b in _stats(sp[m]).items() if a in ('n', 'mean', 'p95', 'max')}
    res['by_key'] = keys
    for name, (vs, vl) in (variants or {}).items():
        if name == 'main':
            continue
        m = (vs['frame'] >= i0) & (vs['frame'] < i1)
        vlk = [L for L in vl if i0 <= L['mid'] < i1]
        blk = {'speed_' + k: v for k, v in _stats(vs['speed'][m]).items() if k in ('n', 'p50', 'p95', 'max')}
        blk.update({k: v for k, v in _lock_stats(vlk).items() if k in ('locks', 'cm_mean', 'net_cm_mean')})
        bc = {}
        for c in CLASSES:
            mc = m & (vs['cls'] == c)
            lc = [L for L in vlk if L['cls'] == c]
            if mc.any() or lc:
                bc[c] = {'speed_' + k: v for k, v in _stats(vs['speed'][mc]).items() if k in ('n', 'p50', 'p95', 'max')}
                bc[c]['locks'] = len(lc)
        blk['by_class'] = bc
        res[name] = blk
    return res


# ------------------------------------------------------------------------------------------------------------------------ session from files
def load_points(pose_files, tz_hours=None):
    """Final-stage model-space positions of BONES over every rig topology of the capture (pose_metrics.load_recording reads only the last
    one; a session that changes the rig, e.g. a melee weapon set, holds several), joined in time order.
    Returns (Recording of the longest part, sec [N], {bone: [N, 3]}, parts [(frames, rig_count)])."""
    import pose_metrics as pm
    import pose_helper_report as phr
    files = [pose_files] if isinstance(pose_files, str) else list(pose_files)
    stamped = all(re.search(r'\d{4}-\d\d-\d\d-\d\d-\d\d-\d\d', os.path.basename(f)) for f in files)
    tz = tz_hours * 3600.0 if tz_hours is not None else None
    if tz is None and not stamped:
        tz = pm.system_offset_seconds(phr.read_header(files[0]))
    groups = phr.load_groups(files, phr.Config(tz_seconds=tz))
    parts = []
    for g in groups:
        rec = pm.Recording.from_group(g, g.setdefault('notes', []))
        need = set()
        for n in BONES:
            if n in rec.cat:
                need.update(pm._chain_to_root(rec.parent, rec.cat[n]))
        keep = sorted(need)
        secs, qs, ts = [], [], []
        for stream, ticks, sec, gidx, v in phr.iter_blocks(g):
            if stream != 0:
                continue
            sel = v[:, keep]
            secs.append(sec)
            ts.append(sel[..., 0:3].astype(np.float32))
            qs.append(pm.qnorm(sel[..., 4:8].astype(np.float64)).astype(np.float32))
        if not secs:
            continue
        data = dict(sec=np.concatenate(secs), q=np.concatenate(qs), t=np.concatenate(ts), keep=keep)
        fk = pm.forward_kinematics(rec, data, 1, BONES)
        if any(b not in fk for b in TOE):
            continue
        parts.append((rec, data['sec'], {b: fk[b][1].astype(np.float64) for b in fk}))
    if not parts:
        raise ValueError('no frames with the toe bones in the recording')
    day0 = parts[0][0].day
    secs = [p[1] + (p[0].day - day0).total_seconds() for p in parts]
    order = np.argsort(np.concatenate(secs), kind='stable')
    sec = np.concatenate(secs)[order]
    pts = {}
    for b in BONES:
        if all(b in p[2] for p in parts):
            pts[b] = np.concatenate([p[2][b] for p in parts])[order]
    main = max(parts, key=lambda p: len(p[1]))[0]
    main.day = day0
    keep_t = np.concatenate([[True], np.diff(sec) > 1e-6])     # overlapping parts: one frame per time stamp
    sec = sec[keep_t]
    pts = {b: v[keep_t] for b, v in pts.items()}
    return main, sec, pts, [(len(p[1]), p[0].rig_count) for p in parts]


def build_session(pose_files, log_path, feed_path, body=None, db_path=None, tz_hours=None, shift=None):
    """Loads a recording, its feed dump and log; returns (Session, info dict, Recording). shift: fixed pose / feed row shift
    (default FEED_SHIFT; the scores of every shift are reported either way)."""
    import pose_metrics as pm
    rec, sec, pts, parts = load_points(pose_files, tz_hours)
    feed = pm.read_feed(feed_path)
    for c in ('capX', 'capY', 'yawDeg', 'dt'):
        if c not in feed:
            raise ValueError(f'feed dump has no column {c}')
    first_wall = rec.wall(sec[0])
    coarse = None
    mf = re.search(r'(\d{4})-(\d\d)-(\d\d)-(\d\d)-(\d\d)-(\d\d)', os.path.basename(feed_path))
    if mf:
        gap = (datetime.datetime(*[int(x) for x in mf.groups()]) - first_wall).total_seconds()
        if abs(gap) > 1.5:
            coarse = int(round(gap / (float(np.median(np.diff(sec))) or 1 / 60)))
    r2f, ainfo = pm.align_feed(sec, feed)
    if r2f is None and coarse is not None:
        r2f, ainfo = pm.align_feed(sec, feed, coarse)
    rows = frame_rows(r2f, len(sec))
    if not body:                                  # a weapon rig part has another bone count: take the first part of a known body rig
        known = [pm.BODY_OF_RIGCOUNT[rc] for _, rc in parts if rc in pm.BODY_OF_RIGCOUNT]
        body = known[0] if known else 'woman'
    db = load_db(db_path or default_db_path(body))
    events = read_motion_log(log_path)

    def sec_of(w):
        return (w - rec.day).total_seconds()

    tr = contact_track(db, events, sec, sec_of)
    toe = [pts[TOE[0]], pts[TOE[1]]]
    heel = [pts[h] for h in HEEL] if all(h in pts for h in HEEL) else None
    ses = Session(sec, toe, feed, rows, tr.cont, tr.state, tr.clip_of, db.names, heel, cont_pose=tr.cont_pose, cont_cut=tr.cont_cut)
    ses.choose_shift(FEED_SHIFT if shift is None else shift)
    nrow = len(feed['dt'])
    info = dict(body=body, frames=int(len(sec)), parts=parts, feed_alignment=ainfo, db=os.path.basename(db.path), contact=tr.notes,
                shift=ses.shift, shift_scores=ses.shift_scores, shift_flag=ses.shift_flag,
                aligned_share=round(float(np.mean(ses.aligned)), 3), teleport_pairs=int(ses.teleport.sum()),
                feed_gap_rows=int(ses.row_bad_cum[-1] - 1) if nrow > 1 else 0)
    return ses, info, rec


def analyze_windows(ses: Session, rec, windows, variants=None):
    """windows: [(label, start datetime, end datetime)] -> ({label: window_stats}, samples, locks, {label: (i0, i1)})."""
    variants = variants or ses.variants()
    smp, locks = variants['main']
    out, spans = {}, {}
    for label, a, b in windows:
        sa, sb = (a - rec.day).total_seconds(), (b - rec.day).total_seconds()
        i0, i1 = int(np.searchsorted(ses.fsec, sa)), int(np.searchsorted(ses.fsec, sb))
        if i1 - i0 < 3:
            continue
        out[label] = window_stats(ses, smp, locks, i0, i1, variants)
        spans[label] = (i0, i1)
    return out, smp, locks, spans


# ------------------------------------------------------------------------------------------------------------------------ run folders
def run_inputs(run_dir):
    """(pose files, log, feed, windows, step metrics) of a night run folder."""
    import gt_collect
    import gt_posemeter as gp
    logs = os.path.join(run_dir, 'logs')
    poses = gt_collect.pose_files(logs)
    if not poses:
        raise ValueError('no pose recording')
    with open(os.path.join(run_dir, 'cycle.json'), encoding='utf-8') as f:
        cycle = json.load(f)
    plan_runs = []
    pj = os.path.join(run_dir, 'gametest', 'plan.json')
    if os.path.isfile(pj):
        with open(pj, encoding='utf-8') as f:
            plan_runs = json.load(f).get('runs', [])
    windows = gp.build_windows(cycle.get('scenarios'), plan_runs)
    feeds = sorted(fn for fn in os.listdir(logs) if re.fullmatch(r'utpp-feed-\d{4}(-\d\d){5}\.txt', fn))
    import gt_log
    log = gt_log.session_log_in(logs)          # the current file; read_motion_log adds the rotated parts
    if not feeds or not log:
        raise ValueError('no feed dump or no mod log')
    steps = {}
    sj = os.path.join(run_dir, 'gametest', 'summary.json')
    if os.path.isfile(sj):
        with open(sj, encoding='utf-8') as f:
            summ = json.load(f)
        for st in summ.get('steps') or []:
            m = st.get('metrics') or {}
            steps[f'{st.get("scenario")}#{st.get("run", 1)}/{st.get("step")}'] = dict(
                ik_correction_max_cm=m.get('footlock_corr_max_cm'), verdict=st.get('verdict'))
    return poses, log, os.path.join(logs, feeds[-1]), windows, steps


def _pool_arrays(variants, keep_frame):
    """Arrays of one run for the pooled summary: per variant the samples and locks whose frame is not in a rejected window."""
    out = {}
    for name, (s, lk) in variants.items():
        m = keep_frame[s['frame']] if len(s['frame']) else np.zeros(0, dtype=bool)
        lk = [L for L in lk if keep_frame[L['mid']]]
        out[name] = dict(speed=s['speed'][m].astype(np.float32), cls=s['cls'][m], key=s['key'][m], blend=s['blend'][m],
                         lock_cm=np.array([L['cm'] for L in lk], dtype=np.float64),
                         lock_net=np.array([np.nan if L['net_cm'] is None else L['net_cm'] for L in lk], dtype=np.float64),
                         lock_cls=np.array([L['cls'] for L in lk], dtype=str), lock_key=np.array([L['key'] for L in lk], dtype=str))
    return out


def analyze_run(run_dir, db_path=None, pool=False):
    """Result dict of one night run folder; pool=True also returns the arrays the pooled summary is made of."""
    poses, log, feed, windows, steps = run_inputs(run_dir)
    ses, info, rec = build_session(poses, log, feed, db_path=db_path)
    variants = ses.variants()
    wins, smp, locks, spans = analyze_windows(ses, rec, windows, variants)
    import gt_posemeter as gp
    keep = np.ones(len(ses.fsec), dtype=bool)
    rejected, excluded = [], []
    for label, w in wins.items():
        if label in steps:
            w['ik_correction_max_cm'] = steps[label]['ik_correction_max_cm']
            w['step_verdict'] = steps[label]['verdict']
        if w['rejected']:
            rejected.append(label)
            if not label.endswith(gp.MACRO_SUFFIX):
                i0, i1 = spans[label]
                keep[i0:i1] = False
                excluded.append(label)
    allw = window_stats(ses, smp, locks, 0, len(ses.fsec), variants)
    info['rejected_windows'] = rejected
    info['excluded_windows'] = excluded
    info['rejected_frames'] = int((~keep).sum())
    res = dict(run=os.path.basename(os.path.abspath(run_dir)), info=info, whole=allw, windows=wins)
    if pool:
        return res, _pool_arrays(variants, keep)
    return res


# ------------------------------------------------------------------------------------------------------------------------ pooled summary
def _st(v):
    v = np.asarray(v, dtype=np.float64)
    if not len(v):
        return None
    return dict(n=int(len(v)), mean=round(float(v.mean()), 1), p50=round(float(np.percentile(v, 50)), 1),
                p95=round(float(np.percentile(v, 95)), 1), max=round(float(v.max()), 1))


def _lst(v, net):
    v = np.asarray(v, dtype=np.float64)
    if not len(v):
        return None
    net = np.asarray(net, dtype=np.float64)
    net = net[np.isfinite(net)]
    return dict(locks=int(len(v)), cm_mean=round(float(v.mean()), 2), cm_p50=round(float(np.percentile(v, 50)), 2),
                cm_p95=round(float(np.percentile(v, 95)), 2), cm_max=round(float(v.max()), 2),
                net_cm_p50=round(float(np.percentile(net, 50)), 2) if len(net) else None,
                net_cm_p95=round(float(np.percentile(net, 95)), 2) if len(net) else None)


def _cat(parts, name):
    ps = [p[name] for p in parts]
    return {k: np.concatenate([p[k] for p in ps]) for k in ps[0]}


def loop_by_key(cat):
    """Speed and lock statistics per gait key of the loop-class samples / locks only (a key such as Crouch/Loop also holds the
    crouch idle loop)."""
    out = {}
    m0, l0 = cat['cls'] == 'loop', cat['lock_cls'] == 'loop'
    for k in sorted(k for k in set(cat['key'][m0].tolist()) | set(cat['lock_key'][l0].tolist()) if k.endswith('/Loop')):
        m, lm = m0 & (cat['key'] == k), l0 & (cat['lock_key'] == k)
        out[str(k)] = dict(speed=_st(cat['speed'][m]), lock=_lst(cat['lock_cm'][lm], cat['lock_net'][lm]))
    return out


def summarize(pooled_by_body, top_keys=20):
    """{body: [per-run pool arrays]} -> summary per body: the main gate's all / by_class (speed, speed_blend, speed_steady, lock) /
    by_key / loop_by_key, and per variant (pose_blend, no_crossfade, short_locks) its by_class (speed, lock), by_key and loop_by_key."""
    summary = {}
    for body, parts in pooled_by_body.items():
        cat = _cat(parts, 'main')
        d = dict(all=dict(speed=_st(cat['speed']), lock=_lst(cat['lock_cm'], cat['lock_net']),
                          blend_share=round(float(cat['blend'].mean()), 3) if len(cat['blend']) else None), by_class={}, by_key={})
        for c in CLASSES:
            m = cat['cls'] == c
            lm = cat['lock_cls'] == c
            if m.any() or lm.any():
                d['by_class'][c] = dict(speed=_st(cat['speed'][m]), speed_blend=_st(cat['speed'][m & cat['blend']]),
                                        speed_steady=_st(cat['speed'][m & ~cat['blend']]), lock=_lst(cat['lock_cm'][lm], cat['lock_net'][lm]))
        keys, cnt = np.unique(cat['key'], return_counts=True)
        for k in keys[np.argsort(-cnt)][:top_keys]:
            m = cat['key'] == k
            d['by_key'][str(k)] = dict(speed=_st(cat['speed'][m]), lock=_lst(cat['lock_cm'][cat['lock_key'] == k], cat['lock_net'][cat['lock_key'] == k]))
        d['loop_by_key'] = loop_by_key(cat)
        d['variants'] = {}
        for name in ('pose_blend', 'no_crossfade', 'short_locks'):
            vc = _cat(parts, name)
            vd = dict(all=dict(speed=_st(vc['speed']), lock=_lst(vc['lock_cm'], vc['lock_net'])), by_class={}, by_key={})
            for c in CLASSES:
                m = vc['cls'] == c
                lm = vc['lock_cls'] == c
                if m.any() or lm.any():
                    vd['by_class'][c] = dict(speed=_st(vc['speed'][m]), speed_steady=_st(vc['speed'][m & ~vc['blend']]),
                                             lock=_lst(vc['lock_cm'][lm], vc['lock_net'][lm]))
            keys, cnt = np.unique(vc['key'], return_counts=True)
            for k in keys[np.argsort(-cnt)][:top_keys]:
                m = vc['key'] == k
                vd['by_key'][str(k)] = dict(speed=_st(vc['speed'][m]),
                                            lock=_lst(vc['lock_cm'][vc['lock_key'] == k], vc['lock_net'][vc['lock_key'] == k]))
            vd['loop_by_key'] = loop_by_key(vc)
            d['variants'][name] = vd
        summary[body] = d
    return summary


def _analyze_entry(arg):
    path, meta, db_path = arg
    try:
        res, pooled = analyze_run(path, db_path, pool=True)
        res.update({k: v for k, v in meta.items() if k != 'run'})
        return res, pooled
    except Exception as e:  # noqa: BLE001
        import traceback
        return dict(run=os.path.basename(os.path.abspath(path)), **{k: v for k, v in meta.items() if k != 'run'}, error=repr(e),
                    trace=traceback.format_exc()[-800:]), None


def _body_name(res):
    b = (res.get('body') or (res.get('info') or {}).get('body') or '').lower()
    return {'woman': 'female', 'man': 'male'}.get(b, b or 'unknown')


def run_all(entries, db_path=None, jobs=1):
    """entries: [(run folder, meta dict)] -> (per-run results, summary per body)."""
    args = [(p, m, db_path) for p, m in entries]
    if jobs > 1 and len(args) > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(min(jobs, len(args))) as ex:
            out = list(ex.map(_analyze_entry, args))
    else:
        out = [_analyze_entry(a) for a in args]
    pooled = {}
    for r, p in out:
        if p is not None:
            pooled.setdefault(_body_name(r), []).append(p)
    return [r for r, _ in out], summarize(pooled)


# ------------------------------------------------------------------------------------------------------------------------ report
def _f(x, nd=1):
    if x is None:
        return '-'
    if isinstance(x, (float, np.floating)):
        return f'{x:.{nd}f}'
    return str(x)


def _g(d, *path):
    for p in path:
        if d is None:
            return None
        d = d.get(p)
    return d


def write_md(doc, path, command=None, notes_path=None):
    """Report of a summary document (the --json output)."""
    S = doc['summary_by_body']
    runs = doc['runs']
    bodies = [b for b in ('female', 'male') if b in S] + [b for b in S if b not in ('female', 'male')]
    L = []
    a = L.append
    a(f"# World-space planted-foot slide ({doc.get('title') or 'summary'})")
    a('')
    if command:
        a('Command (from the project folder):')
        a('')
        a('```')
        a(command)
        a('```')
        a('')
    a(f"Generated {doc['generated']} by `Source/tools/footslide/world_slide.py` from {len(runs)} runs; full numbers per run, step, class "
      'and gait key: the JSON written next to this file.')
    a('')
    a('Metric: horizontal world speed of the toe (ToeBase) while the foot is in contact; world position = capsule (capX, capY) + '
      'Rz(yawDeg) x final-stage model-space toe, at the feed row matched to the frame + 1; distance of a frame pair / game step (feed dt) '
      'of that row. A lock is one contact segment of one foot (contact >= 0.5) with 3 frames trimmed at each end and >= 4 left; '
      '**path** = distance the toe travelled while locked, **net** = start to end. Speeds cm/s, distances cm.')
    a('')
    a('Contact gates:')
    a('- **runtime** (main): the contact the foot locker receives on the frames the motion driver owns - the live crossfade stack mixes '
      'the database contact curves of its layers with the whole-layer fade smoothstep(clock / blend time); older layers keep fading.')
    a('- **pose blend**: the same stack with the leg bones\' shorter fade (blend time / 1.5): when the drawn toe has reached the new clip.')
    a('- **no crossfade**: the contact of the selected clip alone (the frame the matcher plays).')
    a('- **short locks**: runtime gate, 1 frame trimmed, >= 2 left: the 4-9 frame run / sprint loop contacts count.')
    a('')
    a('**steady** = samples outside a clip crossfade (the newest layer fully faded in); **all** includes the crossfades. '
      'Samples of rejected step and scenario windows (< 90 % of the frames aligned with the feed, or contact known on < 80 %) are '
      'left out; a rejected macro window (report only) keeps its samples.')
    a('')
    a('## Per state, runtime gate')
    a('')
    a('| state | body | samples | p50 all | p95 all | p95 steady | max all | locks | path cm p50 / p95 | net cm p50 / p95 |')
    a('|---|---|---|---|---|---|---|---|---|---|')
    for c in CLASSES:
        for b in bodies:
            v = S[b]['by_class'].get(c)
            if not v:
                continue
            sp, st, lk = v['speed'] or {}, v['speed_steady'] or {}, v['lock'] or {}
            a(f"| {c} | {b.capitalize()} | {sp.get('n', 0)} | {_f(sp.get('p50'))} | {_f(sp.get('p95'))} | {_f(st.get('p95'))} | "
              f"{_f(sp.get('max'), 0)} | {lk.get('locks', 0)} | {_f(lk.get('cm_p50'))} / {_f(lk.get('cm_p95'))} | "
              f"{_f(lk.get('net_cm_p50'))} / {_f(lk.get('net_cm_p95'))} |")
    a('')
    a('## Per state: the three contact gates and the short-lock rule (p95 all, cm/s; locks in brackets)')
    a('')
    a('| state | body | runtime | pose blend | no crossfade | short locks | short-lock path cm p50 / p95 |')
    a('|---|---|---|---|---|---|---|')
    for c in CLASSES:
        for b in bodies:
            cells = []
            v = S[b]['by_class'].get(c)
            cells.append(f"{_f(_g(v, 'speed', 'p95'))} ({_g(v, 'lock', 'locks') or 0})" if v else '-')
            for name in ('pose_blend', 'no_crossfade', 'short_locks'):
                w = S[b]['variants'][name]['by_class'].get(c)
                cells.append(f"{_f(_g(w, 'speed', 'p95'))} ({_g(w, 'lock', 'locks') or 0})" if w else '-')
            w = S[b]['variants']['short_locks']['by_class'].get(c)
            cells.append(f"{_f(_g(w, 'lock', 'cm_p50'))} / {_f(_g(w, 'lock', 'cm_p95'))}" if w else '-')
            if any(x != '-' for x in cells):
                a(f'| {c} | {b.capitalize()} | ' + ' | '.join(cells) + ' |')
    a('')
    a('## Loop clips per gait key (loop-class samples only; p50 / p95 cm/s, locks; net cm p50)')
    a('')
    a('| key | body | runtime gate | short locks |')
    a('|---|---|---|---|')
    loop_keys = sorted({k for b in bodies for k in list(S[b]['loop_by_key']) + list(S[b]['variants']['short_locks']['loop_by_key'])})
    for k in loop_keys:
        for b in bodies:
            cells = []
            for v in (S[b]['loop_by_key'].get(k), S[b]['variants']['short_locks']['loop_by_key'].get(k)):
                cells.append(f"{_f(_g(v, 'speed', 'p50'))} / {_f(_g(v, 'speed', 'p95'))}, {_g(v, 'lock', 'locks') or 0} "
                             f"({_f(_g(v, 'lock', 'net_cm_p50'))})" if v else '-')
            a(f'| {k} | {b.capitalize()} | ' + ' | '.join(cells) + ' |')
    a('')
    a('## Gait keys with the most samples (runtime gate; p95 all cm/s, net cm p50)')
    a('')
    for b in bodies:
        items = list(S[b]['by_key'].items())[:8]
        a(f'- **{b.capitalize()}:** ' + '; '.join(f"{k} {_f(_g(v, 'speed', 'p95'))} ({_f(_g(v, 'lock', 'net_cm_p50'))})" for k, v in items))
    a('')
    a('## Runs')
    a('')
    a('| run | set | body | frames | aligned | feed gap rows | shift (mean cm/s at -2..2) | flag | windows rejected | blends / layers max |')
    a('|---|---|---|---|---|---|---|---|---|---|')
    for r in runs:
        if r.get('error'):
            a(f"| {r['run']} | {r.get('set', '')} | {r.get('body', '')} | error: {r['error']} | | | | | | |")
            continue
        i = r['info']
        sc = ' / '.join(_f(i['shift_scores'][str(s)][0]) for s in SHIFTS)
        a(f"| {r['run']} | {r.get('set', '')} | {_body_name(r)} | {i['frames']} | {_f(i['aligned_share'], 3)} | {i.get('feed_gap_rows')} | "
          f"{i['shift']} ({sc}) | {i['shift_flag'] or ''} | {len(i['rejected_windows'])} of {len(r['windows'])} | "
          f"{i['contact']['blends']} / {i['contact']['max_layers']} |")
    a('')
    rej, scen, frames, total = collections.Counter(), collections.Counter(), 0, 0
    for r in runs:
        total += len(r.get('windows') or {})
        for lab in (r.get('info') or {}).get('rejected_windows', []):
            w = (r.get('windows') or {}).get(lab, {})
            for why in (w.get('rejected') or '?').split('; '):
                rej[f'contact known on less than {KNOWN_MIN:.0%} of the frames' if 'contact known' in why else
                    f'less than {ALIGNED_MIN:.0%} of the frames aligned' if 'aligned' in why else why] += 1
            scen[lab.split('#')[0]] += 1
            frames += w.get('frames') or 0
    if rej:
        nexcl = sum(len((r.get('info') or {}).get('excluded_windows', [])) for r in runs)
        a(f'Rejected windows: {sum(scen.values())} of {total} ({frames} frames summed over them; step windows lie inside their scenario window), '
          f'{nexcl} of them left out of the pooled tables (the others are macro windows). '
          'Reasons: ' + '; '.join(f'{n} x {why}' for why, n in rej.most_common()) + '. By scenario: '
          + ', '.join(f'{s} {n}' for s, n in scen.most_common(12)) + '.')
        a('')
    cn = collections.Counter()
    for r in runs:
        for k, v in ((r.get('info') or {}).get('contact') or {}).items():
            if isinstance(v, (int, float)) and k != 'max_layers':
                cn[k] += v
    a('Contact track totals: ' + ', '.join(f'{k} {v}' for k, v in cn.items()) + '.')
    a('')
    if notes_path:
        with open(notes_path, encoding='utf-8') as fh:
            txt = fh.read().strip('\n')
        a('## Notes')
        a('')
        a(txt)
        a('')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(L))


def _rel_command(argv):
    """The command line with paths under the project folder made relative to it."""
    try:
        from utpp_paths import project_root
        root = os.path.abspath(project_root())
    except Exception:  # noqa: BLE001
        root = None
    out = []
    for x in argv:
        if root and os.path.isabs(x) and os.path.abspath(x).lower().startswith(root.lower() + os.sep):
            x = os.path.relpath(x, root)
        x = x.replace('\\', '/')
        out.append(f'"{x}"' if ' ' in x else x)
    return ' '.join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('runs', nargs='*', help='night run folders')
    ap.add_argument('--baseline', help='JSON list of {"run": <night folder name>, "set": ..., "body": ...}')
    ap.add_argument('--night', help='folder of the --baseline runs (default Research/logs/night)')
    ap.add_argument('--json')
    ap.add_argument('--md')
    ap.add_argument('--md-notes', help='text appended to the --md report as its notes section')
    ap.add_argument('--title', help='report title (default: the --baseline file name)')
    ap.add_argument('--jobs', type=int, default=1)
    ap.add_argument('--db', help='motion database (default Data/utpp_motion_<body>.utppmm)')
    a = ap.parse_args(argv)
    entries = [(r, {}) for r in a.runs]
    if a.baseline:
        night = a.night
        if not night:
            from utpp_paths import project_root
            night = os.path.join(project_root(), 'Research', 'logs', 'night')
        with open(a.baseline, encoding='utf-8') as f:
            for e in json.load(f):
                entries.append((os.path.join(night, e['run']), {k: e[k] for k in ('set', 'body') if k in e}))
    if not entries:
        ap.error('no runs')
    runs, summary = run_all(entries, a.db, a.jobs)
    for r in runs:
        w = r.get('whole') or {}
        print(r['run'], r.get('error') or f'p95 {w.get("speed_p95")} max {w.get("speed_max")} cm/s, '
              f'{w.get("locks")} locks {w.get("cm_mean")} cm, shift {r["info"]["shift"]} {r["info"]["shift_flag"] or ""}')
    doc = dict(generated=datetime.datetime.now().isoformat(timespec='seconds'),
               title=a.title or (os.path.splitext(os.path.basename(a.baseline))[0] if a.baseline else None),
               metric='world-space planted-toe slide (Source/tools/footslide/world_slide.py); speeds cm/s, lock distances cm',
               feed_shift=FEED_SHIFT, contact_min=CONTACT_MIN, trim_frames=TRIM, min_seg_frames=MIN_SEG,
               short_lock_rule=dict(trim_frames=SHORT_TRIM, min_seg_frames=SHORT_MIN_SEG), gates=list(GATES),
               summary_by_body=summary, runs=runs)
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1, default=str)
    if a.md:
        write_md(doc, a.md, 'python -I ' + _rel_command([os.path.abspath(__file__)] + list(sys.argv[1:] if argv is None else argv)),
                 a.md_notes)
    return 0


if __name__ == '__main__':
    sys.exit(main())
