"""Smoothness and body-deformation meter for the compact pose recordings (utpp-pose-*.utpppose) of a game session.

  python -I pose_metrics.py POSE_FILE [POSE_FILE ...] [--log LOG] [--feed FEED] [--windows auto|all|LABEL=HH:MM:SS-HH:MM:SS ...]
                            [--json OUT.json] [--table] [--no-helpers] [--helper-hz HZ]

  from pose_metrics import analyze
  result = analyze(pose_file, log_file=None, windows=None)      # dict, see below; windows: [(label, start, end), ...] or 'auto'

POSE_FILE is a recording (or the segments of one capture; they are joined in time order) or a text capture (utpp-bodytrace-*.csv). The mod log and the feed dump of the same session are
found next to the recording and in the folder given by the environment variable UTPP_SESSION_LOGS when not passed explicitly; without them
the pose-only metrics are still produced (clip, head look and gate events then stay empty).

A window is (label, start, end). start / end are naive local datetimes (the log time base, as the in-game runner uses), 'HH:MM:SS[.fff]' strings
(local wall clock) or floats (seconds since the first recorded frame). No windows = one window 'all'. 'auto' makes windows of 1 s each and labels them
  clean_<gait>        steady gait (walk, run, sprint, crouch_walk, idle) with no clip change, head look switch or gate change near it
  headlook_on / _off  around a head look switch of the log
  clipchange_<a>_to_<b>  around a change of gait family in the log
  in_start / in_stop / in_tap / in_switch_quarter / in_switch_reverse / in_straight / in_strafe / in_backward / in_arc
                      kinds of movement input in the feed (needs the feed: move vector and camera angle)
  hitch               around a frame time above 3 x the median

Per window and per stage ('pre_mod' = the pose before the mod writes, 'final' = after it, 'post_helper' = the engine pose after the helper
bones were applied, only in recordings that hold that stream) the result holds
  frame_pacing   frame time median / p95 / max, share of frames above 1.5 x the median (hitches), from the recorded timestamps and, when the feed
                 dump lines up with the recording, from the feed step as well
  smoothness     per tracked bone and space (model = relative to the root, parent = relative to the parent bone, chain = Head vs Spine3 and Spine3
                 vs Hips): angular velocity (deg/s), its change between frames (deg/s^2), single-frame discontinuities (robust test against the
                 +-NEIGHBOURS neighbouring frames), stalls; hips translation speed / acceleration / discontinuities; planted-foot residual
  events         clip changes and clip restarts per second of movement, head look switches, gate changes, and which discontinuities coincide with a
                 clip change, a layer switch or a frame-time hitch
  deformation    helper_mismatch: recorded helper bones against the helper bones recomputed from the recorded core pose (cm, per bone group), and
                 anatomy: share of frames with a core joint outside its rotation band, worst excess in degrees
Quantities are tied to the recording: angular quantities are divided by the step the mod advanced (feed step when aligned, else the timestamp
difference), so uneven frame times do not create spikes of their own.

Read-only on every input; nothing is written next to the recordings (--json goes where it is told).
"""
import argparse
import bisect
import contextlib
import datetime
import itertools
import json
import os
import re
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ANALYSIS = os.path.join(os.path.dirname(HERE), 'analysis')
for _p in (HERE, ANALYSIS):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import pose_capture_to_csv as pc  # noqa: E402
import pose_helper_report as phr  # noqa: E402

ANATOMY_FILE = os.path.join(HERE, 'testdata', 'pose_metrics_anatomy.json')
STAGE_NAMES = ('pre_mod', 'final')            # stream 0 stage 0 / 1
HITCH_FACTOR = 1.5
NEIGHBOURS = 6                                # frames either side for the local median / MAD of the discontinuity test
SPIKE_K = 8.0                                 # a spike exceeds the local median by K local robust sigmas (+ the relative / size floors below)
SPIKE_REL = 0.15                              # ... or K x this fraction of the local median, whichever is larger
SPIKE_MIN_DEG = 0.3                           # smallest excess rotation (deg) that counts, above the quantisation noise of the format
SPIKE_MIN_CM = 0.15
STALL_FRACTION = 0.05
OBSERVED_EPS_DEG = 1.0                        # slack on the observed bands (a hinge joint has a zero-width band on two axes)
PLANT_CHUNK = 6                               # frames per planted-foot chunk
PLANT_HEIGHT_M = 0.03
MOVING_CM_S = 8.0                             # hips speed (cm/s) above which a frame counts as movement when the log gives no clip family
COINCIDE_LOG_S = 0.15                         # log events (ms resolution) match a discontinuity within this many seconds
COINCIDE_FRAMES = 2
TRACK = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Head', 'LeftHand', 'RightHand', 'LeftFoot', 'RightFoot']
CHAINS = [('Head', 'Spine3'), ('Spine3', 'Hips')]          # (bone, reference): rotation of bone relative to reference
GUARD_ORDER = ['Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head', 'LeftShoulder', 'RightShoulder', 'LeftArm', 'RightArm',
               'LeftForeArm', 'RightForeArm', 'LeftHand', 'RightHand', 'LeftUpLeg', 'RightUpLeg', 'LeftLeg', 'RightLeg', 'LeftFoot', 'RightFoot']
BODY_OF_RIGCOUNT = {645: 'woman', 619: 'man'}
HELPER_GROUPS = [('shoulder_chest', r'shl|scapula|deltoid|trapez|latissimus|butterfly|chest|sternocleido|clavicle|pector'),
                 ('arm', r'bicep|tricep|elbow|wrist|forearm|armpit|upperarm|arm_'),
                 ('leg', r'thigh|calf|knee|lowleg|leg_|groin|hip'),
                 ('neck_head', r'neck|head|jaw|throat'),
                 ('hand', r'thumb|finger|hand|palm|index|middle|ring|pinky')]


# ------------------------------------------------------------------------------------------------------------------------ quaternion math
def qmul(a, b):
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz], axis=-1)


def qconj(q):
    return q * np.array([-1.0, -1.0, -1.0, 1.0])


def qnorm(q):
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / np.where(n > 1e-9, n, np.nan)


def qrotate(q, v):
    u = q[..., :3]
    t = 2.0 * np.cross(u, v)
    return v + q[..., 3:4] * t + np.cross(u, t)


def qrotvec_deg(q):
    """Rotation vector (degrees) of quaternions xyzw, shortest way round."""
    q = np.where(q[..., 3:4] < 0, -q, q)
    s = np.linalg.norm(q[..., :3], axis=-1, keepdims=True)
    ang = 2.0 * np.arctan2(s, q[..., 3:4])
    scale = np.where(s > 1e-12, ang / np.where(s > 1e-12, s, 1.0), 2.0)
    return np.degrees(q[..., :3] * scale)


def twist_swing_deg(q):
    """Rest-relative local rotation -> (twist about +X, swing about Y, swing about Z), degrees; q = swing * twist."""
    q = np.where(q[..., 3:4] < 0, -q, q)
    tau = 2.0 * np.arctan2(q[..., 0], q[..., 3])
    t = np.stack([np.sin(tau / 2), np.zeros_like(tau), np.zeros_like(tau), np.cos(tau / 2)], axis=-1)
    sw = qrotvec_deg(qmul(q, qconj(t)))
    tw = (np.degrees(tau) + 180.0) % 360.0 - 180.0
    return tw, sw[..., 1], sw[..., 2]


# ------------------------------------------------------------------------------------------------------------------------ recording
class Recording:
    """Layout of a recording: catalogue names, parents (catalogue indices), rig size, timestamps base. group is None for text captures."""

    def __init__(self, names, parent, rig_count, qpc_hz, day, notes, group=None):
        self.names, self.parent, self.rig_count, self.qpc_hz, self.day, self.notes, self.group = names, parent, rig_count, qpc_hz, day, notes, group
        self.cat = {n: k for k, n in enumerate(names)}

    @classmethod
    def from_group(cls, group, notes):
        h = group['header']
        mapped = h.mapped()
        names = [h.names[i] for i in mapped]
        rigidx = [h.game_index[i] for i in mapped]
        r2c = {g: k for k, g in enumerate(rigidx)}
        parent = [r2c.get(h.parents[g], -1) if g < len(h.parents) and h.parents[g] >= 0 else -1 for g in rigidx]
        return cls(names, parent, h.rig_count, h.qpc_hz, datetime.datetime.strptime(group['day'], '%Y-%m-%d'), notes, group)

    def wall(self, sec):
        return self.day + datetime.timedelta(seconds=float(sec))


def _chain_to_root(parent, k):
    out = []
    while k >= 0 and len(out) < 64:
        out.append(k)
        k = parent[k]
    return out[::-1]


def system_offset_seconds(header):
    u = header.utc
    when = datetime.datetime(u[0], u[1], u[2], u[3], u[4], u[5], tzinfo=datetime.timezone.utc)
    return when.astimezone().utcoffset().total_seconds()


def load_recording(pose_files, bone_names, tz_hours=None):
    """Reads the files; returns (Recording, data) with data = dict(sec, ticks, q[N, nb, 2, 4], t[N, nb, 2, 3], keep=[catalogue index], has_stream1)."""
    files = [pose_files] if isinstance(pose_files, str) else list(pose_files)
    stamped = all(re.search(r'\d{4}-\d\d-\d\d-\d\d-\d\d-\d\d', os.path.basename(f)) for f in files)
    tz = tz_hours * 3600.0 if tz_hours is not None else None
    if tz is None and not stamped:                  # no local time stamp in the file name: use the offset of this machine
        tz = system_offset_seconds(phr.read_header(files[0]))
    groups = phr.load_groups(files, phr.Config(tz_seconds=tz))
    group = groups[-1] if len(groups) > 1 else groups[0]
    rec = Recording.from_group(group, group.setdefault('notes', []))
    if len(groups) > 1:
        rec.notes.append('recording holds %d rig topologies; the last one is analysed' % len(groups))
    need = set()
    for n in bone_names:
        if n in rec.cat:
            need.update(_chain_to_root(rec.parent, rec.cat[n]))
    keep = sorted(need)
    secs, tks, qs, ts = [], [], [], []
    stream1 = 0
    for stream, ticks, sec, gidx, v in phr.iter_blocks(group):
        if stream == 1:
            stream1 += len(ticks)
            continue
        sel = v[:, keep]
        secs.append(sec)
        tks.append(ticks)
        ts.append(sel[..., 0:3].astype(np.float32))
        qs.append(qnorm(sel[..., 4:8].astype(np.float64)).astype(np.float32))
    if not secs:
        raise ValueError('no frames in %s' % ', '.join(files))
    data = dict(sec=np.concatenate(secs), ticks=np.concatenate(tks), q=np.concatenate(qs), t=np.concatenate(ts), keep=keep,
                has_stream1=stream1 > 0, stream1_frames=stream1)
    return rec, data


def load_csv_recording(path, bone_names, tz_hours=None):
    """Reads a text capture of the continuous helper-bone recording (utpp-bodytrace-*.csv: both stages, one row per catalogue bone per frame)
    into the same arrays as load_recording. Streaming: only the rows of the needed bones are parsed."""
    import itertools
    qpc_hz = arm_ticks = 0
    utc = None
    topo = {}
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        first_rows = None
        frames_t, frames_v = [], []
        need_pos = None
        for ln in f:
            if ln.startswith('#'):
                m = re.match(r'# qpc_hz=(\d+) arm_ticks=(\d+) utc=(\S+)', ln)
                if m:
                    qpc_hz, arm_ticks, utc = int(m.group(1)), int(m.group(2)), m.group(3)
                m = re.match(r'# rig_bone index=(\d+) hash=\w+ parent=(-?\d+)', ln)
                if m:
                    topo[int(m.group(1))] = int(m.group(2))
            elif ln.startswith('ticks,'):
                break
        for ticks, grp in itertools.groupby(f, key=lambda x: x[:x.find(',')]):
            lines = list(grp)
            if lines[0].startswith('#'):
                continue
            if first_rows is None:
                first_rows = [(ln.split(',', 8)[6], int(ln.split(',', 8)[7])) for ln in lines]
                mapped = [i for i, (_, gi) in enumerate(first_rows) if gi >= 0]
                names = [first_rows[i][0] for i in mapped]
                rigidx = [first_rows[i][1] for i in mapped]
                r2c = {g: k for k, g in enumerate(rigidx)}
                parent = [r2c.get(topo.get(g, -1), -1) if topo.get(g, -1) >= 0 else -1 for g in rigidx]
                rig_count = int(lines[0].split(',', 8)[5])
                rec = Recording(names, parent, rig_count, qpc_hz, None, [])
                need = set()
                for n in bone_names:
                    if n in rec.cat:
                        need.update(_chain_to_root(parent, rec.cat[n]))
                keep = sorted(need)
                need_pos = [mapped[k] for k in keep]
            if len(lines) != len(first_rows):
                break                                           # a frame cut short at the end of the file
            frames_t.append(int(ticks))
            frames_v.append([[float(x) for x in lines[i].split(',')[8:32]] for i in need_pos])
    if not frames_t:
        raise ValueError('no frames in %s' % path)
    arr = np.asarray(frames_v, dtype=np.float64)                 # [N, nb, 24]: stage 0 = 0..11, stage 1 = 12..23
    arr = arr.reshape(len(arr), len(keep), 2, 12)
    ticks = np.asarray(frames_t, dtype=np.int64)
    m = re.search(r'(\d{4})-(\d\d)-(\d\d)-(\d\d)-(\d\d)-(\d\d)', os.path.basename(path))
    start = datetime.datetime.strptime(utc.rstrip('Z'), '%Y-%m-%dT%H:%M:%S.%f')
    if tz_hours is not None:
        off = tz_hours * 3600.0
    elif m:
        off = round((datetime.datetime(*[int(x) for x in m.groups()]) - start).total_seconds() / 900.0) * 900.0
    else:
        off = start.replace(tzinfo=datetime.timezone.utc).astimezone().utcoffset().total_seconds()
    local = start + datetime.timedelta(seconds=off)
    rec.day = local.replace(hour=0, minute=0, second=0, microsecond=0)
    sec = (local - rec.day).total_seconds() + (ticks - arm_ticks) / float(qpc_hz)
    data = dict(sec=sec, ticks=ticks, q=qnorm(arr[..., 4:8]).astype(np.float32), t=arr[..., 0:3].astype(np.float32), keep=keep,
                has_stream1=False, stream1_frames=0)
    return rec, data


def forward_kinematics(rec, data, stage, names):
    """Model-space (rotation xyzw, position) of the named bones for every frame: dict name -> (q[N,4], t[N,3])."""
    keep = data['keep']
    pos = {k: i for i, k in enumerate(keep)}
    N = data['q'].shape[0]
    mq, mt = {}, {}
    keep_set = set(keep)
    done = set()

    def build(k):
        if k in done:
            return
        p = rec.parent[k]
        if p >= 0 and p in keep_set:
            build(p)
        i = pos[k]
        ql = data['q'][:, i, stage].astype(np.float64)
        tl = data['t'][:, i, stage].astype(np.float64)
        if p < 0 or p not in mq:
            mq[k], mt[k] = ql, tl
        else:
            mq[k] = qmul(mq[p], ql)
            mt[k] = mt[p] + qrotate(mq[p], tl)
        done.add(k)

    for k in keep:
        build(k)
    return {n: (mq[rec.cat[n]], mt[rec.cat[n]]) for n in names if n in rec.cat and rec.cat[n] in mq}


# ------------------------------------------------------------------------------------------------------------------------ log and feed
TS_RE = re.compile(r'^\[(\d{4})-(\d\d)-(\d\d) (\d\d):(\d\d):(\d\d)\.(\d+)\]')
MOTION_RE = re.compile(r'UTPP (?:MOTION|MMV2) t=([\d.]+) db=(\d+):(\S+) clip=(-?\d+):(\S+) ct=(-?[\d.]+)(?: .* body=(\w+))?')
HEADLOOK_RE = re.compile(r'UTPPSetHeadLook\((on|off) ')
GATE_RE = re.compile(r'UTPP MMGATE gate=(\w+)x(\w+) \(was (\w+)x(\w+)\)')
NIGHT_RE = re.compile(r'\[night\] (\S+)(.*)')


def _ts(m):
    y, mo, d, hh, mi, ss, frac = m.groups()
    return datetime.datetime(int(y), int(mo), int(d), int(hh), int(mi), int(ss), int((frac + '000000')[:6]))


def log_parts(path):
    """The session log and its rotated parts (utpp-<stamp>.N.log, renamed when the log passes 10 MB), oldest content first.
    Without this only the newest part was read and every window before the last rotation had no clip changes."""
    try:
        import gt_log
        parts = gt_log.session_files(path)
    except Exception:
        parts = []
    return parts or [path]


@contextlib.contextmanager
def open_log_parts(path):
    """All parts of the session log as one line iterator (oldest first)."""
    files = [open(p, 'r', encoding='utf-8', errors='replace') for p in log_parts(path)]
    try:
        yield itertools.chain.from_iterable(files)
    finally:
        for f in files:
            f.close()


def parse_log(path):
    """-> dict(motion=[(wall, t, clip, ct, body)], headlook=[(wall, 'on'|'off')], gate=[(wall, text)], marks=[(wall, text)])."""
    out = dict(motion=[], headlook=[], gate=[], marks=[], first=None, last=None)
    with open_log_parts(path) as f:
        for ln in f:
            if not ln.startswith('['):
                continue
            m = TS_RE.match(ln)
            if not m:
                continue
            if 'UTPP MOTION t=' in ln or 'UTPP MMV2 t=' in ln:           # the line was called MMV2 in earlier builds
                mm = MOTION_RE.search(ln)
                if mm:
                    t, _, _, _, clip, ct, body = mm.groups()
                    body = body or ''
                    out['motion'].append((_ts(m), float(t), clip, float(ct), body))
            elif 'UTPPSetHeadLook(' in ln:
                hm = HEADLOOK_RE.search(ln)
                if hm:
                    out['headlook'].append((_ts(m), hm.group(1)))
            elif 'UTPP MMGATE ' in ln:
                gm = GATE_RE.search(ln)
                if gm:
                    out['gate'].append((_ts(m), '%sx%s<-%sx%s' % gm.groups()))
            elif '[night]' in ln:
                nm = NIGHT_RE.search(ln)
                if nm and nm.group(1) != 'POS':
                    out['marks'].append((_ts(m), (nm.group(1) + nm.group(2)).strip()[:80]))
            if out['first'] is None:
                out['first'] = _ts(m)
            out['last'] = _ts(m)
    return out


def _session_dirs(pose_path):
    dirs = [os.path.dirname(os.path.abspath(pose_path))]
    env = os.environ.get('UTPP_SESSION_LOGS')
    if env:
        dirs.append(env)
    try:
        sys.path.insert(0, os.path.dirname(HERE))
        import utpp_paths
        root = utpp_paths.project_root()
        dirs.append(os.path.join(root, 'Research', 'logs', 'game'))
    except Exception:
        pass
    return [d for d in dirs if os.path.isdir(d)]


def find_log(pose_path, start_wall):
    """The session log that was running when the capture started: the newest utpp-<stamp>.log with stamp <= start."""
    best = None
    for d in _session_dirs(pose_path):
        for fn in os.listdir(d):
            m = re.fullmatch(r'utpp-(\d{4})-(\d\d)-(\d\d)-(\d\d)-(\d\d)-(\d\d)\.log', fn)
            if not m:
                continue
            st = datetime.datetime(*[int(x) for x in m.groups()])
            if st <= start_wall + datetime.timedelta(seconds=1) and (best is None or st > best[0]):
                best = (st, os.path.join(d, fn))
    return best[1] if best else None


def find_feed(pose_path, start_wall):
    best = None
    for d in _session_dirs(pose_path):
        for fn in os.listdir(d):
            m = re.fullmatch(r'utpp-feed-(\d{4})-(\d\d)-(\d\d)-(\d\d)-(\d\d)-(\d\d)\.txt', fn)
            if not m:
                continue
            st = datetime.datetime(*[int(x) for x in m.groups()])
            gap = abs((st - start_wall).total_seconds())
            if gap <= 3.0 and (best is None or gap < best[0]):
                best = (gap, os.path.join(d, fn))
    return best[1] if best else None


FEED_COLS = ('t', 'dt', 'capX', 'capY', 'capZ', 'effGate', 'upperBody', 'loco', 'mvx', 'mvy', 'vx', 'vy', 'yawDeg', 'camDeg')


def read_feed(path):
    """Feed dump -> dict of float arrays for FEED_COLS ('-' -> nan)."""
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        first = f.readline()
        cols = first.split()[2:] if first.startswith('# cols') else []
        idx = {c: cols.index(c) for c in FEED_COLS if c in cols}
        rows = {c: [] for c in idx}
        for ln in f:
            if ln.startswith('#'):
                continue
            p = ln.split()
            if len(p) < len(cols):
                continue
            for c, i in idx.items():
                try:
                    rows[c].append(float(p[i]))
                except ValueError:
                    rows[c].append(np.nan)
    return {c: np.asarray(v, dtype=np.float64) for c, v in rows.items()}


def align_feed(sec, feed, coarse=None, chunk=600):
    """Pose frame index of every feed row (-1 where the two sequences cannot be matched) and a description. The feed dump and the recording are
    written by different threads, so a frame can be missing on either side: a start shift is searched on a central slice (around `coarse` frames
    when the two started at different times, else +-12), then the shift of every chunk of rows is estimated by matching the frame-step sequences
    (mean absolute difference), keeping the previous shift unless another one is clearly better."""
    if 'dt' not in feed or len(feed['dt']) < 200 or len(sec) < 200:
        return None, 'feed too short'
    pdt = np.diff(sec)                   # pdt[k] = step into pose frame k + 1
    fdt = feed['dt']                     # fdt[j] = step into feed row j
    rows = len(fdt)

    def errors(lo, hi, shifts):
        out = {}
        for sh in shifts:
            a0 = lo - 1 + sh                # feed row lo <-> pose step index a0
            if a0 < 0 or a0 + (hi - lo) > len(pdt):
                continue
            out[sh] = float(np.mean(np.abs(pdt[a0:a0 + hi - lo] - fdt[lo:hi])))
        return out

    centre = max(1, min(rows // 3, rows - 1600))
    c = 0 if coarse is None else int(coarse)
    span = 12 if coarse is None else 150
    first = errors(centre, min(centre + 1500, rows - 1), range(c - span, c + span + 1))
    if not first:
        return None, 'feed and recording do not overlap'
    base = min(first, key=first.get)
    frame_of_row = np.full(rows, -1, dtype=np.int64)
    prev = base
    good_rows, errs_all = 0, []
    for lo in range(1, rows - 1, chunk):
        hi = min(lo + chunk, rows - 1)
        accepted = None
        for width in (8, 60):               # a lost frame on either side moves the shift by one; after a gap look further
            errs = errors(lo, hi, range(prev - width, prev + width + 1))
            if not errs:
                continue
            order = sorted(errs, key=errs.get)
            best = order[0]
            if prev in errs and errs[prev] <= 1.25 * errs[best] + 1e-4:
                best = prev
            rest = [errs[x] for x in order if x != best]
            if errs[best] <= 0.004 and not (rest and errs[best] > 0.8 * float(np.median(rest))):
                accepted = (best, errs[best])
                break
        if accepted is None:
            continue
        prev = accepted[0]
        frame_of_row[lo:hi] = np.arange(lo, hi) + prev
        good_rows += hi - lo
        errs_all.append(accepted[1])
    if good_rows < 0.3 * rows:
        return None, 'dt sequences do not line up (%d of %d feed rows matched)' % (good_rows, rows)
    shifts = sorted(set(int(frame_of_row[j] - j) for j in range(1, rows - 1, chunk) if frame_of_row[j] >= 0))
    return frame_of_row, 'aligned %d of %d feed rows (shifts %s, mean step error %.2f ms)' % (good_rows, rows, shifts, 1000 * float(np.mean(errs_all)))


# ------------------------------------------------------------------------------------------------------------------------ series
def local_stats(x, w=NEIGHBOURS):
    """Median and MAD-sigma of each element's +-w neighbours, the element itself excluded. x: [N] (nan allowed)."""
    n = len(x)
    pad = np.full(n + 2 * w, np.nan)
    pad[w:w + n] = x
    win = np.lib.stride_tricks.sliding_window_view(pad, 2 * w + 1).copy()
    win[:, w] = np.nan
    with np.errstate(all='ignore'):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            med = np.nanmedian(win, axis=1)
            mad = np.nanmedian(np.abs(win - med[:, None]), axis=1) * 1.4826
    return med, mad


def spike_mask(speed, dt, min_size):
    """Single-frame discontinuities of a speed series (deg/s or cm/s): frame f exceeds the local median by SPIKE_K robust sigmas (or the relative
    floor) and by an absolute size of at least min_size (deg or cm) x dt. Returns (mask, excess size [per frame])."""
    med, mad = local_stats(speed)
    scale = np.maximum(mad, SPIKE_REL * np.nan_to_num(med, nan=0.0))
    with np.errstate(invalid='ignore'):
        excess = (speed - med) * dt
        mask = (speed - med > SPIKE_K * scale) & (excess >= min_size)
    return np.where(np.isnan(mask), False, mask), excess


def stall_mask(speed, dt, min_size):
    med, _ = local_stats(speed)
    with np.errstate(invalid='ignore'):
        mask = (speed < STALL_FRACTION * med) & (med * dt >= min_size)
    return np.where(np.isnan(mask), False, mask)


def angular_series(q, dt, local):
    """q [N,4] -> (omega [N,3] deg/s, speed [N] deg/s, change [N] deg/s^2). Frame 0 is nan."""
    n = len(q)
    d = np.full((n, 3), np.nan)
    if n > 1:
        delta = qmul(qconj(q[:-1]), q[1:]) if local else qmul(q[1:], qconj(q[:-1]))
        d[1:] = qrotvec_deg(delta)
    om = d / dt[:, None]
    speed = np.linalg.norm(om, axis=1)
    chg = np.full(n, np.nan)
    if n > 2:
        chg[2:] = np.linalg.norm(om[2:] - om[1:-1], axis=1) / (0.5 * (dt[2:] + dt[1:-1]))
    return om, speed, chg


def linear_series(p, dt):
    n = len(p)
    v = np.full((n, 3), np.nan)
    if n > 1:
        v[1:] = np.diff(p, axis=0)
    v = v / dt[:, None]
    speed = np.linalg.norm(v, axis=1)
    chg = np.full(n, np.nan)
    if n > 2:
        chg[2:] = np.linalg.norm(v[2:] - v[1:-1], axis=1) / (0.5 * (dt[2:] + dt[1:-1]))
    return v, speed, chg


class Series:
    """Per-recording precomputed series of one stage."""

    def __init__(self):
        self.ang = {}         # key -> (speed, change, spike mask, excess, stall mask)
        self.lin = {}
        self.rel = {}         # chain key -> relative rotation [N, 4]


def compute_stage(rec, data, stage, dt):
    s = Series()
    fk = forward_kinematics(rec, data, stage, TRACK)
    keep = data['keep']
    pos = {k: i for i, k in enumerate(keep)}
    for name in TRACK:
        if name not in fk:
            continue
        mq, mt = fk[name]
        om, sp, ch = angular_series(mq, dt, local=False)
        m, ex = spike_mask(sp, dt, SPIKE_MIN_DEG)
        s.ang[(name, 'model')] = (sp, ch, m, ex, stall_mask(sp, dt, SPIKE_MIN_DEG))
        k = rec.cat[name]
        if name != 'Hips' and rec.parent[k] >= 0:
            ql = data['q'][:, pos[k], stage].astype(np.float64)
            om, sp, ch = angular_series(ql, dt, local=True)
            m, ex = spike_mask(sp, dt, SPIKE_MIN_DEG)
            s.ang[(name, 'parent')] = (sp, ch, m, ex, stall_mask(sp, dt, SPIKE_MIN_DEG))
    for name, ref in CHAINS:
        if name in fk and ref in fk:
            rel = qmul(qconj(fk[ref][0]), fk[name][0])
            om, sp, ch = angular_series(rel, dt, local=True)
            m, ex = spike_mask(sp, dt, SPIKE_MIN_DEG)
            s.ang[(name + '_vs_' + ref, 'chain')] = (sp, ch, m, ex, stall_mask(sp, dt, SPIKE_MIN_DEG))
            s.rel[name + '_vs_' + ref] = rel
    for name in ('Hips',):
        if name in fk:
            v, sp, ch = linear_series(fk[name][1] * 100.0, dt)       # cm
            m, ex = spike_mask(sp, dt, SPIKE_MIN_CM)
            s.lin[(name + '_pos', 'model')] = (sp, ch, m, ex, stall_mask(sp, dt, SPIKE_MIN_CM))
    s.fk = fk
    return s


def planted_residual(fk):
    """Planted-foot slide proxy without a world frame: chunks of PLANT_CHUNK consecutive frames in which a foot stays within PLANT_HEIGHT_M of its
    own lowest height of the surrounding second; per chunk the largest horizontal distance (cm) of the foot from the straight line through
    the first and last position of the chunk. A foot planted under constant body velocity moves on that line. Returns [(cm, first, end)]."""
    feet = {n: fk[n][1] for n in ('LeftFoot', 'RightFoot') if n in fk}
    if len(feet) < 2 or 'Hips' not in fk:
        return None
    up = (fk['Hips'][1] - 0.5 * (feet['LeftFoot'] + feet['RightFoot'])).mean(axis=0)
    up = up / max(np.linalg.norm(up), 1e-9)
    out = []
    for p in feet.values():
        h = p @ up
        hor = (p - np.outer(h, up)) * 100.0
        w = 30                                              # rolling floor: lowest height of the foot within half a second either side
        padded = np.pad(h, w, mode='edge')
        floor = np.lib.stride_tricks.sliding_window_view(padded, 2 * w + 1).min(axis=1)
        planted = h < floor + PLANT_HEIGHT_M
        for a in range(0, len(h) - PLANT_CHUNK + 1, PLANT_CHUNK):
            b = a + PLANT_CHUNK
            if not planted[a:b].all():
                continue
            seg = hor[a:b] - hor[a]
            ln = np.linalg.norm(seg[-1])
            resid = seg - np.outer(seg @ (seg[-1] / ln), seg[-1] / ln) if ln > 1e-6 else seg
            out.append((float(np.linalg.norm(resid, axis=1).max()), a, b))
    return out


# ------------------------------------------------------------------------------------------------------------------------ deformation
_ANATOMY = {}


def anatomy_table(body):
    if body not in _ANATOMY:
        with open(ANATOMY_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
        _ANATOMY[body] = data.get(body) or data['woman']
    return _ANATOMY[body]


def anatomy_series(rec, data, stage, body):
    """-> dict joint -> dict(violation [N] deg beyond the tolerant band (0 inside), outside_p [N] bool beyond the observed band, trans [N] cm excess)."""
    tab = anatomy_table(body)['joints']
    keep = data['keep']
    pos = {k: i for i, k in enumerate(keep)}
    out = {}
    for name in GUARD_ORDER:
        if name not in tab or name not in rec.cat or rec.cat[name] not in pos:
            continue
        j = tab[name]
        i = pos[rec.cat[name]]
        ql = data['q'][:, i, stage].astype(np.float64)
        rel = qmul(qconj(np.asarray(j['rest_quat'], dtype=np.float64)[None]), ql)
        tw, sy, sz = twist_swing_deg(rel)
        viol = np.zeros(len(tw))
        outp = np.zeros(len(tw), dtype=bool)
        for val, key in ((tw, 'twist'), (sy, 'swing_y'), (sz, 'swing_z')):
            lo, hi = j[key]
            viol = np.maximum(viol, np.maximum(lo - val, val - hi))
            plo, phi = j[key + '_p']
            outp |= (val < plo - OBSERVED_EPS_DEG) | (val > phi + OBSERVED_EPS_DEG)
        tdev = np.linalg.norm(data['t'][:, i, stage].astype(np.float64) * 100.0 - np.asarray(j['rest_local_t_cm']), axis=1)
        out[name] = dict(violation=np.maximum(viol, 0.0), outside_p=outp, trans=np.maximum(tdev - j['trans_dev_cm'], 0.0))
    return out


def helper_group(name):
    low = name.lower()
    for g, rx in HELPER_GROUPS:
        if re.search(rx, low):
            return g
    return 'other'


class HelperEvaluator:
    """Recomputes helper bones from the core pose with the project's offline evaluation of the deformer graph (tools/analysis)."""

    def __init__(self, body):
        import evaluate_body_deformer_graph as ev
        import replay_body_skin as rb
        self.ev = ev.Evaluator(allow_rotation_bounce=True, body=body)
        self.trs = rb.trs
        self.core = set(self.ev.coreNames)


def helper_mismatch(rec, group, frame_idx, body, notes, want_stream1):
    """Recorded helper bones against the helper bones recomputed from the recorded core pose, on the given frame indices (stream 0) and, when
    the recording holds the post-helper stream, on the matching frames of that stream. Returns {stage: (bone names, [(frame, cm per bone)])}."""
    try:
        he = HelperEvaluator(body)
    except Exception as e:                    # the serialized deformer graph is not part of the repository
        notes.append('helper mismatch skipped: %s' % (str(e)[:120],))
        return None
    names, nm = rec.names, len(rec.names)
    if not he.core <= set(names):
        notes.append('helper mismatch skipped: the recording lacks core bones (%s)' % ', '.join(sorted(he.core - set(names))[:4]))
        return None
    order, seen = [], set()

    def visit(k):
        if k in seen:
            return
        if rec.parent[k] >= 0:
            visit(rec.parent[k])
        seen.add(k)
        order.append(k)

    for k in range(nm):
        visit(k)
    wanted = sorted(set(int(i) for i in frame_idx))
    pick = {}
    for stream, ticks, sec, gidx, v in phr.iter_blocks(group):
        if stream != 0:
            continue
        lo, hi = int(gidx[0]), int(gidx[-1])
        a, b = bisect.bisect_left(wanted, lo), bisect.bisect_right(wanted, hi)
        for i in wanted[a:b]:
            pick[i] = (v[i - lo].copy(), int(ticks[i - lo]))
    s1 = {}
    if want_stream1 and pick:
        wt = np.array(sorted(t for _, t in pick.values()))
        tol = 0.005 * rec.qpc_hz
        for stream, ticks, sec, gidx, v in phr.iter_blocks(group):
            if stream != 1:
                continue
            j = np.clip(np.searchsorted(ticks, wt), 1, len(ticks) - 1)
            near = np.where(np.abs(ticks[j] - wt) < np.abs(ticks[j - 1] - wt), j, j - 1)
            for w, i in zip(wt, near):
                if abs(int(ticks[i]) - int(w)) <= tol:
                    s1[int(w)] = v[i, :, 0].copy()

    def residual(vals):
        loc = {names[k]: he.trs(vals[k]) for k in range(nm)}
        M = {}
        for k in order:
            M[names[k]] = (M[names[rec.parent[k]]] if rec.parent[k] >= 0 else np.eye(4)) @ loc[names[k]]
        res = he.ev.run({n: loc[n] for n in he.core}, stop=1)
        bones = [n for n in names if n not in he.core and n in res and n in he.ev.writes]
        return bones, np.array([np.linalg.norm(M[n][:3, 3] - res[n][:3, 3]) * 100.0 for n in bones])

    results = {}
    for stage in (0, 1):
        bones, rows = [], []
        for i in sorted(pick):
            try:
                bones, cm = residual(pick[i][0][:, stage])
            except Exception as e:
                notes.append('helper mismatch frame %d failed: %s' % (i, str(e)[:80]))
                continue
            rows.append((i, cm))
        results[STAGE_NAMES[stage]] = (bones, rows)
    if s1:
        bones, rows = [], []
        for i in sorted(pick):
            vals = s1.get(pick[i][1])
            if vals is None:
                continue
            try:
                bones, cm = residual(vals)
            except Exception as e:
                notes.append('post-helper frame %d failed: %s' % (i, str(e)[:80]))
                continue
            rows.append((i, cm))
        if rows:
            results['post_helper'] = (bones, rows)
    return results


# ------------------------------------------------------------------------------------------------------------------------ events
FAMILIES = [('idle', r'_Idle'), ('sprint', r'Sprint_Loop'), ('run', r'Run_Loop'), ('walk', r'Walk_Loop'), ('crouch_walk', r'Crouch_(Walk_)?Loop'),
            ('start', r'_Start_'), ('stop', r'_Stop_'), ('turn', r'Turn|Reface|Pivot'), ('air', r'Jump|Fall|Land'), ('transition', r'Transition')]


def clip_family(clip):
    for fam, rx in FAMILIES:
        if re.search(rx, clip):
            return fam
    return 'other'


class Events:
    """Log-derived events on the recording time base (seconds since local midnight of the first frame's day)."""

    def __init__(self, rec, log):
        self.available = log is not None
        self.clip_changes = []        # (sec, from, to)
        self.restarts = []            # (sec, clip)
        self.headlook = []            # (sec, 'on'|'off')
        self.gates = []               # (sec, text)
        self.marks = []
        self.intervals = []           # (sec_start, clip) in time order: the clip in force from sec_start
        self.body = None
        if log is None:
            return

        def sec(w):
            return (w - rec.day).total_seconds()

        prev = None
        bodies = {}
        for wall, t, clip, ct, body in log['motion']:
            s = sec(wall)
            bodies[body] = bodies.get(body, 0) + 1
            if prev is not None:
                if clip != prev[1]:
                    self.clip_changes.append((s, prev[1], clip))
                elif ct < prev[2] - 0.1 and 'Loop' not in clip:
                    self.restarts.append((s, clip))
            if prev is None or clip != prev[1]:
                self.intervals.append((s, clip))
            prev = (s, clip, ct)
        self.body = max(bodies, key=bodies.get) if bodies else None
        self.headlook = [(sec(w), k) for w, k in log['headlook']]
        self.gates = [(sec(w), t) for w, t in log['gate']]
        self.marks = [(sec(w), t) for w, t in log['marks']]

    def family_at(self, s):
        if not self.intervals:
            return None
        i = bisect.bisect_right([a for a, _ in self.intervals], s) - 1
        return clip_family(self.intervals[i][1]) if i >= 0 else None


def times_in(items, lo, hi):
    return [x for x in items if lo <= x[0] <= hi]


# ------------------------------------------------------------------------------------------------------------------------ analysis
def parse_window_time(x, rec, sec0):
    if isinstance(x, datetime.datetime):
        return (x - rec.day).total_seconds()
    if isinstance(x, str):
        t = phr.parse_clock(x)
        day_s = sec0 - (sec0 % 86400.0)
        s = day_s + t
        return s + 86400.0 if s < sec0 - 1.0 else s
    return sec0 + float(x)


def stats_into(d, prefix, x):
    x = x[np.isfinite(x)]
    if len(x) == 0:
        for k in ('p50', 'p95', 'p99', 'max'):
            d['%s_%s' % (prefix, k)] = None
        return
    for k, v in zip(('p50', 'p95', 'p99'), np.percentile(x, (50, 95, 99))):
        d['%s_%s' % (prefix, k)] = round(float(v), 4)
    d[prefix + '_max'] = round(float(x.max()), 4)


def r4(x):
    return None if x is None or not np.isfinite(x) else round(float(x), 4)


def hms(s):
    s = s % 86400.0
    return '%02d:%02d:%06.3f' % (int(s // 3600), int(s % 3600 // 60), s % 60)


def analyze(pose_file, log_file=None, windows=None, feed_file=None, helpers=True, helper_hz=1.0, helper_min_per_window=None, top=6, tz_hours=None):
    """Analyses one recording (or the segments of one capture). windows: None (one window 'all'), 'auto', or [(label, start, end), ...] with
    naive local datetimes, 'HH:MM:SS[.fff]' strings or floats (seconds since the first frame). log_file / feed_file: a path, None (look next to
    the recording) or False (none). tz_hours: local-minus-UTC hours of the log time base (default: the time stamp in the file name, else this
    machine). Returns a JSON-serialisable dict."""
    files = [pose_file] if isinstance(pose_file, str) else list(pose_file)
    if len(files) == 1 and files[0].lower().endswith('.csv'):
        rec, data = load_csv_recording(files[0], set(TRACK) | set(GUARD_ORDER), tz_hours)
        helpers = False
        rec.notes.append('text capture: helper bone comparison not available')
    else:
        rec, data = load_recording(files, set(TRACK) | set(GUARD_ORDER), tz_hours)
    sec = data['sec']
    N = len(sec)
    notes = list(rec.notes)
    first_wall = rec.wall(sec[0])
    log_path = None if log_file is False else (log_file or find_log(files[0], first_wall))
    log = parse_log(log_path) if log_path and os.path.isfile(log_path) else None
    ev = Events(rec, log)
    feed_path = None if feed_file is False else (feed_file or find_feed(files[0], first_wall))
    feed = read_feed(feed_path) if feed_path and os.path.isfile(feed_path) else None
    body = ev.body if ev.body in ('woman', 'man') else BODY_OF_RIGCOUNT.get(rec.rig_count, 'woman')
    tick_dt = np.diff(sec, prepend=np.nan)
    tick_dt[0] = tick_dt[1] if N > 1 else 1.0 / 60
    dt = tick_dt.copy()
    feed_info = 'no feed dump'
    rows_to_frame = None
    feed_events = []
    if feed is not None:
        coarse = None
        mf = re.search(r'(\d{4})-(\d\d)-(\d\d)-(\d\d)-(\d\d)-(\d\d)', os.path.basename(feed_path))
        if mf:
            gap = (datetime.datetime(*[int(x) for x in mf.groups()]) - first_wall).total_seconds()
            if abs(gap) > 1.5:
                coarse = int(round(gap / (float(np.median(np.diff(sec))) or 1 / 60)))
        rows_to_frame, feed_info = align_feed(sec, feed)
        if rows_to_frame is None and coarse is not None:
            rows_to_frame, feed_info = align_feed(sec, feed, coarse)
        if rows_to_frame is not None:
            k = rows_to_frame
            ok = (k >= 0) & (k < N) & np.isfinite(feed['dt']) & (feed['dt'] > 0)
            dt[k[ok]] = feed['dt'][ok]
            gate = feed.get('effGate')
            if gate is not None:
                ch = np.nonzero((np.diff(np.nan_to_num(gate, nan=-1)) != 0) & ok[1:] & ok[:-1])[0] + 1
                feed_events = [(float(sec[k[c]]), 'effGate') for c in ch]
    dt = np.maximum(dt, 1e-3)
    median_dt = float(np.median(tick_dt[1:])) if N > 2 else 1.0 / 60
    hitch = tick_dt > HITCH_FACTOR * median_dt

    series = {s: compute_stage(rec, data, s, dt) for s in (0, 1)}
    anat = {s: anatomy_series(rec, data, s, body) for s in (0, 1)} if os.path.isfile(ANATOMY_FILE) else {}
    planted = {s: planted_residual(series[s].fk) for s in (0, 1)}
    # movement flags
    if ev.available and ev.intervals:
        fam = np.array([ev.family_at(float(x)) or 'other' for x in sec[::max(1, N // 20000)]])
        moving = np.repeat(fam != 'idle', max(1, N // 20000))[:N]
        moving = np.pad(moving, (0, N - len(moving)), constant_values=True)
    else:
        moving = np.ones(N, dtype=bool)

    # ---- windows
    wins = []
    if windows is None:
        wins = [('all', 0, N)]
    elif isinstance(windows, str) and windows == 'auto':
        extra = input_windows(feed, rows_to_frame, sec) if feed is not None and rows_to_frame is not None else []
        wins = auto_windows(rec, ev, sec, tick_dt, median_dt, hitch, extra)
    else:
        for label, a, b in windows:
            sa, sb = parse_window_time(a, rec, sec[0]), parse_window_time(b, rec, sec[0])
            i0, i1 = int(np.searchsorted(sec, sa)), int(np.searchsorted(sec, sb))
            wins.append((label, i0, min(i1, N)))
    wins = [(l, a, b) for l, a, b in wins if b - a >= 3]

    # ---- spikes (frame level, final stage), for the event coincidence and for the helper sample
    spike_frames = {}
    for s in (1,):
        for (name, space), (sp, ch, m, ex, st) in series[s].ang.items():
            for f in np.nonzero(m)[0]:
                spike_frames.setdefault(int(f), []).append((name, space, float(ex[f])))
        for (name, space), (sp, ch, m, ex, st) in series[s].lin.items():
            for f in np.nonzero(m)[0]:
                spike_frames.setdefault(int(f), []).append((name, space, float(ex[f])))
    clip_secs = np.array([c[0] for c in ev.clip_changes] + [r[0] for r in ev.restarts])
    layer_secs = np.array([h[0] for h in ev.headlook] + [g[0] for g in ev.gates] + [g[0] for g in feed_events])
    fps = 1.0 / median_dt

    def tag_frame(f):
        t = float(sec[f])
        tags = []
        if len(clip_secs) and np.min(np.abs(clip_secs - t)) <= COINCIDE_LOG_S:
            tags.append('clip_change')
        if len(layer_secs) and np.min(np.abs(layer_secs - t)) <= max(COINCIDE_LOG_S, COINCIDE_FRAMES * median_dt):
            tags.append('layer_switch')
        if hitch[f] or (f + 1 < N and hitch[f + 1]):
            tags.append('hitch')
        return tags or ['none']

    # ---- helper mismatch sample
    helper_res = None
    h_every = max(1, int(round(fps / max(helper_hz, 1e-6))))
    sample = set(range(0, N, h_every))
    if helper_min_per_window is None:
        helper_min_per_window = 8 if isinstance(windows, list) else 0
    for _, a, b in wins:
        if helper_min_per_window and b - a > 0:
            sample.update(int(x) for x in np.linspace(a, b - 1, helper_min_per_window))
    top_spikes = sorted(spike_frames, key=lambda f: -max(x[2] for x in spike_frames[f]))[:24]
    sample.update(top_spikes)
    if helpers:
        helper_res = helper_mismatch(rec, rec.group, sorted(sample), body, notes, data['has_stream1'])

    out_wins = []
    for label, i0, i1 in wins:
        out_wins.append(window_result(label, i0, i1, rec, data, sec, dt, tick_dt, median_dt, hitch, series, anat, planted, moving, ev, feed_events,
                                      spike_frames, tag_frame, helper_res, top))
    return dict(
        pose_files=[os.path.basename(f) for f in files], log=os.path.basename(log_path) if log_path else None,
        feed=os.path.basename(feed_path) if feed_path else None, feed_alignment=feed_info, body=body, rig_count=rec.rig_count, frames=N,
        duration_s=round(float(sec[-1] - sec[0]), 3), start=rec.wall(sec[0]).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        median_frame_ms=round(median_dt * 1000, 3), post_helper_stream=data['has_stream1'], notes=notes,
        helper_samples=None if not helper_res else {k: len(v[1]) for k, v in helper_res.items()},
        windows=out_wins)


def window_result(label, i0, i1, rec, data, sec, dt, tick_dt, median_dt, hitch, series, anat, planted, moving, ev, feed_events, spike_frames, tag_frame,
                  helper_res, top):
    n = i1 - i0
    t0, t1 = float(sec[i0]), float(sec[i1 - 1])
    dur = t1 - t0
    mv = moving[i0 + 1:i1]
    move_s = float(np.sum(dt[i0 + 1:i1][mv]))
    res = dict(label=label, start=hms(t0), end=hms(t1), t0_s=round(t0 - float(sec[0]), 3), frames=n, duration_s=round(dur, 3),
               movement_s=round(move_s, 3))
    d = dict(frames=n, fps=round(n / dur, 2) if dur > 0 else None)
    td = tick_dt[i0 + 1:i1]
    d['dt_median_ms'] = r4(np.median(td) * 1000)
    d['dt_p95_ms'] = r4(np.percentile(td, 95) * 1000)
    d['dt_max_ms'] = r4(td.max() * 1000)
    d['hitch_share'] = r4(float(np.mean(td > HITCH_FACTOR * np.median(td))))
    d['hitches'] = int(np.sum(td > HITCH_FACTOR * np.median(td)))
    fd = dt[i0 + 1:i1]
    d['step_p95_ms'] = r4(np.percentile(fd, 95) * 1000)
    d['step_vs_timestamp_mean_abs_ms'] = r4(float(np.mean(np.abs(fd - td))) * 1000)
    res['frame_pacing'] = d
    per_s = lambda c: None if move_s < 0.5 else round(c / move_s, 4)  # noqa: E731
    sm = {}
    spikes_union = {}
    for s, sname in ((1, 'final'), (0, 'pre_mod')):
        block = {}
        for table, unit in ((series[s].ang, 'deg'), (series[s].lin, 'cm')):
            for (name, space), (sp, ch, m, ex, st) in table.items():
                key = '%s.%s' % (name, space)
                e = {}
                stats_into(e, 'speed_' + ('deg_s' if unit == 'deg' else 'cm_s'), sp[i0 + 1:i1])
                stats_into(e, 'change_' + ('deg_s2' if unit == 'deg' else 'cm_s2'), ch[i0 + 2:i1])
                mm = m[i0 + 1:i1]
                e['spikes'] = int(mm.sum())
                e['spikes_per_move_s'] = per_s(int(mm.sum()))
                e['spike_max_' + unit] = r4(float(np.nanmax(ex[i0 + 1:i1][mm])) if mm.any() else 0.0)
                e['stalls'] = int(st[i0 + 1:i1].sum())
                if space == 'chain' and n > 1:
                    rel = series[s].rel[name]
                    e['excursion_deg'] = r4(float(np.nanmax(np.linalg.norm(qrotvec_deg(qmul(qconj(rel[i0]), rel[i0:i1])), axis=1))))
                block[key] = e
                if s == 1:
                    for f in np.nonzero(mm)[0] + i0 + 1:
                        spikes_union[int(f)] = True
        pl = planted[s]
        if pl:
            win = [r[0] for r in pl if i0 <= r[1] and r[2] <= i1]
            pf = block.setdefault('planted_foot', {})
            pf['runs'] = len(win)
            if win:
                stats_into(pf, 'residual_cm', np.asarray(win))
        sm[sname] = block
    res['smoothness'] = sm
    res['smoothness']['final_spike_frames'] = len(spikes_union)
    res['smoothness']['final_spike_frames_per_move_s'] = per_s(len(spikes_union))
    worst = sorted(((f, max(spike_frames[f], key=lambda x: x[2])) for f in spikes_union), key=lambda fx: -fx[1][2])[:top]
    res['worst_discontinuities'] = [dict(time=hms(float(sec[f])), t_s=round(float(sec[f] - sec[0]), 3), bone=b[0], space=b[1], size=round(b[2], 3),
                                         tags=tag_frame(f)) for f, b in worst]
    cls = {}
    for f in spikes_union:
        for tg in tag_frame(f):
            cls[tg] = cls.get(tg, 0) + 1
    # events
    lo, hi = t0, t1
    cc = times_in(ev.clip_changes, lo, hi)
    rs = times_in(ev.restarts, lo, hi)
    hl = times_in(ev.headlook, lo, hi)
    e = dict(log_events_available=ev.available, clip_changes=len(cc), clip_changes_per_move_s=per_s(len(cc)), clip_restarts=len(rs),
             headlook_switches=[(hms(t), k) for t, k in hl], headlook_within_1s=len(times_in(ev.headlook, lo - 1.0, hi + 1.0)), gate_changes=len(times_in(ev.gates, lo, hi)) + len(times_in(feed_events, lo, hi)),
             discontinuities_by_tag=cls, clip_resolution_note='clip events come from the log (written when the selection changes, plus every 0.5 s)')
    res['events'] = e
    # deformation
    de = {}
    an = {}
    for s, sname in ((1, 'final'), (0, 'pre_mod')):
        if s not in anat:
            continue
        joints = {}
        anyv = np.zeros(n, dtype=bool)
        worst_deg, worst_joint = 0.0, None
        for name, a in anat[s].items():
            v = a['violation'][i0:i1]
            anyv |= v > 0
            joints[name] = dict(share_outside=r4(float(np.mean(v > 0))), share_outside_observed=r4(float(np.mean(a['outside_p'][i0:i1]))),
                                worst_deg=r4(float(v.max())), translation_excess_cm=r4(float(a['trans'][i0:i1].max())))
            if v.max() > worst_deg:
                worst_deg, worst_joint = float(v.max()), name
        an[sname] = dict(frames_any_share=r4(float(anyv.mean())), worst_deg=r4(worst_deg), worst_joint=worst_joint, joints=joints)
    de['anatomy'] = an
    if helper_res is not None:
        hm = {}
        for sname, (bones, rows) in helper_res.items():
            rows = [(f, cm) for f, cm in rows if i0 <= f < i1]
            if not rows or not bones:
                hm[sname] = dict(samples=0)
                continue
            allcm = np.stack([cm for _, cm in rows])
            groups = {}
            for gname in [g for g, _ in HELPER_GROUPS] + ['other']:
                cols = [i for i, b in enumerate(bones) if helper_group(b) == gname]
                if cols:
                    x = allcm[:, cols]
                    groups[gname] = dict(max_cm=r4(float(x.max())), p95_cm=r4(float(np.percentile(x, 95))), median_cm=r4(float(np.median(x))))
            hm[sname] = dict(samples=len(rows), max_cm=r4(float(allcm.max())), p95_cm=r4(float(np.percentile(allcm, 95))),
                             median_cm=r4(float(np.median(allcm))), groups=groups,
                             worst_bone=bones[int(np.unravel_index(allcm.argmax(), allcm.shape)[1])])
        de['helper_mismatch'] = hm
    res['deformation'] = de
    return res


def input_windows(feed, rows_to_frame, sec, max_per_kind=150):
    """Windows (label, i0, i1, 1 s each) around the kinds of movement input visible in the feed: start, stop, tap (a press shorter than half a
    second), direction switch (net change of the held direction of at least 40 degrees, measured over 0.25 s before and after; quarter / reverse),
    and steady stretches: straight, strafe (60-120 degrees from forward), backward, arc (steady key direction while the camera turns faster than
    30 deg/s). A release shorter than 0.25 s does not end a press. The input is the camera-relative move vector of the feed (mvx, mvy); digital keys
    and analog sticks both work, the kinds are defined on the vector."""
    need = ('t', 'mvx', 'mvy', 'camDeg')
    if rows_to_frame is None or any(k not in feed for k in need):
        return []
    t, mx, my, cam = feed['t'], feed['mvx'], feed['mvy'], feed['camDeg']
    n = min(len(t), len(rows_to_frame), len(mx), len(my), len(cam))
    t, mx, my, cam, r2f = t[:n], np.nan_to_num(mx[:n]), np.nan_to_num(my[:n]), np.nan_to_num(cam[:n]), rows_to_frame[:n]
    on = np.hypot(mx, my) > 0.1
    held = on.copy()                               # presses with short releases bridged
    off_edges = np.diff(np.r_[0, (~on).astype(np.int8), 0])
    for a, b in zip(np.nonzero(off_edges == 1)[0], np.nonzero(off_edges == -1)[0]):
        if 0 < a and b < n and t[b] - t[a] < 0.25:               # a release between two presses, shorter than a quarter second
            held[a:b] = True
    ang = np.where(on, np.degrees(np.arctan2(mx, my)), np.nan)
    unit = np.where(on[:, None], np.stack([np.cos(np.radians(ang)), np.sin(np.radians(ang))], axis=1), 0.0)
    camrate = np.zeros(n)
    unw = np.degrees(np.unwrap(np.radians(cam)))
    step = max(1, int(round(0.25 / max(float(np.median(np.diff(t))), 1e-3))))
    camrate[step:] = np.abs(unw[step:] - unw[:-step]) / np.maximum(t[step:] - t[:-step], 1e-3)
    out = []

    def frames(ta, tb):
        ja, jb = int(np.searchsorted(t, ta)), int(np.searchsorted(t, tb))
        ja, jb = max(ja, 0), min(jb, n - 1)
        if jb <= ja or r2f[ja] < 0 or r2f[jb] < 0:
            return None
        return int(r2f[ja]), int(r2f[jb])

    def add(label, ta, tb):
        fr = frames(ta, tb)
        if fr is not None and len([o for o in out if o[0] == label]) < max_per_kind:
            out.append((label, fr[0], fr[1]))

    def mean_dir(ja, jb):
        u = unit[ja:jb]
        if on[ja:jb].sum() < 5:
            return None
        v = u.sum(axis=0)
        return np.degrees(np.arctan2(v[1], v[0])) if np.hypot(*v) > 1e-6 else None

    edges = np.diff(held.astype(np.int8))
    starts = list(np.nonzero(edges == 1)[0] + 1)
    ends = list(np.nonzero(edges == -1)[0] + 1)
    if held[0]:
        starts.insert(0, 0)
    if held[-1]:
        ends.append(n - 1)
    for a, b in zip(starts, ends):
        ts, te = t[a], t[min(b, n - 1)]
        quiet = not held[max(a - 15, 0):a].any()                  # no input for the quarter second before the press
        if te - ts < 0.5:
            add('in_tap', ts - 0.2, ts + 0.8)
        elif quiet and a > 0:
            add('in_start', ts - 0.2, ts + 0.8)
        if te - ts >= 0.5 and b < n - 1:
            add('in_stop', te - 0.3, te + 0.7)
        # direction switches: net change of the held direction around each sample, one event per local maximum
        w = max(2, int(round(0.25 / max(float(np.median(np.diff(t))), 1e-3))))
        delta = np.zeros(b - a)
        for k, j in enumerate(range(a, b)):
            d0, d1 = mean_dir(max(j - w, a), j), mean_dir(j, min(j + w, b))
            if d0 is not None and d1 is not None:
                delta[k] = abs((d1 - d0 + 180.0) % 360.0 - 180.0)
        last = -1e9
        for k in np.argsort(-delta):
            if delta[k] < 40.0:
                break
            tk = t[a + k]
            if abs(tk - last) < 0.4 or any(abs(tk - o) < 0.4 for o in switch_times(out, t, r2f)):
                continue
            add('in_switch_reverse' if delta[k] > 135.0 else 'in_switch_quarter', tk - 0.3, tk + 0.7)
            last = tk
        cs = ts + 0.3
        while cs + 1.0 <= te:
            ja, jb = int(np.searchsorted(t, cs)), int(np.searchsorted(t, cs + 1.0))
            seg = ang[ja:jb]
            seg = seg[np.isfinite(seg)]
            if len(seg) >= 5:
                spread = np.max(np.abs((seg - seg[0] + 180.0) % 360.0 - 180.0))
                if spread < 15.0:
                    a0 = abs((seg[0] + 180.0) % 360.0 - 180.0)
                    rate = float(np.median(camrate[ja:jb]))
                    if rate > 30.0:
                        add('in_arc', cs, cs + 1.0)
                    elif 60.0 <= a0 <= 120.0:
                        add('in_strafe', cs, cs + 1.0)
                    elif a0 >= 150.0:
                        add('in_backward', cs, cs + 1.0)
                    elif a0 < 30.0 and rate < 10.0:
                        add('in_straight', cs, cs + 1.0)
            cs += 1.0
    return out


def switch_times(out, t, r2f):
    """Feed times of the switch windows added so far (their centre), used to keep one event per key change."""
    res = []
    for label, f0, f1 in out:
        if label.startswith('in_switch'):
            rows = np.nonzero((r2f >= f0) & (r2f <= f1))[0]
            if len(rows):
                res.append(float(t[rows[len(rows) * 3 // 10]]))
    return res


def auto_windows(rec, ev, sec, tick_dt, median_dt, hitch, extra=(), clean_s=1.0, margin_s=0.4, max_event_windows=120):
    """Window list (label, i0, i1) from the log events: clean steady-locomotion windows, windows around head look switches, clip changes
    between locomotion families, and frame-time hitches."""
    N = len(sec)

    def idx(s):
        return int(np.searchsorted(sec, s))

    wins = []
    ivs = ev.intervals
    hl_secs = [t for t, _ in ev.headlook]
    gate_secs = [t for t, _ in ev.gates]
    for n, (s, clip) in enumerate(ivs):
        end = ivs[n + 1][0] if n + 1 < len(ivs) else float(sec[-1])
        fam = clip_family(clip)
        if fam not in ('walk', 'run', 'sprint', 'crouch_walk', 'idle'):
            continue
        a = max(s + margin_s, float(sec[0]))
        while a + clean_s <= end - margin_s and a + clean_s <= float(sec[-1]):
            b = a + clean_s
            near_hl = any(a - 1.0 <= t <= b + 1.0 for t in hl_secs)
            near_gate = any(a - margin_s <= t <= b + margin_s for t in gate_secs)
            if not near_hl and not near_gate:
                wins.append(('clean_' + fam, idx(a), idx(b)))
            a = b
    for t, k in ev.headlook:
        wins.append(('headlook_' + k, idx(t - 0.5), idx(t + 0.5)))
    changes = [c for c in ev.clip_changes if clip_family(c[1]) != clip_family(c[2])]
    for t, a, b in changes[:max_event_windows]:
        wins.append(('clipchange_%s_to_%s' % (clip_family(a), clip_family(b)), idx(t - 0.5), idx(t + 0.5)))
    wins.extend(extra)
    big = np.nonzero(tick_dt > 3 * median_dt)[0]
    for f in big[:max_event_windows]:
        wins.append(('hitch', int(max(f - 30, 0)), int(min(f + 30, N))))
    return wins


# ------------------------------------------------------------------------------------------------------------------------ tables
TABLE_METRICS = [
    ('dt_p95_ms', lambda w: w['frame_pacing']['dt_p95_ms']),
    ('hitch_share', lambda w: w['frame_pacing']['hitch_share']),
    ('Head.parent.chg_p99', lambda w: w['smoothness']['final'].get('Head.parent', {}).get('change_deg_s2_p99')),
    ('Spine3.parent.chg_p99', lambda w: w['smoothness']['final'].get('Spine3.parent', {}).get('change_deg_s2_p99')),
    ('Head_vs_Spine3.speed_p99', lambda w: w['smoothness']['final'].get('Head_vs_Spine3.chain', {}).get('speed_deg_s_p99')),
    ('Head_vs_Spine3.speed_max', lambda w: w['smoothness']['final'].get('Head_vs_Spine3.chain', {}).get('speed_deg_s_max')),
    ('Head_vs_Spine3.excursion_deg', lambda w: w['smoothness']['final'].get('Head_vs_Spine3.chain', {}).get('excursion_deg')),
    ('Spine3_vs_Hips.speed_max', lambda w: w['smoothness']['final'].get('Spine3_vs_Hips.chain', {}).get('speed_deg_s_max')),
    ('Head_vs_Spine3.chg_p99', lambda w: w['smoothness']['final'].get('Head_vs_Spine3.chain', {}).get('change_deg_s2_p99')),
    ('Spine3_vs_Hips.chg_p99', lambda w: w['smoothness']['final'].get('Spine3_vs_Hips.chain', {}).get('change_deg_s2_p99')),
    ('Head.model.chg_p99', lambda w: w['smoothness']['final'].get('Head.model', {}).get('change_deg_s2_p99')),
    ('Hips.model.chg_p99', lambda w: w['smoothness']['final'].get('Hips.model', {}).get('change_deg_s2_p99')),
    ('LeftFoot.model.chg_p99', lambda w: w['smoothness']['final'].get('LeftFoot.model', {}).get('change_deg_s2_p99')),
    ('hips_pos.chg_p99', lambda w: w['smoothness']['final'].get('Hips_pos.model', {}).get('change_cm_s2_p99')),
    ('spike_frames_per_s', lambda w: w['smoothness']['final_spike_frames_per_move_s']),
    ('planted_resid_p95', lambda w: w['smoothness']['final'].get('planted_foot', {}).get('residual_cm_p95')),
    ('clip_changes_per_s', lambda w: w['events']['clip_changes_per_move_s']),
    ('anatomy_any_share', lambda w: w['deformation']['anatomy'].get('final', {}).get('frames_any_share')),
    ('anatomy_worst_deg', lambda w: w['deformation']['anatomy'].get('final', {}).get('worst_deg')),
    ('helper_final_p95_cm', lambda w: w['deformation'].get('helper_mismatch', {}).get('final', {}).get('p95_cm')),
    ('helper_final_max_cm', lambda w: w['deformation'].get('helper_mismatch', {}).get('final', {}).get('max_cm')),
    ('helper_pre_max_cm', lambda w: w['deformation'].get('helper_mismatch', {}).get('pre_mod', {}).get('max_cm')),
]


def flatten(window):
    out = {}
    for name, fn in TABLE_METRICS:
        try:
            out[name] = fn(window)
        except (KeyError, TypeError):
            out[name] = None
    return out


def aggregate(results, group_of=lambda w: w['label'], qs=(50, 90, 99)):
    """results: list of analyze() dicts -> {group: {metric: {n, p50, p90, p99, max}}} over the windows of every session."""
    groups = {}
    for r in results:
        for w in r['windows']:
            groups.setdefault(group_of(w), []).append(flatten(w))
    table = {}
    for g, rows in groups.items():
        t = {}
        for name, _ in TABLE_METRICS:
            x = np.array([row[name] for row in rows if row[name] is not None], dtype=float)
            if len(x):
                t[name] = dict(n=len(x), **{'p%d' % q: round(float(v), 3) for q, v in zip(qs, np.percentile(x, qs))}, max=round(float(x.max()), 3))
        table[g] = dict(windows=len(rows), metrics=t)
    return table


def propose_thresholds(table, groups=('clean_walk', 'clean_run'), margin=1.25):
    """p99 over the clean windows of the given groups x margin, per metric (what a limit for a clean window would be)."""
    out = {}
    for name, _ in TABLE_METRICS:
        vals = [table[g]['metrics'][name]['p99'] for g in groups if g in table and name in table[g]['metrics']]
        if vals:
            out[name] = round(float(max(vals)) * margin, 3)
    return out


def format_aggregate(table, groups=None):
    names = [n for n, _ in TABLE_METRICS]
    lines = []
    for g in sorted(table) if groups is None else groups:
        if g not in table:
            continue
        lines.append('%s  (%d windows)' % (g, table[g]['windows']))
        for n in names:
            m = table[g]['metrics'].get(n)
            if m:
                lines.append('    %-26s n=%-4d p50 %-9s p90 %-9s p99 %-9s max %-9s' % (n, m['n'], m['p50'], m['p90'], m['p99'], m['max']))
    return '\n'.join(lines)


def parse_cli_windows(items):
    wins = []
    for it in items:
        label, _, rng = it.partition('=')
        a, _, b = rng.partition('-')
        wins.append((label, a.strip(), b.strip()))
    return wins


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('pose', nargs='+', help='recording file(s); each argument is one session')
    ap.add_argument('--log')
    ap.add_argument('--feed')
    ap.add_argument('--windows', nargs='*', default=None, help="'auto', 'all' or LABEL=HH:MM:SS-HH:MM:SS")
    ap.add_argument('--json', help='write the full result (list of sessions) here')
    ap.add_argument('--table', action='store_true', help='print the distribution table over all windows of all sessions')
    ap.add_argument('--no-helpers', action='store_true')
    ap.add_argument('--helper-hz', type=float, default=1.0)
    ap.add_argument('--tz-hours', type=float, help='local-minus-UTC hours of the log time base when the file name has no time stamp')
    a = ap.parse_args(argv)
    if a.windows is None or a.windows == ['all']:
        windows = None
    elif a.windows == ['auto']:
        windows = 'auto'
    else:
        windows = parse_cli_windows(a.windows)
    results = []
    for p in a.pose:
        r = analyze(p, a.log, windows, a.feed, helpers=not a.no_helpers, helper_hz=a.helper_hz, tz_hours=a.tz_hours)
        results.append(r)
        print('%s: %s frames %d (%.1f s) windows %d; %s' % (r['pose_files'][0], r['body'], r['frames'], r['duration_s'], len(r['windows']), r['feed_alignment']))
        for n in r['notes']:
            print('  note:', n)
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=1)
    if a.table:
        print(format_aggregate(aggregate(results)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
