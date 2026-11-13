"""Objective check of the fist punch direction in a recorded in-game run: how high does the striking arm point at the impact, is V
upright, and is the hand pop-free. Read-only on every input.

  python -I punch_direction_report.py RUN_DIR [--out report.json] [--sheet sheet.png]
  python -I punch_direction_report.py --log utpp-2026-....log --pose utpp-pose-....utpppose [--game-dir DIR] [--out ...] [--sheet ...]

RUN_DIR is a harness run folder: the session log is taken from its logs\\ folder, the compact pose
recording (utpp-pose-*.utpppose, normally in <game>\\red4ext\\logs and read in place) is found through the log line "pose capture: started
file=..." relative to --game-dir, and gametest\\summary.json (when present) names the scenario step of every punch. With --log / --pose both
are given explicitly.

What it does
  1. Melee part: from the log. MMMASK on/off lines whose `melee=1` give the melee windows, MMMELEEFULL on/off the full-body windows,
     MELEEAIM lines the tilt (the nearest line, `src` verbatim), the last `body=woman|man` line before the first window the body.
  2. Punches: inside every melee window the striking arm is the one whose hand gets furthest from its shoulder joint; a punch is a
     local maximum of (hand - shoulder distance) / arm length of at least --min-extension, at least --min-gap s from another punch, not
     within --skip-after-mask s of the mask start. Frames with both arms fully stretched at once (a one-frame T pose at a clip seam)
     are not punches; they are counted as `seam_frames`.
  3. Clip: when the built fist clips are found (Build\\work\\melee-npc-20261109\\clips for woman, ...\\man\\clips for man, or --clips-dir)
     the clip is identified by matching the local rotations of arms, neck and head (the bones the game takes from the clip) of the
     frames around the peak with every clip frame, over the segment around the peak (mean of the per-frame match, each clamped to 15 deg). The impact frame is the segment frame whose matched
     clip frame is nearest the clip's impact_built frame. Without clips (or when no clip matches within --max-clip-rms) the
     report says so, the peak-extension frame is used and the elevation check cannot be evaluated.
  4. Numbers per punch, for the pose AFTER the mod's write (stage 1 = verdict) and BEFORE it (stage 0 = the engine pose, printed
     beside it), model space (Z up, +Y forward): shoulder-to-wrist elevation above horizontal and azimuth vs +Y (- = left), wrist
     height relative to shoulder and to the Head bone (cm), trunk lean Hips->Spine3 (+ forward), chest / head pitch (+ up; the local
     forward axes are calibrated on the idle frames before the first melee window), hips height above the root and above the lower foot
     (cm), the kneeling flag, full body on, the mask blend (`in_blend` = impact within --blend s of the mask start: stage 1 is still
     rising: a warning, the verdict stays on stage 1 = what the player sees; stage 0 is printed beside it), melee tilt and its source, body. With a clip the same numbers
     for the clip frame and the difference.
  5. Pop indicator: the largest single-frame jump of both hands (world position, cm) and of hands and upper arms (local rotation,
     deg) over all melee windows (stage 1, frame pairs further apart than 2.5 frames skipped), with its time, plus the clip's own largest
     per-clip-frame step for reference.

Verdict per punch (limits are arguments): |elevation - clip elevation| <= --max-elev-diff (8), hips >= --kneel-hips-cm (60) above the
root, and for straight punches the wrist not above the Head bone by more than --max-wrist-above-head-cm (0; ignored when the clip itself
is that high). A punch whose clip is not a straight punch (final / crouch / jump / hook ...) is exempt from the wrist check; without a
clip the punch is assumed straight (`straight_assumed`). Exit code: 1 = a hard check failed (kneeling, wrist above head, elevation
off), 2 = could not evaluate (no melee window / pose / punch, or an elevation check without a clip and nothing failed), 0 = all evaluated
and passed.

Limits of the tool: see the LIMITS list at the end of the --out JSON ("limits_of_tool"). The sheet is a side-view figure; project rules
keep .png files in Media\\png\\<date>_<topic>\\, so pass a scratch path and move it.
"""
import argparse
import bisect
import datetime
import glob
import json
import math
import os
import pathlib
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import pose_helper_report as ph  # noqa: E402

ROOT = pathlib.Path(HERE).resolve().parents[2]
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utpp_paths import game_dir as _utpp_game_dir  # noqa: E402
DEFAULT_GAME_DIR = _utpp_game_dir(required=False) or ''
DEFAULT_CLIPS_ROOT = ROOT / 'Build' / 'work' / 'melee-npc-20261109'
EPOCH = datetime.datetime(1970, 1, 1)

CHAIN = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head', 'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand',
         'LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand', 'LeftUpLeg', 'LeftLeg', 'LeftFoot', 'RightUpLeg', 'RightLeg', 'RightFoot']
MATCH_BONES = ['RightShoulder', 'RightArm', 'RightForeArm', 'RightHand', 'LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand', 'Neck',
               'Neck1', 'Head']
BONES_DRAWN = [('Hips', 'Spine'), ('Spine', 'Spine1'), ('Spine1', 'Spine2'), ('Spine2', 'Spine3'), ('Spine3', 'Neck'), ('Neck', 'Neck1'),
               ('Neck1', 'Head'), ('Spine3', 'RightShoulder'), ('RightShoulder', 'RightArm'), ('RightArm', 'RightForeArm'),
               ('RightForeArm', 'RightHand'), ('Spine3', 'LeftShoulder'), ('LeftShoulder', 'LeftArm'), ('LeftArm', 'LeftForeArm'),
               ('LeftForeArm', 'LeftHand'), ('Hips', 'LeftUpLeg'), ('LeftUpLeg', 'LeftLeg'), ('LeftLeg', 'LeftFoot'),
               ('Hips', 'RightUpLeg'), ('RightUpLeg', 'RightLeg'), ('RightLeg', 'RightFoot')]
POP_BONES = ['LeftHand', 'RightHand', 'LeftArm', 'RightArm']
NOT_STRAIGHT = re.compile(r'final|crouch|jump|hook|upper|kick|heavy|slam', re.IGNORECASE)
NOT_PREFERRED = re.compile(r'_hit$|_deflect$|^jump', re.IGNORECASE)
PUNCH_FAMILY = re.compile(r'^(combo|final|crouch)_attack', re.IGNORECASE)
MATCH_CLAMP_DEG = 15.0      # a frame that matches no clip frame (mask blend, seam) counts at most this much
LIMITS_OF_TOOL = [
    'Detects punches only from the extension of the striking arm in the recorded pose (a one-arm lunge or a block with a stretched arm '
    'looks the same; both arms stretched at once is treated as a seam T pose and dropped).',
    'The clip is identified from arm / neck / head rotations; without the built clips (Build\\work folders are temporary) there is no clip '
    'elevation, so the elevation verdict cannot be evaluated (exit 2) and the impact frame is the peak-extension frame.',
    'Hips, Spine and legs are the engine pose, not the clip, so the clip comparison is by elevation of the arm in model space only.',
    'The impact frame of a clip whose impact_built is not reached in the segment (a lunge cut short) is the nearest frame and is flagged.',
    'The pose recording has no ground or target data: "above the ground" is the Root bone (z = 0) and "kneeling" is a hips-height '
    'threshold. Head / chest pitch need idle frames before the first melee window (else null).',
    'Stage 1 inside the first 0.3 s of the mask is still blending (in_blend, a warning): the verdict is still on stage 1; read the stage-0 '
    'numbers (the engine pose) beside it. The first punch of a combo is what switches the mask on, so it is usually in the blend.',
    'Pop indicator: jumps are measured between recorded frames; a one-frame spike that the recording missed (dropped frames) is not seen.',
    'Only fists clips (pwa_fists__* / pma_fists__*) were available when written; other melee weapons use the same method but are untested.',
    'It measures the animation pose, not the skinned mesh: a pass is not proof that the body looks right.',
]


class Limits:
    def __init__(self, **kw):
        self.max_elev_diff = 8.0
        self.kneel_hips_cm = 60.0
        self.max_wrist_above_head_cm = 0.0
        self.min_extension = 0.8
        self.min_gap_s = 0.30
        self.skip_after_mask_s = 0.15
        self.blend_s = 0.30
        self.aim_max_age_s = 0.30
        self.max_clip_rms = 8.0
        self.seam_extension = 0.9
        self.pop_max_dt_frames = 2.5
        for k, v in kw.items():
            if not hasattr(self, k):
                raise TypeError('unknown option ' + k)
            setattr(self, k, v)

    def as_dict(self):
        return dict(self.__dict__)


# --------------------------------------------------------------------------------------------------------------------- quaternion math
def qnorm(q):
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / np.where(n > 1e-9, n, np.nan)


def rotmat(q):
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    r = np.empty(q.shape[:-1] + (3, 3))
    r[..., 0, 0] = 1 - 2 * (y * y + z * z)
    r[..., 0, 1] = 2 * (x * y - z * w)
    r[..., 0, 2] = 2 * (x * z + y * w)
    r[..., 1, 0] = 2 * (x * y + z * w)
    r[..., 1, 1] = 1 - 2 * (x * x + z * z)
    r[..., 1, 2] = 2 * (y * z - x * w)
    r[..., 2, 0] = 2 * (x * z - y * w)
    r[..., 2, 1] = 2 * (y * z + x * w)
    r[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return r


def quat_angle_deg(a, b):
    """Rotation angle (deg) between two xyzw quaternions (arrays broadcast); sign-safe."""
    d = np.abs(np.sum(a * b, axis=-1))
    return 2.0 * np.degrees(np.arccos(np.clip(d, 0.0, 1.0)))


def elevation_deg(v):
    n = np.linalg.norm(v)
    return float(np.degrees(np.arcsin(np.clip(v[2] / n, -1.0, 1.0)))) if n > 1e-9 else float('nan')


def azimuth_deg(v):
    return float(np.degrees(np.arctan2(v[0], v[1])))


# --------------------------------------------------------------------------------------------------------------------- time
def stamp_epoch(date, hms):
    y, m, d = (int(x) for x in date.split('-'))
    h, mi, s = hms.split(':')
    return (datetime.datetime(y, m, d) - EPOCH).total_seconds() + int(h) * 3600 + int(mi) * 60 + float(s)


def clock(t):
    return (EPOCH + datetime.timedelta(seconds=float(t))).strftime('%H:%M:%S.%f')[:-3]


# --------------------------------------------------------------------------------------------------------------------- log
LINE_RE = re.compile(r'^\[(\d{4}-\d\d-\d\d) (\d\d:\d\d:\d\d(?:\.\d+)?)\]')
MASK_RE = re.compile(r'UTPP MMMASK (on|off)\b(.*)')
FULL_RE = re.compile(r'UTPP MMMELEEFULL (on|off)\b')
AIM_RE = re.compile(r'UTPP MELEEAIM src=(\w+) tiltW=(-?[\d.]+) tgtPitch=(-?[\d.]+) tgtYaw=(-?[\d.]+) rawPitch=(-?[\d.]+) rawYaw=(-?[\d.]+) '
                    r'pitch=(-?[\d.]+) yaw=(-?[\d.]+)')
BODY_RE = re.compile(r'\bbody=(woman|man)\b')
POSE_RE = re.compile(r'pose capture: started file=(\S+\.utpppose)')


class LogInfo:
    def __init__(self):
        self.melee_windows = []      # (start, end) seconds
        self.full_windows = []
        self.mask_full = []          # (t, full weight) of MMMASK lines
        self.aim = []                # dicts
        self.bodies = []             # (t, body)
        self.pose_files = []         # (t, path as logged)
        self.last_t = None


def _windows(events, last_t):
    out, cur = [], None
    for t, on, keep in events:
        if on and keep:
            if cur is None:
                cur = t
        elif cur is not None:
            out.append((cur, t))
            cur = None
    if cur is not None:
        out.append((cur, max(last_t, cur)))
    return out


def parse_log(path):
    info = LogInfo()
    mask_ev, full_ev = [], []
    with open(path, encoding='utf-8', errors='replace') as f:
        for ln in f:
            m = LINE_RE.match(ln)
            if not m:
                continue
            t = stamp_epoch(m.group(1), m.group(2))
            info.last_t = t
            body = ln[m.end():]
            if 'UTPP MMMASK' in body:
                mm = MASK_RE.search(body)
                if mm:
                    on = mm.group(1) == 'on'
                    me = re.search(r'\bmelee=(\d)', mm.group(2))
                    mask_ev.append((t, on, bool(me and me.group(1) == '1')))
                    fu = re.search(r'\bfull=([\d.]+)', mm.group(2))
                    if fu:
                        info.mask_full.append((t, float(fu.group(1))))
            elif 'UTPP MMMELEEFULL' in body:
                mm = FULL_RE.search(body)
                if mm:
                    full_ev.append((t, mm.group(1) == 'on', True))
            elif 'UTPP MELEEAIM' in body:
                mm = AIM_RE.search(body)
                if mm:
                    info.aim.append(dict(t=t, src=mm.group(1), tilt_weight=float(mm.group(2)), target_pitch=float(mm.group(3)),
                                         target_yaw=float(mm.group(4)), applied_pitch=float(mm.group(7)), applied_yaw=float(mm.group(8))))
            else:
                if 'pose capture: started' in body:
                    mm = POSE_RE.search(body)
                    if mm:
                        info.pose_files.append((t, mm.group(1)))
                mm = BODY_RE.search(body)
                if mm:
                    info.bodies.append((t, mm.group(1)))
    last = info.last_t or 0.0
    info.melee_windows = _windows(mask_ev, last)
    info.full_windows = _windows(full_ev, last)
    return info


def body_before(info, t, override=None):
    if override:
        return override
    best = None
    for tb, b in info.bodies:
        if tb <= t:
            best = b
    if best is None and info.bodies:
        best = info.bodies[0][1]
    return best


def nearest_aim(info, t, max_age):
    if not info.aim:
        return None
    ts = [a['t'] for a in info.aim]
    i = bisect.bisect_left(ts, t)
    cands = [info.aim[j] for j in (i - 1, i) if 0 <= j < len(info.aim)]
    a = min(cands, key=lambda a: abs(a['t'] - t))
    if abs(a['t'] - t) > max_age:
        return None
    r = dict(a)
    r['age_s'] = round(a['t'] - t, 3)
    return r


def in_windows(t, windows):
    for a, b in windows:
        if a <= t <= b:
            return True
    return False


def full_weight_at(info, t):
    w = None
    for tt, v in info.mask_full:
        if tt <= t:
            w = v
        else:
            break
    return w


# --------------------------------------------------------------------------------------------------------------------- pose
class PoseData:
    """The recorded frames of the bones needed (stage 0 = before the mod's write, stage 1 = after it), local transforms in cm / unit quaternions."""

    def __init__(self, names, parent, t, P, Q):
        self.names, self.parent, self.t, self.P, self.Q = names, parent, t, P, Q     # P [n, nb, 2, 3] cm, Q [n, nb, 2, 4]
        self.cat = {n: k for k, n in enumerate(names)}
        order, seen = [], set()

        def visit(b):
            if b in seen:
                return
            if parent[b] >= 0:
                visit(parent[b])
            seen.add(b)
            order.append(b)
        for b in range(len(names)):
            visit(b)
        self.order = order
        self._world = {}

    def world(self, stage):
        if stage in self._world:
            return self._world[stage]
        P, Q = self.P[:, :, stage], self.Q[:, :, stage]
        n, nb = P.shape[:2]
        wp = np.zeros((n, nb, 3))
        wr = np.zeros((n, nb, 3, 3))
        for b in self.order:
            r = rotmat(Q[:, b])
            p = self.parent[b]
            if p < 0:
                wp[:, b], wr[:, b] = P[:, b], r
            else:
                wp[:, b] = wp[:, p] + np.einsum('nij,nj->ni', wr[:, p], P[:, b])
                wr[:, b] = np.einsum('nij,njk->nik', wr[:, p], r)
        self._world[stage] = (wp, wr)
        return wp, wr

    def pose_at(self, k, stage):
        wp, wr = self.world(stage)
        return {n: (wp[k, self.cat[n]], wr[k, self.cat[n]]) for n in CHAIN if n in self.cat}


def load_pose(paths, t_lo, t_hi, notes=None):
    """Reads the recordings (stream 0 only), keeps the frames inside [t_lo, t_hi] (seconds since 1970 of the local clock) and the CHAIN bones with
    their ancestors. Returns PoseData or None."""
    groups = ph.load_groups(list(paths), ph.Config())
    for g in groups:
        h = g['header']
        mapped = h.mapped()
        names = [h.names[i] for i in mapped]
        rig = [h.game_index[i] for i in mapped]
        rig2cat = {r: k for k, r in enumerate(rig)}
        par = [rig2cat.get(h.parents[r], -1) if r < len(h.parents) else -1 for r in rig]
        cat = {n: k for k, n in enumerate(names)}
        need = set()
        for n in CHAIN:
            k = cat.get(n)
            while k is not None and k >= 0 and k not in need:
                need.add(k)
                k = par[k] if par[k] >= 0 else None
        sel = sorted(need)
        remap = {k: i for i, k in enumerate(sel)}
        day0 = (datetime.date.fromisoformat(g['day']) - EPOCH.date()).days * 86400.0
        ts, vs = [], []
        for stream, _ticks, sec, _gidx, v in ph.iter_blocks(g):
            if stream != 0:
                continue
            tt = day0 + sec
            m = (tt >= t_lo) & (tt <= t_hi)
            if not m.any():
                continue
            ts.append(tt[m])
            vs.append(v[m][:, sel].astype(np.float64))
        if notes is not None:
            notes.extend(g.get('notes', []))
        if not ts:
            continue
        t = np.concatenate(ts)
        V = np.concatenate(vs)                                        # [n, nb, 2, 12]
        P = V[:, :, :, 0:3] * 100.0
        Q = qnorm(V[:, :, :, 4:8])
        sel_names = [names[k] for k in sel]
        sel_par = [remap.get(par[k], -1) if par[k] >= 0 else -1 for k in sel]
        return PoseData(sel_names, sel_par, t, P, Q)
    return None


# --------------------------------------------------------------------------------------------------------------------- clips
class Clip:
    def __init__(self, name, lr, n152, W, names, info):
        self.name, self.lr, self.n152, self.W, self.names, self.info = name, lr, n152, W, names, info
        self.frames = lr.shape[0]

    def impact_frame(self, side):
        ib = self.info.get('impact_built')
        if ib:
            return int(ib[0] if isinstance(ib, (list, tuple)) else ib)
        if self.W is not None and self.frames:
            s = self.info.get('side', side)
            a, h = self.names.index(s + 'Arm'), self.names.index(s + 'Hand')
            ext = np.linalg.norm(self.W[:, h, :3, 3] - self.W[:, a, :3, 3], axis=-1)
            return int(np.argmax(ext))
        return 0

    def pose_at(self, f):
        return {n: (self.W[f, self.names.index(n), :3, 3] * 100.0, self.W[f, self.names.index(n), :3, :3]) for n in CHAIN if n in self.names}


def load_clips(folder):
    clips = {}
    for f in sorted(glob.glob(os.path.join(folder, '*.npz'))):
        base = os.path.basename(f)
        if '__' not in base:
            continue
        name = base.split('__', 1)[1].split('.')[0]
        with np.load(f, allow_pickle=True) as z:
            n152 = [str(x) for x in z['names152']]
            if not all(b in n152 for b in MATCH_BONES):
                continue
            names = [str(x) for x in z['names']] if 'names' in z.files else []
            W = z['W'].astype(np.float64) if 'W' in z.files else None
            lr = z['lr'].astype(np.float64)[:, [n152.index(b) for b in MATCH_BONES]]
        info = {}
        ip = f[:-len('.npz')] + '.info.json'
        if os.path.exists(ip):
            with open(ip, encoding='utf-8') as fh:
                info = json.load(fh)
        clips[name] = Clip(name, lr, list(MATCH_BONES), W, names, info)
    return clips


def default_clips_dir(body):
    return str(DEFAULT_CLIPS_ROOT / 'man' / 'clips') if body == 'man' else str(DEFAULT_CLIPS_ROOT / 'clips')


def match_segment(clips, q0):
    """q0 [K, 11, 4]: engine locals of MATCH_BONES at the segment frames. Returns (per clip name: (median rms deg, per-frame best clip frame,
    per-frame rms)), candidate order."""
    names = list(clips)
    stack = np.concatenate([clips[n].lr for n in names], axis=0)               # [F, 11, 4]
    starts = np.cumsum([0] + [clips[n].frames for n in names])[:-1]
    d = np.abs(np.einsum('fbc,kbc->kfb', stack, q0))
    ang = 2.0 * np.degrees(np.arccos(np.clip(d, 0.0, 1.0)))
    cost = np.sqrt(np.mean(ang ** 2, axis=-1))                                   # [K, F]
    res = {}
    for i, n in enumerate(names):
        a, b = starts[i], starts[i] + clips[n].frames
        blk = cost[:, a:b]
        fbest = np.argmin(blk, axis=1)
        rms = blk[np.arange(blk.shape[0]), fbest]
        res[n] = (float(np.mean(np.minimum(rms, MATCH_CLAMP_DEG))), fbest, rms)
    return res


def pick_clip(res, tie_deg=0.8):
    """Best clip by the mean clamped match cost over the segment. Clips whose arm data is shared (one punch reused as safe_attack / jump_attack /
    block_attack ...) tie: within tie_deg the punch families (combo / final / crouch attack) win, then the clips that are not hit / deflect /
    jump variants; the rest of the ties are listed as alternates."""
    order = sorted(res, key=lambda n: res[n][0])
    best = res[order[0]][0]
    near = [n for n in order if res[n][0] <= best + tie_deg]
    ranked = ([n for n in near if PUNCH_FAMILY.search(n) and not NOT_PREFERRED.search(n)] + [n for n in near if not NOT_PREFERRED.search(n)] + near)
    chosen = ranked[0]
    return chosen, [n for n in near if n != chosen]


def clip_base(name, clips):
    for suf in ('_hit', '_deflect'):
        if name.endswith(suf) and name[:-len(suf)] in clips:
            return name[:-len(suf)]
    return name


# --------------------------------------------------------------------------------------------------------------------- metrics
def calibrate_axes(pd, t_lo, t_hi):
    """Local forward axis (unit vector in the bone's local frame) of Head and Spine3 from the stage 1 mean rotation of the idle frames."""
    m = (pd.t >= t_lo) & (pd.t <= t_hi)
    if m.sum() < 10:
        m = pd.t <= pd.t[0] + 1.0
    if m.sum() < 5:
        return None
    wp, wr = pd.world(1)
    ax = {}
    for n in ('Head', 'Spine3'):
        if n not in pd.cat:
            continue
        rm = wr[m][:, pd.cat[n]].mean(axis=0)
        u, _s, vt = np.linalg.svd(rm)
        rm = u @ vt
        best = max(((float(rm[1, i]) * s, i, s) for i in range(3) for s in (1.0, -1.0)))
        e = np.zeros(3)
        e[best[1]] = best[2]
        ax[n] = e
    return ax


def metrics(d, side, ax):
    """d: name -> (position cm, rotation matrix) in model space. Returns the numbers of one pose."""
    sh, wr = d[side + 'Arm'][0], d[side + 'Hand'][0]
    v = wr - sh
    out = dict(elevation_deg=elevation_deg(v), azimuth_deg=azimuth_deg(v), extension_cm=float(np.linalg.norm(v)),
               wrist_above_shoulder_cm=float(v[2]), wrist_above_head_cm=float(wr[2] - d['Head'][0][2]))
    t = d['Spine3'][0] - d['Hips'][0]
    out['trunk_lean_deg'] = float(np.degrees(np.arctan2(t[1], t[2])))
    for n, key in (('Head', 'head_pitch_deg'), ('Spine3', 'chest_pitch_deg')):
        out[key] = elevation_deg(d[n][1] @ ax[n]) if ax and n in ax else None
    out['hips_above_root_cm'] = float(d['Hips'][0][2])
    feet = [d[f][0][2] for f in ('LeftFoot', 'RightFoot') if f in d]
    out['hips_above_feet_cm'] = float(d['Hips'][0][2] - min(feet)) if feet else None
    return out


def r2(x, nd=1):
    return None if x is None or (isinstance(x, float) and not math.isfinite(x)) else round(float(x), nd)


def rounded(m):
    return {k: r2(v) for k, v in m.items()} if m else m


# --------------------------------------------------------------------------------------------------------------------- detection
def arm_lengths(pd):
    out = {}
    for s in ('Right', 'Left'):
        fa, ha = pd.cat.get(s + 'ForeArm'), pd.cat.get(s + 'Hand')
        if fa is None or ha is None:
            continue
        out[s] = float(np.median(np.linalg.norm(pd.P[:, fa, 1], axis=-1) + np.linalg.norm(pd.P[:, ha, 1], axis=-1)))
    return out


def detect_punches(pd, windows, lim, lengths):
    """-> (punches [dict(side, k, t, window, ext)], seam frames [t])"""
    wp, _ = pd.world(1)
    ext = {}
    for s, L in lengths.items():
        a, h = pd.cat[s + 'Arm'], pd.cat[s + 'Hand']
        ext[s] = np.linalg.norm(wp[:, h] - wp[:, a], axis=-1) / L
    seams, cands = [], []
    for wi, (a, b) in enumerate(windows):
        ix = np.nonzero((pd.t >= a + lim.skip_after_mask_s) & (pd.t <= b))[0]
        if len(ix) < 3:
            continue
        both = np.ones(len(ix), bool)
        for s in ext:
            both &= ext[s][ix] >= lim.seam_extension
        if len(ext) == 2:
            seams += [float(pd.t[ix[j]]) for j in np.nonzero(both)[0]]
        for s, e in ext.items():
            ee = e[ix]
            for j in range(1, len(ix) - 1):
                if ee[j] >= lim.min_extension and ee[j] >= ee[j - 1] and ee[j] > ee[j + 1] and not (len(ext) == 2 and both[j]):
                    cands.append((float(ee[j]), s, int(ix[j]), wi))
    cands.sort(key=lambda c: -c[0])
    taken = []
    for e, s, k, wi in cands:
        if all(abs(pd.t[k] - pd.t[k2]) >= lim.min_gap_s for _, _, k2, _ in taken):
            taken.append((e, s, k, wi))
    taken.sort(key=lambda c: c[2])
    return [dict(side=s, k=k, t=float(pd.t[k]), window=wi, ext=e) for e, s, k, wi in taken], seams


def pop_report(pd, windows, lim, clips_used):
    wp, _ = pd.world(1)
    best = {}
    for b in POP_BONES:
        if b not in pd.cat:
            continue
        c = pd.cat[b]
        rot = dict(deg=0.0, t=None)
        pos = dict(cm=0.0, t=None)
        for a, e in windows:
            ix = np.nonzero((pd.t >= a) & (pd.t <= e))[0]
            if len(ix) < 2:
                continue
            dt = np.diff(pd.t[ix])
            ok = dt <= lim.pop_max_dt_frames / 60.0
            dr = quat_angle_deg(pd.Q[ix[:-1], c, 1], pd.Q[ix[1:], c, 1])
            dp = np.linalg.norm(wp[ix[1:], c] - wp[ix[:-1], c], axis=-1)
            for arr, rec, key in ((dr, rot, 'deg'), (dp, pos, 'cm')):
                if ok.any():
                    j = int(np.argmax(np.where(ok, arr, -1.0)))
                    if arr[j] > rec[key]:
                        rec[key], rec['t'] = float(arr[j]), float(pd.t[ix[j + 1]])
        best[b] = dict(max_rotation_jump_deg=r2(rot['deg']), rotation_jump_time=clock(rot['t']) if rot['t'] else None,
                       max_position_jump_cm=r2(pos['cm']), position_jump_time=clock(pos['t']) if pos['t'] else None)
    refs = {}
    for name, c in clips_used.items():
        if c.W is None or c.frames < 2 or not c.names:
            continue
        step = {}
        for b in ('LeftHand', 'RightHand'):
            if b in c.names:
                i = c.names.index(b)
                step[b] = float(np.max(np.linalg.norm(np.diff(c.W[:, i, :3, 3], axis=0), axis=-1)) * 100.0)
        arm = [float(np.max(quat_angle_deg(c.lr[:-1, MATCH_BONES.index(b)], c.lr[1:, MATCH_BONES.index(b)]))) for b in ('LeftArm', 'RightArm')]
        refs[name] = dict(max_hand_step_cm_per_clip_frame=r2(max(step.values()) if step else None),
                          max_upper_arm_rotation_step_deg_per_clip_frame=r2(max(arm)))
    worst_rot = max(best.items(), key=lambda kv: kv[1]['max_rotation_jump_deg'] or 0.0) if best else None
    worst_pos = max(best.items(), key=lambda kv: kv[1]['max_position_jump_cm'] or 0.0) if best else None
    return dict(per_bone=best, clip_reference=refs,
                largest_rotation_jump=dict(bone=worst_rot[0], **worst_rot[1]) if worst_rot else None,
                largest_hand_position_jump=dict(bone=worst_pos[0], **worst_pos[1]) if worst_pos else None)


# --------------------------------------------------------------------------------------------------------------------- verdict
def judge(p, lim):
    fails, warns, unevaluated = [], [], []
    g1 = p['game']['stage1']
    if g1['hips_above_root_cm'] < lim.kneel_hips_cm:
        fails.append('kneeling_or_crouched: hips %.0f cm above the root (< %.0f)' % (g1['hips_above_root_cm'], lim.kneel_hips_cm))
    if p['in_blend']:
        warns.append('impact inside the mask blend (stage 1 may not be settled; verdict on stage 1, stage 0 elevation %+.1f)' %
                     p['game']['stage0']['elevation_deg'])
    clip = p['clip']
    if clip['known']:
        worst = abs(g1['elevation_deg'] - clip['pose']['elevation_deg'])
        p['elevation_diff_deg'] = r2(g1['elevation_deg'] - clip['pose']['elevation_deg'])
        p['elevation_diff_stage0_deg'] = r2(p['game']['stage0']['elevation_deg'] - clip['pose']['elevation_deg'])
        if worst > lim.max_elev_diff:
            fails.append('elevation %+.1f deg vs clip %+.1f (diff %+.1f, limit %.0f)' % (
                g1['elevation_deg'], clip['pose']['elevation_deg'], g1['elevation_deg'] - clip['pose']['elevation_deg'], lim.max_elev_diff))
        if not p['impact_reached']:
            warns.append('clip impact frame not reached in the segment (nearest frame used)')
    else:
        unevaluated.append('elevation_vs_clip: ' + clip['reason'])
    if p['straight']:
        wr = g1['wrist_above_head_cm']
        w = wr
        clip_wr = clip['pose']['wrist_above_head_cm'] if clip['known'] else None
        if w > lim.max_wrist_above_head_cm and not (clip_wr is not None and clip_wr > lim.max_wrist_above_head_cm):
            fails.append('wrist %.0f cm above the Head bone (limit %.0f)' % (w, lim.max_wrist_above_head_cm))
        if not clip['known']:
            warns.append('straight punch assumed (no clip)')
    p['kneeling'] = bool(g1['hips_above_root_cm'] < lim.kneel_hips_cm)
    if clip['known']:
        p['diff_vs_clip'] = {k: r2(g1[k] - clip['pose'][k]) for k in g1 if g1[k] is not None and clip['pose'].get(k) is not None}
    p['fails'], p['warnings'], p['unevaluated'] = fails, warns, unevaluated
    p['verdict'] = 'FAIL' if fails else ('UNEVALUATED' if unevaluated else 'PASS')


def overall(punches, have_window, have_pose):
    if not have_pose or not have_window or not punches:
        return 2
    if any(p['verdict'] == 'FAIL' for p in punches):
        return 1
    if any(p['verdict'] == 'UNEVALUATED' for p in punches):
        return 2
    return 0


# --------------------------------------------------------------------------------------------------------------------- analysis
def resolve_pose_files(info, game_dir, t_first, t_last):
    """The logged pose files that cover [t_first, t_last]: the last one started before t_first and every later one started before t_last."""
    started = sorted(info.pose_files)
    keep, last_before = [], None
    for t, p in started:
        if t <= t_first:
            last_before = (t, p)
        elif t <= t_last:
            keep.append((t, p))
    if last_before:
        keep.insert(0, last_before)
    out = []
    for _t, p in keep:
        full = p if os.path.isabs(p) else os.path.join(game_dir, p)
        if full not in out:
            out.append(full)
    return out


def analyze(log_path, pose_paths=None, game_dir=DEFAULT_GAME_DIR, clips_dir=None, body=None, lim=None, steps=None, log=None):
    """Returns the report dict (plus the private '_draw' list for the sheet)."""
    lim = lim or Limits()
    log = log or (lambda *_: None)
    info = parse_log(log_path)
    rep = dict(tool='punch_direction_report', log=os.path.basename(log_path), limits=lim.as_dict(), notes=[], punches=[], pop=None,
               windows=[], limits_of_tool=LIMITS_OF_TOOL)
    rep['windows'] = [dict(start=clock(a), end=clock(b), full_body_windows=[[clock(x), clock(y)] for x, y in info.full_windows if x < b and y > a])
                      for a, b in info.melee_windows]
    if not info.melee_windows:
        rep['notes'].append('no melee window (MMMASK on with melee=1) in the log')
        rep['exit_code'] = 2
        return rep
    t_first, t_last = info.melee_windows[0][0], info.melee_windows[-1][1]
    if not pose_paths:
        pose_paths = resolve_pose_files(info, game_dir, t_first, t_last)
        rep['notes'].append('pose files from the log: ' + (', '.join(os.path.basename(p) for p in pose_paths) or 'none'))
    pose_paths = [p for p in pose_paths if os.path.exists(p)]
    if not pose_paths:
        rep['notes'].append('no pose recording found (not logged, or not in %s)' % game_dir)
        rep['exit_code'] = 2
        return rep
    rep['pose_files'] = [os.path.basename(p) for p in pose_paths]
    pnotes = []
    pd = load_pose(pose_paths, t_first - 2.5, t_last + 0.5, pnotes)
    rep['notes'] += pnotes
    if pd is None or len(pd.t) < 10 or any(n not in pd.cat for n in ('Hips', 'RightArm', 'RightHand', 'LeftArm', 'LeftHand', 'Head', 'Spine3')):
        rep['notes'].append('the pose recording does not cover the melee windows (or lacks the core bones)')
        rep['exit_code'] = 2
        return rep
    body = body_before(info, t_first, body)
    rep['body'] = body
    clips, clips_note = {}, None
    cdir = clips_dir or default_clips_dir(body)
    if os.path.isdir(cdir):
        clips = load_clips(cdir)
        if not clips:
            clips_note = 'no usable clip files in %s' % cdir
    else:
        clips_note = 'clips folder not found: %s' % cdir
    rep['clips_dir'] = cdir if clips else None
    rep['clips_loaded'] = len(clips)
    if clips_note:
        rep['notes'].append(clips_note + ' (clip comparison unavailable)')
    ax = calibrate_axes(pd, t_first - 1.5, t_first - 0.2)
    if ax is None:
        rep['notes'].append('no idle frames before the first melee window: head / chest pitch not calibrated')
    lengths = arm_lengths(pd)
    rep['arm_length_cm'] = {k: r2(v) for k, v in lengths.items()}
    found, seams = detect_punches(pd, info.melee_windows, lim, lengths)
    rep['seam_frames'] = [clock(t) for t in seams]
    wp1, _ = pd.world(1)
    draw, used = [], {}
    for n, c in enumerate(found, 1):
        k, side, t = c['k'], c['side'], c['t']
        a_win, b_win = info.melee_windows[c['window']]
        seg = np.nonzero((pd.t >= max(t - 0.30, a_win)) & (pd.t <= min(t + 0.10, b_win)))[0]
        clip_rec = dict(known=False, reason='no clips loaded' if not clips else '')
        k_imp, reached, f_imp = k, True, None
        if clips and len(seg) >= 3:
            q0 = pd.Q[seg][:, [pd.cat[b] for b in MATCH_BONES], 0]
            res = match_segment(clips, q0)
            chosen, alts = pick_clip(res)
            rms = res[chosen][0]
            clip_rec.update(name=chosen, rms_deg=r2(rms), alternates=alts)
            if rms > lim.max_clip_rms or clips[clip_base(chosen, clips)].W is None:
                clip_rec['reason'] = ('match rms %.1f deg above --max-clip-rms %.1f' % (rms, lim.max_clip_rms)) if rms > lim.max_clip_rms \
                    else 'clip has no model-space data (W)'
                clip_rec['known'] = False
            else:
                base = clip_base(chosen, clips)
                cb = clips[base]
                f_imp = cb.impact_frame(side)
                fm, cs = res[chosen][1], res[chosen][2]
                good = np.nonzero(cs < max(10.0, lim.max_clip_rms))[0]
                good = good if len(good) else np.arange(len(seg))
                j = min(good, key=lambda g: (abs(int(fm[g]) - f_imp), abs(pd.t[seg[g]] - t)))
                k_imp = int(seg[j])
                f_game = int(fm[j])
                reached = abs(f_game - f_imp) <= 2
                src = clips[chosen] if clips[chosen].W is not None else cb      # the matched clip's own frame (a _hit clip has its own data)
                cpose = src.pose_at(f_game if src is clips[chosen] else f_imp)
                clip_rec.update(known=True, reason='', frame_matched=f_game, impact_frame=f_imp, base=base,
                                pose=metrics(cpose, side, ax), _pose=cpose)
                used[base] = cb
                used[chosen] = clips[chosen]
        elif clips:
            clip_rec['reason'] = 'segment too short for matching'
        stage = {s: metrics(pd.pose_at(k_imp, s), side, ax) for s in (0, 1)}
        since = float(pd.t[k_imp] - a_win)
        straight = not (clip_rec['known'] and NOT_STRAIGHT.search(clip_rec['base']))
        aim = nearest_aim(info, pd.t[k_imp], lim.aim_max_age_s)
        step_label = None
        for st in steps or []:
            if st['t0'] <= pd.t[k_imp] <= st['t1']:
                step_label = '%s/%s' % (st['scenario'], st['step'])
        p = dict(n=n, time=clock(pd.t[k_imp]), peak_time=clock(t), side=side, step=step_label, window=c['window'], body=body,
                 peak_extension=r2(c['ext'], 2), t_since_mask_on_s=r2(since, 2), in_blend=bool(since < lim.blend_s),
                 full_body_on=in_windows(pd.t[k_imp], info.full_windows), full_weight=r2(full_weight_at(info, pd.t[k_imp]), 2),
                 tilt=None if aim is None else dict(source=aim['src'], weight=aim['tilt_weight'], pitch_deg=aim['applied_pitch'],
                                                    yaw_deg=aim['applied_yaw'], target_pitch_deg=aim['target_pitch'], age_s=aim['age_s']),
                 impact_reached=reached, straight=bool(straight), straight_assumed=not clip_rec['known'],
                 game=dict(stage0=stage[0], stage1=stage[1]), clip=clip_rec)
        judge(p, lim)
        pub = dict(p)
        pub['game'] = dict(stage0=rounded(stage[0]), stage1=rounded(stage[1]))
        pub['clip'] = {k2: v for k2, v in clip_rec.items() if not k2.startswith('_')}
        if clip_rec['known']:
            pub['clip']['pose'] = rounded(clip_rec['pose'])
        rep['punches'].append(pub)
        draw.append(dict(n=n, side=side, time=pub['time'], verdict=p['verdict'], elev=stage[1]['elevation_deg'],
                         game1=pd.pose_at(k_imp, 1), game0=pd.pose_at(k_imp, 0), clip=clip_rec.get('_pose'),
                         clip_name=clip_rec.get('name'), clip_elev=clip_rec['pose']['elevation_deg'] if clip_rec['known'] else None))
    rep['pop'] = pop_report(pd, info.melee_windows, lim, used)
    rep['exit_code'] = overall(rep['punches'], True, True)
    if not rep['punches']:
        rep['notes'].append('no punch found (no peak of the striking arm >= %.2f of its length in a melee window)' % lim.min_extension)
    rep['_draw'] = draw
    return rep


# --------------------------------------------------------------------------------------------------------------------- output
def table(rep):
    lines = []
    lines.append('body=%s  melee windows=%d  clips=%d  seam frames=%d' % (rep.get('body'), len(rep['windows']), rep.get('clips_loaded', 0),
                                                                       len(rep.get('seam_frames', []))))
    hdr = ' n  time          side  step                           clip(rms)              elev1  elev0  clipEl  diff  azim1 wr>head hipsZ knl lean  headP  full blend tilt        verdict'
    lines.append(hdr)
    for p in rep['punches']:
        g1, g0, c = p['game']['stage1'], p['game']['stage0'], p['clip']
        t = p['tilt']
        alts = c.get('alternates') or []
        lines.append('%2d %s %-5s %-30s %-22s %+6.1f %+6.1f %7s %5s %+6.1f %+6.0f %5.0f %3s %+5.0f %6s %4s %5s %-11s %s' % (
            p['n'], p['time'], p['side'], (p['step'] or '-')[:30], ('%s(%.1f)' % (c['name'], c['rms_deg']) if c.get('name') else '-')[:22],
            g1['elevation_deg'], g0['elevation_deg'], ('%+.1f' % c['pose']['elevation_deg']) if c['known'] else 'n/a',
            ('%+.1f' % p['elevation_diff_deg']) if 'elevation_diff_deg' in p else 'n/a', g1['azimuth_deg'], g1['wrist_above_head_cm'],
            g1['hips_above_root_cm'], 'YES' if p['kneeling'] else 'no', g1['trunk_lean_deg'], ('%+.0f' % g1['head_pitch_deg']) if g1['head_pitch_deg'] is not None else 'n/a',
            'on' if p['full_body_on'] else 'off', 'yes' if p['in_blend'] else 'no',
            ('%s/%.0f' % (t['source'], t['pitch_deg'])) if t else 'n/a', p['verdict']))
        if alts:
            lines.append('     same arm data (clip alternates): ' + ', '.join(alts))
        for f in p['fails']:
            lines.append('     FAIL: ' + f)
        for w in p['warnings']:
            lines.append('     warn: ' + w)
        for u in p['unevaluated']:
            lines.append('     not evaluated: ' + u)
    pop = rep.get('pop')
    if pop and pop['largest_rotation_jump']:
        r, h = pop['largest_rotation_jump'], pop['largest_hand_position_jump']
        lines.append('pop: largest rotation jump %.1f deg (%s at %s), largest hand position jump %.1f cm (%s at %s)' % (
            r['max_rotation_jump_deg'], r['bone'], r['rotation_jump_time'], h['max_position_jump_cm'], h['bone'], h['position_jump_time']))
    for n in rep['notes']:
        lines.append('note: ' + n)
    lines.append('exit code %d (%s)' % (rep['exit_code'], {0: 'pass', 1: 'fail', 2: 'could not evaluate'}[rep['exit_code']]))
    return '\n'.join(lines)


def draw_sheet(rep, path):
    from PIL import Image, ImageDraw
    items = rep.get('_draw') or []
    if not items:
        return False
    cols = min(4, len(items))
    rows = (len(items) + cols - 1) // cols
    pw, ph_ = 380, 430
    img = Image.new('RGB', (cols * pw, rows * ph_), (24, 24, 28))
    dr = ImageDraw.Draw(img)
    y_lo, y_hi, z_hi = -60.0, 110.0, 195.0
    sc = min((pw - 20) / (y_hi - y_lo), (ph_ - 70) / z_hi)

    def pt(ox, oy, p):
        return (ox + 10 + (p[1] - y_lo) * sc, oy + 40 + (z_hi - p[2]) * sc)

    def skeleton(ox, oy, pose, col, width):
        for a, b in BONES_DRAWN:
            if a in pose and b in pose:
                dr.line([pt(ox, oy, pose[a][0]), pt(ox, oy, pose[b][0])], fill=col, width=width)

    for i, it in enumerate(items):
        ox, oy = (i % cols) * pw, (i // cols) * ph_
        dr.rectangle([ox, oy, ox + pw - 1, oy + ph_ - 1], outline=(70, 70, 80))
        gy = pt(ox, oy, (0, 0, 0))[1]
        dr.line([ox + 5, gy, ox + pw - 5, gy], fill=(80, 80, 90))
        skeleton(ox, oy, it['game0'], (90, 90, 100), 1)
        if it['clip']:
            skeleton(ox, oy, it['clip'], (70, 200, 110), 2)
        skeleton(ox, oy, it['game1'], (235, 90, 80) if it['verdict'] == 'FAIL' else (235, 220, 120), 2)
        s = it['side']
        a, h = it['game1'][s + 'Arm'][0], it['game1'][s + 'Hand'][0]
        dr.line([pt(ox, oy, a), pt(ox, oy, h)], fill=(255, 255, 255), width=1)
        txt = '#%d %s %s %s  elev %+.1f' % (it['n'], it['time'], s, it['verdict'], it['elev'])
        dr.text((ox + 8, oy + 6), txt, fill=(240, 240, 240))
        dr.text((ox + 8, oy + 20), ('clip: %s, elev %+.1f' % (it['clip_name'], it['clip_elev'])) if it['clip'] else 'clip: unknown',
                fill=(120, 220, 140) if it['clip'] else (200, 160, 120))
        dr.text((ox + 8, oy + ph_ - 28), 'side view, forward = right', fill=(150, 150, 160))
        dr.text((ox + 8, oy + ph_ - 16), 'yellow/red: after write, grey: before, green: clip', fill=(150, 150, 160))
    pathlib.Path(path).parent.mkdir(parents=True, exist_ok=True)
    img.save(path)
    return True


def load_steps(run_dir):
    p = os.path.join(run_dir, 'gametest', 'summary.json')
    if not os.path.exists(p):
        return []
    try:
        with open(p, encoding='utf-8') as f:
            s = json.load(f)
        out = []
        for st in s.get('steps', []):
            a, b = st['t0'].split(' '), st['t1'].split(' ')
            out.append(dict(scenario=st['scenario'], step=st['step'], t0=stamp_epoch(a[0], a[1]), t1=stamp_epoch(b[0], b[1])))
        return out
    except (OSError, ValueError, KeyError):
        return []


def find_log(run_dir):
    logs = sorted(glob.glob(os.path.join(run_dir, 'logs', 'utpp-2026-*.log')) + glob.glob(os.path.join(run_dir, 'logs', 'utpp-20*.log')))
    logs = sorted(set(logs), key=lambda p: -os.path.getsize(p))
    return logs[0] if logs else None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('run_dir', nargs='?', help='harness run folder')
    ap.add_argument('--log', help='UTPP session log (utpp-*.log)')
    ap.add_argument('--pose', nargs='+', help='.utpppose file(s) (default: from the log line "pose capture: started file=")')
    ap.add_argument('--game-dir', default=DEFAULT_GAME_DIR, help='game folder the logged pose path is relative to')
    ap.add_argument('--clips-dir', help='folder of the built fist clips *.npz (default: Build\\work\\melee-npc-20261109 woman / man)')
    ap.add_argument('--body', choices=('woman', 'man'), help='override the body read from the log')
    ap.add_argument('--out', help='write the report as JSON')
    ap.add_argument('--sheet', help='write the side-view figure (PNG); no default - the project keeps PNGs under Media')
    d = Limits()
    ap.add_argument('--max-elev-diff', type=float, default=d.max_elev_diff, help='allowed |elevation - clip elevation| (deg)')
    ap.add_argument('--kneel-hips-cm', type=float, default=d.kneel_hips_cm, help='hips below this height above the root = kneeling')
    ap.add_argument('--max-wrist-above-head-cm', type=float, default=d.max_wrist_above_head_cm, help='straight punches: wrist above the Head bone')
    ap.add_argument('--min-extension', type=float, default=d.min_extension, help='fraction of the arm length that counts as a punch')
    ap.add_argument('--min-gap', type=float, default=d.min_gap_s, help='minimum seconds between two punches')
    ap.add_argument('--skip-after-mask', type=float, default=d.skip_after_mask_s, help='seconds after a melee mask start ignored')
    ap.add_argument('--blend', type=float, default=d.blend_s, help='seconds after the mask start that count as blending')
    ap.add_argument('--max-clip-rms', type=float, default=d.max_clip_rms, help='worst clip match (deg rms) still accepted')
    a = ap.parse_args(argv)
    log_path, steps = a.log, []
    if a.run_dir:
        if not os.path.isdir(a.run_dir):
            raise SystemExit('run folder not found: %s' % a.run_dir)
        log_path = log_path or find_log(a.run_dir)
        steps = load_steps(a.run_dir)
    if not log_path or not os.path.exists(log_path):
        raise SystemExit('no UTPP log: give RUN_DIR (with logs\\utpp-*.log) or --log')
    lim = Limits(max_elev_diff=a.max_elev_diff, kneel_hips_cm=a.kneel_hips_cm, max_wrist_above_head_cm=a.max_wrist_above_head_cm,
                 min_extension=a.min_extension, min_gap_s=a.min_gap, skip_after_mask_s=a.skip_after_mask, blend_s=a.blend,
                 max_clip_rms=a.max_clip_rms)
    rep = analyze(log_path, a.pose, a.game_dir, a.clips_dir, a.body, lim, steps)
    print(table(rep))
    draw = rep.pop('_draw', None)
    if a.out:
        pathlib.Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        with open(a.out, 'w', encoding='utf-8', newline='\n') as f:
            json.dump(rep, f, indent=1)
            f.write('\n')
        print('wrote', a.out)
    if a.sheet:
        rep['_draw'] = draw
        print('wrote', a.sheet) if draw_sheet(rep, a.sheet) else print('no punch: no sheet written')
    return rep['exit_code']


if __name__ == '__main__':
    sys.exit(main())
