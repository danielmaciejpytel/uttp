"""Door clearance of a car entry clip over the REAL door timeline: V's body must not pass through the front door she enters by - skin, window glass,
sill - while it is closed, swinging open, open AND while it swings shut again.

  python -I check_door_clearance.py --clip <utpp_car_p?a_<class>_<seat>.anims.glb | vanilla p?a_.. | NPC ?a_..passenger_r> --class <class>
         [--seat driver|passenger_r] [--clip-name enter] [--door-data <dir>] [--tol-cm 1.0] [--open-tol-cm 2.0] [--close-after S | --close-vanilla]
         [--window S] [--never-open] [--open-late S] [--expect-fail] [--report out.json] [--solve --fragment out.json] [--mirror]
  python -I check_door_clearance.py --table <glob> [<glob> ...] [--close-vanilla | --close-from-timing [--close-own]] [--waive CLASS:G:SEAT=CM ...]   table of many clips
        (--close-from-timing = the SHIPPED close of the seat: vanilla + UTPPCarDoorCloseDeltaS, max over genders; LATE = close later than clip end + 0.5 s fails)
  python -I check_door_clearance.py --selftest --clip ... --class ... [--seat ...] [--close-after S]        mutations (see below)

Timeline (door_timing.json, ONE source of the timing; measured in the game test 2026-11-01 22:23):
  E  = the door-open event = the instant the door starts to move = mount line + open_start_s = clip time open_start_s - clip_start_after_mount_s
       (the clip's frame 0 plays 10..40 ms after the mount line; --window (default 0.02 s) scans E +- that, the close follows E).
  closed before E; opening over open_dur_s (the vehicle clip door_front_* 'startup' phase mapped linearly onto open_dur_s); open; the close starts
  at E + close_after_s and takes close_dur_s (the 'recover' phase of the same clip, linearly mapped); closed afterwards. close_after_s is
  vanilla (vanilla_close_start_s - open_start_s, ~1.51 s swing / 1.80 s Turbo-R) unless --close-after S (vanilla + the seat's UTPPCarDoorCloseDeltaS,
  which the redscript adds to the event's autoCloseTime) is given. Past the last clip frame V holds the last frame (the seat idle starts from it).
  Classes/doors without a measured ride are flagged in the output (derived_from_left / UNVERIFIED).
  V LAG (door_timing.json v_lag_s, --v-lag): V's body is evaluated that many seconds later than the clip clock - an UNVERIFIED safety allowance (the
  offset between the mount line and V's first motion is not measured to this precision; with 0 the closing sweep at the measured vanilla close clears
  most installed clips although the review saw the door close on V). It applies to every phase and to --solve.
  SEATED BASELINE (PER CAPSULE): the final held pose is the game's seat idle (check m); each capsule's own penetration of the closed door (capsule radii
  by hand) in that last frame is a pose property of THAT capsule, so the closing sweep / closed door afterwards are gated per capsule at
  max(1 cm, that capsule's seated value + 0.5 cm) - a thigh sitting 4 cm into the closed door does not let the arm or head pass. Same per capsule for the
  fully open door (open gate max(open tol, seated-vs-open value + 0.5 cm)). The table / reports list the worst closing / closed value of every capsule
  with its own gate (closing_closed_by_capsule_cm, phases.*.by_capsule); the headline 'capsule' of a phase is the one with the least headroom to its gate.
  A gate >= a capsule's radius cannot fail (penetration <= radius): reports mark such capsules 'ungated' (a 15 cm waiver = every capsule, 8.5 cm = all but the
  trunk capsules) - the build_car.ps1 doorWaive of sport1 drivers (15 cm) is therefore effectively OFF for the opening / open phases.
BODY SHELL + ROOF phases (2026-11-03; extract_door_assets.py --body + extract_door_data.py --body -> <door-data dir>\\..\\body-data\\<class>.json): the car BODY mesh
(ext01_body_01: roof, roof rails, A/B/C pillars, quarter panels, sills; the 'Base' bone frame -> vehicle frame), the static glass of the body entity (windshield, rear / quarter windows)
and the interior roof lining (int01_interior_01 above the seat) as surface point clouds. Static: evaluated once per clip (120 Hz over the clip, independent of the door timeline and of the V lag).
  phase 'body': every body capsule vs shell + static glass; gate per capsule = max(tol, the capsule's penetration in the SEATED pose (last frame of the UNmutated clip) + 0.5 cm).
  phase 'roof': V's head (sphere, centre ROOF_HEAD_UP_M above the Head joint (world up), radius ROOF_HEAD_R_M + ROOF_HAIR_M hair) vs the roof cloud (shell + roof lining above the seat + glass,
  points higher than the seat + ROOF_MIN_ABOVE_SEAT_M); gate max(tol, seated head penetration + 0.5 cm). Both are ABSOLUTE (not relative to the vanilla clip; the vanilla clips are run through the
  same phases for comparison: --body-ref only prints their numbers). Mutations: --shift-rear 0.3 (whole body 0.3 m to the rear) must fail 'body'; --head-raise 0.12 (head + roof sphere up, the last
  --head-raise-skip-end frames unraised) must fail 'roof'; --selftest checks exactly those verdicts. --no-body switches the phases off.
Door geometry (extract_door_data.py): every solid part of the door (skin, window glass, the Turbo-R sill) as a SURFACE point cloud in its bone
frame, moved with its bone of the vehicle clip (the recover frames are sampled here from the vehicle .anims.glb: the door-data json holds only 0..loop).
Frames: vehicle space = seat_bone * inverse(Root(last frame)) * clip world. Body: capsules (BODY) on spine / pelvis / thighs / shins / arms / hands /
neck / head, interpolated linearly between the 30 fps frames and evaluated at 120 Hz (the closing door moves up to 3 cm per step); penetration =
capsule radius - distance of the capsule axis to the door surface.
Gates per phase (penetration cm): closed (before E) 1, opening (until 90 % of the door displacement) 1, open 2 (the fully open door), CLOSING SWEEP 1,
closed after the close 1 (the seated pose against the closed door). Exit 0 = every phase within its gate over the whole E window.
--solve: earliest close start for which the whole closing sweep and the closed door afterwards clear V for this and every later start, + margin_s;
close_after = max(vanilla, that); writes the fragment (--fragment) merged by gen_door_timing.py into door_timing.json and the redscript table.
--selftest: door never opens / opens 1.2 s late / closes right after opening (open end + 0.05 s) must be rejected; closing at vanilla must be rejected
when --solve says it must be later; EDGE: close_after moved earlier in 0.05 s steps must fail within 1.5 s (non-vacuity).
"""
import argparse
import glob
import hashlib
import json
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import skel as S  # noqa: E402
import build_entry_clip as B  # noqa: E402
import gen_door_timing as G  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

FPS = 30.0
HZ = 120.0
DT = 1.0 / HZ
DATA = os.path.join(os.path.join(project_root(), r'Extracted\game\vehicle-entry-car'), 'door-data')
TIMING_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'door_timing.json')
SOLVE_STEP_S = 0.025  # = 3 steps of 1/120 s: the closing sweeps of different starts share their (time, door frame) evaluations

# (name, bone a, bone b, radius m). The head is a capsule from the Head bone up along Neck1 -> Head by 0.17 m.
BODY = [
    ('pelvis', 'Hips', 'Spine', 0.13), ('hips_l', 'LeftUpLeg', 'RightUpLeg', 0.11),
    ('spine', 'Spine', 'Spine2', 0.14), ('chest', 'Spine2', 'Spine3', 0.15), ('neck', 'Spine3', 'Head', 0.07),
    ('thigh_l', 'LeftUpLeg', 'LeftLeg', 0.085), ('thigh_r', 'RightUpLeg', 'RightLeg', 0.085),
    ('shin_l', 'LeftLeg', 'LeftFoot', 0.06), ('shin_r', 'RightLeg', 'RightFoot', 0.06),
    ('shoulder_l', 'LeftShoulder', 'LeftArm', 0.055), ('shoulder_r', 'RightShoulder', 'RightArm', 0.055),
    ('uarm_l', 'LeftArm', 'LeftForeArm', 0.05), ('uarm_r', 'RightArm', 'RightForeArm', 0.05),
    ('farm_l', 'LeftForeArm', 'LeftHand', 0.04), ('farm_r', 'RightForeArm', 'RightHand', 0.04),
    ('hand_l', 'LeftHand', 'EXT:LeftForeArm', 0.04), ('hand_r', 'RightHand', 'EXT:RightForeArm', 0.04),
    ('head', 'Head', 'HEAD_TOP', 0.095),
]
BODY_R = [b[3] for b in BODY]
PHASES = ('closed', 'opening', 'open', 'closing', 'closed_after', 'rear', 'body', 'jamb', 'roof')  # 'rear' = the static rear door / glass / B-pillar block (4-door classes), every time step


def load_timing(path=TIMING_PATH):
    return json.load(open(path, encoding='utf-8'))


def parse_tol(v):
    """A body waiver as given on the command line / in a verdict: None, a number (flat, EVERY capsule) or 'capsule=cm,capsule=cm' (only those capsules; round 4b)."""
    if v is None or isinstance(v, (int, float, dict)):
        return v
    v = str(v)
    if '=' in v:
        return {k.strip(): float(x) for k, x in (kv.split('=') for kv in v.split(','))}
    return float(v)


def parse_waive_body(v):
    """--waive-body value -> (body_tol, roof_tol): 'BODYCM,ROOFCM' (flat numbers, the old form) or 'capsule=cm,capsule=cm[/ROOFCM]' (per-capsule body waiver, roof after a slash; none = no roof waiver)."""
    if '=' in v:
        body, _, roof = v.partition('/')
        return parse_tol(body), (float(roof) if roof.strip() else None)
    b, r = v.split(',')
    return float(b), float(r)


def body_tol_vec(tol, names, base_cm):
    """Per-capsule waiver gate (cm): the absolute tolerance, raised to the waiver where one is given (flat number = every capsule, dict = the named capsules)."""
    g = np.full(len(names), float(base_cm))
    if tol is None:
        return g
    if isinstance(tol, dict):
        for k, x in tol.items():
            if k not in names:
                raise SystemExit('waiver names an unknown capsule: %s' % k)
            g[names.index(k)] = max(g[names.index(k)], float(x))
        return g
    return np.maximum(g, float(tol))


def side_timing(tj, cls, side):
    d = dict(tj['classes'][cls][side])
    d['vanilla_close_after_s'] = d['vanilla_close_start_s'] - d['open_start_s']
    d['event_clip_s'] = d['open_start_s'] - tj['clip_start_after_mount_s']  # E in clip time
    return d


def load_clip(path, clip_name='enter'):
    sk = S.Skeleton.from_glb(path)
    c = sk.clip(clip_name)
    dur, t = B.frames_of(sk, c)
    lt, lr = sk.sample_local(c, t)
    lr = S.qcontinuous(lr)
    Wt, Wr = sk.world(lt, lr)
    return sk, np.asarray(t, float), Wt, Wr


def body_points(sk, Wt, spacing=0.05):
    """(T, P, 3) sample points in clip world, per-point radius (P,), per-point capsule index (P,), capsule names."""
    pts, rad, cid, names = [], [], [], []
    for k, (nm, a, b, r) in enumerate(BODY):
        if b.startswith('EXT:'):  # hand: 0.09 m beyond the wrist along forearm -> hand (the NPC rigs have no HandEnd)
            pa = Wt[:, sk.index[a]]
            d = pa - Wt[:, sk.index[b[4:]]]
            pb = pa + 0.09 * d / np.maximum(np.linalg.norm(d, axis=-1, keepdims=True), 1e-9)
        elif b == 'HEAD_TOP':
            pa = Wt[:, sk.index[a]]
            d = Wt[:, sk.index['Head']] - Wt[:, sk.index['Neck1' if 'Neck1' in sk.index else 'Neck']]
            d = d / np.maximum(np.linalg.norm(d, axis=-1, keepdims=True), 1e-9)
            pb = pa + 0.17 * d
        else:
            pa, pb = Wt[:, sk.index[a]], Wt[:, sk.index[b]]
        L = float(np.linalg.norm(pb - pa, axis=-1).max())
        n = max(2, int(np.ceil(L / spacing)) + 1)
        for u in np.linspace(0, 1, n):
            pts.append(pa * (1 - u) + pb * u)
            rad.append(r)
            cid.append(k)
        names.append(nm)
    return np.stack(pts, axis=1), np.array(rad), np.array(cid), names


def part_pose(part, f):
    """Bone world (t, R) at fractional clip frame f."""
    ft, fq = part['_ft'], part['_fq']
    i0 = int(np.floor(f))
    i0 = min(max(i0, 0), len(ft) - 1)
    i1 = min(i0 + 1, len(ft) - 1)
    u = f - i0
    t = ft[i0] * (1 - u) + ft[i1] * u
    q0, q1 = fq[i0], fq[i1]
    if np.dot(q0, q1) < 0:
        q1 = -q1
    q = q0 * (1 - u) + q1 * u
    return t, S.q2m(q / np.linalg.norm(q))


def min_dist(q, pts, pp):
    """Distance of every row of q (N,3) to the nearest row of pts (M,3) (pp = |pts|^2)."""
    d2 = np.sum(q * q, axis=1)[:, None] + pp[None, :] - 2.0 * (q @ pts.T)
    return np.sqrt(np.maximum(d2.min(axis=1), 0.0))


def prepare(dj):
    """Points + (extended) bone frames: 0..recover_end of the vehicle clip are sampled here (the json holds 0..loop_end only)."""
    vg = dj['vehicle_glb']
    if not os.path.exists(vg):   # 2026-11-05: the extraction moved to Extracted/game/vehicle-entry-car; the door-data json still carries the old absolute path
        alt = vg.replace('\\Extracted\\vehicle-entry-car\\', '\\Extracted\\game\\vehicle-entry-car\\')
        if os.path.exists(alt):
            vg = alt
    sk = S.Skeleton.from_glb(vg)
    for side, sn in (('left', 'front_left'), ('right', 'front_right')):
        sd = dj['door'][side]
        ph = sd['phase_frames']
        sd['_rec'] = (int(ph['recover'][0]), int(ph['recover'][1]))
        sd['_su'] = int(ph['startup'][1])
        n = sd['_rec'][1] + 1
        t = np.arange(n) / FPS
        lt, lr = sk.sample_local(sk.clip('door_' + sn), t)
        Wt, Wr = sk.world(lt, lr)
        for p in sd['parts']:
            bi = sk.index[p['bone']]
            ft, fq = Wt[:, bi], S.m2q(Wr[:, bi])
            old = np.asarray(p['frames_t'])
            if np.abs(ft[:len(old)] - old).max() > 1e-4:  # same sampling as extract_door_data.py
                raise RuntimeError('door frames of %s %s differ from the door-data json (%g m)' % (dj['class'], p['bone'], np.abs(ft[:len(old)] - old).max()))
            p['_ft'], p['_fq'] = ft, fq
            p['_pts'] = np.asarray(p['points'])
            p['_pp'] = np.sum(p['_pts'] ** 2, axis=1)
            p['_c'] = 0.5 * (p['_pts'].min(0) + p['_pts'].max(0))
            p['_r'] = float(np.linalg.norm(p['_pts'] - p['_c'], axis=1).max())
        # recover end pose vs the closed pose (frame 0)
        p0 = sd['parts'][0]
        d_open = float(np.linalg.norm(p0['_ft'][sd['_su']] - p0['_ft'][0]))
        sd['_open_frac'] = np.clip(np.linalg.norm(p0['_ft'] - p0['_ft'][0], axis=1) / max(d_open, 1e-9), 0.0, 1.0)  # 0 closed .. 1 fully open (travel of the door bone), per clip frame
        sd['_close_gap_m'] = float(np.linalg.norm(p0['_ft'][-1] - p0['_ft'][0]))
    return dj


_CACHE = {}
_REAR = {}
_SILL = {}
SILL_LEG_CAPS = ('hips_l', 'thigh_l', 'thigh_r', 'shin_l', 'shin_r')  # capsules that may take the vanilla clip's own sill contact as an allowance (pen_caps_V docstring)


ROOF_HEAD_UP_M = 0.09   # head sphere centre above the Head joint (along neck -> head)
ROOF_HEAD_R_M = 0.10    # skull radius
ROOF_HAIR_M = 0.05      # hair allowance
BODY_BEHIND_EDGE_M = 0.0   # body gate region: y <= the front door's closed rear edge + this
BODY_USE_APERTURE = False  # True = the body phase region also reaches to the real aperture edge (absolute gate: the game's own utility clip then fails by 13 cm); the strip is its own phase 'jamb' instead
BODY_UPPER_ABOVE_SEAT_M = 0.55  # ... or higher than the seat bone + this (roof / rails / pillar tops)
ROOF_MIN_ABOVE_SEAT_M = 0.45  # roof cloud = body / glass / lining points higher than the seat bone + this
_BODY = {}
_JAMB_REF = {}


def load_body(data, cls):
    """Body-shell data of a class (extract_door_data.py --body: <door-data dir>\\..\\body-data\\<class>.json) or None. Points in the VEHICLE frame (x right, y forward, z up)."""
    k = (data, cls)
    if k not in _BODY:
        p = os.path.join(os.path.dirname(os.path.abspath(data)), 'body-data', cls + '.json')
        bd = None
        if os.path.exists(p):
            j = json.load(open(p, encoding='utf-8'))
            sh = np.asarray(j['parts']['shell']['points'], np.float32)
            gl = np.asarray(j['parts']['glass']['points'], np.float32).reshape(-1, 3)
            li = np.asarray(j['parts']['lining']['points'], np.float32).reshape(-1, 3)
            seat_z = min(v[2] for v in j['seat'].values())
            cloud = np.concatenate([sh, gl])
            label = np.concatenate([np.zeros(len(sh), np.int8), np.ones(len(gl), np.int8)])
            o = np.argsort(cloud[:, 0], kind='stable')
            cloud, label = np.ascontiguousarray(cloud[o]), label[o]
            roof = np.concatenate([sh, gl, li])
            roof = roof[roof[:, 2] >= seat_z + ROOF_MIN_ABOVE_SEAT_M]
            bd = {'cloud': cloud, 'label': label, 'xs': np.ascontiguousarray(cloud[:, 0]), 'roof': roof, 'seat_z': seat_z, 'json': j}
        _BODY[k] = bd
    return _BODY[k]


def _min_dist32(q, pts):
    """Distance of every row of q (N,3) to the nearest row of pts (M,3) + the index (float32 BLAS)."""
    q = q.astype(np.float32)
    d2 = np.sum(q * q, axis=1)[:, None] + np.sum(pts * pts, axis=1)[None, :] - 2.0 * (q @ pts.T)
    j = d2.argmin(axis=1)
    return np.sqrt(np.maximum(d2[np.arange(len(q)), j], 0.0)), j


def load_rear(data, cls):
    """Rear-gate data of a class (extract_door_data.py --rear: <door-data dir>\\..\\rear-data\\<class>.json) or None (classes without a rear door)."""
    k = (data, cls)
    if k not in _REAR:
        p = os.path.join(os.path.dirname(os.path.abspath(data)), 'rear-data', cls + '.json')
        rj = None
        if os.path.exists(p):
            rj = json.load(open(p, encoding='utf-8'))
            for sd in rj['side'].values():
                for part in sd['static_parts']:
                    part['_pts'] = np.asarray(part['points'])
                    part['_pp'] = np.sum(part['_pts'] ** 2, axis=1)
                    part['_c'] = 0.5 * (part['_pts'].min(0) + part['_pts'].max(0))
                    part['_r'] = float(np.linalg.norm(part['_pts'] - part['_c'], axis=1).max())
        _REAR[k] = rj  # assigned once, fully built (the joint pre-screen loads models from threads)
    return _REAR[k]


GLTF_TO_VEH = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])  # vehicle (x right, y forward, z up) = M @ glTF rig space


def load_door_data(data, cls):
    k = (data, cls)
    if k not in _CACHE:
        _CACHE[k] = prepare(json.load(open(os.path.join(data, cls + '.json'), encoding='utf-8')))
    return _CACHE[k]


def open_fraction_time(sd, frac=0.9):
    """Fraction (0..1) of the startup phase after which the door skin has covered `frac` of its startup displacement (rotation in rad + translation in m)."""
    p = sd['parts'][0]
    ft, fq = p['_ft'], p['_fq']
    su = sd['_su']
    R = [S.q2m(q) for q in fq[:su + 1]]
    disp = np.array([float(np.linalg.norm(ft[i] - ft[0])) + float(np.arccos(np.clip((np.trace(R[0].T @ R[i]) - 1) / 2, -1, 1))) for i in range(su + 1)])
    k = int(np.argmax(disp >= frac * disp[-1])) if disp[-1] > 1e-6 else 0
    return k / float(su)


class Model:
    """One clip against the door of its seat; evaluates the timeline at 120 Hz with a (time step, door frame) cache."""

    def __init__(self, clip, cls, seat, clip_name='enter', data=DATA, timing=None, mirror=False, lag=None, shift_rear=0.0, no_rear=False, rear_ref=None, rear_pillar=False, no_env=False, no_body=False, head_raise=0.0, head_raise_skip_end=18, head_shift_veh=None, shift_veh=None, bump_rear=None):
        self.tj = timing or load_timing()
        # the verdict (analyse) AND the solve are the worst case over this set of lags: door_timing.json v_lag_set_s plus the lag of the measured clip clock of the class
        # (gen_door_timing.required_lags: T + clip gap - clip_start_after_mount_s, T = the vanilla slide of the class incl. the measured spread; Turbo-R 0.82 s);
        # --v-lag X = just X. self.lag = the largest (V plays this many seconds later than the model clock; sets the clip-hold horizon i_clip and the solve scan end).
        self.lag_set = G.required_lags(self.tj, cls) if lag is None else [float(lag)]
        self.lag = max(self.lag_set)
        dj = load_door_data(data, cls)
        side = 'left' if seat == 'driver' else 'right'
        if mirror:
            side = 'right' if side == 'left' else 'left'
        self.cls, self.seat, self.side, self.clip = cls, seat, side, clip
        self.sd = dj['door'][side]
        self.tm = side_timing(self.tj, cls, side)
        # OPEN-POSE ENVELOPE (door_timing.json classes.<c>.pose_envelope, Turbo-R only): the open door of the game is not exactly the clip pose (probe 'settled' lines of
        # 2026-11-02: 1.05 s after the event the left door sits at the clip pose, the right door and the later left readings 0.1-0.2 m farther forward / 0.1 m higher). Every
        # entry = a vehicle-frame offset (x right, y forward, z up, m) that is added to the door parts scaled by their open fraction; the verdict is the worst over the nominal
        # pose and every entry. no_env = nominal pose only (searches).
        env = [] if no_env else self.tj.get('classes', {}).get(cls, {}).get('pose_envelope', [])
        self.env = [np.array(e['d_veh'], float) @ GLTF_TO_VEH for e in env]  # (row vector @ M) = M.T @ d: vehicle -> rig (glTF) space
        sk, t, Wt, Wr = load_clip(clip, clip_name)
        ri = sk.index['Root']
        Re, pe = Wr[-1, ri], Wt[-1, ri]
        st, sR = np.array(dj['seat'][side]['t']), np.array(dj['seat'][side]['R']).reshape(3, 3)
        P, self.rad, self.cid, self.names = body_points(sk, Wt)
        # BODY MARGIN (door_timing.json classes.<c>.body_margin_cm, Turbo-R): every capsule radius + this many cm - the radii are by hand (no skin mesh), the open door of the
        # game is a physical mesh that does not rest on the clip pose, and the visible body is the clip path plus the native root hold / offsets of the approach
        self.rad = self.rad + float(self.tj.get('classes', {}).get(cls, {}).get('body_margin_cm', 0.0)) / 100.0
        self.A = sR @ Re.T  # clip world -> the vehicle rig space of door-data (glTF Y-up)
        self.V = np.einsum('ij,tpj->tpi', self.A, P - pe) + st  # vehicle space (T, P, 3)
        self.V_seated = self.V[-1].copy()  # the UNmutated seated pose: baseline of the body / roof gates
        # joint tracks of the start-position / joint-rule gates (start_gate.py): rig space like V; the mutations below move them with the body
        self.J = {nm: np.einsum('ij,tj->ti', self.A, Wt[:, sk.index[nm]] - pe) + st for nm in ('Hips', 'LeftUpLeg', 'RightUpLeg', 'LeftLeg', 'RightLeg', 'Spine', 'Spine1', 'Spine2', 'Spine3') if nm in sk.index}
        self._mut_rig = np.zeros(3)  # whole-body mutation shift (rig space): also applied to the root path (root_g) below
        self._mut_t = np.zeros((len(self.V), 3))  # per-frame whole-body mutation shift (rig space): the mid-clip rearward bump
        if bump_rear is not None:  # mutation (round 4b): the WHOLE body (points, head sphere, joints, root) moves bump_rear[0] m to the rear (rig z+) in a smooth bump over the clip fraction bump_rear[1]..bump_rear[2] (sin^2)
            amp, f0, f1 = bump_rear
            tt = np.arange(len(self.V)) / max(1.0, len(self.V) - 1.0)
            w = np.where((tt >= f0) & (tt <= f1), np.sin(np.pi * (tt - f0) / max(1e-6, f1 - f0)) ** 2, 0.0)
            self._mut_t[:, 2] = float(amp) * w
            self.V = self.V + self._mut_t[:, None, :]
        # head sphere of the roof phase (rig space): centre ROOF_HEAD_UP_M above the Head joint (world up)
        hj = sk.index['Head']
        up = np.array([0.0, 1.0, 0.0])  # clip world up (glTF Y): the skull centre is ROOF_HEAD_UP_M ABOVE the Head joint (the head bone's own axis is not vertical in a bent pose)
        self.head_c = np.einsum('ij,tj->ti', self.A, Wt[:, hj] + ROOF_HEAD_UP_M * up - pe) + st  # (T, 3) rig space
        self.head_seated = self.head_c[-1].copy()
        if bump_rear is not None:
            self.head_c = self.head_c + self._mut_t
        if head_raise:  # mutation: head + head sphere this many m up (rig y+) except the last head_raise_skip_end frames (the seated pose is pinned by check (b))
            n = len(self.V) - int(head_raise_skip_end)
            hm = (self.cid == self.names.index('head'))
            self.V[:n][:, hm, 1] += float(head_raise)
            self.head_c[:n, 1] += float(head_raise)
        if head_shift_veh is not None:  # mutation (roofpack2): head + head sphere this many m along a VEHICLE-frame vector (x right, y forward, z up) except the last head_raise_skip_end frames
            n = len(self.V) - int(head_raise_skip_end)
            sh = np.asarray(head_shift_veh, float) @ GLTF_TO_VEH   # vehicle -> rig (glTF) space (row vector @ M = M.T @ v)
            hm = (self.cid == self.names.index('head'))
            self.V[:n][:, hm, :] += sh
            self.head_c[:n] += sh
        if shift_veh is not None:  # mutation (roofpack2): the whole clip body + head sphere this many m along a VEHICLE-frame vector (all frames)
            sh = np.asarray(shift_veh, float) @ GLTF_TO_VEH
            self.V = self.V + sh
            self.head_c = self.head_c + sh
            self._mut_rig = self._mut_rig + sh
        if shift_rear:  # mutation: the whole clip body this many metres to the REAR (rig z+) = a seat-frame z start-back that never goes away
            self.V = self.V + np.array([0.0, 0.0, float(shift_rear)])
            self.head_c = self.head_c + np.array([0.0, 0.0, float(shift_rear)])
            self._mut_rig = self._mut_rig + np.array([0.0, 0.0, float(shift_rear)])
        # approach mode (door_approach.py): the clip root path and the body-forward direction of the clip start in the same space
        self.root_g = np.einsum('ij,tj->ti', self.A, Wt[:, ri] - pe) + st + self._mut_rig + self._mut_t
        self.J = {k: v + self._mut_rig + self._mut_t for k, v in self.J.items()}
        fw = np.zeros(3)
        for fb, tb in (('LeftFoot', 'LeftToeBase'), ('RightFoot', 'RightToeBase')):
            if fb in sk.index and tb in sk.index:
                fw += Wt[0, sk.index[tb]] - Wt[0, sk.index[fb]]
        self.fwd0_g = self.A @ fw
        self.root_R = np.einsum('ij,tjk->tik', self.A, Wr[:, ri])  # root orientation in the same space, (T, 3, 3)
        self._starts = np.flatnonzero(np.r_[True, np.diff(self.cid) != 0])  # first point of every capsule (points are contiguous per capsule)
        self.cap_rad_cm = np.array([b[3] for b in BODY]) * 100.0
        self.t = np.asarray(t, float)
        self.clip_end = float(t[-1])
        self.i_clip = int(math.ceil((self.clip_end + self.lag) * HZ - 1e-9))
        rj = None if no_rear else load_rear(data, cls)
        self.bodyd = None if no_body else load_body(data, cls)
        self._static = None
        self._jamb = None
        self.jcloud = None
        self.body_tol_cm = None  # per-clip WAIVER of the body / roof gate in cm (flat, build_car.ps1 bodyWaive / --body-tol-cm / --roof-tol-cm); None = the absolute 1 cm (+ seated baseline)
        self.roof_tol_cm = None
        if self.bodyd is not None:
            # BODY GATE REGION: the shell / glass the entering body must not go through = everything BEHIND the front door's closed rear edge (B-pillar, quarter panel, rear
            # glass, sill behind the doorway: solid bodywork the door never covers) + the upper frame (points higher than the seat + BODY_UPPER_ABOVE_SEAT_M: roof, roof rail,
            # A / B pillar top, windshield header). The doorway itself (sill, rocker, floor, A-pillar base) is excluded: the legs legitimately cross the sill (the vanilla clips
            # penetrate it by 5-8 cm).
            ar = self.tj['arcs'][cls][side]
            y_edge = ar['hingeY'] + ar['length'] * math.sin(math.radians(ar['closedDirDeg']))
            bdd = self.bodyd
            ylim = np.full(len(bdd['cloud']), y_edge + BODY_BEHIND_EDGE_M)
            if BODY_USE_APERTURE:
                import start_gate as _SG
                zs_, ys_, _ = _SG.aperture_rear(bdd, side, y_edge)
                ylim = np.maximum(ylim, np.interp(bdd['cloud'][:, 2], zs_, ys_) + BODY_BEHIND_EDGE_M)
                self.aperture_edge_strip_m = float(np.max(ys_) - y_edge)
            msk = (bdd['cloud'][:, 1] <= ylim) | (bdd['cloud'][:, 2] >= bdd['seat_z'] + BODY_UPPER_ABOVE_SEAT_M)
            self.bcloud = {'cloud': bdd['cloud'][msk], 'label': bdd['label'][msk], 'xs': np.ascontiguousarray(bdd['cloud'][msk][:, 0]), 'y_edge': y_edge}
            # phase 'jamb' (round 4b): the B-pillar / quarter-panel jamb STRIP between the closed-door arc edge and the real aperture rear edge (shell hole by height, start_gate.aperture_rear), below seat + 0.55 m.
            # Capsules (with their radius) are gated against it relative to the game's own clip of the set, per capsule: max(1 cm, seated baseline + 0.5, vanilla clip's worst + 0.5) - the game's clips lean into
            # the jamb too (utility: 13 cm). No reference clip (--rear-ref) = the absolute gate.
            import start_gate as _SG
            zs_, ys_, _ = _SG.aperture_rear(bdd, side, y_edge)
            cl = bdd['cloud']
            lim_ = np.interp(cl[:, 2], zs_, ys_)
            jm = (cl[:, 1] > y_edge + BODY_BEHIND_EDGE_M) & (cl[:, 1] <= lim_) & (cl[:, 2] < bdd['seat_z'] + BODY_UPPER_ABOVE_SEAT_M)
            self.jcloud = {'cloud': cl[jm], 'label': bdd['label'][jm], 'xs': np.ascontiguousarray(cl[jm][:, 0]), 'y_edge': y_edge, 'n': int(jm.sum()), 'strip_m': float(np.max(ys_) - y_edge)}
        self.rear = None if rj is None else rj['side'][side]
        self.rear_pillar = rear_pillar  # the B-pillar block is SYNTHETIC (no body mesh): off unless asked for (--rear-pillar)
        self.rear_ref = rear_ref  # vanilla player clip of the same set: the rear gate is relative to what the game's own clip does (see rear_ref_caps)
        self._data, self._clip_name, self._no_env, self._no_body = data, clip_name, no_env, no_body
        self.su = self.sd['_su']
        self.rec0, self.rec1 = self.sd['_rec']
        self.s90_frac = open_fraction_time(self.sd)
        self._body = {}
        self._penc = {}
        self._rearc = {}

    def rad_by_cap_cm(self):
        """Capsule radius (cm) per capsule name order of self.names, incl. the class body margin."""
        return [float(self.rad[st_]) * 100.0 for st_ in self._starts]

    # ---- body shell / roof (static: once per clip, 120 Hz over the clip timeline, lag independent)
    def _body_pen(self, Vr, bc=None):
        """Per-capsule penetration (m, floored at -0.15) of body points Vr (P, 3, rig space) vs the shell + static glass; (pen (nc,), cloud label per capsule).
        The cloud is sorted by x: every capsule only looks at the points inside its own bounding box (+ radius + 0.1 m)."""
        bd = self.bodyd
        Vv = (Vr @ GLTF_TO_VEH.T).astype(np.float32)
        bc = self.bcloud if bc is None else bc
        c, lab, xs = bc['cloud'], bc['label'], bc['xs']
        nc = len(self.names)
        pen = np.full(nc, -0.15)
        who = np.zeros(nc, np.int8)
        for ci, st_ in enumerate(self._starts):
            e = self._starts[ci + 1] if ci + 1 < len(self._starts) else len(Vv)
            q = Vv[st_:e]
            m = float(self.rad[st_]) + 0.1
            lo, hi = q.min(0) - m, q.max(0) + m
            i0, i1 = np.searchsorted(xs, lo[0]), np.searchsorted(xs, hi[0])
            if i1 <= i0:
                continue
            sub = c[i0:i1]
            k = np.flatnonzero((sub[:, 1] >= lo[1]) & (sub[:, 1] <= hi[1]) & (sub[:, 2] >= lo[2]) & (sub[:, 2] <= hi[2]))
            if len(k) == 0:
                continue
            d, j = _min_dist32(q, sub[k])
            pr = float(self.rad[st_]) - d
            jj = int(np.argmax(pr))
            pen[ci] = max(float(pr[jj]), -0.15)
            who[ci] = lab[i0 + k[j[jj]]]
        return pen, who

    def _roof_pen(self, hc):
        """Penetration (m) of the head sphere (centre hc, rig space) into the roof cloud (>= -0.15)."""
        rc = self.bodyd['roof']
        if len(rc) == 0:
            return -0.15
        v = (np.asarray(hc) @ GLTF_TO_VEH.T).astype(np.float32)[None]
        d, _ = _min_dist32(v, rc)
        return float(max(ROOF_HEAD_R_M + ROOF_HAIR_M - d[0], -0.15))

    def static_series(self):
        """(body pen (steps, nc), roof pen (steps,), body who (steps, nc), seated body pen (nc,), seated roof pen) - cached."""
        if self._static is not None:
            return self._static
        nc = len(self.names)
        n = int(math.ceil(self.clip_end * HZ)) + 1
        bp = np.full((n + 1, nc), -0.15)
        bw = np.zeros((n + 1, nc), np.int8)
        rp = np.full(n + 1, -0.15)
        hi = self.names.index('head')
        for i in range(n + 1):
            x = min(i * DT * FPS, len(self.t) - 1.0)
            i0 = int(np.floor(x))
            i1 = min(i0 + 1, len(self.t) - 1)
            u = x - i0
            bp[i], bw[i] = self._body_pen(self.V[i0] * (1 - u) + self.V[i1] * u)
            rp[i] = self._roof_pen(self.head_c[i0] * (1 - u) + self.head_c[i1] * u)
        sb, _ = self._body_pen(self.V_seated)
        sr = self._roof_pen(self.head_seated)
        self._static = (bp, rp, bw, np.maximum(sb, 0.0), max(sr, 0.0), hi)
        return self._static

    def jamb_series(self):
        """(jamb pen (steps, nc) of the strip cloud, seated pen (nc,)) - cached; None without a strip cloud."""
        if self.jcloud is None or self.jcloud['n'] == 0:
            return None
        if self._jamb is not None:
            return self._jamb
        nc = len(self.names)
        n = int(math.ceil(self.clip_end * HZ)) + 1
        jp = np.full((n + 1, nc), -0.15)
        for i in range(n + 1):
            x = min(i * DT * FPS, len(self.t) - 1.0)
            i0 = int(np.floor(x))
            i1 = min(i0 + 1, len(self.t) - 1)
            u = x - i0
            jp[i] = self._body_pen(self.V[i0] * (1 - u) + self.V[i1] * u, self.jcloud)[0]
        sj = np.maximum(self._body_pen(self.V_seated, self.jcloud)[0], 0.0)
        self._jamb = (jp, sj)
        return self._jamb

    def jamb_ref_caps(self):
        """Worst jamb penetration (m, >= 0) per capsule of the game's own clip of the set (rear_ref); None without one."""
        if self.rear_ref is None or self.jcloud is None:
            return None
        if getattr(self, '_jamb_ref', None) is None:
            key = (self.rear_ref, self.cls, self.seat)
            if key not in _JAMB_REF:
                r = Model(self.rear_ref, self.cls, self.seat, self._clip_name, self._data, self.tj, no_rear=True, no_env=True)
                js = r.jamb_series()
                _JAMB_REF[key] = None if js is None else np.maximum(js[0].max(axis=0), 0.0)
            self._jamb_ref = _JAMB_REF[key]
        return self._jamb_ref

    def roof_series(self):
        """(roof pen (steps,), seated roof pen) only - the head sphere vs the roof cloud, same sampling as static_series (the body part of it is the expensive one; the roof mutations of the
        self-test need nothing else). Cached."""
        if getattr(self, '_roof_only', None) is None:
            n = int(math.ceil(self.clip_end * HZ)) + 1
            rp = np.full(n + 1, -0.15)
            for i in range(n + 1):
                x = min(i * DT * FPS, len(self.t) - 1.0)
                i0 = int(np.floor(x))
                i1 = min(i0 + 1, len(self.t) - 1)
                u = x - i0
                rp[i] = self._roof_pen(self.head_c[i0] * (1 - u) + self.head_c[i1] * u)
            self._roof_only = (rp, max(self._roof_pen(self.head_seated), 0.0))
        return self._roof_only

    def static_phase(self, ph):
        """[pen (nc,), t (nc,), who list (nc,)] of the 'body' / 'roof' phase, or None without body data."""
        if self.bodyd is None:
            return None
        nc = len(self.names)
        if ph == 'jamb':
            js = self.jamb_series()
            if js is None:
                return None
            pv = js[0].max(axis=0)
            tv = js[0].argmax(axis=0) * DT
            return [pv, tv, ['jamb'] * nc]
        if ph == 'roof':
            rp, _sr = self.roof_series()
            pv = np.full(nc, -0.15)
            tv = np.zeros(nc)
            hi = self.names.index('head')
            pv[hi], tv[hi] = rp.max(), rp.argmax() * DT
            return [pv, tv, ['roof'] * nc]
        bp, rp, bw, sb, sr, hi = self.static_series()
        if ph == 'body':
            pv = bp.max(axis=0)
            tv = bp.argmax(axis=0) * DT
            who = [('glass' if bw[bp[:, c].argmax(), c] == 1 else 'shell') for c in range(nc)]
            return [pv, tv, who]
        pv = np.full(nc, -0.15)
        tv = np.zeros(nc)
        pv[hi], tv[hi] = rp.max(), rp.argmax() * DT
        return [pv, tv, ['roof'] * nc]

    def at_lag(self, lag):
        """The same clip / door at another V lag (data shared, caches fresh)."""
        import copy
        if self.bodyd is not None:
            self.static_series()  # lag independent: computed once, shared by the copies
        m = copy.copy(self)
        m.lag = float(lag)
        m.lag_set = [m.lag]
        m.i_clip = int(math.ceil((self.clip_end + m.lag) * HZ - 1e-9))
        m._body, m._penc, m._rearc = {}, {}, {}
        for a in ('_seat_pen_c', '_seat_pen_open_c', '_seat_pen_rear_c', '_sill_allow'):  # the baseline is the last held frame (same at every lag); dropped anyway
            if hasattr(m, a):
                delattr(m, a)
        return m

    # ---- body / door state
    def body(self, i):
        b = self._body.get(i)
        if b is None:
            x = min(max((i * DT - self.lag) * FPS, 0.0), len(self.t) - 1.0)
            i0 = int(np.floor(x))
            i1 = min(i0 + 1, len(self.t) - 1)
            u = x - i0
            b = self.V[i0] * (1 - u) + self.V[i1] * u
            self._body[i] = b
        return b

    def pen_caps(self, i, f):
        """PER CAPSULE penetration (m; (n_caps,) array, < 0 = clear, floored at -0.15) over the capsule's points and all door parts at 120 Hz step i / door
        frame f, plus the door bone that gives each capsule's value."""
        key = (i, round(f, 3))
        r = self._penc.get(key)
        if r is not None:
            return r
        r = self.pen_caps_V(self.body(i), f)
        self._penc[key] = r
        return r

    def pen_caps_V(self, V, f):
        """pen_caps for an explicit body point array V (P, 3) in the door-data rig space (no cache): the approach mode moves the clip body.
        SILL ALLOWANCE (Turbo-R, --rear-ref / rear_ref = the vanilla player clip of the set): the door floor / sill piece (door ent component door_floor_f*, bone door_front_*_c; it moves
        0.11 m down with the door) is a plate the foot has to cross; the game's own first-person clip puts the shin / hip through it (5.7 / 9.9 cm). It is a DOOR part: this allowance is
        the one remaining known door contact against 'zero clipping'. Per capsule the penetration of that ONE part counts only
        where it exceeds the vanilla clip's own worst sill penetration of the same capsule + 0.5 cm - for the LEG capsules only (SILL_LEG_CAPS: hips_l, thighs, shins; the trunk, arms, neck and head get none) - and only while the door opens / is open (frame <= startup end);
        the door skin and the window glass are always absolute."""
        best = np.full(len(self.names), -0.15)
        bones = [''] * len(self.names)
        allow = self.sill_allow() if f <= self.su + 1e-9 else None
        fr = self.sd['_open_frac']
        k0 = min(max(int(np.floor(f)), 0), len(fr) - 1)
        k1 = min(k0 + 1, len(fr) - 1)
        sf = float(fr[k0] * (1 - (f - k0)) + fr[k1] * (f - k0)) if f - k0 < 1 else float(fr[k1])
        offs = [None] + ([d * sf for d in self.env] if sf > 1e-3 else [])
        for off in offs:
            for part in self.sd['parts']:
                if getattr(self, '_only_sill', False) and not part['bone'].endswith('_c'):
                    continue
                pt, pR = part_pose(part, f)
                if off is not None:
                    if part['bone'].endswith('_c'):
                        continue  # the sill / floor piece (door_front_*_c) travels only 0.11 m down with the door: the envelope offsets the swinging skin and glass, not this plate
                    pt = pt + off
                loc = (V - pt) @ pR  # pR^T (p - pt)
                if (np.linalg.norm(loc - part['_c'], axis=1) - part['_r'] - self.rad).min() > 0.15:
                    continue
                pc = np.maximum.reduceat(self.rad - min_dist(loc, part['_pts'], part['_pp']), self._starts)  # capsule points are contiguous
                if allow is not None and part['bone'].endswith('_c'):
                    pc = np.maximum(pc - allow, -0.15)
                for c in np.flatnonzero(pc > best):
                    bones[c] = part['bone'] + ('' if off is None else '+env')
                best = np.maximum(best, pc)
        return best, bones

    def sill_allow(self):
        """(n_caps,) m: the vanilla clip's own worst penetration + 5 mm of every capsule against the sill part(s) while the door opens / is open (0 without a reference / without
        a sill part). Computed once per (reference clip, lag set member, envelope) on the same timeline as the verdict (E window, vanilla close)."""
        a = getattr(self, '_sill_allow', None)
        if a is not None:
            return a
        sill = [p for p in self.sd['parts'] if p['bone'].endswith('_c')]
        if self.rear_ref is None or not sill or getattr(self, '_is_ref', False):
            self._sill_allow = np.zeros(len(self.names))
            return self._sill_allow
        key = (os.path.abspath(self.rear_ref), self.cls, self.side, round(self.lag, 3), self._no_env, self._data)
        a = _SILL.get(key)
        if a is None:
            r = Model(self.rear_ref, self.cls, self.seat, self._clip_name, self._data, self.tj, lag=self.lag, no_env=self._no_env, no_rear=True)
            r._is_ref = True
            r._sill_allow = np.zeros(len(self.names))
            r._only_sill = True  # pen_caps_V of this reference model looks at the sill piece(s) only (the shared door data is not touched)
            tm = r.tm
            w = np.zeros(len(self.names))
            for off in [0.0] if r.tj['window_s'] <= 0 else [float(x) for x in np.round(np.arange(-r.tj['window_s'], r.tj['window_s'] + 1e-9, 0.01), 4)]:
                E = tm['event_clip_s'] + off
                tc = E + 100.0  # the reference door never closes: the allowance is the vanilla clip's own pose-vs-open-sill contact over its whole length (incl. the seat settle)
                for i in range(0, r.i_clip + 3):
                    ph, f = r.frame_at(i * DT, E, tc, False)
                    if f > r.su + 1e-9:
                        continue
                    w = np.maximum(w, r.pen_caps(i, f)[0])
            leg = np.array([n in SILL_LEG_CAPS for n in self.names])  # only the leg chain has to cross the sill; trunk, arms, neck, head get NO allowance
            a = _SILL[key] = np.where(leg, np.maximum(w, 0.0) + 0.005, 0.0)
        self._sill_allow = a
        return a

    # ---- rear gate: static rear door skin / glass (surface clouds, 1 cm) + the rear door block and the B-pillar block as SOLIDS (a limb that has fully passed through
    # the thin skin or the glass is inside the block: its penetration is radius + depth, a surface-distance check would call it clear)
    def rear_pen_caps_V(self, V):
        """PER CAPSULE penetration (m) of the body points V (P, 3, rig space) against the rear parts: ((n_caps,) array floored at -0.15, part name per capsule)."""
        best = np.full(len(self.names), -0.15)
        who = [''] * len(self.names)
        if self.rear is None:
            return best, who
        for part in self.rear['static_parts']:
            if (np.linalg.norm(V - part['_c'], axis=1) - part['_r'] - self.rad).min() > 0.15:
                continue
            pc = np.maximum.reduceat(self.rad - min_dist(V, part['_pts'], part['_pp']), self._starts)
            for c in np.flatnonzero(pc > best):
                who[c] = part['kind']
            best = np.maximum(best, pc)
        Vv = V @ GLTF_TO_VEH.T
        for bx in self.rear['solid_boxes']:
            if bx['name'].startswith('B-pillar') and not self.rear_pillar:
                continue
            lo = np.array([bx['xmin'], bx['ymin'], bx['zmin']])
            hi = np.array([bx['xmax'], bx['ymax'], bx['zmax']])
            q = np.abs(Vv - 0.5 * (lo + hi)) - 0.5 * (hi - lo)
            sd = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)  # signed distance to the box, < 0 inside
            pc = np.maximum.reduceat(self.rad - sd, self._starts)
            for c in np.flatnonzero(pc > best):
                who[c] = 'solid:' + bx['name'].split(' (')[0]
            best = np.maximum(best, pc)
        return best, who

    def rear_pen_caps(self, i):
        r = self._rearc.get(i)
        if r is None:
            r = self._rearc[i] = self.rear_pen_caps_V(self.body(i))
        return r

    def rear_ref_caps(self):
        """PER CAPSULE worst penetration (m, >= 0) of the VANILLA player clip of the same set against the rear parts (None without --rear-ref): the game's own first-person
        clip rests hands and forearms on the door frame / pillar (3-4 cm into the 3 cm surface cloud of the rear door), so an absolute 1 cm gate fails the vanilla clips
        themselves. The rear gate of a UTPP clip is max(tol, seated baseline + 0.5 cm, vanilla worst of the same capsule + 0.5 cm): it must not go deeper than the game's
        own clip, capsule by capsule. Absolute numbers are reported next to the gates."""
        if self.rear_ref is None or self.rear is None:
            return None
        if not hasattr(self, '_rear_ref_c'):
            r = Model(self.rear_ref, self.cls, self.seat, self._clip_name, self._data, self.tj, lag=self.lag, no_env=True)
            w = np.zeros(len(self.names))
            for i in range(r.i_clip + 3):
                w = np.maximum(w, r.rear_pen_caps(i)[0])
            self._rear_ref_c = w
        return self._rear_ref_c

    def seat_pen_rear_caps(self):
        """Seated (held, last frame) penetration of each capsule against the rear parts (>= 0): the baseline of the 'rear' gate like the front-door ones."""
        if not hasattr(self, '_seat_pen_rear_c'):
            self._seat_pen_rear_c = np.maximum(self.rear_pen_caps(self.i_clip + 2)[0], 0.0)
        return self._seat_pen_rear_c

    def pen(self, i, f):
        """(penetration m (max over capsules; < 0 = clear, floored at -0.15), capsule name, door bone) - the worst capsule only (see pen_caps)."""
        b, bones = self.pen_caps(i, f)
        j = int(np.argmax(b))
        return float(b[j]), self.names[j], bones[j]

    def frame_at(self, tt, E, tc, never_open):
        """(phase, door frame) at clip time tt for the event E and the close start tc."""
        tm = self.tm
        od, cd = tm['open_dur_s'], tm['close_dur_s']
        s = tt - E
        if never_open or s < 0:
            return 'closed', 0.0
        if s < od:
            return ('opening' if s < self.s90_frac * od else 'open'), self.su * s / od
        if tt < tc:
            return 'open', float(self.su)
        if tt < tc + cd:
            return 'closing', self.rec0 + (self.rec1 - self.rec0) * (tt - tc) / cd
        return 'closed_after', float(self.rec1)

    def timeline(self, E, close_after, never_open=False, stop_at=None, i_from=0, worst=None):
        """Per-CAPSULE max penetration per phase over the clip time line for event E and a close E + close_after_s.
        Returns {phase: [pen (n_caps,) m, t (n_caps,) s, door bone per capsule]}."""
        tm = self.tm
        tc = E + max(close_after, tm['open_dur_s'])
        i_end = max(self.i_clip, int(math.ceil((tc + tm['close_dur_s']) * HZ)) + 2)
        res = worst if worst is not None else {}
        nc = len(self.names)
        for i in range(i_from, i_end + 1):
            ph, f = self.frame_at(i * DT, E, tc, never_open)
            pv, bones = self.pen_caps(i, f)
            cur = res.get(ph)
            if cur is None:
                cur = res[ph] = [np.full(nc, -1.0), np.zeros(nc), [''] * nc]
            for c in np.flatnonzero(pv > cur[0]):
                cur[1][c], cur[2][c] = i * DT, bones[c]
            cur[0] = np.maximum(cur[0], pv)
            if self.rear is not None:
                pr, who = self.rear_pen_caps(i)
                cr = res.get('rear')
                if cr is None:
                    cr = res['rear'] = [np.full(nc, -1.0), np.zeros(nc), [''] * nc]
                for c in np.flatnonzero(pr > cr[0]):
                    cr[1][c], cr[2][c] = i * DT, who[c]
                cr[0] = np.maximum(cr[0], pr)
        return res

    def seat_pen_caps(self):
        """PER CAPSULE penetration (m, >= 0) of the FINAL (held, seated) pose against the closed door: a property of the seat idle pose the clip must end
        on (check (m) pins it to frame 0 of the class tpp_idle / idle), not of the entry motion; the capsule radii are by hand. The closing sweep and the
        closed door afterwards are gated PER CAPSULE at max(tol, this capsule's value + 5 mm): the entry must not make that capsule's contact worse, and a
        capsule that touches in the seated pose gets no waiver for any other capsule (an arm / head sweep is gated at tol however deep the thigh sits)."""
        if not hasattr(self, '_seat_pen_c'):
            self._seat_pen_c = np.maximum(self.pen_caps(self.i_clip + 2, float(self.rec1))[0], 0.0)
        return self._seat_pen_c

    def seat_pen_open_caps(self):
        """Same per capsule for the FULLY OPEN door (held seated pose vs the open door): the open-phase gate is max(gate, this capsule + 0.5 cm)."""
        if not hasattr(self, '_seat_pen_open_c'):
            self._seat_pen_open_c = np.maximum(self.pen_caps(self.i_clip + 2, float(self.su))[0], 0.0)
        return self._seat_pen_open_c

    def seat_pen(self):
        """Worst capsule of seat_pen_caps (m): informational only - NOT a gate (the gates are per capsule)."""
        return float(self.seat_pen_caps().max())

    def seat_pen_open(self):
        """Worst capsule of seat_pen_open_caps (m): informational only."""
        return float(self.seat_pen_open_caps().max())

    def gate_cm(self, ph, tol_cm=1.0, open_tol_cm=2.0, opening_tol_cm=None):
        """THE gate of a phase, per capsule (cm; (n_caps,) array) - shared by the verdict (_analyse1) and the solver (sweep_ok).
        closed / opening: scalar (tol / opening_tol); open: max(open_tol, seated-vs-open-door of the capsule + 0.5); closing / closed_after:
        max(tol, seated-vs-closed-door of the capsule + 0.5)."""
        nc = len(self.names)
        if ph == 'closed':
            return np.full(nc, float(tol_cm))
        if ph == 'opening':
            return np.full(nc, float(tol_cm if opening_tol_cm is None else opening_tol_cm))
        if ph == 'rear':
            g = np.maximum(float(tol_cm), self.seat_pen_rear_caps() * 100 + 0.5)
            rc = self.rear_ref_caps()
            return g if rc is None else np.maximum(g, rc * 100 + 0.5)
        if ph == 'body':
            return np.maximum(body_tol_vec(self.body_tol_cm, self.names, tol_cm), self.static_series()[3] * 100 + 0.5)
        if ph == 'jamb':
            g = np.full(nc, float(tol_cm))
            js = self.jamb_series()
            if js is not None:
                g = np.maximum(g, js[1] * 100 + 0.5)
                ref = self.jamb_ref_caps()
                if ref is not None:
                    g = np.maximum(g, ref * 100 + 0.5)
            return g
        if ph == 'roof':
            g = np.maximum(float(self.roof_tol_cm if self.roof_tol_cm is not None else tol_cm), self.roof_series()[1] * 100 + 0.5)
            return np.full(nc, g)
        if ph == 'open':
            return np.maximum(float(open_tol_cm), self.seat_pen_open_caps() * 100 + 0.5)
        return np.maximum(float(tol_cm), self.seat_pen_caps() * 100 + 0.5)

    def sweep_ok(self, tc, tol):
        """Closing sweep + closed door afterwards (per-capsule gates) for a close start at clip time tc (no event offsets: the sweep does not depend on E)."""
        gate = self.gate_cm('closing', tol * 100) / 100.0
        cd = self.tm['close_dur_s']
        i_end = max(self.i_clip, int(math.ceil((tc + cd) * HZ)) + 2)
        i0 = max(0, int(math.floor(tc * HZ)) - 1)
        for i in range(i0, i_end + 1):
            ph, f = self.frame_at(i * DT, -1e9, tc, False)
            if ph not in ('closing', 'closed_after'):
                continue
            if (self.pen_caps(i, f)[0] > gate).any():
                return False
        return True


def _analyse1(model, close_after=None, window=None, never_open=False, open_late=0.0, tol_cm=1.0, open_tol_cm=2.0, opening_tol_cm=None):
    """Verdict over the E window. close_after None = vanilla. Every phase is judged per capsule against model.gate_cm."""
    tj, tm = model.tj, model.tm
    window = tj['window_s'] if window is None else window
    ca = tm['vanilla_close_after_s'] if close_after is None else close_after
    offs = [0.0] if window <= 0 else [float(x) for x in np.round(np.arange(-window, window + 1e-9, 0.01), 4)]
    nc = len(model.names)
    worst, nominal = {}, {}  # phase -> [pen (nc,), t (nc,), bone list, event offset (nc,)]
    for off in offs:
        E = tm['event_clip_s'] + off + open_late
        r = model.timeline(E, ca, never_open)
        for ph, (pv, tv, bv) in r.items():
            w = worst.get(ph)
            if w is None:
                w = worst[ph] = [np.full(nc, -1.0), np.zeros(nc), [''] * nc, np.zeros(nc)]
            for c in np.flatnonzero(pv > w[0]):
                w[1][c], w[2][c], w[3][c] = tv[c], bv[c], off
            w[0] = np.maximum(w[0], pv)
            if abs(off) < 1e-9:
                nominal[ph] = pv
    for ph_ in ('body', 'jamb', 'roof'):
        r_ = model.static_phase(ph_)
        if r_ is not None:
            worst[ph_] = [r_[0], r_[1], r_[2], np.zeros(nc)]
    sp_c, spo_c = model.seat_pen_caps(), model.seat_pen_open_caps()
    out = {'clip': model.clip, 'class': model.cls, 'seat': model.seat, 'door_side': model.side, 'timing_status': tm['status'],
           'event_clip_s': tm['event_clip_s'], 'close_after_s': ca, 'vanilla_close_after_s': tm['vanilla_close_after_s'],
           'close_start_clip_s': tm['event_clip_s'] + ca, 'open_dur_s': tm['open_dur_s'], 'close_dur_s': tm['close_dur_s'],
           'seated_pose_pen_cm': round(float(sp_c.max()) * 100, 2), 'seated_pose_pen_open_cm': round(float(spo_c.max()) * 100, 2),
           'seated_pose_pen_by_capsule_cm': {n: round(float(v) * 100, 2) for n, v in zip(model.names, sp_c) if v > 0},
           'seated_pose_pen_open_by_capsule_cm': {n: round(float(v) * 100, 2) for n, v in zip(model.names, spo_c) if v > 0},
           'v_lag_s': model.lag, 'window_s': window, 'never_open': never_open, 'open_late_s': open_late, 'clip_end_s': model.clip_end, 'phases': {}}
    ok = True
    for ph in PHASES:
        if ph not in worst:
            continue
        pv, tv, bv, ov = worst[ph]
        gate = model.gate_cm(ph, tol_cm, open_tol_cm, opening_tol_cm)
        pcm = pv * 100
        by = {}
        for c, n in enumerate(model.names):
            by[n] = {'max_pen_cm': round(max(0.0, float(pcm[c])), 2), 'clearance_cm': round(-float(pcm[c]), 2), 'gate_cm': round(float(gate[c]), 2),
                     'headroom_cm': round(float(gate[c] - pcm[c]), 2), 'ok': bool(pcm[c] <= gate[c]), 't_clip_s': round(float(tv[c]), 3),
                     'door_bone': bv[c], 'event_offset_s': float(ov[c]),
                     'nominal_max_pen_cm': round(max(0.0, float(nominal[ph][c]) * 100), 2) if ph in nominal else None}
        good = bool((pcm <= gate).all())
        ok &= good
        b = int(np.argmin(gate - pcm))  # the binding capsule: least headroom to its own gate
        a = int(np.argmax(pcm))  # the deepest capsule, whatever its gate
        # capsules a gate cannot fail: penetration <= radius, so a gate >= radius (cm) is no gate at all (a 15 cm waiver covers EVERY capsule)
        ungated = [n for c, n in enumerate(model.names) if gate[c] >= model.cap_rad_cm[c] - 1e-9]
        out['phases'][ph] = {'max_pen_cm': round(max(0.0, float(pcm[b])), 2), 'clearance_cm': round(-float(pcm[b]), 2), 'gate_cm': round(float(gate[b]), 2),
                             'headroom_cm': round(float(gate[b] - pcm[b]), 2), 'ok': good,
                             't_clip_s': by[model.names[b]]['t_clip_s'], 'capsule': model.names[b], 'door_bone': bv[b], 'event_offset_s': float(ov[b]),
                             'nominal_max_pen_cm': by[model.names[b]]['nominal_max_pen_cm'],
                             'deepest_capsule': model.names[a], 'deepest_pen_cm': round(max(0.0, float(pcm[a])), 2),
                             'ungated_capsules': ungated, 'by_capsule': by}
    out['door_clearance_ok'] = bool(ok)
    _summ(out)
    return out


def _summ(out):
    """Summary fields from the phases: deepest penetration (any capsule, any phase but open), open-door contact, closing/closed per-capsule worst."""
    ph = out['phases']
    out['max_pen_window_cm'] = max([ph[p]['deepest_pen_cm'] for p in ph if p != 'open'] + [0.0])
    out['open_door_contact_window_cm'] = ph.get('open', {}).get('deepest_pen_cm', 0.0)
    cl = [ph[p] for p in ('closing', 'closed_after') if p in ph]
    out['closing_closed_by_capsule_cm'] = {n: {'max_pen_cm': max(x['by_capsule'][n]['max_pen_cm'] for x in cl),
                                               'gate_cm': min(x['by_capsule'][n]['gate_cm'] for x in cl)} for n in cl[0]['by_capsule']} if cl else {}
    return out


def analyse(model, close_after=None, window=None, never_open=False, open_late=0.0, tol_cm=1.0, open_tol_cm=2.0, opening_tol_cm=None):
    """Verdict = worst case over the V lag set of the model (door_timing.json v_lag_set_s): every phase and every capsule gated at its worst lag."""
    lags = model.lag_set
    if len(lags) <= 1:
        r = _analyse1(model, close_after, window, never_open, open_late, tol_cm, open_tol_cm, opening_tol_cm)
        r['lags_s'] = [model.lag]
        return r
    rs = [_analyse1(model if abs(l - model.lag) < 1e-9 else model.at_lag(l), close_after, window, never_open, open_late, tol_cm, open_tol_cm, opening_tol_cm)
          for l in lags]
    out = dict(rs[-1])
    out['phases'] = {}
    for ph in PHASES:
        c = [(r['phases'][ph], l) for r, l in zip(rs, lags) if ph in r['phases']]
        if not c:
            continue
        by = {}
        for n in c[0][0]['by_capsule']:  # per capsule: the lag with the least headroom
            e, l = min(((x['by_capsule'][n], ll) for x, ll in c), key=lambda t: t[0]['headroom_cm'])
            by[n] = dict(e, worst_lag_s=l)
            by[n]['ok'] = all(x['by_capsule'][n]['ok'] for x, _ in c)
        bn = min(by, key=lambda n: by[n]['headroom_cm'])
        dn = max(by, key=lambda n: by[n]['max_pen_cm'])
        e = by[bn]
        v = {'max_pen_cm': e['max_pen_cm'], 'clearance_cm': e['clearance_cm'], 'gate_cm': e['gate_cm'], 'headroom_cm': e['headroom_cm'],
             'ok': all(x[0]['ok'] for x in c), 't_clip_s': e['t_clip_s'], 'capsule': bn, 'door_bone': e['door_bone'], 'event_offset_s': e['event_offset_s'],
             'nominal_max_pen_cm': e['nominal_max_pen_cm'], 'deepest_capsule': dn, 'deepest_pen_cm': by[dn]['max_pen_cm'],
             'ungated_capsules': c[0][0]['ungated_capsules'], 'by_capsule': by, 'worst_lag_s': e['worst_lag_s'],
             'by_lag_clearance_cm': {str(ll): x['clearance_cm'] for x, ll in c}, 'by_lag_headroom_cm': {str(ll): x['headroom_cm'] for x, ll in c}}
        out['phases'][ph] = v
    out['lags_s'] = list(lags)
    out['door_clearance_ok'] = bool(all(r['door_clearance_ok'] for r in rs))
    _summ(out)
    return out


def solve(model, margin=None, tol_cm=1.0):
    """Earliest close start (clip time) such that this and every later start clears; close_after = max(vanilla, that + margin) after E."""
    tj, tm = model.tj, model.tm
    margin = tj['margin_s'] if margin is None else margin
    tol = tol_cm / 100.0
    E = tm['event_clip_s']
    tc_van = E + tm['vanilla_close_after_s']
    lo = max(E + tm['open_dur_s'] + 0.05, 0.0)
    k0 = int(math.floor(lo / SOLVE_STEP_S))
    k_hi = int(math.ceil((model.clip_end + model.lag + 0.1) / SOLVE_STEP_S))
    grid = [k * SOLVE_STEP_S for k in range(k0, k_hi + 1)]
    ok = {}

    lag_models = [model] + [model.at_lag(l) for l in model.lag_set if abs(l - model.lag) > 1e-9]  # the closing sweep must clear V at EVERY lag of the set

    def passes(tc):
        key = round(tc, 6)
        if key not in ok:
            ok[key] = all(m.sweep_ok(tc, tol) for m in lag_models)
        return ok[key]
    res = {'class': model.cls, 'seat': model.seat, 'door_side': model.side, 'timing_status': tm['status'], 'event_clip_s': E,
           'vanilla_close_after_s': tm['vanilla_close_after_s'], 'margin_s': margin, 'clip_end_s': model.clip_end, 'step_s': SOLVE_STEP_S,
           'tol_cm': tol_cm, 'lags_s': [round(l, 3) for l in model.lag_set], 'window_s': tj['window_s']}
    if not passes(grid[-1]):
        res.update(feasible=False, earliest_close_clip_s=None, close_after_s=None)
        return res
    star = grid[-1]
    for tc in reversed(grid[:-1]):
        if passes(tc):
            star = tc
        else:
            break
    res['feasible'] = True
    res['earliest_close_clip_s'] = star
    res['earliest_clears_from_scan_floor'] = bool(star <= grid[0] + 1e-9)
    res['earliest_close_after_s'] = star - E
    res['close_after_s'] = round(max(tm['vanilla_close_after_s'], star - E + margin), 3)
    res['vanilla_close_clears'] = bool(passes(tc_van) and all(passes(g) for g in grid if g >= tc_van - 1e-9)) if tc_van <= grid[-1] else True
    res['close_later_than_vanilla'] = bool(res['close_after_s'] > tm['vanilla_close_after_s'] + 1e-6)
    return res


def parse_name(path):
    b = os.path.basename(path)
    m = re.match(r'^(?:utpp_car_)?(?:p)?([wm])a_([a-z0-9]+)_(driver|passenger_r)\.anims\.glb$', b)
    if not m:
        raise ValueError('cannot read class/seat from ' + b)
    return m.group(1), m.group(2), m.group(3)


def fmt_phases(r):
    """One token per phase: the clearance of the binding capsule (least headroom to its own gate); '!' = over its gate; 'U' = EVERY capsule ungated in
    that phase (the gate >= the capsule radius: a waiver that big cannot fail), 'u<n>' = n capsules ungated."""
    def un(v):
        n = len(v.get('ungated_capsules', []))
        return '' if n == 0 else ('U' if n == len(v['by_capsule']) else 'u%d' % n)
    return ' '.join('%s %.1f%s%s' % (p[:7], v['clearance_cm'], '' if v['ok'] else '!', un(v)) for p, v in r['phases'].items())


def fmt_caps(r):
    """Per-capsule worst of the closing sweep / closed door afterwards: 'capsule pen/gate cm' for every capsule that touches (pen > 0) or has a baseline gate."""
    items = [(n, v['max_pen_cm'], v['gate_cm']) for n, v in r.get('closing_closed_by_capsule_cm', {}).items() if v['max_pen_cm'] > 0 or v['gate_cm'] > 1.0 + 1e-9]
    return ', '.join('%s %.2f/%.2f%s' % (n, p, g, '!' if p > g else '') for n, p, g in items) or 'none touching'


def selftest(a, model):
    ok = True
    ca = a.close_after
    base = analyse(model, ca, a.window, tol_cm=a.tol_cm, open_tol_cm=a.open_tol_cm, opening_tol_cm=a.opening_tol_cm)
    print('selftest base: close_after %.3f s -> %s (%s)' % (base['close_after_s'], 'PASS' if base['door_clearance_ok'] else 'FAIL', fmt_phases(base)))
    od = model.tm['open_dur_s']
    for name, kw in (('door never opens', dict(never_open=True)), ('door opens 1.2 s late', dict(open_late=1.2)),
                     ('closes at open end + 0.05 s', dict(close_after=od + 0.05))):
        r = analyse(model, kw.pop('close_after', ca), a.window, tol_cm=a.tol_cm, open_tol_cm=a.open_tol_cm, opening_tol_cm=a.opening_tol_cm, **kw)
        bad = r['door_clearance_ok']
        worst = max((v['max_pen_cm'], p) for p, v in r['phases'].items())
        print('selftest %-28s max pen %.1f cm in %s (%s)' % (name, worst[0], worst[1], 'NOT rejected' if bad else 'rejected'))
        ok &= not bad
    sv = solve(model, tol_cm=a.tol_cm)
    if sv.get('feasible') and sv['earliest_close_after_s'] > model.tm['vanilla_close_after_s'] + 0.01:
        r = analyse(model, None, a.window, tol_cm=a.tol_cm, open_tol_cm=a.open_tol_cm, opening_tol_cm=a.opening_tol_cm)
        print('selftest vanilla close (earliest safe %.3f s > vanilla %.3f s): %s' % (sv['earliest_close_after_s'], model.tm['vanilla_close_after_s'],
                                                                              'NOT rejected' if r['door_clearance_ok'] else 'rejected'))
        ok &= not r['door_clearance_ok']
    elif sv.get('feasible'):
        print('selftest vanilla close: earliest safe %.3f s <= vanilla %.3f s (the vanilla close already clears; margin only) - skipped' % (
            sv['earliest_close_after_s'], model.tm['vanilla_close_after_s']))
    # EDGE: earlier closes in 0.05 s steps must fail within 1.5 s
    first_fail = None
    for k in range(1, 31):
        c = max(ca - 0.05 * k, od)  # the earliest possible close is the end of the opening (clamped, and still evaluated)
        r = analyse(model, c, a.window, tol_cm=a.tol_cm, open_tol_cm=a.open_tol_cm, opening_tol_cm=a.opening_tol_cm)
        if not r['door_clearance_ok']:
            first_fail = ca - c
            break
        if c <= od + 1e-9:
            break  # reached the floor without a failure: vacuous
    if first_fail is None:
        print('selftest edge: close_after can be 1.5 s earlier and the clip still passes - gate vacuous')
        ok = False
    else:
        print('selftest edge: close_after %.3f s passes; %.2f s earlier it fails (rejected)' % (ca, first_fail))
    ok &= selftest_static(model, a)
    ref = rear_ref_for(argparse.Namespace(rear_ref=a.rear_ref or 'auto', door_data=a.door_data), model.clip, model.cls, model.seat) if model.bodyd is not None else None
    if ref is not None:  # start position / joint rule / waiver audit mutations (start_gate.py): every one must be rejected
        import start_gate as SG
        ok_sg, lines = SG.selftest(model.clip, model.cls, model.seat, ref, model.body_tol_cm, model.roof_tol_cm)
        print(os.linesep.join(lines))
        ok &= ok_sg
    return ok


def static_verdict(m, ph, tol_cm=1.0):
    """(ok, worst capsule name, pen cm, gate cm, capsules that CANNOT fail) of a static phase ('body' / 'roof') against the gate the clip SHIPS with (the waiver, if any: m.body_tol_cm /
    m.roof_tol_cm). A flat gate >= a capsule's radius cannot fail that capsule (penetration <= radius): those capsules are listed (the roof sphere is 15 cm)."""
    r = m.static_phase(ph)
    g = m.gate_cm(ph, tol_cm)
    pc = r[0] * 100
    j = int(np.argmax(pc - g))
    rad = np.full(len(m.names), (ROOF_HEAD_R_M + ROOF_HAIR_M) * 100) if ph == 'roof' else m.rad * 100
    idx = [m.names.index('head')] if ph == 'roof' else range(len(m.names))
    cannot = [m.names[c] for c in idx if g[c] >= rad[c] - 1e-9]
    return bool((pc <= g).all()), m.names[j], float(pc[j]), float(g[j]), cannot


def selftest_static(model, a):
    """Mutations judged by the NEW phases' own verdicts against the gate the clip ships with (waiver included; the shifted body also fails the door / rear phases, the exit code proves nothing):
    the whole body 0.3 m to the rear must fail 'body'; the head 0.12 m up (last 18 frames unraised) must fail 'roof'. A phase whose waiver is >= every radius cannot reject anything: it is
    reported as VACUOUS (and counts as a failed self-test unless the unmutated clip is within the ABSOLUTE gate)."""
    if model.bodyd is None:
        print('selftest body/roof: no body-data for %s - skipped' % model.cls)
        return True
    ok = True
    kw0 = dict(no_rear=True, no_env=True, rear_ref=model.rear_ref)
    # roof: the head sphere may meet the ceiling from ABOVE (a low roof: 0.12 m up must fail it) or from the SIDE (utility cab: the nearest roof point is the roof edge beside the head with 65 cm of
    # free air above - the raise is vacuous there). The roof phase is non-vacuous when the head 0.12 m up OR 0.12 m inboard / outboard / forward / back (the first mutation that fails it) is rejected.
    # body: the whole body 0.3 m to the rear must fail it; when the (documented) waiver is large (sport1: the head sits 11 cm inside the lining model, waiver 11.5) the rear shift may stay under it -
    # then the whole body 0.3 m UP (into the roof / pillar frame, the chest capsule radius is 15 cm) is the second mutation
    cases = [('whole body to the rear (0.10 .. 0.30 m)', 'body', [(' 0.10 m', dict(shift_rear=0.10)), (' 0.15 m', dict(shift_rear=0.15)), (' 0.20 m', dict(shift_rear=0.20)), (' 0.30 m', dict(shift_rear=0.30)), (' / up 0.3 m', dict(shift_veh=(0, 0, 0.3))),
                                                                                   # round 7 (r7util): a clip that starts at the game's start (utility drivers / passengers) stays more than 3 cm clear of the quarter panel under every <= 0.30 m rear
                                                                                   # shift; the same gate must reject a LARGER rear displacement (escalation, reported with its size, like the roof case below)
                                                                                   (' / rear 0.45 m', dict(shift_rear=0.45)), (' / rear 0.60 m', dict(shift_rear=0.60))]),
             ('whole body to the rear (0.10 .. 0.30 m), jamb strip', 'jamb', [(' 0.10 m', dict(shift_rear=0.10)), (' 0.15 m', dict(shift_rear=0.15)), (' 0.20 m', dict(shift_rear=0.20)), (' 0.30 m', dict(shift_rear=0.30)), (' 0.45 m', dict(shift_rear=0.45))]),
             ('head 0.12 m up', 'roof', [('', dict(head_raise=0.12))] + [(n_, dict(head_shift_veh=v_)) for n_, v_ in
                                                                              ((' / sideways +x', (0.12, 0, 0)), (' / sideways -x', (-0.12, 0, 0)), (' / forward', (0, 0.12, 0)), (' / back', (0, -0.12, 0)))]
                                                                              # round 4 (startfwd): a clip that starts at the game's start never has its head within 0.12 m of the roof before the last 18 frames (the seated pose is pinned) - the
                                                                              # 0.12 m mutations are then geometrically unable to touch the roof. The same gate must still reject a LARGER displacement (escalation, reported with its size):
                                                                              + [(' / up 0.30 m', dict(head_raise=0.30)), (' / up 0.45 m', dict(head_raise=0.45)), (' / sideways +x 0.30 m', dict(head_shift_veh=(0.30, 0, 0))),
                                                                                 (' / sideways -x 0.30 m', dict(head_shift_veh=(-0.30, 0, 0))), (' / sideways +x 0.45 m', dict(head_shift_veh=(0.45, 0, 0))), (' / sideways -x 0.45 m', dict(head_shift_veh=(-0.45, 0, 0)))])]
    for name, ph, kws in cases:
        if ph == 'jamb' and model.jamb_series() is None:
            # r13d: a class without a jamb strip (standard25: the aperture rear edge coincides with the closed-door edge, no strip points) has NO jamb phase: static_phase returns None,
            # the clip verdict has no jamb record either, so there is no gate to prove non-vacuous (before: TypeError 'NoneType' not subscriptable in static_verdict; the standard25 rows
            # shipped from runs that pre-date this mutation case). Every class WITH a strip still runs the case below.
            print('selftest %-28s phase %-4s: class %s has no jamb strip points - the phase does not exist for it, nothing to reject (skipped)' % (name, ph, model.cls))
            continue
        m0 = Model(model.clip, model.cls, model.seat, model._clip_name, model._data, model.tj, **kw0)
        m0.body_tol_cm, m0.roof_tol_cm = model.body_tol_cm, model.roof_tol_cm
        v0 = static_verdict(m0, ph, a.tol_cm)
        waived = (model.body_tol_cm if ph == 'body' else model.roof_tol_cm) is not None
        rejected = False
        if not v0[0]:
            # r10: the unmutated clip already fails this phase - a mutation can only 'fail' it too, so the phase is NOT shown to be non-vacuous (it counts as a failed self-test)
            print('selftest %-28s phase %-4s: UNMUTATED clip already FAILS (%s %.1f cm vs shipped gate %.1f) - mutation test not meaningful, NOT counted as rejected' % (name, ph, v0[1], v0[2], v0[3]))
            ok = False
            continue
        for suffix, kw in kws:
            mm = Model(model.clip, model.cls, model.seat, model._clip_name, model._data, model.tj, **kw0, **kw)
            mm.body_tol_cm, mm.roof_tol_cm = model.body_tol_cm, model.roof_tol_cm
            v = static_verdict(mm, ph, a.tol_cm)
            if not v[0]:
                rejected = True
                break
        print('selftest %-28s phase %-4s: mutated %s (worst %s %.1f cm, shipped gate %.1f)%s; unmutated %s (%s %.1f cm)%s' % (
            name, ph, 'REJECTED' if rejected else 'NOT rejected', v[1], v[2], v[3], suffix if rejected else ' (all of up / sideways / fore-aft tried)', 'ok' if v0[0] else 'FAILS', v0[1], v0[2],
            ('; WAIVED row, capsules that cannot fail under this waiver: %s' % ','.join(v[4])) if waived and v[4] else ''))
        ok &= rejected
    return ok


def start_gates(m, a, clip, cls, seat, quiet=False):
    """Gates (a) start position, (b) joint rule, (c) waiver audit of start_gate.py for a UTPP clip (None for a game / NPC clip: no game clip to compare with)."""
    import start_gate as SG
    ref = rear_ref_for(argparse.Namespace(rear_ref=a.rear_ref or 'auto', door_data=a.door_data), clip, cls, seat)
    if ref is None:
        return None
    vm = Model(ref, cls, seat, a.clip_name, a.door_data, m.tj, no_rear=True, no_env=True)
    g = parse_name(clip)[0]
    r = SG.evaluate_model(m, vm, 'p%sa_%s_%s' % (g, cls, seat), m.body_tol_cm, m.roof_tol_cm, a.open_tol_cm if a.open_tol_cm != 2.0 else None, a.opening_tol_cm, SG.file_sha(clip))
    if not quiet:
        print(SG.fmt(r))
    return r


def rear_ref_for(a, clip, cls, seat):
    """--rear-ref: an explicit vanilla clip, or 'auto' = the vanilla player clip p?a_<class>_<seat> of the same gender in <door-data>\\..\\glb (None for a clip that IS vanilla)."""
    if not a.rear_ref:
        return None
    if a.rear_ref != 'auto':
        return a.rear_ref
    g = parse_name(clip)[0]
    p = os.path.join(os.path.dirname(os.path.abspath(a.door_data)), 'glb', 'p%sa_%s_%s.anims.glb' % (g, cls, seat))
    return p if os.path.exists(p) and os.path.basename(clip).startswith('utpp_car_') else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clip')
    ap.add_argument('--class', dest='cls')
    ap.add_argument('--seat')
    ap.add_argument('--clip-name', default='enter')
    ap.add_argument('--door-data', default=DATA)
    ap.add_argument('--timing', default=TIMING_PATH)
    ap.add_argument('--tol-cm', type=float, default=1.0)
    ap.add_argument('--open-tol-cm', type=float, default=2.0)
    ap.add_argument('--opening-tol-cm', type=float, default=None, help='gate of the opening phase (default --tol-cm); only for a documented waiver (Turbo-R lifting door model limits)')
    ap.add_argument('--close-after', type=float, help='s after the door-open event at which the door close starts (default: vanilla, or the value in door_timing.json with --close-from-timing)')
    ap.add_argument('--close-vanilla', action='store_true', help='force the vanilla close start (default)')
    ap.add_argument('--close-from-timing', action='store_true', help='the SHIPPED close of the seat: vanilla_close_after_s + UTPPCarDoorCloseDeltaS (max over genders of door_timing.json clips.<class>.<seat>.<gender>.close_after_s, '
                    'gen_door_timing.seat_delta); a close later than the clip end + 0.5 s counts as LATE = failure')
    ap.add_argument('--close-own', action='store_true', help='with --close-from-timing: this clip OWN gender close_after_s instead of the shipped seat value')
    ap.add_argument('--waive', action='append', default=[], metavar='CLASS:G:SEAT=CM',
                    help='table mode: opening / open tolerance CM for the clips of that class / gender (w|m|*) / seat (like build_car.ps1 doorWaive); repeatable')
    ap.add_argument('--window', type=float, default=None, help='event time window +- s (default door_timing.json window_s)')
    ap.add_argument('--v-lag', type=float, default=None, help='V plays this many s later than the model clock (sensitivity)')
    ap.add_argument('--never-open', action='store_true')
    ap.add_argument('--open-late', type=float, default=0.0)
    ap.add_argument('--mirror', action='store_true', help='check against the other front door (mutation)')
    ap.add_argument('--expect-fail', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--solve', action='store_true')
    ap.add_argument('--fragment', help='with --solve: write the per-clip result here (merged by gen_door_timing.py)')
    ap.add_argument('--table', nargs='+')
    ap.add_argument('--report')
    ap.add_argument('--approach', action='store_true', help='approach mode (door_approach.py): V from the mount spot - vanilla slide, native root hold, door from the mount line')
    ap.add_argument('--calibrate', action='store_true', help='with --approach: the 30 measured rides with the shipped female clips (--sources)')
    ap.add_argument('--sources', default=os.path.join(project_root(), r'Build\work\vehicle-entry-car\pk-door4-anims\sources.txt'), help='--calibrate: <clip>  <run dir> lines of the shipped set')
    ap.add_argument('--hold-cap', choices=['on', 'off', 'both'], default='both', help='approach: the planned door-aware cap of the native root hold')
    ap.add_argument('--align', choices=['T', 'door', 'both'], default='both', help='approach: clip clock from mount + T (measured) / from the door start / worst of both')
    ap.add_argument('--open-delay', type=float, default=None, help='approach: ONE door open delay (s after the mount line) instead of the swept measured range +-0.03 s')
    ap.add_argument('--nominal-delay', action='store_true', help='approach: the class open delay of door_timing.json only (no +-0.03 s sweep)')
    ap.add_argument('--fps-set', default=None, help='approach: native sample rates swept, default 40,50,60,90')
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--approach-baseline', default=None, help='approach: a baseline clip (the shipped one) under the SAME settings; a hold / entity / rear failure counts only where it is worse than the baseline')
    ap.add_argument('--no-grid', action='store_true')
    ap.add_argument('--arrival', action='store_true', help='approach (r13): add the ARRIVAL ring (starts 0.06 / 0.12 / 0.15 m around the clip start spot = where the native approach walk ends) to the fixture rides')
    ap.add_argument('--grid-r', type=float, default=0.1)
    ap.add_argument('--grid-deg', type=float, default=5.0)
    ap.add_argument('--only', default=None, help='approach: comma list of start names')
    ap.add_argument('--shift-rear', type=float, default=0.0, help='mutation: move the whole clip body this many m to the rear (rig z+), must fail the rear gate')
    ap.add_argument('--body-tol-cm', type=parse_tol, default=None, help='WAIVER of the body shell phase: flat gate in cm instead of the absolute tolerance (a documented exception only)')
    ap.add_argument('--roof-tol-cm', type=float, default=None, help='WAIVER of the roof phase (head sphere vs ceiling): flat gate in cm (a documented exception only)')
    ap.add_argument('--waive-body', action='append', default=[], metavar='CLASS:G:SEAT=BODYCM,ROOFCM', help="table mode: body / roof waivers per clip like --waive; per-capsule form CLASS:G:SEAT=capsule=cm,capsule=cm[/ROOFCM]")
    ap.add_argument('--head-raise', type=float, default=0.0, help='mutation: head + head sphere this many m up except the last --head-raise-skip-end frames, must fail the roof phase')
    ap.add_argument('--head-raise-skip-end', type=int, default=18)
    ap.add_argument('--no-body', action='store_true', help='switch the body shell / roof phases off (diagnosis only)')
    ap.add_argument('--rear-pillar', action='store_true', help='include the SYNTHETIC B-pillar block in the rear gate (default off: its depth / length are guesses)')
    ap.add_argument('--no-rear', action='store_true', help='switch the rear gate off (diagnosis only)')
    ap.add_argument('--no-env', action='store_true', help='switch the open-pose envelope off (classes.<c>.pose_envelope of door_timing.json; diagnosis / searches only)')
    ap.add_argument('--rear-ref', default=None, help="vanilla player clip glb of the same set (or 'auto': p?a_<class>_<seat> next to the door data) - the rear gate is relative to it")
    a = ap.parse_args()
    tj = load_timing(a.timing)
    if a.approach:
        import door_approach_cli as AC
        sys.exit(AC.approach_cli(a, tj))

    def close_for(path, cls, seat):
        if a.close_after is not None:
            return a.close_after
        if a.close_from_timing and not a.close_vanilla:
            g = parse_name(path)[0]
            if a.close_own:
                v = tj.get('clips', {}).get(cls, {}).get(seat, {}).get('p%sa' % g, {}).get('close_after_s')
                if v is None:
                    raise SystemExit('no close_after_s for %s %s %s in door_timing.json' % (cls, seat, g))
                return v
            d = G.seat_delta(tj, cls, seat)
            if d is None:
                raise SystemExit('no clips entry for %s %s in door_timing.json' % (cls, seat))
            return G.vanilla_close_after(tj, cls, seat) + d
        return None

    def waiver_for(cls, g, seat):
        for w in a.waive:
            k, cm = w.rsplit('=', 1)
            kc, kg, ks = k.split(':')
            if kc == cls and kg in (g, '*') and ks == seat:
                return float(cm)
        return None

    if a.table:
        files = []
        for g in a.table:
            files += sorted(glob.glob(g))
        if not files:
            raise SystemExit('--table: no clip matches ' + ' '.join(a.table))
        rows, bad = [], 0
        for f in files:
            g, cls, seat = parse_name(f)
            m = Model(f, cls, seat, a.clip_name, a.door_data, tj, lag=a.v_lag, shift_rear=a.shift_rear, no_rear=a.no_rear, rear_ref=rear_ref_for(a, f, cls, seat), rear_pillar=a.rear_pillar, no_env=a.no_env,
                  no_body=a.no_body, head_raise=a.head_raise, head_raise_skip_end=a.head_raise_skip_end)
            m.body_tol_cm, m.roof_tol_cm = a.body_tol_cm, a.roof_tol_cm
            for w in a.waive_body:
                k, cm = w.split('=', 1)
                kc, kg, ks = k.split(':')
                if kc == cls and kg in (g, '*') and ks == seat:
                    m.body_tol_cm, m.roof_tol_cm = parse_waive_body(cm)
            wv = waiver_for(cls, g, seat)
            opn, opening = (a.open_tol_cm, a.opening_tol_cm) if wv is None else (wv, wv)
            r = analyse(m, close_for(f, cls, seat), a.window, tol_cm=a.tol_cm, open_tol_cm=opn, opening_tol_cm=opening)
            late = bool(r['close_after_s'] + r['event_clip_s'] > r['clip_end_s'] + G.LATE_SLACK_S)
            r['late'] = late
            r['waiver_cm'] = wv
            r['door_clearance_ok'] = bool(r['door_clearance_ok'] and not late)
            bad += not r['door_clearance_ok']
            print('%-34s %-9s close_after %.3f  %-4s  clearance cm: %s%s%s' % (os.path.basename(f).replace('.anims.glb', '')[:34], seat, r['close_after_s'],
                                                                           'ok' if r['door_clearance_ok'] else 'FAIL', fmt_phases(r),
                                                                           '  LATE' if late else '',
                                                                           '  [%s]' % r['timing_status'] if r['timing_status'] != 'measured' else ''))
            if wv is not None:
                vac = [p for p in ('opening', 'open') if p in r['phases'] and len(r['phases'][p]['ungated_capsules']) == len(r['phases'][p]['by_capsule'])]
                print('    waiver %.1f cm on opening/open: %s' % (wv, ('NO GATE in %s (the waiver is >= every capsule radius, max %.0f cm: the phase cannot fail)' % (
                    '+'.join(vac), max(BODY_R) * 100)) if vac else 'partial (capsules over %s cm radius remain gated)' % wv))
            print('    closing/closed per capsule (pen/gate cm): %s' % fmt_caps(r))
            rows.append(r)
        if a.report:
            json.dump(rows, open(a.report, 'w'), indent=1)
        print('TABLE %d clips, %d over (closed/opening/closing/closed > %.1f cm (+ per-capsule seated baseline) or open > %.1f cm, or LATE)' % (len(files), bad, a.tol_cm, a.open_tol_cm))
        sys.exit(1 if bad else 0)
    if not a.clip:
        ap.error('--clip or --table')
    if not (a.cls and a.seat):
        g, c, s = parse_name(a.clip)
        a.cls, a.seat = a.cls or c, a.seat or s
    m = Model(a.clip, a.cls, a.seat, a.clip_name, a.door_data, tj, a.mirror, a.v_lag, shift_rear=a.shift_rear, no_rear=a.no_rear, rear_ref=rear_ref_for(a, a.clip, a.cls, a.seat), rear_pillar=a.rear_pillar, no_env=a.no_env,
                 no_body=a.no_body, head_raise=a.head_raise, head_raise_skip_end=a.head_raise_skip_end)
    m.body_tol_cm, m.roof_tol_cm = a.body_tol_cm, a.roof_tol_cm
    if a.solve:
        sv = solve(m, tol_cm=a.tol_cm)
        sv['clip'] = a.clip
        sv['gender'] = 'p%sa' % parse_name(a.clip)[0]
        # settings the solve ran with (gen_door_timing.validate_fragment refuses a fragment with weaker ones)
        sv['open_tol_cm'] = a.open_tol_cm
        sv['opening_tol_cm'] = a.tol_cm if a.opening_tol_cm is None else a.opening_tol_cm
        sv['rear'] = m.rear is not None
        sv['body'] = m.bodyd is not None
        sv['body_tol_cm'], sv['roof_tol_cm'] = a.body_tol_cm, a.roof_tol_cm
        sv['rear_ref'] = a.rear_ref
        sg = start_gates(m, a, a.clip, a.cls, a.seat)
        sv['start_gate'] = sg  # None = not a UTPP clip; make_pack.py refuses a fragment without a passing start_gate
        sv['clip_sha256'] = hashlib.sha256(open(a.clip, 'rb').read()).hexdigest().upper()
        if sv.get('feasible'):
            r = analyse(m, sv['close_after_s'], a.window, tol_cm=a.tol_cm, open_tol_cm=a.open_tol_cm, opening_tol_cm=a.opening_tol_cm)
            sv['at_close_after'] = r
        if a.fragment:
            json.dump(sv, open(a.fragment, 'w'), indent=1)
        print('door_close_solve %s %s/%s: %s' % (os.path.basename(a.clip), a.cls, a.seat, json.dumps({k: v for k, v in sv.items() if k not in ('at_close_after', 'clip')})))
        if sv.get('feasible'):
            print('door_close_solve at close_after %.3f s: %s' % (sv['close_after_s'], fmt_phases(sv['at_close_after'])))
        sys.exit(0 if sv.get('feasible') and sv['at_close_after']['door_clearance_ok'] and (sg is None or sg['ok']) else 1)
    a.close_after = close_for(a.clip, a.cls, a.seat)
    if a.selftest:
        if a.close_after is None:
            a.close_after = m.tm['vanilla_close_after_s']
        sys.exit(0 if selftest(a, m) else 1)
    r = analyse(m, a.close_after, a.window, a.never_open, a.open_late, a.tol_cm, a.open_tol_cm, a.opening_tol_cm)
    r['tol_cm'], r['open_tol_cm'] = a.tol_cm, a.open_tol_cm
    if a.report:
        json.dump(r, open(a.report, 'w'), indent=1)
    ok = r['door_clearance_ok']
    sg = start_gates(m, a, a.clip, a.cls, a.seat) if not a.mirror else None
    if sg is not None:
        ok = bool(ok and sg['ok'])
        r['start_gate'] = sg
        if a.report:
            json.dump(r, open(a.report, 'w'), indent=1)
    print('door_clearance %s %s/%s [%s]: close_after %.3f s (vanilla %.3f), E window +-%.2f s, clearance cm: %s; worst closing/closed %.2f cm, open %.2f cm; tol %.1f / %.1f -> %s' % (
        os.path.basename(a.clip), a.cls, a.seat, r['timing_status'], r['close_after_s'], r['vanilla_close_after_s'], r['window_s'], fmt_phases(r),
        r['max_pen_window_cm'], r['open_door_contact_window_cm'], a.tol_cm, a.open_tol_cm, 'PASS' if ok else 'FAIL'))
    for ph, v in r['phases'].items():
        if not v['ok']:
            print('  over gate: %-12s pen %.2f cm (gate %.1f) at clip t=%.3f s: %s vs %s (E offset %+.2f)' % (ph, v['max_pen_cm'], v['gate_cm'], v['t_clip_s'], v['capsule'], v['door_bone'], v['event_offset_s']))
        if v.get('ungated_capsules') and ph in ('opening', 'open') and len(v['ungated_capsules']) == len(v['by_capsule']):
            print('  note: %s gate %.1f cm >= every capsule radius (max %.0f cm): this phase cannot fail (waiver is effectively off)' % (ph, v['gate_cm'], max(BODY_R) * 100))
    print('  closing/closed per capsule (pen/gate cm): %s' % fmt_caps(r))
    if a.expect_fail:
        print('EXPECT-FAIL door_clearance %s' % ('FAIL (as expected)' if not ok else 'PASS (mutation not rejected)'))
        sys.exit(0 if not ok else 1)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
