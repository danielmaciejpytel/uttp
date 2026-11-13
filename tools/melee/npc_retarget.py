"""NPC third-person melee clip -> player-rig clip (dump layout of melee_dump.py / melee_common.py), donor survey, donor contact sheets.

  python -I npc_retarget.py convert <npc set> <npc clip> --out <dump.npz> [--rig woman|man] [--mirror]
  python -I npc_retarget.py survey [--rig woman|man] [--set <npc set>] [--re REGEX] [--out <json>]
  python -I npc_retarget.py sheet --spec "clip[:m]=f,f,f;clip2=f,f" --out <png> [--rig woman|man] [--res 240,300] [--cols 6]

NPC sets: Extracted\\game\\npc-melee\\glb\\<set>.anims.glb (WolvenKit export WITH root motion; read-only).  Player rig: the vanilla FPP
melee set of the rig (Build\\work\\melee-anim-20261107\\glb\\pwa_fists.anims.glb / pma_fists), its 152 joints and reference locals.

Retarget (verified 2026-11-09: the 71-joint NPC rig is a subset of the 152-joint player rig with identical joint frames, so copying LOCAL
rotations by name reproduces the NPC world bone directions exactly; rest-delta transfers are wrong):
  * every player joint that exists on the NPC rig gets the NPC LOCAL rotation; translations = the player's reference locals (player bone
    lengths), except Hips, whose local translation (relative to Root) is the NPC's (legs are equal, hips rest heights differ by 0.5 cm);
  * joints missing on the NPC rig (HandEnd, the IK helpers / FPP spaces ...) = player reference locals; the FINGERS, when the NPC rig has
    none, = a closed fist (finger locals of the vanilla player clip FIST_SRC frame 0);
  * Root = identity on every frame (the clip body is relative to the capsule); the NPC root motion is returned separately (game axes) as
    root_t [T,3] / root_R [T,3,3] for the builder (planted feet under a capsule path);
  * --mirror: left <-> right (x -> -x in game / glTF axes, reference-pose corrected as build_entry_clip.mirror_pose), applied to the
    player-rig pose with Root = identity; the root path is mirrored with it.
Sampling: 30 fps, frame i at t = i / 30, frame count = round(duration * 30) + 1 (duration = last bone key); the Root track of root-motion
exports has NORMALISED key times (0..1) and is rescaled to seconds for every clip length (skel.sample_local only does it above 1 s).
Survey (per clip, game axes, character frame = NPC root at frame 0): root travel / yaw, phases, impact frame (striking hand furthest from
Spine3 near the start of the 'loop' phase), striking hand, strike class (jab / cross / hook / uppercut / body; lead hand from the feet),
hand heights at the ends, foot contacts and steps (contact.py).  Sheet: skinned V (skin_clip --world, graph deformation) of selected
frames of several donors, Blender Workbench stills rendered as one MKV (npc_film_render.py), ffmpeg tile -> one png.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_common as MC  # noqa: E402
import contact as CT  # noqa: E402

sys.path.insert(0, os.path.join(MC.ROOT, 'Source', 'tools', 'anim', 'vehicle-entry'))
import glbanim as GA  # noqa: E402
import skel as S  # noqa: E402

NPC_DIR = os.path.join(MC.ROOT, 'Extracted', 'game', 'npc-melee', 'glb')
OUT = os.path.join(MC.ROOT, 'Build', 'work', 'melee-npc-20261109')
PLAYER_SET = {'woman': 'pwa_fists', 'man': 'pma_fists'}
NPC_PREFIX = {'woman': 'wa_', 'man': 'ma_'}
NPC_ACTION = {r: NPC_PREFIX[r] + 'gang_unarmed_action_combat' for r in PLAYER_SET}
FIST_SRC = 'melee_idlepose'
M = np.diag([-1.0, 1.0, 1.0])   # lateral mirror (glTF x = game x)
BLENDER = os.environ.get('UTPP_BLENDER_EXE') or os.path.join(os.environ.get('ProgramFiles(x86)', ''), 'Steam', 'steamapps', 'common', 'Blender', 'blender.exe')
FFMPEG = os.path.join(MC.ROOT, 'Tools', 'pylibs', 'imageio_ffmpeg', 'binaries', 'ffmpeg-win-x86_64-v7.1.exe')
FINGER_KEYS = ('Thumb', 'Index', 'Middle', 'Ring', 'Pinky')
_SK = {}


def rig_arg(a, default='woman'):
    return a[a.index('--rig') + 1] if '--rig' in a else default


def skeleton(path):
    if path not in _SK:
        _SK[path] = S.Skeleton.from_glb(path)
    return _SK[path]


def npc_set_for(rig, kind='action'):
    return NPC_PREFIX[rig] + 'gang_unarmed_' + kind + '_combat'


def sample_clip(sk, clip):
    """-> lt [T,N,3], lr [T,N,4] (glTF axes, the clip rig), extras, duration (s)."""
    ai = sk.clip(clip)
    ch = GA.channels(sk.gj, sk.binb, ai)
    ex = sk.gj['animations'][ai].get('extras', {})
    dur = max(float(tr[0][-1]) for b, d in ch.items() if b != 'Root' for tr in d.values())
    n = int(round(dur * MC.FPS)) + 1
    times = np.arange(n) / MC.FPS
    if 'Root' in ch and ex.get('rootMotionType') not in (None, 'None', 'Unknown'):
        for k in list(ch['Root']):
            t, v, it = ch['Root'][k]
            if float(t[-1]) <= 1.0001:
                ch['Root'][k] = (t * dur, v, it)
    lt = np.repeat(sk.rest_t[None], n, 0).copy()
    lr = np.repeat(sk.rest_r[None], n, 0).copy()
    for b, d in ch.items():
        if b in sk.index:
            i = sk.index[b]
            if 'translation' in d:
                lt[:, i] = GA.sample(d['translation'], times, 'translation')
            if 'rotation' in d:
                lr[:, i] = GA.sample(d['rotation'], times, 'rotation')
    return lt, S.qcontinuous(lr), ex, dur


def g2game_R(Rg):
    return MC.CI[:3, :3] @ Rg @ MC.C[:3, :3]


def g2game_t(t):
    return np.einsum('ij,...j->...i', MC.CI[:3, :3], t)


class Player:
    """Player rig (152 joints): reference locals (glTF) of the vanilla FPP set + the dump-layout constants (game axes)."""

    def __init__(self, rig='woman'):
        self.rig = rig
        self.set = PLAYER_SET[rig]
        self.sk = skeleton(os.path.join(MC.WORK, 'glb', self.set + '.anims.glb'))
        self.ref = MC.load_dump(MC.clip_path(self.set, FIST_SRC))
        assert self.ref['names152'] == list(self.sk.names), 'dump joint order differs from the player glb'
        self.names = list(self.sk.names)
        self.ix = self.sk.index
        self.par = np.array(self.sk.parent)
        ri = self.ix['Root']
        assert np.allclose(self.sk.rest_t[ri], 0, atol=1e-6) and abs(abs(self.sk.rest_r[ri][3]) - 1) < 1e-6, 'player Root reference is not identity'
        # closed fist: finger locals of the vanilla clip (glTF axes) at frame 0
        sk_lt, sk_lr, _, _ = sample_clip(self.sk, FIST_SRC)
        self.fist = {n: sk_lr[0, self.ix[n]].copy() for n in self.names if any(k in n for k in FINGER_KEYS) and 'Hand' in n}


def retarget(P, npc_sk, nlt, nlr):
    """NPC locals (glTF) -> player locals (glTF) with Root = identity, list of player joints missing on the NPC rig."""
    T = nlt.shape[0]
    lt = np.repeat(P.sk.rest_t[None], T, 0).copy()
    lr = np.repeat(P.sk.rest_r[None], T, 0).copy()
    missing = []
    npc_has_fingers = any(any(k in n for k in FINGER_KEYS) and 'Hand' in n for n in npc_sk.names)
    for j, n in enumerate(P.names):
        k = npc_sk.index.get(n)
        if n == 'Root':
            continue
        if k is None:
            missing.append(n)
            if not npc_has_fingers and n in P.fist:
                lr[:, j] = P.fist[n]
            continue
        lr[:, j] = nlr[:, k]
        if n == 'Hips':
            lt[:, j] = nlt[:, k]
    return lt, S.qcontinuous(lr), missing


def mirror_locals(sk, lt, lr):
    """left <-> right mirrored pose (x -> -x), reference-pose corrected (build_entry_clip.mirror_pose)."""
    Wt, Wr = sk.world(lt, lr)
    rt, rr = sk.rest_world()
    Wt2, Wr2 = np.zeros_like(Wt), np.zeros_like(Wr)
    for b, name in enumerate(sk.names):
        mb = sk.index.get(S.mirror_name(name), b)
        corr = (M @ rr[mb] @ M).T @ rr[b]
        Wr2[:, b] = np.einsum('ij,tjk,kl->til', M, Wr[:, mb], M) @ corr
        Wt2[:, b] = Wt[:, mb] @ M.T
    lt2, lr2 = sk.local_from_world(Wt2, Wr2)
    return lt2, lr2


def locals_to_game(lt, lr):
    """glTF locals -> game-axes locals (L_game = C^-1 L C)."""
    Rl = S.q2m(lr)
    C3, CI3 = MC.C[:3, :3], MC.CI[:3, :3]
    Rg = np.einsum('ij,tnjk,kl->tnil', CI3, Rl, C3)
    tg = np.einsum('ij,tnj->tni', CI3, lt)
    return tg, R.from_matrix(Rg.reshape(-1, 3, 3)).as_quat().reshape(lt.shape[:2] + (4,))


def make_dump(P, lt_g, lr_g, meta):
    """game-axes locals [T,152] -> dump dict (melee_dump layout)."""
    W = MC.fk_world(P.par, lt_g, lr_g)
    ref = P.ref
    return dict(W=W[:, :73], names=np.array(ref['names']), parents=ref['parents'], rest=ref['rest'], fps=30,
                W_extra=W[:, 73:].astype(np.float32), names_extra=np.array([str(x) for x in ref['names_extra']]),
                lt=lt_g.astype(np.float32), lr=lr_g.astype(np.float32), names152=np.array(ref['names152']), parents152=ref['parents152'],
                rest_lt=ref['rest_lt'], rest_lr=ref['rest_lr'], meta=json.dumps(meta))


def save_dump(path, d):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savez_compressed(path, **d)


class Donor:
    """One NPC clip on the player rig: game-axes locals (lt, lr [T,152]), world W [T,152,4,4] (Root = identity), NPC root path
    root_t [T,3] / root_R [T,3,3] (game axes), phases {name: (start, end)}, extras."""

    def __init__(self, P, clip, set_name=None, mirror=False):
        set_name = set_name or NPC_ACTION[P.rig]
        self.P, self.clip, self.set, self.mirror = P, clip, set_name, mirror
        nsk = skeleton(os.path.join(NPC_DIR, set_name + '.anims.glb'))
        nlt, nlr, ex, dur = sample_clip(nsk, clip)
        self.extras, self.dur, self.n = ex, dur, len(nlt)
        ri = nsk.index['Root']
        assert nsk.parent[ri] < 0
        rt = g2game_t(nlt[:, ri])
        rR = g2game_R(S.q2m(nlr[:, ri]))
        lt, lr, self.missing = retarget(P, nsk, nlt, nlr)
        if mirror:
            lt, lr = mirror_locals(P.sk, lt, lr)
            rt = rt * np.array([-1.0, 1.0, 1.0])
            Mg = np.diag([-1.0, 1.0, 1.0])
            rR = np.einsum('ij,tjk,kl->til', Mg, rR, Mg)
        self.root_t, self.root_R = rt, rR
        self.lt, self.lr = locals_to_game(lt, lr)
        self.lr = S.qcontinuous(self.lr)
        self.W = MC.fk_world(P.par, self.lt, self.lr)
        self.phases = {}
        for e in ex.get('animEvents') or []:
            if e.get('type', '').endswith('Phase') or 'Phase' in str(e.get('type', '')):
                pass
        for e in ex.get('animEvents') or []:
            nm = e.get('eventName')
            if nm in ('startup', 'loop', 'recover') and 'Phase' in str(e.get('type', '')):
                s0 = int(e.get('startFrame', 0))
                self.phases[nm] = (s0, s0 + int(e.get('durationInFrames', 0)))

    def dump(self, frames=None):
        f = slice(None) if frames is None else frames
        meta = dict(set=self.P.set, clip='npc:' + self.clip, rig=self.P.rig, animation_type='Normal', additive=False, root_motion_type='None',
                    frames=int(len(self.lt[f])), duration=float((len(self.lt[f]) - 1) / MC.FPS), events=[], npc_set=self.set, npc_clip=self.clip,
                    mirror=self.mirror)
        return make_dump(self.P, self.lt[f], self.lr[f], meta)


# ------------------------------------------------------------------ measures (game axes)
def P_of(W, ix, n):
    return W[:, ix[n], :3, 3]


def body_frame(W, ix):
    """hips heading frame [T,3,3] (columns right, forward, up) and the Spine3 position."""
    return MC.heading_frame(P_of(W, ix, 'LeftUpLeg'), P_of(W, ix, 'RightUpLeg')), P_of(W, ix, 'Spine3')


def hand_rel(W, ix, side):
    """hand position relative to Spine3 in the hips heading frame [T,3] (x right, y forward, z up)."""
    H, s3 = body_frame(W, ix)
    return np.einsum('tji,tj->ti', H, P_of(W, ix, side + 'Hand') - s3)


def world_points(Wbody_t, root_t, root_R):
    """body-relative positions [T,3] -> NPC world (root motion applied)."""
    return np.einsum('tij,tj->ti', root_R, Wbody_t) + root_t


SOLE_OFF = {}


def sole_heights(P, Wt_world):
    """sole height [T,2] (left, right) of world joint positions [T,152,3]: min(ankle - ankle rest height, toe - toe rest height)."""
    if not SOLE_OFF:
        rest = MC.fk_world(P.par, P.ref['rest_lt'][None], P.ref['rest_lr'][None])[0]
        for s in ('Left', 'Right'):
            SOLE_OFF[s] = (rest[P.ix[s + 'Foot'], 2, 3], rest[P.ix[s + 'ToeBase'], 2, 3])
    out = []
    for s in ('Left', 'Right'):
        fo, to = SOLE_OFF[s]
        out.append(CT.sole_height(Wt_world[:, P.ix[s + 'Foot'], 2], Wt_world[:, P.ix[s + 'ToeBase'], 2], fo, to))
    return np.stack(out, 1)


def impact(dn, side=None):
    """(impact frame, striking side, reach curves): hand furthest from Spine3 near the start of the loop phase (or over the clip)."""
    ix = dn.P.ix
    reach = {s: np.linalg.norm(P_of(dn.W, ix, s + 'Hand') - P_of(dn.W, ix, 'Spine3'), axis=1) for s in ('Left', 'Right')}
    lo, hi = 0, dn.n - 1
    if 'loop' in dn.phases:
        a, b = dn.phases['loop']
        lo, hi = max(0, a - 4), min(dn.n - 1, b + 2)
    gain = {s: float(reach[s][lo:hi + 1].max() - reach[s][:max(lo, 1)].min()) for s in reach}
    sd = side or max(gain, key=gain.get)
    f = lo + int(np.argmax(reach[sd][lo:hi + 1]))
    return f, sd, reach, gain


def strike_class(dn, f, sd, lead):
    ix = dn.P.ix
    h = hand_rel(dn.W, ix, sd)
    v = h[f] - h[max(0, f - 4)]
    lat = v[0] * (1 if sd == 'Left' else -1)       # + = toward the midline / across
    kind = 'cross' if sd != lead else 'jab'
    if h[f, 2] < -0.28:
        kind = 'body'
    elif v[2] > 0.18 and v[2] > abs(v[1]):
        kind = 'uppercut'
    elif lat > 0.18 and lat > v[1]:
        kind = 'hook'
    return kind, h[f], v


def survey_clip(P, clip, set_name):
    dn = Donor(P, clip, set_name)
    ix = P.ix
    rt = dn.root_t - dn.root_t[0]
    R0 = dn.root_R[0]
    rel = rt @ R0          # in the frame-0 root frame (R0^T rt)
    fwd0 = R0[:, 1]
    yaw = np.degrees(np.arctan2(-np.einsum('ti,i->t', dn.root_R[:, :, 1], R0[:, 0]), np.einsum('ti,i->t', dn.root_R[:, :, 1], fwd0)))
    Wt = dn.W[:, :, :3, 3]
    Wworld = np.einsum('tij,tnj->tni', dn.root_R, Wt) + dn.root_t[:, None]
    sole = sole_heights(P, Wworld)
    con = CT.detect_contacts(sole)
    feet = {}
    for k, s in enumerate(('Left', 'Right')):
        xy = Wworld[:, ix[s + 'Foot'], :2]
        runs = CT.slide_segments(con[:, k], xy, min_len=3)
        feet[s] = dict(contact_frames=int(con[:, k].sum()), runs=len(runs), max_drift_cm=round(max([r['drift_cm'] for r in runs] or [0]), 1),
                       travel_cm=round(float(np.linalg.norm(xy[-1] - xy[0]) * 100), 1),
                       lift_cm=round(float((sole[:, k] - sole[:, k].min()).max() * 100), 1))
    H, _ = body_frame(dn.W, ix)
    lf = np.einsum('ji,j->i', H[0], P_of(dn.W, ix, 'LeftFoot')[0] - P_of(dn.W, ix, 'RightFoot')[0])
    lead = 'Left' if lf[1] > 0 else 'Right'
    out = dict(frames=dn.n, dur=round(dn.dur, 3), phases=dn.phases, root_end_cm=np.round(rel[-1] * 100, 1).tolist(),
               root_max_cm=round(float(np.linalg.norm(rel[:, :2], axis=1).max() * 100), 1), yaw_end=round(float(yaw[-1]), 1),
               yaw_max=round(float(np.abs(yaw).max()), 1), lead_foot=lead, feet_gap_fwd_cm=round(float(lf[1] * 100), 1), feet=feet,
               hips_z_cm=round(float(P_of(dn.W, ix, 'Hips')[:, 2].mean() * 100), 1),
               heading0_deg=round(float(np.degrees(np.arctan2(H[0][0, 1], H[0][1, 1]))), 1), missing=len(dn.missing))
    f, sd, reach, gain = impact(dn)
    kind, hpos, v = strike_class(dn, f, sd, lead)
    out.update(impact=f, side=sd, kind=kind, reach_cm=round(float(reach[sd][f] * 100), 1), gain_cm={k: round(x * 100, 1) for k, x in gain.items()},
               hand_at_impact_cm=np.round(hpos * 100, 1).tolist(), hand_vel4_cm=np.round(v * 100, 1).tolist())
    for s in ('Left', 'Right'):
        h = hand_rel(dn.W, ix, s)
        out['%s_hand_start_cm' % s[0]] = np.round(h[0] * 100, 1).tolist()
        out['%s_hand_end_cm' % s[0]] = np.round(h[-1] * 100, 1).tolist()
    return out


def survey(a):
    rig = rig_arg(a)
    P = Player(rig)
    set_name = a[a.index('--set') + 1] if '--set' in a else NPC_ACTION[rig]
    rx = re.compile(a[a.index('--re') + 1]) if '--re' in a else None
    sk = skeleton(os.path.join(NPC_DIR, set_name + '.anims.glb'))
    res = {}
    for clip in GA.clips(sk.gj):
        if rx and not rx.search(clip):
            continue
        try:
            res[clip] = survey_clip(P, clip, set_name)
        except Exception as e:      # report and continue
            res[clip] = dict(error=repr(e))
        r = res[clip]
        if 'error' in r:
            print('%-36s ERROR %s' % (clip, r['error']))
            continue
        print('%-34s %3d ph=%-34s root_end=%-20s yaw=%6.1f lead=%s imp=%2d %s %-8s reach=%5.1f feet L%s/%s R%s/%s' % (
            clip, r['frames'], ','.join('%s%d-%d' % (k[0], v[0], v[1]) for k, v in sorted(r['phases'].items())), r['root_end_cm'], r['yaw_end'],
            r['lead_foot'][0], r['impact'], r['side'][0], r['kind'], r['reach_cm'], r['feet']['Left']['runs'], r['feet']['Left']['travel_cm'],
            r['feet']['Right']['runs'], r['feet']['Right']['travel_cm']))
    out = a[a.index('--out') + 1] if '--out' in a else os.path.join(OUT, 'survey_%s.json' % set_name)
    MC.jdump(res, out)
    print('WROTE', out)


# ------------------------------------------------------------------ skinned stills / contact sheets
def skin_W(W73, rest, rig, base, deform='graph'):
    """W [T,73,4,4] (game axes) -> skin cache <base>.npz / .pc2 (skin_clip.py --world)."""
    os.environ['UTPP_RIG'] = rig
    sys.path.insert(0, os.path.join(MC.ROOT, 'Source', 'tools', 'ai_motion'))
    import game_skeleton_bridge as CP
    assert CP.RIG_NAME == rig
    names, par, ref = CP.load_rig()
    clip = CP.to_utppclip(np.asarray(W73, float), names, par, ref, rest, os.path.basename(base), win=1)
    cj = base + '.utppclip.json'
    json.dump(clip, open(cj, 'w'), separators=(',', ':'))
    p = subprocess.run([MC.PY, '-I', os.path.join(MC.ROOT, 'Source', 'tools', 'ai_motion', 'skin_clip.py'), cj, base, '--world', '--deform', deform,
                        '--body', rig], capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError('skin_clip failed: ' + p.stderr[-800:])
    return base


def sheet_from_W(W73, rest, rig, labels, out_png, tmp, res='240,300', cols=6, cam=None, title=''):
    """skinned stills -> ONE png: Blender renders the frames into an MKV video (npc_film_render.py, camera follows the hips per frame),
    ffmpeg tiles the video frames into the sheet.  No per-frame png is written (media rule 2026-11-05: png files are never deleted, so
    none are made as intermediates); the MKV / skin intermediates stay in tmp (Build\work)."""
    os.makedirs(tmp, exist_ok=True)
    base = skin_W(W73, rest, rig, os.path.join(tmp, 'skin'))
    lj = os.path.join(tmp, 'labels.json')
    json.dump(list(labels), open(lj, 'w', encoding='utf-8'))
    mkv = os.path.join(tmp, 'sheet.mkv')
    cmd = [BLENDER, '-b', '--factory-startup', '-P', os.path.join(HERE, 'npc_film_render.py'), '--', base, mkv, lj, '--res', res, '--follow', '1',
           '--title', title]
    if cam:
        cmd += ['--cam', cam]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if 'RENDERED' not in p.stdout:
        raise RuntimeError('blender failed: ' + (p.stdout + p.stderr)[-800:])
    rows = (len(labels) + cols - 1) // cols
    p = subprocess.run([FFMPEG, '-y', '-loglevel', 'error', '-i', mkv, '-vf', 'tile=%dx%d:padding=2:color=white' % (cols, rows), '-frames:v', '1',
                        out_png], capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError('ffmpeg failed: ' + p.stderr[-400:])
    return out_png


def sheet(a):
    """--spec "clip[:m]=f,f,f;clip2=f,f" : rows of skinned stills, one row per donor (':m' = mirrored)."""
    rig = rig_arg(a)
    P = Player(rig)
    set_name = a[a.index('--set') + 1] if '--set' in a else NPC_ACTION[rig]
    out_png = os.path.abspath(a[a.index('--out') + 1])
    res = a[a.index('--res') + 1] if '--res' in a else '240,300'
    cols = int(a[a.index('--cols') + 1]) if '--cols' in a else 6
    Ws, labels = [], []
    for item in a[a.index('--spec') + 1].split(';'):
        name, fr = item.split('=')
        mirror = name.endswith(':m')
        clip = name[:-2] if mirror else name
        dn = Donor(P, clip, set_name, mirror)
        frs = [min(int(x), dn.n - 1) for x in fr.split(',')]
        frs = (frs + [frs[-1]] * cols)[:cols]
        Ws.append(dn.W[frs, :73])
        short = clip.replace('melee_', '').replace('combo_', 'c').replace('infinite_', 'inf').replace('_idle_to_attack', '_i2a')
        labels += ['%s%s %d' % (short, '(m)' if mirror else '', f) for f in frs]
    tmp = os.path.join(OUT, '_tmp', 'sheet_%d' % os.getpid())
    sheet_from_W(np.concatenate(Ws), P.ref['rest'], rig, labels, out_png, tmp, res, cols)      # tmp kept: no png inside, nothing deleted
    print('SHEET', out_png, len(labels))


def convert(a):
    rig = rig_arg(a)
    P = Player(rig)
    dn = Donor(P, a[2], a[1], '--mirror' in a)
    save_dump(a[a.index('--out') + 1], dn.dump())
    print('CONVERTED', a[1], a[2], dn.n, 'frames, missing joints on the NPC rig:', len(dn.missing))


if __name__ == '__main__':
    t0 = time.time()
    cmd = sys.argv[1]
    {'convert': convert, 'survey': survey, 'sheet': sheet}[cmd]([cmd] + sys.argv[2:])
    print('done %.1f s' % (time.time() - t0))
