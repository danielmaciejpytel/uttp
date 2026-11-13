"""Cyberpunk 2077 player skeleton (player_woman_skeleton, bones 0..72 = the .utppclip set) <-> SOMA77 bridge for
MotionBricks / Kimodo. Uses skeleton_bridge.Bridge with a CP profile, so game animations and source pack clips land in the same
SOMA space (soma npz: G, hips, names, k) and can be composed / in-betweened together.

CP dump npz (same layout as mannequin_dump.py dumps): names [73], parents [73], W [T,73,4,4] bone world matrices in game
space (metres, +Z up, character faces +Y, left = -X) INCLUDING root motion, rest [73,4,4] bind world matrices of the
rig reference pose, fps.
  - .utppclip.json: Root is identity, Trajectory carries the root motion, the rest is in the in-place model frame,
    so world = Trajectory * model for every bone below Root.
  - game .anims glb (WolvenKit export, glTF Y up): plain FK (root motion is on the Root track), converted to game
    axes with W_game = C^-1 W_gltf C (C = game Z up -> glTF Y up, as in skin_clip.py).

  python -I game_skeleton_bridge.py dump-clip <clip.utppclip.json> <out_dump.npz>
  python -I game_skeleton_bridge.py dump-glb <anims.glb> <clip name> <out_dump.npz> [--from S] [--to S]   (seconds)
  python -I game_skeleton_bridge.py fwd <dump.npz> <tpose.npz> <out_soma.npz>
  python -I game_skeleton_bridge.py inv <soma.npz> <template_dump.npz> <tpose.npz> <out.utppclip.json> [--frames a:b]
                                  [--leg-ik] [--name NAME] [--traj-window N] [--dump OUT_DUMP.npz]
                                  [--blend-span c:g]   (template frames c..g interpolated: the generated gap)
  python -I game_skeleton_bridge.py roundtrip <dump.npz> <tpose.npz>       (fwd + inv on itself, prints errors)
inv: bones SOMA does not drive (heels, InHand*, fingers, weapon slots, eyes) take the template dump's locals at
frame a+i. Trajectory = hips ground point and hips heading, both smoothed (centred window, default 15 frames);
bone translations below Hips are the template's locals (game clips animate clavicle / Spine3 translations, up to
~22 cm in vehicle exits, which SOMA cannot carry).
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'anim', 'vehicle-entry'))
import skeleton_bridge as SB  # noqa: E402

# rig of the clips: default = female V (unchanged). Male V (pma_* sets): environment UTPP_RIG=man (no command-line switch: compose_transition has its own --rig) -> the reference pose / bone lengths of player_man_skeleton
RIGS = {'woman': os.path.join(ROOT, 'Extracted', 'game', 'json', 'vanilla', 'player_woman_skeleton.rig.json'),
        'man': os.path.join(ROOT, 'Extracted', 'game', 'json', 'vanilla-man', 'player_man_skeleton.rig.json')}
RIG_NAME = os.environ.get('UTPP_RIG', 'woman')
RIG = RIGS[RIG_NAME]
NB = 73  # .utppclip bone set
C = np.eye(4)
C[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]  # game Z up -> glTF Y up
CI = np.linalg.inv(C)
CM_CP = np.array([[-1.0, 0, 0], [0, 0, 1.0], [0, 1.0, 0]])  # game (left -X, fwd +Y, up +Z) -> SOMA (left +X, up +Y, fwd +Z)

SIDES = ('Left', 'Right')
MAP = [  # soma joint, cp bone, soma primary pair, cp primary pair, secondary
    ('Hips', 'Hips', ('Hips', 'Neck1'), ('Hips', 'Neck'), 'LR'),
    ('Spine1', 'Spine', ('Spine1', 'Neck1'), ('Spine', 'Neck'), 'LR'),
    ('Spine2', 'Spine2', ('Spine2', 'Neck1'), ('Spine2', 'Neck'), 'LR'),
    ('Chest', 'Spine3', ('Chest', 'Neck1'), ('Spine3', 'Neck'), 'LR'),
    ('Neck1', 'Neck', ('Neck1', 'Head'), ('Neck', 'Head'), 'LR'),
    ('Neck2', 'Neck1', ('Neck2', 'Head'), ('Neck1', 'Head'), 'LR'),
    ('Head', 'Head', ('Neck1', 'Head'), ('Neck', 'Head'), 'LR'),
]
for S in SIDES:
    MAP += [
        (S + 'Shoulder', S + 'Shoulder', (S + 'Shoulder', S + 'Arm'), (S + 'Shoulder', S + 'Arm'), 'FWD'),
        (S + 'Arm', S + 'Arm', (S + 'Arm', S + 'ForeArm'), (S + 'Arm', S + 'ForeArm'), 'FWD'),
        (S + 'ForeArm', S + 'ForeArm', (S + 'ForeArm', S + 'Hand'), (S + 'ForeArm', S + 'Hand'), 'FWD'),
        (S + 'Hand', S + 'Hand', (S + 'Hand', S + 'HandMiddle2'), (S + 'Hand', S + 'HandMiddle1'),
         ('PALM', (S + 'HandIndex2', S + 'HandPinky2'), (S + 'HandIndex1', S + 'HandPinky1'))),
        (S + 'Leg', S + 'UpLeg', (S + 'Leg', S + 'Shin'), (S + 'UpLeg', S + 'Leg'), 'FWD'),
        (S + 'Shin', S + 'Leg', (S + 'Shin', S + 'Foot'), (S + 'Leg', S + 'Foot'), 'FWD'),
        (S + 'Foot', S + 'Foot', (S + 'Foot', S + 'ToeBase'), (S + 'Foot', S + 'ToeBase'), 'LR'),
        (S + 'ToeBase', S + 'ToeBase', (S + 'Foot', S + 'ToeBase'), (S + 'Foot', S + 'ToeBase'), 'LR'),
    ]
PROFILE_CP = dict(map=MAP, spine_fill={'Spine1': ('Spine', 'Spine2', 0.5)}, derived={}, cm=CM_CP, unit=1.0,
                  pelvis='Hips', legs={S: (S + 'UpLeg', S + 'Leg', S + 'Foot') for S in SIDES})


def mat(t, q):
    import skel as S
    m = np.eye(4)
    q = np.asarray(q, dtype=np.float64)
    m[:3, :3] = S.q2m(q / np.linalg.norm(q))
    m[:3, 3] = t
    return m


def load_rig():
    rc = json.load(open(RIG, encoding='utf-8'))['Data']['RootChunk']
    names = [b['$value'] for b in rc['boneNames']][:NB]
    par = np.array(rc['boneParentIndexes'][:NB])
    ref = []
    for t in rc['boneTransforms'][:NB]:
        p, q = t['Translation'], t['Rotation']
        ref.append(mat([p['X'], p['Y'], p['Z']], [q['i'], q['j'], q['k'], q['r']]))
    return names, par, np.array(ref)


def fk(par, L):
    W = np.zeros_like(L)
    for i in range(L.shape[-3]):
        W[..., i, :, :] = L[..., i, :, :] if par[i] < 0 else W[..., par[i], :, :] @ L[..., i, :, :]
    return W


def rest_world(par, ref):
    return fk(par, ref[None])[0]


def world_from_utppclip(path, names, par):
    clip = json.load(open(path, encoding='utf-8'))
    assert clip['bone_names'][:NB] == names, 'utppclip bone order differs from the rig'
    L = np.array([[mat(b[0:3], b[3:7]) for b in fr['bones'][:NB]] for fr in clip['frames']])
    Wm = fk(par, L)  # in-place model frame (Root identity, Trajectory = root motion under Root)
    traj = Wm[:, 1]
    W = traj[:, None] @ Wm
    W[:, 0] = np.eye(4)
    W[:, 1] = traj
    return W, clip


def world_from_glb(path, clip_name, t0=None, t1=None, fps=30):
    import glbanim as GA
    import skel as S
    sk = S.Skeleton.from_glb(path)
    ai = sk.clip(clip_name)
    ch = GA.channels(sk.gj, sk.binb, ai)
    dur = max(float(tr[0][-1]) for b, d in ch.items() if b != 'Root' for tr in d.values())
    t0 = 0.0 if t0 is None else t0
    t1 = dur if t1 is None else min(t1, dur)
    times = np.arange(t0, t1 + 1e-6, 1.0 / fps)
    lt, lr = sk.sample_local(ai, times)
    Wt, Wr = sk.world(lt, lr)
    names, par, _ = load_rig()
    W = np.tile(np.eye(4), (len(times), NB, 1, 1))
    for i, n in enumerate(names):
        j = sk.index[n]
        g = np.tile(np.eye(4), (len(times), 1, 1))
        g[:, :3, :3] = Wr[:, j]
        g[:, :3, 3] = Wt[:, j]
        W[:, i] = CI @ g @ C
    return W, dict(duration=dur, t0=t0, t1=float(times[-1]), clip=clip_name, glb=os.path.basename(path))


def save_dump(path, W, names, par, rest, **extra):
    np.savez(path, W=W, names=np.array(names), parents=par, rest=rest, fps=30, **extra)


def smooth(x, win):
    if win <= 1 or len(x) < 3:
        return x.copy()
    h = win // 2
    xp = np.concatenate([np.repeat(x[:1], h, 0), x, np.repeat(x[-1:], h, 0)])
    k = np.ones(2 * h + 1) / (2 * h + 1)
    return np.stack([np.convolve(xp[:, c], k, mode='valid') for c in range(x.shape[1])], 1)


def trajectory(W, rest, names, win=15):
    """Per frame Trajectory world matrix: smoothed hips ground point (z = 0) and smoothed hips heading (yaw about +Z)."""
    hi = names.index('Hips')
    R0 = rest[hi, :3, :3]
    fwd = np.einsum('tij,j->ti', W[:, hi, :3, :3] @ R0.T, np.array([0.0, 1.0, 0.0]))
    yaw = np.unwrap(np.arctan2(-fwd[:, 0], fwd[:, 1]))
    xy = smooth(W[:, hi, :2, 3], win)
    yaw = smooth(yaw[:, None], win)[:, 0]
    T = np.tile(np.eye(4), (len(W), 1, 1))
    c, s = np.cos(yaw), np.sin(yaw)
    T[:, 0, 0], T[:, 0, 1], T[:, 1, 0], T[:, 1, 1] = c, -s, s, c
    T[:, :2, 3] = xy
    return T


def to_utppclip(W, names, par, ref, rest, name, win=15, note=''):
    import skel as S
    traj = trajectory(W, rest, names, win)
    Tinv = np.linalg.inv(traj)
    frames = []
    for t in range(len(W)):
        Wm = Tinv[t] @ W[t]  # in-place model frame
        bones = []
        for i, n in enumerate(names):
            if i == 0:
                m = np.eye(4)
            elif i == 1:
                m = traj[t]
            elif par[i] == 0:
                m = Wm[i] if n == 'Hips' else ref[i]
            else:
                m = np.linalg.inv(Wm[par[i]]) @ Wm[i]
            q = S.m2q(SB.ortho(m[:3, :3]))
            bones.append([float(x) for x in m[:3, 3]] + [float(x) for x in q] + [1.0, 1.0, 1.0])
        frames.append(bones)
    for i in range(len(names)):  # quaternion sign continuity
        for t in range(1, len(frames)):
            a, b = np.array(frames[t - 1][i][3:7]), np.array(frames[t][i][3:7])
            if a.dot(b) < 0:
                frames[t][i][3:7] = [-x for x in frames[t][i][3:7]]
    return {'format': 'utppclip', 'version': 1, 'source_clip': name, 'fps': 30, 'num_frames': len(W),
            'bone_names': names, 'rig': 'player_%s_skeleton' % RIG_NAME, 'rig_bone_count': 152, 'curves': {}, 'notifies': [],
            'retarget': {'method': 'game_skeleton_bridge.py inv (SOMA77 -> CP); Trajectory = smoothed hips ground point and '
                                   'heading (window %d)' % win, 'note': note},
            'frames': [{'bones': [[round(x, 6) for x in b] for b in fr]} for fr in frames]}


def locals_of(W, par):
    L = W.copy()
    for i in range(W.shape[1]):
        if par[i] >= 0:
            L[:, i] = np.linalg.inv(W[:, par[i]]) @ W[:, i]
    return L


def blend_span(L, c, g):
    """Template locals for frames c+1..g-1 interpolated between frames c and g (lerp translation, slerp rotation)."""
    from scipy.spatial.transform import Rotation as Rot
    from scipy.spatial.transform import Slerp
    L = L.copy()
    w = np.linspace(0, 1, g - c + 1)[1:-1]
    for i in range(L.shape[1]):
        r = Slerp([0, 1], Rot.from_matrix(np.stack([SB.ortho(L[c, i, :3, :3]), SB.ortho(L[g, i, :3, :3])])))(w)
        L[c + 1:g, i, :3, :3] = r.as_matrix()
        L[c + 1:g, i, :3, 3] = (1 - w)[:, None] * L[c, i, :3, 3] + w[:, None] * L[g, i, :3, 3]
    return L


def cp_inv(br, d, G, hips, frames, leg_ik=False, blend=None):
    """SOMA -> CP world matrices. Bone translations below Hips come from the template locals (game clips animate
    clavicle / spine translations that SOMA cannot carry); blend=(c, g) interpolates the template across a gap."""
    par = np.asarray(d['parents'])
    Lt = locals_of(d['W'], par)
    if blend:
        Lt = blend_span(Lt, *blend)
    Wt = fk(par, Lt)
    W = br.inv(G, hips, Wt, frames, leg_ik=leg_ik)
    L = locals_of(W, par)
    keep = [i for i, n in enumerate(br.unames) if par[i] >= 0 and n not in ('Trajectory', br.pelvis)]
    L[:, keep, :3, 3] = Lt[frames][:, keep, :3, 3]
    return fk(par, L)


def load_cp_bridge(tpose, dump):
    d = np.load(dump)
    br = SB.Bridge(tpose, d['rest'], d['names'], PROFILE_CP)
    br.uparents = np.asarray(d['parents'])
    return br, d


def pose_errors(W0, W1, names):
    """max / mean joint position error (cm) and max rotation error (deg) over the SOMA-driven bones."""
    idx = [names.index(u) for _, u, *_ in MAP]
    A, B = W0[:, idx], W1[:, idx]  # (avoid mixed advanced indexing: it moves the bone axis first)
    dp = np.linalg.norm(A[..., :3, 3] - B[..., :3, 3], axis=-1) * 100
    R = np.einsum('tbji,tbjk->tbik', A[..., :3, :3], B[..., :3, :3])
    ang = np.degrees(np.arccos(np.clip((np.trace(R, axis1=-2, axis2=-1) - 1) / 2, -1, 1)))
    return dp.max(), dp.mean(), ang.max(), [names[idx[k]] for k in np.argsort(-dp.max(0))[:3]]


def main(a):
    cmd = a[0]
    if cmd in ('dump-clip', 'dump-glb'):
        names, par, ref = load_rig()
        rest = rest_world(par, ref)
        if cmd == 'dump-clip':
            W, _ = world_from_utppclip(a[1], names, par)
            save_dump(a[2], W, names, par, rest, src=os.path.basename(a[1]))
            out = a[2]
        else:
            t0 = float(a[a.index('--from') + 1]) if '--from' in a else None
            t1 = float(a[a.index('--to') + 1]) if '--to' in a else None
            W, info = world_from_glb(a[1], a[2], t0, t1)
            save_dump(a[3], W, names, par, rest, src='%s:%s' % (info['glb'], info['clip']), t0=info['t0'], t1=info['t1'])
            out = a[3]
        print('CP_DUMP %s frames %d' % (out, len(W)))
    elif cmd == 'fwd':
        br, d = load_cp_bridge(a[2], a[1])
        G, hips, pos = br.fwd(d['W'])
        np.savez(a[3], G=G, hips=hips, pos=pos, names=np.array(br.snames), k=br.k, fps=30)
        print('CP_FWD %s frames %d k=%.4f' % (a[3], len(G), br.k))
    elif cmd == 'inv':
        br, d = load_cp_bridge(a[3], a[2])
        s = np.load(a[1])
        T = len(s['G'])
        f0 = int(a[a.index('--frames') + 1].split(':')[0]) if '--frames' in a else 0
        frames = [min(f0 + i, len(d['W']) - 1) for i in range(T)]
        blend = tuple(int(x) for x in a[a.index('--blend-span') + 1].split(':')) if '--blend-span' in a else None
        W = cp_inv(br, d, s['G'], s['hips'], frames, leg_ik='--leg-ik' in a, blend=blend)
        names, par, ref = load_rig()
        win = int(a[a.index('--traj-window') + 1]) if '--traj-window' in a else 15
        name = a[a.index('--name') + 1] if '--name' in a else os.path.basename(a[4]).split('.')[0]
        clip = to_utppclip(W, names, par, ref, d['rest'], name, win, note='soma %s, template %s'
                           % (os.path.basename(a[1]), os.path.basename(a[2])))
        json.dump(clip, open(a[4], 'w', encoding='utf-8'), separators=(',', ':'))
        if '--dump' in a:
            save_dump(a[a.index('--dump') + 1], W, names, par, d['rest'])
        print('CP_INV %s frames %d' % (a[4], T))
    elif cmd == 'roundtrip':
        br, d = load_cp_bridge(a[2], a[1])
        W0 = d['W']
        G, hips, pos = br.fwd(W0)
        W1 = cp_inv(br, d, G, hips, list(range(len(W0))))
        names = [str(x) for x in d['names']]
        mx, mean, rmax, worst = pose_errors(W0, W1, names)
        _, par, ref = load_rig()
        clip = to_utppclip(W1, names, par, ref, d['rest'], 'roundtrip')
        tmp = a[1] + '.rt.utppclip.json'
        json.dump(clip, open(tmp, 'w', encoding='utf-8'))
        W2, _ = world_from_utppclip(tmp, names, par)
        os.remove(tmp)
        mx2, mean2, rmax2, _ = pose_errors(W1, W2, names)
        print('ROUNDTRIP %s frames %d k=%.4f | bridge: pos max %.3f mean %.3f cm, rot max %.3f deg (worst %s) | '
              'utppclip write/read: pos max %.4f cm, rot max %.4f deg'
              % (a[1], len(W0), br.k, mx, mean, rmax, worst, mx2, rmax2))


if __name__ == '__main__':
    main(sys.argv[1:])
