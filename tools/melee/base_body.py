"""Base body of the melee redo (Female or Male V as he/she looks in game, source pack poses on the V rig) + human joint ranges from source pack clips.

  python -I base_body.py build [--rig man]   -> Build\\...\\redo\\base_reference.json (+ base_pose.npz); man -> ...\\redo_man\\
  python -I base_body.py show  [--rig man]   -> prints the base measures
Rig = environment variable UTPP_RIG ('woman' default, 'man'), resolved ONCE at import (same switch as game_skeleton_bridge).  A `--rig <woman|man>` command-line
argument of any tool that imports this module sets the variable and is REMOVED from sys.argv at import (before game_skeleton_bridge reads it), so the
positional arguments of the tools stay as they were.  Per-rig constants: OVERLAY, IDLE_POSE, CLIP_DIR, REDO.

Data: Animations\\retarget\\out_sourcepack_set3_woman (source pack Neutral set retargeted to the V rig, the source of the shipped MM
database, .utppclip.json: locals of the 73 body joints, Trajectory = root motion).  A clip pose is turned into the dump layout of the
melee tools (W [T,73,4,4] world, game axes, rest) with game_skeleton_bridge.world_from_utppclip.
The base pose = ONE frame of a source pack clip (BASE_CLIP / BASE_FRAME), expressed in the in-place frame (Trajectory removed, hips heading = +Y).
Ranges: per feature of anatomy_gate.features over the source pack locomotion / idle / crouch / traversal clips; these (not the vanilla melee
clips, whose FPP arms are broken) are the reference of anatomy_gate_base.py.
"""
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import anatomy_gate as AG  # noqa: E402
import melee_common as MC  # noqa: E402

if '--rig' in sys.argv:     # explicit CLI switch: must happen before game_skeleton_bridge is imported (it reads UTPP_RIG once)
    _i = sys.argv.index('--rig')
    os.environ['UTPP_RIG'] = sys.argv[_i + 1]
    del sys.argv[_i:_i + 2]
os.environ.setdefault('UTPP_RIG', 'woman')
RIG_NAME = os.environ['UTPP_RIG']
assert RIG_NAME in ('woman', 'man'), 'UTPP_RIG must be woman or man, got %r' % RIG_NAME
sys.path.insert(0, os.path.join(MC.ROOT, 'Source', 'tools', 'ai_motion'))
sys.path.insert(0, os.path.join(MC.ROOT, 'Source', 'tools', 'anim', 'vehicle-entry'))
import game_skeleton_bridge as CP  # noqa: E402
sys.path.insert(0, os.path.join(MC.ROOT, 'Source', 'tools', 'analysis'))
from export_p1h_blender_data import overlay_skeleton, overlay_static_pose  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402
from pathlib import Path  # noqa: E402

assert CP.RIG_NAME == RIG_NAME, 'game_skeleton_bridge was imported before base_body with another UTPP_RIG (%s vs %s)' % (CP.RIG_NAME, RIG_NAME)
# woman: B307 = E7E5 in every skeleton pose (E7E5 only differs in 4 curve values); man: the shipped file (= Data\motion\utpp_overlays_man.utppov, SHA 8F2C1D08...)
OVERLAY = Path(MC.ROOT) / 'Data' / 'overlay' / ('utpp_overlays_%s.utppov' % RIG_NAME)
READY_POSE = 'Pose_Pistol2H_Stand_Ready_Idle'   # legs / hips / clavicles / arms of the melee base (shipped combat-ready stance), both rigs
# neutral standing idle: spine / neck / head of the base (upright) + the static 'V idle' panel (MotionPose.hpp: Masculine style policy for Body::Man)
IDLE_POSE = {'woman': 'Pose_Feminine_Stand_Idle', 'man': 'Pose_Masculine_Stand_Idle'}[RIG_NAME]

REDO = os.path.join(MC.WORK, 'redo' if RIG_NAME == 'woman' else 'redo_man')      # woman path unchanged
CLIP_DIR = os.path.join(MC.ROOT, 'Animations', 'retarget', 'out_sourcepack_set3_' + RIG_NAME)
BASE_CLIP = 'M_Neutral_Stand_Idle_Loop'
BASE_FRAME = 0
# source pack clips that define the human ranges (name patterns inside CLIP_DIR)
RANGE_PATTERNS = ['M_Neutral_Stand_Idle_Loop', 'M_Neutral_Stand_Idle_Break_v0*', 'M_Neutral_Walk_Loop_*', 'M_Neutral_Run_Loop_*',
                  'M_Neutral_Sprint_Loop_*', 'M_Neutral_Crouch_Idle_Loop', 'M_Neutral_Crouch_Loop_*', 'M_Neutral_Jump_*',
                  'M_Neutral_Traversal_*', 'M_Neutral_Run_Start_*', 'M_Neutral_Run_Stop_*', 'M_Neutral_Stand_Turn_*',
                  'AO_Pistol1H_Stand_Sweep', 'AO_Pistol2H_Stand_Sweep', 'AO_Rifle_Stand_Sweep', 'M_Neutral_Run_Turn_*']

_RIG = None


def rig():
    global _RIG
    if _RIG is None:
        names, par, ref = CP.load_rig()
        _RIG = (names, par, ref, CP.rest_world(par, ref))
    return _RIG


def sourcepack_W(clip):
    """World matrices [T,73,4,4] (game axes, root motion included) of a source pack clip on the V rig of the selected body (CLIP_DIR)."""
    names, par, ref, rest = rig()
    W, _ = CP.world_from_utppclip(os.path.join(CLIP_DIR, clip + '.utppclip.json'), names, par)
    return W


def inplace(W, names):
    """Remove the root motion: hips ground point at the origin, hips heading along +Y (yaw about +Z), per frame."""
    traj = CP.trajectory(W, rig()[3], names, win=1)
    return np.linalg.inv(traj)[:, None] @ W


def overlay_names():
    return overlay_skeleton(OVERLAY)[0]


def overlay_L(name):
    """Local transforms [73,4,4] of a shipped overlay pose (game axes, same joint order as the rig)."""
    p = overlay_static_pose(OVERLAY, name, len(overlay_names()))
    L = np.tile(np.eye(4), (73, 1, 1))
    for i in range(73):
        L[i, :3, :3] = R.from_quat(p[i][3:7]).as_matrix()
        L[i, :3, 3] = p[i][:3]
    return L


def overlay_pose_names():
    import struct
    data = OVERLAY.read_bytes()
    _, sc = struct.unpack_from('<II', data, 24)
    sec = {}
    for i in range(sc):
        tag, off, size, count = struct.unpack_from('<4sIII', data, 64 + i * 16)
        sec[tag.decode()] = (off, size, count)
    po, _, pn = sec['POSE']
    return [data[po + i * 104:po + i * 104 + 64].split(b"\x00", 1)[0].decode() for i in range(pn)]


def fkL(L):
    return CP.fk(rig()[1], L[None])[0]


def compose_base():
    """locals [73,4,4] of the melee base body: READY_POSE (hips, legs, clavicles, arms) with the spine + neck + head of IDLE_POSE (upright). -> (L, W, idle L)."""
    names = rig()[0]
    L = overlay_L(READY_POSE)
    Li = overlay_L(IDLE_POSE)
    for n in ('Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head', 'LeftEye', 'RightEye'):
        if n in names:
            L[names.index(n)] = Li[names.index(n)]
    W = fkL(L)
    return L, W[None], overlay_L(IDLE_POSE)


def rig_for_gate():
    names, par, ref, rest = rig()
    return AG.Rig(names, par, rest)


def collect_ranges(verbose=False):
    rg = rig_for_gate()
    names = rig()[0]
    files = []
    for pat in RANGE_PATTERNS:
        files += glob.glob(os.path.join(CLIP_DIR, pat + '.utppclip.json'))
    files = sorted(set(files))
    series = {}
    used = []
    for f in files:
        c = os.path.basename(f)[:-len('.utppclip.json')]
        W = sourcepack_W(c)
        if len(W) < 5:
            continue
        ft = AG.features(W, rg)
        used.append((c, len(W)))
        for k, v in ft.items():
            series.setdefault(k, []).append(np.asarray(v))
    out = {}
    for k, vs in series.items():
        a = np.concatenate(vs)
        out[k] = dict(min=float(a.min()), max=float(a.max()), p01=float(np.percentile(a, 0.5)), p99=float(np.percentile(a, 99.5)), n=int(len(a)))
    return out, used


def measures(W1, rg):
    """Base-body measures of ONE pose W1 [1,73,4,4] (in-place frame)."""
    ix = rg.ix
    f = {k: float(v[0]) for k, v in AG.features(W1, rg, step_frames=False).items()}
    P = W1[0, :, :3, 3]
    fl = min(P[ix['LeftFoot'], 2], P[ix['RightFoot'], 2])
    f.update(clavicle_joint_gap_cm=float((P[ix['RightShoulder'], 0] - P[ix['LeftShoulder'], 0]) * 100),
             arm_joint_gap_cm=float((P[ix['RightArm'], 0] - P[ix['LeftArm'], 0]) * 100),
             hips_height_cm=float((P[ix['Hips'], 2] - fl) * 100), stance_width_cm=float(abs(P[ix['RightFoot'], 0] - P[ix['LeftFoot'], 0]) * 100),
             stance_depth_cm=float((P[ix['RightFoot'], 1] - P[ix['LeftFoot'], 1]) * 100),
             neck_len_cm=float(np.linalg.norm(P[ix['Head']] - P[ix['Spine3']]) * 100),
             chest_to_hips_cm=float(np.linalg.norm(P[ix['Spine3']] - P[ix['Hips']]) * 100),
             spine3_height_cm=float((P[ix['Spine3'], 2] - fl) * 100), eye_height_cm=float(((P[ix['LeftEye'], 2] + P[ix['RightEye'], 2]) / 2 - fl) * 100),
             upper_arm_cm=float(np.linalg.norm(P[ix['LeftForeArm']] - P[ix['LeftArm']]) * 100),
             forearm_cm=float(np.linalg.norm(P[ix['LeftHand']] - P[ix['LeftForeArm']]) * 100))
    return f


def build():
    os.makedirs(REDO, exist_ok=True)
    names, par, ref, rest = rig()
    rg = rig_for_gate()
    L, Wb, Li = compose_base()
    Wi = fkL(Li)[None]
    base, idle = measures(Wb, rg), measures(Wi, rg)
    ranges, used = collect_ranges()
    # shipped overlay poses = the in-game poses (source pack Pistol1H/2H/Rifle/Feminine/Neutral/Crouch ...): their features widen the ranges
    ov = {}
    for pn in overlay_pose_names():
        try:
            Wp = fkL(overlay_L(pn))[None]
        except Exception:
            continue
        for k, v in AG.features(Wp, rg, step_frames=False).items():
            ov.setdefault(k, []).append(float(v[0]))
    ov_r = {k: dict(min=min(v), max=max(v), n=len(v)) for k, v in ov.items()}
    MC.jdump(dict(base_pose=READY_POSE, base_head_from=IDLE_POSE, base=base, idle=idle, ranges=ranges, overlay_ranges=ov_r, range_clips=used),
             os.path.join(REDO, 'base_reference.json'))
    np.savez(os.path.join(REDO, 'base_pose.npz'), W=Wb, Wi=Wi, L=L, Li=Li, names=np.array(names), parents=np.array(par), rest=rest, fps=30)
    return base, idle, ranges, used


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'show'
    if cmd == 'build':
        m, i, r, u = build()
        print('BASE', json.dumps({k: round(v, 1) for k, v in m.items()}))
        print('IDLE', json.dumps({k: round(v, 1) for k, v in i.items()}))
        print(len(u), 'range clips', sum(n for _, n in u), 'frames')
    else:
        d = json.load(open(os.path.join(REDO, 'base_reference.json')))
        print(json.dumps({k: round(v, 1) for k, v in d['base'].items()}, indent=1))
