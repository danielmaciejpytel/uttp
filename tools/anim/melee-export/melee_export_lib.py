"""Shared helpers of the melee export pipeline (Source\\tools\\anim\\melee-export; numpy only, run with
Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I).

Axes: the clip npz files (dumps and the re-solved clips) are in GAME axes (z up, +Y forward); WolvenKit GLBs are glTF (Y up).
W_game = CI @ W_gltf @ C  (melee_common.C / CI), so W_gltf = C @ W_game @ CI.
Timing: 30 fps, frame i at t = i / 30.  The WolvenKit export writes the Root track of root-motion clips with NORMALISED key times
(0..1); they are rescaled by the clip duration of the serialized .anims (same rule as Source\\tools\\melee\\melee_dump.py, also for
clips <= 1 s, which skel.sample_local leaves alone).
"""
import hashlib
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
VE = os.path.join(ROOT, 'Source', 'tools', 'anim', 'vehicle-entry')
if VE not in sys.path:
    sys.path.insert(0, VE)
import glbanim as G  # noqa: E402
import skel as S  # noqa: E402

FPS = 30.0
C = np.eye(4)
C[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
CI = np.linalg.inv(C)

# SETS = the sets the pipeline knows (make_melee_xl / coverage / registration_evidence loop over it; run_melee_export.ps1 -Sets all = ALL_SETS
# below, the fourteen first-person melee sets).  The baton and the EP1 Gorilla Arms (StrongArms) sets are known since 2026-11-10 (merged from the
# one-handed-blunt / baton and combined-fists shims); the default vanilla-frozen dir has no json for them.
ALL_SETS = ['default_melee_weapon', 'pma_default_melee_weapon', 'pwa_fists', 'pma_fists', 'pwa_katana', 'pma_katana',
            'pwa_knife', 'pma_knife', 'pwa_one_handed_blunt', 'pma_one_handed_blunt', 'pwa_two_handed_blunt', 'pma_two_handed_blunt',
            'pwa_two_handed_hammer', 'pma_two_handed_hammer']
EXTRA_SETS = ['pwa_baton', 'pma_baton', 'pwa_strongarms_melee_fists', 'pma_strongarms_melee_fists']
SETS = ALL_SETS + EXTRA_SETS
RIG_OF_SET = {s: ('man' if s.startswith('pma_') else 'woman') for s in SETS}
# depot path of the vanilla set (game 2.5) per set name
VANILLA_DEPOT = {
    'default_melee_weapon': r'base\animations\weapon\default_melee_weapon.anims',
    'pma_default_melee_weapon': r'base\animations\weapon\pma_default_melee_weapon.anims',
    'pwa_fists': r'base\animations\weapon\melee_fists\pwa_fists.anims',
    'pma_fists': r'base\animations\weapon\melee_fists\pma_fists.anims',
    'pwa_katana': r'base\animations\weapon\katana\pwa_katana.anims',
    'pma_katana': r'base\animations\weapon\katana\pma_katana.anims',
    'pwa_knife': r'base\animations\weapon\knife\pwa_knife.anims',
    'pma_knife': r'base\animations\weapon\knife\pma_knife.anims',
    'pwa_one_handed_blunt': r'base\animations\weapon\one_handed_blunt\pwa_one_handed_blunt.anims',
    'pma_one_handed_blunt': r'base\animations\weapon\one_handed_blunt\pma_one_handed_blunt.anims',
    'pwa_two_handed_blunt': r'base\animations\weapon\two_handed_blunt\pwa_two_handed_blunt.anims',
    'pma_two_handed_blunt': r'base\animations\weapon\two_handed_blunt\pma_two_handed_blunt.anims',
    'pwa_two_handed_hammer': r'base\animations\weapon\two_handed_hammer\pwa_two_handed_hammer.anims',
    'pma_two_handed_hammer': r'base\animations\weapon\two_handed_hammer\pma_two_handed_hammer.anims',
    'pwa_baton': r'base\animations\weapon\baton\pwa_baton.anims',
    'pma_baton': r'base\animations\weapon\baton\pma_baton.anims',
    'pwa_strongarms_melee_fists': r'ep1\animations\cyberware\strongarms\pwa_strongarms_melee_fists.anims',
    'pma_strongarms_melee_fists': r'ep1\animations\cyberware\strongarms\pma_strongarms_melee_fists.anims',
}
# weapon IK helpers: in every vanilla melee set arm_right_ik_weapon sits on WeaponRight and arm_left_ik_weapon on WeaponLeft (0-1.4 cm, same
# rotation; probe 2026-11-09/10).  Sets in WEAPON_HELPER_SETS: the builder puts arm_*_ik_weapon on its weapon joint on every frame farther
# than WEAPON_HELPER_LIMIT_CM (build_melee_set.snap_weapon_helpers) and the verify gates it.  Frames within the limit are not touched.
#   hammer (sliding two-hand grip), one-handed blunt and baton (NPC grip: another local WeaponRight than the game's, helpers up to 7 cm / 46 cm
#   on throws off the joint), and the two default sets (game convention is <= 1.4 cm; the fists-based exports
#   reached 99.8 cm there).
# NOT in the list (their exports stay byte-identical to the accepted / installed ones; the distance is only reported, ik_weapon_vs_weapon_joint_cm):
#   pwa_fists / pma_fists (accepted in game), knife (<= vanilla + 0.27 cm), katana, two_handed_blunt, StrongArms.
WEAPON_HELPERS = {'arm_right_ik_weapon': 'WeaponRight', 'arm_left_ik_weapon': 'WeaponLeft'}
WEAPON_HELPER_SETS = ('pwa_two_handed_hammer', 'pma_two_handed_hammer',
                      'pwa_one_handed_blunt', 'pma_one_handed_blunt', 'pwa_baton', 'pma_baton',
                      'default_melee_weapon', 'pma_default_melee_weapon',
                      'pwa_knife', 'pma_knife', 'pwa_katana', 'pma_katana')
WEAPON_HELPER_LIMIT_CM = 1.5
UTPP_DEPOT_DIR = r'utpp\melee'

# UTPP-only clip names (2026-11-10).  The pipeline works per clip name of the vanilla set; a name that the vanilla set does not have is built ONLY
# when it matches this allowlist, as a clone of its template clip (same events / duration / motion extraction / buffer type; its anim data is
# replaced by the input clip).  Needed for the graph patch melee-parried-graph-20261110, which picks parried_<left|right>_<attack> per attack
# (combo_01..04, final_01..05, heavy_01..02, and the attack types block / safe / crouch / jump / sprint / equip) when the set has both sides.
# Unknown names stay as before: not built, listed in the build report ('not_built'); a clip that reaches the import without a vanilla
# entry is refused by patch_melee_json.py (exit 1) and verify_melee_set.py.  aug_vanilla.py (called by run_melee_export.ps1) makes the clones.
UTPP_CLIP_TEMPLATES = (
    (r'parried_(left|right)_((combo|final|heavy)_\d\d|block|safe|crouch|jump|sprint|equip)', r'parried_\1'),
)


def template_clip(name):
    """template (vanilla clip name) of an allowlisted UTPP-only clip name, else None."""
    import re
    for pat, tmpl in UTPP_CLIP_TEMPLATES:
        m = re.fullmatch(pat, name)
        if m:
            return m.expand(tmpl)
    return None


def canonical_w_nonneg(q):
    """quaternions (..., 4) as (x, y, z, w) with w >= 0 (q and -q are the same rotation).  2026-11-10: uncompressed (SIMD) clips whose keys
    have w < 0 came back from the WolvenKit import with a different rotation on some keys (Male pma_knife throw_start LeftUpLeg 18 deg,
    arm_left_ik 38 deg; compressed clips never showed it), keys with w >= 0 never did."""
    q = np.asarray(q)
    return np.where(q[..., 3:4] < 0, -q, q)


def utpp_name(s):
    return 'utpp_%s.anims' % s


# never taken from the re-solved clip: the root-motion / trajectory joints of the rig (v4 moves Trajectory / reference_joint by cm)
KEEP_VANILLA_BODY = ('Root', 'Trajectory', 'reference_joint')
# hand / foot IK helpers gated against their bone (same set as Source\tools\ai_motion\bike_mount_gate.py / bike_mount_fix.py)
GATE_HELPERS = {}
for _s in ('Left', 'Right'):
    _l = _s.lower()
    for _h in ('arm_%s_ik' % _l, 'arm_%s_ik_offset' % _l, 'Torso_%sArm_Hand_IK_JNT' % _s, 'Torso_World_%sArm_Hand_IK_JNT' % _s):
        GATE_HELPERS[_h] = _s + 'Hand'
    for _h in ('leg_%s_ik' % _l, 'Torso_%sLeg_Foot_IK_JNT' % _s, 'Torso_World_%sLeg_Foot_IK_JNT' % _s):
        GATE_HELPERS[_h] = _s + 'Foot'
HAND_LIMIT_CM = 1.5
FOOT_LIMIT_CM = 1.5
FPP_CAMERA = ('Torso_fppCamera_Control_GRP', 'Torso_fppCamera_Aim_JNT', 'Torso_fppCamera_Target_JNT', 'Torso_fppCamera_UpOffset_GRP', 'Torso_fppCamera_Up_GRP')


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest().upper()


def jdump(obj, path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(obj, f, indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else (o.item() if hasattr(o, 'item') else str(o)))


def anims_json_clips(path):
    """serialized .anims json -> {clip: dict(numFrames, duration, animationType, me (motionExtraction Data or None), buffer type)}"""
    doc = json.load(open(path, encoding='utf-8'))
    rc = doc['Data']['RootChunk']
    out = {}
    for e in rc['animations']:
        a = e['Data']['animation']['Data']
        buf = (a.get('animBuffer') or {}).get('Data') or {}
        out[a['name']['$value']] = dict(numFrames=buf.get('numFrames'), duration=a.get('duration'), animationType=a.get('animationType'),
                                        me=(a.get('motionExtraction') or {}).get('Data'), buffer=buf.get('$type'),
                                        numJoints=buf.get('numJoints'), numTracks=buf.get('numTracks'))
    return out, rc.get('rig', {}).get('DepotPath', {}).get('$value')


def sample_clip(sk, clip, nframes, duration, root_seconds=False):
    """locals (T,N,3) / (T,N,4) of every skin joint at t = i / 30 (glTF axes).  root_seconds=False: WolvenKit export rule (Root of a
    root-motion clip has normalised key times -> rescaled by `duration`); True: our own GLBs (all tracks in seconds)."""
    ai = G.clips(sk.gj)[clip]
    ch = G.channels(sk.gj, sk.binb, ai)
    ex = sk.gj['animations'][ai].get('extras', {})
    rm = ex.get('rootMotionType')
    if not root_seconds and 'Root' in ch and rm not in (None, 'None', 'Unknown'):
        for k in list(ch['Root'].keys()):
            t, v, it = ch['Root'][k]
            if float(t[-1]) <= 1.0001:
                ch['Root'][k] = (t * float(duration), v, it)
    times = np.arange(nframes) / FPS
    lt = np.repeat(sk.rest_t[None], nframes, 0).copy()
    lr = np.repeat(sk.rest_r[None], nframes, 0).copy()
    for b, d in ch.items():
        if b in sk.index:
            i = sk.index[b]
            if 'translation' in d:
                lt[:, i] = G.sample(d['translation'], times, 'translation')
            if 'rotation' in d:
                lr[:, i] = G.sample(d['rotation'], times, 'rotation')
    return lt, S.qcontinuous(lr), ex, set(ch.keys())


def world_mats(sk, lt, lr):
    Wt, Wr = sk.world(lt, lr)
    T, N = Wt.shape[:2]
    M = np.tile(np.eye(4), (T, N, 1, 1))
    M[..., :3, :3] = Wr
    M[..., :3, 3] = Wt
    return M


def quat_angle_deg(q1, q2):
    d = np.abs(np.sum(q1 * q2, axis=-1)) / (np.linalg.norm(q1, axis=-1) * np.linalg.norm(q2, axis=-1))
    return np.degrees(2 * np.arccos(np.clip(d, 0, 1)))


def write_glb_multi(sk, out, clips):
    """GLB with one animation per clip (template = the vanilla GLB skeleton, no meshes).  clips: list of
    (name, times (T,), lt (T,N,3), lr (T,N,4), extras).  Every joint gets translation / rotation / scale (1) tracks, LINEAR."""
    import copy
    gj = copy.deepcopy(sk.gj)
    if gj.get('meshes'):
        raise SystemExit('template has meshes; not supported')
    binb = bytearray()
    accessors, views = [], []

    def add(arr, typ, minmax=False):
        nonlocal binb
        arr = np.ascontiguousarray(arr, dtype=np.float32)
        while len(binb) % 4:
            binb += b'\0'
        off = len(binb)
        binb += arr.tobytes()
        views.append(dict(buffer=0, byteOffset=off, byteLength=arr.nbytes))
        acc = dict(bufferView=len(views) - 1, componentType=5126, count=int(arr.shape[0]), type=typ)
        if minmax:
            acc['min'] = [float(arr.min())]
            acc['max'] = [float(arr.max())]
        accessors.append(acc)
        return len(accessors) - 1

    ibm = gj['skins'][0].get('inverseBindMatrices')
    if ibm is not None:
        gj['skins'][0]['inverseBindMatrices'] = add(G.accessor(sk.gj, sk.binb, ibm), 'MAT4')
    anims = []
    for name, times, lt, lr, extras in clips:
        tin = add(np.asarray(times, dtype=np.float32), 'SCALAR', minmax=True)
        ones = add(np.ones((len(times), 3)), 'VEC3')
        samplers, chans = [], []
        for j, bn in enumerate(sk.names):
            node = sk.node_of[bn]
            for path, acc in (('translation', add(lt[:, j], 'VEC3')), ('rotation', add(lr[:, j], 'VEC4')), ('scale', ones)):
                samplers.append(dict(input=tin, output=acc, interpolation='LINEAR'))
                chans.append(dict(sampler=len(samplers) - 1, target=dict(node=node, path=path)))
        anims.append(dict(name=name, samplers=samplers, channels=chans, extras=extras))
    gj['animations'] = anims
    gj['accessors'] = accessors
    gj['bufferViews'] = views
    gj['buffers'] = [dict(byteLength=len(binb))]
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    G.write_glb(str(out), gj, binb)
