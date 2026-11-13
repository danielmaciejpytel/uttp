"""Batch retarget source pack (UEFN mannequin) FBX clips onto the CP player_woman_skeleton (run inside Blender, background).

Usage:
  blender --background --factory-startup --python retarget_batch.py -- [--root <project>] [--clips a,b,...] [--list <file>]
      [--no-preview] [--fbx-dir D] [--meta-dir D] [--out-dir D] [--preview-dir D] [--report-dir D]
      [--cp-blend F] [--rig-json F] [--bonemap F] [--keep-all-curves] [--rtz]

Defaults (relative to --root, default: the project folder):
  Animations\\sourcepack\\SourcePack_export\\fbx\\*.fbx, Animations\\sourcepack\\SourcePack_export\\meta\\<clip>.json, Animations\\retarget\\cp_player_woman_skeleton.blend,
  Extracted\\game\\json\\vanilla\\player_woman_skeleton.rig.json, Source\\tools\\retarget\\bonemap_source_to_game_woman.json
  -> Animations\\retarget\\out\\<clip>.utppclip.json, Animations\\retarget\\preview\\<clip>.blend, Animations\\retarget\\report.json/.txt
Path options override single defaults. With --out-dir given, --preview-dir defaults to <out-dir>\\preview and --report-dir
to <out-dir> (a custom export never overwrites the default report or previews); without --out-dir both keep the defaults
above. Clip selection: all *.fbx in --fbx-dir, or the union of --clips (comma separated) and --list (text file, one clip
name per line, blank lines and '#' comments ignored, optional .fbx suffix); requested names without an FBX are logged as
UTPP_RETARGET_MISSING and skipped. The clip "rig" field is the --rig-json file name up to the first dot.
Curves: by default only contact_l, contact_r, phase and movedata_speed of the meta json are copied (lower-case names).
--keep-all-curves (opt-in, used for the source pack export) copies every list-valued curve of the meta json unchanged under
its lower-case name and adds retarget.curve_policy / retarget.curve_names to the clip; without it the output is unchanged.
--rtz (opt-in, jump / land / traversal clips; spec Source\\tools\\retarget\\RTZ.txt): the source root's vertical
translation is removed from the in-place pose (Hips keeps only its height relative to the root) and written as the
per-frame track "root_z" (m, CP +Z up, scaled by hip_ratio like Trajectory) plus retarget.root_z; without it the output
is unchanged.

Method (see bonemap notes): all work happens in CP game model space (metres, Z up, character faces +Y).
Source world = FBX armature object (root motion) * pose bone; planar root P = T(x,y,0)*Rz(yaw) is removed, then the
180 deg facing correction C = Rz(180) is applied. Each mapped CP bone gets a world rotation from an aim basis
(direction to the mapped child + stable secondary axis) or from the source world delta (world/inherit). Hips translation =
CP rest + hip_ratio * (source pelvis - source bind pelvis). Two-bone leg IK puts the CP ankles on the scaled source ankle
paths (height offsets kept relative to each rig's bind pose). Trajectory = scaled planar root; Root = identity.
"""
import argparse
import glob
import json
import math
import os
import sys
import time
import traceback

import bpy
from mathutils import Matrix, Quaternion, Vector

EXPORT_COUNT = 73
FPS = 30
FACING_DEG = 180.0
C_ROT = Quaternion((0.0, 0.0, 1.0), math.radians(FACING_DEG))


def parse_args(argv=None):
    if argv is None:
        argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    ap = argparse.ArgumentParser(prog='retarget_batch.py')
    ap.add_argument('--root', default=os.environ.get('UTPP_PROJECT_ROOT') or os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')))
    ap.add_argument('--clips', default='', help='comma-separated clip names (subset of --fbx-dir)')
    ap.add_argument('--list', default='', help='text file with clip names, one per line (# comments); union with --clips')
    ap.add_argument('--no-preview', action='store_true')
    ap.add_argument('--fbx-dir', default=None, help=r'default <root>\Animations\sourcepack\SourcePack_export\fbx')
    ap.add_argument('--meta-dir', default=None, help=r'default <root>\Animations\sourcepack\SourcePack_export\meta')
    ap.add_argument('--out-dir', default=None, help=r'default <root>\Animations\retarget\out')
    ap.add_argument('--preview-dir', default=None, help=r'default <out-dir>\preview if --out-dir given, else <root>\Animations\retarget\preview')
    ap.add_argument('--report-dir', default=None, help=r'default <out-dir> if --out-dir given, else <root>\Animations\retarget')
    ap.add_argument('--cp-blend', default=None, help=r'default <root>\Animations\retarget\cp_player_woman_skeleton.blend')
    ap.add_argument('--rig-json', default=None, help=r'default <root>\Extracted\game\json\vanilla\player_woman_skeleton.rig.json')
    ap.add_argument('--bonemap', default=None, help=r'default <root>\Source\tools\retarget\bonemap_source_to_game_woman.json')
    ap.add_argument('--keep-all-curves', action='store_true',
                    help='copy every list-valued meta curve (lower-case name) instead of only contact_l/r, phase, movedata_speed')
    ap.add_argument('--knee-fix', action='store_true',
                    help='knee fidelity (opt-in): IK knee plane = UE knee plane / carried UE knee hinge per frame, thigh and '
                         'shin rolled so the CP knee bends about its bind lateral hinge only, hip drop also keeps at least the UE '
                         'knee flex up to UTPP_KNEE_MIN_FLEX (default 25 deg [P]; no near-straight CP knee where UE bends)')
    ap.add_argument('--rtz', action='store_true',
                    help='keep the source root Z as the per-frame track root_z (m, hip_ratio scaled) and remove it from Hips (RTZ.txt)')
    a = ap.parse_args(argv)
    r = a.root
    custom_out = a.out_dir is not None
    defaults = {
        'fbx_dir': os.path.join(r, 'Animations', 'sourcepack', 'SourcePack_export', 'fbx'),
        'meta_dir': os.path.join(r, 'Animations', 'sourcepack', 'SourcePack_export', 'meta'),
        'cp_blend': os.path.join(r, 'Animations', 'retarget', 'cp_player_woman_skeleton.blend'),
        'rig_json': os.path.join(r, 'Extracted', 'game', 'json', 'vanilla', 'player_woman_skeleton.rig.json'),
        'bonemap': os.path.join(r, 'Source', 'tools', 'retarget', 'bonemap_source_to_game_woman.json'),
        'out_dir': os.path.join(r, 'Animations', 'retarget', 'out'),
        'preview_dir': os.path.join(a.out_dir, 'preview') if custom_out else os.path.join(r, 'Animations', 'retarget', 'preview'),
        'report_dir': a.out_dir if custom_out else os.path.join(r, 'Animations', 'retarget'),
    }
    for k, v in defaults.items():
        if getattr(a, k) is None:
            setattr(a, k, v)
    return a


def requested_clips(args):
    """None = all clips; otherwise the ordered, de-duplicated union of --clips and --list (may be empty)."""
    if not args.clips and not args.list:
        return None
    names = [c.strip() for c in args.clips.split(',')] if args.clips else []
    if args.list:
        with open(args.list, encoding='utf-8-sig') as fh:
            for line in fh:
                line = line.split('#', 1)[0].strip()
                if line:
                    names.append(line)
    out = []
    for c in names:
        if c.lower().endswith('.fbx'):
            c = c[:-4]
        if c and c not in out:
            out.append(c)
    return out


# ---------------------------------------------------------------- game rig

class GameRig:
    def __init__(self, path):
        rc = json.load(open(path, encoding='utf-8'))['Data']['RootChunk']
        self.names = [b['$value'] for b in rc['boneNames']]
        self.parents = list(rc['boneParentIndexes'])
        self.index = {n: i for i, n in enumerate(self.names)}
        self.lpos, self.lrot, self.lscl = [], [], []
        for t in rc['boneTransforms']:
            p, q, s = t['Translation'], t['Rotation'], t['Scale']
            self.lpos.append(Vector((p['X'], p['Y'], p['Z'])))
            self.lrot.append(Quaternion((q['r'], q['i'], q['j'], q['k'])).normalized())
            self.lscl.append(Vector((s['X'], s['Y'], s['Z'])))
        for i, p in enumerate(self.parents):
            assert p < i, 'parents must precede children'
        self.wpos, self.wrot = fk(self.parents, self.lpos, self.lrot, len(self.names))


def fk(parents, lpos, lrot, count):
    wpos, wrot = [None] * count, [None] * count
    for i in range(count):
        p = parents[i]
        if p < 0:
            wpos[i], wrot[i] = Vector(lpos[i]), Quaternion(lrot[i])
        else:
            wpos[i] = wpos[p] + wrot[p] @ lpos[i]
            wrot[i] = wrot[p] @ lrot[i]
    return wpos, wrot


# ---------------------------------------------------------------- math helpers

def basis(d, s, fallback):
    d = d.normalized()
    so = s - d * s.dot(d)
    if so.length < 1e-6:
        so = fallback - d * fallback.dot(d)
    so.normalize()
    m = Matrix.Identity(3)
    m.col[0], m.col[1], m.col[2] = d, so, d.cross(so)
    return m, s.normalized().dot(d) if s.length > 1e-9 else 1.0


def yaw_of(q):
    """Twist angle of q about world Z."""
    return 2.0 * math.atan2(q.z, q.w)


def ankle_offset_yaw(q):
    """Z twist of the foot delta, independent of foot pitch crossing vertical.
    A 180-degree horizontal swing has no defined Z twist: use identity yaw,
    deterministically per frame (never carry state across frames or clips).
    """
    if not all(math.isfinite(v) for v in (q.z, q.w)) or math.hypot(q.z, q.w) < 1e-6:
        return 0.0
    return 2.0 * math.atan2(q.z, q.w)


def planar_yaw_of_axis(q, axis):
    v = q @ axis
    if math.hypot(v.x, v.y) < 1e-6:
        return None
    return math.atan2(v.y, v.x)


def rot_between(a, b):
    return a.normalized().rotation_difference(b.normalized())


def finite(vals):
    return all(math.isfinite(x) for x in vals)


# ---------------------------------------------------------------- Blender action helpers

def action_fcurves(action):
    fcs = []
    if getattr(action, 'layers', None) is not None and len(action.layers):
        for layer in action.layers:
            for strip in layer.strips:
                for cb in strip.channelbags:
                    fcs.extend(cb.fcurves)
    elif hasattr(action, 'fcurves'):
        fcs.extend(action.fcurves)
    return fcs


def key_pose(obj, action_name, frames_data, bone_names):
    """frames_data[f][i] = (pos Vector, rot Quaternion) as pose bone location/rotation_quaternion."""
    act = bpy.data.actions.new(action_name)
    obj.animation_data_create()
    obj.animation_data.action = act
    if hasattr(act, 'slots') and obj.animation_data.action_slot is None and len(act.slots):
        obj.animation_data.action_slot = act.slots[0]
    n = len(frames_data)
    for i, bn in enumerate(bone_names):
        pb = obj.pose.bones[bn]
        pb.rotation_mode = 'QUATERNION'
        for path, size, getter in (('location', 3, lambda f: frames_data[f][i][0]),
                                   ('rotation_quaternion', 4, lambda f: frames_data[f][i][1])):
            dp = 'pose.bones["%s"].%s' % (bn, path)
            for ax in range(size):
                if hasattr(act, 'fcurve_ensure_for_datablock'):
                    fc = act.fcurve_ensure_for_datablock(obj, dp, index=ax, group_name=bn)
                else:
                    fc = act.fcurves.new(dp, index=ax, action_group=bn)
                fc.keyframe_points.add(n)
                co = []
                for f in range(n):
                    co += [float(f + 1), float(getter(f)[ax])]
                fc.keyframe_points.foreach_set('co', co)
                for kp in fc.keyframe_points:
                    kp.interpolation = 'LINEAR'
                fc.update()
    return act


# ---------------------------------------------------------------- retarget core

class Retargeter:
    def __init__(self, rig, bonemap, src_rest):
        """src_rest[name] = (pos, rot) bind pose in source root space (metres), before facing correction."""
        self.rig = rig
        self.map = bonemap
        self.entries = bonemap['bones']
        self.cp2src = {e['cp']: e['src'] for e in self.entries}
        self.keep_rest = set(bonemap['keep_rest'])
        for e in self.entries:
            assert e['cp'] in rig.index and rig.index[e['cp']] < EXPORT_COUNT, e['cp']
        self.order = sorted(self.entries, key=lambda e: rig.index[e['cp']])
        self.src_rest = {n: (C_ROT @ p, (C_ROT @ q).normalized()) for n, (p, q) in src_rest.items()}
        self.hip_ratio = rig.wpos[rig.index['Hips']].z / self.src_rest['pelvis'][0].z
        self.warnings = []
        # reference alignment R_ref per CP bone (aim evaluated on the source bind pose)
        self.R_ref = {}
        self.rest_align_deg = {}
        for e in self.order:
            R = self._rotation(e, self.src_rest, self.R_ref, rest=True)
            self.R_ref[e['cp']] = R
            self.rest_align_deg[e['cp']] = round(math.degrees(R.angle), 2)
        # leg IK offsets (in-place frame)
        self.legs = []
        for leg in bonemap['ik_legs']:
            cp_foot = rig.wpos[rig.index[leg['foot']]]
            src_foot = self.src_rest[leg['src_foot']][0]
            cp_toe = rig.wpos[rig.index[leg['toe']]]
            src_toe = self.src_rest[leg['src_toe']][0]
            self.legs.append(dict(leg, off=cp_foot - src_foot * self.hip_ratio,
                                  toe_off=cp_toe - src_toe * self.hip_ratio,
                                  foot_len=(cp_toe - cp_foot).length,
                                  src_ankle_rest_z=src_foot.z * self.hip_ratio))
        # knee fix (--knee-fix): bind lateral axis (left hip minus right hip) = knee hinge of both rigs; the UE hinge
        # is carried per frame by the source calf, the CP hinge is the same lateral axis in the CP thigh / shin bind frames.
        self.knee_fix = False
        self.KNEE_BLEND_LO, self.KNEE_BLEND_HI = 5.0, 20.0   # deg source flex: hinge axis below LO, UE plane above HI [P]
        self.KNEE_MIN_FLEX = float(os.environ.get('UTPP_KNEE_MIN_FLEX', '25'))  # deg [P]: flex floor kept from UE (hip drop)
        L_cp = (rig.wpos[rig.index['LeftUpLeg']] - rig.wpos[rig.index['RightUpLeg']]).normalized()
        self.lat_src = (self.src_rest['thigh_l'][0] - self.src_rest['thigh_r'][0]).normalized()
        for leg in self.legs:
            s = leg['src_foot'][-1]
            leg['src_hip'], leg['src_knee'] = 'thigh_' + s, 'calf_' + s
            A0, B0, C0 = (rig.wpos[rig.index[leg[k]]] for k in ('upleg', 'leg', 'foot'))
            leg['B_th_rest_T'] = basis(B0 - A0, L_cp, Vector((0, 1, 0)))[0].transposed()
            leg['B_sh_rest_T'] = basis(C0 - B0, L_cp, Vector((0, 1, 0)))[0].transposed()
            leg['src_len'] = ((self.src_rest[leg['src_knee']][0] - self.src_rest[leg['src_hip']][0]).length +
                              (self.src_rest[leg['src_foot']][0] - self.src_rest[leg['src_knee']][0]).length)
        self.last_knee = {}

    def _knee_source(self, leg, pose):
        """UE knee for --knee-fix: (desired plane normal n, UE flex deg, UE extension hip-ankle / leg length).
        n = UE plane normal (thigh x shin) above KNEE_BLEND_HI source flex, the UE calf's carried bind lateral axis
        below KNEE_BLEND_LO (straight leg: plane undefined), smooth blend between; both continuous per frame."""
        H, K, F = pose[leg['src_hip']][0], pose[leg['src_knee']][0], pose[leg['src_foot']][0]
        sk = leg['src_knee']
        h = pose[sk][1] @ (self.src_rest[sk][1].inverted() @ self.lat_src)
        t, sh = K - H, F - K
        flex = math.degrees(t.angle(sh)) if t.length > 1e-9 and sh.length > 1e-9 else 0.0
        npl = t.cross(sh)
        x = max(0.0, min(1.0, (flex - self.KNEE_BLEND_LO) / (self.KNEE_BLEND_HI - self.KNEE_BLEND_LO)))
        w = x * x * (3.0 - 2.0 * x)
        n = h.normalized()
        if npl.length > 1e-9 and w > 0.0:
            npl = npl.normalized()
            if npl.dot(n) < 0.0:   # hyperextended source knee: keep the hinge orientation
                npl = -npl
            n = n.lerp(npl, w)
            if n.length < 1e-9:
                n = npl
            n = n.normalized()
        ext = (F - H).length / leg['src_len'] if leg['src_len'] > 1e-9 else 1.0
        return n, flex, ext

    # Heel-raise height (m, scaled source ankle above its bind height) over which the toe becomes the pivot.
    TOE_PIVOT_HEIGHT = float(os.environ.get('UTPP_TOE_PIVOT_HEIGHT', '0'))  # <= 0 disables the toe pivot (default: measured worse)

    def _ankle_target(self, leg, pose):
        """IK target for the CP ankle. Flat foot: follow the scaled source ankle. Heel raised (toe pivot): keep the
        toe on the scaled source toe and place the ankle at the CP foot length along the source foot direction, so
        the different CP foot length does not make the planted toe slide. Blended by heel-raise height."""
        src_p, src_q = pose[leg['src_foot']]
        dyaw = ankle_offset_yaw(src_q @ self.src_rest[leg['src_foot']][1].inverted())
        yaw_q = Quaternion((0, 0, 1), dyaw)
        off, toff = leg['off'], leg['toe_off']
        ankle = src_p * self.hip_ratio + yaw_q @ Vector((off.x, off.y, 0)) + Vector((0, 0, off.z))
        toe = pose[leg['src_toe']][0] * self.hip_ratio + yaw_q @ Vector((toff.x, toff.y, 0)) + Vector((0, 0, toff.z))
        d = ankle - toe
        if d.length < 1e-6:
            return ankle
        ankle_from_toe = toe + d.normalized() * leg['foot_len']
        if self.TOE_PIVOT_HEIGHT <= 0.0:
            return ankle
        x = max(0.0, min(1.0, (src_p.z * self.hip_ratio - leg['src_ankle_rest_z']) / self.TOE_PIVOT_HEIGHT))
        w = x * x * (3.0 - 2.0 * x)
        return ankle.lerp(ankle_from_toe, w)

    def _src_vec_pair(self, sec, pose):
        a, b = self.cp2src[sec['a']], self.cp2src[sec['b']]
        return pose[b][0] - pose[a][0]

    def _rotation(self, e, pose, R_cur, rest=False):
        """World rotation delta R (CP rest world -> CP posed world) for entry e, given source pose (corrected, in place)."""
        rig, cp, src = self.rig, e['cp'], e['src']
        mode = e['mode']
        q_src, q_src_rest = pose[src][1], self.src_rest[src][1]
        dq = q_src @ q_src_rest.inverted()
        if mode == 'world':
            return dq
        if mode == 'inherit':
            parent_cp = rig.names[rig.parents[rig.index[cp]]]
            return dq @ self.R_ref.get(parent_cp, Quaternion())
        # aim
        aim_cp = e['aim']
        aim_src = self.cp2src[aim_cp]
        d_cp = rig.wpos[rig.index[aim_cp]] - rig.wpos[rig.index[cp]]
        d_src = pose[aim_src][0] - pose[src][0]
        sec = e['secondary']
        t = sec['type']
        if t == 'carry_axis':
            ax = Vector(sec['axis'])
            sb = sec.get('src_bone', src)
            s_cp = ax
            s_src = pose[sb][1] @ (self.src_rest[sb][1].inverted() @ ax)
        elif t == 'live_pair':
            s_cp = rig.wpos[rig.index[sec['b']]] - rig.wpos[rig.index[sec['a']]]
            s_src = self._src_vec_pair(sec, pose)
        elif t == 'carry_pair':
            s_cp = rig.wpos[rig.index[sec['b']]] - rig.wpos[rig.index[sec['a']]]
            s_src = dq @ self._src_vec_pair(sec, self.src_rest)
        else:
            raise ValueError(t)
        fb = Vector((0, 0, 1)) if abs(d_cp.normalized().z) < 0.9 else Vector((0, 1, 0))
        B_cp, c1 = basis(d_cp, s_cp, fb)
        B_src, c2 = basis(d_src, s_src, dq @ fb)
        if rest and max(abs(c1), abs(c2)) > 0.95:
            self.warnings.append('%s: secondary nearly parallel to aim (cos cp %.3f src %.3f)' % (cp, c1, c2))
        return (B_src @ B_cp.transposed()).to_quaternion().normalized()

    def solve_frame(self, pose, contact=None, forced_drop=None):
        """pose[src_name] = (pos, rot) corrected, in-place. Returns (wpos, wrot, ik_clamped) for bones 0..72 in model space."""
        rig = self.rig
        R = {}
        for e in self.order:
            R[e['cp']] = self._rotation(e, pose, R)
        wrot_mapped = {rig.index[cp]: (r @ rig.wrot[rig.index[cp]]).normalized() for cp, r in R.items()}
        hips_i = rig.index['Hips']
        p_hips = rig.wpos[hips_i] + (pose['pelvis'][0] - self.src_rest['pelvis'][0]) * self.hip_ratio

        def run_fk():
            wpos, wrot = [None] * EXPORT_COUNT, [None] * EXPORT_COUNT
            for i in range(EXPORT_COUNT):
                p = rig.parents[i]
                if p < 0 or i == 1:  # Root, Trajectory: identity in the in-place model frame
                    wpos[i], wrot[i] = Vector((0, 0, 0)), Quaternion()
                    continue
                wpos[i] = p_hips.copy() if i == hips_i else wpos[p] + wrot[p] @ rig.lpos[i]
                wrot[i] = wrot_mapped[i] if i in wrot_mapped else wrot[p] @ rig.lrot[i]
            return wpos, wrot

        wpos, wrot = run_fk()
        # Hip drop: where a source ankle target is beyond the CP leg's reach (longer source pack stride relative to
        # CP leg length), lower the hips vertically just enough for the farthest leg to reach, instead of
        # straightening the knee and leaving the foot short (which showed up as foot sliding).
        drop = 0.0
        for leg in self.legs:
            iu, il, ifo = rig.index[leg['upleg']], rig.index[leg['leg']], rig.index[leg['foot']]
            T = self._ankle_target(leg, pose)
            reach = ((wpos[il] - wpos[iu]).length + (wpos[ifo] - wpos[il]).length) * 0.9995
            if self.knee_fix:   # CP knee flex >= min(UE flex, KNEE_MIN_FLEX): no near-straight CP knee where UE bends
                la, lb = (wpos[il] - wpos[iu]).length, (wpos[ifo] - wpos[il]).length
                cf = math.cos(math.radians(min(self._knee_source(leg, pose)[1], self.KNEE_MIN_FLEX)))
                reach = min(reach, math.sqrt(max(la * la + lb * lb + 2.0 * la * lb * cf, 0.0)) * 0.9995)
            AT = T - wpos[iu]
            h2 = AT.x * AT.x + AT.y * AT.y
            if AT.length > reach and h2 < reach * reach and AT.z < 0:
                drop = max(drop, -math.sqrt(reach * reach - h2) - AT.z + 1e-4)
        self.last_required_drop = drop
        if forced_drop is not None:  # temporally smoothed drop from the first pass (never below the requirement)
            drop = max(drop, forced_drop)
        if drop > 0.0:
            p_hips = p_hips - Vector((0, 0, drop))
            wpos, wrot = run_fk()
        self.last_hip_drop = drop
        clamped, over = 0, 0.0
        for leg in self.legs:
            iu, il, ifo = rig.index[leg['upleg']], rig.index[leg['leg']], rig.index[leg['foot']]
            A, B, Cc = wpos[iu], wpos[il], wpos[ifo]
            T = self._ankle_target(leg, pose)
            a, b = (B - A).length, (Cc - B).length
            AT = T - A
            dist = AT.length
            dmax, dmin = (a + b) * 0.9995, abs(a - b) + 1e-4
            if dist > dmax or dist < dmin:
                clamped += 1
                over = max(over, dist - dmax, dmin - dist)
                dist = min(max(dist, dmin), dmax)
            u = AT.normalized()
            pole = (B - A) - u * (B - A).dot(u)
            if self.knee_fix:
                n, sflex, _ = self._knee_source(leg, pose)
                n_perp = n - u * n.dot(u)
                if n_perp.length > 1e-6:
                    pole = u.cross(n_perp.normalized())   # knee plane normal (B-A) x (T-B) = n
                    cos_a = max(-1.0, min(1.0, (a * a + dist * dist - b * b) / (2 * a * dist)))
                    sin_a = math.sqrt(max(0.0, 1 - cos_a * cos_a))
                    B2 = A + u * (cos_a * a) + pole * (sin_a * a)
                    T2 = A + u * dist
                    nn = n_perp.normalized()
                    wrot_mapped[iu] = (basis(B2 - A, nn, Vector((0, 1, 0)))[0] @ leg['B_th_rest_T']).to_quaternion().normalized() @ rig.wrot[iu]
                    wrot_mapped[il] = (basis(T2 - B2, nn, Vector((0, 1, 0)))[0] @ leg['B_sh_rest_T']).to_quaternion().normalized() @ rig.wrot[il]
                    wrot_mapped[iu].normalize()
                    wrot_mapped[il].normalize()
                    self.last_knee[leg['upleg']] = sflex
                    continue
            if pole.length < 1e-6:
                pole = wrot[iu] @ Vector((0, 1, 0))
                pole = pole - u * pole.dot(u)
            v = pole.normalized()
            cos_a = max(-1.0, min(1.0, (a * a + dist * dist - b * b) / (2 * a * dist)))
            sin_a = math.sqrt(max(0.0, 1 - cos_a * cos_a))
            B2 = A + u * (cos_a * a) + v * (sin_a * a)
            r1 = rot_between(B - A, B2 - A)
            C1 = A + r1 @ (Cc - A)
            T2 = A + u * dist
            r2 = rot_between(C1 - B2, T2 - B2)
            wrot_mapped[iu] = (r1 @ wrot_mapped[iu]).normalized()
            wrot_mapped[il] = (r2 @ r1 @ wrot_mapped[il]).normalized()
        wpos, wrot = run_fk()
        return wpos, wrot, clamped, over


def smooth_hip_drop(req, looped, radius=4):
    """Max-filter then triangle-blur a per-frame hip drop sequence; result >= req at every frame."""
    n = len(req)
    if n == 0 or max(req) <= 0.0:
        return [0.0] * n
    if looped and n > 2:
        # Loop clips repeat frame 0 as the last frame: smooth over the period n-1 and copy the result so the
        # first and last frames stay identical (the baker's loop detection relies on it).
        period = req[:-1]
        m = len(period)
        dil = [max(period[(i + k) % m] for k in range(-radius, radius + 1)) for i in range(m)]
        weights = [radius + 1 - abs(k) for k in range(-radius, radius + 1)]
        wsum = float(sum(weights))
        blur = [sum(w * dil[(i + k) % m] for w, k in zip(weights, range(-radius, radius + 1))) / wsum for i in range(m)]
        body = [max(b, r) for b, r in zip(blur, period)]
        body.append(max(body[0], req[-1]))
        body[0] = body[-1]
        return body

    def at(seq, i):
        if looped:
            return seq[i % n]
        return seq[min(max(i, 0), n - 1)]

    dil = [max(at(req, i + k) for k in range(-radius, radius + 1)) for i in range(n)]
    weights = [radius + 1 - abs(k) for k in range(-radius, radius + 1)]
    wsum = float(sum(weights))
    blur = [sum(w * at(dil, i + k) for w, k in zip(weights, range(-radius, radius + 1))) / wsum for i in range(n)]
    return [max(b, r) for b, r in zip(blur, req)]


def to_local(rig, wpos, wrot, traj):
    """Game local transforms for bones 0..72; traj = (pos, rot) of Trajectory under Root."""
    out = []
    for i in range(EXPORT_COUNT):
        p = rig.parents[i]
        if i == 0:
            pos, rot = Vector((0, 0, 0)), Quaternion()
        elif i == 1:
            pos, rot = traj
        elif p == 0 or p < 0:
            pos, rot = wpos[i].copy(), wrot[i].copy()
        else:
            pos = rig.lpos[i].copy() if rig.names[i] != 'Hips' else wpos[i].copy()
            rot = (wrot[p].inverted() @ wrot[i]).normalized()
        out.append((pos, rot.normalized(), rig.lscl[i]))
    return out


# ---------------------------------------------------------------- per clip

def sample_source(src_obj, bone_names, n_frames, tilt_out=None):
    """tilt_out (list, --rtz diagnostics): per frame angle (deg) between the root's up axis and world Z (not transferred)."""
    scene = bpy.context.scene
    frames = []
    for f in range(n_frames):
        scene.frame_set(f + 1)
        O = src_obj.matrix_world.copy()
        root_loc = O.translation.copy()
        root_q = O.to_3x3().normalized().to_quaternion()
        yaw = yaw_of(root_q)
        if tilt_out is not None:
            tilt_out.append(math.degrees(math.acos(max(-1.0, min(1.0, (root_q @ Vector((0, 0, 1))).z)))))
        bones = {}
        for bn in bone_names:
            m = O @ src_obj.pose.bones[bn].matrix
            loc, rot, _ = m.decompose()
            bones[bn] = (loc, rot.normalized())
        frames.append((root_loc, yaw, bones))
    return frames


def fbx_key_info(src_obj, n_frames):
    act = src_obj.animation_data.action if src_obj.animation_data else None
    if act is None:
        return {'keys': 0, 'ok': False}
    fcs = action_fcurves(act)
    nk = max(len(fc.keyframe_points) for fc in fcs)
    dup_ok = True
    for fc in fcs:
        kp = fc.keyframe_points
        if len(kp) >= 2 and abs(kp[-1].co[1] - kp[-2].co[1]) > 1e-4 * max(1.0, abs(kp[-1].co[1])):
            dup_ok = False
            break
    first = min(fc.keyframe_points[0].co[0] for fc in fcs if len(fc.keyframe_points))
    return {'keys': nk, 'first_frame': first, 'expected_keys': n_frames + 1,
            'last_key_duplicates_previous': dup_ok, 'ok': nk == n_frames + 1 and first == 1.0}


def foot_slide_matched(cp_ankle, cp_toe, src_ankle, src_toe, contact):
    """Per contact frame pick the point (ankle or toe) that is planted in the SOURCE (slower there) and measure the
    CP speed of that same point; reports CP and source speeds of the planted point (cm/s)."""
    cp_v, src_v = [], []
    for i in range(1, len(cp_ankle)):
        if contact is not None and contact[i] > 0.5 and contact[i - 1] > 0.5:
            sa = (src_ankle[i] - src_ankle[i - 1]).length
            st = (src_toe[i] - src_toe[i - 1]).length
            if st < sa:
                cp_v.append((cp_toe[i] - cp_toe[i - 1]).length * FPS * 100.0)
                src_v.append(st * FPS * 100.0)
            else:
                cp_v.append((cp_ankle[i] - cp_ankle[i - 1]).length * FPS * 100.0)
                src_v.append(sa * FPS * 100.0)
    if not cp_v:
        return {'frames': 0, 'cp_mean_cm_s': None, 'src_mean_cm_s': None, 'excess_mean_cm_s': None}
    return {'frames': len(cp_v), 'cp_mean_cm_s': round(sum(cp_v) / len(cp_v), 3), 'src_mean_cm_s': round(sum(src_v) / len(src_v), 3),
            'excess_mean_cm_s': round((sum(cp_v) - sum(src_v)) / len(cp_v), 3), 'cp_max_cm_s': round(max(cp_v), 3)}


def foot_slide(positions, contact, toes=None):
    """positions: list of Vector (m); returns mean/max cm/s over consecutive contact frames.
    With toes given, the per-frame speed is min(ankle, toe) (pivot metric: heel or toe planted)."""
    v = []
    for i in range(1, len(positions)):
        if contact is not None and contact[i] > 0.5 and contact[i - 1] > 0.5:
            sp = (positions[i] - positions[i - 1]).length
            if toes is not None:
                sp = min(sp, (toes[i] - toes[i - 1]).length)
            v.append(sp * FPS * 100.0)
    if not v:
        return {'frames': 0, 'mean_cm_s': None, 'max_cm_s': None}
    return {'frames': len(v), 'mean_cm_s': round(sum(v) / len(v), 3), 'max_cm_s': round(max(v), 3)}


def fk_world_from_clip(rig, frame_bones):
    lpos = [Vector(b[0:3]) for b in frame_bones]
    lrot = [Quaternion((b[6], b[3], b[4], b[5])) for b in frame_bones]
    return fk(rig.parents, lpos, lrot, EXPORT_COUNT)


def process_clip(args, rig, bonemap, clip):
    t0 = time.time()
    res = {'clip': clip, 'status': 'failed'}
    meta = json.load(open(os.path.join(args.meta_dir, clip + '.json'), encoding='utf-8'))
    n = int(meta['num_frames'])
    bpy.ops.wm.open_mainfile(filepath=args.cp_blend, load_ui=False)
    cp_obj = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
    before = set(bpy.data.objects)
    bpy.ops.import_scene.fbx(filepath=os.path.join(args.fbx_dir, clip + '.fbx'),
                             automatic_bone_orientation=False, use_anim=True)
    src_obj = next(o for o in bpy.data.objects if o not in before and o.type == 'ARMATURE')
    scene = bpy.context.scene
    scene.render.fps, scene.render.fps_base = FPS, 1.0
    res['fbx_keys'] = fbx_key_info(src_obj, n)

    # source bind pose in root space (metres)
    S = Matrix.Diagonal((0.01, 0.01, 0.01, 1.0))
    needed = sorted(set(e['src'] for e in bonemap['bones']) | {'spine_05'})
    src_rest = {}
    for bn in needed:
        loc, rot, _ = (S @ src_obj.data.bones[bn].matrix_local).decompose()
        src_rest[bn] = (loc, rot.normalized())
    rt = Retargeter(rig, bonemap, src_rest)
    rt.knee_fix = bool(getattr(args, 'knee_fix', False))
    ratio = rt.hip_ratio

    rtz = bool(getattr(args, 'rtz', False))
    root_tilt = [] if rtz else None
    frames_src = sample_source(src_obj, needed, n, root_tilt)
    curves_src = {k.lower(): v for k, v in meta.get('curves', {}).items()}
    contact = {k: curves_src.get(k) for k in ('contact_l', 'contact_r')}

    frames_out, cp_world_feet, src_world_feet, ik_clamped = [], {'l': [], 'r': [], 'ltoe': [], 'rtoe': []}, {'l': [], 'r': [], 'ltoe': [], 'rtoe': []}, 0
    prev_q = [None] * EXPORT_COUNT
    ik_over = 0.0
    traj_list = []
    hip_drops = []
    # Pass 1: required hip drop per frame (see Retargeter.solve_frame), then smooth it in time so the hips
    # never pop: dilate (max filter) and blur with a triangle kernel, keeping at least the requirement.
    # --rtz: the removed root offset also contains the root Z (pose relative to the full root translation).
    req = []
    for f, (root_loc, yaw, bones) in enumerate(frames_src):
        P_inv_rot = Quaternion((0, 0, 1), -yaw)
        planar = Vector((root_loc.x, root_loc.y, 0.0))
        origin = Vector((root_loc.x, root_loc.y, root_loc.z)) if rtz else planar
        pose1 = {bn: (C_ROT @ (P_inv_rot @ (p - origin)), (C_ROT @ P_inv_rot @ q).normalized()) for bn, (p, q) in bones.items()}
        rt.solve_frame(pose1)
        req.append(rt.last_required_drop)
    smooth_drop = smooth_hip_drop(req, looped='_Loop' in clip)
    nan = False
    root_z = [] if rtz else None
    for f, (root_loc, yaw, bones) in enumerate(frames_src):
        P_inv_rot = Quaternion((0, 0, 1), -yaw)
        planar = Vector((root_loc.x, root_loc.y, 0.0))
        origin = Vector((root_loc.x, root_loc.y, root_loc.z)) if rtz else planar
        if rtz:
            root_z.append(round(root_loc.z * ratio, 7))
            if not finite([root_loc.z]):
                nan = True
        pose = {}
        for bn, (p, q) in bones.items():
            pose[bn] = (C_ROT @ (P_inv_rot @ (p - origin)), (C_ROT @ P_inv_rot @ q).normalized())
        wpos, wrot, cl, ov = rt.solve_frame(pose, forced_drop=smooth_drop[f])
        hip_drops.append(getattr(rt, 'last_hip_drop', 0.0))
        ik_clamped += cl
        ik_over = max(ik_over, ov)
        traj = ((C_ROT @ planar) * ratio, Quaternion((0, 0, 1), yaw))
        traj_list.append(traj)
        loc = to_local(rig, wpos, wrot, traj)
        row = []
        for i, (p, q, s) in enumerate(loc):
            if prev_q[i] is None:
                if q.w < 0:
                    q = -q
            elif prev_q[i].dot(q) < 0:
                q = -q
            prev_q[i] = q
            vals = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, s.x, s.y, s.z]
            if not finite(vals):
                nan = True
            row.append([round(v, 7) for v in vals])
        frames_out.append({'bones': row})
        for side, bn, tb in (('l', 'foot_l', 'ball_l'), ('r', 'foot_r', 'ball_r')):
            src_world_feet[side].append(bones[bn][0] * ratio)
            src_world_feet[side + 'toe'].append(bones[tb][0] * ratio)

    curves = {}
    curve_names = None
    if getattr(args, 'keep_all_curves', False):
        # opt-in: every list-valued meta curve, unchanged, lower-case name (first spelling wins on a case clash)
        curve_names, dup = {}, []
        for k, v in meta.get('curves', {}).items():
            if not isinstance(v, list):
                continue
            lk = k.lower()
            if lk in curve_names:
                dup.append(k)
                continue
            curve_names[lk] = k
            curves[lk] = v
        res['curves_kept'] = len(curves)
        res['curves_length_mismatch'] = sorted(k for k, v in curves.items() if len(v) != n)
        if dup:
            res['curves_case_duplicates'] = dup
    else:
        for k in ('contact_l', 'contact_r', 'phase', 'movedata_speed'):
            if k in curves_src:
                curves[k] = curves_src[k]
    clip_json = {
        'format': 'utppclip', 'version': 1, 'source_clip': clip, 'fps': FPS, 'num_frames': n,
        'bone_names': rig.names[:EXPORT_COUNT], 'rig': os.path.basename(args.rig_json).split('.')[0], 'rig_bone_count': len(rig.names),
        'frames': frames_out, 'curves': curves, 'notifies': meta.get('notifies', []),
        'retarget': {'hip_ratio': round(ratio, 6), 'facing_correction_deg': FACING_DEG,
                     'method': 'aim+secondary-axis world-space retarget from matched bind pose; planar root -> Trajectory; '
                               'hips translation scaled by hip ratio; two-bone leg IK to scaled source ankle paths',
                     'bonemap': os.path.basename(args.bonemap),
                     'notes': 'Local transforms relative to game parents (rig.json). Quaternion xyzw. Metres. '
                              'Curves copied unchanged from UE meta (movedata_speed in source cm/s, unscaled). '
                              'Bones 73..151 not exported (keep rest).'}}
    if rt.knee_fix:
        clip_json['retarget']['knee_fix'] = {
            'version': 2, 'blend_flex_deg': [rt.KNEE_BLEND_LO, rt.KNEE_BLEND_HI],
            'method': 'IK knee plane normal = UE thigh x shin (flex >= hi) / UE calf carried bind lateral axis (flex <= lo), '
                      'thigh and shin aimed with the CP bind lateral axis on that normal (hinge-only knee); hip drop also '
                      'keeps the CP knee flex >= min(UE flex, min_flex_deg) (hip-ankle <= law-of-cosines length)',
            'min_flex_deg': rt.KNEE_MIN_FLEX}
    if curve_names is not None:
        clip_json['retarget']['curve_policy'] = 'all meta curves (--keep-all-curves)'
        clip_json['retarget']['curve_names'] = curve_names
    if rtz:
        clip_json['retarget']['root_z'] = {
            'spec': 'Source/tools/retarget/RTZ.txt', 'version': 1, 'track': 'root_z', 'units': 'm',
            'axis': 'CP model space +Z (up), same frame as Trajectory', 'scale': round(ratio, 6),
            'definition': 'root_z[f] = hip_ratio * source root Z (UE root bone = FBX armature object, world, m) at frame f, '
                          'absolute (0 = floor level of the source animation origin); the in-place pose (Hips and every '
                          'bone) is relative to the full root translation, so Hips holds only its height above the root',
            'not_transferred': 'root pitch/roll (diagnostic retarget report rtz.root_tilt_max_deg)',
            'trajectory': 'unchanged: planar root XY * hip_ratio and yaw'}
        clip_json['root_z'] = root_z
    os.makedirs(args.out_dir, exist_ok=True)
    out_path = os.path.join(args.out_dir, clip + '.utppclip.json')
    with open(out_path, 'w', encoding='utf-8') as fh:
        json.dump(clip_json, fh, separators=(',', ':'))

    # ---- reload written clip; FK in game space (independent of solver state)
    clip_back = json.load(open(out_path, encoding='utf-8'))
    fk_frames = [fk_world_from_clip(rig, fr['bones']) for fr in clip_back['frames']]

    # ---- key Blender target (game local -> pose basis) and compare evaluated world with FK
    arm = cp_obj.data
    names = rig.names[:EXPORT_COUNT]
    rest_rel_inv = {}
    for bn in names:
        b = arm.bones[bn]
        rr = b.matrix_local if b.parent is None else b.parent.matrix_local.inverted() @ b.matrix_local
        rest_rel_inv[bn] = rr.inverted()
    keyed = []
    for fr in clip_back['frames']:
        row = []
        for i, bn in enumerate(names):
            v = fr['bones'][i]
            L = Matrix.LocRotScale(Vector(v[0:3]), Quaternion((v[6], v[3], v[4], v[5])), None)
            basis_m = rest_rel_inv[bn] @ L
            loc, rot, _ = basis_m.decompose()
            row.append((loc, rot))
        keyed.append(row)
    # sign continuity of keyed quaternions (Blender interpolation)
    for i in range(EXPORT_COUNT):
        for f in range(1, len(keyed)):
            if keyed[f][i][1].dot(keyed[f - 1][i][1]) < 0:
                keyed[f][i] = (keyed[f][i][0], -keyed[f][i][1])
    key_pose(cp_obj, 'UTPP_' + clip, keyed, names)
    scene.frame_start, scene.frame_end = 1, n
    max_pos, max_rot, worst = 0.0, 0.0, None
    for f in range(n):
        scene.frame_set(f + 1)
        wpos, wrot = fk_frames[f]
        mw = cp_obj.matrix_world
        for i, bn in enumerate(names):
            m = mw @ cp_obj.pose.bones[bn].matrix
            dp = (m.translation - wpos[i]).length
            dr = max(abs(a - b) for ra, rb in zip(m.to_3x3().normalized(), wrot[i].to_matrix()) for a, b in zip(ra, rb))
            if dp > max_pos:
                max_pos = dp
            if dr > max_rot:
                max_rot, worst = dr, (f, bn)
    res['check_fk_vs_blender'] = {'frames': n, 'max_pos_err_m': max_pos, 'max_rot_err': max_rot,
                                  'worst': worst, 'pass': max_pos < 1e-4 and max_rot < 1e-4}

    # ---- foot sliding in game world (Trajectory * model)
    idx = {'l': rig.index['LeftFoot'], 'r': rig.index['RightFoot'],
           'ltoe': rig.index['LeftToeBase'], 'rtoe': rig.index['RightToeBase']}
    for f in range(n):
        wpos, _ = fk_frames[f]
        tv = clip_back['frames'][f]['bones'][1]
        tp, tq = Vector(tv[0:3]), Quaternion((tv[6], tv[3], tv[4], tv[5]))
        if rtz:  # game world = Trajectory * model + root_z (source feet keep the root Z)
            tp = tp + Vector((0.0, 0.0, clip_back['root_z'][f]))
        for k, i in idx.items():
            cp_world_feet[k].append(tp + tq @ wpos[i])
    slide = {}
    for side in ('l', 'r'):
        c = contact.get('contact_' + side)
        slide[side] = {'cp': foot_slide(cp_world_feet[side], c),
                       'source_scaled': foot_slide(src_world_feet[side], c),
                       'matched': foot_slide_matched(cp_world_feet[side], cp_world_feet[side + 'toe'], src_world_feet[side], src_world_feet[side + 'toe'], c),
                       'cp_pivot': foot_slide(cp_world_feet[side], c, cp_world_feet[side + 'toe']),
                       'source_scaled_pivot': foot_slide(src_world_feet[side], c, src_world_feet[side + 'toe'])}
    means = [slide[s]['cp']['mean_cm_s'] for s in ('l', 'r') if slide[s]['cp']['mean_cm_s'] is not None]
    excess = [slide[s]['cp']['mean_cm_s'] - slide[s]['source_scaled']['mean_cm_s'] for s in ('l', 'r')
              if slide[s]['cp']['mean_cm_s'] is not None]
    pivot = [slide[s]['cp_pivot']['mean_cm_s'] for s in ('l', 'r') if slide[s]['cp_pivot']['mean_cm_s'] is not None]
    res['foot_slide'] = slide
    res['foot_slide_flag'] = bool(means and max(means) > 5.0)          # spec metric: ankle speed during contact
    res['foot_slide_pivot_flag'] = bool(pivot and max(pivot) > 5.0)     # min(ankle, toe) speed during contact
    res['foot_slide_excess_vs_source_flag'] = bool(excess and max(excess) > 5.0)

    # ---- root displacement check
    r0, r1 = frames_src[0][0], frames_src[-1][0]
    src_disp = C_ROT @ Vector((r1.x - r0.x, r1.y - r0.y, 0.0)) * ratio
    t0v, t1v = clip_back['frames'][0]['bones'][1], clip_back['frames'][-1]['bones'][1]
    traj_disp = Vector(t1v[0:3]) - Vector(t0v[0:3])
    yaw_src = math.degrees(frames_src[-1][1] - frames_src[0][1])
    q0 = Quaternion((t0v[6], t0v[3], t0v[4], t0v[5]))
    q1 = Quaternion((t1v[6], t1v[3], t1v[4], t1v[5]))
    yaw_traj = math.degrees(yaw_of(q0.inverted() @ q1))
    yerr = (yaw_traj - yaw_src + 180.0) % 360.0 - 180.0
    res['root_motion'] = {'source_disp_scaled_m': [round(x, 5) for x in src_disp],
                          'source_disp_raw_m': [round(r1.x - r0.x, 5), round(r1.y - r0.y, 5)],
                          'trajectory_disp_m': [round(x, 5) for x in traj_disp],
                          'disp_err_m': round((src_disp - traj_disp).length, 7),
                          'source_yaw_deg': round(yaw_src, 3), 'trajectory_yaw_deg': round(yaw_traj, 3),
                          'pass': (src_disp - traj_disp).length < 1e-4 and abs(yerr) < 0.01}
    if rtz:
        src_z = [fr[0].z for fr in frames_src]
        hi = rig.index['Hips']
        hips_dev = [fr['bones'][hi][2] - rig.lpos[hi].z for fr in clip_back['frames']]  # Hips parent = Root (identity)
        res['rtz'] = {'scale': round(ratio, 6),
                      'src_root_z_first_m': round(src_z[0], 5), 'src_root_z_last_m': round(src_z[-1], 5),
                      'src_root_z_min_m': round(min(src_z), 5), 'src_root_z_max_m': round(max(src_z), 5),
                      'root_z_rounding_err_m': max(abs(a / ratio - b) for a, b in zip(clip_back['root_z'], src_z)),
                      'root_tilt_max_deg': round(max(root_tilt), 4),
                      'hips_z_dev_min_m': round(min(hips_dev), 5), 'hips_z_dev_max_m': round(max(hips_dev), 5),
                      'hips_z_dev_abs_max_m': round(max(abs(d) for d in hips_dev), 5)}
    res['nan'] = nan
    res['ik_clamped_leg_frames'] = ik_clamped
    res['hip_drop_max_cm'] = round(max(hip_drops, default=0.0) * 100.0, 3)
    res['hip_drop_frames'] = sum(1 for d in hip_drops if d > 0)
    res['hip_drop_max_step_cm'] = round(max((abs(a - b) for a, b in zip(hip_drops, hip_drops[1:])), default=0.0) * 100.0, 3)
    res['ik_max_overshoot_cm'] = round(ik_over * 100.0, 3)
    res['hip_ratio'] = round(ratio, 6)
    res['num_frames'] = n
    res['warnings'] = rt.warnings
    res['rest_alignment_deg'] = rt.rest_align_deg

    # ---- preview
    if not args.no_preview:
        world = bpy.data.objects.new('CP_world_preview', cp_obj.data)
        scene.collection.objects.link(world)
        world.animation_data_create()
        world.animation_data.action = cp_obj.animation_data.action
        if hasattr(world.animation_data, 'action_slot') and world.animation_data.action_slot is None:
            world.animation_data.action_slot = cp_obj.animation_data.action_slot
        con = world.constraints.new('CHILD_OF')
        con.target, con.subtarget = cp_obj, 'Trajectory'
        con.inverse_matrix = Matrix.Identity(4)
        off = bpy.data.objects.new('sourcepack_offset', None)
        scene.collection.objects.link(off)
        off.location = (1.0, 0.0, 0.0)
        off.rotation_euler = (0.0, 0.0, math.radians(FACING_DEG))
        src_obj.parent = off
        src_obj.matrix_parent_inverse = Matrix.Identity(4)
        cp_obj['utpp_note'] = 'In-place game pose (Root identity). CP_world_preview = same pose carried by Trajectory. GASP source offset +1 m X, rotated 180 deg.'
        os.makedirs(args.preview_dir, exist_ok=True)
        bpy.context.preferences.filepaths.save_version = 0  # no .blend1 backups when re-running
        bpy.ops.wm.save_as_mainfile(filepath=os.path.join(args.preview_dir, clip + '.blend'), compress=True)
    res['status'] = 'ok'
    res['seconds'] = round(time.time() - t0, 2)
    return res


def main():
    args = parse_args()
    rig = GameRig(args.rig_json)
    bonemap = json.load(open(args.bonemap, encoding='utf-8'))
    clips = sorted(os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(args.fbx_dir, '*.fbx')))
    want = requested_clips(args)
    if want is not None:
        have = set(clips)
        for c in want:
            if c not in have:
                print('UTPP_RETARGET_MISSING %s (no %s.fbx in %s)' % (c, c, args.fbx_dir), flush=True)
        want = set(want)
        clips = [c for c in clips if c in want]
    results = []
    for clip in clips:
        try:
            r = process_clip(args, rig, bonemap, clip)
        except Exception as ex:  # keep batch going; record failure
            r = {'clip': clip, 'status': 'failed', 'error': repr(ex), 'trace': traceback.format_exc()}
        results.append(r)
        print('UTPP_RETARGET %s %s fk=%s slide=%s root=%s' % (
            clip, r['status'], r.get('check_fk_vs_blender', {}).get('pass'), r.get('foot_slide_flag'),
            r.get('root_motion', {}).get('pass')), flush=True)
    write_report(args, results)


def write_report(args, results):
    ok = [r for r in results if r['status'] == 'ok']
    failed = [r for r in results if r['status'] != 'ok']
    summary = {
        'clips_total': len(results), 'clips_ok': len(ok), 'clips_failed': [r['clip'] for r in failed],
        'check_a_fk_pass': sum(1 for r in ok if r['check_fk_vs_blender']['pass']),
        'check_a_max_pos_err_m': max((r['check_fk_vs_blender']['max_pos_err_m'] for r in ok), default=None),
        'check_a_max_rot_err': max((r['check_fk_vs_blender']['max_rot_err'] for r in ok), default=None),
        'check_b_slide_flagged_ankle_gt5': [r['clip'] for r in ok if r['foot_slide_flag']],
        'check_b_slide_flagged_pivot_gt5': [r['clip'] for r in ok if r['foot_slide_pivot_flag']],
        'check_b_slide_flagged_excess_vs_source_gt5': [r['clip'] for r in ok if r['foot_slide_excess_vs_source_flag']],
        'check_d_nan_clips': [r['clip'] for r in ok if r['nan']],
        'root_motion_pass': sum(1 for r in ok if r['root_motion']['pass']),
        'fbx_key_layout_ok': sum(1 for r in ok if r['fbx_keys']['ok']),
        'ik_clamped_clips': [r['clip'] for r in ok if r['ik_clamped_leg_frames']],
        'ik_clamped_leg_frames_total': sum(r['ik_clamped_leg_frames'] for r in ok),
        'ik_max_overshoot_cm': max((r['ik_max_overshoot_cm'] for r in ok), default=None),
        'warnings': sorted(set(w for r in ok for w in r['warnings'])),
    }
    rtz_ok = [r for r in ok if 'rtz' in r]
    if rtz_ok:
        summary['rtz_clips'] = len(rtz_ok)
        summary['rtz_hips_z_dev_abs_max_m'] = max(r['rtz']['hips_z_dev_abs_max_m'] for r in rtz_ok)
        summary['rtz_hips_z_dev_gt_0p25'] = [r['clip'] for r in rtz_ok if r['rtz']['hips_z_dev_abs_max_m'] > 0.25]
        summary['rtz_root_tilt_max_deg'] = max(r['rtz']['root_tilt_max_deg'] for r in rtz_ok)
    os.makedirs(args.report_dir, exist_ok=True)
    json.dump({'summary': summary, 'clips': results}, open(os.path.join(args.report_dir, 'report.json'), 'w'), indent=1)
    lines = ['UTPP retarget report (GASP -> player_woman_skeleton).',
             json.dumps(summary, indent=1), '',
             'Slide columns: ankle mean/max cm/s during contact>0.5 (source*ratio mean) [pivot=min(ankle,toe) mean cp/src]',
             'clip | frames | fkA pass (pos/rot err) | slide L | slide R | flag | traj disp m (src*ratio) | yaw traj/src | ik clamp | nan']
    for r in results:
        if r['status'] != 'ok':
            lines.append('%s | FAILED %s' % (r['clip'], r.get('error')))
            continue
        a, s, rm = r['check_fk_vs_blender'], r['foot_slide'], r['root_motion']

        def sl(side):
            c, src = s[side]['cp'], s[side]['source_scaled']
            return '%s/%s (%s) [%s/%s]' % (c['mean_cm_s'], c['max_cm_s'], src['mean_cm_s'],
                                           s[side]['cp_pivot']['mean_cm_s'], s[side]['source_scaled_pivot']['mean_cm_s'])
        lines.append('%s | %d | %s (%.1e/%.1e) | %s | %s | %s | %s (%s) | %s/%s | %d | %s' % (
            r['clip'], r['num_frames'], a['pass'], a['max_pos_err_m'], a['max_rot_err'], sl('l'), sl('r'),
            ('SLIDE>5' if r['foot_slide_flag'] else '-') + ('/PIVOT>5' if r['foot_slide_pivot_flag'] else '') +
            ('/EXCESS>5' if r['foot_slide_excess_vs_source_flag'] else ''), rm['trajectory_disp_m'], rm['source_disp_scaled_m'],
            rm['trajectory_yaw_deg'], rm['source_yaw_deg'], r['ik_clamped_leg_frames'], r['nan']))
    open(os.path.join(args.report_dir, 'report.txt'), 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
    print('UTPP_RETARGET_DONE ok=%d failed=%d' % (len(ok), len(failed)), flush=True)


if __name__ == '__main__':
    main()
