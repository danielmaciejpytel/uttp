"""Offline IK authoring candidate; preserve arm lengths and a rigid hand/weapon group.

Consumes the existing Nue retarget, not a new retail capture. Its input contact
fidelity is NOT established. Output is separate from all originals and game data.
"""
from pathlib import Path
import copy
import argparse
import hashlib
import json
import numpy as np
from replay_body_skin import trs

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'Animations/Blender/Pistol1H/p1h_nue_mag_reload_gasp_core.json'
REFERENCE = ROOT / 'Animations/Blender/Pistol1H/p1h_overture_reload_blender.json'
OUT = SOURCE.with_name('p1h_nue_mag_reload_rigid_contacts_core.json')
REPORT = ROOT / 'Research/logs/game/aim-pose-regression-20261015/p1h-rigid-contacts.json'


def matrix(row):
    return trs([*row[:3], 1, *row[3:7], *row[7:10], 1])


def unit(v):
    n = np.linalg.norm(v)
    if n < 1e-10 or not np.isfinite(n):
        raise ValueError('Degenerate IK direction')
    return v / n


def rotation(a, b):
    a, b = unit(a), unit(b)
    c = float(np.clip(a @ b, -1, 1))
    if c < -1 + 1e-9:
        axis = np.eye(3)[int(np.argmin(np.abs(a)))]
        axis = unit(np.cross(a, axis))
        return 2 * np.outer(axis, axis) - np.eye(3)
    v = np.cross(a, b)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


def local_row(m):
    scale = np.linalg.norm(m[:3,:3], axis=0)
    if np.min(scale) < 1e-9:
        raise ValueError('Singular output scale')
    r = m[:3, :3] / scale
    if np.max(np.abs(r.T @ r - np.eye(3))) > 1e-5 or np.linalg.det(r) < .99999:
        raise ValueError('Nonrigid output transform')
    # Eigenvector formulation avoids special cases near 180 degrees.
    xx, yy, zz = r[0, 0], r[1, 1], r[2, 2]
    k = np.array([[xx-yy-zz, r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
                  [r[0,1]+r[1,0], yy-xx-zz, r[1,2]+r[2,1], r[0,2]-r[2,0]],
                  [r[0,2]+r[2,0], r[1,2]+r[2,1], zz-xx-yy, r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], xx+yy+zz]])
    _, vec = np.linalg.eigh(k)
    q = vec[:, -1]
    if q[3] < 0:
        q = -q
    row = [*m[:3, 3], *q, *scale]
    if np.max(np.abs(matrix(row) - m)) > 1e-6:
        raise ValueError('TRS roundtrip failed')
    return [float(x) for x in row]


def models(rows, parents):
    out = []
    for i, row in enumerate(rows):
        out.append((out[parents[i]] if parents[i] >= 0 else np.eye(4)) @ matrix(row))
    return out


def common_shift(targets, shoulders, radii):
    # Exact projection onto two reach balls in GROUP-TRANSLATION space.
    # Both hands/weapons receive the same shift. A disjoint intersection rejects.
    centers = [shoulders[i] - targets[i] for i in range(2)]
    def inside(x):
        return all(np.linalg.norm(x-centers[i]) <= radii[i]+1e-9 for i in range(2))
    if inside(np.zeros(3)):
        return np.zeros(3)
    candidates = []
    for c, r in zip(centers, radii):
        x = c * max(0., 1-r/max(np.linalg.norm(c), 1e-15))
        if inside(x):
            candidates.append(x)
    if candidates:
        return min(candidates, key=lambda x: np.linalg.norm(x))
    delta = centers[1]-centers[0]
    distance = np.linalg.norm(delta)
    if distance > sum(radii) or distance <= abs(radii[0]-radii[1]):
        raise ValueError(f'No shared reach solution: centerDistance={distance:.9f}, radiusSum={sum(radii):.9f}')
    axis = delta/distance
    along = (radii[0]**2-radii[1]**2+distance**2)/(2*distance)
    circle = centers[0]+axis*along
    radius = np.sqrt(max(0., radii[0]**2-along**2))
    direction = -circle+axis*(circle @ axis)
    if np.linalg.norm(direction) < 1e-10:
        direction = np.cross(axis, np.eye(3)[int(np.argmin(np.abs(axis)))])
    x = circle+unit(direction)*radius
    if not inside(x):
        raise ValueError('Analytic reach projection violated bounds')
    return x


def rigid_reach_rotation(targets, shoulders, radius_sum):
    # Minimal rotation toward the shoulder baseline before shared translation.
    # Rotating the WHOLE group preserves every relative hand/weapon transform.
    s, h = shoulders[0]-shoulders[1], targets[0]-targets[1]
    ns, nh = np.linalg.norm(s), np.linalg.norm(h)
    if np.linalg.norm(s-h) <= radius_sum:
        return np.eye(3), 0.
    if abs(ns-nh) >= radius_sum:
        raise ValueError('Rigid hand group cannot fit any orientation at authored arm lengths')
    angle = np.arccos(np.clip(unit(h) @ unit(s), -1, 1))
    limit = np.arccos(np.clip((ns*ns+nh*nh-radius_sum*radius_sum)/(2*ns*nh), -1, 1))
    turn = max(0., angle-limit)+1e-7
    axis = np.cross(h, s)
    if np.linalg.norm(axis) < 1e-10:
        axis = np.cross(unit(h), np.eye(3)[int(np.argmin(np.abs(unit(h))))])
    x, y, z = unit(axis)
    k = np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    return np.eye(3)+np.sin(turn)*k+(1-np.cos(turn))*k@k, float(turn)


def clip_contact_shift(track, parents, chains, shoulders, radii):
    """One translation for the WHOLE clip, projected onto all reach balls."""
    centers=np.array([shoulders[s]-models(f['local'],parents)[c[2]][:3,3]
                      for f in track['frames'] for s,c in enumerate(chains)])
    radius=np.tile(radii,len(track['frames']))
    x=np.zeros(3); residual=np.zeros_like(centers)
    for iteration in range(2000):
        for i,(center,r) in enumerate(zip(centers,radius)):
            y=x+residual[i]; delta=y-center; distance=np.linalg.norm(delta)
            projected=center+delta*min(1.,r/max(distance,1e-15))
            residual[i]=y-projected; x=projected
        violation=float(np.max(np.linalg.norm(x-centers,axis=1)-radius))
        if violation<1e-8:
            return x
    raise ValueError(f'No converged whole-clip shared translation; max reach violation {violation:.6f}m')


def main():
    global SOURCE, OUT, REPORT
    parser=argparse.ArgumentParser()
    parser.add_argument('--transport-elbow-guide',action='store_true')
    parser.add_argument('--constant-contact-shift',action='store_true')
    parser.add_argument('--smooth-contact-shift',action='store_true')
    parser.add_argument('--reach-margin',type=float,default=1e-6)
    parser.add_argument('--input',type=Path)
    parser.add_argument('--output-tag')
    parser.add_argument('--native-elbow-guide',action='store_true')
    parser.add_argument('--native-both-elbows',action='store_true')
    parser.add_argument('--continuous-elbow-guide',action='store_true')
    parser.add_argument('--wrist-elbow-guide',action='store_true')
    parser.add_argument('--hand-relative-forearm-seed',action='store_true')
    parser.add_argument('--left-elbow-down',action='store_true')
    parser.add_argument('--authored-left-forearm-roll',action='store_true')
    parser.add_argument('--native-forearm-roll',action='store_true')
    args=parser.parse_args()
    if args.native_both_elbows:
        args.native_elbow_guide = True
    if args.wrist_elbow_guide:
        args.native_forearm_roll = True
    if args.hand_relative_forearm_seed:
        args.native_forearm_roll = True
    if args.smooth_contact_shift and (args.constant_contact_shift or args.transport_elbow_guide):
        raise ValueError('Smoothed translation is a separate experiment')
    if args.native_elbow_guide and args.transport_elbow_guide:
        raise ValueError('Native and authored transport guides are separate experiments')
    if args.input:
        SOURCE=args.input.resolve(strict=True)
    if not 1e-6<=args.reach_margin<=.1:
        raise ValueError('Reach margin outside authoring range')
    if args.transport_elbow_guide and args.constant_contact_shift:
        raise ValueError('Isolate one experiment at a time')
    if args.transport_elbow_guide:
        OUT=OUT.with_name('p1h_nue_mag_reload_transport_elbow_core.json')
        REPORT=REPORT.with_name('p1h-transport-elbow-contacts.json')
    if args.constant_contact_shift:
        OUT=OUT.with_name('p1h_nue_mag_reload_constant_shift_core.json')
        REPORT=REPORT.with_name('p1h-constant-shift-contacts.json')
    if args.reach_margin!=1e-6:
        suffix=f'_margin{round(args.reach_margin*1000)}mm'
        OUT=OUT.with_stem(OUT.stem+suffix)
        REPORT=REPORT.with_stem(REPORT.stem+suffix)
    if args.output_tag:
        if not args.output_tag.replace('-','').isalnum():
            raise ValueError('Invalid output tag')
        OUT=OUT.with_name('p1h_nue_'+args.output_tag+'_core.json')
        REPORT=REPORT.with_name('p1h-'+args.output_tag+'-contacts.json')
    data = json.loads(SOURCE.read_text())
    native=json.loads(SOURCE.with_name('p1h_nue_native_core.json').read_text()) if args.native_elbow_guide or args.native_forearm_roll else None
    if native and native['bone_names']!=data['bone_names']:
        raise ValueError('Native guide rig mismatch')
    if native and native['source_sha256']!=data['source_sha256']:
        raise ValueError('Native guide source identity mismatch')
    reference = json.loads(REFERENCE.read_text())
    names, parents = data['bone_names'], data['parents']
    if names != reference['bone_names'] or parents != reference['parents']:
        raise ValueError('Rig identity mismatch')
    idx = {n: i for i, n in enumerate(names)}
    bases = {t['name']: t['frames'][0]['local'] for t in reference['tracks'] if t['name'].startswith('gasp_authored_')}
    chains = [[idx[s + n] for n in ('Arm', 'ForeArm', 'Hand')] for s in ('Left', 'Right')]
    report = {'scope': 'Offline arm-length correction of existing retarget; no source-contact, skin or runtime acceptance',
              'elbowGuide': 'native_wrist_axis_projection' if args.wrist_elbow_guide else 'bilateral_native_contact_group_projection' if args.native_both_elbows else 'left_native_contact_group_projection_right_authored' if args.native_elbow_guide else 'minimal_swing_from_authored_axis_experimental' if args.transport_elbow_guide else 'projected_authored_pole',
              'reachShift': 'temporally_smoothed_feasible' if args.smooth_contact_shift else 'whole_clip_constant' if args.constant_contact_shift else 'per_frame_minimal',
              'reachMarginMeters':args.reach_margin,
              'forearmRoll':'native_projected_to_authored_axis' if args.native_forearm_roll else 'authored_minimal_swing',
              'continuousElbowGuide': args.continuous_elbow_guide,
              'wristElbowGuide': args.wrist_elbow_guide,
              'handRelativeForearmSeed': args.hand_relative_forearm_seed,
              'leftElbowDown': args.left_elbow_down,
              'authoredLeftForearmRoll': args.authored_left_forearm_roll,
              'readyForGameIntegration': False,
              'remainingGates': ['Native-source hand/magazine contact verification', 'Skin, collision and elbow continuity review',
                                 'Hand-path authoring review, including reported shared reach displacement'],
              'sourceSha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(), 'tracks': []}
    output = copy.deepcopy(data)
    for track in output['tracks']:
        native_track=next(t for t in native['tracks'] if t['name']==track['name']) if native else None
        if native_track and len(native_track['frames'])!=len(track['frames']):
            raise ValueError('Native guide timing mismatch')
        base = bases[track['base']]
        bm = models(base, parents)
        lengths = [[np.linalg.norm(np.array(base[b][:3])) for b in c[1:]] for c in chains]
        shoulders = [bm[c[0]][:3, 3] for c in chains]
        poles = [bm[c[1]][:3, 3] - shoulders[i] for i, c in enumerate(chains)]
        max_shift = max_turn = max_hand_error = max_contact_error = max_length_error = 0.
        input_length_ranges = {names[b]: [] for c in chains for b in c[1:]}
        fixed_shift=clip_contact_shift(track,parents,chains,shoulders,[sum(v)-args.reach_margin for v in lengths]) if args.constant_contact_shift else None
        smooth_shifts = None
        if args.smooth_contact_shift:
            radii = [sum(v)-args.reach_margin for v in lengths]
            hand_targets = [[models(f['local'], parents)[c[2]][:3, 3] for c in chains] for f in track['frames']]
            smooth_shifts = np.array([common_shift(t, shoulders, radii) for t in hand_targets])
            # Neighbour relaxation plus exact projection onto both reach balls.
            # Feasibility is mandatory; this does not claim a global optimum.
            for _ in range(120):
                proposal = smooth_shifts.copy()
                proposal[1:-1] = .5*smooth_shifts[1:-1] + .25*(smooth_shifts[:-2]+smooth_shifts[2:])
                for k, candidate in enumerate(proposal):
                    shifted = [t+candidate for t in hand_targets[k]]
                    smooth_shifts[k] = candidate + common_shift(shifted, shoulders, radii)
        previous_guides = [None, None]
        limited_guides = 0
        for frame in track['frames']:
            original_rows = frame['local']
            im = models(original_rows, parents)
            targets = [im[c[2]][:3, 3] for c in chains]
            radii = [sum(v)-args.reach_margin for v in lengths]
            group_rotation, turn = rigid_reach_rotation(targets, shoulders, sum(radii))
            if smooth_shifts is not None and turn > 0:
                raise ValueError('Smoothed translation cannot use per-frame rotation')
            if fixed_shift is not None and turn>0:
                raise ValueError('Whole-clip translation cannot use per-frame rotation')
            pivot = targets[1].copy()
            targets = [pivot+group_rotation@(t-pivot) for t in targets]
            try:
                shift = common_shift(targets, shoulders, radii)
                if fixed_shift is not None:
                    shift=fixed_shift
                if smooth_shifts is not None:
                    shift=smooth_shifts[frame['ordinal']]
            except ValueError as exc:
                raise ValueError(f'{track["name"]}/{track["base"]}/frame{frame["ordinal"]}: {exc}') from exc
            max_shift = max(max_shift, float(np.linalg.norm(shift)))
            max_turn = max(max_turn, turn)
            group = np.eye(4)
            group[:3,:3] = group_rotation
            group[:3,3] = pivot-group_rotation@pivot+shift
            explicit = {}
            for side, c in enumerate(chains):
                upper, lower, hand = c
                shoulder = shoulders[side]
                target = targets[side] + shift
                distance = np.linalg.norm(target - shoulder)
                l1, l2 = lengths[side]
                if distance <= abs(l1-l2) + 1e-8:
                    raise ValueError('Inner reach boundary; no independent hand clamping allowed')
                axis = unit(target - shoulder)
                pole = poles[side] - axis * (poles[side] @ axis)
                if args.transport_elbow_guide:
                    ref_axis=unit(bm[hand][:3,3]-shoulder)
                    ref_pole=poles[side]-ref_axis*(poles[side]@ref_axis)
                    pole=rotation(ref_axis,axis)@ref_pole
                if args.native_elbow_guide and native_track and (side==0 or args.native_both_elbows):
                    nf=native_track['frames'][frame['ordinal']]
                    if abs(nf['time_seconds']-frame['time_seconds'])>1e-7:
                        raise ValueError('Native guide sample mismatch')
                    nh=np.array(nf['models'][chains[1][2]]).reshape(4,4)
                    ne=np.array(nf['models'][lower]).reshape(4,4)
                    guide=(group@im[chains[1][2]]@np.linalg.inv(nh)@ne)[:3,3]-shoulder
                    projected=guide-axis*(guide@axis)
                    if np.linalg.norm(projected)>1e-8:
                        pole=projected
                if args.wrist_elbow_guide:
                    nf = native_track['frames'][frame['ordinal']]
                    anchor_hand = hand if args.hand_relative_forearm_seed else chains[1][2]
                    nh = np.array(nf['models'][anchor_hand]).reshape(4,4)
                    nl = np.array(nf['models'][lower]).reshape(4,4)
                    seed = (group @ im[anchor_hand] @ np.linalg.inv(nh) @ nl)[:3,:3]
                    desired_elbow = target - seed @ np.array(base[hand][:3])
                    guide = desired_elbow - shoulder
                    projected = guide-axis*(guide@axis)
                    if np.linalg.norm(projected) > 1e-8:
                        pole = projected
                if args.left_elbow_down and side == 0:
                    down=np.array([0.,0.,-1.])
                    projected=down-axis*(down@axis)
                    if np.linalg.norm(projected)>1e-8:
                        pole=projected
                if np.linalg.norm(pole) < 1e-8:
                    raise ValueError('Authored elbow guide singularity')
                pole = unit(pole)
                if args.continuous_elbow_guide:
                    previous = previous_guides[side]
                    if previous is not None:
                        old_axis, old_pole, old_time = previous
                        transported = unit(rotation(old_axis, axis) @ old_pole)
                        angle = float(np.arctan2(axis @ np.cross(transported, pole), transported @ pole))
                        # 360 degrees/second, 12 degrees per sample at 30 FPS.
                        limit = np.radians(360.) * (frame['time_seconds'] - old_time)
                        clipped = float(np.clip(angle, -limit, limit))
                        limited_guides += abs(clipped-angle) > 1e-8
                        pole = unit(transported*np.cos(clipped) + np.cross(axis, transported)*np.sin(clipped))
                    previous_guides[side] = (axis.copy(), pole.copy(), frame['time_seconds'])
                along = (l1*l1-l2*l2+distance*distance)/(2*distance)
                elbow = shoulder + axis*along + unit(pole)*np.sqrt(max(0., l1*l1-along*along))
                for bone, point, end, child in ((upper, shoulder, elbow, lower), (lower, elbow, target, hand)):
                    m = np.eye(4)
                    seed=bm[bone][:3,:3]
                    if args.native_forearm_roll and bone==lower and not (side==0 and args.authored_left_forearm_roll):
                        nf=native_track['frames'][frame['ordinal']]
                        if abs(nf['time_seconds']-frame['time_seconds'])>1e-7:
                            raise ValueError('Native roll sample mismatch')
                        anchor_hand = hand if args.hand_relative_forearm_seed else chains[1][2]
                        nh=np.array(nf['models'][anchor_hand]).reshape(4,4)
                        nl=np.array(nf['models'][lower]).reshape(4,4)
                        seed=(group@im[anchor_hand]@np.linalg.inv(nh)@nl)[:3,:3]
                    m[:3, :3] = rotation(seed @ np.array(base[child][:3]), end-point) @ seed
                    m[:3, 3] = point
                    explicit[bone] = m
                explicit[hand] = group @ im[hand]
                for b in c[1:]:
                    input_length_ranges[names[b]].append(float(np.linalg.norm(original_rows[b][:3])))
            # Preserve all hand descendants and weapon attachments as the input
            # rigid contact group, rather than retargeting each one separately.
            for i, n in enumerate(names):
                p = parents[i]
                under_hand = False
                while p >= 0:
                    under_hand |= p in (chains[0][2], chains[1][2])
                    p = parents[p]
                if under_hand or n in ('WeaponLeft', 'WeaponRight'):
                    explicit[i] = group @ im[i]
            final, rows = [], []
            for i in range(len(names)):
                parent = final[parents[i]] if parents[i] >= 0 else np.eye(4)
                m = explicit[i] if i in explicit else parent @ matrix(base[i])
                rows.append(local_row(np.linalg.inv(parent) @ m))
                final.append(parent @ matrix(rows[-1]))
            for c in chains:
                for b in c:
                    max_length_error = max(max_length_error, float(np.linalg.norm(np.array(rows[b][:3])-base[b][:3])))
                max_hand_error = max(max_hand_error, float(np.linalg.norm(final[c[2]][:3,3] - (group @ im[c[2]])[:3,3])))
            for a, b in [('LeftHand','RightHand'), ('LeftHand','WeaponLeft'), ('RightHand','WeaponRight')]:
                before = np.linalg.inv(im[idx[a]]) @ im[idx[b]]
                after = np.linalg.inv(final[idx[a]]) @ final[idx[b]]
                max_contact_error = max(max_contact_error, float(np.max(np.abs(before-after))))
            frame['local'] = rows
            frame['phase'] = 'offline_fixed_length_ik_shared_contact_group'
        if max(max_hand_error, max_contact_error, max_length_error) > 1e-6:
            raise ValueError('Fixed-length/contact invariant failed')
        report['tracks'].append({'clip': track['name'], 'base': track['base'], 'frames': len(track['frames']),
            'maxCommonReachShiftMeters': max_shift, 'maxHandTargetErrorMeters': max_hand_error,
            'maxCommonReachRotationDegrees': float(np.degrees(max_turn)),
            'elbowGuideLimitedSamples': int(limited_guides),
            'maxReachShiftStepMeters': float(np.max(np.linalg.norm(np.diff(smooth_shifts, axis=0), axis=1))) if smooth_shifts is not None else None,
            'maxContactMatrixError': max_contact_error, 'maxArmLocalTranslationErrorMeters': max_length_error,
            'inputBoneLengthRangesMeters': {n: [min(v), max(v)] for n,v in input_length_ranges.items()}})
    output['method'] = 'Fixed GASP torso/shoulders, fixed-length analytic bilateral IK, shared rigid rotation/translation for reach, input hand/weapon-relative transforms preserved.'
    output['limitations'] += ['Input contact accuracy is not proven against native clip; preserving it does not prove magazine contact.',
        'Common reach shift changes the hand path; its magnitude is reported. Elbow continuity and body collisions need review.',
        'Separate offline authoring candidate; not deployed and not visual acceptance.']
    OUT.write_text(json.dumps(output, separators=(',', ':'))+'\n')
    report['output'] = str(OUT.relative_to(ROOT))
    report['outputSha256'] = hashlib.sha256(OUT.read_bytes()).hexdigest()
    REPORT.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
