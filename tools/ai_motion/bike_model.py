"""Offline motorcycle geometry (WolvenKit exports) for leg clearance checks of the player's bike mount / dismount clips (bike13, 2026-11-09).

Vehicle space = game axes (x right, y forward, z up, metres), origin = the vehicle rig Root (the wheel axle plane for the sportbikes;
in game the vehicle origin sits 0.32-0.335 m above the ground = the tire radius: harness log 2026-11-08 22:35, player z 182.02 on foot,
vehicle 182.35, player 182.91 = vehicle + seat_front z 0.5565 when seated).
The workspot of the seat (OccupantSlots slot seat_front_left -> bone seat_front, zero offset) is the frame of the player's clips: clip
Root of the LAST frame = seat_front. Meshes: the parts of one appearance of the .app (identity component transforms; wheels / tires at
the rig's wheel bones), raw glTF vertices (no node transforms: WolvenKit exports the vehicle parts in vehicle space; glTF Y-up -> game:
x, -z, y). Sources (read-only extraction, 2026-11-09): Extracted\game\vehicle-bike\mesh-glb\<bike>\*.glb (WolvenKit export of
basegame_4_appearance meshes), Extracted\game\vehicle-bike\ent-json\*.app.json / *.rig.json (WolvenKit serialize).

    model = BikeModel.for_set('sportbike3')   # Apollo (player_01 appearance: body + rear top case bags_b_01); 'sportbike2' = Arch
    rows = clearance_report(model, P_seat_m, ix, 'Right')     # per frame: min (surface distance - limb radius) of the leg samples
    n = sweep_crossings(model, P_seat_m, ix, 'Right')         # leg sample points passing THROUGH a surface between two frames
Requires trimesh (Tools\ai-motion\envs\kimodo). Sets: 'sportbike' (Yaiba Kusanagi, player_01), 'sportbike2' (Arch), 'sportbike3' (Apollo).
"""
import json
import os

import numpy as np
import trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
EJ = os.path.join(ROOT, 'Extracted', 'game', 'vehicle-bike', 'ent-json')
MG = os.path.join(ROOT, 'Extracted', 'game', 'vehicle-bike', 'mesh-glb')


def quat_to_mat(i, j, k, r):
    x, y, z, w = i, j, k, r
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_rig(path):
    """WolvenKit .rig.json -> {bone: world 4x4} (bind pose, game axes)"""
    d = json.load(open(path, encoding='utf-8'))
    root = d['Data']['RootChunk']
    names = [b['$value'] for b in root['boneNames']]
    par = [int(p) for p in root['boneParentIndexes']]
    W = []
    for b, tr in enumerate(root['boneTransforms']):
        t = tr['Translation']; q = tr['Rotation']
        T = np.eye(4)
        T[:3, :3] = quat_to_mat(float(q['i']), float(q['j']), float(q['k']), float(q['r']))
        T[:3, 3] = [float(t['X']), float(t['Y']), float(t['Z'])]
        W.append(T if par[b] < 0 else W[par[b]] @ T)
    return dict(zip(names, W))


def app_components(path, appearance):
    """mesh components of one appearance of a WolvenKit .app.json (shadow / proxy meshes skipped)"""
    d = json.load(open(path, encoding='utf-8'))
    for a in d['Data']['RootChunk']['appearances']:
        ad = a.get('Data', a)
        if ad.get('name', {}).get('$value') != appearance:
            continue
        out = []
        for c in ad.get('components', []):
            dd = c.get('Data', c)
            mesh = dd.get('mesh', {}).get('DepotPath', {}).get('$value') if isinstance(dd.get('mesh'), dict) else None
            if not mesh or 'shadow' in mesh or 'proxy' in mesh:
                continue
            lt = dd.get('localTransform', {}); pos = lt.get('Position', {}); ori = lt.get('Orientation', {})
            pv = np.array([float(pos.get(k, {}).get('Bits', 0)) / 131072.0 if isinstance(pos.get(k), dict) else float(pos.get(k) or 0) for k in ('x', 'y', 'z')])
            q = [float(ori.get(k, 0)) for k in ('i', 'j', 'k', 'r')]
            if abs(np.linalg.norm(q) - 1) > 1e-3:
                q = [0.0, 0.0, 0.0, 1.0]
            out.append(dict(name=dd.get('name', {}).get('$value'), mesh=os.path.basename(mesh.replace('\\', '/')), pos=pv, quat=q))
        return out
    raise KeyError(appearance)


def load_mesh_game(path):
    """raw vertices / faces of every primitive of a WolvenKit GLB, glTF -> game axes"""
    sc = trimesh.load(path, force='scene', process=False)
    vs, fs = [], []
    n = 0
    for g in sc.geometry.values():
        v = np.asarray(g.vertices, float)
        v = np.stack([v[:, 0], -v[:, 2], v[:, 1]], 1)
        vs.append(v); fs.append(np.asarray(g.faces) + n); n += len(v)
    return trimesh.Trimesh(np.concatenate(vs), np.concatenate(fs), process=False)


class BikeModel:
    SETS = {
        'sportbike': dict(bike='v_sportbike1_yaiba_kusanagi', rig='v_sportbike1_yaiba_kusanagi.rig.json', app='yaiba_kusanagi_basic.app.json', appearance='player_01',
                          wheels={'tire_r_01': 'wheel_back', 'wheel_r_01': 'wheel_back', 'tire_f_01': 'wheel_front', 'wheel_f_01': 'wheel_front'},
                          skip=('license_plate', 'fuelport', 'fuel_cap', 'rod_01', 'kickstand')),
        'sportbike3': dict(bike='v_sportbike3_brennan_apollo', rig='v_sportbike3_brennan_apollo_01.rig.json', app='brennan_apollo_basic.app.json', appearance='player_01',
                           wheels={'tire_b_01': 'wheel_back', 'wheel_b_01': 'wheel_back', 'tire_f_01': 'wheel_front', 'wheel_f_01': 'wheel_front', 'break_01': 'wheel_front'},
                           skip=('license_plate', 'fuelport', 'fuelcap')),
        'sportbike2': dict(bike='v_sportbike2_arch_nemesis', rig='v_sportbike2_arch_nemesis_01.rig.json', app='arch_nemesis_basic.app.json', appearance='player_01',
                           wheels={'tire_b_01': 'wheel_back', 'wheel_b_01': 'wheel_back', 'tire_f_01': 'wheel_front', 'wheel_f_01': 'wheel_front'},
                           skip=('license_plate', 'fuelport', 'fuelcap', 'sticker', 'cables', 'collider', 'brakelight')),
    }

    def __init__(self, bike, rig_json, app_json, appearance, wheel_parts, skip=()):
        self.bike = bike
        self.appearance = appearance
        self.rig = load_rig(rig_json)
        self.seat_T = self.rig['seat_front']
        comps = app_components(app_json, appearance)
        parts, names = [], []
        for c in comps:
            nm = c['mesh'].replace('.mesh', '')
            if any(s in nm for s in skip):
                continue
            p = os.path.join(MG, bike, nm + '.glb')
            if not os.path.exists(p):
                continue
            m = load_mesh_game(p)
            T = np.eye(4); T[:3, :3] = quat_to_mat(*c['quat']); T[:3, 3] = c['pos']
            short = nm.split('__ext01_')[-1]
            if short in wheel_parts:
                T = T.copy(); T[:3, 3] += self.rig[wheel_parts[short]][:3, 3]
            m.apply_transform(T)
            parts.append(m); names.append(short)
        self.parts, self.part_names = parts, names
        self.mesh = trimesh.util.concatenate(parts)
        self.pq = trimesh.proximity.ProximityQuery(self.mesh)
        self.face_part = np.concatenate([np.full(len(p.faces), i) for i, p in enumerate(parts)])
        self.ground_z = float(min(p.bounds[0, 2] for p, n in zip(parts, names) if n.startswith('tire')))
        self._ray = None

    @classmethod
    def for_set(cls, vehicle_set):
        """vehicle_set: 'sportbike2' | 'sportbike3' (also accepts 'pwa_sportbike3' / 'pma_sportbike2' style names)"""
        key = vehicle_set.split('_')[-1]
        if key not in cls.SETS:
            raise KeyError('no offline bike geometry for %r (have %s)' % (vehicle_set, sorted(cls.SETS)))
        s = cls.SETS[key]
        return cls(s['bike'], os.path.join(EJ, s['rig']), os.path.join(EJ, s['app']), s['appearance'], s['wheels'], s['skip'])

    def seat_to_vehicle(self, P_seat_m):
        return P_seat_m @ self.seat_T[:3, :3].T + self.seat_T[:3, 3]

    def surface_distance(self, pts_vehicle, normals=False):
        """unsigned distance (m) to the nearest surface and the part index, points (N, 3) in vehicle space; normals=True adds the outward face normal of the closest face"""
        closest, dist, fid = self.pq.on_surface(pts_vehicle)
        if normals:
            return dist, self.face_part[fid], self.mesh.face_normals[fid]
        return dist, self.face_part[fid]

    def ray(self):
        if self._ray is None:
            self._ray = trimesh.ray.ray_triangle.RayMeshIntersector(self.mesh)
        return self._ray


# limb cylinders: (proximal joint, distal joint, radius m); the foot segment runs on past the toe bone to the shoe tip (toe_ext)
LIMBS = {'Left': (('LeftUpLeg', 'LeftLeg', 0.085), ('LeftLeg', 'LeftFoot', 0.06), ('LeftFoot', 'LeftToeBase', 0.05)),
         'Right': (('RightUpLeg', 'RightLeg', 0.085), ('RightLeg', 'RightFoot', 0.06), ('RightFoot', 'RightToeBase', 0.05))}


def leg_samples(P, ix, side, step=0.04, toe_ext=0.09, thigh=True):
    """sample points (m, every `step` along the thigh / shin / foot) of one leg for every frame: list of (segment label, u, (T,3) points, radius)"""
    out = []
    for a, b, r in LIMBS[side]:
        if not thigh and a.endswith('UpLeg'):
            continue
        A, B = P[:, ix[a]], P[:, ix[b]]
        if b.endswith('ToeBase'):
            d = B - A
            B = B + d / np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9) * toe_ext
        L = np.linalg.norm(B - A, axis=1).max()
        n = max(2, int(np.ceil(L / step)) + 1)
        for u in np.linspace(0, 1, n):
            out.append(('%s-%s' % (a, b), float(u), A + (B - A) * u, r))
    return out


def clearance_report(model, P_seat_m, ix, side, zshift_m=0.0, frames=None, thigh=True):
    """per frame: min (surface distance - radius) in cm over the leg samples (segment, u, part, sample z in vehicle space).
    P_seat_m (T, N, 3): joint positions in the SEAT frame (m, game axes)."""
    T = len(P_seat_m)
    frames = range(T) if frames is None else frames
    samples = leg_samples(P_seat_m, ix, side, thigh=thigh)
    labels = [s[0] for s in samples]; us = [s[1] for s in samples]
    pts = np.stack([s[2] for s in samples], 1)
    rad = np.array([s[3] for s in samples])
    rows = []
    for f in frames:
        pv = model.seat_to_vehicle(pts[f]); pv[:, 2] += zshift_m
        dist, part, nrm = model.surface_distance(pv, normals=True)
        c = dist - rad
        k = int(np.argmin(c))
        rows.append(dict(frame=int(f), clear_cm=round(float(c[k] * 100), 2), seg=labels[k], u=round(us[k], 2), part=model.part_names[part[k]], z_cm=round(float(pv[k, 2] * 100), 1),
                         nrm=[round(float(x), 3) for x in nrm[k]]))
    return rows


def sweep_crossings(model, P_seat_m, ix, side, zshift_m=0.0, thigh=True):
    """per frame transition f-1 -> f: number of leg sample points whose straight path crosses a mesh surface (a point inside a closed part is invisible to the distance)"""
    samples = leg_samples(P_seat_m, ix, side, step=0.06, thigh=thigh)
    pts = np.stack([s[2] for s in samples], 1)
    V = model.seat_to_vehicle(pts.reshape(-1, 3)).reshape(pts.shape)
    V[..., 2] += zshift_m
    ray = model.ray()
    out = [0]
    for f in range(1, len(V)):
        o = V[f - 1]; d = V[f] - V[f - 1]
        L = np.linalg.norm(d, axis=1)
        m = L > 1e-6
        if not m.any():
            out.append(0); continue
        loc, idx_ray, idx_tri = ray.intersects_location(o[m], d[m] / L[m][:, None], multiple_hits=True)
        n = 0
        if len(loc):
            t = np.linalg.norm(loc - o[m][idx_ray], axis=1)
            n = int((t <= L[m][idx_ray] + 1e-9).sum())
        out.append(n)
    return out
