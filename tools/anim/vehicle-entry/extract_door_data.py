"""Door data of the six player car classes for check_door_clearance.py (offline, reads only the files extracted by 01_extract_car.ps1 -Doors).

  python -I extract_door_data.py [--root <Extracted\\game\\vehicle-entry-car>] [--out <dir>]

Per class (standard2 Archer Hella, standard25 Villefort Columbus, standard3 Chevalier Emperor, sport1 Quadra Turbo-R, sport2 Herrera Outlaw
(v_sport2_porsche_911turbo), utility Kaukaz Bratsk) writes <out>\\<class>.json with
  seat.left / seat.right   world transform (t, R row-major 3x3) of the rig bones seat_front_left / seat_front_right (rest pose of the vehicle
                           .anims rig; the entity slot has an identity offset to the bone: ent json slot seat_front_left -> bone seat_front_left),
  door.left / door.right   phase_frames (the vehicle clip door_front_* Phase events startup / loop / recover, 30 fps) and 'parts': every solid piece
                           of the front door, each with the rig bone it hangs on (door_front_*_a: the door skin and, for the Outlaw, the rear quarter
                           window; window_front_*_a (child of the door bone): the window glass; door_front_*_c: the Turbo-R sill / floor piece),
                           the world transform of that bone for frames 0..loop_end of the vehicle clip (the additive clip is exported added on the rest
                           pose, so the glass and sill follow the door) and the mesh SURFACE as points (about 3-4 cm apart, triangle sampling, in the
                           bone frame; glTF Y-up / -Z forward as every glb of this pipeline). A surface (not a box per height band) keeps the window
                           opening and the hollow door open.
All inputs are hashed into <out>\\provenance.txt.

  python -I extract_door_data.py --rear [--root <Extracted\\game\\vehicle-entry-car>]      ADDITIVE (nothing existing is touched): the REAR gate data
Per 4-door class (standard2 Hella, standard25 Columbus, standard3 Emperor) and side writes <root>\\rear-data\\<class>.json from the rear door meshes exported by
extract_door_assets.py --rear (<root>\\glb-rear) and the rest pose of the vehicle rig (glb-veh): 'static_parts' = rear door skin (bone door_back_{left,right}_a) and
rear glass (window_back_*_a, Hella / Emperor; the Columbus ent has no window component) as surface point clouds (3 cm) in the RIG space of door-data (rest pose,
the rear doors never move during a front-seat entry), and 'solid_boxes' = the B-pillar block (depth / length SYNTHETIC, no body mesh was extracted) and the rear glass slab as
axis-aligned boxes in VEHICLE axes. Mesh names verified from the door_br / door_bl .ent files (json-rear). Provenance (sha256 of ent, mesh, glb) is in the json and in
rear-data\\provenance.txt.
"""
import argparse
import glob
import hashlib
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import skel as S  # noqa: E402
import glbanim as G  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

# class -> (vehicle .anims glb stem, parts(s, sd) -> [(mesh stem, bone kind)]); s = l / r, sd = fl / fr; bone kind: door | window | sill
CLASSES = {
    'standard2': ('v_standard2_archer_hella', lambda s, sd: [('v_standard2_archer_hella__ext01_door_%s_01' % sd, 'door'),
                                                             ('v_standard2_archer_hella__ext01_window_%s_01' % sd, 'window')]),
    'standard25': ('v_standard25_villefort_columbus', lambda s, sd: [('v_standard25_villefort_columbus__ext01_door_%s_01' % sd, 'door'),
                                                                     ('v_standard25_villefort_columbus__ext01_window_%s_01' % sd, 'window')]),
    'standard3': ('v_standard3_chevalier_emperor', lambda s, sd: [('v_standard3_chevalier_emperor__ext01_door_%s_01' % sd, 'door'),
                                                                  ('v_standard3_chevalier_emperor__ext01_door_window_%s_01' % sd, 'window')]),
    'sport1': ('v_sport1_quadra_turbo', lambda s, sd: [('v_sport1_quadra_turbo_ext01_door_%s_01' % s, 'door'),
                                                       ('v_sport1_quadra_turbo_ext01_door_%s_window_01' % s, 'window'),
                                                       ('v_sport1_quadra_turbo_ext01_door_%s_floor_01' % s, 'sill')]),
    'sport2': ('v_sport2_porsche_911turbo', lambda s, sd: [('v_sport2_porsche_911turbo__ext01_door_%s_01' % sd, 'door'),
                                                           ('v_sport2_porsche_911turbo__ext01_window_%s_a_01' % sd, 'window'),
                                                           ('v_sport2_porsche_911turbo__ext01_window_%s_b_01' % sd, 'door')]),
    'utility': ('v_utility4_kaukaz_bratsk', lambda s, sd: [('v_utility4_kaukaz_bratsk__ext01_door_%s_01' % sd, 'door'),
                                                           ('v_utility4_kaukaz_bratsk__ext01_window_%s_01' % sd, 'window')]),
}
SPACING = 0.03


def sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for c in iter(lambda: f.read(1 << 20), b''):
            h.update(c)
    return h.hexdigest().upper()


def surface_points(path, spacing=SPACING):
    """Points on the triangles of every primitive of a glb (barycentric grid, edge length / spacing), voxel-thinned to spacing."""
    gj, b = G.read_glb(path)
    pts = []
    for m in gj['meshes']:
        for p in m['primitives']:
            V = G.accessor(gj, b, p['attributes']['POSITION']).astype(np.float64)
            idx = G.accessor(gj, b, p['indices']).astype(np.int64).reshape(-1, 3) if 'indices' in p else np.arange(len(V)).reshape(-1, 3)
            pts.append(V)  # vertices themselves
            for a, bb, c in V[idx]:
                k = int(np.ceil(max(np.linalg.norm(bb - a), np.linalg.norm(c - a), np.linalg.norm(c - bb)) / spacing))
                if k < 2:
                    continue
                i, j = np.meshgrid(np.arange(k + 1), np.arange(k + 1), indexing='ij')
                m_ = (i + j) <= k
                u, v = i[m_] / k, j[m_] / k
                pts.append(a[None] * (1 - u - v)[:, None] + bb[None] * u[:, None] + c[None] * v[:, None])
    P = np.concatenate(pts)
    key = np.floor(P / (spacing * 0.8)).astype(np.int64)
    _, first = np.unique(key, axis=0, return_index=True)
    return P[np.sort(first)]


def phases(vjson, clip):
    d = json.load(open(vjson, encoding='utf-8'))
    for a in d['Data']['RootChunk']['animations']:
        ad = a['Data']
        an = ad['animation']['Data']
        if an['name']['$value'] != clip:
            continue
        ev = {}
        for e in ad['events']['Data']['events']:
            e = e['Data']
            ev[e['eventName']['$value']] = [int(e['startFrame']), int(e['startFrame'] + e['durationInFrames'])]
        return ev, float(an['duration'])
    raise KeyError(clip)


# --rear: class -> (vehicle .anims stem, {door / window: mesh stem pattern or None}, ent file pattern); {s} = l / r
REAR = {
    'standard2': ('v_standard2_archer_hella', {'door': 'v_standard2_archer_hella__ext01_door_b{s}_01', 'window': 'v_standard2_archer_hella__ext01_window_b{s}_01'},
                  'v_standard2_archer_hella__ext01_door_b{s}_01.ent'),
    'standard25': ('v_standard25_villefort_columbus', {'door': 'v_standard25_villefort_columbus__ext01_door_b{s}_01', 'window': None},
                   'v_standard25_villefort_columbus__ext01_door_b{s}_01.ent'),
    'standard3': ('v_standard3_chevalier_emperor', {'door': 'v_standard3_chevalier_emperor__ext01_door_b{s}_01', 'window': 'v_standard3_chevalier_emperor__ext01_door_window_b{s}_01'},
                  'v_standard3_chevalier_emperor_door_b{s}.ent'),
}
PILLAR_DEPTH_M = 0.30  # SYNTHETIC: how far the B-pillar block reaches inboard of the outer skin (no body mesh extracted)
PILLAR_LEN_M = 0.12  # SYNTHETIC: pillar length along the car, from the front door's rear edge backwards
GLTF_TO_VEHICLE = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])


def rear_main(root):
    out = os.path.join(root, 'rear-data')
    os.makedirs(out, exist_ok=True)
    prov = []
    for cls, (veh, stems, ent) in REAR.items():
        vg = os.path.join(root, 'glb-veh', veh + '.anims.glb')
        sk = S.Skeleton.from_glb(vg)
        rWt, rWr = sk.rest_world()
        dj = json.load(open(os.path.join(root, 'door-data', cls + '.json'), encoding='utf-8'))
        res = {'class': cls, 'vehicle': veh, 'frame': 'static_parts: door-data rig space (glTF Y-up; vehicle x right, y forward, z up = (gx, -gz, gy)); solid_boxes: vehicle axes',
               'side': {}}
        prov.append((vg, sha(vg)))
        for side, s_ in (('left', 'l'), ('right', 'r')):
            parts = []
            for kind, pat in stems.items():
                if pat is None:
                    continue
                stem = pat.format(s=s_)
                g = os.path.join(root, 'glb-rear', stem + '.glb')
                raw = glob.glob(os.path.join(root, 'raw-rear', '**', stem + '.mesh'), recursive=True)
                bone = ('door_back_%s_a' if kind == 'door' else 'window_back_%s_a') % side
                if bone not in sk.index:
                    raise KeyError('%s: rig bone %s missing' % (cls, bone))
                if not os.path.exists(g) or len(raw) != 1:
                    raise FileNotFoundError('%s (glb %s, raw %s)' % (stem, os.path.exists(g), raw))
                P = surface_points(g)
                i = sk.index[bone]
                W = (rWr[i] @ P.T).T + rWt[i]
                parts.append({'mesh': stem, 'kind': kind, 'bone': bone, 'rest_t': rWt[i].tolist(), 'n_points': int(len(W)), 'points': np.round(W, 4).tolist(),
                              'sha256_mesh': sha(raw[0]), 'sha256_glb': sha(g)})
                prov += [(g, sha(g)), (raw[0], sha(raw[0]))]
            entp = glob.glob(os.path.join(root, 'raw-rear', '**', ent.format(s=s_)), recursive=True)
            if len(entp) != 1:
                raise FileNotFoundError(ent.format(s=s_))
            prov.append((entp[0], sha(entp[0])))
            V = (GLTF_TO_VEHICLE @ np.array(parts[0]['points']).T).T  # rear door skin, vehicle frame
            fd = dj['door'][side]['parts'][0]  # front door skin, closed: its rear edge
            fw = (GLTF_TO_VEHICLE @ ((S.q2m(np.array(fd['frames_q'][0])) @ np.array(fd['points']).T).T + np.array(fd['frames_t'][0])).T).T
            y_fd_rear = float(fw[:, 1].min())
            sg = 1.0 if side == 'right' else -1.0
            xs = np.abs(V[:, 0])
            x_in, x_out = float(xs.min()), float(xs.max())
            y_rd_front = float(V[:, 1].max())
            z0, z1 = float(V[:, 2].min()), float(V[:, 2].max())
            # solids: (1) the B-pillar block behind the front door's rear edge (SYNTHETIC depth / length: no body mesh), (2) the rear GLASS slab (extent of the glass mesh:
            # a limb whose axis is inside it has passed through the pane; a surface-distance check calls a limb that is deeper than its radius clear). The rear door
            # SKIN is only a surface cloud: the door is hollow / open above the belt line, a solid door block flags the game's own clips (Hella vanilla 21 cm).
            p_in = sg * (x_out - PILLAR_DEPTH_M)
            boxes = [{'name': 'B-pillar block (SYNTHETIC depth / length)', 'xmin': min(p_in, sg * x_out), 'xmax': max(p_in, sg * x_out), 'ymin': y_fd_rear - PILLAR_LEN_M,
                      'ymax': y_fd_rear, 'zmin': z0, 'zmax': z1}]
            for pt in parts:
                if pt['kind'] == 'window':
                    gv = (GLTF_TO_VEHICLE @ np.array(pt['points']).T).T
                    boxes.append({'name': 'rear glass slab', 'xmin': float(gv[:, 0].min()), 'xmax': float(gv[:, 0].max()), 'ymin': float(gv[:, 1].min()), 'ymax': float(gv[:, 1].max()),
                                  'zmin': float(gv[:, 2].min()), 'zmax': float(gv[:, 2].max())})
            res['side'][side] = {'static_parts': parts, 'solid_boxes': boxes, 'front_door_rear_edge_y': round(y_fd_rear, 4),
                                 'rear_door_front_edge_y': round(y_rd_front, 4), 'outer_skin_x': round(sg * x_out, 4),
                                 'ent': os.path.relpath(entp[0], root), 'sha256_ent': sha(entp[0])}
        with open(os.path.join(out, cls + '.json'), 'w', encoding='utf-8') as f:
            json.dump(res, f, separators=(',', ':'))
        print(cls, 'rear ok', {sd: [(p['bone'], p['n_points']) for p in res['side'][sd]['static_parts']] for sd in res['side']},
              {sd: (res['side'][sd]['front_door_rear_edge_y'], res['side'][sd]['rear_door_front_edge_y']) for sd in res['side']})
    with open(os.path.join(out, 'provenance.txt'), 'w', encoding='ascii') as f:
        for p, h in prov:
            f.write('%s  %s\n' % (h, os.path.relpath(p, root)))


# --body: BODY SHELL data (additive: <root>/body-data). The body mesh (ext01_body_01, static entPhysicalMeshComponent, identity transform, no parent bone) and the interior mesh (int01_interior_01:
# roof lining) are authored in the 'Base' bone frame of the vehicle rig (the car centre): vehicle frame = GLTF_TO_VEHICLE @ glTF + Base bone. Checked per class against the rig's own roof landmark (bone
# roof_border_front: its z against the highest body point at that y, printed) and against the door-data door skin (the closed door's outer x lies at the shell's outer x +- 5 cm).
BODY_CROP_BACK_M = 1.7    # behind the rearmost seat
BODY_CROP_FRONT_M = 1.5   # in front of the foremost seat
BODY_LINING_ABOVE_M = 0.45  # interior mesh: only above the seat (roof lining; seats / console / pedals are not obstacles)
BODY_SPACING = 0.03
# static glass of the body entity (ext01_body_01 ent components window_*; names read from the ents; the door glass is in door-data, the rear DOOR glass in rear-data)
BODY_GLASS = {'standard2': ['ext01_window_f_01', 'ext01_window_b_01', 'ext01_window_bl_02_01', 'ext01_window_br_02_01'], 'standard25': ['ext01_window_f_01'],
              'standard3': ['ext01_windshield_01'], 'sport1': ['ext01_window_a_01', 'ext01_window_b_01', 'ext01_window_c_01', 'ext01_windshield_01'],
              'sport2': ['ext01_window_f_01', 'ext01_window_b_01', 'ext01_window_bl_01', 'ext01_window_br_01'],
              'utility': ['ext01_window_f_a_01', 'ext01_window_f_b_01', 'ext01_window_f_c_01', 'ext01_window_f_d_01']}


def body_main(root):
    out = os.path.join(root, 'body-data')
    os.makedirs(out, exist_ok=True)
    prov = []
    for cls, (veh, _pf) in CLASSES.items():
        vg = os.path.join(root, 'glb-veh', veh + '.anims.glb')
        sk = S.Skeleton.from_glb(vg)
        rWt, rWr = sk.rest_world()
        base = rWt[sk.index['Base']] @ GLTF_TO_VEHICLE.T
        seats = {sd: (rWt[sk.index['seat_front_' + sd]] @ GLTF_TO_VEHICLE.T) for sd in ('left', 'right')}
        sy0 = min(v[1] for v in seats.values())
        sy1 = max(v[1] for v in seats.values())
        sz0 = min(v[2] for v in seats.values())
        lo = np.array([-1.7, sy0 - BODY_CROP_BACK_M, sz0 - 0.45])
        hi = np.array([1.7, sy1 + BODY_CROP_FRONT_M, 99.0])
        res = {'class': cls, 'vehicle': veh, 'frame': 'vehicle axes (x right, y forward, z up) = GLTF_TO_VEHICLE @ glTF mesh + base_bone', 'base_bone': base.tolist(),
               'seat': {sd: v.tolist() for sd, v in seats.items()}, 'crop_lo': lo.tolist(), 'crop_hi': hi.tolist(), 'parts': {}}
        for kind, sufs in (('shell', ['ext01_body_01']), ('lining', ['int01_interior_01']), ('glass', BODY_GLASS[cls])):
            Ps, n_all, names = [], 0, []
            for suf in sufs:
                g = glob.glob(os.path.join(root, 'glb-body', veh + '__' + suf + '.glb'))
                raw = glob.glob(os.path.join(root, 'raw-body', '**', veh + '__' + suf + '.mesh'), recursive=True)
                if len(g) != 1 or len(raw) != 1:
                    raise FileNotFoundError('%s %s (glb %s, raw %s)' % (cls, suf, g, raw))
                P = surface_points(g[0], BODY_SPACING) @ GLTF_TO_VEHICLE.T + base
                n_all += len(P)
                k = np.all((P >= lo) & (P <= hi), axis=1)
                if kind == 'lining':
                    k &= P[:, 2] >= sz0 + BODY_LINING_ABOVE_M
                Ps.append(P[k])
                names.append(suf)
                prov += [(g[0], sha(g[0])), (raw[0], sha(raw[0]))]
            P = np.concatenate(Ps)
            res['parts'][kind] = {'meshes': [veh + '__' + n for n in names], 'n_points_all': int(n_all), 'n_points': int(len(P)), 'points': np.round(P, 4).tolist()}
        sh = np.array(res['parts']['shell']['points'])
        rb = sk.index.get('roof_border_front')
        note = ''
        if rb is not None:
            rv = rWt[rb] @ GLTF_TO_VEHICLE.T
            near = sh[(np.abs(sh[:, 1] - rv[1]) < 0.3) & (np.abs(sh[:, 0]) < 0.4)]
            note = 'roof_border_front bone z %.3f vs shell max z within 0.3 m of its y: %.3f' % (rv[2], near[:, 2].max() if len(near) else float('nan'))
        dj = json.load(open(os.path.join(root, 'door-data', cls + '.json'), encoding='utf-8'))
        dp = dj['door']['left']['parts'][0]
        W = ((q2m_(dp['frames_q'][0]) @ np.array(dp['points']).T).T + np.array(dp['frames_t'][0])) @ GLTF_TO_VEHICLE.T
        res['check'] = {'roof_border': note, 'shell_x_max_abs': float(np.abs(sh[:, 0]).max()), 'door_skin_x_max_abs': float(np.abs(W[:, 0]).max()),
                        'shell_z_max': float(sh[:, 2].max()), 'lining_z_max': float(np.array(res['parts']['lining']['points'])[:, 2].max()) if res['parts']['lining']['n_points'] else None}
        with open(os.path.join(out, cls + '.json'), 'w', encoding='utf-8') as f:
            json.dump(res, f, separators=(',', ':'))
        print(cls, 'body ok', {k: v['n_points'] for k, v in res['parts'].items()}, res['check'])
    with open(os.path.join(out, 'provenance.txt'), 'w', encoding='ascii') as f:
        for p, h in prov:
            f.write('%s  %s\n' % (h, os.path.relpath(p, root)))


def q2m_(q):
    return S.q2m(np.asarray(q))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=os.path.join(project_root(), r'Extracted\game\vehicle-entry-car'))
    ap.add_argument('--out')
    ap.add_argument('--body', action='store_true', help='additive: body shell + roof lining data of the six classes -> <root>/body-data (extract_door_assets.py --body first)')
    ap.add_argument('--rear', action='store_true', help='additive: rear door / glass / B-pillar data of the 4-door classes -> <root>\\rear-data (extract_door_assets.py --rear first)')
    a = ap.parse_args()
    if a.body:
        return body_main(a.root)
    if a.rear:
        return rear_main(a.root)
    out = a.out or os.path.join(a.root, 'door-data')
    os.makedirs(out, exist_ok=True)
    prov = []
    for cls, (veh, partsf) in CLASSES.items():
        vg = os.path.join(a.root, 'glb-veh', veh + '.anims.glb')
        vj = os.path.join(a.root, 'json-veh', veh + '.anims.json')
        if not os.path.exists(vj):
            vj = os.path.join(a.root, 'json', veh + '.anims.json')
        sk = S.Skeleton.from_glb(vg)
        rWt, rWr = sk.rest_world()
        res = {'class': cls, 'vehicle': veh, 'vehicle_glb': vg, 'seat': {}, 'door': {}}
        prov.append((vg, sha(vg)))
        prov.append((vj, sha(vj)))
        for side, sd, sn in (('left', 'fl', 'front_left'), ('right', 'fr', 'front_right')):
            i = sk.index['seat_' + sn]
            res['seat'][side] = {'t': rWt[i].tolist(), 'R': rWr[i].reshape(-1).tolist()}
            ev, dur = phases(vj, 'door_' + sn)
            n = ev['loop'][1] + 1
            t = np.arange(n) / 30.0
            lt, lr = sk.sample_local(sk.clip('door_' + sn), t)
            Wt, Wr = sk.world(lt, lr)
            parts = []
            for stem, kind in partsf('l' if side == 'left' else 'r', sd):
                g = glob.glob(os.path.join(a.root, 'glb-doors', '**', stem + '.glb'), recursive=True)
                if not g:
                    raise FileNotFoundError(stem)
                prov.append((g[0], sha(g[0])))
                bone = {'door': 'door_%s_a' % sn, 'window': 'window_%s_a' % sn, 'sill': 'door_%s_c' % sn}[kind]
                bi = sk.index[bone]
                P = surface_points(g[0])
                parts.append({'mesh': stem, 'bone': bone, 'n_points': int(len(P)), 'points': np.round(P, 4).tolist(),
                              'frames_t': Wt[:, bi].tolist(), 'frames_q': S.m2q(Wr[:, bi]).tolist()})
            res['door'][side] = {'clip': 'door_' + sn, 'clip_duration_s': dur, 'phase_frames': ev, 'parts': parts}
        with open(os.path.join(out, cls + '.json'), 'w', encoding='utf-8') as f:
            json.dump(res, f, separators=(',', ':'))
        print(cls, 'ok', {s: [(p['bone'], p['n_points']) for p in res['door'][s]['parts']] for s in res['door']})
    with open(os.path.join(out, 'provenance.txt'), 'w', encoding='ascii') as f:
        for p, h in prov:
            f.write('%s  %s\n' % (h, os.path.relpath(p, a.root)))


if __name__ == '__main__':
    main()
