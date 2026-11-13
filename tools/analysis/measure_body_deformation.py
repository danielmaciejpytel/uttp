"""Offline skin-deformation metrics of V body meshes (female pwa / male pma) in the key poses, same pipeline for both bodies.

python -I measure_body_deformation.py --body woman|man --out <json> [--dump <dir> --cases label:clip:frame,...] [--poses all]

Pipeline (identical for both bodies, only --body differs): retargeted .utppclip.json core pose -> deformation rig FK
(player_woman_base_deformations / player_man_fpp_deformations reference locals) -> the game's deformer animgraph evaluated offline
(evaluate_body_deformer_graph.Evaluator, --strict: any graph error aborts) -> linear-blend skin of body + arms_hq l/r + head
(skin_clip.py helpers). Output = geometry only: no overlay composition, AimOffset, IK, BodyHelperRuntime correction (APPLY), clothing.
Rest reference = mesh bind positions (A-pose). The REF row skins the rig reference pose (all core locals = rig reference): the bind is an
A-pose and the rig reference is not, so REF already shows armpit/shoulder stretch; pose rows mean something relative to REF.

Metrics per region (vertex -> joint with the largest weight; elbow/knee limited to a radius around the bind joint):
  edge_max, edge_p99   per-edge length ratio pose/bind (stretch; only edges whose bind length > 1e-4 m)
  edge_min             smallest ratio (compression)
  area_min             smallest triangle area ratio pose/bind (collapse)
  sev, sev_p999        severity = max(edge_max, 1/edge_min) / same on the 99.9 / 0.1 percentiles (ranking key: robust to single bad edges)
  all pose rows are REF-relative (pose/REF edge and area ratios); sev_bind = severity against the raw mesh bind
  neck_head            (neck region) Neck->Head joint distance pose/REF
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'ai_motion'))
import skin_clip as SK  # noqa: E402  (also puts vehicle-entry and analysis on sys.path)

SC = SK.SC
RETARGET = {'woman': os.path.join(ROOT, 'Animations', 'retarget', 'out_sourcepack_woman'),
            'man': os.path.join(ROOT, 'Animations', 'retarget', 'out_sourcepack_man')}
REGIONS = {
    'shoulder': dict(pat=('Shoulder', 'deltoid', 'scapula', 'butterfly', 'chest_front', 'latissimus', 'SHL', 'LeftArm', 'RightArm'), joint=None),
    'elbow': dict(pat=('ForeArm', 'elbow', 'biceps', 'triceps'), joint=('LeftForeArm', 'RightForeArm'), radius=0.10),
    'neck': dict(pat=('Neck', 'neck', 'throat'), joint=None),
    'knee': dict(pat=('knee', 'calf', 'LeftLeg', 'RightLeg', 'lowLeg', 'thigh', 'leg_back'), joint=('LeftLeg', 'RightLeg'), radius=0.12),
}

# label -> (clip, frame selector). 'all' = every frame of a short pose clip, int = N evenly spaced samples of a loop
POSES = [
    ('P1H_stand_relax(low)', 'Pose_Pistol1H_Stand_Relax_Idle', 'all'),
    ('P1H_stand_ready', 'Pose_Pistol1H_Stand_Ready_Idle', 'all'),
    ('P1H_stand_aim', 'Pose_Pistol1H_Stand_Aim_Idle', 'all'),
    ('P2H_stand_relax(low)', 'Pose_Pistol2H_Stand_Relax_Idle', 'all'),
    ('P2H_stand_ready', 'Pose_Pistol2H_Stand_Ready_Idle', 'all'),
    ('P2H_stand_aim', 'Pose_Pistol2H_Stand_Aim_Idle', 'all'),
    ('Rifle_stand_relax', 'Pose_Rifle_Stand_Relax_Idle', 'all'),
    ('Rifle_stand_ready', 'Pose_Rifle_Stand_Ready_Idle', 'all'),
    ('Rifle_stand_aim', 'Pose_Rifle_Stand_Aim_Idle', 'all'),
    ('P1H_crouch_ready', 'Pose_Pistol1H_Crouch_Ready', 'all'),
    ('P1H_crouch_aim', 'Pose_Pistol1H_Crouch_Aim', 'all'),
    ('P2H_crouch_relax', 'Pose_Pistol2H_Crouch_Relax', 'all'),
    ('P2H_crouch_ready', 'Pose_Pistol2H_Crouch_Ready', 'all'),
    ('P2H_crouch_aim', 'Pose_Pistol2H_Crouch_Aim', 'all'),
    ('Rifle_crouch_aim', 'Pose_Rifle_Crouch_Aim', 'all'),
    ('Crouch_idle_pose', 'Pose_Neutral_Crouch_Idle', 'all'),
    ('Crouch_idle_loop', 'M_Neutral_Crouch_Idle_Loop', 6),
    ('Crouch_walk_loop', 'M_Neutral_Crouch_Loop_F', 8),
    ('Walk_loop', 'M_Neutral_Walk_Loop_F', 8),
    ('Run_loop', 'M_Neutral_Run_Loop_F', 8),
    ('Sprint_loop', 'M_Neutral_Sprint_Loop_F', 8),
    ('AO_P1H_stand_sweep', 'AO_Pistol1H_Stand_Sweep', 8),
    ('AO_P2H_stand_sweep', 'AO_Pistol2H_Stand_Sweep', 8),
    ('AO_Rifle_stand_sweep', 'AO_Rifle_Stand_Sweep', 8),
    ('AO_P1H_crouch_sweep', 'AO_Pistol1H_Crouch_Sweep', 8),
    ('AO_P2H_crouch_sweep', 'AO_Pistol2H_Crouch_Sweep', 8),
    ('AO_Rifle_crouch_sweep', 'AO_Rifle_Crouch_Sweep', 8),
]


class Rig:
    def __init__(self, body):
        self.body = body
        self.names, self.par, self.ref = SK.load_rig(SK.DEFORM_RIG_BY_BODY[body])
        self.idx = {n: i for i, n in enumerate(self.names)}
        from evaluate_body_deformer_graph import Evaluator
        self.ev = Evaluator(allow_rotation_bounce=True, body=body)
        self.parts = SC.load_meshes(body)
        self.binds, self.faces, off = [], [], 0
        self.pos, self.dom, self.part_of = [], [], []
        self.domname = []
        for k, p in enumerate(self.parts):
            self.faces.append(p['faces'] + off)
            off += len(p['pos'])
            b = SK.bindings(p, self.idx, p['name'].startswith('h0_'))
            self.binds.append(([n for n, _ in b], np.stack([m for _, m in b])))
            self.pos.append(p['pos'])
            d = p['js'][np.arange(len(p['js'])), p['ws'].argmax(1)]
            self.domname += [p['jn'][j] for j in d]
            self.part_of += [k] * len(p['pos'])
        self.faces = np.concatenate(self.faces)
        self.pos = np.concatenate(self.pos)
        self.part_of = np.array(self.part_of)
        self.domname = np.array(self.domname)
        # unique edges, bind lengths, triangle areas
        e = np.concatenate([self.faces[:, [0, 1]], self.faces[:, [1, 2]], self.faces[:, [2, 0]]])
        e = np.unique(np.sort(e, axis=1), axis=0)
        L0 = np.linalg.norm(self.pos[e[:, 0]] - self.pos[e[:, 1]], axis=1)
        keep = L0 > 1e-4
        self.edges, self.L0 = e[keep], L0[keep]
        self.A0 = self.area(self.pos)
        # regions: vertex masks; elbow/knee limited around the bind joint (glb frame)
        self.region = {}
        p0 = self.parts[0]
        for r, spec in REGIONS.items():
            m = np.array([any(s in n for s in spec['pat']) for n in self.domname])
            if spec.get('joint'):
                near = np.zeros(len(m), bool)
                for jn in spec['joint']:
                    jp = p0['nodew'][p0['node_of'][jn]][:3, 3]
                    near |= np.linalg.norm(self.pos - jp, axis=1) < spec['radius']
                m &= near
            self.region[r] = m
        # seams: coincident bind vertices of different parts (torso vs arms, torso vs head)
        self.seams = {}
        try:
            from scipy.spatial import cKDTree
            for name, (a, b) in {'seam_arm_torso': ((1, 2), (0,)), 'seam_head_torso': ((3,), (0,))}.items():
                ia = np.where(np.isin(self.part_of, a))[0]
                ib = np.where(np.isin(self.part_of, b))[0]
                d, j = cKDTree(self.pos[ib]).query(self.pos[ia])
                ok = d < 1e-5
                if ok.sum() > 0:
                    self.seams[name] = (ia[ok], ib[j[ok]])
        except Exception:  # noqa: BLE001 - seams are optional
            pass

    def area(self, v):
        a, b, c = v[self.faces[:, 0]], v[self.faces[:, 1]], v[self.faces[:, 2]]
        return 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)

    def skin(self, core):
        W = SK.fk_rigid(self.names, self.par, self.ref, core)
        M = self.ev.run({n: m for n, m in core.items() if n in self.ev.coreNames})
        W.update({n: m for n, m in M.items() if n in W})
        vs = []
        for p, (jn, pre) in zip(self.parts, self.binds):
            Wl = np.stack([SK.C @ W[n] @ SK.CI for n in jn])
            skin = Wl @ pre
            ph = np.c_[p['pos'], np.ones(len(p['pos']))]
            v = np.zeros((len(ph), 3))
            for c in range(p['js'].shape[1]):
                v += p['ws'][:, c:c + 1] * np.einsum('vij,vj->vi', skin[p['js'][:, c]], ph)[:, :3]
            vs.append(v)
        return np.concatenate(vs), W

    def ref_core(self):
        return {n: self.ref[self.idx[n]] for n in self.ev.coreNames if n in self.idx}

    def metrics(self, v, W, refW, refstate=None):
        """refstate=(edge ratio, area ratio) of the REF pose: when given, the headline numbers are REF-relative (pose/REF), the
        bind-relative severity is kept as sev_bind. Without it (REF row itself) everything is bind-relative."""
        rb = np.linalg.norm(v[self.edges[:, 0]] - v[self.edges[:, 1]], axis=1) / self.L0
        arb = self.area(v) / np.maximum(self.A0, 1e-12)
        ok_area = self.A0 > 1e-9
        if refstate is None:
            r, ar = rb, arb
        else:
            r = rb / refstate[0]
            ar = arb / np.maximum(refstate[1], 1e-9)
            ok_area = ok_area & (refstate[1] > 1e-3)
        out = {}
        for name, m in self.region.items():
            em = m[self.edges[:, 0]] & m[self.edges[:, 1]]
            fm = m[self.faces].all(axis=1) & ok_area
            rr = r[em]
            o = dict(n=int(m.sum()), edge_max=float(rr.max()), edge_p999=float(np.percentile(rr, 99.9)), edge_p99=float(np.percentile(rr, 99)),
                     edge_min=float(rr.min()), area_min=float(ar[fm].min()) if fm.any() else None)
            o['sev'] = max(o['edge_max'], 1.0 / max(o['edge_min'], 1e-6))
            o['sev_p999'] = max(o['edge_p999'], 1.0 / max(float(np.percentile(rr, 0.1)), 1e-6))
            rbr = rb[em]
            dcm = (rb[em] - (refstate[0][em] if refstate is not None else 1.0)) * self.L0[em] * 100.0  # absolute edge length change vs REF (bind for the REF row), cm
            o['stretch_cm_max'] = float(dcm.max())
            o['stretch_cm_p999'] = float(np.percentile(dcm, 99.9))
            o['compress_cm_max'] = float(-dcm.min())
            o['sev_bind'] = max(float(rbr.max()), 1.0 / max(float(rbr.min()), 1e-6))
            out[name] = o
        d = lambda Wd: float(np.linalg.norm(Wd['Head'][:3, 3] - Wd['Neck'][:3, 3]))  # noqa: E731
        out['neck']['neck_head'] = d(W) / d(refW)
        for name, (ia, ib) in self.seams.items():
            g = np.linalg.norm(v[ia] - v[ib], axis=1)
            out[name] = dict(n=len(ia), gap_max=float(g.max()), gap_p99=float(np.percentile(g, 99)))
        return out, r, (rb, arb)

    def vertex_stretch(self, r):
        s = np.ones(len(self.pos))
        lr = np.maximum(r, 1.0 / np.maximum(r, 1e-6))
        np.maximum.at(s, self.edges[:, 0], lr)
        np.maximum.at(s, self.edges[:, 1], lr)
        return s


def frames_for(sel, n):
    if sel == 'all':
        return list(range(n))
    return sorted({int(round(x)) for x in np.linspace(0, n - 1, sel, endpoint=False)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--body', default='woman', choices=['woman', 'man'])
    ap.add_argument('--out', required=True)
    ap.add_argument('--dump', help='directory for per-case npz (verts, faces, vertex stretch, region masks, joints)')
    ap.add_argument('--cases', help='label:clip:frame,... for --dump (clip without .utppclip.json)')
    ap.add_argument('--clips-suffix', default='', help="clip set: '' = out_sourcepack_<body> (default), '_knee' = out_sourcepack_<body>_knee (the installed MM databases are built from the knee set)")
    a = ap.parse_args()
    RETARGET[a.body] = RETARGET[a.body] + a.clips_suffix
    rig = Rig(a.body)
    refv, refW = rig.skin(rig.ref_core())
    ref_m, _, refstate = rig.metrics(refv, refW, refW)
    result = dict(body=a.body, graph=str(rig.ev.graphPath), rig=str(rig.ev.rigPath), verts=int(len(rig.pos)), edges=int(len(rig.edges)),
                  seams={k: int(len(v[0])) for k, v in rig.seams.items()}, ref=ref_m, poses=[])
    if a.dump:
        os.makedirs(a.dump, exist_ok=True)
        want = [c.split(':') for c in a.cases.split(',')]
        for label, clip, fr in want:
            core = SK.clip_cores(json.load(open(os.path.join(RETARGET[a.body], clip + '.utppclip.json'), encoding='utf-8')), set(rig.names))[int(fr)]
            v, W = rig.skin(core)
            m, r, _ = rig.metrics(v, W, refW, refstate)
            st = rig.vertex_stretch(r)
            np.savez(os.path.join(a.dump, '%s_%s.npz' % (a.body, label)), verts=np.stack([v[:, 0], -v[:, 2], v[:, 1]], 1).astype(np.float32),
                     faces=rig.faces.astype(np.int32), stretch=st.astype(np.float32),
                     bind=np.stack([rig.pos[:, 0], -rig.pos[:, 2], rig.pos[:, 1]], 1).astype(np.float32),
                     **{'mask_' + k: x for k, x in rig.region.items()},
                     joints=np.array([W[n][:3, 3] for n in SK.CHECK]), joint_names=np.array(SK.CHECK), domname=rig.domname.astype(str),
                     part_of=rig.part_of, edges=rig.edges.astype(np.int32), edge_ratio=r.astype(np.float32), metrics=json.dumps(m))
            print('dumped', a.body, label, clip, fr)
        return
    for label, clip, sel in POSES:
        path = os.path.join(RETARGET[a.body], clip + '.utppclip.json')
        if not os.path.exists(path):
            result['poses'].append(dict(label=label, clip=clip, missing=True))
            continue
        cores = SK.clip_cores(json.load(open(path, encoding='utf-8')), set(rig.names))
        per = []
        for f in frames_for(sel, len(cores)):
            v, W = rig.skin(cores[f])  # strict: a graph error propagates
            m, _, _ = rig.metrics(v, W, refW, refstate)
            per.append(dict(frame=f, m=m))
        worst = {}
        for reg in REGIONS:
            b = max(per, key=lambda x: x['m'][reg]['sev_p999'])
            worst[reg] = dict(frame=b['frame'], **b['m'][reg])
        gaps = {}
        for k in rig.seams:
            b = max(per, key=lambda x: x['m'][k]['gap_max'])
            gaps[k] = dict(frame=b['frame'], **b['m'][k])
        neck_head = [x['m']['neck']['neck_head'] for x in per]
        result['poses'].append(dict(label=label, clip=clip, frames=len(per), worst=worst, seams=gaps,
                                    neck_head_min=min(neck_head), neck_head_max=max(neck_head)))
        print(a.body, label, ' '.join('%s=%.2f@%d' % (k, v['sev_p999'], v['frame']) for k, v in worst.items()), flush=True)
    json.dump(result, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
