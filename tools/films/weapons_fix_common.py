"""Rifle / 2H hand fix 2026-11-09 - shared offline helpers (run with Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I).

Reads the overlay layer file (.utppov, format Source\\tools\\mmbake\\FORMAT_OVERLAY.txt) read-only, computes the melee anatomy-gate
features (Source\\tools\\melee\\anatomy_gate.py, unchanged) of 73-bone overlay-skeleton locals, and finger curl angles.
Writes nothing by itself; callers write into Build\\work\\rifle-hand-fix-20261109.
"""
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import weapons_common as WC  # noqa: E402

sys.path.insert(0, os.path.join(WC.ROOT, 'Source', 'tools', 'melee'))
import npc_build as NB  # noqa: E402

WORK = os.path.join(WC.ROOT, 'Build', 'work', 'rifle-hand-fix-20261109')
FINGERS_L = ['LeftHandThumb1', 'LeftHandThumb2', 'LeftHandIndex1', 'LeftHandIndex2', 'LeftHandIndex3', 'LeftHandMiddle1', 'LeftHandMiddle2',
             'LeftHandMiddle3', 'LeftHandRing1', 'LeftHandRing2', 'LeftHandRing3', 'LeftHandPinky1', 'LeftHandPinky2', 'LeftHandPinky3']
ARM_FEATS = ['wrist_twist_L', 'forearm_roll_L', 'pronation_L', 'uparm_twist_L', 'elbow_flex_L', 'elbow_dev_L', 'wrist_flex_L', 'wrist_dev_L',
             'wrist_twist_R', 'pronation_R', 'elbow_flex_R']


class Utppov:
    """read-only .utppov reader (version 1)."""

    def __init__(self, path):
        b = open(path, 'rb').read()
        assert b[:6] == b'UTPPOV', path
        self.nb = struct.unpack_from('<I', b, 24)[0]
        nsec = struct.unpack_from('<I', b, 28)[0]
        sec = {}
        for i in range(nsec):
            tag, off, size, cnt = struct.unpack_from('<4sIII', b, 64 + 16 * i)
            sec[tag.decode()] = (off, size, cnt)
        off, _, cnt = sec['BONE']
        self.names, self.par = [], []
        for i in range(cnt):
            nm, p = struct.unpack_from('<32si', b, off + 36 * i)
            self.names.append(nm.split(b'\0')[0].decode())
            self.par.append(p)
        off, _, cnt = sec['CURV']
        self.curves = [struct.unpack_from('<48s', b, off + 48 * i)[0].split(b'\0')[0].decode() for i in range(cnt)]
        C = len(self.curves)
        off, _, cnt = sec['POSE']
        self.poses = []
        for i in range(cnt):
            nm, grp, ff, nf, add, basep, basef, basek, _r, cmask = struct.unpack_from('<64sIIIIiIIIQ', b, off + 104 * i)
            self.poses.append(dict(name=nm.split(b'\0')[0].decode(), group=grp, first=ff, frames=nf, additive=add, base=basep, cmask=cmask))
        off, _, cnt = sec['FRAM']
        per = self.nb * 17 + C
        self.fram = np.frombuffer(b, dtype='<f4', count=cnt * per, offset=off).reshape(cnt, per).astype(float)
        off, _, cnt = sec['OVLY']
        self.ovly = []
        for i in range(cnt):
            nm, kind, flags, fp, npose = struct.unpack_from('<32sIIII', b, off + 48 * i)
            self.ovly.append(dict(name=nm.split(b'\0')[0].decode(), kind=kind, first=fp, count=npose))
        self.ix = {p['name']: i for i, p in enumerate(self.poses)}

    def frame(self, pose, k=0):
        p = self.poses[self.ix[pose]] if isinstance(pose, str) else self.poses[pose]
        row = self.fram[p['first'] + k]
        bones = row[:self.nb * 17].reshape(self.nb, 17)
        cur = {c: float(row[self.nb * 17 + i]) for i, c in enumerate(self.curves) if (p['cmask'] >> i) & 1}
        return bones[:, 0:3].copy(), bones[:, 3:7].copy(), cur


def world(lt73, lr73):
    lt, lr = WC.full152(np.asarray(lt73, float), np.asarray(lr73, float))
    return WC.fk(lt, lr)


def feats(W):
    """W [T,152|73,4,4] -> gate feature dict (anatomy_gate.features via the melee base-body rig)."""
    g = NB.GB.frame_gate(np.asarray(W, float)[:, :73], NB.RG, steps=False, al=NB.AL)
    return g['feat']


def finger_curl(lr73, names, ref_lr=None):
    """per frame: angle (deg) of each left finger local rotation vs the player reference local (open hand = rest)."""
    P = WC.player()
    ref = P.ref['rest_lr'] if ref_lr is None else ref_lr
    out = {}
    for n in FINGERS_L:
        j = names.index(n)
        q = np.asarray(lr73, float)[:, j]
        r = np.asarray(ref, float)[P.ix[n]]
        d = np.abs(np.sum(q * r[None], 1))
        out[n] = np.degrees(2 * np.arccos(np.clip(d, -1, 1)))
    return out


AJAX_DIR = os.path.join(WC.ROOT, 'Extracted', 'game', 'weapon-mesh-ajax-20261109', 'base', 'weapons', 'firearms', 'rifle_assault',
                        'militech_ajax', 'entities', 'meshes')
AJAX_PARTS = ('w_rifle_assault__militech_ajax__base1_01.glb', 'w_rifle_assault__militech_ajax__mag_std_01.glb')


def ajax_mesh():
    """real Ajax mesh (base + standard magazine, WolvenKit glb) in the weapon frame, game axes (x right, y barrel, z up) -> (V [n,3], F [m,3])."""
    import weapons_fix_mesh as MESH
    Vs, Fs, off = [], [], 0
    for f in AJAX_PARTS:
        js, binb = MESH.load(os.path.join(AJAX_DIR, f))
        for m in js['meshes']:
            for p in m['primitives']:
                v = MESH.accessor(js, binb, p['attributes']['POSITION'])
                Vs.append(np.stack([v[:, 0], -v[:, 2], v[:, 1]], 1))
                Fs.append(MESH.accessor(js, binb, p['indices']).reshape(-1, 3).astype(np.int64) + off)
                off += len(v)
    return np.concatenate(Vs), np.concatenate(Fs)


def gun_cache_mesh(W, base, V, F):
    """mesh in the WeaponRight frame -> world per frame: <base>.npz (rest, faces) + <base>.pc2 (weapons_film_render.py --guns)."""
    M = W[:, WC.player().ix['WeaponRight']]
    Vv = (np.einsum('tij,nj->tni', M[:, :3, :3], V) + M[:, None, :3, 3]).astype('<f4')
    np.savez(base + '.npz', rest=Vv[0], faces=np.asarray(F, np.int32))
    with open(base + '.pc2', 'wb') as fh:
        fh.write(b'POINTCACHE2\0' + struct.pack('<iiffi', 1, Vv.shape[1], 0.0, 1.0, Vv.shape[0]))
        fh.write(Vv.tobytes())
    return base


def render(W, name, res, cam, tz, lens, gun, labels=None, title='', follow=1, extra=(), color='0.25,0.62,0.45', title_size='0.05'):
    """skinned Female V + gun (('mesh', V, F) real mesh or a proxy box list) -> mkv in WORK\\_tmp (weapons_film.render pipeline)."""
    import subprocess
    import weapons_film as F
    tmp = os.path.join(WORK, '_tmp')
    os.makedirs(tmp, exist_ok=True)
    base = os.path.join(tmp, name)
    WC.NR.skin_W(W[:, :73], WC.player().ref['rest'], 'woman', base)
    if isinstance(gun, tuple) and gun[0] == 'mesh':
        gb = gun_cache_mesh(W, base + '_gun', gun[1], gun[2])
    else:
        gb = F.gun_cache(W, base + '_gun', gun)
    lj = base + '.labels.json'
    import json
    json.dump(labels if labels is not None else [''] * len(W), open(lj, 'w', encoding='utf-8'), ensure_ascii=False)
    mkv = base + '.mkv'
    cmd = [WC.NR.BLENDER, '-b', '--factory-startup', '-P', os.path.join(HERE, 'weapons_film_render.py'), '--', base, mkv, lj, '--res', res,
           '--follow', str(follow), '--title', title, '--color', color, '--cam', cam, '--guns', gb, '--tz', str(tz), '--lens', str(lens),
           '--title-size', title_size, '--title-align', 'center', '--label-size', '0.045'] + list(extra)
    p = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace')
    if 'RENDERED' not in p.stdout:
        raise RuntimeError('blender failed: ' + (p.stdout + p.stderr)[-1500:])
    return mkv


def summ(x):
    x = np.asarray(x, float).reshape(-1)
    return [round(float(x.min()), 1), round(float(np.median(x)), 1), round(float(x.max()), 1)]
