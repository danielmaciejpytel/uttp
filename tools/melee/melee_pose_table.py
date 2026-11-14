"""Builds the melee full pose tables: one binary file per body (utpp_melee_pose_female.bin, utpp_melee_pose_male.bin) that the DLL
loads from <plugin dir>\\data\\ (mm/MeleePose.hpp, MeleePoseRuntime). Each weapon (keyed by the anim variable of the drawn weapon,
e.g. w_melee_fists) has its own section of clips; every row = one frame of a standing clip:
  * the 13 key bone local rotations + the clip's Spine3 model orientation (the same layout as the chest correction table), used to
    recognise the clip frame the game plays;
  * the clip's lower body: local rotations of Hips, Spine and both legs (UpLeg, Leg, Foot, ToeBase) and the Hips local translation.

usage: python -I melee_pose_table.py --in DIR [--in DIR ...] --out DIR
Inputs: <dir>/<prefix>_<weapon>__<clip>.npc1.npz (prefix pwa = Female V, pma = Male V). Arrays used: names152, parents152, lr, lt
(local rotations / translations), names + W (model matrices, metres), fps.
"""
import argparse
import glob
import hashlib
import os
import struct
import sys

import numpy as np

KEYS = ['Neck', 'Head', 'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand', 'LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand',
        'Spine1', 'Spine2', 'Spine3']
LOWER = ['Hips', 'Spine', 'LeftUpLeg', 'LeftLeg', 'LeftFoot', 'RightUpLeg', 'RightLeg', 'RightFoot']
# clips that the game plays on bases the mod owns (sprint / crouch / slide / air) or that move the character away from a standing
# stance; matched as substrings of the clip name
EXCLUDE = ('crouch', 'slide', 'sprint', 'air_', 'groundslam', 'offscreen', 'throw', 'jump', 'land', 'fall', 'dodge')
# weapon key in the file name -> anim variable of the weapon
VARIABLES = {
    'fists': 'w_melee_fists',
    'knife': 'w_melee_knife',
    'katana': 'w_melee_katana',
    'one_handed_blunt': 'w_melee_one_hand_blunt',
    'baton': 'w_melee_baton',
    'mantis_blade': 'mantis_blade',
    'two_handed_blunt': 'w_two_hand_blunt',
}
BODIES = (('female', 'pwa'), ('male', 'pma'))
MAGIC = b'UTPPMPOS'
FORMAT = 1
NAME_BYTES = 40
LOWER_INTS = len(LOWER) * 4 + 4  # quaternions + hips translation (mm) + pad


def cname_hash(s):
    h = 0xCBF29CE484222325
    for c in s.encode('utf-8'):
        h ^= c
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def quat_from_matrix(m):
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q, dtype=np.float64)
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def enc(q):
    q = np.asarray(q, dtype=np.float64)
    q = q / np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return [int(round(v * 32767.0)) for v in q]


def collect(dirs, prefix):
    """{weapon key: [(clip name, path)]}, the first directory that has a clip wins (later directories add weapons / clips)"""
    out = {}
    for d in dirs:
        for f in sorted(glob.glob(os.path.join(d, prefix + '_*__*.npc1.npz'))):
            base = os.path.basename(f)
            weapon = base.split('__')[0][len(prefix) + 1:]
            clip = base.split('__')[1].split('.')[0]
            if weapon not in VARIABLES or any(x in clip for x in EXCLUDE):
                continue
            lst = out.setdefault(weapon, [])
            if all(c != clip for c, _ in lst):
                lst.append((clip, f))
    return out


def check_parents(z, path):
    n152, p152 = list(z['names152']), list(z['parents152'])
    par = {n: (n152[p] if p >= 0 else None) for n, p in zip(n152, p152)}
    want = {'Spine': 'Hips', 'LeftUpLeg': 'Hips', 'RightUpLeg': 'Hips', 'LeftLeg': 'LeftUpLeg', 'RightLeg': 'RightUpLeg',
            'LeftFoot': 'LeftLeg', 'RightFoot': 'RightLeg', 'Spine1': 'Spine'}
    for b, p in want.items():
        if par.get(b) != p:
            raise SystemExit('%s: parent of %s is %s, expected %s' % (path, b, par.get(b), p))
    return par['Hips']


def build_body(dirs, prefix):
    weapons = collect(dirs, prefix)
    order = [w for w in VARIABLES if w in weapons]
    keys, lower, clip_local, clips, wtab, digests = [], [], [], [], [], []
    hips_parent = set()
    for w in order:
        first_clip, first_row = len(clips), len(keys)
        if len(weapons[w]) > 255:
            raise SystemExit('%s %s: more than 255 clips' % (prefix, w))
        for li, (clip, f) in enumerate(weapons[w]):
            with open(f, 'rb') as fh:
                digests.append((os.path.basename(f), hashlib.sha256(fh.read()).hexdigest()))
            z = np.load(f, allow_pickle=False)
            hips_parent.add(check_parents(z, f))
            n152, n73 = list(z['names152']), list(z['names'])
            lr, lt, W = z['lr'].astype(np.float64), z['lt'].astype(np.float64), z['W'].astype(np.float64)
            r0 = len(keys)
            for i in range(lr.shape[0]):
                row = []
                for k in KEYS:
                    row += enc(lr[i, n152.index(k)])
                row += enc(quat_from_matrix(W[i, n73.index('Spine3'), :3, :3]))
                keys.append(row)
                lrow = []
                for b in LOWER:
                    lrow += enc(lr[i, n152.index(b)])
                p = lt[i, n152.index('Hips')]
                lrow += [int(round(float(v) * 1000.0)) for v in p] + [0]
                lower.append(lrow)
                clip_local.append(li)
            clips.append((r0, len(keys) - r0, clip[:NAME_BYTES - 1]))
        wtab.append((cname_hash(VARIABLES[w]), first_clip, len(clips) - first_clip, first_row, len(keys) - first_row, w))
    return wtab, clips, np.array(keys, dtype=np.int16), np.array(lower, dtype=np.int16), np.array(clip_local, dtype=np.uint8), digests, hips_parent


def write(path, body_index, wtab, clips, keys, lower, clip_local):
    rows = keys.shape[0]
    hdr = MAGIC + struct.pack('<9I', FORMAT, body_index, len(KEYS), len(LOWER), len(wtab), len(clips), rows, 30, 0)
    hdr += b'\0' * (64 - len(hdr))
    out = bytearray(hdr)
    for h, fc, nc, fr, nr, _ in wtab:
        out += struct.pack('<QIIII', h, fc, nc, fr, nr)
    for r0, n, name in clips:
        out += struct.pack('<II', r0, n) + name.encode('ascii').ljust(NAME_BYTES, b'\0')
    out += keys.astype('<i2').tobytes()
    out += lower.astype('<i2').tobytes()
    out += clip_local.tobytes()
    while len(out) % 4:
        out += b'\0'
    with open(path, 'wb') as fh:
        fh.write(out)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='dirs', action='append', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    for bi, (body, prefix) in enumerate(BODIES):
        wtab, clips, keys, lower, clip_local, digests, hp = build_body([os.path.abspath(d) for d in a.dirs], prefix)
        if not wtab:
            print(body, 'no clips')
            continue
        path = os.path.join(a.out, 'utpp_melee_pose_%s.bin' % body)
        write(path, bi, wtab, clips, keys, lower, clip_local)
        with open(path, 'rb') as fh:
            sha = hashlib.sha256(fh.read()).hexdigest()
        print(body, 'hips parent', sorted(hp), 'rows', keys.shape[0], 'clips', len(clips), 'bytes', os.path.getsize(path), 'sha256', sha)
        for h, fc, nc, fr, nr, w in wtab:
            print('  %-18s clips %3d rows %5d' % (w, nc, nr))
        hz = lower[:, len(LOWER) * 4 + 2]
        print('  hips height mm min %d max %d' % (int(hz.min()), int(hz.max())))
        with open(path + '.inputs.txt', 'w', newline='\n') as fh:
            for n, h in digests:
                fh.write('%s  %s\n' % (h, n))
