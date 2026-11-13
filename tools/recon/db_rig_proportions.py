"""Compare bone local translations in a .utppmm POSE section against rig.json reference poses.

Usage: python db_rig_proportions.py <db.utppmm> <rig.json> [<rig2.json> ...]
Prints, per bone 2..72, the bone length (|local translation|) in the database (mean over frames, and std)
and in each rig, plus the largest relative differences. Read-only.
"""
import json
import math
import struct
import sys


def read_db(path):
    data = open(path, 'rb').read()
    nfr, nb = struct.unpack_from('<II', data, 36)
    nsec, = struct.unpack_from('<I', data, 84)
    secs = {}
    for i in range(nsec):
        tag, cnt, elem, _, off, _ = struct.unpack_from('<4sIIIQQ', data, 128 + i * 32)
        secs[tag.decode()] = (off, cnt, elem)
    strs_off = secs['STRS'][0]
    names = []
    boff = secs['BONE'][0]
    for b in range(nb):
        no, par, nl, _ = struct.unpack_from('<IiII', data, boff + b * 16)
        names.append(data[strs_off + no:strs_off + no + nl].decode())
    poff = secs['POSE'][0]
    return data, nfr, nb, names, poff


def rig_rest(path):
    j = json.load(open(path, encoding='utf-8'))
    root = j['Data']['RootChunk']
    names = [n['$value'] for n in root['boneNames']]
    poses = root['referencePoseLS'] if root.get('referencePoseLS') else root['boneTransforms']
    out = {}
    for n, t in zip(names, poses):
        tr = t['Translation']
        out[n] = (tr['X'], tr['Y'], tr['Z'])
    return out


def main():
    data, nfr, nb, names, poff = read_db(sys.argv[1])
    rigs = [(p, rig_rest(p)) for p in sys.argv[2:]]
    stride = nb * 28
    step = max(1, nfr // 4000)
    rows = []
    for b in range(2, nb):
        ls = []
        for f in range(0, nfr, step):
            px, py, pz = struct.unpack_from('<3f', data, poff + f * stride + b * 28)
            ls.append(math.sqrt(px * px + py * py + pz * pz))
        m = sum(ls) / len(ls)
        sd = math.sqrt(sum((x - m) ** 2 for x in ls) / len(ls))
        refs = []
        for _, r in rigs:
            t = r.get(names[b])
            refs.append(math.sqrt(sum(c * c for c in t)) if t else float('nan'))
        rows.append((names[b], m, sd, refs))
    print('bone'.ljust(28), 'db_mean  db_std ', '  '.join(p.split('\\')[-2][:10].rjust(10) for p, _ in rigs))
    for n, m, sd, refs in rows:
        rel = max(abs(m - r) / r for r in refs if r > 1e-4) if any(r > 1e-4 for r in refs) else 0
        flag = ' <<' if rel > 0.05 and abs(m - refs[0]) > 0.005 else ''
        print(n.ljust(28), f'{m:7.4f} {sd:7.4f}', '  '.join(f'{r:10.4f}' for r in refs), flag)


if __name__ == '__main__':
    main()
