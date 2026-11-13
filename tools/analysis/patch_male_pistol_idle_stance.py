"""Curve patch of the relaxed one-handed pistol idle pose (male counterpart of patch_female_pistol_idle_stance.py).

Four curve values plus the two CRCs may change: in the two frames of Pose_Pistol1H_Stand_Relax_Idle the
layering_legs / layering_pelvis curves go 1 -> 0 (the relaxed 1H pose then keeps the locomotion lower body).
Skeleton, transforms, deltas, every other pose/curve stay byte-identical (asserted).

Input is hash-locked (utpp_overlays_man.utppov 8F2C1D08...). The output is always a NEW file; an existing
output is never overwritten. Nothing is installed. Usage:
    python -I patch_male_pistol_idle_stance.py [--source <man.utppov>] [--output <new file>] [--report <json>]
Default output: Data\\motion\\utpp_overlays_man_p1h_idle_stance.utppov
"""
import argparse, hashlib, json, struct, sys, zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2].parent
EXPECTED_SOURCE_SHA = '8F2C1D085E9793424C0C7127AC82163B6265921D14286284A99FD83A8E8E5545'
POSE_NAME = 'Pose_Pistol1H_Stand_Relax_Idle'
sha = lambda b: hashlib.sha256(b).hexdigest().upper()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default=str(ROOT / 'Data/motion/utpp_overlays_man.utppov'))
    ap.add_argument('--output', default=str(ROOT / 'Data/motion/utpp_overlays_man_p1h_idle_stance.utppov'))
    ap.add_argument('--report', default='')
    args = ap.parse_args()
    out = Path(args.output)
    if out.exists():
        sys.exit(f'refusing to overwrite existing {out}')
    original = Path(args.source).read_bytes()
    data = bytearray(original)
    assert sha(original) == EXPECTED_SOURCE_SHA, 'unexpected source overlay hash'
    assert original[:8] == b'UTPPOV\0\0' and struct.unpack_from('<II', original, 8) == (1, 64)
    assert struct.unpack_from('<I', original, 32)[0] == len(original)
    assert struct.unpack_from('<I', original, 36)[0] == zlib.crc32(original[64:])
    header = bytearray(original[:64]); struct.pack_into('<I', header, 40, 0)
    assert struct.unpack_from('<I', original, 40)[0] == zlib.crc32(header)
    sections = {}
    for i in range(struct.unpack_from('<I', data, 28)[0]):
        tag, off, size, count = struct.unpack_from('<4sIII', data, 64 + i * 16)
        assert off + size <= len(data)
        sections[tag] = (off, size, count)
    co, _, nc = sections[b'CURV']
    curves = [bytes(data[co + i * 48:co + (i + 1) * 48]).split(b'\0')[0].decode() for i in range(nc)]
    po, _, npz = sections[b'POSE']
    matches = []
    for i in range(npz):
        off = po + i * 104
        if bytes(data[off:off + 64]).split(b'\0')[0].decode() == POSE_NAME:
            matches.append(off)
    assert len(matches) == 1
    off = matches[0]
    first, count = struct.unpack_from('<II', data, off + 68)
    assert count == 2
    mask = struct.unpack_from('<Q', data, off + 96)[0]
    fo, _, _ = sections[b'FRAM']
    bones = struct.unpack_from('<I', data, 24)[0]
    stride = (bones * 17 + nc) * 4
    allowed = set(range(36, 44))  # the two CRC fields in the header
    changes = []
    for frame in range(first, first + count):
        for curve in ('layering_legs', 'layering_pelvis'):
            ci = curves.index(curve)
            assert mask & (1 << ci)
            offset = fo + frame * stride + bones * 17 * 4 + ci * 4
            assert struct.unpack_from('<f', data, offset)[0] == 1.0
            struct.pack_into('<f', data, offset, 0.0)
            allowed.update(range(offset, offset + 4))
            changes.append({'frame': frame, 'curve': curve, 'before': 1, 'after': 0, 'offset': offset})
    struct.pack_into('<I', data, 36, zlib.crc32(data[64:]))
    struct.pack_into('<I', data, 40, 0)
    struct.pack_into('<I', data, 40, zlib.crc32(data[:64]))
    changed = [i for i, (a, b) in enumerate(zip(original, data)) if a != b]
    assert set(changed) <= allowed
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, 'xb') as f:  # 'x': fails if the file appeared meanwhile
        f.write(data)
    report = {'sourceSha256': sha(original), 'patchedSha256': sha(data), 'output': str(out),
              'curveChanges': changes, 'changedByteCount': len(changed), 'allowedByteChangesOnly': True,
              'readyForVisualAcceptance': False}
    if args.report:
        Path(args.report).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
