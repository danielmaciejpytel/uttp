"""Work around the WolvenKit.CLI 9.0.1 glTF->anims root-motion bug.

Observed (vanilla pwa_sport2_driver 'enter' exported and re-imported unchanged, Build/work/vehicle-entry-archive/
import-vanilla-rt): the importer writes animSplineCompressedMotionExtraction keys for ALL joints (second u16 of every
key = flag | joint index, up to 151) and without the glTF(Y-up) -> RED(Z-up) axis conversion (stored (x, 0, -y)
instead of (x, y, 0)); re-export then fails with 'Motion extraction in joint 1'.

This script rewrites motionExtraction.posKeysData / rotKeysData of one clip in the serialized JSON from the authored
GLB Root track, in the vanilla layout (16-byte keys):
  pos: <u16 t = frame/last*65535> <u16 0>    <f32 x> <f32 y> <f32 z>         (RED axes: x right, y forward, z up)
  rot: <u16 t>                    <u16 8192> <f32 sx> <f32 sy> <f32 sz>     (s = axis * sqrt(1 - w), w = 1 - |s|^2)
with the vanilla tail (an extra key at 65534 before the last at 65535).

Usage (python -I): patch_motion_extraction.py <imported.anims.json> <authored.anims.glb> <out.anims.json> [--clip enter]
"""
import argparse
import hashlib
import json
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
import glbanim as G  # noqa: E402


def gltf_to_red_pos(v):
    return np.stack([v[:, 0], -v[:, 2], v[:, 1]], 1)


def gltf_to_red_quat(q):
    # rotation axis mapping (x, y, z) -> (x, -z, y), w unchanged
    return np.stack([q[:, 0], -q[:, 2], q[:, 1], q[:, 3]], 1)


def keys(times, vals, flag, dur):
    out = bytearray()
    n = len(times)
    tn = np.round(times / dur * 65535).astype(int)
    for i in range(n):
        t = int(tn[i])
        if i == n - 1 and n > 1:
            out += struct.pack('<HHfff', 65534, flag, *map(float, vals[i]))
            t = 65535
        out += struct.pack('<HHfff', t, flag, *map(float, vals[i]))
    return list(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('json_in')
    ap.add_argument('glb')
    ap.add_argument('json_out')
    ap.add_argument('--clip', default='enter')
    ap.add_argument('--vanilla-json', help="exit clips (build_exit.ps1): the serialized vanilla set (subset base json) - the clip's motionExtraction block is copied VERBATIM from it instead of being re-encoded from the GLB Root track (root motion unchanged byte for byte)")
    a = ap.parse_args()
    gj, binb = G.read_glb(a.glb)
    ch = G.channels(gj, binb, G.clips(gj)[a.clip])['Root']
    tt, tv, _ = ch['translation']
    rt, rv, _ = ch['rotation']
    doc = json.loads(Path(a.json_in).read_text(encoding='utf-8'))
    done = 0
    for an in doc['Data']['RootChunk']['animations']:
        d = an['Data']['animation']['Data']
        if d['name']['$value'] != a.clip:
            continue
        if a.vanilla_json:
            vdoc = json.loads(Path(a.vanilla_json).read_text(encoding='utf-8'))
            vme = next(x['Data']['animation']['Data']['motionExtraction']['Data'] for x in vdoc['Data']['RootChunk']['animations'] if x['Data']['animation']['Data']['name']['$value'] == a.clip)
            d['motionExtraction']['Data'] = vme
            done += 1
            end = dict(copied_verbatim_from=str(a.vanilla_json))
            pos = np.zeros((1, 3))
            continue
        me = d['motionExtraction']['Data']
        assert me['$type'] == 'animSplineCompressedMotionExtraction', me['$type']
        dur = float(me['duration'])
        pos = gltf_to_red_pos(tv.astype(np.float64))
        q = gltf_to_red_quat(rv.astype(np.float64))
        q = q / np.linalg.norm(q, axis=1, keepdims=True)
        q[q[:, 3] < 0] *= -1
        me['posKeysData'] = keys(tt.astype(np.float64), pos, 0, dur)
        # RED stores the vector part rescaled so that w = 1 - |s|^2 (verified on the vanilla bytes: yaw 80.75 deg,
        # w 0.762 -> stored z 0.488 = sqrt(1 - w)); WolvenKit's exporter decodes it the same way.
        vn = np.linalg.norm(q[:, :3], axis=1, keepdims=True)
        s = np.where(vn > 1e-9, q[:, :3] / np.maximum(vn, 1e-12), 0.0) * np.sqrt(np.clip(1 - q[:, 3:4], 0, 1))
        me['rotKeysData'] = keys(rt.astype(np.float64), s, 8192, dur)
        done += 1
        end = dict(pos=pos[-1].round(4).tolist(), yaw_deg=float(np.degrees(2 * np.arctan2(q[-1, 2], q[-1, 3]))))
    assert done == 1, 'clip not found'
    Path(a.json_out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.json_out).write_text(json.dumps(doc, separators=(',', ':')) + '\n', encoding='utf-8')
    print(json.dumps(dict(out=a.json_out, sha256=hashlib.sha256(Path(a.json_out).read_bytes()).hexdigest(),
                          pos_keys=len(tt), rot_keys=len(rt), root_end_red=end)))


if __name__ == '__main__':
    main()
