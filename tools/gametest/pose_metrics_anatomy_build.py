"""Builds testdata/pose_metrics_anatomy.json, the per-joint rest rotation and rotation bands used by pose_metrics.py.

  python -I pose_metrics_anatomy_build.py --limits woman=LIMITS_WOMAN.json man=LIMITS_MAN.json --rig woman=RIG_WOMAN.json man=RIG_MAN.json [--out FILE]

LIMITS_*.json hold, per joint, the bands of the rest-relative local rotation (twist about the local +X axis, swing about local Y and Z, in
degrees; the keys twist / swing_y / swing_z are the tolerant bands, the *_p keys the observed ones) and the allowed local translation
deviation. RIG_*.json is the serialized skeleton resource of the body (bone names and reference local transforms). The output stores, for each
joint, the reference local rotation (xyzw) next to the bands so the analysis needs neither file at run time.
"""
import argparse
import json
import os

DEFAULT_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'testdata', 'pose_metrics_anatomy.json')
KEYS = ('twist', 'swing_y', 'swing_z', 'twist_p', 'swing_y_p', 'swing_z_p', 'trans_dev_cm', 'trans_dev_cm_p', 'rest_local_t_cm')


def parse_pairs(items):
    out = {}
    for item in items:
        name, _, path = item.partition('=')
        if not path:
            raise SystemExit('expected NAME=PATH, got %r' % item)
        out[name] = path
    return out


def build(limits_path, rig_path):
    limits = json.load(open(limits_path, encoding='utf-8'))
    root = json.load(open(rig_path, encoding='utf-8'))['Data']['RootChunk']
    names = [b['$value'] for b in root['boneNames']]
    quats = {}
    for n, t in zip(names, root['boneTransforms']):
        r = t['Rotation']
        quats[n] = [round(float(r[k]), 7) for k in ('i', 'j', 'k', 'r')]
    joints = {}
    for name, lim in limits['joints'].items():
        entry = {k: lim[k] for k in KEYS if k in lim}
        entry['rest_quat'] = quats[name]
        joints[name] = entry
    return {'joints': joints}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--limits', nargs='+', required=True)
    ap.add_argument('--rig', nargs='+', required=True)
    ap.add_argument('--out', default=DEFAULT_OUT)
    a = ap.parse_args(argv)
    limits, rigs = parse_pairs(a.limits), parse_pairs(a.rig)
    data = {'note': 'Per-joint reference local rotation (xyzw) and rest-relative rotation bands (degrees) of each body; see pose_metrics_anatomy_build.py.'}
    for body in sorted(limits):
        data[body] = build(limits[body], rigs[body])
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(data, f, indent=0, sort_keys=True)
        f.write('\n')
    print('wrote', a.out, os.path.getsize(a.out), 'bytes')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
