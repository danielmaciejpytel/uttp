"""Shared TPP nails plane fix (female pwa + male pma): clear the FPP animation-driven render plane binding
(renderingPlaneAnimationParam renderPlane/renderPlaneLeftArm -> None) on the TPP nail components only.

Input: WolvenKit CLI JSON (cr2w -s) of a0_000_base__nails.app / a0_000_base__full.app (vanilla).
usage: patch_tpp_nails_plane.py <in_json_dir> <out_json_dir> <report.json> [--female-only]
Scope: entMorphTargetSkinnedMeshComponent named a0_000_{pwa,pma}_base_(_)nails_{l,r}; FPP resource is never read/written.
"""
import json, sys, hashlib
from pathlib import Path

FEMALE = {'a0_000_pwa_base_nails_l', 'a0_000_pwa_base_nails_r'}
MALE = {'a0_000_pma_base__nails_l', 'a0_000_pma_base__nails_r'}
RESOURCES = ('a0_000_base__nails.app', 'a0_000_base__full.app')


def value(x):
    return x.get('$value') if isinstance(x, dict) else x


def patch(x, target, edits, path=''):
    if isinstance(x, dict):
        if x.get('$type') == 'entMorphTargetSkinnedMeshComponent' and value(x.get('name')) in target:
            f = x['renderingPlaneAnimationParam']; old = f['$value']
            assert old in ('renderPlaneLeftArm', 'renderPlane'), (path, old)
            f['$value'] = 'None'
            edits.append({'component': value(x['name']), 'path': path, 'before': old, 'after': 'None'})
        for k, y in x.items(): patch(y, target, edits, path + '/' + k)
    elif isinstance(x, list):
        for i, y in enumerate(x): patch(y, target, edits, '%s[%d]' % (path, i))


def main():
    src, out, rep = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    target = FEMALE if '--female-only' in sys.argv else FEMALE | MALE
    out.mkdir(parents=True, exist_ok=True)
    report = {'target': sorted(target), 'assets': []}
    for name in RESOURCES:
        p = src / (name + '.json')
        d = json.loads(p.read_text(encoding='utf-8')); edits = []; patch(d, target, edits)
        assert edits and {e['component'] for e in edits} == target, name
        (out / (name + '.json')).write_text(json.dumps(d, indent=2), encoding='utf-8')
        report['assets'].append({'resource': name, 'sourceJsonSHA256': hashlib.sha256(p.read_bytes()).hexdigest(),
                                 'count': len(edits), 'components': sorted({e['component'] for e in edits}),
                                 'before': sorted({e['before'] for e in edits})})
        print(name, len(edits))
    rep.write_text(json.dumps(report, indent=2), encoding='utf-8')


if __name__ == '__main__': main()
