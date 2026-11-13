"""Dump the full anim-node structs stored on an AnimBlueprint's generated-class CDO
(cdo_other_overrides in the per-Blueprint JSON of the source pack extraction).

Unlike the editor-graph T3D (delta export), the CDO holds every anim node's complete
runtime struct, so this is the authoritative list of node settings.
Read-only. Usage:
  python -I bp_animnode_cdo.py <blueprint.json> [name_filter_regex] > out.txt
"""
import json
import re
import sys

DROP_KEYS = {'InitialUpdateFunction', 'BecomeRelevantFunction', 'UpdateFunction', 'LinkID',
             'SourceLinkID'}


def prune(v):
    if isinstance(v, dict):
        out = {}
        for k, x in v.items():
            if k in DROP_KEYS:
                if isinstance(x, dict) and (x.get('FunctionName') or x.get('ClassName')):
                    out[k] = x
                continue
            px = prune(x)
            if px in ({}, [], None):
                continue
            out[k] = px
        if set(out.keys()) <= {'LinkID', 'SourceLinkID'}:
            return {}
        return out
    if isinstance(v, list):
        return [prune(x) for x in v]
    return v


def main():
    bp = json.load(open(sys.argv[1], encoding='utf-8'))
    flt = re.compile(sys.argv[2]) if len(sys.argv) > 2 else None
    cdo = bp.get('cdo_other_overrides') or {}
    for k, v in cdo.items():
        if not (k.startswith('AnimGraphNode_') or k.startswith('__AnimBlueprintMutables')):
            continue
        if flt and not flt.search(k):
            continue
        print('==', k)
        print(json.dumps(prune(v), ensure_ascii=False, separators=(',', ':')))


if __name__ == '__main__':
    main()
