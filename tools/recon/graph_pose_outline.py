"""Compact pose-flow outline of an animgraph (WolvenKit JSON export). Read-only analysis helper.

Walks pose links (animPoseLink) from the Root node and prints one line per pose node with the fields that matter
for layering: masks, slot names, bone/chain names, sub-graph paths, and a short description of the weight/condition
inputs (resolved through float/int/bool nodes to the feeding variable / anim feature names).
State machines print their state names; their contents are expanded only when --expand-sm is given.

Usage: python -I graph_pose_outline.py <graph.animgraph.json> [--max-depth N] [--expand-sm] [--skip-skanim]
"""
import json
import sys

NOISE = {'$type', 'debugName', 'id', 'poseInfoLogger', 'visAxes', 'visMask', 'visNames', 'visPostPose',
         'visPostPoseColor', 'visPrePose', 'visPrePoseColor', 'visRigPartMask', 'visWhenActive'}


def cname(v):
    if isinstance(v, dict) and v.get('$type') == 'CName':
        return v.get('$value')
    return None


class Graph:
    def __init__(self, root):
        self.by_id = {}
        self._index(root)

    def _index(self, n):
        if isinstance(n, dict):
            if 'HandleId' in n and isinstance(n.get('Data'), dict):
                self.by_id[n['HandleId']] = n['Data']
            for v in n.values():
                self._index(v)
        elif isinstance(n, list):
            for v in n:
                self._index(v)

    def deref(self, h):
        if not isinstance(h, dict):
            return None, None
        if 'HandleId' in h:
            return h['HandleId'], h.get('Data')
        if 'HandleRefId' in h:
            return h['HandleRefId'], self.by_id.get(h['HandleRefId'])
        return None, None


def tname(d):
    return (d or {}).get('$type', '?').replace('animAnimNode_', '')


def scalar_desc(g, link, depth=0, seen=None):
    """Describe a float/int/bool/vector link source briefly (variable / feature names)."""
    seen = seen or set()
    if not isinstance(link, dict):
        return None
    node = link.get('node') if link.get('$type', '').endswith('Link') else link
    hid, d = g.deref(node)
    if d is None or hid in seen or depth > 4:
        return None if d is None else tname(d)
    seen.add(hid)
    t = tname(d)
    names = []
    for k in ('variableName', 'name', 'animFeatureName', 'featureName', 'featurePropertyName', 'propertyName',
              'trackName', 'curveName', 'eventName', 'floatTrackName', 'animSetTag'):
        v = d.get(k)
        nv = cname(v)
        if nv is None and isinstance(v, dict) and 'name' in v:
            nv = cname(v.get('name')) if isinstance(v.get('name'), dict) else v.get('name')
        if nv and nv != 'None':
            names.append(f'{k}={nv}')
    if t in ('FloatConstant',):
        names.append(f"v={d.get('value')}")
    if t == 'MathExpressionFloat' or t == 'MathExpressionPose':
        expr = d.get('expression') or {}
        ex = expr.get('expression') if isinstance(expr, dict) else None
        if ex:
            names.append(f'expr="{ex}"')
    subs = []
    for k, v in d.items():
        if isinstance(v, dict) and v.get('$type', '').startswith('anim') and v.get('$type', '').endswith('Link') \
                and v.get('$type') != 'animPoseLink':
            s = scalar_desc(g, v, depth + 1, seen)
            if s:
                subs.append(s)
        elif isinstance(v, list):
            for x in v:
                if isinstance(x, dict) and x.get('$type', '').endswith('Link') and x.get('$type') != 'animPoseLink':
                    s = scalar_desc(g, x, depth + 1, seen)
                    if s:
                        subs.append(s)
    out = t + ('(' + ','.join(names) + ')' if names else '')
    if subs:
        out += '[' + '; '.join(subs[:6]) + (' ...' if len(subs) > 6 else '') + ']'
    return out


def pose_children(d):
    kids = []
    for k, v in d.items():
        if k in NOISE:
            continue
        if isinstance(v, dict) and v.get('$type') == 'animPoseLink':
            kids.append((k, v.get('node')))
        elif isinstance(v, list):
            for i, x in enumerate(v):
                if isinstance(x, dict) and x.get('$type') == 'animPoseLink':
                    kids.append((f'{k}[{i}]', x.get('node')))
                elif isinstance(x, dict) and ('HandleId' in x or 'HandleRefId' in x):
                    kids.append((f'{k}[{i}]', x))
        elif isinstance(v, dict) and ('HandleId' in v or 'HandleRefId' in v) and k not in ('node',):
            dd = v.get('Data') or {}
            if dd.get('$type') in ('animAnimNode_Output', 'animAnimNode_State', 'animAnimNode_StateFrozen'):
                kids.append((k, v))
    return kids


def extra_fields(d):
    out = []
    for k, v in d.items():
        if k in NOISE:
            continue
        nv = cname(v)
        if nv and nv != 'None':
            out.append(f'{k}={nv}')
        elif isinstance(v, dict) and v.get('$type') in ('animTransformIndex', 'animNamedTrackIndex'):
            nm = cname(v.get('name')) if isinstance(v.get('name'), dict) else v.get('name')
            if nm and nm != 'None':
                out.append(f'{k}={nm}')
        elif isinstance(v, dict) and 'DepotPath' in v:
            p = v['DepotPath']
            p = p.get('$value') if isinstance(p, dict) else p
            if p:
                out.append(f'{k}={p}')
        elif isinstance(v, list) and v and all(isinstance(x, (str, dict)) and (isinstance(x, str) or x.get('$type') == 'CName') for x in v):
            vals = [x if isinstance(x, str) else x.get('$value') for x in v]
            out.append(f'{k}=[{",".join(map(str, vals))}]')
        elif isinstance(v, (int, float)) and not isinstance(v, bool) and k in ('blendTime', 'weight', 'blendTimeIn', 'blendTimeOut', 'bodyPart', 'blendMode', 'priority'):
            out.append(f'{k}={v}')
        elif isinstance(v, str) and k not in ('HandleId',) and v and v != 'None' and len(v) < 60:
            out.append(f'{k}={v}')
        elif isinstance(v, dict) and v.get('$type', '').startswith('anim') and not v.get('$type', '').endswith('Link'):
            # nested request structs (IK requests etc.): list their named fields
            sub = extra_fields(v)
            if sub:
                out.append(f'{k}{{{"; ".join(sub)}}}')
    return out


def main():
    args = sys.argv[1:]
    path = args[0]
    max_depth = int(args[args.index('--max-depth') + 1]) if '--max-depth' in args else 60
    expand_sm = '--expand-sm' in args
    skip_sk = '--skip-skanim' in args
    root = json.load(open(path, encoding='utf-8'))['Data']['RootChunk']
    g = Graph(root)
    printed = set()

    def visit(label, h, depth):
        hid, d = g.deref(h)
        if d is None:
            return
        t = tname(d)
        if skip_sk and t.startswith('Sk') and t.endswith('Anim'):
            nm = cname(d.get('animation'))
            print('  ' * depth + f'{label}: {t} #{hid} anim={nm}')
            return
        if hid in printed:
            print('  ' * depth + f'{label}: -> #{hid} {t} (shown above)')
            return
        printed.add(hid)
        dn = cname(d.get('debugName'))
        line = f'{label}: {t} #{hid}' + (f' "{dn}"' if dn and dn != 'None' else '')
        ex = extra_fields(d)
        if ex:
            line += ' | ' + '; '.join(ex)
        conds = []
        for k, v in d.items():
            if isinstance(v, dict) and v.get('$type', '').endswith('Link') and v.get('$type') != 'animPoseLink' and v.get('node'):
                s = scalar_desc(g, v)
                if s:
                    conds.append(f'{k}<-{s}')
        if conds:
            line += ' || ' + ' | '.join(conds)
        print('  ' * depth + line)
        if depth >= max_depth:
            return
        if t == 'StateMachine':
            states = d.get('states') or []
            names = []
            for s in states:
                sid, sd = g.deref(s)
                if sd:
                    names.append(f'{cname(sd.get("name"))}#{sid}')
            print('  ' * (depth + 1) + f'states({len(names)}): ' + ', '.join(names))
            if not expand_sm:
                return
            for s in states:
                visit('state', s, depth + 1)
            return
        for k, child in pose_children(d):
            visit(k, child, depth + 1)

    nodes = root.get('nodesToInit') or []
    for n in nodes:
        visit('root', n, 0)


if __name__ == '__main__':
    # deep linear chains (deformation graphs) need a large recursion budget
    import threading
    sys.setrecursionlimit(200000)
    threading.stack_size(256 * 1024 * 1024 - 4096)
    th = threading.Thread(target=main)
    th.start()
    th.join()
