"""Render the source pack chooser tables (Extracted\\sourcepack\\sourcepack\\choosers.json) as readable rule lists.

Each row is printed as `IF <conditions> => <result> [outputs]`, nested tables are expanded in place.
Cell semantics (UE 5.8 Chooser plugin, T3D delta values; missing keys = struct defaults):
  EnumColumn        ValueName + Comparison (default MatchEqual)
  MultiEnumColumn   Value = bitmask over enum values (bit i = enum value i)
  BoolColumn        MatchTrue / MatchFalse / MatchAny
  FloatRangeColumn  Min/Max (default 0), bNoMin/bNoMax
  OutputStruct/OutputEnum columns write values into the output context struct
  RandomizeColumn   per-row weight
Read-only. Usage: python -I chooser_dump.py <choosers.json> <blueprints.json> > out.txt
"""
import json
import re
import sys

GUID_RE = re.compile(r'_\d+_[0-9A-F]{32}$')
ENUM_VALUES = {}


def short(p):
    if not isinstance(p, str):
        return str(p)
    m = re.match(r"^[\w./]+'(.+)'$", p)
    if m:
        p = m.group(1)
    return p.rsplit('.', 1)[-1] if p.startswith('/') else p


def multienum(col, v):
    try:
        mask = int((v or {}).get('Value', '0'))
    except ValueError:
        return str(v)
    names = ENUM_VALUES.get(short(col.get('input_enum') or ''), [])
    sel = []
    for i in range(32):
        if mask & (1 << i):
            sel.append(names[i] if i < len(names) and names[i] else 'bit%d' % i)
    return '{' + '|'.join(sel) + '}'


def fmt_cell(col, v):
    t = col['type']
    name = col.get('input_display') or '.'.join(col.get('input_binding') or []) or t
    if isinstance(col.get('input_binding'), list) and col['input_binding']:
        name = '.'.join(GUID_RE.sub('', x) for x in col['input_binding'])
    if t == 'EnumColumn':
        if not v:
            v = {}
        vn = v.get('ValueName') or 'value0'
        vn = re.sub(r'^\w+::NewEnumerator\d+ \((.+)\)$', r'\1', vn)
        cmp_ = v.get('Comparison', 'MatchEqual')
        if cmp_ == 'MatchAny':
            return None
        return '%s %s %s' % (name, '!=' if cmp_ == 'MatchNotEqual' else '==', vn)
    if t == 'MultiEnumColumn':
        r = multienum(col, v)
        return None if r == '{}' else '%s in %s' % (name, r)
    if t == 'BoolColumn':
        v = v or col.get('settings', {}).get('DefaultRowValue', 'MatchAny')
        if v == 'MatchAny':
            return None
        return ('%s' if v == 'MatchTrue' else '!%s') % name
    if t == 'FloatRangeColumn':
        v = v if isinstance(v, dict) else {}
        lo = None if v.get('bNoMin') == 'True' else float(v.get('Min', '0'))
        hi = None if v.get('bNoMax') == 'True' else float(v.get('Max', '0'))
        if lo is None and hi is None:
            return None
        if lo is None:
            return '%s <= %g' % (name, hi)
        if hi is None:
            return '%s >= %g' % (name, lo)
        return '%g <= %s <= %g' % (lo, name, hi)
    if t == 'OutputStructColumn':
        if isinstance(v, dict):
            kv = ['%s=%s' % (GUID_RE.sub('', k), x) for k, x in v.items() if k != '_type' and x not in ('', None)]
            return 'OUT{' + ', '.join(kv) + '}'
        return 'OUT{default}'
    if t == 'OutputEnumColumn':
        if isinstance(v, dict):
            return 'OUT %s=%s' % (name, re.sub(r'^\w+::NewEnumerator\d+ \((.+)\)$', r'\1', v.get('ValueName', '')))
        return 'OUT %s=default' % name
    if t == 'RandomizeColumn':
        return 'weight=%s' % (v if v not in ('', None, []) else col.get('settings', {}).get('DefaultRowValue'))
    return '%s:%s' % (t, json.dumps(v))


def fmt_result(r):
    if not r:
        return '<none>'
    t = r.get('type')
    if t == 'AssetChooser' or 'Asset' in r:
        return short(r.get('Asset'))
    if t == 'NestedChooser':
        return 'NESTED "%s"' % r.get('Chooser')
    if t == 'EvaluateChooser':
        return 'EVAL %s' % short(r.get('Chooser'))
    return json.dumps(r)


def dump(tb, ind, out, nested_by_name):
    cols = [c for c in tb.get('columns', [])]
    out.append('%sTABLE %s  (output %s)' % ('  ' * ind, tb.get('name'), short(tb.get('output_object_type'))))
    for c in cols:
        out.append('%s  col[%d] %s bind=%s%s' % ('  ' * ind, c['index'], c['type'],
                                                  '.'.join(GUID_RE.sub('', x) for x in (c.get('input_binding') or [])) if isinstance(c.get('input_binding'), list) else c.get('input_binding'),
                                                  ' DISABLED' if c.get('disabled') else ''))
    for row in tb.get('rows', []):
        conds = []
        outs = []
        for key, v in (row.get('cells') or {}).items():
            idx = int(key.split(':', 1)[0])
            col = next((c for c in cols if c['index'] == idx), None)
            if col is None:
                continue
            if col.get('disabled'):
                continue
            s = fmt_cell(col, v)
            if s is None:
                continue
            (outs if s.startswith('OUT') or s.startswith('weight') else conds).append(s)
        res = fmt_result(row.get('result'))
        out.append('%s  row %d%s: IF %s => %s %s' % ('  ' * ind, row['row'], ' (DISABLED)' if row.get('disabled') else '',
                                                  ' && '.join(conds) or 'true', res, ' '.join(outs)))
        r = row.get('result') or {}
        if r.get('type') == 'NestedChooser' and r.get('Chooser') in nested_by_name:
            dump(nested_by_name[r['Chooser']], ind + 2, out, nested_by_name)
    fb = tb.get('fallback_result')
    if fb:
        out.append('%s  FALLBACK => %s' % ('  ' * ind, json.dumps(fb)))


def main():
    ch = json.load(open(sys.argv[1], encoding='utf-8'))
    bpj = json.load(open(sys.argv[2], encoding='utf-8'))
    for e in bpj.get('enums', []):
        vals = []
        for x in sorted(e['entries'], key=lambda z: z['value']):
            if x.get('is_max'):
                continue
            vals.append((x.get('display_name') or x['internal_name']).replace(' ', ''))
        ENUM_VALUES[e['name']] = vals
    out = []
    for t in ch['tables']:
        tb = t['table']
        out.append('=' * 100)
        out.append('CHOOSER %s' % t['path'])
        out.append('context: %s' % json.dumps(tb.get('context')))
        nested_by_name = {}

        def collect(x):
            for n in x.get('nested', []) or []:
                nested_by_name[n.get('name')] = n
                collect(n)
        collect(tb)
        dump(tb, 0, out, nested_by_name)
    sys.stdout.write('\n'.join(out) + '\n')


if __name__ == '__main__':
    main()
