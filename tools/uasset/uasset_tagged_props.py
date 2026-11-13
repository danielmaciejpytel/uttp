"""Offline, read-only dump of TAGGED property values from uncooked UE 5.x editor packages (.uasset).

Editor packages serialize every UPROPERTY whose value differs from the class / archetype default as an FPropertyTag
(UE 5.8: Engine/Source/Runtime/CoreUObject/Private/UObject/PropertyTag.cpp operator<<):
    FName Name (int32 name index, int32 number); FPropertyTypeName Type = pre-order list of nodes
    {FName, int32 InnerCount}; int32 Size; uint8 Flags (0x01 ArrayIndex, 0x02 Guid, 0x04 Extensions,
    0x08 BinaryOrNative, 0x10 BoolTrue, 0x20 Skipped); [int32 ArrayIndex]; [FGuid]; [extensions]; value (Size bytes).
Properties that are NOT listed therefore keep the C++ default of their class (defaults: engine headers).

The whole file is scanned for tag headers (no export walking), validated (name indexes, type tree, size, flags) and
nested by byte ranges, so struct members / array elements appear with a path (Node.BlendTime, Samples[2].Offset).
Heuristic scanner: a random byte run may in theory validate; values are reported raw where not decodable.

Usage (system python, nothing is written next to the source files):
  python -I uasset_tagged_props.py <out_dir> <file_or_dir> [<file_or_dir> ...] [--name-filter SUBSTR ...]
"""
import json
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import uasset_names  # noqa: E402  (same folder, read-only header reader)

INT_TYPES = {
    'IntProperty': '<i', 'Int8Property': '<b', 'Int16Property': '<h', 'Int64Property': '<q',
    'UInt16Property': '<H', 'UInt32Property': '<I', 'UInt64Property': '<Q',
}
OBJ_TYPES = {'ObjectProperty', 'ClassProperty', 'WeakObjectProperty', 'LazyObjectProperty', 'InterfaceProperty'}
BIN_STRUCTS = {
    'Vector': ('<3d', 24), 'Vector2D': ('<2d', 16), 'Rotator': ('<3d', 24), 'Quat': ('<4d', 32),
    'Vector4': ('<4d', 32), 'LinearColor': ('<4f', 16), 'Vector3f': ('<3f', 12), 'Vector2f': ('<2f', 8),
    'IntPoint': ('<2i', 8), 'IntVector': ('<3i', 12),
}


def export_serial_ranges(data):
    """(serial_offset, serial_size) per export; same summary walk as uasset_names.read_package."""
    U = uasset_names
    r = U.Reader(data)
    r.u32()
    legacy = r.i32()
    if legacy != -4:
        r.i32()
    r.i32()
    ue5 = r.i32() if legacy <= -8 else 0
    r.i32()
    if ue5 >= U.UE5_PACKAGE_SAVED_HASH:
        r.skip(20)
        r.i32()
    if legacy <= -2:
        n = r.i32()
        r.skip(n * 20)
    if ue5 < U.UE5_PACKAGE_SAVED_HASH:
        r.i32()
    r.fstring()
    flags = r.u32()
    r.i32(), r.i32()
    if ue5 >= U.UE5_ADD_SOFTOBJECTPATH_LIST:
        r.i32(), r.i32()
    if not flags & U.PKG_FILTER_EDITOR_ONLY:
        r.fstring()
    r.i32(), r.i32()
    export_count, export_offset = r.i32(), r.i32()
    out = []
    r.p = export_offset
    for _ in range(export_count):
        r.skip(16)
        r.skip(8)
        r.u32()
        size, offset = r.i64(), r.i64()
        r.skip(12)
        if ue5 < U.UE5_REMOVE_OBJECT_EXPORT_PACKAGE_GUID:
            r.skip(16)
        if ue5 >= U.UE5_TRACK_OBJECT_EXPORT_IS_INHERITED:
            r.skip(4)
        r.skip(4)
        r.skip(8)
        if ue5 >= U.UE5_OPTIONAL_RESOURCES:
            r.skip(4)
        r.skip(20)
        if ue5 >= U.UE5_SCRIPT_SERIALIZATION_OFFSET:
            r.skip(16)
        out.append((offset, size))
    return out


class Pkg:
    def __init__(self, path):
        with open(path, 'rb') as f:
            self.data = f.read()
        self.summary, self.names, self.imports, self.exports = uasset_names.read_package(path)
        self.path = path
        try:
            self.ranges = export_serial_ranges(self.data)
        except Exception:
            self.ranges = []

    def export_at(self, offset):
        for i, (o, s) in enumerate(self.ranges):
            if o <= offset < o + s and i < len(self.exports):
                e = self.exports[i]
                return '%s [%s]' % (e.get('path') or e['name'], e.get('class'))
        return None

    def fname(self, idx, num):
        if not (0 <= idx < len(self.names)):
            return None
        base = self.names[idx]
        return base if num == 0 else '%s_%d' % (base, num - 1)

    def obj(self, idx):
        if idx == 0:
            return None
        if idx < 0 and -idx - 1 < len(self.imports):
            return self.imports[-idx - 1].get('path') or self.imports[-idx - 1]['name']
        if idx > 0 and idx - 1 < len(self.exports):
            return self.exports[idx - 1].get('path') or self.exports[idx - 1]['name']
        return '<bad obj %d>' % idx


def parse_tag(pkg, start):
    d = pkg.data
    n = len(d)
    if start < 0 or start + 8 > n:
        return None
    pidx, pnum = struct.unpack_from('<ii', d, start)
    if not (0 <= pidx < len(pkg.names)) or not (0 <= pnum < 4096):
        return None
    pname = pkg.fname(pidx, pnum)
    if not pname or pname == 'None' or pname.endswith('Property') or pname.startswith('/'):
        return None
    p = start + 8
    nodes = []
    remaining = 1
    while remaining > 0:
        if p + 12 > n or len(nodes) > 16:
            return None
        tidx, tnum, inner = struct.unpack_from('<iii', d, p)
        p += 12
        if not (0 <= tidx < len(pkg.names)) or not (0 <= tnum < 4096) or not (0 <= inner <= 8):
            return None
        nodes.append((pkg.fname(tidx, tnum), inner))
        remaining += inner - 1
    if not nodes[0][0].endswith('Property'):
        return None
    if p + 5 > n:
        return None
    size = struct.unpack_from('<i', d, p)[0]
    flags = d[p + 4]
    p += 5
    if size < 0 or size > n or flags > 0x3F:
        return None
    array_index = 0
    if flags & 0x01:
        array_index = struct.unpack_from('<i', d, p)[0]
        p += 4
        if not (0 <= array_index < 1024):
            return None
    if flags & 0x02:
        p += 16
    if flags & 0x04:
        if p >= n:
            return None
        ext = d[p]
        p += 1
        if ext & ~0x06:
            return None
        if ext & 0x02:
            p += 1 + 4
        if ext & 0x04:
            p += 4
    if p + size > n:
        return None
    t = nodes[0][0]
    if t == 'BoolProperty' and size != 0:
        return None
    if t in ('FloatProperty',) and size != 4:
        return None
    if t in ('DoubleProperty',) and size != 8:
        return None
    if t in INT_TYPES and size != struct.calcsize(INT_TYPES[t]):
        return None
    return {'name': pname, 'type_nodes': nodes, 'size': size, 'flags': flags, 'array_index': array_index,
            'start': start, 'value_start': p, 'value_end': p + size}


def type_str(nodes):
    out = []

    def rec(i):
        name, inner = nodes[i]
        j = i + 1
        args = []
        for _ in range(inner):
            s, j = rec(j)
            args.append(s)
        return (name + ('(' + ','.join(args) + ')' if args else '')), j

    s, _ = rec(0)
    out.append(s)
    return out[0]


def decode(pkg, tag):
    d = pkg.data
    t = tag['type_nodes'][0][0]
    p, size = tag['value_start'], tag['size']
    try:
        if t == 'FloatProperty':
            return round(struct.unpack_from('<f', d, p)[0], 6)
        if t == 'DoubleProperty':
            return round(struct.unpack_from('<d', d, p)[0], 9)
        if t in INT_TYPES:
            return struct.unpack_from(INT_TYPES[t], d, p)[0]
        if t == 'BoolProperty':
            return bool(tag['flags'] & 0x10)
        if t in ('NameProperty', 'EnumProperty') or (t == 'ByteProperty' and size == 8):
            return pkg.fname(*struct.unpack_from('<ii', d, p))
        if t == 'ByteProperty' and size == 1:
            return d[p]
        if t == 'StrProperty':
            r = uasset_names.Reader(d)
            r.p = p
            return r.fstring()
        if t in OBJ_TYPES:
            return pkg.obj(struct.unpack_from('<i', d, p)[0])
        if t == 'StructProperty':
            sname = tag['type_nodes'][1][0] if len(tag['type_nodes']) > 1 else '?'
            if sname in BIN_STRUCTS and BIN_STRUCTS[sname][1] == size:
                return [round(v, 6) for v in struct.unpack_from(BIN_STRUCTS[sname][0], d, p)]
            if sname == 'Guid' and size == 16:
                return d[p:p + 16].hex()
            return None  # nested tags (children)
        if t == 'ArrayProperty':
            cnt = struct.unpack_from('<i', d, p)[0]
            inner = tag['type_nodes'][1][0] if len(tag['type_nodes']) > 1 else '?'
            q = p + 4
            if inner == 'FloatProperty' and size == 4 + 4 * cnt:
                return {'count': cnt, 'items': [round(v, 6) for v in struct.unpack_from('<%df' % cnt, d, q)]}
            if inner == 'IntProperty' and size == 4 + 4 * cnt:
                return {'count': cnt, 'items': list(struct.unpack_from('<%di' % cnt, d, q))}
            if inner in ('NameProperty', 'EnumProperty') and size == 4 + 8 * cnt:
                return {'count': cnt, 'items': [pkg.fname(*struct.unpack_from('<ii', d, q + 8 * k)) for k in range(cnt)]}
            if inner in OBJ_TYPES and size == 4 + 4 * cnt:
                return {'count': cnt, 'items': [pkg.obj(v) for v in struct.unpack_from('<%di' % cnt, d, q)]}
            return {'count': cnt}
        if t in ('MapProperty', 'SetProperty'):
            return {'raw_count': struct.unpack_from('<i', d, p + 4)[0] if size >= 8 else None}
        if t == 'SoftObjectProperty':
            return 'soft:' + d[p:p + min(size, 24)].hex()
    except struct.error:
        return '<undecodable>'
    return 'raw:' + d[p:p + min(size, 32)].hex()


def scan(pkg):
    type_idx = [i for i, nm in enumerate(pkg.names) if nm.endswith('Property')]
    found = {}
    d = pkg.data
    for ti in type_idx:
        pat = struct.pack('<ii', ti, 0)
        pos = d.find(pat)
        while pos != -1:
            tag = parse_tag(pkg, pos - 8)
            if tag:
                found[tag['start']] = tag
            pos = d.find(pat, pos + 1)
    cand = sorted(found.values(), key=lambda x: x['start'])
    # a tag header cannot start inside an accepted tag's header (name / type tree / size / flags region)
    tags = []
    for t in cand:
        if tags and tags[-1]['start'] <= t['start'] < tags[-1]['value_start']:
            continue
        tags.append(t)
    for t in tags:
        t['type'] = type_str(t['type_nodes'])
        t['value'] = decode(pkg, t)
    # nesting by containment
    stack = []
    none_idx = pkg.names.index('None') if 'None' in pkg.names else -1
    for t in tags:
        while stack and not (stack[-1]['value_start'] <= t['start'] and t['value_end'] <= stack[-1]['value_end']):
            stack.pop()
        t['parent'] = stack[-1] if stack else None
        stack.append(t)
    # element index inside struct arrays: count 'None' terminators of the array between its start and the child
    none_pat = struct.pack('<ii', none_idx, 0) if none_idx >= 0 else None
    children = {}
    for t in tags:
        t['elem'] = None
        if t['parent'] is not None:
            children.setdefault(id(t['parent']), []).append(t)
    for t in tags:
        kids = children.get(id(t), [])
        if not kids or t['type_nodes'][0][0] != 'ArrayProperty' or not none_pat:
            continue
        # element index of each direct child: 'None' terminators at the array level (outside the children's ranges)
        cnt = 0
        pos = t['value_start'] + 4
        for k in kids:
            q = d.find(none_pat, pos, k['start'])
            while q != -1:
                cnt += 1
                q = d.find(none_pat, q + 8, k['start'])
            k['elem'] = cnt
            pos = max(pos, k['value_end'])
    for t in tags:
        parts = []
        cur = t
        while cur is not None:
            seg = cur['name'] + ('[%d]' % cur['array_index'] if cur['array_index'] else '')
            parts.append(seg)
            if cur['elem'] is not None:
                parts.append('[%d]' % cur['elem'])
            cur = cur['parent']
        path = ''
        for seg in reversed(parts):
            path = path + seg if seg.startswith('[') else (path + '.' + seg if path else seg)
        t['path'] = path
    return tags


def export_of(pkg, offset):
    # exports in uasset_names do not keep the serial offset; locate by re-reading the export table offsets
    return None


def dump(pkg, tags, name_filter):
    rows = []
    for t in tags:
        if name_filter and not any(f.lower() in t['path'].lower() for f in name_filter):
            continue
        rows.append({'offset': t['start'], 'export': pkg.export_at(t['start']), 'path': t['path'], 'type': t['type'],
                     'value': t['value']})
    return rows


def main():
    args = sys.argv[1:]
    if len(args) < 2:
        print(__doc__)
        return 2
    out_dir = args[0]
    inputs, filters = [], []
    i = 1
    while i < len(args):
        if args[i] == '--name-filter':
            filters.append(args[i + 1])
            i += 2
        else:
            inputs.append(args[i])
            i += 1
    files = []
    for x in inputs:
        if os.path.isdir(x):
            for root, _, fs in os.walk(x):
                files += [os.path.join(root, f) for f in fs if f.endswith('.uasset')]
        else:
            files.append(x)
    os.makedirs(out_dir, exist_ok=True)
    index = []
    for f in sorted(files):
        try:
            pkg = Pkg(f)
            tags = scan(pkg)
        except Exception as e:  # report and continue (read-only tool)
            index.append({'file': f, 'error': repr(e)})
            continue
        rows = dump(pkg, tags, filters)
        base = os.path.splitext(os.path.basename(f))[0]
        with open(os.path.join(out_dir, base + '.json'), 'w', encoding='utf-8') as o:
            json.dump({'file': f, 'package': pkg.summary.get('package_name'), 'tags': rows}, o, indent=1)
        with open(os.path.join(out_dir, base + '.txt'), 'w', encoding='utf-8') as o:
            last = object()
            for r in rows:
                if r['export'] != last:
                    o.write('== export %s\n' % r['export'])
                    last = r['export']
                o.write('  %-78s %-40s %s\n' % (r['path'], r['type'], json.dumps(r['value'])))
        index.append({'file': f, 'tags': len(rows)})
    with open(os.path.join(out_dir, 'index.json'), 'w', encoding='utf-8') as o:
        json.dump(index, o, indent=1)
    print('files=%d' % len(files))
    return 0


if __name__ == '__main__':
    sys.exit(main())
