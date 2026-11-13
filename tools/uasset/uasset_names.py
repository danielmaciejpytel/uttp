"""Offline, read-only .uasset/.umap header reader (UE 5.x editor packages, uncooked).

Reads the package summary, name table, import table and export table of every package under a content
root and writes, per package, a JSON with the graph "vocabulary": referenced node classes (K2Node_*,
AnimGraphNode_*, AnimStateNode*, RigVM*), functions, imported packages/objects, exports (with their class),
and the full name table (variable / pin / function / enum names all live there as FNames).
No engine is needed; nothing is written next to the source files.

Usage (system python):
  python -I uasset_names.py <content_root> <mount_point> <out_dir> [--all]
    content_root  e.g. "<GASP-ALS project>\\Plugins\\GASPALS\\Content"
    mount_point   e.g. "/GASPALS"
    out_dir       per-package JSON goes to <out_dir>\\<flattened package path>.json, plus index.json
    --all         write per-package JSON for every package (default: only "logic" packages, see LOGIC_CLASSES)
"""
import json
import os
import struct
import sys

PACKAGE_FILE_TAG = 0x9E2A83C1
PKG_FILTER_EDITOR_ONLY = 0x80000000

# EUnrealEngineObjectUE5Version (Engine/Source/Runtime/Core/Public/UObject/ObjectVersion.h, UE 5.8)
UE5_OPTIONAL_RESOURCES = 1003
UE5_REMOVE_OBJECT_EXPORT_PACKAGE_GUID = 1005
UE5_TRACK_OBJECT_EXPORT_IS_INHERITED = 1006
UE5_ADD_SOFTOBJECTPATH_LIST = 1008
UE5_SCRIPT_SERIALIZATION_OFFSET = 1010
UE5_METADATA_SERIALIZATION_OFFSET = 1014
UE5_VERSE_CELLS = 1015
UE5_PACKAGE_SAVED_HASH = 1016

LOGIC_CLASSES = {
    'Blueprint', 'AnimBlueprint', 'WidgetBlueprint', 'ControlRigBlueprint', 'EditorUtilityBlueprint',
    'EditorUtilityWidgetBlueprint', 'BlueprintGeneratedClass', 'AnimBlueprintGeneratedClass',
    'WidgetBlueprintGeneratedClass', 'ChooserTable', 'ProxyTable', 'ProxyAsset', 'PoseSearchDatabase',
    'PoseSearchSchema', 'PoseSearchNormalizationSet', 'DataTable', 'UserDefinedEnum', 'UserDefinedStruct',
    'PrimaryDataAsset', 'DataAsset', 'CurveTable', 'IKRigDefinition', 'IKRetargeter', 'MirrorDataTable',
    'InputAction', 'InputMappingContext', 'Skeleton', 'BlendProfile', 'AnimMontage', 'BlendSpace',
    'BlendSpace1D', 'AimOffsetBlendSpace', 'AimOffsetBlendSpace1D', 'PoseAsset', 'AnimComposite',
    'CameraAnimationSequence', 'CurveFloat', 'CurveVector', 'StateTree', 'MetaSoundSource', 'CameraRigAsset',
    'CameraAsset', 'CameraDirector', 'ControlRigShapeLibrary', 'PhysicsAsset',
}
NODE_PREFIXES = ('K2Node_', 'AnimGraphNode_', 'AnimStateNode', 'AnimStateTransitionNode', 'AnimStateEntryNode',
                 'AnimStateConduitNode', 'AnimStateAliasNode', 'EdGraphNode_', 'RigVM', 'ControlRigGraphNode',
                 'MaterialGraphNode', 'AnimationStateMachineGraph', 'AnimationTransitionGraph',
                 'AnimationStateGraph', 'AnimationGraph', 'AnimationConduitGraph', 'AnimationCustomTransitionGraph')


class Reader:
    def __init__(self, data):
        self.d = data
        self.p = 0

    def i32(self):
        v = struct.unpack_from('<i', self.d, self.p)[0]
        self.p += 4
        return v

    def u32(self):
        v = struct.unpack_from('<I', self.d, self.p)[0]
        self.p += 4
        return v

    def i64(self):
        v = struct.unpack_from('<q', self.d, self.p)[0]
        self.p += 8
        return v

    def skip(self, n):
        self.p += n

    def fstring(self):
        n = self.i32()
        if n == 0:
            return ''
        if n < 0:
            raw = self.d[self.p:self.p + (-n) * 2]
            self.p += (-n) * 2
            return raw.decode('utf-16-le', errors='replace').rstrip('\x00')
        raw = self.d[self.p:self.p + n]
        self.p += n
        return raw.decode('latin-1').rstrip('\x00')


def read_package(path):
    with open(path, 'rb') as f:
        data = f.read()
    r = Reader(data)
    s = {}
    tag = r.u32()
    if tag != PACKAGE_FILE_TAG:
        raise ValueError('not a UE package (tag %08x)' % tag)
    legacy = r.i32()
    s['legacy_file_version'] = legacy
    if legacy != -4:
        r.i32()  # LegacyUE3Version
    s['file_version_ue4'] = r.i32()
    s['file_version_ue5'] = r.i32() if legacy <= -8 else 0
    s['file_version_licensee'] = r.i32()
    ue5 = s['file_version_ue5']
    if ue5 >= UE5_PACKAGE_SAVED_HASH:
        r.skip(20)  # FIoHash SavedHash
        s['total_header_size'] = r.i32()
    if legacy <= -2:
        n = r.i32()
        r.skip(n * 20)  # optimized custom versions: FGuid + int32
        s['custom_version_count'] = n
    if ue5 < UE5_PACKAGE_SAVED_HASH:
        s['total_header_size'] = r.i32()
    s['package_name'] = r.fstring()
    flags = r.u32()
    s['package_flags'] = flags
    name_count, name_offset = r.i32(), r.i32()
    if ue5 >= UE5_ADD_SOFTOBJECTPATH_LIST:
        s['soft_object_paths_count'], s['soft_object_paths_offset'] = r.i32(), r.i32()
    if not flags & PKG_FILTER_EDITOR_ONLY:
        r.fstring()  # LocalizationId
    r.i32(), r.i32()  # gatherable text
    export_count, export_offset = r.i32(), r.i32()
    import_count, import_offset = r.i32(), r.i32()

    # name table
    names = []
    r.p = name_offset
    for _ in range(name_count):
        names.append(r.fstring())
        r.skip(4)  # case-preserving + non-case-preserving hashes

    def fname(idx, num):
        base = names[idx] if 0 <= idx < len(names) else '<bad:%d>' % idx
        return base if num == 0 else '%s_%d' % (base, num - 1)

    imports = []
    r.p = import_offset
    for _ in range(import_count):
        cp = fname(r.i32(), r.i32())
        cn = fname(r.i32(), r.i32())
        outer = r.i32()
        on = fname(r.i32(), r.i32())
        pn = fname(r.i32(), r.i32())
        if ue5 >= UE5_OPTIONAL_RESOURCES:
            r.skip(4)
        imports.append({'class_package': cp, 'class': cn, 'outer': outer, 'name': on, 'package': pn})

    exports = []
    r.p = export_offset
    for _ in range(export_count):
        ci, si, ti, oi = r.i32(), r.i32(), r.i32(), r.i32()
        on = fname(r.i32(), r.i32())
        oflags = r.u32()
        size, offset = r.i64(), r.i64()
        r.skip(12)  # bForcedExport, bNotForClient, bNotForServer
        if ue5 < UE5_REMOVE_OBJECT_EXPORT_PACKAGE_GUID:
            r.skip(16)
        if ue5 >= UE5_TRACK_OBJECT_EXPORT_IS_INHERITED:
            r.skip(4)
        r.skip(4)  # PackageFlags
        r.skip(8)  # bNotAlwaysLoadedForEditorGame, bIsAsset
        if ue5 >= UE5_OPTIONAL_RESOURCES:
            r.skip(4)
        r.skip(20)  # preload dependency indices
        if ue5 >= UE5_SCRIPT_SERIALIZATION_OFFSET:
            r.skip(16)
        exports.append({'class_index': ci, 'super_index': si, 'template_index': ti, 'outer': oi, 'name': on,
                        'flags': oflags, 'serial_size': size, 'serial_offset': offset})

    def res_name(idx):
        if idx == 0:
            return None
        if idx < 0:
            i = -idx - 1
            return imports[i]['name'] if i < len(imports) else '<bad import %d>' % idx
        i = idx - 1
        return exports[i]['name'] if i < len(exports) else '<bad export %d>' % idx

    def full_path(idx, depth=0):
        if idx == 0 or depth > 32:
            return ''
        if idx < 0:
            imp = imports[-idx - 1]
            parent = full_path(imp['outer'], depth + 1)
            return (parent + ('.' if parent and imp['outer'] < 0 and imports[-imp['outer'] - 1]['class'] == 'Package' else ':' if parent else '') + imp['name'])
        exp = exports[idx - 1]
        parent = full_path(exp['outer'], depth + 1)
        return (parent + '.' if parent else '') + exp['name']

    for i, e in enumerate(exports):
        e['class'] = res_name(e['class_index']) or 'Class'
        e['super'] = res_name(e['super_index'])
        e['outer_path'] = full_path(e['outer'])
        e['path'] = full_path(i + 1)
    for i, imp in enumerate(imports):
        imp['path'] = full_path(-i - 1)
    s['_data'] = data
    return s, names, imports, exports


def decode_enum_names(data, names, export):
    """UEnum::Serialize writes TArray<TPair<FName,int64>> Names after the tagged properties. Scan the export blob for
    the first array whose FNames are all 'Enum::Entry' names; the array order is the value order."""
    off, size = export.get('serial_offset', 0), export.get('serial_size', 0)
    blob = data[off:off + size]
    n_names = len(names)
    for i in range(0, max(0, len(blob) - 4)):
        n = struct.unpack_from('<i', blob, i)[0]
        if not 1 <= n <= 512 or i + 4 + n * 16 > len(blob):
            continue
        ok = True
        out = []
        for k in range(n):
            idx, num, val = struct.unpack_from('<iiq', blob, i + 4 + k * 16)
            if not (0 <= idx < n_names) or num < 0 or not (-1 <= val <= 1 << 40) or '::' not in names[idx]:
                ok = False
                break
            nm = names[idx] if num == 0 else '%s_%d' % (names[idx], num - 1)
            out.append({'name': nm, 'value': val})
        if ok:
            return out
    return None


def main_asset_class(pkg_short, exports):
    for e in exports:
        if e['outer'] == 0 and e['name'] == pkg_short:
            return e['class']
    for e in exports:
        if e['outer'] == 0 and not e['class'].endswith('GeneratedClass') and e['class'] not in ('MetaData', 'Package', 'ObjectRedirector'):
            return e['class']
    return exports[0]['class'] if exports else None


def summarize(names, imports, exports):
    node_classes = sorted({i['name'] for i in imports if i['class'] == 'Class' and i['name'].startswith(NODE_PREFIXES)})
    node_instances = {}
    graphs = []
    for e in exports:
        if e['class'].startswith(NODE_PREFIXES):
            node_instances.setdefault(e['class'], []).append(e['path'])
        if 'Graph' in e['class'] and not e['class'].startswith(('K2Node_', 'AnimGraphNode_')):
            graphs.append({'name': e['name'], 'class': e['class'], 'outer': e['outer_path']})
    imported_functions = sorted({i['path'] for i in imports if i['class'] == 'Function'})
    own_functions = sorted({e['path'] for e in exports if e['class'] == 'Function'})
    imported_classes = sorted({i['path'] for i in imports if i['class'] in ('Class', 'BlueprintGeneratedClass',
                               'AnimBlueprintGeneratedClass', 'WidgetBlueprintGeneratedClass', 'ScriptStruct',
                               'UserDefinedStruct', 'Enum', 'UserDefinedEnum')})
    imported_packages = sorted({i['name'] for i in imports if i['class'] == 'Package'})
    imported_assets = sorted({i['path'] for i in imports if i['class'] not in ('Package', 'Class', 'Function',
                              'ScriptStruct', 'Enum', 'DelegateFunction', 'SparseDelegateFunction')
                              and i['outer'] < 0 and imports[-i['outer'] - 1]['class'] == 'Package'})
    asset_names = sorted({n for n in names if n.startswith('/') and not n.startswith('/Script/')})
    script_names = sorted({n for n in names if n.startswith('/Script/')})
    pinish = sorted({n for n in names if not n.startswith('/') and n not in node_classes
                     and not n.endswith('Property') and n not in ('None', 'Class', 'Package', 'Function', 'Object')})
    return {
        'node_classes_imported': node_classes,
        'node_instances_by_class': {k: len(v) for k, v in sorted(node_instances.items())},
        'node_instances': {k: sorted(v) for k, v in sorted(node_instances.items())},
        'graphs': graphs,
        'functions_defined': own_functions,
        'functions_called_imported': imported_functions,
        'classes_structs_enums_imported': imported_classes,
        'packages_imported': imported_packages,
        'assets_imported': imported_assets,
        'asset_paths_in_names': asset_names,
        'script_paths_in_names': script_names,
        'member_pin_variable_names_candidates': pinish,
    }


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return 2
    root, mount, out = sys.argv[1], sys.argv[2].rstrip('/'), sys.argv[3]
    write_all = '--all' in sys.argv[4:]
    os.makedirs(out, exist_ok=True)
    index = []
    for dp, _, files in os.walk(root):
        for fn in sorted(files):
            if not fn.endswith(('.uasset', '.umap')):
                continue
            fp = os.path.join(dp, fn)
            rel = os.path.relpath(fp, root).replace('\\', '/')
            pkg = mount + '/' + rel.rsplit('.', 1)[0]
            short = pkg.rsplit('/', 1)[-1]
            rec = {'package': pkg, 'file': fp, 'size': os.path.getsize(fp)}
            try:
                s, names, imports, exports = read_package(fp)
                cls = main_asset_class(short, exports)
                rec.update({'class': cls, 'names': len(names), 'imports': len(imports), 'exports': len(exports),
                            'file_version_ue4': s['file_version_ue4'], 'file_version_ue5': s['file_version_ue5']})
                if write_all or cls in LOGIC_CLASSES or (cls or '').endswith('_C'):
                    doc = {'package': pkg, 'file': fp, 'main_class': cls, 'summary': {k: v for k, v in s.items() if k != '_data'}}
                    doc.update(summarize(names, imports, exports))
                    doc['exports'] = [{k: e[k] for k in ('path', 'class', 'super', 'serial_size')} for e in exports]
                    if cls == 'UserDefinedEnum':
                        main = next((e for e in exports if e['outer'] == 0 and e['class'] == 'UserDefinedEnum'), None)
                        doc['enum_entries'] = decode_enum_names(s['_data'], names, main) if main else None
                    doc['imports'] = [{k: i[k] for k in ('path', 'class', 'class_package', 'package')} for i in imports]
                    doc['names'] = names
                    of = os.path.join(out, pkg.strip('/').replace('/', '__') + '.json')
                    with open(of, 'w', encoding='utf-8') as f:
                        json.dump(doc, f, indent=1)
                    rec['json'] = os.path.basename(of)
            except Exception as ex:  # keep going; report per package
                rec['error'] = '%s: %s' % (type(ex).__name__, ex)
            index.append(rec)
    with open(os.path.join(out, 'index.json'), 'w', encoding='utf-8') as f:
        json.dump({'root': root, 'mount': mount, 'packages': index}, f, indent=1)
    counts = {}
    for rec in index:
        counts[rec.get('class') or 'ERROR'] = counts.get(rec.get('class') or 'ERROR', 0) + 1
    print(json.dumps({'packages': len(index), 'written': sum(1 for r in index if 'json' in r),
                      'errors': sum(1 for r in index if 'error' in r), 'classes': dict(sorted(counts.items()))}, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
