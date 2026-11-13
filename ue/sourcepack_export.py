"""READ-ONLY full animation export of the source pack project for the UTPP port.

Run inside UnrealEditor-Cmd (-run=pythonscript) on the source pack UE project (GASPALS.uproject), only while no
other editor has the project open (same flags as the 2026-10-12 extraction), e.g. in the background:
  "<UE install>\\Engine\\Binaries\\Win64\\UnrealEditor-Cmd.exe" "<source pack project>\\GASPALS.uproject"
    -run=pythonscript -script="<project folder>/Source/ue/sourcepack_export.py" -EnablePlugins=PythonScriptPlugin
    -unattended -nop4 -nosplash -NoSound -culture=en -abslog="<project folder>/Research/logs/sourcepack-export-ue.log"
  then poll <out>\\_progress.log until "DONE".

Never saves, resaves or modifies a package: it only loads assets, runs FBX export tasks into files OUTSIDE the project
and writes JSON next to them. The editor itself may write its own caches (Saved\\, Intermediate\\, DDC).

Selection:
  - every animation asset under /GASPALS/Characters/UEFN_Mannequin/Animations/ (locomotion, crouch, idle + idle breaks,
    jumps, AimOffset (42 + BS), GetUp, Flail, Traversal incl. Catch, ExperimentalStateMachineData, lean poses);
  - OverlaySystem folders, the logic assets whose animation references are recorded and the AO stand blend space are
    read from section ue_export of Research\\inventory\\sourcepack-names.json (source pack names
    stay outside the git repo): overlay_export (styles, selected pose overlays, BasePoses, Transitions), overlay_data_only
    (no animation asset of their own, the DA/ABP and the animations they reference are recorded), overlay_unused
    (exported as local data but unused at runtime, keeps sourcepack-clip-map.json consistent:
    every clip with motion_src sourcepack_export is exported); meta/manifest field selection_kind = export |
    data_only | unused;
  - not exported: UE5_Mannequins corrective poses, MetaHumans.
  Names file: script argument --names PATH (-script="<this file> --names <PATH>") > env UTPP_SOURCEPACK_NAMES > default
  Research\\inventory\\sourcepack-names.json.
  Offline check without UE (system Python, prints the selection from the read-only extraction, writes nothing unless
  --json is given):
    python -I Source\\ue\\sourcepack_export.py [--names PATH] [--assets Extracted\\sourcepack\\sourcepack\\assets.json] [--json OUT]

Output (default <project folder>\\Animations\\sourcepack\\als_export):
  fbx\\<Name>.fbx    every AnimSequence: skeletal animation incl. root bone, no mesh, same FbxExportOption settings as
                    Source\\ue\\export_sourcepack.py (retarget_batch.py reads both exports the same way). The UE FBX exporter samples
                    the data-model bone tracks, so additive assets are written as FULL source poses, not deltas.
  meta\\<Name>.json  per asset (sequence, montage, blend space): skeleton, frames, fps, length, root motion, additive type +
                    base pose (ref_pose_seq, ref_frame_index), float curves per frame + raw keys, notifies (class, name,
                    time, duration, track, props), sync markers, montage sections/slots, blend-space samples, PoseSearch
                    databases, referencers, source FBX import data, FBX MD5, comparison with Animations\\sourcepack\\SourcePack_export.
  manifest.json     everything exported + per-group counts, failures, AO / lean blend-space usage, overlay data, totals.
  _progress.log     progress of the run.

Env (all optional):
  UTPP_SOURCEPACK_EXPORT_OUT            output folder
  UTPP_SOURCEPACK_EXPORT_ONLY           regex on the object path (probe runs)
  UTPP_SOURCEPACK_EXPORT_LIMIT          max number of assets (probe runs, 0 = all)
  UTPP_SOURCEPACK_EXPORT_SKIP_EXISTING  1 = keep an existing fbx + meta pair (resume after a crash)
  UTPP_SOURCEPACK_EXPORT_DIR               source pack 5.8 export used for the comparison (default Animations\\sourcepack\\SourcePack_export)
  UTPP_SOURCEPACK_EX                    read-only extraction (default Extracted\\sourcepack\\sourcepack) for PoseSearch database usage
  UTPP_SOURCEPACK_NAMES                 source pack names data file (section ue_export); the --names script argument wins
"""
import hashlib
import json
import os
import re
import sys
import time
import traceback

try:
    import unreal
except ImportError:         # system Python: offline selection check only (offline_main), nothing is exported
    unreal = None

def _project_root():
    """The project folder: UTPP_PROJECT_ROOT, else two levels above this script (<root>/Source/ue/<script>)."""
    root = os.environ.get('UTPP_PROJECT_ROOT')
    if root:
        return root
    import sys
    here = globals().get('__file__') or (sys.argv[0] if getattr(sys, 'argv', None) else '')
    if not here:
        raise RuntimeError('Set the environment variable UTPP_PROJECT_ROOT (the project folder)')
    return os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(here)), '..', '..'))


PROJECT_ROOT = _project_root()
OUT = os.environ.get('UTPP_SOURCEPACK_EXPORT_OUT', os.path.join(PROJECT_ROOT, 'Animations', 'sourcepack', 'als_export'))
GASP_EXPORT = os.environ.get('UTPP_SOURCEPACK_EXPORT_DIR', os.path.join(PROJECT_ROOT, 'Animations', 'sourcepack', 'SourcePack_export'))
EX = os.environ.get('UTPP_SOURCEPACK_EX', os.path.join(PROJECT_ROOT, 'Extracted', 'sourcepack', 'sourcepack'))
ONLY = re.compile(os.environ['UTPP_SOURCEPACK_EXPORT_ONLY']) if os.environ.get('UTPP_SOURCEPACK_EXPORT_ONLY') else None
LIMIT = int(os.environ.get('UTPP_SOURCEPACK_EXPORT_LIMIT', '0') or 0)
SKIP_EXISTING = os.environ.get('UTPP_SOURCEPACK_EXPORT_SKIP_EXISTING', '0') == '1'
NAMES_DEFAULT = os.path.join(PROJECT_ROOT, 'Research', 'inventory', 'sourcepack-names.json')

MOUNT = '/GASPALS/'
UE_PROJECT_DIR = os.environ.get('UTPP_SOURCEPACK_PROJECT') or os.path.join(os.path.dirname(_project_root()), 'Unreal Engine', 'Test', 'ALSV5', 'GASP-ALS')   # folder of GASPALS.uproject
CONTENT_DIR = os.path.join(UE_PROJECT_DIR, 'Plugins', 'GASPALS', 'Content')
UEFN_ANIMS = '/GASPALS/Characters/UEFN_Mannequin/Animations/'
OVERLAY = '/GASPALS/OverlaySystem/'

SEQ_CLASS = 'AnimSequence'
META_CLASSES = {'AnimMontage', 'AnimComposite', 'BlendSpace', 'BlendSpace1D', 'AimOffsetBlendSpace', 'AimOffsetBlendSpace1D',
                'PoseAsset'}
ANIM_CLASSES = META_CLASSES | {SEQ_CLASS, 'AnimStreamable'}


# ---------------------------------------------------------------- selection data and pure selection (no UE)
def names_file(argv=None, environ=None):
    """source pack names data file: script argument --names PATH / --names=PATH > env UTPP_SOURCEPACK_NAMES > NAMES_DEFAULT."""
    argv = list((getattr(sys, 'argv', None) or [])[1:] if argv is None else argv)
    environ = os.environ if environ is None else environ
    for i, x in enumerate(argv):
        if x == '--names' and i + 1 < len(argv):
            return argv[i + 1]
        if x.startswith('--names='):
            return x.split('=', 1)[1]
    return environ.get('UTPP_SOURCEPACK_NAMES') or NAMES_DEFAULT


def load_selection(path):
    """Section ue_export of the names file -> overlay folder lists, logic assets, AO stand blend space."""
    with open(path, encoding='utf-8') as f:
        d = json.load(f)
    if d.get('format') != 'utpp-gaspals-names':
        raise RuntimeError('names file %s: unexpected format %r' % (path, d.get('format')))
    u = d['ue_export']
    return {'overlay_export': [str(x) for x in u['overlay_export']],
            'overlay_data_only': [str(x) for x in u['overlay_data_only']],
            'overlay_unused': [str(x) for x in u['overlay_unused']],
            'logic_assets': [str(x) for x in u['logic_assets']],
            'ao_stand_blendspace': str(u['ao_stand_blendspace'])}


NAMES_FILE = names_file()
SEL = load_selection(NAMES_FILE)
OVERLAY_EXPORT = SEL['overlay_export']
OVERLAY_DATA_ONLY = SEL['overlay_data_only']
OVERLAY_UNUSED = SEL['overlay_unused']
# Blueprints whose animation references are recorded (overlay layering, character, ABP)
LOGIC_ASSETS = SEL['logic_assets']
AO_STAND_BS = SEL['ao_stand_blendspace']


def group_of(folder, sel=None):
    """-> (group, kind) with kind export | data_only | unused | logic | None (outside the scope)."""
    sel = SEL if sel is None else sel
    f = str(folder).rstrip('/') + '/'
    if f.startswith(UEFN_ANIMS):
        rel = f[len(UEFN_ANIMS):].strip('/')
        return ('UEFN/' + rel) if rel else 'UEFN', 'export'
    if f.startswith(OVERLAY):
        rel = f[len(OVERLAY):].strip('/')
        for lst, kind in ((sel['overlay_export'], 'export'), (sel['overlay_data_only'], 'data_only'),
                          (sel['overlay_unused'], 'unused')):
            for x in lst:
                if rel == x or rel.startswith(x + '/'):
                    return 'Overlay/' + x, kind
        return 'Overlay/' + (rel or '.'), 'logic'
    return None, None


def select_assets(items, sel=None):
    """Pure selection, same for the UE run and the offline check.
    items: [(package_path, package_name, asset_name, asset_class, payload)] in registry order ->
    (selected [(group, payload)], sequences first then by object path; not_exported {group or top folders: {class: n}})."""
    selected, not_exported = [], {}
    for package_path, package_name, asset_name, cls, payload in items:
        if cls not in ANIM_CLASSES:
            continue
        grp, kind = group_of(package_path, sel)
        if kind in ('export', 'data_only', 'unused'):
            selected.append((grp, cls, '%s.%s' % (package_name, asset_name), payload))
        else:
            key = grp or '/'.join(str(package_path).replace(MOUNT, '', 1).split('/')[:3])
            not_exported.setdefault(key, {}).setdefault(cls, 0)
            not_exported[key][cls] += 1
    # sequences first (so a crash in a montage/BS dump cannot cost FBX output), then by path
    selected.sort(key=lambda x: (x[1] != SEQ_CLASS, x[2]))
    return [(grp, payload) for grp, _, _, payload in selected], not_exported


def offline_main(argv):
    """System Python, no UE: print the selection of a read-only extraction (Extracted\\sourcepack\\sourcepack\\assets.json)."""
    import argparse
    ap = argparse.ArgumentParser(prog='sourcepack_export.py (offline selection check, no UE)')
    ap.add_argument('--names', default=NAMES_FILE, help='source pack names data file (resolved: %(default)s)')
    ap.add_argument('--assets', default=os.path.join(EX, 'assets.json'), help='read-only asset extraction')
    ap.add_argument('--json', default=None, help='also write the selection here as JSON')
    a = ap.parse_args(argv)
    with open(a.assets, encoding='utf-8') as f:
        assets = [x for x in json.load(f)['assets'] if str(x['path']).startswith(MOUNT)]
    sel = load_selection(a.names)
    selected, not_exported = select_assets(
        [(x['folder'], x['package'], x['name'], x['class'], x) for x in assets], sel)
    rows = [{'path': x['path'], 'class': x['class'], 'group': grp, 'kind': group_of(x['folder'], sel)[1]}
            for grp, x in selected]
    for r in rows:
        print('%s\t%s\t%s\t%s' % (r['kind'], r['group'], r['class'], r['path']))
    kinds, classes = {}, {}
    for r in rows:
        kinds[r['kind']] = kinds.get(r['kind'], 0) + 1
        classes[r['class']] = classes.get(r['class'], 0) + 1
    summary = {'names_file': a.names, 'assets_file': a.assets, 'registry_assets': len(assets), 'selected': len(rows),
               'by_kind': dict(sorted(kinds.items())), 'by_class': dict(sorted(classes.items())),
               'not_exported_anim_assets': not_exported, 'selection': sel}
    print('UTPP_SOURCEPACK_SELECTION ' + json.dumps({k: summary[k] for k in ('selected', 'by_kind', 'by_class')}))
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(dict(summary, rows=rows), f, indent=1)
    return 0


if unreal is None:
    sys.exit(offline_main((getattr(sys, 'argv', None) or [])[1:]))

FBX_DIR = os.path.join(OUT, 'fbx')
META_DIR = os.path.join(OUT, 'meta')
os.makedirs(FBX_DIR, exist_ok=True)
os.makedirs(META_DIR, exist_ok=True)
PROGRESS = os.path.join(OUT, '_progress.log')
T0 = time.time()
lib = unreal.AnimationLibrary


def progress(msg):
    line = '[%7.1fs] %s' % (time.time() - T0, msg)
    unreal.log('[UTPP-SOURCEPACK-EXPORT] ' + line)
    with open(PROGRESS, 'a', encoding='utf-8') as f:
        f.write(line + '\n')


# ---------------------------------------------------------------- generic reflection dump (as sourcepack_extract.py)
_PROP_RE = re.compile(r'^\s*-\s+``(\w+)``\s+\((.*?)\):\s+\[(Read-Write|Read-Only)\]', re.M)
_prop_cache = {}
SEQ_TYPES = tuple([list, tuple] + [getattr(unreal, n) for n in ('Array', 'FixedArray', 'Set') if hasattr(unreal, n)])
MAP_TYPE = getattr(unreal, 'Map', None)


def prop_names(t):
    if t in _prop_cache:
        return _prop_cache[t]
    names, seen = [], set()
    for c in getattr(t, '__mro__', [t]):
        for m in _PROP_RE.finditer(getattr(c, '__doc__', None) or ''):
            if m.group(1) not in seen:
                seen.add(m.group(1))
                names.append(m.group(1))
    _prop_cache[t] = names
    return names


def obj_path(o):
    try:
        return o.get_path_name()
    except Exception:
        return str(o)


def short(p):
    return str(p).rsplit('.', 1)[-1].rsplit('/', 1)[-1] if p else p


def is_inner_of(o, owner):
    try:
        cur = o.get_outer()
        for _ in range(8):
            if cur is None:
                return False
            if cur == owner:
                return True
            cur = cur.get_outer()
    except Exception:
        pass
    return False


def plain(v, depth=0, owner=None):
    if v is None or isinstance(v, (bool, int, float, str)):
        return v
    if isinstance(v, unreal.EnumBase):
        return getattr(v, 'name', str(v))
    if isinstance(v, (unreal.Name, unreal.Text)):
        return str(v)
    if isinstance(v, SEQ_TYPES):
        return [plain(x, depth + 1, owner) for x in list(v)]
    if MAP_TYPE is not None and isinstance(v, MAP_TYPE):
        return {str(plain(k, depth + 1, owner)): plain(x, depth + 1, owner) for k, x in v.items()}
    if isinstance(v, unreal.Class):
        return obj_path(v)
    if isinstance(v, unreal.Object):
        if owner is not None and depth < 6 and is_inner_of(v, owner):
            return dump_object(v, depth + 1, owner)
        return obj_path(v)
    if isinstance(v, unreal.StructBase):
        if depth > 10:
            return str(v)
        d = {}
        names = prop_names(type(v)) or [a for a in dir(v) if not a.startswith('_') and not callable(getattr(v, a, None))]
        for n in names:
            try:
                d[n] = plain(v.get_editor_property(n), depth + 1, owner)
            except Exception:
                try:
                    d[n] = plain(getattr(v, n), depth + 1, owner)
                except Exception:
                    pass
        return d if d else str(v)
    return str(v)


def dump_object(o, depth=0, owner=None, exclude=()):
    d = {'_class': o.get_class().get_name(), '_path': obj_path(o)}
    own = owner if owner is not None else o
    for n in prop_names(type(o)):
        if n in exclude or n == 'thumbnail_info':
            continue
        try:
            d[n] = plain(o.get_editor_property(n), depth, own)
        except Exception:
            d.setdefault('_unreadable', []).append(n)
    return d


def prop(o, name, default=None):
    try:
        return o.get_editor_property(name)
    except Exception:
        return default


def tr(t):
    try:
        l, r, s = t.translation, t.rotation, t.scale3d
        return {'translation': [l.x, l.y, l.z], 'rotation_xyzw': [r.x, r.y, r.z, r.w], 'scale': [s.x, s.y, s.z]}
    except Exception as e:
        return {'_error': str(e)[:200]}


def md5_file(path):
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------- registry and selection
registry = unreal.AssetRegistryHelpers.get_asset_registry()
try:
    registry.scan_paths_synchronous(['/GASPALS'], True)
except Exception as e:
    progress('scan_paths_synchronous: %s' % e)
ALL = list(registry.get_assets_by_path('/GASPALS', recursive=True))


def ad_class(ad):
    try:
        return str(ad.asset_class_path.asset_name)
    except Exception:
        return str(ad.asset_class)


def ad_path(ad):
    return '%s.%s' % (ad.package_name, ad.asset_name)


SELECTED, NOT_EXPORTED = select_assets(
    [(str(ad.package_path), str(ad.package_name), str(ad.asset_name), ad_class(ad), ad) for ad in ALL])
if ONLY is not None:
    SELECTED = [x for x in SELECTED if ONLY.search(ad_path(x[1]))]
if LIMIT:
    SELECTED = SELECTED[:LIMIT]
names = {}
for grp, ad in SELECTED:
    names.setdefault(str(ad.asset_name), []).append(ad_path(ad))
DUPLICATE_NAMES = {n: p for n, p in names.items() if len(p) > 1}
progress('START out=%s registry=%d selected=%d (only=%s limit=%s skip_existing=%s) duplicate_names=%d engine=%s names=%s' % (
    OUT, len(ALL), len(SELECTED), ONLY.pattern if ONLY else None, LIMIT, SKIP_EXISTING, len(DUPLICATE_NAMES),
    unreal.SystemLibrary.get_engine_version(), NAMES_FILE))


def file_name(ad, grp):
    n = str(ad.asset_name)
    return (grp.replace('/', '_') + '__' + n) if n in DUPLICATE_NAMES else n


# ---------------------------------------------------------------- side information
DEP_OPTS = unreal.AssetRegistryDependencyOptions()
for k, v in (('include_soft_package_references', True), ('include_hard_package_references', True),
             ('include_searchable_names', False), ('include_soft_management_references', False),
             ('include_hard_management_references', False)):
    try:
        DEP_OPTS.set_editor_property(k, v)
    except Exception:
        pass
PKG_CLASS = {str(ad.package_name): ad_class(ad) for ad in ALL}


def referencers(pkg):
    try:
        return sorted({str(x) for x in (registry.get_referencers(pkg, DEP_OPTS) or [])})
    except Exception as e:
        return ['_error: %s' % str(e)[:200]]


def anim_dependencies(pkg):
    try:
        deps = registry.get_dependencies(pkg, DEP_OPTS) or []
    except Exception as e:
        return ['_error: %s' % str(e)[:200]]
    return sorted({str(d) for d in deps if PKG_CLASS.get(str(d)) in ANIM_CLASSES})


def load_json(path, what):
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except Exception as e:
        progress('cannot read %s (%s): %s' % (what, path, e))
        return None


ps = load_json(os.path.join(EX, 'posesearch.json'), 'PoseSearch extraction') or {}
ANIM_TO_DBS = ps.get('anim_to_databases') or {}
del ps

RE_MD5 = re.compile(rb'"FileMD5"\s*:\s*"([0-9a-fA-F]{32})"')
RE_REL = re.compile(rb'"RelativeFilename"\s*:\s*"([^"]{1,600})"')
RE_TS = re.compile(rb'"Timestamp"\s*:\s*"([^"]{1,64})"')


def source_import(pkg):
    """AssetImportData strings read from the .uasset bytes (read-only)."""
    p = str(pkg)
    if not p.startswith(MOUNT):
        return None
    f = os.path.join(CONTENT_DIR, p[len(MOUNT):].replace('/', os.sep) + '.uasset')
    if not os.path.exists(f):
        return {'_error': 'package file not found'}
    with open(f, 'rb') as fh:
        data = fh.read()
    return {'uasset_bytes': len(data),
            'files': [m.group(1).decode('utf-8', 'replace').replace('\\\\', '\\') for m in RE_REL.finditer(data)][:3],
            'md5': [m.group(1).decode('latin-1').lower() for m in RE_MD5.finditer(data)][:3],
            'timestamps': [m.group(1).decode('latin-1') for m in RE_TS.finditer(data)][:3]}


def gasp_export_compare(name, length, num_frames):
    mp = os.path.join(GASP_EXPORT, 'meta', name + '.json')
    if not os.path.exists(mp):
        return {'exists': False}
    g = load_json(mp, 'SourcePack_export meta') or {}
    gl, gn = g.get('length_s'), g.get('num_frames')
    same_len = gl is not None and length is not None and abs(float(gl) - float(length)) < 1e-4
    same_frames = gn is not None and num_frames is not None and int(gn) == int(num_frames)
    return {'exists': True, 'path': g.get('path'), 'length_s': gl, 'num_frames': gn, 'same_length': same_len,
            'same_frames': same_frames, 'identical_length_frames': bool(same_len and same_frames),
            'fbx_exists': os.path.exists(os.path.join(GASP_EXPORT, 'fbx', name + '.fbx')),
            'curve_names': sorted((g.get('curves') or {}).keys())}


# ---------------------------------------------------------------- per-asset readers
def notify_events(a):
    """Notifies with the key names of Animations\\sourcepack\\SourcePack_export meta (notify_name, trigger_time, duration, class) + extras."""
    out, track_of = [], {}
    try:
        for tn in lib.get_animation_notify_track_names(a):
            try:
                for ev in lib.get_animation_notify_events_for_track(a, tn):
                    key = (str(ev.get_editor_property('notify_name')), round(float(lib.get_anim_notify_event_trigger_time(ev)), 5))
                    track_of.setdefault(key, str(tn))
            except Exception:
                pass
    except Exception:
        pass
    try:
        events = lib.get_animation_notify_events(a)
    except Exception as e:
        return [{'_error': str(e)[:300]}]
    for ev in events:
        rec = {}
        try:
            rec['notify_name'] = str(ev.get_editor_property('notify_name'))
        except Exception:
            pass
        for key, fn in (('trigger_time', 'get_anim_notify_event_trigger_time'), ('duration', 'get_anim_notify_event_duration')):
            try:
                rec[key] = float(getattr(lib, fn)(ev))
            except Exception as e:
                rec[key + '_error'] = str(e)[:200]
        if 'trigger_time' in rec and 'duration' in rec:
            rec['end_time'] = rec['trigger_time'] + rec['duration']
        obj = None
        for pn in ('notify_state_class', 'notify'):
            obj = prop(ev, pn)
            if obj:
                rec['kind'] = 'state' if pn == 'notify_state_class' else 'notify'
                break
        if obj:
            rec['class'] = obj.get_class().get_name()
            rec['class_path'] = obj.get_class().get_path_name()
            try:
                props = dump_object(obj)
                props.pop('_path', None)
                props.pop('_class', None)
                rec['props'] = props
            except Exception as e:
                rec['props_error'] = str(e)[:200]
        for pn in ('track_index', 'trigger_weight_threshold', 'notify_trigger_chance', 'notify_filter_type', 'notify_filter_lod',
                   'trigger_on_dedicated_server', 'trigger_on_follower', 'montage_tick_type'):
            try:
                rec[pn] = plain(ev.get_editor_property(pn))
            except Exception:
                pass
        if 'notify_name' in rec and 'trigger_time' in rec:
            t = track_of.get((rec['notify_name'], round(rec['trigger_time'], 5)))
            if t is not None:
                rec['track_name'] = t
        out.append(rec)
    return out


def curves(a, num_keys, fps, length):
    """float curves: values at every frame (key) time + raw keys; vector/transform curve names."""
    res = {'curves': {}, 'curve_keys': {}, 'curve_errors': {}}
    try:
        names = [str(c) for c in lib.get_animation_curve_names(a, unreal.RawCurveTrackTypes.RCT_FLOAT)]
    except Exception as e:
        res['curve_errors']['_names'] = str(e)[:300]
        names = []
    for n in names:
        try:
            times, values = lib.get_float_keys(a, n)
            res['curve_keys'][n] = {'t': [float(x) for x in times], 'v': [float(x) for x in values]}
        except Exception as e:
            res['curve_errors'][n + ':keys'] = str(e)[:200]
        try:
            vals = []
            for f in range(num_keys):
                t = min(length, f / fps) if fps else 0.0
                vals.append(float(lib.get_float_value_at_time(a, n, t)))
            res['curves'][n] = vals
        except Exception as e:
            res['curve_errors'][n + ':sample'] = str(e)[:200]
    for key, ct in (('vector_curves', 'RCT_VECTOR'), ('transform_curves', 'RCT_TRANSFORM')):
        try:
            lst = [str(c) for c in lib.get_animation_curve_names(a, getattr(unreal.RawCurveTrackTypes, ct))]
            if lst:
                res[key] = lst
        except Exception:
            pass
    if not res['curve_errors']:
        del res['curve_errors']
    return res


def timing(a):
    length = float(a.get_play_length())
    try:
        frames_ue = int(lib.get_num_frames(a))
    except Exception:
        frames_ue = None
    try:
        keys_ue = int(lib.get_num_keys(a))
    except Exception:
        keys_ue = None
    fps = None
    try:
        dt = float(lib.get_time_at_frame(a, 1))
        if dt > 0:
            fps = round(1.0 / dt, 6)
    except Exception:
        pass
    if not fps and frames_ue and length > 0:
        fps = round(frames_ue / length, 6)
    tfr = plain(prop(a, 'platform_target_frame_rate'))
    num_frames = frames_ue + 1 if frames_ue is not None else (int(round(length * (fps or 30))) + 1)
    return {'length_s': length, 'num_frames': num_frames, 'num_frames_ue': frames_ue, 'num_keys_ue': keys_ue, 'fps': fps,
            'fps_target': tfr, 'rate_scale': plain(prop(a, 'rate_scale'))}


def additive_info(seq):
    at = plain(prop(seq, 'additive_anim_type'))
    bt = plain(prop(seq, 'ref_pose_type'))
    rs = prop(seq, 'ref_pose_seq')
    idx = plain(prop(seq, 'ref_frame_index'))
    info = {'additive_anim_type': at, 'ref_pose_type': bt, 'ref_pose_seq': obj_path(rs) if rs else None,
            'ref_pose_seq_name': rs.get_name() if rs else None, 'ref_frame_index': idx}
    if at in (None, 'AAT_NONE'):
        info['base'] = None
        info['note'] = 'not additive in UE (ref_pose_* fields, if set, are ignored by UE)'
    elif bt == 'ABPT_REF_POSE':
        info['base'] = {'kind': 'skeleton_ref_pose'}
    elif bt == 'ABPT_ANIM_SCALED':
        info['base'] = {'kind': 'anim_scaled', 'seq': info['ref_pose_seq_name']}
    elif bt == 'ABPT_ANIM_FRAME':
        info['base'] = {'kind': 'anim_frame', 'seq': info['ref_pose_seq_name'], 'frame': idx}
    elif bt == 'ABPT_LOCAL_ANIM_FRAME':
        info['base'] = {'kind': 'local_anim_frame', 'seq': seq.get_name(), 'frame': idx}
    else:
        info['base'] = {'kind': str(bt)}
    return info


def export_fbx(seq, fbx_path):
    opts = unreal.FbxExportOption()
    for p, val in (('export_morph_targets', False), ('export_preview_mesh', False), ('export_local_time', True),
                   ('collision', False), ('level_of_detail', False), ('vertex_color', False)):
        try:
            opts.set_editor_property(p, val)
        except Exception:
            pass
    task = unreal.AssetExportTask()
    task.set_editor_property('object', seq)
    task.set_editor_property('filename', fbx_path)
    task.set_editor_property('automated', True)
    task.set_editor_property('prompt', False)
    task.set_editor_property('replace_identical', True)
    task.set_editor_property('options', opts)
    ok = bool(unreal.Exporter.run_asset_export_task(task))
    errs = []
    try:
        errs = [str(x) for x in task.get_editor_property('errors')]
    except Exception:
        pass
    return ok, errs


BS_USAGE = {}       # anim object path -> [{blendspace, x, y, rate_scale}]
BS_SUMMARY = {}


def read_blendspace(a):
    samples = []
    for s in list(prop(a, 'sample_data') or []):
        anim = prop(s, 'animation')
        v = prop(s, 'sample_value')
        samples.append({'anim': obj_path(anim) if anim else None, 'anim_name': anim.get_name() if anim else None,
                        'x': float(v.x) if v is not None else None, 'y': float(v.y) if v is not None else None,
                        'rate_scale': plain(prop(s, 'rate_scale'))})
    axes = []
    for i, bp in enumerate(list(prop(a, 'blend_parameters') or [])):
        axes.append({'axis': i, 'name': plain(prop(bp, 'display_name')), 'min': plain(prop(bp, 'min')),
                     'max': plain(prop(bp, 'max')), 'grid_num': plain(prop(bp, 'grid_num')),
                     'snap_to_grid': plain(prop(bp, 'snap_to_grid')), 'wrap_input': plain(prop(bp, 'wrap_input'))})
    return axes, samples


def prepass_blendspaces():
    for grp, ad in SELECTED:
        if not ad_class(ad).startswith(('BlendSpace', 'AimOffsetBlendSpace')):
            continue
        try:
            a = unreal.load_asset(ad_path(ad))
            axes, samples = read_blendspace(a)
        except Exception as e:
            progress('blend space prepass failed %s: %s' % (ad_path(ad), e))
            continue
        BS_SUMMARY[str(ad.asset_name)] = {'path': ad_path(ad), 'axes': axes, 'samples': samples}
        for s in samples:
            if s['anim']:
                BS_USAGE.setdefault(s['anim'], []).append({'blendspace': str(ad.asset_name), 'x': s['x'], 'y': s['y'],
                                                           'rate_scale': s['rate_scale']})


def overlay_data():
    """DA / ABP of every overlay folder (exported, data only, unused) and the animation packages they reference."""
    out = {}
    folders = [(x, 'export') for x in OVERLAY_EXPORT] + [(x, 'data_only') for x in OVERLAY_DATA_ONLY] + \
              [(x, 'unused') for x in OVERLAY_UNUSED]
    for folder, kind in folders:
        base = OVERLAY + folder
        rec = {'kind': kind, 'assets': {}}
        for ad in ALL:
            if str(ad.package_path) != base or ad_class(ad) in ANIM_CLASSES:
                continue
            rec['assets'][ad_path(ad)] = {'class': ad_class(ad), 'anim_dependencies': anim_dependencies(ad.package_name)}
        rec['animations_in_folder'] = sorted(ad_path(ad) for ad in ALL
                                             if str(ad.package_path) == base and ad_class(ad) in ANIM_CLASSES)
        out['Overlay/' + folder] = rec
    logic = {}
    for pkg in LOGIC_ASSETS:
        logic[pkg] = anim_dependencies(pkg)
    return out, logic


# ---------------------------------------------------------------- main loop
def process(grp, ad, stats):
    path, cls, name = ad_path(ad), ad_class(ad), str(ad.asset_name)
    fname = file_name(ad, grp)
    meta_path = os.path.join(META_DIR, fname + '.json')
    fbx_path = os.path.join(FBX_DIR, fname + '.fbx') if cls == SEQ_CLASS else None
    if SKIP_EXISTING and os.path.exists(meta_path) and (fbx_path is None or os.path.exists(fbx_path)):
        old = load_json(meta_path, 'existing meta') or {}
        if old.get('export_ok'):
            stats['skipped'] += 1
            return old.get('_manifest_row') or {'name': name, 'path': path, 'group': grp, 'class': cls, 'export_ok': True,
                                                'skipped_existing': True}
    a = unreal.load_asset(path)
    if a is None:
        raise RuntimeError('load_asset returned None')
    meta = {'name': name, 'file_name': fname, 'path': path, 'package': str(ad.package_name), 'group': grp, 'class': cls,
            'selection_kind': group_of(ad.package_path)[1],
            'skeleton': obj_path(prop(a, 'skeleton')) if prop(a, 'skeleton') else None}
    errors = []
    if cls == SEQ_CLASS or cls in ('AnimMontage', 'AnimComposite'):
        meta.update(timing(a))
    else:
        try:
            meta['length_s'] = float(a.get_play_length())
        except Exception:
            pass
    ok = True
    if cls == SEQ_CLASS:
        ok, errs = export_fbx(a, fbx_path)
        errors += errs
        exists = os.path.exists(fbx_path) and os.path.getsize(fbx_path) > 0
        if not exists:
            ok = False
            errors.append('FBX file missing or empty after export')
        meta['fbx'] = fbx_path if exists else None
        meta['fbx_file'] = 'fbx/' + fname + '.fbx' if exists else None
        meta['fbx_bytes'] = os.path.getsize(fbx_path) if exists else None
        meta['fbx_md5'] = md5_file(fbx_path) if exists else None
        meta['fbx_content'] = ('full source pose from the data-model bone tracks (UE FbxAnimationExport '
                               'EvaluateBoneTrackTransform), also for additive assets; deltas are computed by the baker')
        meta['enable_root_motion'] = plain(prop(a, 'enable_root_motion'))
        meta['force_root_lock'] = plain(prop(a, 'force_root_lock'))
        meta['root_motion_root_lock'] = plain(prop(a, 'root_motion_root_lock'))
        meta['use_normalized_root_motion_scale'] = plain(prop(a, 'use_normalized_root_motion_scale'))
        try:
            meta['root_motion_enabled_api'] = bool(lib.is_root_motion_enabled(a))
        except Exception:
            pass
        try:
            meta['root_track_start'] = tr(lib.extract_root_track_transform(a, 0.0))
            meta['root_track_end'] = tr(lib.extract_root_track_transform(a, float(meta['length_s'])))
        except Exception as e:
            meta['root_track_error'] = str(e)[:200]
        meta['interpolation'] = plain(prop(a, 'interpolation'))
        meta['loop'] = plain(prop(a, 'loop'))
        meta['additive'] = additive_info(a)
        try:
            meta['bone_tracks'] = [str(x) for x in lib.get_animation_track_names(a)]
        except Exception as e:
            meta['bone_tracks_error'] = str(e)[:200]
        try:
            meta['sync_markers'] = [plain(m) for m in lib.get_animation_sync_markers(a)]
        except Exception as e:
            meta['sync_markers_error'] = str(e)[:200]
    if cls == SEQ_CLASS or cls in ('AnimMontage', 'AnimComposite'):
        meta.update(curves(a, meta['num_frames'], meta.get('fps') or 30.0, meta['length_s']))
        try:
            meta['notify_tracks'] = [str(x) for x in lib.get_animation_notify_track_names(a)]
        except Exception:
            pass
        meta['notifies'] = notify_events(a)
    if cls in ('AnimMontage', 'AnimComposite'):
        try:
            meta['slot_names'] = [str(x) for x in lib.get_montage_slot_names(a)]
        except Exception:
            pass
        for pn in ('composite_sections', 'slot_anim_tracks', 'animation_track', 'blend_in', 'blend_out',
                   'blend_out_trigger_time', 'enable_auto_blend_out', 'sync_group', 'sync_slot_index'):
            v = prop(a, pn)
            if v is not None:
                meta[pn] = plain(v, 0, a)
        refs = set()
        for st in meta.get('slot_anim_tracks') or []:
            for seg in ((st or {}).get('anim_track') or {}).get('anim_segments') or []:
                if seg.get('anim_reference'):
                    refs.add(short(seg['anim_reference']))
        meta['referenced_sequences'] = sorted(refs)
    if cls.startswith(('BlendSpace', 'AimOffsetBlendSpace')):
        axes, samples = read_blendspace(a)
        meta['axes'] = axes
        meta['samples'] = samples
        for pn in ('additive_anim_type', 'interpolation_param', 'per_bone_blend_mode', 'manual_per_bone_overrides',
                   'target_weight_interpolation_speed_per_sec', 'notify_trigger_mode', 'loop', 'allow_mesh_space_blending',
                   'should_match_sync_phases', 'interpolate_using_grid'):
            v = prop(a, pn)
            if v is not None:
                meta[pn] = plain(v, 0, a)
    if cls == 'PoseAsset':
        for key, fn in (('pose_names', 'get_pose_names'), ('base_pose_name', 'get_base_pose_name')):
            try:
                meta[key] = plain(getattr(a, fn)())
            except Exception:
                pass
    meta['databases'] = ANIM_TO_DBS.get(path, [])
    meta['used_by_blendspaces'] = BS_USAGE.get(path, [])
    if grp == 'UEFN/AimOffset' and cls == SEQ_CLASS:
        meta['used_by_bs_neutral_ao_stand'] = any(u['blendspace'] == AO_STAND_BS for u in meta['used_by_blendspaces'])
    meta['referenced_by'] = referencers(ad.package_name)
    try:
        meta['source_import'] = source_import(ad.package_name)
    except Exception as e:
        meta['source_import'] = {'_error': str(e)[:200]}
    if cls == SEQ_CLASS:
        meta['gasp_export'] = gasp_export_compare(name, meta.get('length_s'), meta.get('num_frames'))
    meta['export_ok'] = bool(ok)
    meta['export_errors'] = errors
    row = {'name': name, 'file_name': fname, 'path': path, 'group': grp, 'class': cls,
           'selection_kind': meta['selection_kind'], 'skeleton': short(meta['skeleton']), 'export_ok': bool(ok),
           'fbx': meta.get('fbx_file'), 'fbx_md5': meta.get('fbx_md5'), 'fbx_bytes': meta.get('fbx_bytes'),
           'length_s': meta.get('length_s'), 'num_frames': meta.get('num_frames'), 'fps': meta.get('fps'),
           'enable_root_motion': meta.get('enable_root_motion'),
           'additive_anim_type': (meta.get('additive') or {}).get('additive_anim_type'),
           'additive_base': (meta.get('additive') or {}).get('base'),
           'curves': sorted((meta.get('curves') or {}).keys()), 'notify_count': len(meta.get('notifies') or []),
           'sync_marker_count': len(meta.get('sync_markers') or []), 'databases': meta['databases'],
           'used_by_blendspaces': [u['blendspace'] for u in meta['used_by_blendspaces']],
           'source_fbx_md5': ((meta.get('source_import') or {}).get('md5') or [None])[0]}
    if 'used_by_bs_neutral_ao_stand' in meta:
        row['used_by_bs_neutral_ao_stand'] = meta['used_by_bs_neutral_ao_stand']
    if 'gasp_export' in meta:
        g = meta['gasp_export']
        row['in_gasp_export'] = g.get('exists')
        row['gasp_export_identical_length_frames'] = g.get('identical_length_frames') if g.get('exists') else None
    if errors:
        row['errors'] = errors
    meta['_manifest_row'] = row
    with open(meta_path, 'w', encoding='utf-8') as f:
        json.dump(meta, f, indent=1, default=str)
    if not ok:
        stats['failed'] += 1
        progress('EXPORT FAILED %s: %s' % (path, errors))
    return row


def write_manifest(rows, failures, stats, final, overlays, logic):
    groups = {}
    for r in rows:
        g = groups.setdefault(r['group'], {'sequences': 0, 'sequences_ok': 0, 'meta_only': 0, 'meta_only_ok': 0,
                                           'length_s': 0.0, 'frames': 0, 'fbx_bytes': 0})
        if r['class'] == SEQ_CLASS:
            g['sequences'] += 1
            g['sequences_ok'] += 1 if r.get('export_ok') else 0
            g['length_s'] += float(r.get('length_s') or 0)
            g['frames'] += int(r.get('num_frames') or 0)
            g['fbx_bytes'] += int(r.get('fbx_bytes') or 0)
        else:
            g['meta_only'] += 1
            g['meta_only_ok'] += 1 if r.get('export_ok') else 0
    for f in failures:
        groups.setdefault(f['group'], {'sequences': 0, 'sequences_ok': 0, 'meta_only': 0, 'meta_only_ok': 0,
                                       'length_s': 0.0, 'frames': 0, 'fbx_bytes': 0}).setdefault('exceptions', 0)
        groups[f['group']]['exceptions'] += 1
    seqs = [r for r in rows if r['class'] == SEQ_CLASS]
    in_ge = [r for r in seqs if r.get('in_gasp_export')]
    ao = BS_SUMMARY.get(AO_STAND_BS, {})
    man = {
        'complete': final,
        'engine': unreal.SystemLibrary.get_engine_version(),
        'project': os.path.join(UE_PROJECT_DIR, 'GASPALS.uproject'),
        'script': 'Source/ue/sourcepack_export.py',
        'output': OUT,
        'filters': {'only': ONLY.pattern if ONLY else None, 'limit': LIMIT, 'skip_existing': SKIP_EXISTING},
        'fbx_settings': {'exporter': 'unreal.Exporter.run_asset_export_task (AnimSequenceExporterFBX)',
                         'export_morph_targets': False, 'export_preview_mesh': False, 'export_local_time': True,
                         'collision': False, 'level_of_detail': False, 'vertex_color': False,
                         'same_as': 'Source/ue/export_sourcepack.py'},
        'conventions': {
            'num_frames': 'number of sampled keys = UE GetNumFrames + 1 (same as Animations/sourcepack/SourcePack_export meta)',
            'curves': 'float curve value at t = frame / fps for every key (data-model curve, GetFloatValueAtTime); raw keys in curve_keys',
            'notifies': 'notify_name, trigger_time, duration, end_time, class, kind, track_name, props (same key names as SourcePack_export meta)',
            'additive': 'FBX holds full poses; additive.base = base for the delta (anim_frame: ref_pose_seq@ref_frame_index, '
                        'local_anim_frame: the clip itself@ref_frame_index, skeleton_ref_pose)',
            'gasp_export': 'same name in Animations/sourcepack/SourcePack_export; identical_length_frames = |length diff| < 1e-4 s and same num_frames',
            'source_import': 'AssetImportData strings read from the .uasset bytes (FileMD5 = MD5 of the source FBX at import)'},
        'selection': {'uefn_animations': UEFN_ANIMS, 'overlay_export': OVERLAY_EXPORT, 'overlay_data_only': OVERLAY_DATA_ONLY,
                      'overlay_unused_exported_as_data': OVERLAY_UNUSED, 'names_file': NAMES_FILE,
                      'note': 'overlay_unused_exported_as_data: exported like the rest but unused at runtime'},
        'stats': {'selected': len(SELECTED), 'processed': len(rows), 'exceptions': len(failures),
                  'export_failed': stats['failed'], 'skipped_existing': stats['skipped'],
                  'sequences': len(seqs), 'sequences_ok': sum(1 for r in seqs if r.get('export_ok')),
                  'meta_only_assets': len(rows) - len(seqs), 'sequence_length_s': round(sum(float(r.get('length_s') or 0) for r in seqs), 3),
                  'fbx_bytes': sum(int(r.get('fbx_bytes') or 0) for r in seqs),
                  'in_gasp_export_by_name': len(in_ge),
                  'gasp_export_identical_length_frames': sum(1 for r in in_ge if r.get('gasp_export_identical_length_frames')),
                  'gasp_export_different_length_or_frames': sorted(r['name'] for r in in_ge if not r.get('gasp_export_identical_length_frames')),
                  'elapsed_s': round(time.time() - T0, 1)},
        'groups': dict(sorted(groups.items())),
        'not_exported_anim_assets': NOT_EXPORTED,
        'duplicate_names': DUPLICATE_NAMES,
        'aimoffset': {'blendspace': ao.get('path'), 'axes': ao.get('axes'),
                      'samples_used_by_bs_neutral_ao_stand': [s['anim_name'] for s in ao.get('samples', [])],
                      'note': 'all 42 AimOffset sequences are exported; used_by_bs_neutral_ao_stand marks the BS samples'},
        'blendspaces': BS_SUMMARY,
        'overlays': overlays,
        'logic_asset_anim_dependencies': logic,
        'failures': failures,
        'sequences': rows,
    }
    tmp = os.path.join(OUT, 'manifest.json.tmp')
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(man, f, indent=1, default=str)
    os.replace(tmp, os.path.join(OUT, 'manifest.json'))


def main():
    prepass_blendspaces()
    progress('blend spaces: %s; AO samples=%d' % (sorted(BS_SUMMARY), len(BS_SUMMARY.get(AO_STAND_BS, {}).get('samples', []))))
    overlays, logic = overlay_data()
    progress('overlay folders: %d' % len(overlays))
    rows, failures, stats = [], [], {'failed': 0, 'skipped': 0}
    for i, (grp, ad) in enumerate(SELECTED):
        try:
            rows.append(process(grp, ad, stats))
        except Exception:
            failures.append({'path': ad_path(ad), 'group': grp, 'class': ad_class(ad), 'error': traceback.format_exc()[-3000:]})
            progress('FAIL %s: %s' % (ad_path(ad), traceback.format_exc().strip().splitlines()[-1][:300]))
        if i % 25 == 0 or i == len(SELECTED) - 1:
            progress('%d/%d %s' % (i + 1, len(SELECTED), ad_path(ad)))
        if i % 50 == 49:
            unreal.SystemLibrary.collect_garbage()
        if i % 200 == 199:
            write_manifest(rows, failures, stats, False, overlays, logic)
    write_manifest(rows, failures, stats, True, overlays, logic)
    progress('DONE processed=%d sequences_ok=%d export_failed=%d exceptions=%d skipped=%d' % (
        len(rows), sum(1 for r in rows if r['class'] == SEQ_CLASS and r.get('export_ok')), stats['failed'], len(failures),
        stats['skipped']))


main()
