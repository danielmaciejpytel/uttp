"""READ-ONLY extractor for the source pack project (run inside UnrealEditor-Cmd, -run=pythonscript).

Never saves, resaves or modifies any package: it only loads assets, reads properties through the
Python API and writes text files OUTSIDE the project (default <project folder>\\Extracted\\sourcepack\\sourcepack).

Raw UE-side output goes to <OUT>\\ue_raw\\*.json and <OUT>\\t3d\\*.t3d; the offline post-processor
Source\\tools\\uasset\\sourcepack_post.py (system python) merges them into the final top-level JSON files.

Env:
  UTPP_SOURCEPACK_EXTRACT_OUT     output folder (default <project folder>\\Extracted\\sourcepack\\sourcepack)
  UTPP_SOURCEPACK_PHASES  comma list of phases (default all): assets,anims,posesearch,choosers,skeletons,blueprints,t3d,classt3d
  UTPP_SOURCEPACK_LIMIT   per-phase item limit for probe runs (0 = no limit)
  UTPP_SOURCEPACK_SKIP_EXISTING  1 = do not re-export T3D files that already exist
  UTPP_SOURCEPACK_ONLY    optional regex: only assets whose object path matches (probe runs)
Notes (UE 5.8 Python API):
  - PoseSearchDatabase entries are not exposed as properties: list via get_num_animation_assets()/get_animation_asset(i),
    per-entry flags (sampling range, mirroring, enabled, ...) come from the T3D export.
  - ChooserTable columns/results are FInstancedStruct arrays without Python access: T3D export.
  - Blueprint variables/graphs/nodes are not enumerable through Python: T3D export of the Blueprint (graphs, nodes, pins)
    and of the generated-class CDO (variable defaults).
"""
import json
import os
import re
import time
import traceback

import unreal


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


OUT = os.environ.get('UTPP_SOURCEPACK_EXTRACT_OUT', os.path.join(_project_root(), 'Extracted', 'sourcepack', 'sourcepack'))
RAW = os.path.join(OUT, 'ue_raw')
T3D = os.path.join(OUT, 't3d')
PHASES = [p.strip() for p in os.environ.get(
    'UTPP_SOURCEPACK_PHASES', 'assets,anims,posesearch,choosers,skeletons,blueprints,t3d,classt3d').split(',') if p.strip()]
LIMIT = int(os.environ.get('UTPP_SOURCEPACK_LIMIT', '0') or 0)
SKIP_EXISTING = os.environ.get('UTPP_SOURCEPACK_SKIP_EXISTING', '0') == '1'
ONLY = re.compile(os.environ['UTPP_SOURCEPACK_ONLY']) if os.environ.get('UTPP_SOURCEPACK_ONLY') else None  # path regex (probe runs)
ROOTS = ['/GASPALS', '/Game']
UE_PROJECT_DIR = os.environ.get('UTPP_SOURCEPACK_PROJECT') or os.path.join(os.path.dirname(_project_root()), 'Unreal Engine', 'Test', 'ALSV5', 'GASP-ALS')   # folder of GASPALS.uproject
CONTENT_DIRS = {
    '/GASPALS/': os.path.join(UE_PROJECT_DIR, 'Plugins', 'GASPALS', 'Content'),
    '/Game/': os.path.join(UE_PROJECT_DIR, 'Content'),
}
os.makedirs(RAW, exist_ok=True)
os.makedirs(T3D, exist_ok=True)
PROGRESS = os.path.join(OUT, '_progress.log')
FAILURES = []
T0 = time.time()

ANIM_CLASSES = {'AnimSequence', 'AnimMontage', 'AnimComposite', 'BlendSpace', 'BlendSpace1D', 'AimOffsetBlendSpace',
                'AimOffsetBlendSpace1D', 'PoseAsset', 'AnimStreamable'}
PS_CLASSES = {'PoseSearchDatabase', 'PoseSearchSchema', 'PoseSearchNormalizationSet'}
CHOOSER_CLASSES = {'ChooserTable', 'ProxyTable', 'ProxyAsset'}
BP_CLASSES = {'Blueprint', 'AnimBlueprint', 'WidgetBlueprint', 'EditorUtilityBlueprint', 'EditorUtilityWidgetBlueprint',
              'ControlRigBlueprint', 'AnimLayerInterface'}
DATA_CLASSES = {'DataTable', 'CompositeDataTable', 'CurveTable', 'UserDefinedEnum', 'UserDefinedStruct', 'MirrorDataTable',
                'PrimaryDataAsset', 'DataAsset', 'CurveFloat', 'CurveVector', 'CurveLinearColor', 'InputAction',
                'InputMappingContext', 'CameraRigAsset', 'CameraAsset', 'CameraDirector', 'CameraShakeAsset',
                'PhysicalMaterial'}
RIG_CLASSES = {'Skeleton', 'IKRigDefinition', 'IKRetargeter', 'BlendProfile', 'PhysicsAsset'}
# T3D is skipped for heavy MetaHuman-only content that is irrelevant for the UTPP mod (face boards, RBF correctives).
T3D_SKIP_PREFIXES = ('/GASPALS/MetaHumans/',)
T3D_SKIP_CLASSES_UNDER_PREFIX = {'ControlRigBlueprint', 'PoseAsset', 'GroomAsset', 'GroomBindingAsset'}


def progress(msg):
    line = '[%7.1fs] %s' % (time.time() - T0, msg)
    unreal.log('[UTPP-SOURCEPACK] ' + line)
    with open(PROGRESS, 'a', encoding='utf-8') as f:
        f.write(line + '\n')


def fail(phase, asset, err):
    FAILURES.append({'phase': phase, 'asset': asset, 'error': str(err)[:2000]})
    progress('FAIL %s %s: %s' % (phase, asset, str(err)[:300]))


def write_json(name, data):
    path = os.path.join(RAW, name)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=1, default=str)
    progress('wrote %s (%d bytes)' % (path, os.path.getsize(path)))


# ---------------------------------------------------------------- generic reflection dump
_PROP_RE = re.compile(r'^\s*-\s+``(\w+)``\s+\((.*?)\):\s+\[(Read-Write|Read-Only)\]', re.M)
_prop_cache = {}


def prop_names(t):
    if t in _prop_cache:
        return _prop_cache[t]
    names, seen = [], set()
    for c in getattr(t, '__mro__', [t]):
        doc = getattr(c, '__doc__', None) or ''
        for m in _PROP_RE.finditer(doc):
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


SKIP_PROPS = {'thumbnail_info'}
SEQ_TYPES = tuple([list, tuple] + [getattr(unreal, n) for n in ('Array', 'FixedArray', 'Set') if hasattr(unreal, n)])
MAP_TYPE = getattr(unreal, 'Map', None)


def plain(v, depth=0, owner=None, max_items=4000):
    if v is None or isinstance(v, (bool, int, float, str)):
        return v
    if isinstance(v, unreal.EnumBase):
        return getattr(v, 'name', str(v))
    if isinstance(v, (unreal.Name, unreal.Text)):
        return str(v)
    if isinstance(v, SEQ_TYPES):
        items = list(v)
        out = [plain(x, depth + 1, owner, max_items) for x in items[:max_items]]
        if len(items) > max_items:
            out.append('...(%d more)' % (len(items) - max_items))
        return out
    if MAP_TYPE is not None and isinstance(v, MAP_TYPE):
        return {str(plain(k, depth + 1, owner)): plain(x, depth + 1, owner, max_items) for k, x in v.items()}
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
        names = prop_names(type(v))
        if not names:
            names = [a for a in dir(v) if not a.startswith('_') and not callable(getattr(v, a, None))]
        for n in names:
            try:
                d[n] = plain(v.get_editor_property(n), depth + 1, owner, max_items)
            except Exception:
                try:
                    d[n] = plain(getattr(v, n), depth + 1, owner, max_items)
                except Exception:
                    pass
        return d if d else str(v)
    return str(v)


def dump_object(o, depth=0, owner=None, exclude=()):
    d = {'_class': o.get_class().get_name(), '_path': obj_path(o)}
    own = owner if owner is not None else o
    for n in prop_names(type(o)):
        if n in exclude or n in SKIP_PROPS:
            continue
        try:
            d[n] = plain(o.get_editor_property(n), depth, own)
        except Exception as e:
            d.setdefault('_unreadable', []).append(n)
    return d


def safe(fn, *a):
    try:
        return fn(*a)
    except Exception as e:
        return {'_error': str(e)[:300]}


def first_str(res):
    if isinstance(res, str):
        return res
    if isinstance(res, (tuple, list)):
        for x in res:
            if isinstance(x, str):
                return x
    return None


# ---------------------------------------------------------------- asset registry
registry = unreal.AssetRegistryHelpers.get_asset_registry()
try:
    registry.scan_paths_synchronous(ROOTS, True)
except Exception as e:
    progress('scan_paths_synchronous: %s' % e)

ALL = []
for root in ROOTS:
    try:
        ALL.extend(registry.get_assets_by_path(root, recursive=True))
    except Exception as e:
        progress('get_assets_by_path %s: %s' % (root, e))
if ONLY is not None:
    ALL = [ad for ad in ALL if ONLY.search('%s.%s' % (ad.package_name, ad.asset_name))]
progress('registry: %d assets under %s (filter=%s)' % (len(ALL), ROOTS, ONLY.pattern if ONLY else None))


def ad_class(ad):
    try:
        return str(ad.asset_class_path.asset_name)
    except Exception:
        return str(ad.asset_class)


def ad_path(ad):
    return '%s.%s' % (ad.package_name, ad.asset_name)


def disk_file(pkg):
    pkg = str(pkg)
    for mount, folder in CONTENT_DIRS.items():
        if pkg.startswith(mount):
            base = os.path.join(folder, pkg[len(mount):].replace('/', os.sep))
            for ext in ('.uasset', '.umap'):
                if os.path.exists(base + ext):
                    return base + ext
    return None


BY_CLASS = {}
for ad in ALL:
    BY_CLASS.setdefault(ad_class(ad), []).append(ad)


def limited(lst):
    return lst[:LIMIT] if LIMIT else lst


def load(path, phase):
    try:
        a = unreal.load_asset(path)
        if a is None:
            fail(phase, path, 'load_asset returned None')
        return a
    except Exception as e:
        fail(phase, path, e)
        return None


def phase_assets():
    opts = unreal.AssetRegistryDependencyOptions()
    for k, v in (('include_soft_package_references', True), ('include_hard_package_references', True),
                 ('include_searchable_names', False), ('include_soft_management_references', False),
                 ('include_hard_management_references', False)):
        try:
            opts.set_editor_property(k, v)
        except Exception:
            pass
    rows = []
    for ad in ALL:
        pkg = str(ad.package_name)
        f = disk_file(pkg)
        rec = {'path': ad_path(ad), 'package': pkg, 'name': str(ad.asset_name), 'class': ad_class(ad),
               'class_path': str(getattr(ad.asset_class_path, 'package_name', '')) + '.' + ad_class(ad),
               'folder': str(ad.package_path), 'file': f, 'size_bytes': os.path.getsize(f) if f else None}
        try:
            deps = registry.get_dependencies(ad.package_name, opts)
            rec['dependencies'] = sorted({str(x) for x in (deps or [])})
        except Exception as e:
            rec['dependencies_error'] = str(e)[:200]
        rows.append(rec)
    rows.sort(key=lambda r: r['path'])
    classes = {}
    for r in rows:
        classes[r['class']] = classes.get(r['class'], 0) + 1
    write_json('assets.json', {'roots': ROOTS, 'count': len(rows), 'classes': dict(sorted(classes.items())),
                               'engine': unreal.SystemLibrary.get_engine_version(), 'assets': rows})


# ---------------------------------------------------------------- animations
lib = unreal.AnimationLibrary


def notify_events(seq):
    out = []
    track_of = {}
    try:
        for tn in lib.get_animation_notify_track_names(seq):
            try:
                for ev in lib.get_animation_notify_events_for_track(seq, tn):
                    key = (str(ev.get_editor_property('notify_name')), round(float(lib.get_anim_notify_event_trigger_time(ev)), 5))
                    track_of.setdefault(key, str(tn))
            except Exception:
                pass
    except Exception:
        pass
    try:
        events = lib.get_animation_notify_events(seq)
    except Exception as e:
        return [{'_error': str(e)[:300]}]
    for ev in events:
        rec = {}
        try:
            rec['name'] = str(ev.get_editor_property('notify_name'))
        except Exception:
            pass
        for key, fn in (('time', 'get_anim_notify_event_trigger_time'), ('duration', 'get_anim_notify_event_duration')):
            try:
                rec[key] = float(getattr(lib, fn)(ev))
            except Exception as e:
                rec[key + '_error'] = str(e)[:200]
        obj = None
        for pn in ('notify_state_class', 'notify'):
            try:
                obj = ev.get_editor_property(pn)
            except Exception:
                obj = None
            if obj:
                rec['kind'] = 'state' if pn == 'notify_state_class' else 'notify'
                break
        if obj:
            rec['class'] = obj.get_class().get_name()
            rec['class_path'] = obj.get_class().get_path_name()
            props = dump_object(obj)
            props.pop('_path', None)
            props.pop('_class', None)
            rec['props'] = props
        for pn in ('trigger_weight_threshold', 'notify_trigger_chance', 'notify_filter_type', 'notify_filter_lod',
                   'trigger_on_dedicated_server', 'trigger_on_follower', 'montage_tick_type', 'track_index'):
            try:
                rec[pn] = plain(ev.get_editor_property(pn))
            except Exception:
                pass
        if 'name' in rec and 'time' in rec:
            t = track_of.get((rec['name'], round(rec['time'], 5)))
            if t is not None:
                rec['track'] = t
        out.append(rec)
    return out


ANIM_EXCLUDE = {'asset_import_data', 'thumbnail_info', 'raw_curve_data', 'compressed_data', 'controller', 'data_model',
                'data_model_interface'}


def anim_record(a, cls, curves_out):
    path = obj_path(a)
    rec = {'path': path, 'name': a.get_name(), 'class': cls}
    try:
        rec['skeleton'] = obj_path(a.get_editor_property('skeleton')) if a.get_editor_property('skeleton') else None
    except Exception:
        try:
            rec['skeleton'] = obj_path(a.get_skeleton())
        except Exception:
            pass
    for key, fn in (('length_s', lambda: a.get_play_length()),):
        try:
            rec[key] = float(fn())
        except Exception:
            pass
    if isinstance(a, unreal.AnimSequenceBase):
        try:
            rec['num_frames'] = int(lib.get_num_frames(a))
        except Exception:
            pass
        try:
            rec['rate_scale'] = float(lib.get_rate_scale(a))
        except Exception:
            pass
        try:
            rec['notify_tracks'] = [str(x) for x in lib.get_animation_notify_track_names(a)]
        except Exception:
            pass
        rec['notifies'] = notify_events(a)
        try:
            names = [str(c) for c in lib.get_animation_curve_names(a, unreal.RawCurveTrackTypes.RCT_FLOAT)]
        except Exception as e:
            names = []
            rec['curves_error'] = str(e)[:200]
        rec['curves'] = names
        ck = {}
        for n in names:
            try:
                times, values = lib.get_float_keys(a, n)
                ck[n] = {'t': [round(float(x), 5) for x in times], 'v': [round(float(x), 6) for x in values]}
            except Exception as e:
                ck[n] = {'_error': str(e)[:200]}
        if ck:
            curves_out[path] = ck
        try:
            vnames = [str(c) for c in lib.get_animation_curve_names(a, unreal.RawCurveTrackTypes.RCT_VECTOR)]
            tnames = [str(c) for c in lib.get_animation_curve_names(a, unreal.RawCurveTrackTypes.RCT_TRANSFORM)]
            if vnames:
                rec['vector_curves'] = vnames
            if tnames:
                rec['transform_curves'] = tnames
        except Exception:
            pass
    if isinstance(a, unreal.AnimSequence):
        try:
            rec['sync_markers'] = [plain(m) for m in lib.get_animation_sync_markers(a)]
        except Exception as e:
            rec['sync_markers_error'] = str(e)[:200]
        try:
            rec['root_motion_enabled'] = bool(lib.is_root_motion_enabled(a))
        except Exception:
            pass
        try:
            rec['additive_type'] = plain(lib.get_additive_animation_type(a))
            rec['additive_base_pose_type'] = plain(lib.get_additive_base_pose_type(a))
        except Exception:
            pass
        try:
            tr = lib.get_animation_track_names(a)
            rec['bone_track_count'] = len(tr)
        except Exception:
            pass
        try:
            rt0 = lib.extract_root_track_transform(a, 0.0)
            rt1 = lib.extract_root_track_transform(a, float(rec.get('length_s', 0.0)))
            rec['root_track_start'] = plain(rt0)
            rec['root_track_end'] = plain(rt1)
        except Exception:
            pass
    if isinstance(a, unreal.AnimMontage):
        try:
            rec['slot_names'] = [str(x) for x in lib.get_montage_slot_names(a)]
        except Exception:
            pass
        sections = []
        try:
            for i in range(a.get_num_sections()):
                sections.append({'index': i, 'name': str(a.get_section_name(i))})
        except Exception:
            pass
        rec['sections_api'] = sections
        for pn in ('composite_sections', 'slot_anim_tracks'):
            try:
                rec[pn] = plain(a.get_editor_property(pn), 0, a)
            except Exception:
                pass
    if isinstance(a, unreal.PoseAsset):
        for key, fn in (('pose_names', 'get_pose_names'), ('base_pose_name', 'get_base_pose_name')):
            try:
                v = getattr(a, fn)()
                rec[key] = plain(v)
            except Exception:
                pass
    rec['props'] = dump_object(a, exclude=ANIM_EXCLUDE)
    try:
        ld = rec['props']
        if 'num_frames' in rec and rec.get('length_s'):
            rec['frame_rate_est'] = round(rec['num_frames'] / rec['length_s'], 4)
        tfr = ld.get('platform_target_frame_rate')
        if isinstance(tfr, dict):
            rec['target_frame_rate'] = tfr
    except Exception:
        pass
    return rec


def gasp_export_names():
    names = {}
    base = os.path.join(_project_root(), 'Animations', 'sourcepack', 'SourcePack_export')
    try:
        inv = json.load(open(os.path.join(base, 'inventory.json'), encoding='utf-8'))
        for p in inv.get('sequences', {}):
            names.setdefault(p.rsplit('.', 1)[-1], {})['inventory'] = p
    except Exception as e:
        progress('GASP inventory not readable: %s' % e)
    try:
        man = json.load(open(os.path.join(base, 'manifest.json'), encoding='utf-8'))
        for s in man.get('sequences', []):
            names.setdefault(s['name'], {})['manifest'] = s['path']
            names[s['name']]['manifest_exported'] = s.get('exported')
    except Exception as e:
        progress('GASP manifest not readable: %s' % e)
    return names


def phase_anims():
    gasp = gasp_export_names()
    rows, curves = [], {}
    targets = [ad for ad in ALL if ad_class(ad) in ANIM_CLASSES]
    targets.sort(key=ad_path)
    targets = limited(targets)
    progress('anims: %d assets' % len(targets))
    for i, ad in enumerate(targets):
        p = ad_path(ad)
        a = load(p, 'anims')
        if a is None:
            continue
        try:
            rec = anim_record(a, ad_class(ad), curves)
        except Exception as e:
            fail('anims', p, traceback.format_exc())
            continue
        g = gasp.get(str(ad.asset_name))
        rec['in_gasp_export'] = bool(g)
        if g:
            rec['gasp_export'] = g
            gp = g.get('inventory') or g.get('manifest') or ''
            rec['gasp_same_relative_path'] = gp.replace('/Game/', '/GASPALS/', 1) == p
        rows.append(rec)
        if i % 100 == 0:
            progress('anims %d/%d %s' % (i, len(targets), p))
            unreal.SystemLibrary.collect_garbage()
    write_json('anims.json', {'count': len(rows), 'anims': rows})
    write_json('anim_curves.json', {'note': 'float curve keys per animation: t = key times (s), v = values', 'curves': curves})


# ---------------------------------------------------------------- pose search
def phase_posesearch():
    out = {'schemas': [], 'databases': [], 'normalization_sets': []}
    for cls, key in (('PoseSearchSchema', 'schemas'), ('PoseSearchNormalizationSet', 'normalization_sets'),
                     ('PoseSearchDatabase', 'databases')):
        for ad in limited(sorted(BY_CLASS.get(cls, []), key=ad_path)):
            p = ad_path(ad)
            o = load(p, 'posesearch')
            if o is None:
                continue
            try:
                rec = dump_object(o)
                if cls == 'PoseSearchDatabase':
                    ents = []
                    n = o.get_num_animation_assets()
                    for i in range(n):
                        a = o.get_animation_asset(i)
                        e = {'index': i, 'asset': obj_path(a) if a else None,
                             'asset_class': a.get_class().get_name() if a else None}
                        try:
                            e['play_length_s'] = float(a.get_play_length()) if a else None
                        except Exception:
                            pass
                        ents.append(e)
                    rec['entries_api'] = ents
                out[key].append(rec)
            except Exception:
                fail('posesearch', p, traceback.format_exc())
        progress('posesearch %s: %d' % (cls, len(out[key])))
    write_json('posesearch.json', out)


# ---------------------------------------------------------------- choosers
def phase_choosers():
    out = {'tables': []}
    for cls in sorted(CHOOSER_CLASSES):
        for ad in limited(sorted(BY_CLASS.get(cls, []), key=ad_path)):
            p = ad_path(ad)
            o = load(p, 'choosers')
            if o is None:
                continue
            try:
                rec = dump_object(o)
                for pn in ('columns_structs', 'results_structs', 'disabled_rows', 'nested_choosers', 'fallback_result',
                           'context_data', 'output_object_type', 'result_type', 'entries', 'type', 'inherit_entries_from'):
                    if pn not in rec:
                        try:
                            rec[pn] = plain(o.get_editor_property(pn), 0, o)
                        except Exception:
                            pass
                out['tables'].append(rec)
            except Exception:
                fail('choosers', p, traceback.format_exc())
    progress('choosers: %d' % len(out['tables']))
    write_json('choosers.json', out)


# ---------------------------------------------------------------- skeletons / rigs
def tr_list(t):
    try:
        l, r, s = t.translation, t.rotation, t.scale3d
        return [round(l.x, 5), round(l.y, 5), round(l.z, 5), round(r.x, 7), round(r.y, 7), round(r.z, 7), round(r.w, 7),
                round(s.x, 5), round(s.y, 5), round(s.z, 5)]
    except Exception as e:
        return str(e)[:100]


def phase_skeletons():
    out = {'transform_format': '[tx,ty,tz, qx,qy,qz,qw, sx,sy,sz] (UE units: cm, left-handed Z-up)', 'skeletons': [],
           'skeletal_meshes': [], 'rigs': []}
    # an anim per skeleton (needed for FindBonePathToRoot)
    anim_for_skel = {}
    for ad in BY_CLASS.get('AnimSequence', []):
        try:
            sk = None
            tv = ad.get_tag_value('Skeleton')
            if tv:
                sk = str(tv).split("'")[1] if "'" in str(tv) else str(tv)
            if sk and sk not in anim_for_skel:
                anim_for_skel[sk] = ad_path(ad)
        except Exception:
            pass
    progress('skeletons: anim candidates for %d skeletons' % len(anim_for_skel))
    space_local = unreal.AnimPoseSpaces.LOCAL
    space_world = unreal.AnimPoseSpaces.WORLD
    for ad in limited(sorted(BY_CLASS.get('Skeleton', []), key=ad_path)):
        p = ad_path(ad)
        sk = load(p, 'skeletons')
        if sk is None:
            continue
        rec = {'path': p}
        try:
            pose = unreal.AnimPoseExtensions.get_reference_pose(sk)
            bones = [str(b) for b in unreal.AnimPoseExtensions.get_bone_names(pose)]
            anim = unreal.load_asset(anim_for_skel[p]) if p in anim_for_skel else None
            rec['parent_source'] = ('AnimationLibrary.find_bone_path_to_root via ' + anim_for_skel[p]) if anim else None
            blist = []
            for b in bones:
                e = {'name': b}
                try:
                    e['ref_local'] = tr_list(unreal.AnimPoseExtensions.get_ref_bone_pose(pose, b, space_local))
                    e['ref_world'] = tr_list(unreal.AnimPoseExtensions.get_ref_bone_pose(pose, b, space_world))
                except Exception as ex:
                    e['ref_error'] = str(ex)[:200]
                if anim:
                    try:
                        chain = [str(x) for x in lib.find_bone_path_to_root(anim, b)]
                        # FindBonePathToRoot returns [bone, parent, ..., root]
                        e['parent'] = chain[1] if len(chain) > 1 and chain[0] == b else (chain[0] if chain and chain[0] != b else None)
                        e['depth'] = len(chain) - 1
                    except Exception as ex:
                        e['parent_error'] = str(ex)[:200]
                blist.append(e)
            rec['bones'] = blist
            rec['bone_count'] = len(blist)
            try:
                socks = []
                for s in unreal.AnimPoseExtensions.get_socket_names(pose):
                    socks.append({'name': str(s), 'local': tr_list(unreal.AnimPoseExtensions.get_socket_pose(pose, s, space_local)),
                                  'world': tr_list(unreal.AnimPoseExtensions.get_socket_pose(pose, s, space_world))})
                rec['sockets_from_ref_pose'] = socks
            except Exception as ex:
                rec['sockets_error'] = str(ex)[:200]
        except Exception:
            fail('skeletons', p, traceback.format_exc())
        try:
            names = [str(n) for n in lib.get_curve_meta_data_names(sk)]
            cm = []
            for n in names:
                e = {'name': n}
                for k, fn in (('material', 'get_curve_meta_data_material'), ('morph_target', 'get_curve_meta_data_morph_target')):
                    try:
                        e[k] = bool(getattr(lib, fn)(sk, n))
                    except Exception:
                        pass
                cm.append(e)
            rec['curve_metadata'] = cm
        except Exception as ex:
            rec['curve_metadata_error'] = str(ex)[:200]
        try:
            pm = lib.get_skeleton_preview_mesh(sk, False)
            rec['preview_mesh'] = obj_path(pm) if pm else None
        except Exception:
            pass
        rec['props'] = dump_object(sk)
        out['skeletons'].append(rec)
        progress('skeleton %s: %d bones' % (p, rec.get('bone_count', -1)))
    for ad in limited(sorted(BY_CLASS.get('SkeletalMesh', []), key=ad_path)):
        p = ad_path(ad)
        if p.startswith('/GASPALS/MetaHumans/') and 'Body' not in p:
            continue
        m = load(p, 'skeletal_meshes')
        if m is None:
            continue
        rec = {'path': p}
        try:
            rec['skeleton'] = obj_path(m.get_editor_property('skeleton'))
        except Exception:
            pass
        try:
            socks = []
            for i in range(m.num_sockets()):
                s = m.get_socket_by_index(i)
                socks.append({'name': str(s.get_editor_property('socket_name')), 'bone': str(s.get_editor_property('bone_name')),
                              'location': plain(s.get_editor_property('relative_location')),
                              'rotation': plain(s.get_editor_property('relative_rotation')),
                              'scale': plain(s.get_editor_property('relative_scale')),
                              'owner': obj_path(s.get_outer())})
            rec['sockets'] = socks
        except Exception as ex:
            rec['sockets_error'] = str(ex)[:300]
        for pn in ('physics_asset', 'post_process_anim_blueprint', 'default_animating_rig', 'mesh_clothing_assets'):
            try:
                rec[pn] = plain(m.get_editor_property(pn))
            except Exception:
                pass
        out['skeletal_meshes'].append(rec)
    for cls in ('IKRigDefinition', 'IKRetargeter', 'MirrorDataTable', 'BlendProfile', 'PhysicsAsset'):
        for ad in limited(sorted(BY_CLASS.get(cls, []), key=ad_path)):
            p = ad_path(ad)
            if cls == 'PhysicsAsset' and p.startswith('/GASPALS/MetaHumans/'):
                continue
            o = load(p, 'rigs')
            if o is None:
                continue
            try:
                rec = dump_object(o)
                if cls == 'MirrorDataTable':
                    try:
                        rec['csv'] = first_str(unreal.DataTableFunctionLibrary.export_data_table_to_csv_string(o))
                        rec['json'] = first_str(unreal.DataTableFunctionLibrary.export_data_table_to_json_string(o))
                    except Exception as ex:
                        rec['export_error'] = str(ex)[:300]
                out['rigs'].append(rec)
            except Exception:
                fail('rigs', p, traceback.format_exc())
    write_json('skeletons.json', out)


# ---------------------------------------------------------------- blueprints / data
def bp_generated_class(p):
    try:
        return unreal.load_object(None, p + '_C')
    except Exception:
        return None


def phase_blueprints():
    out = {'blueprints': [], 'enums': [], 'structs': [], 'data_assets': [], 'data_tables': [], 'other_data': []}
    knh = getattr(unreal, 'KismetNodeHelperLibrary', None)  # not exposed to Python in UE 5.8: enum entries come from T3D
    for ad in limited(sorted([a for a in ALL if ad_class(a) in BP_CLASSES], key=ad_path)):
        p = ad_path(ad)
        bp = load(p, 'blueprints')
        if bp is None:
            continue
        rec = {'path': p, 'class': ad_class(ad)}
        for tag in ('ParentClass', 'NativeParentClass', 'BlueprintType', 'NumReplicatedProperties', 'IsDataOnly',
                    'TargetSkeleton', 'BlueprintPath', 'ImplementedInterfaces'):
            try:
                v = ad.get_tag_value(tag)
                if v:
                    rec.setdefault('registry_tags', {})[tag] = str(v)
            except Exception:
                pass
        for pn in ('blueprint_description', 'blueprint_category', 'blueprint_namespace', 'target_skeleton',
                   'use_multi_threaded_animation_update', 'warn_about_blueprint_usage', 'is_template',
                   'default_binding_mode', 'preview_skeletal_mesh', 'parent_asset_overrides'):
            try:
                rec[pn] = plain(bp.get_editor_property(pn))
            except Exception:
                pass
        if ad_class(ad) == 'AnimBlueprint':
            try:
                rec['anim_graphs'] = [obj_path(g) for g in lib.get_animation_graphs(bp)]
            except Exception as ex:
                rec['anim_graphs_error'] = str(ex)[:200]
        gc = bp_generated_class(p)
        rec['generated_class'] = obj_path(gc) if gc else None
        out['blueprints'].append(rec)
    for ad in limited(sorted(BY_CLASS.get('UserDefinedEnum', []), key=ad_path)):
        p = ad_path(ad)
        e = load(p, 'enums')
        if e is None:
            continue
        rec = {'path': p, 'entries': []}
        try:
            rec['props'] = dump_object(e)
        except Exception:
            pass
        try:
            pt = unreal.get_type_from_enum(e)
            ents = []
            for name in dir(pt):
                if name.startswith('_'):
                    continue
                try:
                    v = getattr(pt, name)
                except Exception:
                    continue
                if isinstance(v, unreal.EnumBase) and not callable(v):
                    ent = {'py_name': name}
                    try:
                        ent['value'] = int(v.value)
                    except Exception:
                        pass
                    ents.append(ent)
            rec['py_entries'] = sorted(ents, key=lambda x: x.get('value', 9999))
            rec['py_type'] = getattr(pt, '__name__', str(pt))
        except Exception as ex:
            rec['py_entries_error'] = str(ex)[:200]
        for v in (range(256) if knh else []):
            try:
                n = str(knh.get_enumerator_name(e, v))
            except Exception:
                break
            if not n or n == 'None':
                break
            try:
                fn = str(knh.get_enumerator_user_friendly_name(e, v))
            except Exception:
                fn = None
            rec['entries'].append({'value': v, 'name': n, 'display_name': fn})
            if n.endswith('_MAX'):
                break
        out['enums'].append(rec)
    for ad in limited(sorted(BY_CLASS.get('UserDefinedStruct', []), key=ad_path)):
        p = ad_path(ad)
        s = load(p, 'structs')
        if s is None:
            continue
        rec = {'path': p}
        try:
            rec['props'] = dump_object(s)
        except Exception:
            pass
        try:
            pt = unreal.get_type_from_struct(s)
            rec['py_type'] = getattr(pt, '__name__', str(pt))
            rec['py_fields'] = prop_names(pt)
            rec['py_doc'] = (pt.__doc__ or '')[:20000]
        except Exception as ex:
            rec['py_type_error'] = str(ex)[:200]
        out['structs'].append(rec)
    known = BP_CLASSES | ANIM_CLASSES | PS_CLASSES | CHOOSER_CLASSES | RIG_CLASSES | {
        'UserDefinedEnum', 'UserDefinedStruct', 'SkeletalMesh', 'StaticMesh', 'Texture2D', 'TextureCube', 'Material',
        'MaterialInstanceConstant', 'MaterialFunction', 'SoundWave', 'SoundCue', 'MetaSoundSource', 'SoundSubmix',
        'GroomAsset', 'GroomBindingAsset', 'World', 'SkeletalMeshLODSettings', 'SubsurfaceProfile',
        'CurveLinearColorAtlas', 'ControlRigShapeLibrary', 'AnimCurveCompressionSettings', 'SubmixEffectReverbPreset',
        'TouchInterface'}
    for ad in limited(sorted([a for a in ALL if ad_class(a) not in known], key=ad_path)):
        p = ad_path(ad)
        o = load(p, 'data')
        if o is None:
            continue
        try:
            rec = dump_object(o)
            cls = ad_class(ad)
            if isinstance(o, unreal.DataTable):
                rec['csv'] = first_str(unreal.DataTableFunctionLibrary.export_data_table_to_csv_string(o))
                rec['json'] = first_str(unreal.DataTableFunctionLibrary.export_data_table_to_json_string(o))
                out['data_tables'].append(rec)
            elif cls.endswith('_C') or isinstance(o, unreal.DataAsset):
                out['data_assets'].append(rec)
            else:
                out['other_data'].append(rec)
        except Exception:
            fail('data', p, traceback.format_exc())
    progress('blueprints: %d BPs, %d enums, %d structs, %d data assets, %d tables, %d other' % (
        len(out['blueprints']), len(out['enums']), len(out['structs']), len(out['data_assets']), len(out['data_tables']),
        len(out['other_data'])))
    write_json('blueprints_ue.json', out)


# ---------------------------------------------------------------- T3D
T3D_EXPORTER = None
try:
    T3D_EXPORTER = unreal.ObjectExporterT3D()
except Exception as e:
    progress('ObjectExporterT3D not constructible (%s); relying on extension lookup' % e)


def export_t3d(obj, filename):
    if SKIP_EXISTING and os.path.exists(filename) and os.path.getsize(filename) > 0:
        return True, 'skipped-existing'
    task = unreal.AssetExportTask()
    task.set_editor_property('object', obj)
    task.set_editor_property('filename', filename)
    task.set_editor_property('automated', True)
    task.set_editor_property('prompt', False)
    task.set_editor_property('replace_identical', True)
    try:
        task.set_editor_property('use_file_archive', True)
    except Exception:
        pass
    if T3D_EXPORTER is not None:
        task.set_editor_property('exporter', T3D_EXPORTER)
    ok = unreal.Exporter.run_asset_export_task(task)
    errs = []
    try:
        errs = [str(x) for x in task.get_editor_property('errors')]
    except Exception:
        pass
    # UE 5.8 returns False for text exports written through the file archive although the file is complete;
    # success = a non-empty file that ends with 'End Object'.
    written = os.path.exists(filename) and os.path.getsize(filename) > 0
    if written:
        with open(filename, 'rb') as f:
            f.seek(max(0, os.path.getsize(filename) - 64))
            tail = f.read()
        if b'End Object' not in tail:
            errs.append('file does not end with End Object (truncated?)')
            written = False
    if not ok and not written:
        errs.append('run_asset_export_task returned False')
    return written, errs


def flat(p):
    return p.split('.', 1)[0].strip('/').replace('/', '__')


T3D_CLASSES = BP_CLASSES | PS_CLASSES | CHOOSER_CLASSES | RIG_CLASSES | {
    'AnimMontage', 'AnimComposite', 'BlendSpace', 'BlendSpace1D', 'AimOffsetBlendSpace', 'AimOffsetBlendSpace1D', 'PoseAsset',
    'UserDefinedEnum', 'UserDefinedStruct', 'DataTable', 'CurveTable', 'MirrorDataTable', 'CurveFloat', 'CurveVector',
    'InputAction', 'InputMappingContext', 'CameraRigAsset', 'CameraAsset', 'CameraDirector', 'PrimaryDataAsset', 'DataAsset',
    'ControlRigShapeLibrary'}


def phase_t3d():
    targets = []
    for ad in ALL:
        c = ad_class(ad)
        p = ad_path(ad)
        if not (c in T3D_CLASSES or c.endswith('_C')):
            continue
        if p.startswith(T3D_SKIP_PREFIXES) and c in T3D_SKIP_CLASSES_UNDER_PREFIX:
            continue
        targets.append(ad)
    targets.sort(key=ad_path)
    targets = limited(targets)
    progress('t3d: %d assets' % len(targets))
    index = []
    for i, ad in enumerate(targets):
        p = ad_path(ad)
        c = ad_class(ad)
        o = load(p, 't3d')
        if o is None:
            continue
        fn = os.path.join(T3D, flat(p) + '.t3d')
        rec = {'path': p, 'class': c, 'file': os.path.basename(fn)}
        try:
            ok, errs = export_t3d(o, fn)
            rec['ok'] = ok
            if errs:
                rec['errors'] = errs
            if ok and os.path.exists(fn):
                rec['bytes'] = os.path.getsize(fn)
            if not ok:
                fail('t3d', p, 'export returned False %s' % errs)
        except Exception:
            fail('t3d', p, traceback.format_exc())
            rec['ok'] = False
        if c in BP_CLASSES:
            gc = bp_generated_class(p)
            if gc is not None:
                try:
                    cdo = unreal.get_default_object(gc)
                    cfn = os.path.join(T3D, flat(p) + '.cdo.t3d')
                    ok2, errs2 = export_t3d(cdo, cfn)
                    rec['cdo_file'] = os.path.basename(cfn) if ok2 else None
                    if ok2:
                        rec['cdo_bytes'] = os.path.getsize(cfn)
                    else:
                        fail('t3d-cdo', p, 'export returned False %s' % errs2)
                except Exception:
                    fail('t3d-cdo', p, traceback.format_exc())
        index.append(rec)
        if i % 25 == 0:
            progress('t3d %d/%d %s' % (i, len(targets), p))
            write_json('t3d_index.json', {'count': len(index), 'files': index})
    write_json('t3d_index.json', {'count': len(index), 'files': index})


def phase_classt3d():
    """Generated classes of plain Blueprints (component templates of SCS components, timelines). Run last (riskiest)."""
    index = []
    for ad in limited(sorted(BY_CLASS.get('Blueprint', []), key=ad_path)):
        p = ad_path(ad)
        if p.startswith('/GASPALS/MetaHumans/'):
            continue
        bp = load(p, 'classt3d')
        gc = bp_generated_class(p) if bp else None
        if gc is None:
            continue
        fn = os.path.join(T3D, flat(p) + '.class.t3d')
        try:
            ok, errs = export_t3d(gc, fn)
            index.append({'path': p, 'file': os.path.basename(fn), 'ok': ok, 'bytes': os.path.getsize(fn) if ok else None})
        except Exception:
            fail('classt3d', p, traceback.format_exc())
    write_json('t3d_class_index.json', {'count': len(index), 'files': index})


PHASE_FN = {'assets': phase_assets, 'anims': phase_anims, 'posesearch': phase_posesearch, 'choosers': phase_choosers,
            'skeletons': phase_skeletons, 'blueprints': phase_blueprints, 't3d': phase_t3d, 'classt3d': phase_classt3d}

progress('START phases=%s limit=%s out=%s engine=%s' % (PHASES, LIMIT, OUT, unreal.SystemLibrary.get_engine_version()))
for ph in PHASES:
    fn = PHASE_FN.get(ph)
    if fn is None:
        progress('unknown phase %s' % ph)
        continue
    progress('PHASE %s begin' % ph)
    try:
        fn()
    except Exception:
        fail(ph, '<phase>', traceback.format_exc())
    write_json('failures.json', FAILURES)
    progress('PHASE %s end' % ph)
    try:
        unreal.SystemLibrary.collect_garbage()
    except Exception:
        pass
write_json('failures.json', FAILURES)
progress('DONE failures=%d' % len(FAILURES))
