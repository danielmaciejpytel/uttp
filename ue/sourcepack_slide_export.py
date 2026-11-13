"""READ-ONLY export of the source pack Slide animations for the UTPP port (slide / sprint-to-crouch research 2026-10-19).

Run inside UnrealEditor-Cmd (-run=pythonscript) on the GASPALS 1.2 UE project, ONLY while no editor has the
project open (source pack may be open in the GUI editor: do not run then, never kill it), e.g. in the background:
  "<UE install>\\Engine\\Binaries\\Win64\\UnrealEditor-Cmd.exe" "<GASPALS 1.2 project>\\GASPALS.uproject"
    -run=pythonscript -script="<project folder>/Source/ue/sourcepack_slide_export.py" -EnablePlugins=PythonScriptPlugin
    -unattended -nop4 -nosplash -NoSound -culture=en -abslog="<project folder>/Research/logs/sourcepack-slide-export-ue.log"
  then poll Animations\\sourcepack\\slide_export\\_progress.log until "DONE".

Never saves, resaves or modifies a package: it only loads assets, runs FBX export tasks into files OUTSIDE the project and
writes JSON next to them (same FbxExportOption and meta fields as sourcepack_export.py).

Selection: every AnimSequence named M_(Neutral|Relaxed)_Slide_* under /GASPALS (expected 40 = 16 Neutral + 24 Relaxed, all in
/GASPALS/Characters/UEFN_Mannequin/Animations/Slide/). Also recorded as JSON (read-only reflection dump, no FBX): the PoseSearch
databases PSD_*Slide* and schemas PSS_*Slide* (MotionMatchingData/Databases|Schemas) and the chooser table
CHT_PoseSearchDatabases_Relaxed incl. its nested table 'Slide' -> meta\\_data__<Name>.json and manifest.json section 'data_assets'.

Offline selftest (system Python, no UE, writes nothing): prints the planned selection from the inventory JSONs, asserts 40 clips:
    python -I Source\\ue\\sourcepack_slide_export.py [--analysis PATH] [--inventory PATH]

Output (default <project folder>\\Animations\\sourcepack\\slide_export): fbx\\, meta\\, manifest.json, _progress.log.
Env (optional): UTPP_SOURCEPACK_SLIDE_OUT (output folder), UTPP_SOURCEPACK_SLIDE_SKIP_EXISTING=1 (resume).
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
except ImportError:         # system Python: offline selftest only
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
UE_PROJECT_DIR = os.environ.get('UTPP_SOURCEPACK_SLIDE_PROJECT') or os.path.join(os.path.dirname(PROJECT_ROOT), 'Unreal Engine', 'GASPALS-1.2')   # folder of GASPALS.uproject
OUT = os.environ.get('UTPP_SOURCEPACK_SLIDE_OUT', os.path.join(PROJECT_ROOT, 'Animations', 'sourcepack', 'slide_export'))
SKIP_EXISTING = os.environ.get('UTPP_SOURCEPACK_SLIDE_SKIP_EXISTING', '0') == '1'
ANALYSIS = os.path.join(PROJECT_ROOT, 'Research', 'logs', 'sourcepack-slide-analysis-2026-10-19.json')
INVENTORY = os.path.join(PROJECT_ROOT, 'Research', 'logs', 'sourcepack-vs-base-inventory-2026-10-19.json')
MOUNT = '/GASPALS/'
SLIDE_FOLDER = '/GASPALS/Characters/UEFN_Mannequin/Animations/Slide'   # verified on disk 2026-10-19
MM_DIR = '/GASPALS/Characters/UEFN_Mannequin/Animations/MotionMatchingData'
CHOOSER_PKG = MM_DIR + '/CHT_PoseSearchDatabases_Relaxed'
SLIDE_RE = re.compile(r'^M_(Neutral|Relaxed)_Slide_')
DATA_RE = re.compile(r'^PS[DS]_.*Slide')
SEQ_CLASS = 'AnimSequence'


def planned_clips(analysis_path=ANALYSIS):
    """Slide clip names from the analysis JSON (clips array) -> object paths under SLIDE_FOLDER."""
    with open(analysis_path, encoding='utf-8') as f:
        clips = json.load(f)['clips']
    names = sorted(c['name'] for c in clips if SLIDE_RE.match(c['name']))
    return names, ['%s/%s.%s' % (SLIDE_FOLDER, n, n) for n in names]


def offline_main(argv):
    import argparse
    ap = argparse.ArgumentParser(prog='sourcepack_slide_export.py (offline selftest, no UE)')
    ap.add_argument('--analysis', default=ANALYSIS)
    ap.add_argument('--inventory', default=INVENTORY)
    a = ap.parse_args(argv)
    names, paths = planned_clips(a.analysis)
    for p in paths:
        print(p)
    by = {s: sum(1 for n in names if n.startswith('M_%s_' % s)) for s in ('Neutral', 'Relaxed')}
    with open(a.analysis, encoding='utf-8') as f:
        d = json.load(f)
    dbs = sorted(d.get('databases') or {})
    used = sorted({x for c in d['clips'] for x in c.get('used_by_databases_in_1_2', [])})
    with open(a.inventory, encoding='utf-8') as f:
        inv_text = f.read()
    in_inv = sum(1 for n in names if n in inv_text)
    print('databases (analysis): %s' % dbs)
    print('databases referenced by clips: %s' % used)
    print('chooser: %s nested table Slide' % CHOOSER_PKG)
    print('UTPP_SLIDE_SELFTEST ' + json.dumps({'clips': len(names), 'by_style': by, 'unique': len(set(names)),
                                                'named_in_inventory_json': in_inv, 'databases': len(dbs)}))
    assert len(names) == 40 and len(set(names)) == 40, 'expected 40 slide clips, got %d' % len(names)
    assert by == {'Neutral': 16, 'Relaxed': 24}, by
    print('SELFTEST OK')
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
    unreal.log('[UTPP-SOURCEPACK-SLIDE] ' + line)
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


SELECTED = sorted((ad for ad in ALL if ad_class(ad) == SEQ_CLASS and SLIDE_RE.match(str(ad.asset_name))), key=ad_path)
DATA_ASSETS = sorted((ad for ad in ALL if DATA_RE.match(str(ad.asset_name))), key=ad_path)
EXPECTED, _ = planned_clips()
MISSING = sorted(set(EXPECTED) - {str(ad.asset_name) for ad in SELECTED})
EXTRA = sorted({str(ad.asset_name) for ad in SELECTED} - set(EXPECTED))
progress('START out=%s registry=%d slide_sequences=%d (expected %d) data_assets=%d missing=%s extra=%s engine=%s' % (
    OUT, len(ALL), len(SELECTED), len(EXPECTED), len(DATA_ASSETS), MISSING, EXTRA, unreal.SystemLibrary.get_engine_version()))
DEP_OPTS = unreal.AssetRegistryDependencyOptions()
for k, v in (('include_soft_package_references', True), ('include_hard_package_references', True),
             ('include_searchable_names', False), ('include_soft_management_references', False),
             ('include_hard_management_references', False)):
    try:
        DEP_OPTS.set_editor_property(k, v)
    except Exception:
        pass


def referencers(pkg):
    try:
        return sorted({str(x) for x in (registry.get_referencers(pkg, DEP_OPTS) or [])})
    except Exception as e:
        return ['_error: %s' % str(e)[:200]]


def load_json(path, what):
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except Exception as e:
        progress('cannot read %s (%s): %s' % (what, path, e))
        return None


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



def process(ad, stats):
    path, name = ad_path(ad), str(ad.asset_name)
    meta_path = os.path.join(META_DIR, name + '.json')
    fbx_path = os.path.join(FBX_DIR, name + '.fbx')
    if SKIP_EXISTING and os.path.exists(meta_path) and os.path.exists(fbx_path):
        old = load_json(meta_path, 'existing meta') or {}
        if old.get('export_ok'):
            stats['skipped'] += 1
            return old.get('_manifest_row') or {'name': name, 'path': path, 'export_ok': True, 'skipped_existing': True}
    a = unreal.load_asset(path)
    if a is None:
        raise RuntimeError('load_asset returned None')
    meta = {'name': name, 'file_name': name, 'path': path, 'package': str(ad.package_name), 'group': 'Slide',
            'class': SEQ_CLASS, 'selection_kind': 'export',
            'skeleton': obj_path(prop(a, 'skeleton')) if prop(a, 'skeleton') else None}
    meta.update(timing(a))
    ok, errors = export_fbx(a, fbx_path)
    exists = os.path.exists(fbx_path) and os.path.getsize(fbx_path) > 0
    if not exists:
        ok = False
        errors.append('FBX file missing or empty after export')
    meta['fbx'] = fbx_path if exists else None
    meta['fbx_file'] = 'fbx/' + name + '.fbx' if exists else None
    meta['fbx_bytes'] = os.path.getsize(fbx_path) if exists else None
    meta['fbx_md5'] = md5_file(fbx_path) if exists else None
    meta['fbx_content'] = 'full source pose from the data-model bone tracks (also for additive assets)'
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
    meta.update(curves(a, meta['num_frames'], meta.get('fps') or 30.0, meta['length_s']))
    try:
        meta['notify_tracks'] = [str(x) for x in lib.get_animation_notify_track_names(a)]
    except Exception:
        pass
    meta['notifies'] = notify_events(a)
    meta['referenced_by'] = referencers(ad.package_name)
    meta['export_ok'] = bool(ok)
    meta['export_errors'] = errors
    row = {'name': name, 'path': path, 'class': SEQ_CLASS, 'export_ok': bool(ok), 'fbx': meta.get('fbx_file'),
           'fbx_md5': meta.get('fbx_md5'), 'fbx_bytes': meta.get('fbx_bytes'), 'length_s': meta.get('length_s'),
           'num_frames': meta.get('num_frames'), 'fps': meta.get('fps'), 'enable_root_motion': meta.get('enable_root_motion'),
           'force_root_lock': meta.get('force_root_lock'), 'root_track_end': meta.get('root_track_end'),
           'curves': sorted((meta.get('curves') or {}).keys()), 'notify_count': len(meta.get('notifies') or [])}
    if errors:
        row['errors'] = errors
    meta['_manifest_row'] = row
    with open(meta_path, 'w', encoding='utf-8') as f:
        json.dump(meta, f, indent=1, default=str)
    if not ok:
        stats['failed'] += 1
        progress('EXPORT FAILED %s: %s' % (path, errors))
    return row


def dump_data_assets():
    """Read-only reflection dump of PSD_/PSS_ *Slide* and the Relaxed chooser table (incl. nested table 'Slide')."""
    out = {}
    targets = [(ad_path(ad), ad_class(ad)) for ad in DATA_ASSETS]
    targets.append((CHOOSER_PKG + '.CHT_PoseSearchDatabases_Relaxed', 'ChooserTable'))
    for path, cls in targets:
        name = path.rsplit('.', 1)[-1]
        try:
            a = unreal.load_asset(path)
            if a is None:
                raise RuntimeError('load_asset returned None')
            d = dump_object(a)
            with open(os.path.join(META_DIR, '_data__' + name + '.json'), 'w', encoding='utf-8') as f:
                json.dump(d, f, indent=1, default=str)
            out[name] = {'path': path, 'class': cls, 'meta': 'meta/_data__' + name + '.json', 'ok': True}
        except Exception:
            out[name] = {'path': path, 'class': cls, 'ok': False, 'error': traceback.format_exc()[-1500:]}
            progress('DATA DUMP FAIL %s' % path)
    return out


def write_manifest(rows, failures, stats, final, data_assets):
    man = {'complete': final, 'engine': unreal.SystemLibrary.get_engine_version(),
           'project': os.path.join(UE_PROJECT_DIR, 'GASPALS.uproject'), 'script': 'Source/ue/sourcepack_slide_export.py',
           'output': OUT, 'skip_existing': SKIP_EXISTING,
           'fbx_settings': {'exporter': 'unreal.Exporter.run_asset_export_task (AnimSequenceExporterFBX)',
                            'export_morph_targets': False, 'export_preview_mesh': False, 'export_local_time': True,
                            'collision': False, 'level_of_detail': False, 'vertex_color': False,
                            'same_as': 'Source/ue/sourcepack_export.py'},
           'selection': {'regex': SLIDE_RE.pattern, 'folder': SLIDE_FOLDER, 'expected': len(EXPECTED),
                         'missing_in_registry': MISSING, 'extra_in_registry': EXTRA},
           'stats': {'selected': len(SELECTED), 'processed': len(rows), 'exceptions': len(failures),
                     'export_failed': stats['failed'], 'skipped_existing': stats['skipped'],
                     'sequences_ok': sum(1 for r in rows if r.get('export_ok')),
                     'fbx_bytes': sum(int(r.get('fbx_bytes') or 0) for r in rows),
                     'elapsed_s': round(time.time() - T0, 1)},
           'data_assets': data_assets, 'failures': failures, 'sequences': rows}
    tmp = os.path.join(OUT, 'manifest.json.tmp')
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(man, f, indent=1, default=str)
    os.replace(tmp, os.path.join(OUT, 'manifest.json'))


def main():
    rows, failures, stats = [], [], {'failed': 0, 'skipped': 0}
    for i, ad in enumerate(SELECTED):
        try:
            rows.append(process(ad, stats))
        except Exception:
            failures.append({'path': ad_path(ad), 'error': traceback.format_exc()[-3000:]})
            progress('FAIL %s: %s' % (ad_path(ad), traceback.format_exc().strip().splitlines()[-1][:300]))
        if i % 10 == 0 or i == len(SELECTED) - 1:
            progress('%d/%d %s' % (i + 1, len(SELECTED), ad_path(ad)))
        if i % 20 == 19:
            unreal.SystemLibrary.collect_garbage()
    data_assets = dump_data_assets()
    write_manifest(rows, failures, stats, True, data_assets)
    progress('DONE processed=%d sequences_ok=%d export_failed=%d exceptions=%d data_assets=%d' % (
        len(rows), sum(1 for r in rows if r.get('export_ok')), stats['failed'], len(failures), len(data_assets)))


main()
