"""Offline post-processor for the source pack read-only extraction (system python, no engine).

Inputs  (all inside <OUT>, default <project folder>\\Extracted\\sourcepack\\sourcepack):
  ue_raw\\*.json          written by Source\\ue\\sourcepack_extract.py inside UnrealEditor-Cmd
  t3d\\*.t3d              T3D exports of logic assets (written by the same script)
  uasset_names\\*.json    offline name/import/export tables (Source\\tools\\uasset\\uasset_names.py)
Reads (copies only) the project config: <PROJECT>\\Config\\*.ini, Config\\Tags\\*.ini, plugin Config.
Outputs (top level of <OUT>): assets.json, anims.json, anim_curves.json, posesearch.json, choosers.json,
  blueprints.json + blueprints\\<asset>.json, skeletons.json, data.json, gameplay_tags.json, config\\*, README.txt
Usage: python -I sourcepack_post.py [OUT] [PROJECT]
"""
import collections
import json
import os
import shutil
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import t3d  # noqa: E402
from t3d import objref, as_float, as_bool, text_of  # noqa: E402

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))   # the tools folder
from utpp_paths import project_root, local_path  # noqa: E402

_UE_DIR = os.path.join(os.path.dirname(project_root()), 'Unreal Engine')   # sibling folder of the project folder; override with local_paths.json / UTPP_<KEY>
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(project_root(), 'Extracted', 'sourcepack', 'sourcepack')
PROJECT = sys.argv[2] if len(sys.argv) > 2 else local_path('sourcepack_project', os.path.join(_UE_DIR, 'Test', 'ALSV5', 'GASP-ALS'))
RAW = os.path.join(OUT, 'ue_raw')
T3D_DIR = os.path.join(OUT, 't3d')
NAMES_DIR = os.path.join(OUT, 'uasset_names')
BP_DIR = os.path.join(OUT, 'blueprints')
PROBLEMS = []
GASP_PROJECT_CONTENT = os.path.join(local_path('gasp_original_project', os.path.join(_UE_DIR, 'GameAnimationSample')), 'Content')  # original source pack (file listing only, read-only)


def problem(where, err):
    PROBLEMS.append({'where': where, 'error': str(err)[:1500]})


def load_raw(name):
    p = os.path.join(RAW, name)
    if not os.path.exists(p):
        problem('load_raw', 'missing ' + p)
        return None
    with open(p, encoding='utf-8') as f:
        return json.load(f)


def dump(name, data, indent=1):
    p = os.path.join(OUT, name)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=indent, ensure_ascii=False, default=str)
    return p


def flat(path):
    return path.split('.', 1)[0].strip('/').replace('/', '__')


def short(p):
    if not p:
        return p
    return str(p).rsplit('/', 1)[-1].split('.')[-1]


# ------------------------------------------------------------------ T3D cache
T3D_INDEX = {}
_t3d_cache = {}


def t3d_for(asset_path, kind=''):
    """kind '' | 'cdo' | 'class'"""
    fn = os.path.join(T3D_DIR, flat(asset_path) + ('.' + kind if kind else '') + '.t3d')
    if not os.path.exists(fn):
        return None
    if fn in _t3d_cache:
        return _t3d_cache[fn]
    try:
        with open(fn, encoding='utf-8-sig', errors='replace') as f:
            root = t3d.parse_t3d(f.read())
    except Exception:
        problem('t3d-parse ' + fn, traceback.format_exc())
        root = None
    if len(_t3d_cache) > 40:
        _t3d_cache.clear()
    _t3d_cache[fn] = root
    return root


# ------------------------------------------------------------------ enums (needed by choosers/blueprints)
ENUMS = {}  # enum short name -> {entry internal name -> display}


def build_enums(bp_raw):
    out = []
    for e in (bp_raw or {}).get('enums', []):
        path = e['path']
        root = t3d_for(path)
        disp = {}
        if root is not None:
            for pair in root.props.get('DisplayNameMap', []) or []:
                if isinstance(pair, list) and len(pair) >= 2:
                    disp[str(pair[0])] = text_of(pair[1])
        entries = []
        nj = os.path.join(NAMES_DIR, flat(path) + '.json')
        decoded = None
        if os.path.exists(nj):
            decoded = json.load(open(nj, encoding='utf-8')).get('enum_entries')
        if decoded:
            for d in decoded:
                internal = d['name'].split('::', 1)[-1]
                ent = {'value': d['value'], 'internal_name': internal, 'display_name': disp.get(internal)}
                if internal.endswith('_MAX'):
                    ent['is_max'] = True
                entries.append(ent)
        else:
            entries = [{'value': None, 'internal_name': k, 'display_name': v} for k, v in disp.items()]
        rec = {'path': path, 'name': short(path), 'entries': entries,
               'source': ('values/order: UEnum::Names decoded from the .uasset export data (uasset_names); display names: T3D '
                          'DisplayNameMap. UE Python get_type_from_enum returns only EnumBase for user enums in 5.8.')}
        if not decoded:
            rec['warning'] = 'value order unknown (enum Names array not decoded)'
        ENUMS[short(path)] = {x.get('internal_name'): x.get('display_name') for x in entries if x.get('internal_name')}
        out.append(rec)
    return out


def enum_display(qualified):
    """'E_Gait::NewEnumerator1' -> 'Run' (None when unknown)."""
    if not qualified or '::' not in str(qualified):
        return None
    en, internal = str(qualified).split('::', 1)
    return ENUMS.get(en, {}).get(internal)


def resolve_enum_strings(v):
    if isinstance(v, dict):
        return {k: resolve_enum_strings(x) for k, x in v.items()}
    if isinstance(v, list):
        return [resolve_enum_strings(x) for x in v]
    if isinstance(v, str) and '::NewEnumerator' in v:
        d = enum_display(v)
        return '%s (%s)' % (v, d) if d else v
    return v


# ------------------------------------------------------------------ assets
def post_assets(assets_raw):
    rows = assets_raw['assets']
    refby = collections.defaultdict(set)
    for r in rows:
        for d in r.get('dependencies', []):
            refby[d].add(r['package'])
    names_index = {}
    ip = os.path.join(NAMES_DIR, 'index.json')
    if os.path.exists(ip):
        for rec in json.load(open(ip, encoding='utf-8'))['packages']:
            names_index[rec['package']] = rec
    t3d_files = set(os.listdir(T3D_DIR)) if os.path.isdir(T3D_DIR) else set()
    folders = collections.Counter()
    for r in rows:
        r['referenced_by'] = sorted(refby.get(r['package'], []))
        base = flat(r['path'])
        r['t3d'] = [f for f in (base + '.t3d', base + '.cdo.t3d', base + '.class.t3d') if f in t3d_files]
        ni = names_index.get(r['package'])
        if ni:
            r['uasset_names'] = ni.get('json')
            r['uasset_main_class'] = ni.get('class')
            r['uasset_file_version_ue5'] = ni.get('file_version_ue5')
        parts = r['folder'].strip('/').split('/')
        folders['/' + '/'.join(parts[:3])] += 1
    assets_raw['folders_top3'] = dict(sorted(folders.items()))
    assets_raw['note'] = ('dependencies = AssetRegistry hard+soft package dependencies (incl. /Script/ native packages); '
                          'referenced_by = reverse of dependencies inside /GASPALS and /Game; t3d = exported text files in t3d\\; '
                          'uasset_names = offline header dump in uasset_names\\')
    dump('assets.json', assets_raw)
    return rows


# ------------------------------------------------------------------ anims
def montage_from_t3d(root):
    p = root.props
    secs = []
    for s in p.get('CompositeSections', []) or []:
        if not isinstance(s, dict):
            s = {}
        secs.append({'name': s.get('SectionName', 'Default'), 'start_time': as_float(s.get('LinkValue'), 0.0),
                     'next_section': s.get('NextSectionName'), 'segment_index': s.get('SegmentIndex'),
                     'linked_sequence': objref(s.get('LinkedSequence'))})
    slots = []
    for st in p.get('SlotAnimTracks', []) or []:
        if not isinstance(st, dict):
            st = {}
        segs = []
        for seg in ((st.get('AnimTrack') or {}).get('AnimSegments') or []):
            if not isinstance(seg, dict):
                seg = {}
            segs.append({'anim': objref(seg.get('AnimReference')), 'start_pos': as_float(seg.get('StartPos'), 0.0),
                         'anim_start_time': as_float(seg.get('AnimStartTime'), 0.0),
                         'anim_end_time': as_float(seg.get('AnimEndTime'), 0.0),
                         'play_rate': as_float(seg.get('AnimPlayRate'), 1.0), 'looping_count': int(as_float(seg.get('LoopingCount'), 1))})
        slots.append({'slot_name': st.get('SlotName', 'DefaultSlot'), 'segments': segs})
    keep = {k: v for k, v in p.items() if k in ('BlendIn', 'BlendOut', 'BlendOutTriggerTime', 'bEnableAutoBlendOut', 'SyncGroup',
                                               'SyncSlotIndex', 'BlendModeIn', 'BlendModeOut', 'BlendProfileIn', 'BlendProfileOut',
                                               'CommonTargetFrameRate', 'bEnableRootMotionTranslation', 'bEnableRootMotionRotation',
                                               'RootMotionRootLock', 'TimeStretchCurve', 'TimeStretchCurveName', 'PreviewBasePose')}
    return {'sections': secs, 'slot_tracks': slots, 'other': keep,
            'defaults_note': 'T3D is a delta export: missing SlotName = DefaultSlot, missing StartPos/AnimStartTime/LinkValue = 0, '
                             'missing AnimPlayRate/LoopingCount = 1, missing BlendIn/BlendOut = engine defaults (0.25 s linear)'}


def blendspace_from_t3d(root):
    p = root.props
    params = []
    for i, bp in enumerate(p.get('BlendParameters', []) or []):
        if bp is None:
            continue
        if not isinstance(bp, dict):
            bp = {}
        params.append({'axis': i, 'name': bp.get('DisplayName', 'None'), 'min': as_float(bp.get('Min'), 0.0),
                       'max': as_float(bp.get('Max'), 100.0), 'grid_num': int(as_float(bp.get('GridNum'), 4)),
                       'snap_to_grid': as_bool(bp.get('bSnapToGrid'), False), 'wrap_input': as_bool(bp.get('bWrapInput'), False)})
    samples = []
    for s in p.get('SampleData', []) or []:
        if not isinstance(s, dict):
            s = {}
        sv = s.get('SampleValue') or {}
        samples.append({'anim': objref(s.get('Animation')), 'x': as_float(sv.get('X') if isinstance(sv, dict) else None, 0.0),
                        'y': as_float(sv.get('Y') if isinstance(sv, dict) else None, 0.0),
                        'rate_scale': as_float(s.get('RateScale'), 1.0)})
    other = {k: v for k, v in p.items() if k not in ('BlendParameters', 'SampleData', 'BlendSpaceData', 'GridSamples')}
    return {'axes': params, 'samples': samples, 'other': other,
            'defaults_note': 'missing axis Min/Max/GridNum = 0/100/4; missing SampleValue component = 0; RateScale default 1'}


def gasp_project_index():
    idx = {}
    for dp, _, files in os.walk(GASP_PROJECT_CONTENT):
        for fn in files:
            if fn.endswith('.uasset'):
                rel = os.path.relpath(os.path.join(dp, fn), GASP_PROJECT_CONTENT).replace(os.sep, '/')
                idx.setdefault(fn[:-7], []).append('/Game/' + rel[:-7])
    return idx


def post_anims(anims_raw, gasp_names_note):
    rows = anims_raw['anims']
    gidx = gasp_project_index()
    vs_project = collections.Counter()
    by_class = collections.Counter()
    new_vs = collections.Counter()
    notify_classes = collections.Counter()
    curve_counts = collections.Counter()
    folders = collections.Counter()
    for r in rows:
        by_class[r['class']] += 1
        new_vs['in_gasp_export' if r.get('in_gasp_export') else 'not_in_gasp_export'] += 1
        hits = gidx.get(r['name'])
        r['in_original_gasp_project'] = bool(hits)
        if hits:
            r['original_gasp_project_paths'] = hits
            r['original_gasp_same_relative_path'] = ('/Game/' + r['path'].split('/GASPALS/', 1)[-1].split('.')[0]) in hits
        vs_project['in_original_gasp_project' if hits else 'new_in_gasp_als'] += 1
        folders[r['path'].rsplit('/', 1)[0]] += 1
        for n in r.get('notifies', []) or []:
            notify_classes[n.get('class') or n.get('name')] += 1
        for c in r.get('curves', []) or []:
            curve_counts[c] += 1
        try:
            if r['class'] == 'AnimMontage':
                root = t3d_for(r['path'])
                if root is not None:
                    r['montage'] = montage_from_t3d(root)
            elif r['class'] in ('BlendSpace', 'BlendSpace1D', 'AimOffsetBlendSpace', 'AimOffsetBlendSpace1D'):
                root = t3d_for(r['path'])
                if root is not None:
                    r['blendspace'] = blendspace_from_t3d(root)
            elif r['class'] == 'PoseAsset':
                root = t3d_for(r['path'])
                if root is not None:
                    pp = root.props
                    r['pose_asset'] = {'source_animation': objref(pp.get('SourceAnimation')),
                                       'retarget_source': pp.get('RetargetSource'), 'additive': as_bool(pp.get('bAdditivePose'), False),
                                       'base_pose_index': pp.get('BasePoseIndex'),
                                       'pose_container_keys': sorted((pp.get('PoseContainer') or {}).keys()) if isinstance(pp.get('PoseContainer'), dict) else None}
        except Exception:
            problem('anims ' + r['path'], traceback.format_exc())
    summary = {
        'count': len(rows), 'by_class': dict(by_class), 'gasp_export_comparison': dict(new_vs),
        'original_gasp_project_comparison': dict(vs_project),
        'original_gasp_project_comparison_note': ('same .uasset file name found anywhere under ' + GASP_PROJECT_CONTENT +
                                                  ' (directory listing only, project not opened)'),
        'new_in_gasp_als_by_class': dict(collections.Counter(r['class'] for r in rows if not r.get('in_original_gasp_project'))),
        'new_in_gasp_als_by_folder': dict(collections.Counter(r['path'].rsplit('/', 1)[0] for r in rows
                                                              if not r.get('in_original_gasp_project')).most_common()),
        'gasp_export_comparison_note': gasp_names_note,
        'notify_classes': dict(notify_classes.most_common()), 'float_curves': dict(curve_counts.most_common()),
        'folders': dict(sorted(folders.items())),
    }
    anims_raw['summary'] = summary
    anims_raw['field_notes'] = {
        'length_s': 'UAnimSequenceBase::GetPlayLength', 'num_frames': 'AnimationLibrary.get_num_frames (frames, keys = frames + 1)',
        'frame_rate_est': 'num_frames / length_s', 'target_frame_rate': 'AnimSequence.PlatformTargetFrameRate',
        'notifies[].time/duration': 'seconds (AnimationLibrary.get_anim_notify_event_trigger_time/duration)',
        'notifies[].track': 'notify track name (matched by name+time via get_animation_notify_events_for_track)',
        'notifies[].props': 'editor properties of the notify / notify-state object',
        'curves': 'float curve names; keys in anim_curves.json',
        'root_track_start/end': 'root bone transform at t=0 and t=length (AnimationLibrary.extract_root_track_transform)',
        'in_gasp_export': 'same asset name exists in Animations\\sourcepack\\SourcePack_export inventory.json/manifest.json (UTPP export of the 35 Dense databases of the original GASP)',
        'in_original_gasp_project': 'same asset file name exists in the original GASP project content; false = new in GASP-ALS',
        'gasp_same_relative_path': 'GASP path with /Game/ replaced by /GASPALS/ equals this path',
        'montage / blendspace / pose_asset': 'parsed from T3D (delta export, see defaults_note)',
        'props': 'all Python-visible editor properties (generic reflection dump)',
    }
    dump('anims.json', anims_raw)
    return rows


# ------------------------------------------------------------------ pose search
MIRROR_DEFAULT = 'UnmirroredOnly'


def channel_summary(ch):
    cls = ch.get('_class')
    s = {'type': cls, 'weight': ch.get('weight')}
    if 'sampled_bones' in ch:
        s['bones'] = [{'bone': (b.get('reference') or {}).get('bone_name'), 'flags': b.get('flags'), 'weight': b.get('weight')}
                      for b in ch.get('sampled_bones') or []]
        s['bone_flags_note'] = 'EPoseSearchBoneFlags bitmask (PoseSearchFeatureChannel_Pose.h:11, UE 5.8): 1 Velocity, 2 Position, 4 Rotation, 8 Phase'
    if 'samples' in ch:
        s['samples'] = [{'offset': x.get('offset'), 'flags': x.get('flags'), 'weight': x.get('weight')} for x in ch.get('samples') or []]
        s['sample_flags_note'] = 'EPoseSearchTrajectoryFlags bitmask (PoseSearchFeatureChannel_Trajectory.h:12, UE 5.8): 1 Velocity, 2 Position, 4 VelocityDirection, 8 FacingDirection, 16 VelocityXY, 32 PositionXY, 64 VelocityDirectionXY, 128 FacingDirectionXY'
    for k in ('bone', 'sample_time_offset', 'sample_role', 'origin_role', 'input_query_pose', 'heading_axis', 'component_stripping',
              'curve_name', 'use_character_space_velocities', 'permutation_time_type', 'normalize_display_name', 'time_to_event'):
        if k in ch:
            s[k] = ch[k]
    if 'sub_channels' in ch:
        s['sub_channels'] = [channel_summary(x) if isinstance(x, dict) else x for x in ch.get('sub_channels') or []]
    return s


def post_posesearch(ps_raw, anims_by_path, choosers_refs):
    schemas = {s['_path']: s for s in ps_raw['schemas']}
    anim_dbs = collections.defaultdict(list)
    for s in ps_raw['schemas']:
        s['channels_summary'] = [channel_summary(c) for c in s.get('channels') or [] if isinstance(c, dict)]
    for db in ps_raw['databases']:
        path = db['_path']
        root = t3d_for(path)
        t3d_entries = (root.props.get('DatabaseAnimationAssets') if root is not None else None) or []
        sch = schemas.get(db.get('schema'))
        rate = (sch or {}).get('sample_rate', 30) or 30
        ex = db.get('exclude_from_database_parameters') or {}
        ents = []
        tot_poses = 0
        tot_secs = 0.0
        for i, api in enumerate(db.get('entries_api') or []):
            te = t3d_entries[i] if i < len(t3d_entries) and isinstance(t3d_entries[i], dict) else {}
            asset = api.get('asset')
            if te and objref(te.get('AnimAsset')) not in (None, asset):
                problem('posesearch ' + path, 'entry %d asset mismatch api=%s t3d=%s' % (i, asset, objref(te.get('AnimAsset'))))
            sr = te.get('SamplingRange') if isinstance(te.get('SamplingRange'), dict) else {}
            e = {
                'index': i, 'asset': asset, 'asset_class': api.get('asset_class'), 'play_length_s': api.get('play_length_s'),
                'enabled': as_bool(te.get('bEnabled'), True), 'disable_reselection': as_bool(te.get('bDisableReselection'), False),
                'mirror_option': te.get('MirrorOption', MIRROR_DEFAULT), 'branch_in_id': int(as_float(te.get('BranchInId'), 0)),
                'sampling_range': [as_float(sr.get('Min'), 0.0), as_float(sr.get('Max'), 0.0)],
            }
            if api.get('asset_class') in ('BlendSpace', 'BlendSpace1D'):
                e.update({'use_single_sample': as_bool(te.get('bUseSingleSample'), False),
                          'use_grid_for_sampling': as_bool(te.get('bUseGridForSampling'), False),
                          'horizontal_samples': int(as_float(te.get('NumberOfHorizontalSamples'), 9)),
                          'vertical_samples': int(as_float(te.get('NumberOfVerticalSamples'), 2)),
                          'blend_param_x': as_float(te.get('BlendParamX'), 0.0), 'blend_param_y': as_float(te.get('BlendParamY'), 0.0)})
            extra = {k: v for k, v in te.items() if k not in ('AnimAsset', 'bEnabled', 'bDisableReselection', 'MirrorOption', 'BranchInId',
                                                             'SamplingRange', 'bUseSingleSample', 'bUseGridForSampling',
                                                             'NumberOfHorizontalSamples', 'NumberOfVerticalSamples', 'BlendParamX', 'BlendParamY')}
            if extra:
                e['other_t3d'] = extra
            ln = api.get('play_length_s') or 0.0
            a, b = e['sampling_range']
            if a == 0.0 and b == 0.0:
                a, b = 0.0, ln
            b = min(b, ln) if ln else b
            secs = max(0.0, b - a)
            looping = (anims_by_path.get(asset) or {}).get('props', {}).get('loop')
            e['looping'] = looping
            poses = int(secs * rate) + 1 if secs > 0 else 0
            mult = 2 if e['mirror_option'] == 'UnmirroredAndMirrored' else 1
            if api.get('asset_class') in ('BlendSpace', 'BlendSpace1D') and not e.get('use_single_sample'):
                mult *= max(1, e.get('horizontal_samples', 1)) * max(1, e.get('vertical_samples', 1))
            e['estimated_poses'] = poses * mult if e['enabled'] else 0
            e['sampled_seconds'] = round(secs, 4)
            if e['enabled']:
                tot_poses += e['estimated_poses']
                tot_secs += secs * mult
            ents.append(e)
            if asset:
                anim_dbs[asset].append(short(path))
        db['entries'] = ents
        db['entry_count'] = len(ents)
        db['estimated_total_poses'] = tot_poses
        db['estimated_total_seconds'] = round(tot_secs, 3)
        db['estimate_note'] = ('estimated_poses = floor(sampled_seconds * schema.sample_rate) + 1, x2 for UnmirroredAndMirrored, '
                               'x horizontal*vertical samples for blend spaces; ExcludeFromDatabaseParameters (%s) and pruning not applied' % ex)
        db['referenced_by_choosers'] = sorted(choosers_refs.get(path, []))
        db['t3d_defaults_note'] = ('entry fields missing from T3D are C++ defaults: bEnabled=True, bDisableReselection=False, '
                                   'MirrorOption=UnmirroredOnly, BranchInId=0, SamplingRange=(0,0)=whole asset')
        db.pop('entries_api', None)
    ps_raw['anim_to_databases'] = {k: sorted(set(v)) for k, v in sorted(anim_dbs.items())}
    ps_raw['summary'] = {'schemas': len(ps_raw['schemas']), 'databases': len(ps_raw['databases']),
                         'normalization_sets': len(ps_raw['normalization_sets']),
                         'unique_assets_in_databases': len(anim_dbs),
                         'estimated_total_poses_all_dbs': sum(d['estimated_total_poses'] for d in ps_raw['databases'])}
    dump('posesearch.json', ps_raw)
    return anim_dbs


# ------------------------------------------------------------------ choosers
def chooser_table(obj, root_path):
    p = obj.props
    cols = []
    for ci, c in enumerate(p.get('ColumnsStructs') or []):
        if not isinstance(c, dict):
            cols.append({'index': ci, 'raw': c})
            continue
        iv = c.get('InputValue') if isinstance(c.get('InputValue'), dict) else {}
        binding = iv.get('Binding') if isinstance(iv.get('Binding'), dict) else {}
        col = {'index': ci, 'type': short(c.get('_type')), 'type_path': c.get('_type'),
               'input_type': short(iv.get('_type')) if iv else None,
               'input_binding': binding.get('PropertyBindingChain'), 'input_display': binding.get('DisplayName'),
               'input_context_index': binding.get('ContextIndex'), 'input_enum': objref(binding.get('Enum')),
               'disabled': as_bool(c.get('bDisabled'), False)}
        rv = c.get('RowValues')
        if rv is None:
            rv = c.get('RowValuesWithAny')
        col['row_values'] = resolve_enum_strings(rv)
        col['settings'] = {k: v for k, v in c.items() if k not in ('_type', 'InputValue', 'RowValues', 'RowValuesWithAny', 'EditorColumnWidth', 'bDisabled')}
        cols.append(col)
    results = []
    for ri, r in enumerate(p.get('ResultsStructs') or []):
        if isinstance(r, dict):
            res = {'type': short(r.get('_type'))}
            for k, v in r.items():
                if k == '_type':
                    continue
                res[k] = objref(v) if isinstance(v, str) and "'" in v else v
            results.append(res)
        else:
            results.append({'raw': r})
    disabled = [as_bool(x, False) for x in (p.get('DisabledRows') or [])]
    rows = []
    for ri, res in enumerate(results):
        cells = {}
        for col in cols:
            rv = col.get('row_values')
            if isinstance(rv, list) and ri < len(rv):
                label = col.get('input_display') or '.'.join(str(x) for x in (col.get('input_binding') or [])) or str(col.get('type'))
                cells['%d:%s' % (col['index'], label)] = rv[ri]
        rows.append({'row': ri, 'disabled': disabled[ri] if ri < len(disabled) else False, 'result': res, 'cells': cells})
    ctx = []
    for c in p.get('ContextData') or []:
        if isinstance(c, dict):
            ctx.append({'type': short(c.get('_type')), **{k: (objref(v) if isinstance(v, str) and "'" in v else v) for k, v in c.items() if k != '_type'}})
        else:
            ctx.append(c)
    t = {'name': obj.name, 'class': obj.short_class, 'output_object_type': objref(p.get('OutputObjectType')),
         'result_type': p.get('ResultType'), 'context': ctx, 'columns': cols, 'results': results, 'rows': rows,
         'fallback_result': p.get('FallbackResult'), 'nested_choosers': [objref(x) for x in (p.get('NestedChoosers') or [])]}
    t['nested'] = [chooser_table(ch, root_path) for ch in obj.children.values() if ch.short_class == 'ChooserTable']
    return t


def collect_refs(v, out):
    if isinstance(v, dict):
        for x in v.values():
            collect_refs(x, out)
    elif isinstance(v, list):
        for x in v:
            collect_refs(x, out)
    elif isinstance(v, str) and ("'/GASPALS/" in v or "'/Game/" in v):
        r = objref(v)
        if r:
            out.add(r)


def post_choosers(ch_raw, assets_rows):
    tables = []
    refs_to = collections.defaultdict(set)
    for t in ch_raw.get('tables', []):
        path = t['_path']
        root = t3d_for(path)
        rec = {'path': path, 'class': t.get('_class'), 'python': {k: v for k, v in t.items() if k not in ('columns_structs', 'context_data', 'results_structs')}}
        if root is None:
            rec['error'] = 'no T3D export'
        else:
            try:
                rec['table'] = chooser_table(root, path)
                rs = set()
                collect_refs(root.props, rs)
                for ch in root.walk():
                    collect_refs(ch.props, rs)
                rec['references'] = sorted(rs)
                for r in rs:
                    refs_to[r].add(path)
            except Exception:
                problem('choosers ' + path, traceback.format_exc())
        rec['referenced_by'] = next((a.get('referenced_by') for a in assets_rows if a['path'] == path), [])
        tables.append(rec)
    out = {'tables': tables, 'proxy_tables': [], 'proxy_assets': [],
           'note': ('Columns/rows come from the T3D export of each ChooserTable (FInstancedStruct arrays are not readable '
                    'through UE Python). Row values are delta-exported: a missing field equals the C++ default of the '
                    'row struct (e.g. FloatRange Min/Max 0 with bNoMin/bNoMax False, Enum Comparison MatchEqual Value 0). '
                    'Enum values are shown as Enum::NewEnumeratorN (Display). No ProxyTable/ProxyAsset assets exist in /GASPALS or /Game.')}
    dump('choosers.json', out)
    return refs_to


# ------------------------------------------------------------------ blueprints
GRAPH_HINT = ('Graph',)
COMPILER_INTERMEDIATE = ('_MERGED',)


def is_graph(o):
    sc = o.short_class
    return sc.endswith('Graph') and 'Node' not in sc and sc not in ('AnimGraphNodeBinding_Base',)


PIN_KEEP = ('PinName', 'Direction', 'PinType.PinCategory', 'PinType.PinSubCategory', 'PinType.PinSubCategoryObject',
            'PinType.ContainerType', 'DefaultValue', 'DefaultObject', 'DefaultTextValue', 'LinkedTo', 'bHidden', 'PinType.bIsReference',
            'PinType.PinValueType', 'SubPins', 'ParentPin', 'PinFriendlyName')


def node_label(o):
    p = o.props
    sc = o.short_class
    def mref(k):
        v = p.get(k)
        if isinstance(v, dict):
            parent = objref(v.get('MemberParent'))
            name = v.get('MemberName')
            return (short(parent) + '::' if parent else ('self::' if as_bool(v.get('bSelfContext')) else '')) + str(name)
        return None
    if sc.startswith('K2Node_CallFunction') or sc in ('K2Node_CallArrayFunction', 'K2Node_CommutativeAssociativeBinaryOperator',
                                                     'K2Node_PromotableOperator', 'K2Node_CallParentFunction', 'K2Node_Message',
                                                     'K2Node_CallMaterialParameterCollectionFunction', 'K2Node_AnimGetter'):
        return 'Call ' + (mref('FunctionReference') or '?')
    if sc in ('K2Node_VariableGet', 'K2Node_VariableSet', 'K2Node_StructMemberGet', 'K2Node_StructMemberSet'):
        return ('Get ' if sc.endswith('Get') else 'Set ') + (mref('VariableReference') or '?')
    if sc in ('K2Node_Event', 'K2Node_ComponentBoundEvent', 'K2Node_InputAction', 'K2Node_EnhancedInputAction'):
        return 'Event ' + str(p.get('CustomFunctionName') or mref('EventReference') or objref(p.get('InputAction')) or p.get('InputActionName') or '')
    if sc == 'K2Node_CustomEvent':
        return 'CustomEvent ' + str(p.get('CustomFunctionName'))
    if sc in ('K2Node_FunctionEntry', 'K2Node_FunctionResult'):
        return sc[8:] + ' ' + str(mref('FunctionReference') or '')
    if sc == 'K2Node_DynamicCast':
        return 'Cast to ' + str(short(objref(p.get('TargetType'))))
    if sc == 'K2Node_MacroInstance':
        mg = p.get('MacroGraphReference') or {}
        return 'Macro ' + str(short(objref(mg.get('MacroGraph'))) if isinstance(mg, dict) else mg)
    if sc in ('K2Node_SwitchEnum', 'K2Node_EnumEquality', 'K2Node_EnumInequality', 'K2Node_GetEnumeratorNameAsString', 'K2Node_EnumLiteral'):
        return sc[8:] + ' ' + str(short(objref(p.get('Enum'))))
    if sc in ('K2Node_BreakStruct', 'K2Node_MakeStruct', 'K2Node_SetFieldsInStruct'):
        return sc[8:] + ' ' + str(short(objref(p.get('StructType'))))
    if sc == 'K2Node_PropertyAccess':
        return 'PropertyAccess ' + str(p.get('Path') or p.get('TextPath') or '')
    if sc == 'K2Node_EvaluateChooser2' or sc.startswith('K2Node_EvaluateChooser'):
        return 'EvaluateChooser ' + str(objref(p.get('Chooser')) or '')
    if sc == 'EdGraphNode_Comment':
        return 'Comment: ' + str(p.get('NodeComment', ''))[:120]
    if sc in ('AnimStateNode', 'AnimStateConduitNode', 'AnimStateAliasNode'):
        return sc + ' ' + str(short(objref(p.get('BoundGraph'))) or p.get('StateAliasName') or o.name)
    if sc.startswith('AnimGraphNode_'):
        node = p.get('Node') if isinstance(p.get('Node'), dict) else {}
        bits = []
        for k in ('Sequence', 'BlendSpace', 'Database', 'Chooser', 'Asset', 'AnimationAsset', 'SlotName', 'Interface', 'Layer',
                  'InstanceClass', 'Tag', 'StateMachineName'):
            if k in node:
                bits.append('%s=%s' % (k, short(objref(node[k])) if isinstance(node[k], str) else node[k]))
        return sc[14:] + (' ' + ', '.join(bits) if bits else '')
    return sc


def collect_graphs(bp_root):
    graphs = []

    def visit(o, parent_graph, owner_node):
        for ch in o.children.values():
            if is_graph(ch):
                graphs.append((ch, parent_graph, owner_node))
                for n in ch.children.values():
                    visit(n, ch, n)
            else:
                visit(ch, parent_graph, owner_node)
    visit(bp_root, None, None)
    return graphs


def graph_detail(g, parent_graph, owner_node, bp_name):
    nodes = list(g.children.values())
    pin_index = {}
    for n in nodes:
        for kind, pv in n.custom:
            if kind == 'Pin' and isinstance(pv, dict):
                pin_index[(n.name, pv.get('PinId'))] = pv.get('PinName')
    out_nodes = []
    for n in nodes:
        pins = []
        for kind, pv in n.custom:
            if kind != 'Pin' or not isinstance(pv, dict):
                continue
            pin = {'name': pv.get('PinName'), 'dir': 'out' if pv.get('Direction') == 'EGPD_Output' else 'in',
                   'type': pv.get('PinType.PinCategory')}
            sub = pv.get('PinType.PinSubCategory')
            if sub:
                pin['subtype'] = sub
            so = pv.get('PinType.PinSubCategoryObject')
            if so and so != 'None':
                pin['type_object'] = objref(so)
            if pv.get('PinType.ContainerType') not in (None, 'None'):
                pin['container'] = pv.get('PinType.ContainerType')
            for k in ('DefaultValue', 'DefaultObject'):
                if pv.get(k) not in (None, '', 'None'):
                    pin['default'] = pv.get(k) if k == 'DefaultValue' else objref(pv.get(k))
            if pv.get('DefaultTextValue') not in (None, ''):
                pin['default_text'] = text_of(pv.get('DefaultTextValue'))
            links = []
            for lt in pv.get('LinkedTo') or []:
                parts = str(lt).split(' ')
                if len(parts) == 2:
                    links.append('%s.%s' % (parts[0], pin_index.get((parts[0], parts[1]), parts[1])))
            if links:
                pin['links'] = links
            if as_bool(pv.get('bHidden')):
                pin['hidden'] = True
            if pv.get('PinFriendlyName'):
                pin['friendly'] = text_of(pv.get('PinFriendlyName'))
            pins.append(pin)
        props = {k: v for k, v in n.props.items() if k not in ('NodePosX', 'NodePosY', 'NodeGuid', 'ShowPinForProperties', 'NodeWidth',
                                                              'NodeHeight', 'ErrorType', 'ErrorMsg', 'bCommentBubblePinned',
                                                              'bCommentBubbleVisible', 'Binding')}
        shown = [x.get('PropertyName') for x in (n.props.get('ShowPinForProperties') or []) if isinstance(x, dict) and as_bool(x.get('bShowPin'))]
        nd = {'name': n.name, 'class': n.short_class, 'label': node_label(n),
              'pos': [int(as_float(n.props.get('NodePosX'), 0)), int(as_float(n.props.get('NodePosY'), 0))],
              'props': props, 'pins': pins}
        if shown:
            nd['pins_shown_for_properties'] = shown
        sub = [c.name for c in n.children.values() if is_graph(c)]
        if sub:
            nd['sub_graphs'] = sub
        out_nodes.append(nd)
    gd = {'name': g.name, 'class': g.short_class, 'object_path': g.path(), 'parent_graph': parent_graph.name if parent_graph else None,
          'owner_node': owner_node.name if owner_node else None, 'node_count': len(out_nodes), 'nodes': out_nodes}
    if g.name.endswith(COMPILER_INTERMEDIATE) or g.name.startswith('ExecuteUbergraph_') or \
            (parent_graph is not None and (parent_graph.name.startswith('ExecuteUbergraph_') or parent_graph.name.endswith('_MERGED'))):
        gd['compiler_intermediate'] = True
    # state machine summary
    if g.short_class == 'AnimationStateMachineGraph':
        states = {}
        for n in g.children.values():
            if n.short_class in ('AnimStateNode', 'AnimStateConduitNode', 'AnimStateAliasNode', 'AnimStateEntryNode'):
                states[n.name] = short(objref(n.props.get('BoundGraph'))) or n.props.get('StateAliasName') or n.name
        trans = []
        for n in g.children.values():
            if n.short_class != 'AnimStateTransitionNode':
                continue
            frm = to = None
            for kind, pv in n.custom:
                if kind == 'Pin' and isinstance(pv, dict):
                    for lt in pv.get('LinkedTo') or []:
                        node = str(lt).split(' ')[0]
                        if pv.get('PinName') == 'In':
                            frm = states.get(node, node)
                        elif pv.get('PinName') == 'Out':
                            to = states.get(node, node)
            tr = {'node': n.name, 'from': frm, 'to': to}
            for k in ('PriorityOrder', 'CrossfadeDuration', 'BlendMode', 'LogicType', 'Bidirectional', 'bAutomaticRuleBasedOnSequencePlayerInState',
                      'AutomaticRuleTriggerTime', 'TransitionStart', 'TransitionEnd', 'TransitionInterrupt', 'BlendProfile', 'CustomBlendCurve',
                      'SyncGroupNameToRequireValidMarkersRule', 'bDisabled'):
                if k in n.props:
                    tr[k] = n.props[k]
            rule = next((c for c in n.children.values() if is_graph(c)), None)
            if rule is not None:
                tr['rule_nodes'] = [node_label(x) for x in rule.children.values() if x.short_class not in ('EdGraphNode_Comment', 'K2Node_Knot')]
            trans.append(tr)
        gd['state_machine'] = {'states': states, 'transitions': trans,
                               'defaults_note': 'transition fields missing in T3D = UAnimStateTransitionNode constructor defaults: CrossfadeDuration=0.2, BlendMode=HermiteCubic, PriorityOrder=1, LogicType=TLT_StandardBlend'}
    return gd


def bp_variables(root, cdo_root):
    out = []
    cdo = cdo_root.props if cdo_root is not None else {}
    for v in root.props.get('NewVariables') or []:
        if not isinstance(v, dict):
            continue
        vt = v.get('VarType') if isinstance(v.get('VarType'), dict) else {}
        md = {}
        for m in v.get('MetaDataArray') or []:
            if isinstance(m, dict):
                md[m.get('DataKey')] = m.get('DataValue')
        name = v.get('VarName')
        rec = {'name': name, 'friendly_name': v.get('FriendlyName'), 'category': text_of(v.get('Category')),
               'type': vt.get('PinCategory'), 'subtype': vt.get('PinSubCategory') or None,
               'type_object': objref(vt.get('PinSubCategoryObject')), 'container': vt.get('ContainerType'),
               'value_type': vt.get('PinValueType') or None, 'property_flags': v.get('PropertyFlags'),
               'rep_notify': v.get('RepNotifyFunc'), 'metadata': md or None, 'default_value_t3d': v.get('DefaultValue')}
        if name in cdo:
            rec['cdo_value'] = resolve_enum_strings(cdo[name])
        out.append({k: x for k, x in rec.items() if x not in (None, '', [], {})})
    return out


def bp_functions(graphs_detail, root):
    fnames = [short(objref(x)) for x in (root.props.get('FunctionGraphs') or [])]
    out = []
    for g in graphs_detail:
        if g['name'] not in fnames or g.get('compiler_intermediate') or g['parent_graph']:
            continue
        f = {'name': g['name'], 'graph_class': g['class'], 'inputs': [], 'outputs': [], 'node_count': g['node_count']}
        for n in g['nodes']:
            if n['class'] == 'K2Node_FunctionEntry':
                f['inputs'] = [{'name': p['name'], 'type': p.get('type'), 'type_object': p.get('type_object'), 'subtype': p.get('subtype')}
                               for p in n['pins'] if p['dir'] == 'out' and p.get('type') != 'exec']
                md = n['props'].get('MetaData') if isinstance(n['props'].get('MetaData'), dict) else {}
                f['metadata'] = {k: text_of(v) for k, v in md.items()} if md else None
                f['extra_flags'] = n['props'].get('ExtraFlags')
                try:
                    fl = int(n['props'].get('ExtraFlags') or 0)
                    f['pure'] = bool(fl & 0x10000000)
                    f['const'] = bool(fl & 0x00400000)
                    f['thread_safe'] = str(md.get('bThreadSafe', '')).lower() == 'true' or 'BlueprintThreadSafe' in json.dumps(md)
                except Exception:
                    pass
                lv = n['props'].get('LocalVariables')
                if lv:
                    f['local_variables'] = [x.get('VarName') for x in lv if isinstance(x, dict)]
            if n['class'] == 'K2Node_FunctionResult' and not f['outputs']:
                f['outputs'] = [{'name': p['name'], 'type': p.get('type'), 'type_object': p.get('type_object'), 'subtype': p.get('subtype')}
                                for p in n['pins'] if p['dir'] == 'in' and p.get('type') != 'exec']
        f['calls'] = sorted({n['label'][5:] for n in g['nodes'] if n['label'].startswith('Call ')})
        out.append(f)
    return out


def components_from_class(cls_root):
    comps = []
    if cls_root is None:
        return comps
    for o in cls_root.walk():
        if o is cls_root:
            continue
        sc = o.short_class
        if sc in ('Function', 'DelegateFunction', 'SparseDelegateFunction'):
            continue
        if o.name.endswith('_GEN_VARIABLE') or sc.endswith('Component') or sc in ('SCS_Node', 'SimpleConstructionScript', 'TimelineTemplate',
                                                                                  'InheritableComponentHandler'):
            comps.append({'name': o.name, 'class': sc, 'class_path': o.cls, 'props': o.props})
    return comps


def post_blueprints(bp_raw, assets_rows):
    os.makedirs(BP_DIR, exist_ok=True)
    index = []
    raw_by_path = {b['path']: b for b in (bp_raw or {}).get('blueprints', [])}
    asset_by_path = {a['path']: a for a in assets_rows}
    for path, braw in sorted(raw_by_path.items()):
        try:
            root = t3d_for(path)
            if root is None:
                index.append({'path': path, 'class': braw.get('class'), 'error': 'no T3D export'})
                continue
            cdo = t3d_for(path, 'cdo')
            cls_root = t3d_for(path, 'class')
            graphs = collect_graphs(root)
            gdet = [graph_detail(g, pg, on, root.name) for g, pg, on in graphs]
            variables = bp_variables(root, cdo)
            functions = bp_functions(gdet, root)
            node_hist = collections.Counter()
            calls = collections.Counter()
            for g in gdet:
                if g.get('compiler_intermediate'):
                    continue
                for n in g['nodes']:
                    node_hist[n['class']] += 1
                    if n['label'].startswith('Call '):
                        calls[n['label'][5:]] += 1
            ifaces = []
            for it in root.props.get('ImplementedInterfaces') or []:
                if isinstance(it, dict):
                    ifaces.append(objref(it.get('Interface')))
            cdo_props = {}
            if cdo is not None:
                varnames = {v.get('name') for v in variables}
                cdo_props = {k: resolve_enum_strings(v) for k, v in cdo.props.items() if k not in varnames}
            doc = {
                'path': path, 'class': braw.get('class'), 'parent_class': objref(root.props.get('ParentClass')),
                'generated_class': braw.get('generated_class'), 'registry_tags': braw.get('registry_tags'),
                'target_skeleton': objref(root.props.get('TargetSkeleton')), 'interfaces': ifaces,
                'description': text_of(root.props.get('BlueprintDescription')),
                'variables': variables, 'functions': functions,
                'graphs': gdet,
                'cdo_other_overrides': cdo_props,
                'components_and_templates': components_from_class(cls_root),
                'cdo_default_subobjects': [{'name': c.name, 'class': c.short_class, 'props': c.props} for c in (cdo.children.values() if cdo is not None else [])],
                'node_class_histogram': dict(node_hist.most_common()),
                'called_functions': dict(calls.most_common()),
                'dependencies': (asset_by_path.get(path) or {}).get('dependencies'),
                'referenced_by': (asset_by_path.get(path) or {}).get('referenced_by'),
                'sources': {'t3d': flat(path) + '.t3d', 'cdo_t3d': flat(path) + '.cdo.t3d' if cdo is not None else None,
                            'class_t3d': flat(path) + '.class.t3d' if cls_root is not None else None},
                'notes': ['T3D is a delta export (values equal to defaults are omitted)',
                          'graphs with compiler_intermediate=true (_MERGED, ExecuteUbergraph_*) are compile-on-load artifacts, not authored graphs',
                          'pin links are NodeName.PinName inside the same graph'],
            }
            fn = flat(path) + '.json'
            dump(os.path.join('blueprints', fn), doc, indent=None)
            index.append({
                'path': path, 'class': braw.get('class'), 'parent_class': doc['parent_class'], 'file': 'blueprints\\' + fn,
                'variables': [v.get('name') for v in variables], 'functions': [f['name'] for f in functions],
                'graphs': [{'name': g['name'], 'class': g['class'], 'nodes': g['node_count'], 'parent': g['parent_graph']}
                           for g in gdet if not g.get('compiler_intermediate')],
                'compiler_intermediate_graphs': sum(1 for g in gdet if g.get('compiler_intermediate')),
                'node_count': sum(node_hist.values()), 'interfaces': ifaces,
                'state_machines': [{'graph': g['name'], 'states': list(g['state_machine']['states'].values()),
                                    'transitions': len(g['state_machine']['transitions'])} for g in gdet if 'state_machine' in g and not g.get('compiler_intermediate')],
                'components': [c['name'] for c in doc['components_and_templates']],
            })
        except Exception:
            problem('blueprints ' + path, traceback.format_exc())
            index.append({'path': path, 'class': braw.get('class'), 'error': 'post-processing failed'})
    return index


def post_structs(bp_raw):
    out = []
    for s in (bp_raw or {}).get('structs', []):
        root = t3d_for(s['path'])
        fields = []
        if root is not None:
            ed = next((c for c in root.children.values() if c.short_class == 'UserDefinedStructEditorData'), None)
            for v in (ed.props.get('VariablesDescriptions') if ed is not None else None) or []:
                if isinstance(v, dict):
                    fields.append({'name': v.get('FriendlyName'), 'internal_name': v.get('VarName'), 'type': v.get('Category'),
                                   'subtype': v.get('SubCategory'), 'type_object': v.get('SubCategoryObject'),
                                   'container': v.get('ContainerType'), 'default': resolve_enum_strings(v.get('DefaultValue')),
                                   'tooltip': v.get('ToolTip')})
        out.append({'path': s['path'], 'name': short(s['path']), 'fields': fields, 'py_type': s.get('py_type'),
                    'py_fields': s.get('py_fields')})
    return out


def post_data(bp_raw, assets_rows):
    """Data assets (BP data-asset instances), input, cameras, curves, other data objects."""
    out = {'data_assets': [], 'input_actions': [], 'input_mapping_contexts': [], 'cameras': [], 'curves': [], 'data_tables': [],
           'other': []}
    for d in (bp_raw or {}).get('data_assets', []) + (bp_raw or {}).get('other_data', []) + (bp_raw or {}).get('data_tables', []):
        path = d['_path']
        cls = d['_class']
        root = t3d_for(path)
        rec = {'path': path, 'class': cls, 'python': d}
        if root is not None:
            rec['t3d_props'] = resolve_enum_strings(root.props)
            if root.children:
                rec['t3d_subobjects'] = [{'name': c.name, 'class': c.short_class, 'props': c.props} for c in root.walk() if c is not root]
        if cls.endswith('_C') or 'DataAsset' in cls:
            out['data_assets'].append(rec)
        elif cls == 'InputAction':
            out['input_actions'].append(rec)
        elif cls == 'InputMappingContext':
            out['input_mapping_contexts'].append(rec)
        elif cls.startswith('Camera'):
            out['cameras'].append(rec)
        elif cls.startswith('Curve'):
            if root is not None:
                fc = root.props.get('FloatCurve') or {}
                keys = []
                for k in (fc.get('Keys') if isinstance(fc, dict) else None) or []:
                    k = k if isinstance(k, dict) else {}
                    keys.append({'time': as_float(k.get('Time'), 0.0), 'value': as_float(k.get('Value'), 0.0),
                                 'interp': k.get('InterpMode', 'RCIM_Linear'), 'tangent_mode': k.get('TangentMode', 'RCTM_Auto'),
                                 'arrive': as_float(k.get('ArriveTangent'), 0.0), 'leave': as_float(k.get('LeaveTangent'), 0.0)})
                rec['keys'] = keys
            out['curves'].append(rec)
        elif 'DataTable' in cls:
            out['data_tables'].append(rec)
        else:
            out['other'].append(rec)
    out['note'] = ('python = UE Python reflection dump; t3d_props = T3D (delta) export incl. Blueprint-defined fields of data-asset '
                   'instances (e.g. PDA_OverlayPose: Overlay Animation Blueprint, Static Mesh, Left Hand, Socket Name).')
    return out


# ------------------------------------------------------------------ skeletons / rigs
def post_skeletons(sk_raw, anims_rows):
    for s in sk_raw.get('skeletons', []):
        root = t3d_for(s['path'])
        if root is None:
            continue
        p = root.props
        s['virtual_bones'] = [{'name': v.get('VirtualBoneName'), 'source': v.get('SourceBoneName'), 'target': v.get('TargetBoneName')}
                              for v in (p.get('VirtualBones') or []) if isinstance(v, dict)]
        socks = []
        for c in root.children.values():
            if c.short_class == 'SkeletalMeshSocket':
                cp = c.props
                socks.append({'name': cp.get('SocketName'), 'bone': cp.get('BoneName'), 'location': cp.get('RelativeLocation'),
                              'rotation': cp.get('RelativeRotation'), 'scale': cp.get('RelativeScale'),
                              'force_always_animated': cp.get('bForceAlwaysAnimated')})
        s['sockets'] = socks
        s['slot_groups'] = p.get('SlotGroups')
        s['blend_profiles'] = [{'name': c.name, 'mode': c.props.get('Mode', 'WeightFactor'),
                                'entries': [{'bone': (e.get('BoneReference') or {}).get('BoneName'), 'scale': as_float(e.get('BlendScale'))}
                                            for e in (c.props.get('ProfileEntries') or []) if isinstance(e, dict)]}
                               for c in root.children.values() if c.short_class == 'BlendProfile']
        s['retarget_sources'] = p.get('AnimRetargetSources')
        s['compatible_skeletons'] = p.get('CompatibleSkeletons')
        s['t3d_other'] = {k: v for k, v in p.items() if k not in ('BoneTree', 'VirtualBones', 'Sockets', 'BlendProfiles', 'SlotGroups',
                                                                 'AnimRetargetSources', 'CompatibleSkeletons')}
        s['notes'] = ['bones/ref pose from AnimPoseExtensions.get_reference_pose (includes virtual bones, whose parent lookup fails)',
                      'parents from AnimationLibrary.find_bone_path_to_root using one AnimSequence of that skeleton',
                      'blend profile mode missing in T3D = WeightFactor (UBlendProfile constructor default)',
                      'sockets/virtual_bones/blend_profiles/slot_groups from T3D of the Skeleton; sockets_from_ref_pose from AnimPoseExtensions']
    rigs = []
    for r in sk_raw.get('rigs', []):
        path = r['_path']
        root = t3d_for(path)
        rec = {'path': path, 'class': r['_class'], 'python': r}
        if root is not None:
            p = root.props
            rec['t3d_props'] = p
            rec['t3d_subobjects'] = [{'name': c.name, 'class': c.short_class, 'props': c.props} for c in root.walk() if c is not root]
            if r['_class'] == 'IKRigDefinition':
                skel = p.get('Skeleton') if isinstance(p.get('Skeleton'), dict) else {}
                names = skel.get('BoneNames') or []
                parents = skel.get('ParentIndices') or []
                rec['ik_skeleton'] = [{'bone': n, 'parent': names[int(parents[i])] if i < len(parents) and str(parents[i]).lstrip('-').isdigit() and int(parents[i]) >= 0 else None}
                                      for i, n in enumerate(names)]
                rd = p.get('RetargetDefinition') if isinstance(p.get('RetargetDefinition'), dict) else {}
                rec['retarget_chains'] = [{'chain': c.get('ChainName'), 'start': (c.get('StartBone') or {}).get('BoneName'),
                                           'end': (c.get('EndBone') or {}).get('BoneName'), 'ik_goal': c.get('IKGoalName')}
                                          for c in rd.get('BoneChains') or [] if isinstance(c, dict)]
                rec['retarget_root'] = rd.get('RootBone')
                rec['goals'] = [{'name': c.props.get('GoalName'), 'bone': c.props.get('BoneName')} for c in root.children.values()
                                if c.short_class == 'IKRigEffectorGoal']
            if r['_class'] == 'IKRetargeter':
                rec['source_ik_rig'] = objref(p.get('SourceIKRigAsset'))
                rec['target_ik_rig'] = objref(p.get('TargetIKRigAsset'))
                rec['ops'] = [{'type': short(o.get('_type')), 'name': o.get('Name'), 'parent_op': o.get('ParentOpName')}
                              for o in (p.get('RetargetOps') or []) if isinstance(o, dict)]
        rigs.append(rec)
    sk_raw['rigs'] = rigs
    # curve usage across all anims, grouped by prefix (layering curves etc.)
    usage = collections.defaultdict(list)
    for a in anims_rows:
        for c in a.get('curves') or []:
            usage[c].append(a['path'])
    groups = collections.defaultdict(dict)
    for c, lst in sorted(usage.items()):
        pref = c.split('_')[0] if '_' in c else c
        groups[pref][c] = {'anim_count': len(lst), 'examples': lst[:5]}
    sk_raw['curves_used_by_anims'] = dict(groups)
    sk_raw['layering_curves'] = sorted(c for c in usage if c.lower().startswith(('layering_', 'enable_', 'mask_', 'hipsdirection', 'pose_')))
    dump('skeletons.json', sk_raw)


# ------------------------------------------------------------------ config / tags
def post_config():
    cfg_out = os.path.join(OUT, 'config')
    copied = []
    srcs = [(os.path.join(PROJECT, 'Config'), 'project'), (os.path.join(PROJECT, 'Plugins', 'GASPALS', 'Config'), 'plugin')]
    for src, tag in srcs:
        for dp, _, files in os.walk(src):
            for fn in files:
                if not fn.lower().endswith('.ini'):
                    continue
                rel = os.path.relpath(os.path.join(dp, fn), src)
                dst = os.path.join(cfg_out, tag, rel)
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.copyfile(os.path.join(dp, fn), dst)
                copied.append(os.path.join(tag, rel))
    tags = {}
    for rel in copied:
        p = os.path.join(cfg_out, rel)
        with open(p, encoding='utf-8-sig', errors='replace') as f:
            for line in f:
                line = line.strip()
                if 'GameplayTagList=' in line:
                    v = t3d.parse_value(line.split('=', 1)[1])
                    if isinstance(v, dict) and v.get('Tag'):
                        tags.setdefault(v['Tag'], {'comment': v.get('DevComment'), 'sources': []})['sources'].append(rel)
    tree = {}
    for t in sorted(tags):
        cur = tree
        for part in t.split('.'):
            cur = cur.setdefault(part, {})
    dump('gameplay_tags.json', {'tags': tags, 'tree': tree, 'config_files': copied})
    return copied, tags


# ------------------------------------------------------------------ README
def write_readme(stats):
    L = []
    w = L.append
    w('GASP-ALS read-only extraction (UTPP)')
    w('=====================================')
    w('Source project : the GASP-ALS project (GASPALS.uproject, UE 5.8; content in plugin GASPALS mounted at /GASPALS/; /Game/ is empty)')
    w('Upstream        : https://github.com/PolygonHive/GASP-ALS')
    w('Engine         : %s' % stats.get('engine'))
    w('Produced by    : Source\\ue\\sourcepack_extract.py (UnrealEditor-Cmd -run=pythonscript, read-only: loads assets, never saves),')
    w('                 Source\\tools\\uasset\\uasset_names.py (offline .uasset header reader, system python),')
    w('                 Source\\tools\\uasset\\sourcepack_post.py + t3d.py (offline T3D parser / merger, system python).')
    w('UE log         : Research\\logs\\sourcepack-extract-ue.log')
    w('')
    w('FILES')
    w('-----')
    for name, desc in stats['files']:
        p = os.path.join(OUT, name)
        size = os.path.getsize(p) if os.path.isfile(p) else sum(os.path.getsize(os.path.join(dp, f)) for dp, _, fs in os.walk(p) for f in fs) if os.path.isdir(p) else 0
        w('%-22s %10.1f KB  %s' % (name, size / 1024.0, desc))
    w('')
    w('COUNTS')
    w('------')
    for k, v in stats['counts'].items():
        w('%-40s %s' % (k, v))
    w('')
    w('HOW GRAPH / TABLE DATA WAS OBTAINED')
    w('-----------------------------------')
    for line in stats['method']:
        w('- ' + line)
    w('')
    w('FAILURES')
    w('--------')
    if not stats['failures']:
        w('UE side: no asset failed to load or export (ue_raw\\failures.json is empty).')
    for f in stats['failures']:
        w('UE  %s %s: %s' % (f.get('phase'), f.get('asset'), str(f.get('error'))[:300].replace('\n', ' | ')))
    if not PROBLEMS:
        w('Post-processing: no problems.')
    for p in PROBLEMS:
        w('POST %s: %s' % (p['where'], p['error'][:300].replace('\n', ' | ')))
    w('')
    w('UE LOG WARNINGS (unique, from the extraction run)')
    w('-------------------------------------------------')
    for line in stats.get('ue_warnings', []):
        w('  ' + line)
    w('')
    w('LIMITATIONS')
    w('-----------')
    for line in stats['limitations']:
        w('- ' + line)
    with open(os.path.join(OUT, 'README.txt'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')


def ue_warnings():
    p = os.path.join(project_root(), 'Research', 'logs', 'sourcepack-extract-ue.log')
    out = []
    if not os.path.exists(p):
        return ['(log not found)']
    lines = open(p, encoding='utf-8', errors='replace').read().splitlines()
    try:
        i = next(k for k, l in enumerate(lines) if 'Warning/Error Summary' in l)
        for l in lines[i + 2:i + 40]:
            if 'LogInit: Display:' in l:
                t = l.split('LogInit: Display:', 1)[1].strip()
                if t and not t.startswith('---'):
                    out.append(t[:300])
            if 'Execution of commandlet' in l:
                out.append(l.strip())
                break
    except StopIteration:
        out.append('(no summary block)')
    return [x for x in out if x]


def main():
    assets_raw = load_raw('assets.json')
    anims_raw = load_raw('anims.json')
    ps_raw = load_raw('posesearch.json')
    ch_raw = load_raw('choosers.json')
    sk_raw = load_raw('skeletons.json')
    bp_raw = load_raw('blueprints_ue.json')
    failures = load_raw('failures.json') or []
    curves_raw = load_raw('anim_curves.json')

    enums = build_enums(bp_raw)
    assets_rows = post_assets(assets_raw)
    refs_to = post_choosers(ch_raw, assets_rows)
    anims_rows = post_anims(anims_raw, 'compared by asset short name with Animations\\sourcepack\\SourcePack_export\\inventory.json (718 sequences) '
                                       'and manifest.json; gasp_same_relative_path compares the /Game/ path mapped to /GASPALS/')
    anims_by_path = {a['path']: a for a in anims_rows}
    post_posesearch(ps_raw, anims_by_path, refs_to)
    if curves_raw is not None:
        dump('anim_curves.json', curves_raw, indent=None)
    bp_index = post_blueprints(bp_raw, assets_rows)
    structs = post_structs(bp_raw)
    dump('blueprints.json', {
        'blueprints': bp_index, 'enums': enums, 'structs': structs,
        'note': ('Per-Blueprint detail (variables with CDO defaults, functions with signatures, every graph with nodes, pins and '
                 'links, state machines, components) is in blueprints\\<asset>.json. Graph data comes from T3D exports '
                 '(t3d\\*.t3d); vocabulary fallback for every package in uasset_names\\.')})
    data = post_data(bp_raw, assets_rows)
    dump('data.json', data)
    post_skeletons(sk_raw, anims_rows)
    copied, tags = post_config()

    n_new = sum(1 for a in anims_rows if not a.get('in_gasp_export'))
    n_same = len(anims_rows) - n_new
    seq_new = sum(1 for a in anims_rows if a['class'] == 'AnimSequence' and not a.get('in_gasp_export'))
    seq_all = sum(1 for a in anims_rows if a['class'] == 'AnimSequence')
    by_cls = collections.Counter(a['class'] for a in anims_rows)
    t3d_files = os.listdir(T3D_DIR)
    names_idx = json.load(open(os.path.join(NAMES_DIR, 'index.json'), encoding='utf-8'))['packages']
    stats = {
        'engine': assets_raw.get('engine'),
        'files': [
            ('assets.json', 'every asset under /GASPALS and /Game: path, class, folder, .uasset file + size, registry dependencies, referenced_by, links to t3d/uasset_names'),
            ('anims.json', 'every AnimSequence/AnimMontage/BlendSpace/BlendSpace1D/PoseAsset: skeleton, length, frames, fps, root motion, additive, curves, notifies (class, time, duration, track, props), sync markers, montage sections/slots, blend space axes/samples, GASP-export comparison'),
            ('anim_curves.json', 'float curve keys (times/values) per animation'),
            ('posesearch.json', 'PoseSearchSchemas (channels incl. bones/samples/weights/sub-channels), PoseSearchDatabases (settings + every entry with sampling range, mirroring, flags, estimated poses), normalization sets, anim->database index'),
            ('choosers.json', 'ChooserTables (incl. nested tables): context, columns (type, bound property, enum), rows with values and results, references'),
            ('blueprints.json', 'index of all Blueprints/AnimBPs/Widget/ControlRig BPs (variables, functions, graphs, state machines, components) + enums + structs'),
            ('blueprints', 'per-Blueprint full detail JSON (graphs, nodes, pins, links, CDO defaults)'),
            ('data.json', 'data-asset instances (overlay definitions, foley bank), input actions/mapping context, cameras (Gameplay Cameras rigs), curves, other data objects'),
            ('skeletons.json', 'skeletons (bones, parents, ref pose local/world, sockets, virtual bones, blend profiles, slot groups, curve metadata), mannequin skeletal-mesh sockets, IK rigs (skeleton, chains, goals), IK retargeters (ops), mirror table, physics assets, curve usage/layering curves'),
            ('gameplay_tags.json', 'gameplay tags parsed from project + plugin Config ini files'),
            ('config', 'verbatim copies of project Config\\*.ini and plugin Config\\*.ini'),
            ('t3d', 'raw T3D text exports (asset, .cdo = generated-class default object, .class = generated class) of all logic assets'),
            ('uasset_names', 'offline header dump per logic package (name table, imports, exports, node classes, functions) + index.json for all 2090 packages'),
            ('ue_raw', 'unmodified UE-side JSON (input of the post-processor) incl. failures.json and t3d_index.json'),
            ('_progress.log', 'progress log of the UE-side script'),
        ],
        'counts': collections.OrderedDict([
            ('assets (registry)', assets_raw['count']),
            ('anim assets total', len(anims_rows)),
            ('  by class', dict(by_cls)),
            ('  same name in SourcePack_export / not in it', '%d / %d' % (n_same, n_new)),
            ('  AnimSequence total / not in SourcePack_export', '%d / %d' % (seq_all, seq_new)),
            ('  in original GASP project / NEW in GASP-ALS', '%d / %d' % (
                sum(1 for a in anims_rows if a.get('in_original_gasp_project')),
                sum(1 for a in anims_rows if not a.get('in_original_gasp_project')))),
            ('  NEW in GASP-ALS by class', dict(collections.Counter(a['class'] for a in anims_rows if not a.get('in_original_gasp_project')))),
            ('PoseSearchDatabases', len(ps_raw['databases'])),
            ('PoseSearchSchemas', len(ps_raw['schemas'])),
            ('PoseSearchNormalizationSets', len(ps_raw['normalization_sets'])),
            ('ChooserTables (top level)', len(ch_raw.get('tables', []))),
            ('ProxyTable / ProxyAsset', 0),
            ('Blueprints (all BP classes)', len(bp_index)),
            ('Blueprints with full graph detail', sum(1 for b in bp_index if 'error' not in b)),
            ('enums / structs', '%d / %d' % (len(enums), len(structs))),
            ('data assets / input actions / cameras / curves', '%d / %d / %d / %d' % (len(data['data_assets']), len(data['input_actions']), len(data['cameras']), len(data['curves']))),
            ('skeletons / IK rigs+retargeters+misc', '%d / %d' % (len(sk_raw.get('skeletons', [])), len(sk_raw.get('rigs', [])))),
            ('skeleton curve metadata entries', {short(k['path']): len(k.get('curve_metadata') or []) for k in sk_raw.get('skeletons', [])}),
            ('T3D files', len(t3d_files)),
            ('uasset_names packages parsed / JSON written', '%d / %d' % (len(names_idx), sum(1 for r in names_idx if r.get('json')))),
            ('gameplay tags', len(tags)),
            ('UE failures / post problems', '%d / %d' % (len(failures), len(PROBLEMS))),
        ]),
        'method': [
            'Animation, PoseSearch (schemas, channels, database settings, entry asset list), skeleton bones/ref pose, sockets, curve keys: UE Python API (reflection over editor properties + AnimationLibrary/AnimPoseExtensions).',
            'PoseSearchDatabase entry flags (SamplingRange, MirrorOption, bEnabled, bDisableReselection, BranchInId, blend-space sampling): T3D (DatabaseAnimationAssets is a non-editable UPROPERTY, not visible to Python in 5.8).',
            'ChooserTable columns/rows/results/nested tables: T3D (FInstancedStruct arrays are opaque to Python).',
            'Blueprint graphs, nodes, pins, links, variables, functions, state machines: T3D of the Blueprint asset (UObjectExporterT3D exports all inner EdGraph/EdGraphNode objects and their pins via ExportCustomProperties); variable defaults from T3D of the generated-class CDO; SCS component templates/timelines from T3D of the generated class.',
            'Enums: value order decoded offline from the UEnum Names array in the .uasset export data (uasset_names enum_entries), display names from T3D DisplayNameMap (UE 5.8 Python exposes neither: KismetNodeHelperLibrary is not script-exposed and get_type_from_enum returns plain EnumBase for user enums). Structs: T3D UserDefinedStructEditorData.',
            'Animation set comparison: in_gasp_export = same name in Animations\\sourcepack\\SourcePack_export (only the 35 Dense databases of the original GASP were exported there); in_original_gasp_project = same file name anywhere in the original GASP project content (directory listing only); false = NEW in GASP-ALS.',
            'Fallback / cross-check: uasset_names\\ (offline parser of the package name/import/export tables) gives node classes, function names, variable/pin names and referenced assets for every Blueprint-like package even without the engine.',
        ],
        'failures': failures,
        'ue_warnings': ue_warnings(),
        'limitations': [
            'T3D is a DELTA export: properties equal to the class/struct default are omitted. Defaults used by this post-processor are stated in the *_note fields; anything else missing = engine default.',
            'Graph data is the editor graph (authoring view); compile-on-load created transient *_MERGED / ExecuteUbergraph_* graphs inside the loaded Blueprints - they are exported too but flagged compiler_intermediate and excluded from summaries.',
            'Compiled bytecode is not exported; behaviour must be read from the node graphs (pins + links).',
            'PoseSearch pose counts are estimates (sample rate x sampled seconds, mirroring, blend-space samples); the real search index (PCA/KD-tree, pruning, ExcludeFromDatabaseParameters) is not exported.',
            'Bone parents come from FindBonePathToRoot via one AnimSequence of the skeleton; skeletons without animations (MetaHuman face/clothing, props) have no parents; virtual bones are listed but have no parent lookup (warnings in the UE log).',
            'T3D was skipped for MetaHuman ControlRigBlueprints and MetaHuman PoseAssets (face board / RBF correctives, irrelevant for UTPP); they are still listed in assets.json/anims.json and covered by uasset_names.',
            'No FBX/bone-track data is exported here (only metadata and curves); animation transforms need a separate export (Source\\ue\\export_sourcepack.py pattern).',
            'Pin tooltips are English (-culture=en); localized text appears as NSLOCTEXT macro objects ({_macro,_args,_text}).',
            'Names are case-insensitive FNames with case-preserving display: casing can differ between sources (e.g. skeleton ref pose reports "Pelvis" while IK rig/T3D use "pelvis"; curves exist as both Layering_* and layering_*). Compare case-insensitively.',
            'Skeleton curve metadata (AnimationLibrary.get_curve_meta_data_names) is empty for several skeletons incl. SK_UEFN_Mannequin (see counts); curve names used by the animations are listed in skeletons.json curves_used_by_anims instead.',
            'T3D values are strings (numbers/bools not converted); object references are kept as Class\'Path\' strings or resolved to paths in the summaries.',
            'The DDC/asset-registry caches made the run fast (39 s); a cold DDC would only change the duration, not the output.',
            'Running the editor writes engine caches/logs into the GASP-ALS project folders Saved\\ and Intermediate\\ (AutoSDKInfo logs, CrashReportClient.ini, EditorPerProjectUserSettings.ini, PipInstall, cached asset registry) and the shared DDC; Plugins\\ and Config\\ were verified unchanged (mtime+size snapshot before/after).',
            'GASP-ALS .git\\FETCH_HEAD and refs/remotes/origin/HEAD got a new timestamp (2026-10-12 23:40:12) during the run; origin unknown - this extractor issued no git command and UE logged "Revision control is disabled"; no tracked content changed.',
            'Extracted\\sourcepack\\sourcepack\\mm_tagged_props\\ was NOT produced by this extractor (written by another process during the same session).',
        ],
    }
    write_readme(stats)
    print(json.dumps({'counts': stats['counts'], 'problems': len(PROBLEMS)}, indent=1, default=str))
    if PROBLEMS:
        print(json.dumps(PROBLEMS[:20], indent=1))


if __name__ == '__main__':
    main()
