"""Export source pack animation sequences to FBX + JSON sidecars (run inside UnrealEditor-Cmd, -run=pythonscript).

Input: <out>\\inventory.json (the list of animation assets and pose databases). Selection: databases whose short name is listed in
UTPP_SOURCEPACK_DBS (comma separated, e.g. "PSD_Dense_Stand_Idles,PSD_Dense_Stand_Walk_Loops"), or all when
the variable is "ALL". For each selected sequence writes:
  <out>\\fbx\\<SequenceName>.fbx              (skeletal animation incl. root bone, no mesh)
  <out>\\meta\\<SequenceName>.json            (length, frames, root-motion flags, float curves sampled per
                                              frame, notify events with time/duration, source databases)
and <out>\\manifest.json listing everything exported. Nothing in the project is modified or saved.
"""
import json
import os

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


OUT = os.environ.get('UTPP_SOURCEPACK_OUT', os.path.join(_project_root(), 'Animations', 'sourcepack', 'SourcePack_export'))
SELECT = os.environ.get('UTPP_SOURCEPACK_DBS', '')
lib = unreal.AnimationLibrary


def log(msg):
    unreal.log('[UTPP] ' + str(msg))


def curves_per_frame(seq, num_frames, length):
    out = {}
    try:
        names = lib.get_animation_curve_names(seq, unreal.RawCurveTrackTypes.RCT_FLOAT)
    except Exception as e:
        return {'_error': str(e)}
    for n in names:
        vals = []
        for f in range(num_frames):
            t = min(length, f * (length / max(1, num_frames - 1)))
            try:
                vals.append(lib.get_float_value_at_time(seq, n, t) if hasattr(lib, 'get_float_value_at_time')
                            else lib.evaluate_float_curve(seq, n, t))
            except Exception:
                vals = None
                break
        if vals is None:
            try:
                times, values = lib.get_float_keys(seq, n)
                out[str(n)] = {'keys_time': list(times), 'keys_value': list(values)}
            except Exception as e:
                out[str(n)] = {'_error': str(e)}
        else:
            out[str(n)] = vals
    return out


def notifies(seq):
    events = []
    try:
        for ev in lib.get_animation_notify_events(seq):
            rec = {}
            for attr in ('notify_name', 'trigger_time', 'duration', 'track_name'):
                try:
                    rec[attr] = str(ev.get_editor_property(attr)) if attr in ('notify_name', 'track_name') else float(ev.get_editor_property(attr))
                except Exception:
                    pass
            # UE 5.8 exposes the timing of FAnimNotifyEvent only through AnimationLibrary helpers.
            for key, fn in (('trigger_time', 'get_anim_notify_event_trigger_time'), ('duration', 'get_anim_notify_event_duration')):
                if key not in rec and hasattr(lib, fn):
                    try:
                        rec[key] = float(getattr(lib, fn)(ev))
                    except Exception as e:
                        rec[key + '_error'] = str(e)
            try:
                obj = ev.get_editor_property('notify_state_class') or ev.get_editor_property('notify')
                if obj:
                    rec['class'] = obj.get_class().get_name()
            except Exception:
                pass
            events.append(rec)
    except Exception as e:
        events.append({'_error': str(e)})
    return events


inv = json.load(open(os.path.join(OUT, 'inventory.json'), encoding='utf-8'))
wanted = None if SELECT.strip().upper() == 'ALL' else {s.strip() for s in SELECT.split(',') if s.strip()}
seq_dbs = {}
for db in inv['databases']:
    short = db['path'].rsplit('/', 1)[-1].split('.')[0]
    if wanted is not None and short not in wanted:
        continue
    for a in db['animations']:
        if a.get('asset'):
            seq_dbs.setdefault(a['asset'], []).append(short)
log(f'{len(seq_dbs)} sequences selected from {SELECT or "<none>"}')

fbx_dir, meta_dir = os.path.join(OUT, 'fbx'), os.path.join(OUT, 'meta')
os.makedirs(fbx_dir, exist_ok=True)
os.makedirs(meta_dir, exist_ok=True)
manifest = {'engine': unreal.SystemLibrary.get_engine_version(), 'selection': SELECT, 'sequences': []}
failed = 0
for path, dbs in sorted(seq_dbs.items()):
    seq = unreal.load_asset(path)
    name = seq.get_name()
    fbx_path = os.path.join(fbx_dir, name + '.fbx')
    opts = unreal.FbxExportOption()
    for prop, val in (('export_morph_targets', False), ('export_preview_mesh', False), ('export_local_time', True),
                      ('collision', False), ('level_of_detail', False), ('vertex_color', False)):
        try:
            opts.set_editor_property(prop, val)
        except Exception:
            pass
    task = unreal.AssetExportTask()
    task.set_editor_property('object', seq)
    task.set_editor_property('filename', fbx_path)
    task.set_editor_property('automated', True)
    task.set_editor_property('prompt', False)
    task.set_editor_property('replace_identical', True)
    task.set_editor_property('options', opts)
    ok = unreal.Exporter.run_asset_export_task(task)
    length = seq.get_play_length()
    try:
        num_frames = lib.get_num_frames(seq) + 1
    except Exception:
        num_frames = int(round(length * 30)) + 1
    meta = {
        'name': name, 'path': path, 'databases': dbs, 'fbx': fbx_path if ok else None, 'length_s': length,
        'num_frames': num_frames,
        'enable_root_motion': seq.get_editor_property('enable_root_motion'),
        'force_root_lock': seq.get_editor_property('force_root_lock'),
        'curves': curves_per_frame(seq, num_frames, length),
        'notifies': notifies(seq),
        'skeleton': seq.get_editor_property('skeleton').get_path_name() if seq.get_editor_property('skeleton') else None,
    }
    with open(os.path.join(meta_dir, name + '.json'), 'w', encoding='utf-8') as f:
        json.dump(meta, f, indent=1, default=str)
    manifest['sequences'].append({'name': name, 'path': path, 'databases': dbs, 'exported': bool(ok)})
    if not ok:
        failed += 1
        log(f'EXPORT FAILED {path}')
with open(os.path.join(OUT, 'manifest.json'), 'w', encoding='utf-8') as f:
    json.dump(manifest, f, indent=1)
log(f'exported {len(manifest["sequences"]) - failed}/{len(manifest["sequences"])} sequences to {fbx_dir}')
