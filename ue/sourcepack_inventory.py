"""Read-only inventory of source pack Pose Search Databases (run inside UnrealEditor-Cmd via -run=pythonscript).

Writes <out_dir>/inventory.json: for every PoseSearchDatabase under the given content path, its schema,
the animation assets it references (with per-entry options when readable) and basic sequence info
(length, frame rate, frames, root motion). Nothing in the project is modified or saved.

Env: UTPP_SOURCEPACK_ROOT (content path, default Dense databases), UTPP_SOURCEPACK_OUT (output folder).
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


ROOT = os.environ.get('UTPP_SOURCEPACK_ROOT', '/Game/Characters/UEFN_Mannequin/Animations/MotionMatchingData/Databases/Dense')
OUT = os.environ.get('UTPP_SOURCEPACK_OUT', os.path.join(_project_root(), 'Animations', 'sourcepack', 'SourcePack_export'))
log = []


def note(msg):
    log.append(str(msg))
    unreal.log('[UTPP] ' + str(msg))


def props(obj, names):
    out = {}
    for n in names:
        try:
            out[n] = obj.get_editor_property(n)
        except Exception:  # property names differ between engine versions; record only what exists
            pass
    return out


def plain(v):
    if v is None or isinstance(v, (bool, int, float, str)):
        return v
    if isinstance(v, (list, tuple, unreal.Array)):
        return [plain(x) for x in v]
    if isinstance(v, unreal.Object):
        return v.get_path_name()
    if isinstance(v, unreal.Name):
        return str(v)
    if isinstance(v, unreal.StructBase):
        d = {}
        for attr in dir(v):
            if attr.startswith('_') or callable(getattr(v, attr, None)):
                continue
            try:
                d[attr] = plain(getattr(v, attr))
            except Exception:
                pass
        return d or str(v)
    return str(v)


def sequence_info(asset):
    info = {'path': asset.get_path_name(), 'class': asset.get_class().get_name()}
    if isinstance(asset, unreal.AnimSequence):
        try:
            info['length_s'] = asset.get_play_length()
        except Exception:
            pass
        lib = unreal.AnimationLibrary
        for key, fn in (('frame_rate', 'get_frame_rate'), ('num_frames', 'get_num_frames')):
            try:
                info[key] = plain(getattr(lib, fn)(asset))
            except Exception:
                pass
        info.update({k: plain(v) for k, v in props(asset, ['enable_root_motion', 'root_motion_root_lock', 'force_root_lock']).items()})
        try:
            info['curves'] = [str(c) for c in lib.get_animation_curve_names(asset, unreal.RawCurveTrackTypes.RCT_FLOAT)]
        except Exception:
            pass
        try:
            info['notifies'] = sorted({str(e.notify_name) if hasattr(e, 'notify_name') else str(e) for e in lib.get_animation_notify_events(asset)})
        except Exception:
            pass
    return info


registry = unreal.AssetRegistryHelpers.get_asset_registry()
assets = registry.get_assets_by_path(ROOT, recursive=True)
dbs = [a for a in assets if str(a.asset_class_path.asset_name) == 'PoseSearchDatabase']
note(f'{len(dbs)} PoseSearchDatabase assets under {ROOT}')
result = {'root': ROOT, 'engine': unreal.SystemLibrary.get_engine_version(), 'databases': []}
seen = {}
for ad in dbs:
    db = ad.get_asset()
    entry = {'path': db.get_path_name()}
    entry.update({k: plain(v) for k, v in props(db, ['schema', 'tags', 'pose_search_mode', 'normalization_set']).items()})
    # UE 5.8: the entry array is not exposed as a property; use the BlueprintPure accessors.
    entry['animations'] = []
    count = db.get_num_animation_assets()
    for i in range(count):
        asset = db.get_animation_asset(i)
        rec = {'index': i, 'asset': asset.get_path_name() if asset else None}
        if asset and rec['asset'] not in seen:
            seen[rec['asset']] = sequence_info(asset)
        entry['animations'].append(rec)
    result['databases'].append(entry)
    note(f"{entry['path']}: {len(entry['animations'])} entries")
result['sequences'] = seen
result['log'] = log
os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, 'inventory.json'), 'w', encoding='utf-8') as f:
    json.dump(result, f, indent=2, default=str)
note(f'wrote {os.path.join(OUT, "inventory.json")} with {len(seen)} unique sequences')
