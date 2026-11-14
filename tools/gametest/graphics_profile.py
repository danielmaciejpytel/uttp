"""Graphics profile of the game for automated test runs: lowest quality settings for a steady frame rate.

  python graphics_profile.py --status
  python graphics_profile.py --apply      lowest settings for tests; the user's values are remembered
  python graphics_profile.py --restore    the user's values back (only the options this tool changed)

The game keeps its settings in UserSettings.json (in the user profile, outside the game folder). --apply changes only
the quality options listed in PROFILE and stores the previous value of each one in a state file; --restore writes those
values back into the current file, so settings the game or the user changed meanwhile (key bindings, audio ...) stay.
A full copy of the file before the first --apply is kept next to the state file. Nothing is done while the game runs
(the game reads the file at start and writes it at exit).

Resolution, window mode, upscaler mode and the frame cap are left alone: their choices are not listed in the file.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utpp_paths import project_root  # noqa: E402

GAME_EXE = 'Cyberpunk2077.exe'
LOWEST = object()      # first entry of the option's "values" list
PROFILE: dict[str, dict[str, object]] = {
    '/graphics/presets': {'TextureQuality': LOWEST, 'DLSS_D': False, 'DLSSFrameGen': False, 'FSR3_FrameGeneration': False,
                          'FSR4_FrameGeneration': False, 'XESS_FrameGeneration': False, 'DynamicResolutionScaling': False},
    '/graphics/raytracing': {'RayTracing': False, 'RayTracedReflections': False, 'RayTracedSunShadows': False,
                             'RayTracedLocalShadows': False, 'RayTracedLighting': LOWEST, 'RayTracedPathTracing': False,
                             'RayTracedPathTracingForPhotoMode': False},
    '/graphics/advanced': {'ContactShadows': False, 'FacialTangentUpdates': False, 'Anisotropy': LOWEST,
                           'ShadowMeshQuality': LOWEST, 'LocalShadowsQuality': LOWEST, 'CascadedShadowsRange': LOWEST,
                           'CascadedShadowsResolution': LOWEST, 'DistantShadowsResolution': LOWEST,
                           'VolumetricFogResolution': LOWEST, 'VolumetricCloudsQuality': LOWEST, 'MaxDynamicDecals': LOWEST,
                           'ScreenSpaceReflectionsQuality': LOWEST, 'SubsurfaceScatteringQuality': LOWEST,
                           'AmbientOcclusion': LOWEST, 'ColorPrecision': LOWEST, 'GlobaIlluminationRange': LOWEST,
                           'MirrorQuality': LOWEST, 'LODPreset': LOWEST},
    '/graphics/performance': {'CrowdDensity': LOWEST},
    '/gameplay/performance': {'CrowdDensity': LOWEST},
    '/graphics/basic': {'FilmGrain': False, 'ChromaticAberration': False, 'DepthOfField': False, 'LensFlares': False,
                        'MotionBlur': LOWEST, 'Vignette': False},
}


def settings_path() -> str:
    return os.path.join(os.environ.get('LOCALAPPDATA', ''), 'CD Projekt Red', 'Cyberpunk 2077', 'UserSettings.json')


def state_dir() -> str:
    return os.path.join(str(project_root()), 'Research', 'logs', 'night', 'graphics_profile')


def game_running() -> bool:
    out = subprocess.run(['tasklist', '/FI', f'IMAGENAME eq {GAME_EXE}', '/NH'], capture_output=True, text=True).stdout
    return GAME_EXE in out


def load(path: str) -> dict:
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def save(path: str, doc: dict) -> None:
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(doc, f, indent=4, ensure_ascii=False)
    os.replace(tmp, path)


def options(doc: dict) -> dict[tuple[str, str], dict]:
    return {(g.get('group_name'), o.get('name')): o for g in doc.get('data', []) for o in g.get('options', [])}


def target(opt: dict, want: object):
    """(value, index or None) the profile asks for, or None when the option cannot take it."""
    if want is LOWEST:
        vals = opt.get('values')
        if not vals:
            return None
        return vals[0], 0
    return want, None


def apply_profile(doc: dict) -> tuple[dict, list[str]]:
    """Set the profile in `doc`; returns ({'group|name': previous value / index}, list of text lines)."""
    prev: dict[str, dict] = {}
    lines: list[str] = []
    opts = options(doc)
    for group, names in PROFILE.items():
        for name, want in names.items():
            o = opts.get((group, name))
            if o is None:
                lines.append(f'  missing  {group} {name}')
                continue
            t = target(o, want)
            if t is None:
                lines.append(f'  skipped  {group} {name} (no list of values)')
                continue
            value, index = t
            if o.get('value') == value and (index is None or o.get('index') == index):
                continue
            prev[f'{group}|{name}'] = {'value': o.get('value'), 'index': o.get('index')}
            lines.append(f'  {group} {name}: {o.get("value")!r} -> {value!r}')
            o['value'] = value
            if index is not None and 'index' in o:
                o['index'] = index
    return prev, lines


def restore_profile(doc: dict, prev: dict) -> list[str]:
    lines: list[str] = []
    opts = options(doc)
    for key, old in prev.items():
        group, name = key.split('|', 1)
        o = opts.get((group, name))
        if o is None:
            lines.append(f'  missing  {group} {name}')
            continue
        lines.append(f'  {group} {name}: {o.get("value")!r} -> {old.get("value")!r}')
        o['value'] = old.get('value')
        if old.get('index') is not None and 'index' in o:
            o['index'] = old.get('index')
    return lines


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--restore', action='store_true')
    ap.add_argument('--status', action='store_true')
    ap.add_argument('--settings', default=settings_path())
    ap.add_argument('--state-dir', default=state_dir())
    a = ap.parse_args(argv)
    state_file = os.path.join(a.state_dir, 'state.json')
    have_state = os.path.exists(state_file)
    if a.status or not (a.apply or a.restore):
        print(f'settings file: {a.settings}')
        print(f'test profile applied: {have_state}')
        if have_state:
            st = load(state_file)
            print(f'  applied {st.get("applied_at")}, {len(st.get("previous", {}))} options changed')
        return 0
    if not os.path.exists(a.settings):
        print(f'no settings file: {a.settings}', file=sys.stderr)
        return 2
    if game_running():
        print('the game is running: nothing changed', file=sys.stderr)
        return 3
    doc = load(a.settings)
    os.makedirs(a.state_dir, exist_ok=True)
    if a.apply:
        if have_state:
            st = load(state_file)
            prev_all = st.get('previous', {})
        else:
            shutil.copy2(a.settings, os.path.join(a.state_dir, 'UserSettings.before.json'))
            prev_all = {}
        prev, lines = apply_profile(doc)
        for k, v in prev.items():
            prev_all.setdefault(k, v)          # keep the user's value from before the FIRST apply
        save(a.settings, doc)
        with open(state_file, 'w', encoding='utf-8') as f:
            json.dump({'applied_at': time.strftime('%Y-%m-%d %H:%M:%S'), 'previous': prev_all}, f, indent=1)
        print('\n'.join(lines))
        print(f'test profile applied: {len(prev)} options changed now, {len(prev_all)} remembered')
        return 0
    if not have_state:
        print('no test profile is applied: nothing to restore')
        return 0
    st = load(state_file)
    lines = restore_profile(doc, st.get('previous', {}))
    save(a.settings, doc)
    os.replace(state_file, os.path.join(a.state_dir, 'state.restored-%s.json' % time.strftime('%Y%m%d-%H%M%S')))
    print('\n'.join(lines))
    print(f'user settings restored: {len(lines)} options')
    return 0


if __name__ == '__main__':
    sys.exit(main())
