"""Paths of the project folder and of the game, without machine-specific paths in the sources.

    import os, sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))   # the tools folder
    from utpp_paths import project_root, game_dir, local_path

project_root()  the folder that contains Source, Build, Data ... (environment variable UTPP_PROJECT_ROOT, else the
                nearest folder above this file that contains both "Source" and "Build")
game_dir()      the Cyberpunk 2077 folder (environment variable UTPP_GAME_DIR, else "game_dir" in local_paths.json)
local_path(key) any other machine-specific path: environment variable UTPP_<KEY in upper case>, else <key> in
                local_paths.json, else the default

local_paths.json is not tracked. It is read from the folder of this file, else from Source/tools of the nearest
folder above it that has one (so a second working copy inside the project uses the same file);
local_paths.example.json shows its keys.
"""
import json
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_CACHE = None


def project_root():
    env = os.environ.get('UTPP_PROJECT_ROOT')
    if env:
        return env
    d = _HERE
    while True:
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
        if os.path.isdir(os.path.join(d, 'Source')) and os.path.isdir(os.path.join(d, 'Build')):
            return d
    return os.path.abspath(os.path.join(_HERE, '..', '..'))


def _local():
    global _CACHE
    if _CACHE is None:
        _CACHE = {}
        candidates = [os.path.join(_HERE, 'local_paths.json')]
        d = _HERE
        while os.path.dirname(d) != d:   # every folder above this file that has Source/tools (nearest first)
            d = os.path.dirname(d)
            candidates.append(os.path.join(d, 'Source', 'tools', 'local_paths.json'))
        for p in candidates:
            try:
                with open(p, encoding='utf-8') as f:
                    _CACHE = json.load(f)
                break
            except FileNotFoundError:
                continue
    return _CACHE


def local_path(key, default=None):
    v = os.environ.get('UTPP_' + key.upper()) or _local().get(key)
    return v if v else default


def game_dir(required=True):
    v = local_path('game_dir')
    if not v and required:
        raise SystemExit('The game folder is not configured: set the environment variable UTPP_GAME_DIR '
                         'or "game_dir" in ' + os.path.join(_HERE, 'local_paths.json'))
    return v
