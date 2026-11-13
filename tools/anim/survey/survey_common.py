"""Shared paths / helpers of the player-animation deformation survey (2026-11-09, offline only).
Run with Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I.  Writes only under Build\\work\\anim-survey-20261109 (WORK) and
Extracted\\game\\player-anims (EXT).  Reads (never writes) the melee tools in Source\\tools\\melee and their reference folders.
"""
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..'))
WORK = os.path.join(ROOT, 'Build', 'work', 'anim-survey-20261109')
EXT = os.path.join(ROOT, 'Extracted', 'game', 'player-anims')
RAW, JSON_DIR, GLB, LOGS = (os.path.join(EXT, k) for k in ('raw', 'json', 'glb', 'logs'))
DUMPS = os.path.join(WORK, 'dumps')
PY = os.path.join(ROOT, 'Tools', 'ai-motion', 'envs', 'kimodo', 'Scripts', 'python.exe')
SETS_JSON = os.path.join(ROOT, 'Build', 'work', 'melee-export-20261109', 'evidence', 'player_entity_sets_unique.json')
MELEE = os.path.join(ROOT, 'Source', 'tools', 'melee')
REPORT = os.path.join(WORK, 'report.txt')
FPS = 30.0


def add_tool_paths():
    for p in (MELEE, os.path.join(ROOT, 'Source', 'tools', 'ai_motion'), os.path.join(ROOT, 'Source', 'tools', 'anim', 'vehicle-entry')):
        if p not in sys.path:
            sys.path.insert(0, p)


def set_key(depot):
    """depot path of a set -> file name (unique over the 317 player sets, checked in triage)."""
    return os.path.basename(depot.replace('\\', '/'))


def json_path(depot):
    return os.path.join(JSON_DIR, depot.replace('/', '\\') + '.json')


def load_set_json(depot):
    p = json_path(depot)
    if not os.path.exists(p):
        return None
    return json.load(open(p, encoding='utf-8'))


def clips_of(js):
    """serialized .anims json -> (rig depot path, [dict(name, frames, duration, type, events)])."""
    rc = js['Data']['RootChunk']
    rig = rc.get('rig', {}).get('DepotPath', {}).get('$value', '')
    out = []
    for e in rc.get('animations', []) or []:
        a = e['Data']['animation']['Data']
        ab = (a.get('animBuffer') or {}).get('Data') or {}
        out.append(dict(name=a['name']['$value'], frames=int(ab.get('numFrames', 0) or 0), duration=float(a.get('duration', 0.0) or 0.0),
                        type=a.get('animationType', ''), rootmotion=str((a.get('motionExtraction') or {}).get('Data') is not None)))
    return rig, out


def report(section, text):
    """Append a section to report.txt (incremental: every stage writes its own block at once)."""
    os.makedirs(WORK, exist_ok=True)
    with open(REPORT, 'a', encoding='utf-8') as f:
        f.write('\n==== %s ====\n%s\n' % (section, text.rstrip()))


def jdump(obj, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(obj, f, indent=1, ensure_ascii=False, default=lambda o: o.item() if hasattr(o, 'item') else str(o))
