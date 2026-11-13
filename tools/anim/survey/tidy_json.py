"""Remove the flattened duplicates a directory-level `convert serialize` writes for sub-directories: keep only json\\<depot>.json
of the raw files (mirrored layout). Only touches Extracted\\game\\player-anims\\json (output of extract_player_anims.ps1)."""
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import survey_common as SC  # noqa: E402

raws = {p[len(SC.RAW) + 1:].lower() for p in glob.glob(os.path.join(SC.RAW, '**', '*.anims'), recursive=True)}
keep = rm = 0
for p in glob.glob(os.path.join(SC.JSON_DIR, '**', '*.json'), recursive=True):
    rel = p[len(SC.JSON_DIR) + 1:-len('.json')].lower()
    if rel in raws:
        keep += 1
    else:
        os.remove(p)
        rm += 1
missing = [r for r in raws if not os.path.exists(SC.json_path(r))]
print('kept', keep, 'removed', rm, 'raw without json', len(missing))
for m in missing:
    print('  NOJSON', m)
