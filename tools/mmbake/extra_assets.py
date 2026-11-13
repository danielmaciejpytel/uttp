"""Authored-clip extension of the v2 baker (optional: without --extra-assets nothing here runs and the output is unchanged).

--extra-assets <spec.json> appends retargeted clips (utppclip.json, same rig / fps as the bake) as extra ENTRIES of EXISTING
pose search databases, by name. The clips are not source pack assets (CF_ADDED, derived curves / events like the slide
clips), their entries are 'added' so they sort after the regular entries of a database and every other database /
entry keeps its id. Clips sort by name with the others; a name that sorts after all existing clip names (e.g. "RC28...")
leaves every existing first_frame unchanged.

Spec format "utpp-mmbake-v2-extra-assets" version 1:
  {"format": ..., "version": 1,
   "clips":   [{"name": "X", "file": "<path of X.utppclip.json, relative to the project root or absolute>"}],
   "entries": [{"db": "<existing database>", "clip": "X", "disable_reselection": <bool, optional>,
                "sampling_range": [a, b] (optional, [0, 0] = whole clip)}]}
disable_reselection defaults to the majority of the database's existing Neutral entries (the rule of the female clips).
The whole clip (minus the database's exclude-from-database tail) is indexed; no authored notify windows.
"""
from __future__ import annotations

import json
import os

SPEC_FORMAT = "utpp-mmbake-v2-extra-assets"
SPEC_VERSION = 1


def load_spec(path):
    with open(path, encoding="utf-8") as f:
        spec = json.load(f)
    if spec.get("format") != SPEC_FORMAT or int(spec.get("version", 0)) != SPEC_VERSION:
        raise SystemExit(f"extra assets: format/version {spec.get('format')}/{spec.get('version')} != {SPEC_FORMAT}/{SPEC_VERSION}")
    return spec


def resolve_clips(spec, root, taken_names):
    """{clip name: file path} of the spec; raises on a name that is already a baked clip or on a missing file."""
    out = {}
    for c in spec["clips"]:
        n = c["name"]
        if n in taken_names or n in out:
            raise SystemExit(f"extra assets: clip {n} already among the baked clips")
        p = c["file"] if os.path.isabs(c["file"]) else os.path.join(root, c["file"])
        if not os.path.isfile(p):
            raise SystemExit(f"extra assets: clip file {p} not found")
        out[n] = p
    return out


def make_entries(spec, clip_files, db_index, sourcepack_dr):
    """Entry dicts (mmbake_motion 'entries' format) of the spec. sourcepack_dr = {db: [disable_reselection of its Neutral entries]}."""
    ents = []
    for e in spec["entries"]:
        d, cn = e["db"], e["clip"]
        if d not in db_index:
            raise SystemExit(f"extra assets: database {d} is not a baked database")
        if cn not in clip_files:
            raise SystemExit(f"extra assets: entry clip {cn} is not in the spec clips")
        votes = sourcepack_dr.get(d, [True])
        dr = bool(e["disable_reselection"]) if e.get("disable_reselection") is not None else sum(votes) * 2 >= len(votes)
        rng = tuple(float(x) for x in e.get("sampling_range", [0.0, 0.0]))
        ents.append(dict(db=d, clip=cn, disable_reselection=dr, looping=None, range=rng, added=True))
    return ents
