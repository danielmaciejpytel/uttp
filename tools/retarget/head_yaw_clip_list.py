#!/usr/bin/env python3
"""Clip list for head_yaw_audit.py on the source pack retarget output, offline.

head_yaw_audit.py (measure / apply) must run only on locomotion clips, never on Pose_*, AO_*, *_Transition or other
single-frame / additive assets. This script writes the --keep list for it from the clip map:
  include: purpose group in head_yaw.include_groups (sourcepack-names.json: source pack names and name
           patterns stay outside the git repo);
  exclude: additive_type other than AAT_NONE (head_yaw.exclude_additive), fewer than head_yaw.min_ue_frames UE frames,
           a name matching any head_yaw.exclude_name_regex, and (with --clips-dir) clips without <name>.utppclip.json.
Output: {"clips": [{"name": ...}, ...], "excluded": {...}} = the format head_yaw_audit.py --keep reads.
Usage: python head_yaw_clip_list.py --clips-dir <retarget out dir> --out <out dir>\\headyaw_keep.json
       python head_yaw_audit.py measure --dirs <retarget out dir> --keep <out dir>\\headyaw_keep.json --out <log>
Outputs stay outside Source/.
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
CLIP_MAP = os.path.join(ROOT, "Research", "inventory", "sourcepack-clip-map.json")
NAMES = os.path.join(ROOT, "Research", "inventory", "sourcepack-names.json")
CLIP_SUFFIX = ".utppclip.json"


def load(p):
    with open(p, encoding="utf-8") as fh:
        return json.load(fh)


def select(clip_map, rule, clips_dir=None):
    """-> (kept records, excluded {reason: [names]}) in clip-map order."""
    groups = set(rule["include_groups"])
    rx = [re.compile(x) for x in rule["exclude_name_regex"]]
    have = None
    if clips_dir:
        have = {f[:-len(CLIP_SUFFIX)] for f in os.listdir(clips_dir) if f.endswith(CLIP_SUFFIX)}
    keep, excl = [], collections.defaultdict(list)
    for c in clip_map["clips"]:
        name, grp = c["name"], c["purpose"]["group"]
        if grp not in groups:
            excl["group:" + grp].append(name)
            continue
        if rule.get("exclude_additive", True) and c.get("additive_type") not in (None, "AAT_NONE"):
            excl["additive"].append(name)
            continue
        if int(c.get("num_frames") or 0) < int(rule.get("min_ue_frames", 2)):
            excl["single_frame"].append(name)
            continue
        hit = next((r.pattern for r in rx if r.search(name)), None)
        if hit:
            excl["name:" + hit].append(name)
            continue
        if have is not None and name not in have:
            excl["not_in_clips_dir"].append(name)
            continue
        keep.append({"name": name, "group": grp})
    return keep, excl


def main(argv=None):
    ap = argparse.ArgumentParser(prog="head_yaw_clip_list.py")
    ap.add_argument("--clip-map", default=CLIP_MAP)
    ap.add_argument("--names", default=NAMES)
    ap.add_argument("--clips-dir", default=None, help="retarget output dir; keep only clips with a .utppclip.json there")
    ap.add_argument("--out", required=True, help="keep-list json for head_yaw_audit.py --keep (outside Source/)")
    a = ap.parse_args(argv)
    if os.path.normcase(os.path.abspath(a.out)).startswith(os.path.normcase(os.path.join(ROOT, "Source")) + os.sep):
        sys.exit("--out must be outside Source/ (source-pack-derived clip names)")
    rule = load(a.names)["head_yaw"]
    keep, excl = select(load(a.clip_map), rule, a.clips_dir)
    out = {"format": "utpp-headyaw-keep", "version": 1, "generator": "Source/tools/retarget/head_yaw_clip_list.py",
           "licence": "Licensed animation data",
           "rule": rule, "clip_map": a.clip_map, "clips_dir": a.clips_dir, "count": len(keep), "clips": keep,
           "excluded_counts": {k: len(v) for k, v in sorted(excl.items())},
           "excluded": {k: v for k, v in sorted(excl.items()) if not k.startswith("group:")}}
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(out, fh, indent=1)
    print("UTPP_HEADYAW_LIST %d clips -> %s; excluded %s" % (len(keep), a.out, json.dumps(out["excluded_counts"])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
