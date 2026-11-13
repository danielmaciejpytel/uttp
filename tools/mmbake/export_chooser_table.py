#!/usr/bin/env python3
"""Export a source pack chooser table as a runtime data file for UTPP (src/mm/Chooser.hpp).

Output format "utpp-chooser-table" version 1 - the same layout mmbake_motion.py writes next to a baked database
(<db>.chooser.json), so the runtime loads either file:
  inputs[]   key, kind (float | bool | enum), property_type (float | double | int | bool | enum), display[],
             enum {name, values[{value, name, display}]}
  tables[]   reachable tables from the root (tables[0] = root), each {label, columns[], rows[], fallback}
             columns: {input, key, kind, disabled, [wrap, min_value, max_value], cells[]}
               float cells [min, max, no_min, no_max] (min/max already rounded to float32)
               bool cells  "MatchAny" | "MatchTrue" | "MatchFalse"
               enum cells  [comparison, value]
             rows: {disabled, result}; result {kind: database, name, db} | {kind: table, table} | {kind: invalid}
             fallback: result or null
Extra blocks written only by this exporter (the runtime treats them as optional):
  mm_rules.blend_time   blend-time function bound to the MM node (parsed from the extracted ABP pseudo code; fixed
                        graph shape, names read from the data): mode input, ground / air enum values, ground->ground,
                        landing, rising in air (vertical velocity > threshold, cm/s) and other in-air blend times (s)
  mm_rules.interrupt    interrupt-mode function of the MM node: mode input, ground value, ground state inputs
  blend_profile         blend profile of the Motion Matching node (skeletons.json entries, UE bone names, mapped
                        to Cyberpunk bone names through a retarget bone map when given)

Semantics are taken from motion_chooser_ref.py (UE 5.8 Chooser plugin port): the tables, enum values (resolved from
ValueName), float32 bounds and bound-property types come from the same ChooserRef objects. The default export is
the tables AS WRITTEN (needed for 100% agreement with chooser_oracle.json / chooser_vectors.json); --overrides
applies row overrides ({"row_overrides": [{"table": label, "row": n, "action": "disable", "reason": ...}]} or the
"chooser" block of a mmbake_motion config).

The output is source-pack-derived data; keep it outside the Source git repo.

Usage:
  python export_chooser_table.py --choosers Extracted\\sourcepack\\sourcepack\\choosers.json --out Data\\sourcepack_chooser_table.json
"""
import argparse
import hashlib
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import motion_chooser_ref as gcr  # noqa: E402

PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def rel(path):
    try:
        return os.path.relpath(os.path.abspath(path), PROJECT_ROOT).replace("\\", "/")
    except ValueError:
        return os.path.abspath(path)


# ------------------------------------------------------------------------------------------------ tables

def export_tables(ref, root, overrides):
    tables = ref.reachable(root)
    tidx = {id(t): i for i, t in enumerate(tables)}
    binds = ref.bindings(root)
    keys = sorted(binds)
    inputs = []
    for k in keys:
        b = binds[k]
        e = {"key": k, "kind": b["kind"], "property_type": b.get("prop_type"), "display": sorted(b.get("display") or [])}
        c0 = b["columns"][0]
        if b["kind"] == "enum" and c0.enum is not None:
            e["enum"] = {"name": c0.enum.name,
                         "values": [{"value": int(v), "name": i, "display": d} for v, i, d in c0.enum.entries]}
        inputs.append(e)

    def res(r):
        if r is None:
            return None
        if r.kind == "asset":
            if not r.target:
                return {"kind": "database", "name": None, "db": -1}
            return {"kind": "database", "name": gcr.short_name(r.target), "db": -1}
        if r.kind in ("evaluate", "nested"):
            t = ref.resolve(r)
            return {"kind": "table", "table": tidx[id(t)] if t is not None else -1}
        return {"kind": "invalid"}

    applied = []
    out_tables = []
    for t in tables:
        cols = []
        for c in t.columns:
            if not c.has_filters:
                continue
            if c.kind == "unsupported":
                raise gcr.ChooserRefError("%s: column type not supported by the runtime" % c.where())
            cd = {"input": keys.index(c.key) if c.key in keys else -1, "key": c.key, "kind": c.kind,
                  "disabled": bool(c.disabled)}
            if c.kind == "float":
                cd.update(wrap=bool(c.wrap), min_value=float(c.min_value), max_value=float(c.max_value))
                cd["cells"] = [[float(mn), float(mx), bool(a), bool(b)] for mn, mx, a, b in c.rows]
            elif c.kind == "enum":
                cd["cells"] = [[cmp, int(v)] for cmp, v in c.rows]
            else:
                cd["cells"] = list(c.rows)
            cols.append(cd)
        rows = []
        for i, r in enumerate(t.results):
            ov = overrides.get((t.label, i))
            disabled = bool(t.disabled[i]) or bool(ov and ov["action"] == "disable")
            if ov:
                applied.append({"table": t.label, "row": i, "action": ov["action"], "reason": ov.get("reason")})
            rows.append({"disabled": disabled, "result": res(r)})
        out_tables.append({"label": t.label, "columns": cols, "rows": rows,
                           "fallback": res(t.fallback) if t.fallback is not None else None})
    unknown = set(overrides) - {(a["table"], a["row"]) for a in applied}
    if unknown:
        raise gcr.ChooserRefError("row overrides for unknown rows: %s" % sorted(unknown))
    return inputs, out_tables, applied


# ------------------------------------------------------------------------------------------------ ABP rules

def _graph(text, name, pseudo_path):
    m = re.search(r'## GRAPH "%s".*?(?=\n## GRAPH |\Z)' % re.escape(name), text, re.S)
    if not m:
        raise gcr.ChooserRefError("graph %s not found in %s" % (name, pseudo_path))
    return m.group(0)


def _enum_input(binds, key, where):
    b = binds.get(key)
    if b is None or b["kind"] != "enum" or b.get("enum") is None:
        raise gcr.ChooserRefError("%s: %r is not an enum input of the chooser (inputs: %s)" % (where, key, sorted(binds)))
    return b["enum"]


def parse_blend_time(pseudo_path, binds):
    """Blend-time function of the ABP pseudo code (graph name from the MM node binding) -> data-driven rule.
    The expected graph SHAPE is fixed (the names are read from the data, none is hard-coded):
      switch <Enum> M: case G: switch <Enum> M_LastFrame: case G: return A; case B: return Bv
                       case B: if (<velocity> > T): return C; else: return D
    M must be an enum input of the chooser; G and B are resolved to its enum values."""
    with open(pseudo_path, encoding="utf-8") as f:
        text = f.read()
    m = re.search(r"AnimGraphNode_MotionMatching_0: BlendTime<=(\w+)\(\)", text)
    if not m:
        raise gcr.ChooserRefError("MM node BlendTime binding not found in %s" % pseudo_path)
    fn = m.group(1)
    block = _graph(text, fn, pseudo_path)
    seq = []
    for ln in (x.strip() for x in block.splitlines()):
        mm = re.fullmatch(r"switch \w+ (\w+):.*", ln)
        if mm:
            seq.append(("switch", mm.group(1)))
            continue
        mm = re.fullmatch(r"case (\w+):", ln)
        if mm:
            seq.append(("case", mm.group(1)))
            continue
        mm = re.fullmatch(r"if \(([\w.]+) > ([-0-9.eE+]+)\):.*", ln)
        if mm:
            seq.append(("if_gt", (mm.group(1), float(mm.group(2)))))
            continue
        if ln == "else:":
            seq.append(("else", None))
            continue
        mm = re.fullmatch(r"return ReturnValue=([-0-9.eE+]+)", ln)
        if mm:
            seq.append(("return", float(mm.group(1))))
    shape = ["switch", "case", "switch", "case", "return", "case", "return", "case", "if_gt", "return", "else",
             "return"]
    if [s[0] for s in seq] != shape:
        raise gcr.ChooserRefError("%s: unexpected structure %r" % (fn, seq))
    mode, ground, air = seq[0][1], seq[1][1], seq[5][1]
    if seq[2][1] != mode + "_LastFrame" or seq[3][1] != ground or seq[7][1] != air or ground == air:
        raise gcr.ChooserRefError("%s: unexpected structure %r" % (fn, seq))
    enum = _enum_input(binds, mode, fn)
    return {
        "function": fn,
        "mode_input": mode,
        "ground_value": enum.value_of(ground), "ground_name": ground,
        "air_value": enum.value_of(air), "air_name": air,
        "ground_to_ground_s": seq[4][1],
        "landing_s": seq[6][1],
        "air_rising_s": seq[9][1],
        "air_other_s": seq[11][1],
        "rising_velocity": seq[8][1][0],
        "rising_threshold_cm_s": seq[8][1][1],
        "rule": "mode == ground: last frame ground -> ground_to_ground_s, last frame air -> landing_s; "
                "mode == air: vertical velocity (cm/s) > rising_threshold_cm_s -> air_rising_s, else air_other_s; "
                "any other combination: no value (caller default)",
    }


_INTERRUPT_TEMPLATE = ("(((V0 != V0_LastFrame) || (((V1 != V1_LastFrame) || (V2 != V2_LastFrame) || "
                       "(V3 != V3_LastFrame)) && (V0 == E::G))) ? EPoseSearchInterruptMode::InterruptOnDatabaseChange"
                       " : EPoseSearchInterruptMode::DoNotInterrupt)")


def parse_interrupt(pseudo_path, binds):
    """Interrupt-mode function of the MM node (UE EPoseSearchInterruptMode) -> data-driven rule. Fixed shape
    (_INTERRUPT_TEMPLATE); variable names and the ground enumerator are read from the data."""
    with open(pseudo_path, encoding="utf-8") as f:
        text = f.read()
    m = re.search(r"SetDatabasesToSearch\([^\n]*?InterruptMode=(\w+)\(\)", text) or \
        re.search(r'## GRAPH "(\w*InterruptMode\w*)"', text)
    if not m:
        raise gcr.ChooserRefError("interrupt mode function not found in %s" % pseudo_path)
    fn = m.group(1)
    block = _graph(text, fn, pseudo_path)
    mm = re.search(r"return ReturnValue=(.*)", block)
    if not mm:
        raise gcr.ChooserRefError("%s: no return expression" % fn)
    expr = mm.group(1).strip()
    names = re.findall(r"\((\w+) != (\w+)_LastFrame\)", expr)
    if len(names) != 4 or any(a != b for a, b in names):
        raise gcr.ChooserRefError("%s: unexpected expression %s" % (fn, expr))
    g = re.search(r"\((\w+) == (\w+)::(\w+)\)", expr)
    if not g or g.group(1) != names[0][0]:
        raise gcr.ChooserRefError("%s: unexpected expression %s" % (fn, expr))
    norm = expr
    for i, (a, _) in enumerate(names):
        norm = re.sub(r"\b%s(_LastFrame)?\b" % re.escape(a), lambda x, i=i: "V%d%s" % (i, x.group(1) or ""), norm)
    norm = norm.replace("%s::%s" % (g.group(2), g.group(3)), "E::G")
    if norm != _INTERRUPT_TEMPLATE:
        raise gcr.ChooserRefError("%s: expression shape changed: %s" % (fn, expr))
    mode = names[0][0]
    enum = _enum_input(binds, mode, fn)
    for a, _ in names[1:]:
        if a not in binds:
            raise gcr.ChooserRefError("%s: %s is not a chooser input" % (fn, a))
    return {"function": fn, "expression": expr,
            "mode_input": mode, "ground_value": enum.value_of(g.group(3)), "ground_name": g.group(3),
            "ground_state_inputs": [a for a, _ in names[1:]],
            "rule": "InterruptOnDatabaseChange when mode_input changed since the last frame, or when mode_input == "
                    "ground and any ground_state_inputs changed; otherwise DoNotInterrupt"}


def parse_mm_node(cdo_path):
    with open(cdo_path, encoding="utf-8") as f:
        lines = f.read().splitlines()
    for i, ln in enumerate(lines):
        if ln.strip() == "== AnimGraphNode_MotionMatching" and i + 1 < len(lines):
            node = json.loads(lines[i + 1])
            return {k: node.get(k) for k in ("BlendTime", "BlendProfile", "BlendOption", "bUseInertialBlend",
                                             "MaxActiveBlends", "PoseReselectHistory")}
    raise gcr.ChooserRefError("AnimGraphNode_MotionMatching not found in %s" % cdo_path)


def export_blend_profile(skeletons_path, profile_ref, bonemap_path):
    m = re.search(r"([\w/]+\.\w+):([^'\"]+)'?$", profile_ref or "")
    if not m:
        raise gcr.ChooserRefError("blend profile reference %r not understood" % profile_ref)
    skel_path, name = m.group(1), m.group(2)
    with open(skeletons_path, encoding="utf-8") as f:
        sk = json.load(f)
    hit = None
    for s in sk["skeletons"]:
        if (s.get("path") or s.get("name")) == skel_path:
            for bp in s.get("blend_profiles", []):
                if bp.get("name") == name:
                    hit = bp
    if hit is None:
        raise gcr.ChooserRefError("blend profile %s not found on %s" % (name, skel_path))
    cp_of = {}
    target_rig = None
    if bonemap_path:
        with open(bonemap_path, encoding="utf-8") as f:
            bm = json.load(f)
        target_rig = bm.get("target_rig")
        for b in bm.get("bones", []):
            if b.get("src") and b.get("cp"):
                cp_of.setdefault(b["src"], b["cp"])
        rm = bm.get("root_motion") or {}
        if rm.get("root_bone_identity"):
            cp_of.setdefault("root", rm["root_bone_identity"])
    return {"name": name, "skeleton": skel_path, "mode": hit.get("mode"),
            "mode_semantics": "WeightFactor: bone duration = blend time / scale; TimeFactor: blend time * scale "
                              "(UE BlendProfile.cpp FillSkeletonBoneDurationsArray)",
            "bonemap": rel(bonemap_path) if bonemap_path else None, "target_rig": target_rig,
            "entries": [{"bone": e["bone"], "scale": float(e["scale"]), "cp": cp_of.get(e["bone"])}
                        for e in hit.get("entries", [])]}


# ------------------------------------------------------------------------------------------------ main

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--choosers", default=os.path.join(PROJECT_ROOT, "Extracted", "sourcepack", "sourcepack", "choosers.json"))
    ap.add_argument("--root", default=None, help="root chooser (default: chooser.root of --names)")
    ap.add_argument("--names", default=gcr.NAMES_JSON)
    ap.add_argument("--abp-pseudo", default=os.path.join(PROJECT_ROOT, "Extracted", "sourcepack", "sourcepack", "pseudo",
                                                         "GASPALS__Blueprints__ABP_SandboxCharacter.txt"))
    ap.add_argument("--abp-cdo", default=os.path.join(PROJECT_ROOT, "Extracted", "sourcepack", "sourcepack", "pseudo",
                                                      "ABP_SandboxCharacter.animnodes_cdo.txt"))
    ap.add_argument("--skeletons", default=os.path.join(PROJECT_ROOT, "Extracted", "sourcepack", "sourcepack", "skeletons.json"))
    ap.add_argument("--bonemap", default=os.path.join(HERE, "..", "retarget", "bonemap_source_to_game_woman.json"))
    ap.add_argument("--overrides", default=None, help="json with row_overrides (or a mmbake_motion config with chooser)")
    ap.add_argument("--no-rules", action="store_true", help="omit mm_rules / blend_profile")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)

    ref = gcr.ChooserRef(a.choosers, names_json=a.names)
    root = ref.find_root(a.root)
    for w in ref.warnings:
        print("WARNING:", w, file=sys.stderr)
    overrides = {}
    if a.overrides:
        with open(a.overrides, encoding="utf-8") as f:
            od = json.load(f)
        lst = od.get("row_overrides") or (od.get("chooser") or {}).get("row_overrides") or []
        overrides = {(o["table"], int(o["row"])): o for o in lst}
    inputs, tables, applied = export_tables(ref, root, overrides)

    data = {
        "format": "utpp-chooser-table", "version": 1,
        "licence": "Licensed animation data",
        "generator": "Source/tools/mmbake/export_chooser_table.py (tables parsed by motion_chooser_ref.py)",
        "database_file": None,
        "semantics": {"mode": "AllResults: union of the databases of every matching row (UE callback order); "
                              "FirstResult: first database only",
                      "ranges": "closed [min, max]; no_min / no_max open a side; bounds float32",
                      "bool_cells": "MatchAny | MatchTrue | MatchFalse",
                      "enum_cells": "[comparison, value] with comparison MatchEqual | MatchNotEqual | MatchAny",
                      "results": "database: name (db = DBAS index of a database file, -1 = none); table: index into "
                                 "tables",
                      "fallback": "evaluated only when no row of that table succeeded; the table still reports "
                                  "Failed",
                      "root": "tables[0]"},
        "source": {"choosers_json": rel(a.choosers), "sha256": sha256_file(a.choosers), "root": root.asset_path},
        "inputs": inputs, "tables": tables, "row_overrides_applied": applied,
    }
    if not a.no_rules:
        mm_node = parse_mm_node(a.abp_cdo)
        binds = ref.bindings(root)
        data["mm_rules"] = {
            "source": {"abp_pseudo": rel(a.abp_pseudo), "abp_pseudo_sha256": sha256_file(a.abp_pseudo),
                       "abp_cdo": rel(a.abp_cdo), "abp_cdo_sha256": sha256_file(a.abp_cdo)},
            "blend_time": parse_blend_time(a.abp_pseudo, binds),
            "interrupt": parse_interrupt(a.abp_pseudo, binds),
            "mm_node": mm_node,
        }
        data["blend_profile"] = export_blend_profile(a.skeletons, mm_node.get("BlendProfile"), a.bonemap)
        data["blend_profile"]["skeletons_sha256"] = sha256_file(a.skeletons)

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    tmp = a.out + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        json.dump(data, f, indent=1, ensure_ascii=False)
        f.write("\n")
    os.replace(tmp, a.out)
    ndb = len({r["result"]["name"] for t in tables for r in t["rows"]
               if r["result"] and r["result"]["kind"] == "database"} |
              {t["fallback"]["name"] for t in tables if t["fallback"] and t["fallback"]["kind"] == "database"})
    print("tables %d, inputs %d, databases %d, overrides applied %d -> %s"
          % (len(tables), len(inputs), ndb, len(applied), os.path.abspath(a.out)))
    if "mm_rules" in data:
        print("blend_time:", json.dumps({k: v for k, v in data["mm_rules"]["blend_time"].items() if k != "rule"}))
        print("blend_profile: %s (%s), %d entries, %d mapped to %s"
              % (data["blend_profile"]["name"], data["blend_profile"]["mode"], len(data["blend_profile"]["entries"]),
                 sum(1 for e in data["blend_profile"]["entries"] if e["cp"]), data["blend_profile"]["target_rig"]))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except gcr.ChooserRefError as e:
        print("ERROR:", e, file=sys.stderr)
        sys.exit(2)
