"""source pack slide extension of the v2 baker (optional: without --slide-spec nothing here runs).

Two pure parts, used by mmbake_motion.bake_v2 and tested in test_slide_spec.py:

  merge_into_posesearch(ps, spec, dump_dir)
      appends the slide schemas / databases / normalization sets to the parsed posesearch.json (same record format as
      the source pack dump: the PoseSearch data dumps meta/_data__*.json of the source pack export are the schema and
      database records, the entries come from the spec) and returns the authored notify windows per clip.
  apply_chooser(data, spec, db_index)
      adds the slide model to an exported chooser table (export_chooser): enum value Sliding of MovementMode, the
      inputs MovementDirection / MovementMode_LastFrame, a Slide table, the Sliding root row and the exit rows
      (MovementMode_LastFrame == Sliding) of the movement tables; writes data["slide"] (FORMAT.txt, slide section).

Spec format "utpp-mmbake-v2-slide-spec" version 1, see make_slide_spec.py.
"""
from __future__ import annotations

import copy
import json
import os

SPEC_FORMAT = "utpp-mmbake-v2-slide-spec"
SPEC_VERSION = 1
SLIDE_SECTION_VERSION = 1
ANY_ENUM = ["MatchAny", 0]
ANY_FLOAT = [0.0, 999.0, True, True]      # [min, max, no_min, no_max] as exported by export_chooser


def load_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _gpath(kind, name):
    return f"/GASPALS/Slide/{name}.{name}"


def merge_into_posesearch(ps, spec, dump_dir):
    """Mutates ps. Returns (extra_db_names, extra_schema_names, windows) with windows = {clip: {"exclude": [[a, b]], "block": [[a, b]]}}."""
    if spec.get("format") != SPEC_FORMAT or int(spec.get("version", 0)) != SPEC_VERSION:
        raise SystemExit(f"slide spec: format/version {spec.get('format')}/{spec.get('version')} != {SPEC_FORMAT}/{SPEC_VERSION}")
    have_schema = {s["_path"].rsplit("/", 1)[-1].split(".", 1)[-1] for s in ps["schemas"]}
    have_db = {d["_path"].rsplit("/", 1)[-1].split(".", 1)[-1] for d in ps["databases"]}
    schemas = []
    for sn in spec["schemas"]:
        if sn in have_schema:
            raise SystemExit(f"slide spec: schema {sn} already in posesearch.json")
        rec = load_json(os.path.join(dump_dir, f"_data__{sn}.json"))
        rec = copy.deepcopy(rec)
        rec["_path"] = _gpath("schema", sn)
        ps["schemas"].append(rec)
        schemas.append(sn)
    nset_of = {}
    for ns, members in spec["normalization_sets"].items():
        ps["normalization_sets"].append({"_class": "PoseSearchNormalizationSet", "_path": _gpath("nset", ns),
                                         "databases": [_gpath("db", m) for m in members]})
        for m in members:
            nset_of[m] = ns
    dbs, windows = [], {}
    for d in spec["databases"]:
        name = d["name"]
        if name in have_db:
            raise SystemExit(f"slide spec: database {name} already in posesearch.json")
        rec = copy.deepcopy(load_json(os.path.join(dump_dir, f"_data__{d['dump']}.json")))
        rec["_path"] = _gpath("db", name)
        rec["schema"] = _gpath("schema", d["schema"])
        if d.get("continuing_pose_cost_bias") is not None:      # design override of the 1.2 value (make_slide_spec.py [P])
            rec["continuing_pose_cost_bias"] = float(d["continuing_pose_cost_bias"])
        ns = d.get("normalization_set") or nset_of.get(name)
        rec["normalization_set"] = _gpath("nset", ns) if ns else None
        ents = []
        for i, e in enumerate(d["entries"]):
            ents.append({"index": i, "asset": e["clip"], "asset_class": "AnimSequence", "enabled": True,
                         "disable_reselection": bool(e.get("disable_reselection", True)), "mirror_option": "UnmirroredOnly",
                         "sampling_range": [float(x) for x in e.get("sampling_range", [0.0, 0.0])],
                         "looping": None if e.get("looping") is None else bool(e["looping"])})
            w = windows.setdefault(e["clip"], {"exclude": [], "block": []})
            for kind in ("exclude", "block"):
                for r in e.get(kind, []):
                    if [float(r[0]), float(r[1])] not in w[kind]:
                        w[kind].append([float(r[0]), float(r[1])])
        rec["entries"] = ents
        ps["databases"].append(rec)
        dbs.append(name)
    return dbs, schemas, windows


def _enum_cell(value):
    return ["MatchEqual", int(value)]


def _any_cell(kind):
    if kind == "bool":
        return "MatchAny"
    if kind == "enum":
        return list(ANY_ENUM)
    if kind == "float":
        return list(ANY_FLOAT)
    raise SystemExit(f"slide chooser: column kind {kind} has no match-any cell")


def _col(table, key):
    for c in table["columns"]:
        if c["key"] == key:
            return c
    return None


def _add_row(table, cells, result):
    """cells: {key: cell}; every other column gets its match-any cell. The columns of cells must exist."""
    nrows = len(table["rows"])
    for key, cell in cells.items():
        if _col(table, key) is None:
            raise KeyError(key)
    for c in table["columns"]:
        c["cells"].append(cells[c["key"]] if c["key"] in cells else _any_cell(c["kind"]))
    table["rows"].append({"disabled": False, "result": result})
    return nrows


def apply_chooser(data, spec, db_index):
    """Slide model of the chooser table. data is the export_chooser dict (mutated and returned). The input list is
    extended by MovementDirection / MovementMode_LastFrame; MovementMode gets the value Sliding."""
    ch = spec["chooser"]
    mm, md, ml = ch["movement_mode_input"], ch["direction_input"], ch["last_mode_input"]
    sliding = int(ch["sliding_value"])
    dirs = ch["direction_values"]
    inputs = data["inputs"]
    if not any(i["key"] == mm for i in inputs):
        raise SystemExit(f"slide chooser: input {mm} not in the table")
    inp_mm = next(i for i in inputs if i["key"] == mm)
    if any(v["value"] == sliding for v in inp_mm["enum"]["values"]):
        raise SystemExit(f"slide chooser: MovementMode value {sliding} already used")
    inp_mm["enum"]["values"].append({"value": sliding, "name": "Sliding", "display": "Sliding"})
    for key, vals in ((md, [{"value": int(v), "name": k, "display": k} for k, v in dirs.items()]),
                      (ml, copy.deepcopy(inp_mm["enum"]["values"]))):
        if any(i["key"] == key for i in inputs):
            raise SystemExit(f"slide chooser: input {key} already in the table")
        inputs.append({"key": key, "kind": "enum", "property_type": "enum", "display": [], "enum": {"name": key, "values": vals}})
    keys = [i["key"] for i in inputs]
    tables = data["tables"]

    def dbref(name):
        if name not in db_index:
            raise SystemExit(f"slide chooser: database {name} not baked")
        return {"kind": "database", "name": name, "db": db_index[name]}

    def new_col(key):
        return {"input": keys.index(key), "key": key, "kind": "enum", "disabled": False, "cells": []}

    # Slide table: Forward -> FeetOut, other -> KneesOut (MovementDirection)
    st = {"label": ch["slide_table_label"], "columns": [new_col(md)], "rows": [], "fallback": None}
    for r in ch["slide_table"]:
        cell = ["MatchNotEqual", int(dirs["Forward"])] if r["direction"] == "Other" else _enum_cell(dirs[r["direction"]])
        st["columns"][0]["cells"].append(cell)
        st["rows"].append({"disabled": False, "result": dbref(r["db"])})
    tables.append(st)
    st_idx = len(tables) - 1
    n_root = n_exit = 0
    exit_log = []
    for fam in ch["families"]:
        root = next((t for t in tables if t["label"] == fam), None)
        if root is None or _col(root, mm) is None:
            raise SystemExit(f"slide chooser: mover table {fam} with column {mm} not found")
        _add_row(root, {mm: _enum_cell(sliding)}, {"kind": "table", "table": st_idx})
        n_root += 1
        for ex in ch["exit_rows"]:
            t = next((t for t in tables if t["label"] == f"{fam}/{ex['table']}"), None)
            if t is None:
                raise SystemExit(f"slide chooser: table {fam}/{ex['table']} not found")
            if _col(t, ml) is None:
                col = new_col(ml)
                col["cells"] = [list(ANY_ENUM) for _ in t["rows"]]
                t["columns"].append(col)
            _add_row(t, {ml: _enum_cell(sliding)}, dbref(ex["db"]))
            exit_log.append({"table": t["label"], "row": len(t["rows"]) - 1, "db": ex["db"]})
            n_exit += 1
    data["slide"] = {
        "version": SLIDE_SECTION_VERSION,
        "movement_mode_input": mm, "sliding_value": sliding,
        "direction_input": md, "direction_values": dirs,
        "last_mode_input": ml,
        "slide_table": st_idx, "families": list(ch["families"]),
        "databases": {n: db_index[n] for n in ch["databases"]},
        "exit_rows": exit_log, "root_rows": n_root,
        "semantics": {
            "root_row": "MovementMode == Sliding -> table slide_table (union of the matching rows, as every table)",
            "slide_table": "Forward -> FeetOut; Other (not Forward) -> KneesOut",
            "exit_rows": "appended to the movement tables: MovementMode_LastFrame == Sliding, every other column MatchAny; "
                         "MovementMode is OnGround at that time, so the row joins the regular rows of the table (union)",
            "inputs": "MovementDirection and MovementMode_LastFrame are appended to inputs; MovementMode gains value Sliding",
        },
    }
    return data
