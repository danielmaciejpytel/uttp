#!/usr/bin/env python3
"""Export the source pack traversal chooser into a normalized decision table json.

Input (source-pack-derived, outside git): Extracted\\sourcepack\\sourcepack\\choosers.json (+ blueprints.json for enum names).
Output (source-pack-derived, outside git): Data\\traversal\\traversal_table.json, consumed by traversal_ref.py and by
the C++ runtime src\\mm\\BaseTraversal.hpp (utpp::mm::traversal::DecisionTable). No Neutral number, table, montage or
enum name is written in this file: everything is read from the json files given on the command line.

Optional corrections (--corrections json, also source-pack-derived data outside git) patch known data errors before
export, e.g. a float cell bound typo or byte-identical duplicate rows (analysis 3.14 pt 4). Every applied
correction is recorded in the output ("corrections_applied") so a raw export stays reproducible
(run without --corrections).

Normalized format (version 1):
  {"format": "utpp_traversal_table", "version": 1, "root": <table index>,
   "inputs": [names], "outputs": [names],
   "action_enum": [{"value": v, "name": display}], "action_none": <value of the first enum entry>,
   "movement_mode_enum": [...engine EMovementMode names...],
   "tables": [{"name": n, "columns": [col...], "rows": [{"result": res}], "fallback": res|null}],
   "source": {...sha256...}, "corrections_applied": [...]}
  col = {"kind": "bool", "input": name, "cells": ["true"|"false"|"any"]}
      | {"kind": "float", "input": name, "cells": [[min, max, no_min, no_max]]}   (bounds rounded to float32)
      | {"kind": "multienum", "input": name, "cells": [mask]}                      (0 = any)
      | {"kind": "output_enum", "output": name, "cells": [value]}
  res = {"type": "nested", "table": index} | {"type": "asset", "asset": short_name} | {"type": "none"}

UE semantics this format feeds (C = UE_5.8\\Engine\\Plugins\\Chooser\\Source\\Chooser): see traversal_ref.py.
"""
import argparse
import hashlib
import json
import os
import re
import struct
import sys

# UE engine fact (not source pack data): EMovementMode order, Engine\Source\Runtime\Engine\Classes\Engine\EngineTypes.h.
ENGINE_MOVEMENT_MODES = ["MOVE_None", "MOVE_Walking", "MOVE_NavWalking", "MOVE_Falling", "MOVE_Swimming",
                         "MOVE_Flying", "MOVE_Custom"]


def f32(x):
    return struct.unpack("<f", struct.pack("<f", float(x)))[0]


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def bstr(s, default=False):
    if s is None:
        return default
    if isinstance(s, bool):
        return s
    return str(s).strip().lower() == "true"


def member_name(binding):
    """Last binding element without the BP '_<n>_<GUID>' suffix."""
    if not binding:
        return ""
    m = re.match(r"^(.*?)_\d+_[0-9A-F]{32}$", binding[-1])
    return m.group(1) if m else binding[-1]


def short_asset(path):
    return path.rsplit(".", 1)[-1] if path else path


def load_action_enum(blueprints_path, enum_path):
    with open(blueprints_path, encoding="utf-8") as f:
        bp = json.load(f)
    for e in bp.get("enums", []):
        if e.get("path") == enum_path:
            return [{"value": int(x["value"]), "name": x.get("display_name") or x["internal_name"],
                     "internal": x["internal_name"]} for x in e["entries"] if not x.get("is_max")]
    raise SystemExit("enum %s not found in %s" % (enum_path, blueprints_path))


def resolve_output_value(cell, enum_entries, warnings, where):
    value = int(cell.get("Value", 0))
    vn = cell.get("ValueName") or ""
    internal = vn.split("::", 1)[-1].split(" ", 1)[0] if vn else ""
    for e in enum_entries:
        if internal and e["internal"] == internal:
            if e["value"] != value:
                warnings.append("%s: ValueName %s -> %d, stored %d (editor PostLoad uses ValueName)"
                                % (where, internal, e["value"], value))
            return e["value"]
    return value


def export(choosers_path, blueprints_path, root_name, corrections=None):
    with open(choosers_path, encoding="utf-8") as f:
        ch = json.load(f)
    root_raw = None
    for t in ch["tables"]:
        if t["table"].get("name") == root_name or t["path"].endswith("." + root_name):
            root_raw = t["table"]
    if root_raw is None:
        raise SystemExit("root chooser %s not found" % root_name)
    nested = {n["name"]: n for n in root_raw.get("nested") or []}
    tables, index_of, warnings = [], {}, []
    inputs, outputs = [], []
    action_enum = None

    def add(raw):
        name = raw["name"]
        if name in index_of:
            return index_of[name]
        idx = len(tables)
        index_of[name] = idx
        tables.append(None)
        nrows = len(raw.get("results") or [])
        cols = []
        for c in raw["columns"]:
            if c.get("disabled"):
                continue
            nm = member_name(c.get("input_binding"))
            cells_raw = c.get("row_values") or []
            where = "%s col %s" % (name, c.get("index"))
            t = c["type"]
            if t == "BoolColumn":
                cells = [{"MatchTrue": "true", "MatchFalse": "false", "MatchAny": "any"}[v] for v in cells_raw]
                cols.append({"kind": "bool", "input": nm, "cells": cells})
            elif t == "FloatRangeColumn":
                if bstr((c.get("settings") or {}).get("bWrapInput")):
                    raise SystemExit("%s: wrapped float column not supported by the traversal format" % where)
                cells = []
                for v in cells_raw:
                    v = v if isinstance(v, dict) else {}
                    cells.append([f32(v.get("Min", 0.0)), f32(v.get("Max", 0.0)),
                                  bstr(v.get("bNoMin")), bstr(v.get("bNoMax"))])
                cols.append({"kind": "float", "input": nm, "cells": cells})
            elif t == "MultiEnumColumn":
                cols.append({"kind": "multienum", "input": nm,
                             "cells": [int((v or {}).get("Value", 0)) for v in cells_raw]})
            elif t == "OutputEnumColumn":
                nonlocal_enum = load_action_enum(blueprints_path, c.get("input_enum"))
                tables_enum[0] = nonlocal_enum
                cols.append({"kind": "output_enum", "output": nm,
                             "cells": [resolve_output_value(v or {}, nonlocal_enum, warnings, "%s row %d" % (where, i))
                                       for i, v in enumerate(cells_raw)]})
                if nm not in outputs:
                    outputs.append(nm)
                continue
            else:
                raise SystemExit("%s: column type %s not supported" % (where, t))
            if nm not in inputs:
                inputs.append(nm)
        disabled = {r["row"] for r in raw.get("rows") or [] if r.get("disabled")}
        rows = []
        for i, r in enumerate(raw.get("results") or []):
            if i in disabled:
                rows.append({"result": {"type": "none"}, "disabled": True})
                continue
            rows.append({"result": conv_result(r)})
        tables[idx] = {"name": name, "columns": cols, "rows": rows,
                       "fallback": conv_result(raw.get("fallback_result")) if raw.get("fallback_result") else None,
                       "nrows": nrows}
        return idx

    def conv_result(r):
        if not r:
            return {"type": "none"}
        if r.get("type") == "NestedChooser":
            return {"type": "nested", "table": add(nested[r["Chooser"]])}
        if r.get("type") == "AssetChooser":
            return {"type": "asset", "asset": short_asset(r.get("Asset"))} if r.get("Asset") else {"type": "none"}
        raise SystemExit("result type %s not supported" % r.get("type"))

    tables_enum = [None]
    root = add(root_raw)
    action_enum = tables_enum[0]
    applied = apply_corrections(tables, corrections or [], warnings)
    out = {
        "format": "utpp_traversal_table", "version": 1, "root": root,
        "inputs": inputs, "outputs": outputs,
        "action_enum": [{"value": e["value"], "name": e["name"]} for e in action_enum],
        "action_none": action_enum[0]["value"],
        "movement_mode_enum": ENGINE_MOVEMENT_MODES,
        "tables": tables,
        "source": {"choosers": os.path.basename(choosers_path), "choosers_sha256": sha256_file(choosers_path),
                   "blueprints_sha256": sha256_file(blueprints_path), "root_name": root_name},
        "corrections_applied": applied, "warnings": warnings,
    }
    return out


def apply_corrections(tables, corrections, warnings):
    """corrections: list of {"op": "set_float_bound", "table", "rows", "input", "field": "min"|"max", "from", "to"}
    and {"op": "drop_duplicate_rows", "table"} (later rows whose cells and result equal an earlier row)."""
    by_name = {t["name"]: t for t in tables}
    applied = []
    for c in corrections:
        t = by_name[c["table"]]
        if c["op"] == "set_float_bound":
            col = next(x for x in t["columns"] if x["kind"] == "float" and x["input"] == c["input"])
            k = 0 if c["field"] == "min" else 1
            for r in c["rows"]:
                if f32(col["cells"][r][k]) != f32(c["from"]):
                    raise SystemExit("correction %r: row %d has %r" % (c, r, col["cells"][r][k]))
                col["cells"][r][k] = f32(c["to"])
            applied.append(c)
        elif c["op"] == "drop_duplicate_rows":
            seen, keep = set(), []
            for i, row in enumerate(t["rows"]):
                key = json.dumps([[col["cells"][i] for col in t["columns"]], row["result"]], sort_keys=True)
                if key in seen:
                    keep.append(False)
                else:
                    seen.add(key)
                    keep.append(True)
            dropped = [i for i, k in enumerate(keep) if not k]
            for col in t["columns"]:
                col["cells"] = [v for v, k in zip(col["cells"], keep) if k]
            t["rows"] = [r for r, k in zip(t["rows"], keep) if k]
            t["nrows"] = len(t["rows"])
            applied.append(dict(c, dropped_rows=dropped))
        else:
            raise SystemExit("unknown correction op %s" % c["op"])
    return applied


def main(argv=None):
    root_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--choosers", default=os.path.join(root_dir, "Extracted", "sourcepack", "sourcepack", "choosers.json"))
    ap.add_argument("--blueprints", default=None, help="default: blueprints.json next to choosers.json")
    ap.add_argument("--root", required=True, help="root traversal chooser name")
    ap.add_argument("--corrections", default=None, help="json list of corrections (data, outside git)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    bp = a.blueprints or os.path.join(os.path.dirname(os.path.abspath(a.choosers)), "blueprints.json")
    corr = None
    if a.corrections:
        with open(a.corrections, encoding="utf-8") as f:
            corr = json.load(f)
    out = export(a.choosers, bp, a.root, corr)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    print("wrote %s: %d tables, inputs %s, corrections %d, warnings %d"
          % (a.out, len(out["tables"]), ",".join(out["inputs"]), len(out["corrections_applied"]), len(out["warnings"])))
    for w in out["warnings"]:
        print("WARNING:", w)
    return 0


if __name__ == "__main__":
    sys.exit(main())
