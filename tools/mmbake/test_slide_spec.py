"""Tests of slide_spec.py (merge into posesearch + chooser model) with synthetic data, no Neutral content, plus a mutation guard
that the C++ oracle test (utpp_mm_chooser_slide_test) fails when the slide chooser data is damaged.

python test_slide_spec.py [--exe utpp_mm_chooser_slide_test.exe --slide a.chooser.json --base b.chooser.json --oracle o.json]
Every check prints passed / failed / not run; exit 0 only if nothing failed.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import slide_spec as ss  # noqa: E402

RESULTS = []


def check(name, ok, info=""):
    RESULTS.append((name, ok))
    print(("passed " if ok else "FAILED ") + name + (" " + info if info else ""))


def base_chooser():
    def col(key, kind, cells, **kw):
        c = {"input": 0, "key": key, "kind": kind, "disabled": False, "cells": cells}
        c.update(kw)
        return c
    mm = {"key": "MovementMode", "kind": "enum", "property_type": "enum", "display": [],
          "enum": {"name": "E", "values": [{"value": 0, "name": "OnGround", "display": ""}, {"value": 1, "name": "InAir", "display": ""}]}}
    sp = {"key": "Speed2D", "kind": "float", "property_type": "double", "display": []}
    root = {"label": "FAM", "columns": [col("MovementMode", "enum", [["MatchEqual", 0], ["MatchEqual", 1]])],
            "rows": [{"disabled": False, "result": {"kind": "table", "table": 1}},
                     {"disabled": False, "result": {"kind": "database", "name": "AIR", "db": 1}}], "fallback": None}
    idle = {"label": "FAM/Stand Idles", "columns": [col("Speed2D", "float", [[0.0, 20.0, False, False]], wrap=False, min_value=-1.0, max_value=1.0)],
            "rows": [{"disabled": False, "result": {"kind": "database", "name": "IDLE", "db": 0}}], "fallback": None}
    return {"format": "utpp-chooser-table", "version": 1, "inputs": [mm, sp], "tables": [root, idle], "databases_not_in_file": []}


def spec():
    return {"format": ss.SPEC_FORMAT, "version": 1,
            "chooser": {"movement_mode_input": "MovementMode", "sliding_value": 2, "direction_input": "MovementDirection",
                        "direction_values": {"Forward": 0, "Other": 1}, "last_mode_input": "MovementMode_LastFrame",
                        "slide_table_label": "Slide",
                        "slide_table": [{"direction": "Forward", "db": "F"}, {"direction": "Other", "db": "K"}],
                        "families": ["FAM"], "exit_rows": [{"table": "Stand Idles", "db": "EX"}], "databases": ["F", "K", "EX"]}}


def test_chooser():
    d = base_chooser()
    before = copy.deepcopy(d)
    out = ss.apply_chooser(d, spec(), {"IDLE": 0, "AIR": 1, "F": 2, "K": 3, "EX": 4})
    keys = [i["key"] for i in out["inputs"]]
    check("chooser: inputs appended", keys[-2:] == ["MovementDirection", "MovementMode_LastFrame"], str(keys))
    vals = [v["value"] for v in out["inputs"][0]["enum"]["values"]]
    check("chooser: Sliding value 2 added to MovementMode", vals == [0, 1, 2], str(vals))
    root = out["tables"][0]
    check("chooser: root row Sliding -> Slide table", root["columns"][0]["cells"][-1] == ["MatchEqual", 2] and root["rows"][-1]["result"] == {"kind": "table", "table": 2})
    st = out["tables"][2]
    check("chooser: slide table rows", [r["result"]["db"] for r in st["rows"]] == [2, 3] and
          st["columns"][0]["cells"] == [["MatchEqual", 0], ["MatchNotEqual", 0]])
    idle = out["tables"][1]
    last = next(c for c in idle["columns"] if c["key"] == "MovementMode_LastFrame")
    check("chooser: exit row appended with LastFrame == Sliding", last["cells"] == [["MatchAny", 0], ["MatchEqual", 2]] and idle["rows"][-1]["result"]["db"] == 4)
    spd = next(c for c in idle["columns"] if c["key"] == "Speed2D")
    check("chooser: other columns match-any for the exit row", spd["cells"][-1] == [0.0, 999.0, True, True])
    check("chooser: every column has one cell per row", all(len(c["cells"]) == len(t["rows"]) for t in out["tables"] for c in t["columns"]))
    check("chooser: slide section", out["slide"]["version"] == 1 and out["slide"]["slide_table"] == 2 and out["slide"]["databases"]["EX"] == 4)
    check("chooser: base data untouched elsewhere", before["tables"][1]["rows"][0] == out["tables"][1]["rows"][0])
    d2 = base_chooser()
    sp2 = spec()
    sp2["chooser"]["sliding_value"] = 1
    try:
        ss.apply_chooser(d2, sp2, {"IDLE": 0, "AIR": 1, "F": 2, "K": 3, "EX": 4})
        check("chooser: used enum value refused", False)
    except SystemExit:
        check("chooser: used enum value refused", True)


def test_merge():
    with tempfile.TemporaryDirectory() as td:
        for n, rec in (("_data__SCH.json", {"_class": "PoseSearchSchema", "channels": []}),
                       ("_data__DBX.json", {"_class": "PoseSearchDatabase", "base_cost_bias": -0.1, "schema": "x", "normalization_set": "y"})):
            with open(os.path.join(td, n), "w", encoding="utf-8") as f:
                json.dump(rec, f)
        ps = {"schemas": [{"_path": "/a/B.B"}], "databases": [{"_path": "/a/D0.D0"}], "normalization_sets": []}
        sp = {"format": ss.SPEC_FORMAT, "version": 1, "schemas": ["SCH"], "normalization_sets": {"NS": ["PSD_X"]},
              "databases": [{"name": "PSD_X", "dump": "DBX", "schema": "SCH", "entries": [
                  {"clip": "c1", "exclude": [[2.1, 5.0]], "sampling_range": [0, 0], "looping": None},
                  {"clip": "c2", "block": [[0.5, 3.0]], "disable_reselection": False}]}]}
        dbs, schemas, win = ss.merge_into_posesearch(ps, sp, td)
        d = ps["databases"][-1]
        check("merge: names", dbs == ["PSD_X"] and schemas == ["SCH"])
        check("merge: database record from the dump with entries", d["base_cost_bias"] == -0.1 and len(d["entries"]) == 2 and d["entries"][0]["asset"] == "c1"
              and d["entries"][0]["looping"] is None and d["entries"][0]["disable_reselection"] is True and d["entries"][1]["disable_reselection"] is False)
        check("merge: schema and set paths resolve", d["schema"].endswith("SCH.SCH") and d["normalization_set"].endswith("NS.NS")
              and ps["normalization_sets"][0]["databases"][0].endswith("PSD_X.PSD_X"))
        check("merge: windows", win == {"c1": {"exclude": [[2.1, 5.0]], "block": []}, "c2": {"exclude": [], "block": [[0.5, 3.0]]}}, str(win))
        try:
            ss.merge_into_posesearch(ps, sp, td)
            check("merge: duplicate schema refused", False)
        except SystemExit:
            check("merge: duplicate schema refused", True)
        sp["version"] = 2
        try:
            ss.merge_into_posesearch({"schemas": [], "databases": [], "normalization_sets": []}, sp, td)
            check("merge: wrong version refused", False)
        except SystemExit:
            check("merge: wrong version refused", True)


def run_exe(exe, slide, base, oracle):
    r = subprocess.run([os.path.abspath(exe), slide, base, oracle], capture_output=True, text=True)
    return r.returncode, r.stdout


def test_mutation(a):
    if not (a.exe and a.slide and a.base and a.oracle and all(os.path.isfile(x) for x in (a.exe, a.slide, a.base, a.oracle))):
        print("not run: C++ mutation guard (needs --exe --slide --base --oracle)")
        return
    rc, out = run_exe(a.exe, a.slide, a.base, a.oracle)
    check("oracle test passes on the real data", rc == 0, out.strip().splitlines()[-1] if out.strip() else "")
    with open(a.slide, encoding="utf-8") as f:
        data = json.load(f)

    def mutate(name, fn):
        d = copy.deepcopy(data)
        fn(d)
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "m.chooser.json")
            with open(p, "w", encoding="utf-8") as f:
                json.dump(d, f)
            rc2, out2 = run_exe(a.exe, p, a.base, a.oracle)
        check("mutation fails the oracle test: " + name, rc2 != 0)

    def swap_slide(d):
        t = d["tables"][d["slide"]["slide_table"]]
        t["rows"][0]["result"], t["rows"][1]["result"] = t["rows"][1]["result"], t["rows"][0]["result"]

    def drop_exit(d):
        i = next(i for i, t in enumerate(d["tables"]) if t["label"].endswith("/Crouch Moving"))
        t = d["tables"][i]
        t["rows"][-1]["result"] = None

    def wrong_exit_db(d):
        i = next(i for i, t in enumerate(d["tables"]) if t["label"].endswith("/Stand Sprint"))
        d["tables"][i]["rows"][-1]["result"] = copy.deepcopy(d["tables"][i]["rows"][0]["result"])

    def last_any(d):
        i = next(i for i, t in enumerate(d["tables"]) if t["label"].endswith("/Stand Idles"))
        c = next(c for c in d["tables"][i]["columns"] if c["key"] == "MovementMode_LastFrame")
        c["cells"][-1] = ["MatchAny", 0]

    def root_row_gone(d):
        for t in d["tables"]:
            if t["label"] == "CHT_PoseSearchDatabases_Dense":
                t["rows"][-1]["disabled"] = True

    for n, fn in (("slide table rows swapped", swap_slide), ("exit row result removed", drop_exit), ("wrong exit database", wrong_exit_db),
                  ("exit row ignores LastFrame", last_any), ("Dense Sliding root row disabled", root_row_gone)):
        mutate(n, fn)


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe")
    ap.add_argument("--slide")
    ap.add_argument("--base")
    ap.add_argument("--oracle")
    a = ap.parse_args(argv)
    test_chooser()
    test_merge()
    test_mutation(a)
    bad = [n for n, ok in RESULTS if not ok]
    print("passed %d failed %d" % (len(RESULTS) - len(bad), len(bad)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
