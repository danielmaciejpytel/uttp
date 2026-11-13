#!/usr/bin/env python3
"""Tests for motion_chooser_ref.py.

A. UE Chooser semantics on small synthetic tables (no Neutral data): float32 bounds, closed/open ranges,
   missing cells, disabled rows/columns, enum comparisons, wrap, passthrough, fallback chain and the
   Failed status after a fallback (Chooser.cpp:763-797, 824), AllResults vs FirstResult.
B. 100% agreement with the hand-written oracle Research\\inventory\\chooser_oracle.json (written independently of
   the reference evaluator; the test waits for it: poll every --poll-seconds up to --wait-minutes).
C. Research\\inventory\\chooser_vectors.json is current: same choosers.json hash, same vector count,
   every vector re-evaluates to the stored databases / first result, every reachable row selected.
D. Every database the grid can emit exists in posesearch.json (stands in for the IsA(PoseSearchDatabase)
   type check of EvaluateObjectChooserBaseMulti, which needs asset classes).

Exit code 0 only if every check passes.
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, HERE)
import motion_chooser_ref as g  # noqa: E402

fails = 0
passes = 0


def check(name, ok, detail=""):
    global fails, passes
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ""))
    if ok:
        passes += 1
    else:
        fails += 1


# ----------------------------------------------------------------------------- A. synthetic semantics

def _fcol(key, rows, wrap=False, disabled=False):
    return {"type": "FloatRangeColumn", "input_type": "FloatContextProperty", "input_binding": key.split("."),
            "disabled": disabled, "row_values": rows,
            "settings": {"bWrapInput": "True" if wrap else "False", "MinValue": "-180.000000", "MaxValue": "180.000000"}}


def _bcol(key, rows, disabled=False):
    return {"type": "BoolColumn", "input_type": "BoolContextProperty", "input_binding": [key],
            "disabled": disabled, "row_values": rows}


def _ecol(key, rows):
    return {"type": "EnumColumn", "input_type": "EnumContextProperty", "input_binding": [key],
            "input_enum": "/T/E_Test.E_Test", "disabled": False, "row_values": rows}


def _asset(name):
    return {"type": "AssetChooser", "Asset": "/T/%s.%s" % (name, name)} if name else {"type": "AssetChooser", "Asset": None}


def _table(name, columns, results, disabled=None, fallback=None, nested=()):
    for i, c in enumerate(columns):
        c["index"] = i
    return {"name": name, "columns": columns, "results": results,
            "rows": [{"row": i, "disabled": bool(disabled and disabled[i]), "result": r} for i, r in enumerate(results)],
            "fallback_result": fallback, "nested": list(nested)}


def _ref(root_table, prop_types):
    data = {"tables": [{"path": "/T/Root.Root", "table": root_table}], "proxy_tables": [], "proxy_assets": []}
    # path only anchors the (absent) sibling jsons; enums come from the cells, float types from prop_types
    return g.ChooserRef(os.path.join(HERE, "__synthetic__", "choosers.json"), data=data, root="Root",
                        enums_json=os.path.join(HERE, "__synthetic__", "none.json"), prop_types=prop_types)


def synthetic_tests():
    # A1: float32 bounds. Min "1.2" is stored as float32; a double input 1.2 is below it, a float input equals it.
    t = _table("Root", [_fcol("X", [{"Min": "1.200000", "Max": "5.000000"}])], [_asset("A")])
    r_dbl = _ref(t, {"X": "double"})
    r_flt = _ref(json.loads(json.dumps(t)), {"X": "float"})
    check("A1 float32 bound vs double input 1.2 (FloatRangeColumn.h:47-51, cpp:100)",
          r_dbl.evaluate({"X": 1.2}).names == [] and r_dbl.evaluate({"X": 1.2000001}).names == ["A"])
    check("A1 float32 bound vs float input 1.2 (TimeToLand case)", r_flt.evaluate({"X": 1.2}).names == ["A"])

    # A2: closed ranges, defaults of a delta-exported cell, bNoMax ignores Max, short RowValues array.
    t = _table("Root", [_fcol("X", [{"Max": "20.000000"}, {"Min": "20.000000", "Max": "999.000000", "bNoMax": "True"},
                                    [], {"bNoMin": "True", "bNoMax": "True"}])],
               [_asset("R0"), _asset("R1"), _asset("R2"), _asset("R3"), _asset("R4")])
    r = _ref(t, {"X": "double"})
    got = {x: r.evaluate({"X": x}).names for x in (-0.5, 0.0, 20.0, 5000.0)}
    check("A2 closed ranges / defaults / bNoMax / missing cell row filtered",
          got == {-0.5: ["R3"], 0.0: ["R0", "R2", "R3"], 20.0: ["R0", "R1", "R3"], 5000.0: ["R1", "R3"]}, str(got))

    # A3: disabled row skipped, disabled column ignored, bool MatchFalse/MatchTrue/MatchAny.
    t = _table("Root", [_bcol("B", ["MatchFalse", "MatchTrue", "MatchAny"]),
                        _bcol("C", ["MatchTrue", "MatchTrue", "MatchTrue"], disabled=True)],
               [_asset("F"), _asset("T"), _asset("Any")], disabled=[False, False, True])
    r = _ref(t, {})
    check("A3 bool cells, disabled row and disabled column",
          r.evaluate({"B": False, "C": False}).names == ["F"] and r.evaluate({"B": True, "C": False}).names == ["T"])

    # A4: enum MatchEqual / MatchNotEqual / MatchAny; absent Value = 0.
    t = _table("Root", [_ecol("E", [{"ValueName": "E_Test::NewEnumerator0 (Alpha)"},
                                    {"ValueName": "E_Test::NewEnumerator0 (Alpha)", "Comparison": "MatchNotEqual"},
                                    {"ValueName": "E_Test::NewEnumerator3 (Beta)", "Comparison": "MatchAny", "Value": "1"},
                                    {"ValueName": "E_Test::NewEnumerator3 (Beta)", "Value": "1"}])],
               [_asset("Eq"), _asset("Ne"), _asset("Any"), _asset("EqB")])
    r = _ref(t, {})
    check("A4 enum comparisons (EnumColumn.cpp:71-87)",
          r.evaluate({"E": "Alpha"}).names == ["Eq", "Any"] and r.evaluate({"E": "Beta"}).names == ["Ne", "Any", "EqB"]
          and r.evaluate({"E": 1}).names == ["Ne", "Any", "EqB"])

    # A5: wrap input with a reversed range (FloatRangeColumn.cpp:70-91).
    t = _table("Root", [_fcol("Ang", [{"Min": "135.000000", "Max": "-135.000000"}, {"Min": "-45.000000", "Max": "45.000000"}],
                              wrap=True)], [_asset("Back"), _asset("Front")])
    r = _ref(t, {"Ang": "double"})
    check("A5 wrapped reversed range", r.evaluate({"Ang": 190.0}).names == ["Back"]
          and r.evaluate({"Ang": 370.0}).names == ["Front"] and r.evaluate({"Ang": 90.0}).names == [])

    # A6: unreadable input = passthrough (FloatRangeColumn.cpp:109-113); missing input = error in strict mode.
    t = _table("Root", [_fcol("X", [{"Max": "1.000000"}, {"Min": "2.000000", "Max": "3.000000"}])], [_asset("A"), _asset("B")])
    r = _ref(t, {"X": "double"})
    try:
        r.evaluate({})
        strict = False
    except g.MissingInput:
        strict = True
    check("A6 passthrough for unreadable input, error for missing input",
          r.evaluate({"X": None}).names == ["A", "B"] and strict)

    # A7: fallback chain. Nested table: no row passes -> its fallback emits FB_N, but the nested table still
    # returns Failed (Chooser.cpp:824), so the root row counts as failed and the root fallback runs too.
    nested = _table("N", [_bcol("B", ["MatchTrue"])], [_asset("N0")], fallback=_asset("FB_N"))
    root = _table("Root", [_bcol("B", ["MatchAny", "MatchTrue"])],
                  [{"type": "NestedChooser", "Chooser": "N"}, _asset("R1")], fallback=_asset("FB_Root"), nested=[nested])
    r = _ref(root, {})
    all_false = r.evaluate({"B": False})
    first_false = r.evaluate({"B": False}, mode="first")
    all_true = r.evaluate({"B": True})
    check("A7 fallback only when no row succeeded; Failed after fallback propagates (AllResults)",
          all_false.names == ["FB_N", "FB_Root"] and all_false.status == g.FAILED and all_true.names == ["N0", "R1"],
          "%s / %s" % (all_false.names, all_true.names))
    check("A7 FirstResult stops at the first emitted object", first_false.names == ["FB_N"]
          and r.evaluate({"B": True}, mode="first").names == ["N0"])

    # A8: a null asset is Failed (IObjectChooser.h:155-163) and therefore enables the fallback.
    t = _table("Root", [], [_asset(None)], fallback=_asset("FB"))
    r = _ref(t, {})
    check("A8 null asset row fails -> fallback", r.evaluate({}).names == ["FB"])


# ----------------------------------------------------------------------------- B. oracle

def wait_for(path, minutes, poll):
    deadline = time.time() + minutes * 60
    while not os.path.isfile(path):
        if time.time() >= deadline:
            return False
        print("waiting for %s (poll %ds, up to %d min)" % (path, poll, minutes), flush=True)
        time.sleep(poll)
    return True


def oracle_tests(ref, oracle_path):
    with open(oracle_path, encoding="utf-8") as f:
        oracle = json.load(f)
    vectors = oracle.get("vectors", [])
    root = ref.find_root()
    known = set(ref.bindings(root))
    agree, order_diff, problems = 0, [], []
    for v in vectors:
        vid = v.get("id")
        unknown = set(v["inputs"]) - known
        if unknown:
            problems.append("%s: unknown inputs %s" % (vid, sorted(unknown)))
            continue
        res = ref.evaluate(v["inputs"])
        exp = v["expected_databases"]
        if set(res.names) == set(exp) and len(res.names) == len(set(res.names)):
            agree += 1
            if res.names != exp:
                order_diff.append(vid)
        else:
            problems.append("%s: reference %s, oracle %s (trace %s)" % (vid, res.names, exp, res.trace))
        if v.get("expected_other", {}) != {}:
            problems.append("%s: expected_other %r but the reachable tables have no output columns" % (vid, v["expected_other"]))
    for p in problems:
        print("    DISAGREE", p)
    check("B oracle agreement (database sets)", agree == len(vectors) and not problems and len(vectors) > 0,
          "%d/%d" % (agree, len(vectors)))
    print("    info: order differs from the oracle list in %d vectors%s" % (len(order_diff), (": " + ", ".join(order_diff)) if order_diff else ""))
    outputs = [c.where() for t in ref.reachable(root) for c in t.columns if c.kind == "output" and not c.disabled]
    check("B reachable tables have no output columns (expected_other is {})", not outputs, ", ".join(outputs))


# ----------------------------------------------------------------------------- C/D. grid file

def grid_tests(ref, vectors_path, posesearch_path):
    if not os.path.isfile(vectors_path):
        check("C grid file exists", False, vectors_path)
        return
    with open(vectors_path, encoding="utf-8") as f:
        grid = json.load(f)
    check("C grid built from the current choosers.json",
          grid["sources"]["choosers_json_sha256"] == g.sha256_file(ref.choosers_path))
    root_t, binds, cands, regen = ref.grid(grid["sources"]["root"], 0.01)
    check("C grid vector count matches a fresh generation", grid["count"] == len(grid["vectors"]) == len(regen),
          "%d stored, %d regenerated" % (len(grid["vectors"]), len(regen)))
    bad = []
    for i, v in enumerate(grid["vectors"]):
        r = ref.evaluate(v["inputs"], root_t)
        f = ref.evaluate(v["inputs"], root_t, mode="first")
        if r.names != v["databases"] or (f.names[0] if f.names else None) != v["first_result"]:
            bad.append(i)
    check("C every stored vector re-evaluates to its databases and first_result", not bad,
          "mismatching indices %s" % bad[:10] if bad else "%d vectors" % len(grid["vectors"]))
    never = [k for k, n in grid["row_coverage"].items() if n == 0]
    check("C every reachable row (and fallback) selected by at least one vector", not never, ", ".join(never))
    if not os.path.isfile(posesearch_path):
        check("D posesearch.json present", False, posesearch_path)
        return
    with open(posesearch_path, encoding="utf-8") as f:
        dbs = {d["_path"] for d in json.load(f)["databases"]}
    emitted = set()
    for t in ref.reachable(root_t):
        for r in t.results + ([t.fallback] if t.fallback else []):
            if r.kind == "asset":
                emitted.add(r.target)
    missing = sorted(g.short_name(p) for p in emitted if p not in dbs)
    check("D every reachable asset result is a PoseSearchDatabase in posesearch.json", not missing,
          "%d assets" % len(emitted) if not missing else ", ".join(missing))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--choosers", default=os.path.join(ROOT, "Extracted", "sourcepack", "sourcepack", "choosers.json"))
    ap.add_argument("--posesearch", default=os.path.join(ROOT, "Extracted", "sourcepack", "sourcepack", "posesearch.json"))
    ap.add_argument("--oracle", default=os.path.join(ROOT, "Research", "inventory", "chooser_oracle.json"))
    ap.add_argument("--vectors", default=os.path.join(ROOT, "Research", "inventory", "chooser_vectors.json"))
    ap.add_argument("--wait-minutes", type=float, default=45)
    ap.add_argument("--poll-seconds", type=float, default=60)
    a = ap.parse_args()

    synthetic_tests()
    ref = g.ChooserRef(a.choosers)
    for w in ref.warnings:
        print("    warning:", w)
    if wait_for(a.oracle, a.wait_minutes, a.poll_seconds):
        oracle_tests(ref, a.oracle)
    else:
        check("B oracle file appeared within %g min" % a.wait_minutes, False, a.oracle)
    grid_tests(ref, a.vectors, a.posesearch)
    print("passed %d, failed %d" % (passes, fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
