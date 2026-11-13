#!/usr/bin/env python3
"""Python reference of the source pack traversal decision: CBP gate + traversal chooser.

Data driven: evaluates the normalized table written by export_traversal_table.py (Data\\traversal\\
traversal_table.json, source-pack-derived, outside git). No Neutral thresholds, montage or table names live here.
The C++ runtime (src\\mm\\BaseTraversal.hpp, DecisionTable::Evaluate) implements the same semantics and is
tested against the same oracle.

UE semantics (C = UE_5.8\\Engine\\Plugins\\Chooser\\Source\\Chooser):
  * UChooserTable::EvaluateChooser, C\\Private\\Chooser.cpp:592-825: rows in order, every filter column
    narrows the list (disabled rows/columns removed at export), then for every surviving row the output
    columns are written BEFORE the row result is evaluated (Chooser.cpp:716-737), so the last surviving
    row wins the output value even when its nested table fails; AllResults keeps going (Continue).
    FallbackResult (and fallback outputs) only when no row succeeded (763-797); a table without rows
    succeeding returns Failed.
  * FBoolColumn (BoolColumn.cpp:29-61), FFloatRangeColumn closed range with float32 bounds compared in
    double (FloatRangeColumn.cpp:55-114), FMultiEnumColumn: bit (1 << value) tested against the row mask,
    mask 0 = any (MultiEnumColumn.cpp:28-60, MultiEnumColumn.h:25-28).
  * Output struct starts at its default: action = first enum entry ("action_none").
CBP gate (pseudo CBP_SandboxCharacter TryTraversalAction):
  * HasFrontLedge false (no hit / no ledge / no room)            -> TraversalCheckFailed;
  * chooser output ActionType == none                             -> TraversalCheckFailed;
  * no montage from the chooser (MotionMatch cast fails)          -> MontageSelectionFailed;
  * otherwise ok: candidate montages (row order, AllResults) go to MotionMatch.

Usage:
  python traversal_ref.py --table Data\\traversal\\traversal_table.json eval --set ObstacleHeight=90 ...
  python traversal_ref.py --table ... grid --out <json>        (boundary grid of input vectors)
  python traversal_ref.py --table ... oracle --oracle Research\\inventory\\traversal_oracle.json
"""
import argparse
import itertools
import json
import os
import struct
import sys

STATUS_OK = "ok"
STATUS_CHECK_FAILED = "check_failed"
STATUS_MONTAGE_FAILED = "montage_failed"
STATUS_SKIP = "skip"
# oracle outcome names (Research\inventory\traversal_oracle.json meta.semantics)
OUTCOME = {STATUS_OK: "traverse", STATUS_CHECK_FAILED: "jump:TraversalCheckFailed",
           STATUS_MONTAGE_FAILED: "jump:MontageSelectionFailed", STATUS_SKIP: "skip"}


def f32(x):
    return struct.unpack("<f", struct.pack("<f", float(x)))[0]


class TraversalTable:
    def __init__(self, data):
        if data.get("format") != "utpp_traversal_table" or data.get("version") != 1:
            raise ValueError("not a utpp_traversal_table v1")
        self.d = data
        self.tables = data["tables"]
        self.root = data["root"]
        self.inputs = data["inputs"]
        self.action_names = {e["value"]: e["name"] for e in data["action_enum"]}
        self.action_values = {e["name"]: e["value"] for e in data["action_enum"]}
        self.action_none = data["action_none"]
        self.mm_names = data["movement_mode_enum"]

    @staticmethod
    def load(path):
        with open(path, encoding="utf-8") as f:
            return TraversalTable(json.load(f))

    def movement_mode_value(self, v):
        if isinstance(v, str):
            names = [n.lower() for n in self.mm_names]
            key = v.lower()
            if key in names:
                return names.index(key)
            if ("move_" + key) in names:
                return names.index("move_" + key)
            return int(v)
        return int(v)

    # ------------------------------------------------------------------ chooser evaluation
    def _filter(self, col, inputs, idx):
        name = col["input"]
        if name not in inputs or inputs[name] is None:
            return list(idx)                                   # passthrough fallback
        v = inputs[name]
        cells = col["cells"]
        k = col["kind"]
        if k == "bool":
            b = bool(v)
            return [i for i in idx if i < len(cells) and (cells[i] == "any" or b == (cells[i] == "true"))]
        if k == "float":
            x = float(v)
            return [i for i in idx if i < len(cells)
                    and (cells[i][2] or x >= cells[i][0]) and (cells[i][3] or x <= cells[i][1])]
        if k == "multienum":
            bit = 1 << (int(v) & 31)
            return [i for i in idx if i < len(cells) and (cells[i] == 0 or (cells[i] & bit) != 0)]
        raise ValueError("column kind %s" % k)

    def _eval(self, ti, inputs, out, montages):
        t = self.tables[ti]
        idx = list(range(len(t["rows"])))
        for col in t["columns"]:
            if col["kind"] == "output_enum":
                continue
            idx = self._filter(col, inputs, idx)
        any_ok = False
        for i in idx:
            for col in t["columns"]:
                if col["kind"] == "output_enum" and i < len(col["cells"]):
                    out[col["output"]] = col["cells"][i]
            if self._result(t["rows"][i]["result"], inputs, out, montages):
                any_ok = True
        if not any_ok and t.get("fallback"):
            if self._result(t["fallback"], inputs, out, montages):
                any_ok = True                                   # fallback outputs: none in this format
        return any_ok

    def _result(self, r, inputs, out, montages):
        if r["type"] == "asset":
            montages.append(r["asset"])
            return True
        if r["type"] == "nested":
            return self._eval(r["table"], inputs, out, montages)
        return False

    def evaluate(self, inputs):
        """inputs: dict name -> value (MovementMode may be a name). Returns dict(status, action, montages)."""
        inp = dict(inputs)
        if "MovementMode" in inp and inp["MovementMode"] is not None:
            inp["MovementMode"] = self.movement_mode_value(inp["MovementMode"])
        out = {"ActionType": self.action_none}
        montages = []
        if inp.get("DoingTraversalAction", False):             # IA_Jump: only when !DoingTraversalAction
            return {"status": STATUS_SKIP, "action": self.action_names[self.action_none], "montages": []}
        if not inp.get("HasFrontLedge", False) or inp.get("RoomCheckBlocked", False):
            return {"status": STATUS_CHECK_FAILED, "action": self.action_names[self.action_none], "montages": []}
        self._eval(self.root, inp, out, montages)
        action = out.get("ActionType", self.action_none)
        if action == self.action_none:
            status = STATUS_CHECK_FAILED
            montages = []
        elif not montages:
            status = STATUS_MONTAGE_FAILED
        else:
            status = STATUS_OK
        return {"status": status, "action": self.action_names.get(action, str(action)), "montages": montages}

    # ------------------------------------------------------------------ boundary grid
    def candidate_values(self, delta=0.5):
        vals = {}
        for t in self.tables:
            for col in t["columns"]:
                k = col["kind"]
                if k == "bool":
                    vals.setdefault(col["input"], {False, True})
                elif k == "float":
                    s = vals.setdefault(col["input"], set())
                    for mn, mx, nmn, nmx in col["cells"]:
                        for b, use in ((mn, not nmn), (mx, not nmx)):
                            if use:
                                s.update((b - delta, b, b + delta))
                elif k == "multienum":
                    vals.setdefault(col["input"], set(range(len(self.mm_names))))
        return {k: sorted(v) for k, v in vals.items()}

    def grid(self, delta=0.5):
        cv = self.candidate_values(delta)
        names = list(cv.keys())
        for combo in itertools.product(*(cv[n] for n in names)):
            yield dict(zip(names, combo))


# ---------------------------------------------------------------------- oracle comparison
def _camel(k):
    return "".join(p[:1].upper() + p[1:] for p in k.split("_"))


def _case_inputs(case):
    raw = None
    for k in ("inputs", "input", "in", "vector"):
        if isinstance(case.get(k), dict):
            raw = case[k]
            break
    if raw is None:
        raw = {k: v for k, v in case.items() if k[:1].isupper()}
    out = {}
    for k, v in raw.items():
        out[_camel(k) if "_" in k or k[:1].islower() else k] = v
    if "MovementModeValue" in out:                            # numeric value wins over the name
        out["MovementMode"] = out.pop("MovementModeValue")
    return out


def _case_expected(case):
    for k in ("expected", "expect", "output", "out", "result"):
        if isinstance(case.get(k), dict):
            return case[k]
    return case


def compare_case(tt, case):
    got = tt.evaluate(_case_inputs(case))
    exp = _case_expected(case)
    diffs = []
    ea = exp.get("action", exp.get("ActionType", exp.get("action_type")))
    if ea is not None and str(ea).lower() != got["action"].lower():
        diffs.append("action %s != %s" % (got["action"], ea))
    es = exp.get("status")
    if es is not None and es != got["status"]:
        diffs.append("status %s != %s" % (got["status"], es))
    eo = exp.get("outcome")
    if eo is not None and eo != OUTCOME[got["status"]]:
        diffs.append("outcome %s != %s" % (OUTCOME[got["status"]], eo))
    em = exp.get("montages", exp.get("candidates"))
    if em is not None:
        if sorted(set(em)) != sorted(set(got["montages"])):
            diffs.append("montages %s != %s" % (sorted(set(got["montages"])), sorted(set(em))))
    return got, diffs


def run_oracle(tt, oracle_path, verbose=False):
    with open(oracle_path, encoding="utf-8") as f:
        o = json.load(f)
    cases = (o.get("cases") or o.get("vectors") or []) if isinstance(o, dict) else o
    fails = 0
    for i, c in enumerate(cases):
        _, diffs = compare_case(tt, c)
        if diffs:
            fails += 1
            if verbose or fails <= 20:
                print("FAIL case %d %s: %s" % (i, c.get("name", c.get("id", "")), "; ".join(diffs)))
    n = len(cases)
    print("oracle %s: %d/%d agree (%.2f%%)" % (os.path.basename(oracle_path), n - fails, n,
                                               100.0 * (n - fails) / max(n, 1)))
    return fails, n


def _parse_set(items):
    out = {}
    for s in items:
        k, v = s.split("=", 1)
        if v.lower() in ("true", "false"):
            out[k] = v.lower() == "true"
        else:
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


def main(argv=None):
    root_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--table", default=os.path.join(root_dir, "Data", "traversal", "traversal_table.json"))
    sub = ap.add_subparsers(dest="cmd", required=True)
    pe = sub.add_parser("eval")
    pe.add_argument("--set", action="append", default=[])
    pg = sub.add_parser("grid")
    pg.add_argument("--out", required=True)
    pg.add_argument("--delta", type=float, default=0.5)
    pv = sub.add_parser("vectors", help="compact text vectors for the C++ test (HasFrontLedge=true grid)")
    pv.add_argument("--out", required=True)
    pv.add_argument("--delta", type=float, default=0.5)
    po = sub.add_parser("oracle")
    po.add_argument("--oracle", default=os.path.join(root_dir, "Research", "inventory", "traversal_oracle.json"))
    po.add_argument("-v", action="store_true")
    a = ap.parse_args(argv)
    tt = TraversalTable.load(a.table)
    if a.cmd == "eval":
        print(json.dumps(tt.evaluate(_parse_set(a.set)), indent=1))
        return 0
    if a.cmd == "grid":
        cases = []
        for v in tt.grid(a.delta):
            cases.append({"inputs": v, "expected": tt.evaluate(v)})
        with open(a.out, "w", encoding="utf-8", newline="\n") as f:
            json.dump({"format": "utpp_traversal_vectors", "table": os.path.basename(a.table),
                       "cases": cases}, f, indent=0)
        print("wrote %d vectors to %s" % (len(cases), a.out))
        return 0
    if a.cmd == "vectors":
        # line: front back floor depth backLedgeH height speed mm | status action m1,m2,...
        n = 0
        with open(a.out, "w", encoding="utf-8", newline="\n") as f:
            f.write("# utpp_traversal_vectors v1 table=%s\n" % os.path.basename(a.table))
            cv = tt.candidate_values(a.delta)
            cv["HasFrontLedge"] = [True]
            keys = ["HasFrontLedge", "HasBackLedge", "HasBackFloor", "ObstacleDepth", "BackLedgeHeight",
                    "ObstacleHeight", "Speed", "MovementMode"]
            for combo in itertools.product(*(cv.get(k, [0.0]) for k in keys)):
                v = dict(zip(keys, combo))
                r = tt.evaluate(v)
                f.write("%d %d %d %r %r %r %r %d | %s %s %s\n" % (
                    v["HasFrontLedge"], v["HasBackLedge"], v["HasBackFloor"], float(v["ObstacleDepth"]),
                    float(v["BackLedgeHeight"]), float(v["ObstacleHeight"]), float(v["Speed"]),
                    int(v["MovementMode"]), r["status"], r["action"], ",".join(r["montages"]) or "-"))
                n += 1
        print("wrote %d vectors to %s" % (n, a.out))
        return 0
    fails, n = run_oracle(tt, a.oracle, a.v)
    return 1 if fails or n == 0 else 0


if __name__ == "__main__":
    sys.exit(main())
