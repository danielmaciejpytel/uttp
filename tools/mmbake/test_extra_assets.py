"""Tests of extra_assets.py (--extra-assets of the v2 baker): spec validation, entry defaults, and - when two bake summaries
are given - that the clip insertion leaves every other clip / database untouched.

python test_extra_assets.py [--off base.summary.json --on extra.summary.json --clip NAME --db DB]
Every check prints passed / failed / not run; exit 0 only if nothing failed.
Without --on/--off only the pure functions run; the byte-identity of a bake WITHOUT --extra-assets is proven by baking the
same inputs with the extended baker and comparing the SHA-256 of the .utppmm.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import extra_assets as xa  # noqa: E402

RESULTS = []


def check(name, ok, info=""):
    RESULTS.append((name, ok))
    print(("passed " if ok else "FAILED ") + name + (" " + info if info else ""))


def raises(fn):
    try:
        fn()
    except SystemExit:
        return True
    return False


def test_pure():
    spec = {"format": xa.SPEC_FORMAT, "version": 1,
            "clips": [{"name": "NEWCLIP", "file": "clip.utppclip.json"}],
            "entries": [{"db": "A", "clip": "NEWCLIP"}, {"db": "B", "clip": "NEWCLIP", "disable_reselection": False,
                                                         "sampling_range": [0.5, 1.5]}]}
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "s.json")
        json.dump(spec, open(p, "w"))
        check("spec loads", xa.load_spec(p)["clips"][0]["name"] == "NEWCLIP")
        bad = dict(spec, version=2)
        json.dump(bad, open(p, "w"))
        check("wrong version refused", raises(lambda: xa.load_spec(p)))
        check("missing clip file refused", raises(lambda: xa.resolve_clips(spec, td, set())))
        open(os.path.join(td, "clip.utppclip.json"), "w").write("{}")
        files = xa.resolve_clips(spec, td, {"OTHER"})
        check("clip resolved relative to the root", files == {"NEWCLIP": os.path.join(td, "clip.utppclip.json")})
        check("existing clip name refused", raises(lambda: xa.resolve_clips(spec, td, {"NEWCLIP"})))
    db_index = {"A": 0, "B": 1}
    e = xa.make_entries(spec, files, db_index, {"A": [True, True, False], "B": [True]})
    check("entries are added, whole clip, looping from the asset", all(x["added"] and x["looping"] is None for x in e))
    check("disable_reselection default = majority of the Neutral entries (A: 2 of 3 -> True)", e[0]["disable_reselection"] is True)
    check("explicit disable_reselection / sampling_range kept", e[1]["disable_reselection"] is False and e[1]["range"] == (0.5, 1.5))
    check("default range = whole clip", e[0]["range"] == (0.0, 0.0))
    check("database without Neutral entries -> True", xa.make_entries(spec, files, {"A": 0, "B": 1}, {})[0]["disable_reselection"] is True)
    check("unknown database refused", raises(lambda: xa.make_entries(spec, files, {"A": 0}, {})))
    spec2 = dict(spec, entries=[{"db": "A", "clip": "MISSING"}])
    check("entry of a clip outside the spec refused", raises(lambda: xa.make_entries(spec2, files, db_index, {})))


def test_summaries(off_p, on_p, clip, db):
    off, on = json.load(open(off_p)), json.load(open(on_p))
    oc = {c["name"]: c for c in off["clips"]}
    nc = {c["name"]: c for c in on["clips"]}
    check("on-bake has exactly the one extra clip", set(nc) - set(oc) == {clip} and set(oc) <= set(nc))
    check("off-bake summary has no extra_assets key, on-bake has", "extra_assets" not in off and "extra_assets" in on)
    same = all({k: v for k, v in oc[n].items() if k not in ("first_frame",)} == {k: v for k, v in nc[n].items() if k not in ("first_frame",)}
               for n in oc)
    check("every existing clip: num_frames / loop / selectable / databases unchanged", same)
    new_last = nc[clip]["first_frame"] == max(c["first_frame"] for c in nc.values())
    if new_last:
        check("new clip sorts last: every existing first_frame unchanged", all(oc[n]["first_frame"] == nc[n]["first_frame"] for n in oc))
    else:
        print("not run first_frame identity (new clip does not sort last)")
    check("new clip is in the intended database only", nc[clip]["databases"] == [db], str(nc[clip]["databases"]))
    od = {d["name"]: d for d in off["databases"]}
    nd = {d["name"]: d for d in on["databases"]}
    check("same databases", list(od) == list(nd))
    other_same = all(od[n].get("num_entries") == nd[n].get("num_entries") and od[n].get("num_rows") == nd[n].get("num_rows") for n in od if n != db)
    check("rows / entries of every other database unchanged", other_same)
    check("target database gained one entry", nd[db].get("num_entries") == od[db].get("num_entries", 0) + 1,
          f"{od[db].get('num_entries')} -> {nd[db].get('num_entries')}")
    check("target database gained rows", nd[db].get("num_rows", 0) > od[db].get("num_rows", 0), f"{od[db].get('num_rows')} -> {nd[db].get('num_rows')}")
    check("total frames grew by the clip length", on["num_frames"] - off["num_frames"] == nc[clip]["num_frames"])


def sha256(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def test_identity_off(off_db, ref_db):
    """A bake of the same inputs WITHOUT --extra-assets (by the extended baker) is byte-identical to the reference file."""
    a, b = sha256(off_db), sha256(ref_db)
    check("bake without --extra-assets is byte-identical to the reference database", a == b, a[:16] + (" == " if a == b else " != ") + b[:16])


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--off")
    ap.add_argument("--on")
    ap.add_argument("--off-db", help=".utppmm baked by the extended baker without --extra-assets")
    ap.add_argument("--ref-db", help="reference .utppmm (e.g. the installed database) that --off-db must equal byte for byte")
    ap.add_argument("--clip", default="RC28_N8_MB")
    ap.add_argument("--db", default="PSD_Dense_Crouch_Walk_Loops")
    a = ap.parse_args(argv)
    test_pure()
    if a.off and a.on:
        test_summaries(a.off, a.on, a.clip, a.db)
    else:
        print("not run summary comparison (no --off/--on)")
    if a.off_db and a.ref_db:
        test_identity_off(a.off_db, a.ref_db)
    else:
        print("not run byte-identity (no --off-db/--ref-db)")
    bad = [n for n, ok in RESULTS if not ok]
    print(f"{len(RESULTS) - len(bad)} passed, {len(bad)} failed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
