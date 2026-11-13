"""Read-only structural delta check for the rebuilt utpp_nue_reload graphs.

Usage: python -I graph_delta_check.py <vanilla231.json> <utpp231.json> <vanilla25.json> <utpp25.json>
Compares the UTPP delta (UTPP minus vanilla) of 2.31 and 2.5 by:
  * multiset of node $type counts,
  * set of CName values (events / features / clips),
  * dangling handle references (HandleRefId without HandleId),
  * animation clip names added.
Prints a JSON verdict; exit code 1 when the 2.5 delta differs from the 2.31 delta or references dangle.
"""
import json
import sys
from collections import Counter


def scan(path):
    root = json.load(open(path, encoding="utf-8"))["Data"]["RootChunk"]
    types, names, clips = Counter(), Counter(), Counter()
    ids, refs = set(), set()

    def visit(o, key=None):
        if isinstance(o, dict):
            if "HandleId" in o:
                ids.add(o["HandleId"])
            if "HandleRefId" in o:
                refs.add(o["HandleRefId"])
            t = o.get("$type")
            if t == "CName":
                v = o.get("$value")
                if v and v != "None":
                    names[v] += 1
                    if key == "animation":
                        clips[v] += 1
                return
            if t and t not in ("CString", "ResourcePath"):
                types[t] += 1
            for k, v in o.items():
                if not k.startswith("$"):
                    visit(v, k)
        elif isinstance(o, list):
            for v in o:
                visit(v, key)

    visit(root)
    return types, names, clips, sorted(refs - ids)


def delta(a, b):
    out = {}
    for k in set(a) | set(b):
        d = b[k] - a[k]
        if d:
            out[k] = d
    return out


def main():
    v231, u231, v25, u25 = (scan(p) for p in sys.argv[1:5])
    d231 = [delta(v231[i], u231[i]) for i in range(3)]
    d25 = [delta(v25[i], u25[i]) for i in range(3)]
    old_names = ["Reload", "EndReload", "ReloadData", "ReloadSpeed", "ReloadLoop", "InterruptReload", "WeaponStats",
                 "WeaponData", "equipUnequipItem", "Shoot", "CycleRound"]
    stale_in_25 = [n for n in old_names if u25[1][n] and not v25[1][n]]
    right_names = sorted(n for n in v25[1] if n.startswith(("Right", "Left", "EndRight", "EndLeft", "InterruptRight",
                                                               "InterruptLeft", "right", "left")))
    missing_right = [n for n in right_names if not u25[1][n]]
    verdict = {
        "typeDelta231": d231[0], "typeDelta25": d25[0], "typeDeltaEqual": d231[0] == d25[0],
        "nameDelta231": d231[1], "nameDelta25": d25[1], "nameDeltaEqual": d231[1] == d25[1],
        "clipDelta25": d25[2], "clipDeltaEqual": d231[2] == d25[2],
        "dangling25": u25[3], "danglingVanilla25": v25[3],
        "staleOldEventNamesIn25": stale_in_25,
        "rightLeftNamesInVanilla25": right_names, "rightLeftNamesMissingInUtpp25": missing_right,
    }
    ok = verdict["typeDeltaEqual"] and verdict["nameDeltaEqual"] and verdict["clipDeltaEqual"] and not u25[3] \
        and not stale_in_25 and not missing_right
    verdict["pass"] = ok
    json.dump(verdict, sys.stdout, indent=1, sort_keys=True)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
