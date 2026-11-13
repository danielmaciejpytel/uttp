#!/usr/bin/env python3
"""Writes a clip keep-list for mmbake --clip-names-from: the clips of a base .summary.json minus a curation file's
"exclude" entries (Research/inventory/female-curation-v2.json format). Fails if an excluded clip is not in the base."""
import argparse
import json
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--base", required=True, help="base database .summary.json")
    ap.add_argument("--curation", required=True)
    ap.add_argument("--out", required=True, help="keep-list (summary-format JSON: {\"clips\": [{\"name\": ...}]})")
    a = ap.parse_args()
    base = [c["name"] for c in json.load(open(a.base, encoding="utf-8"))["clips"]]
    excl = [e["clip"] for e in json.load(open(a.curation, encoding="utf-8"))["exclude"]]
    missing = sorted(set(excl) - set(base))
    if missing:
        sys.exit("excluded clips not in the base: " + ", ".join(missing))
    keep = [n for n in base if n not in set(excl)]
    json.dump({"source": a.base, "curation": a.curation, "excluded": len(excl), "clips": [{"name": n} for n in keep]},
              open(a.out, "w", encoding="utf-8"), indent=1)
    print(f"base {len(base)} clips, excluded {len(excl)}, kept {len(keep)} -> {a.out}")


if __name__ == "__main__":
    main()
