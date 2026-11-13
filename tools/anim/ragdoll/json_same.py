"""Exit 0 when two WolvenKit JSON files hold the same Data. Handle ids are compared by first-appearance order (WolvenKit
renumbers them on a re-write), so a reference that points at a different object still counts as a difference.
usage: json_same.py a.json b.json"""
import json, sys


def canon(doc):
    ids = {}

    def rec(o):
        if isinstance(o, dict):
            r = {}
            for k, v in o.items():
                if k in ("HandleId", "HandleRefId"):
                    r[k] = ids.setdefault(v, len(ids))
                else:
                    r[k] = rec(v)
            return r
        if isinstance(o, list):
            return [rec(v) for v in o]
        return o

    return rec(doc["Data"])


a, b = (canon(json.load(open(p, encoding="utf-8"))) for p in sys.argv[1:3])
sys.exit(0 if a == b else 1)
