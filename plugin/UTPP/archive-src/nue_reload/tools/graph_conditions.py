"""Read-only: list transition conditions that mention given event/feature names in a WolvenKit graph JSON.

Usage: python -I graph_conditions.py <graph.json> NAME [NAME...]
For every RedClass dict that contains (anywhere below it, without crossing another state machine) a CName equal
to one of NAMEs, print the nearest enclosing transition/state machine path. Output is compact text.
"""
import json
import sys


def cname(v):
    return v.get("$value") if isinstance(v, dict) and v.get("$type") == "CName" else None


def summarize(o, depth=0, limit=3):
    """One-line summary of a condition subtree."""
    if isinstance(o, dict):
        if "HandleId" in o and "Data" in o:
            return summarize(o["Data"], depth, limit)
        if "HandleRefId" in o:
            return "ref#%s" % o["HandleRefId"]
        t = o.get("$type", "")
        if t == "CName":
            return o.get("$value")
        parts = []
        for k, v in o.items():
            if k.startswith("$"):
                continue
            c = cname(v)
            if c and c != "None":
                parts.append("%s=%s" % (k, c))
            elif isinstance(v, (dict, list)) and depth < limit:
                s = summarize(v, depth + 1, limit)
                if s:
                    parts.append("%s{%s}" % (k, s))
            elif isinstance(v, (int, float)) and k in ("value", "compareValue", "comparisonType", "operation", "invert"):
                parts.append("%s=%s" % (k, v))
        return "%s(%s)" % (t.replace("animAnimStateTransitionCondition_", "C_").replace("animAnimNode_", "N_"), ", ".join(parts))
    if isinstance(o, list):
        return "[" + "; ".join(filter(None, (summarize(x, depth, limit) for x in o))) + "]"
    return ""


def main():
    path, names = sys.argv[1], set(sys.argv[2:])
    root = json.load(open(path, encoding="utf-8"))["Data"]["RootChunk"]
    out = []

    def contains(o):
        if isinstance(o, dict):
            if cname(o) in names:
                return True
            return any(contains(v) for k, v in o.items() if not k.startswith("$"))
        if isinstance(o, list):
            return any(contains(v) for v in o)
        return False

    def visit(o, sm, st):
        if isinstance(o, dict):
            if "HandleId" in o and "Data" in o:
                visit(o["Data"], sm, st)
                return
            t = o.get("$type", "")
            if t == "animAnimNode_StateMachine":
                sm = "SM(id=%s)" % o.get("id", "?")
            if t == "animAnimNode_State":
                st = cname(o.get("name"))
            if t.startswith("animAnimStateTransitionDescription") or t == "animAnimStateTransitionDescription":
                if contains(o):
                    out.append("%s | %s" % (sm, summarize(o, 0, 6)))
                return
            for k, v in o.items():
                if not k.startswith("$"):
                    visit(v, sm, st)
        elif isinstance(o, list):
            for v in o:
                visit(v, sm, st)

    visit(root, "-", "-")
    for line in out:
        print(line[:1200])


if __name__ == "__main__":
    main()
