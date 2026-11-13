"""Offline inspector for WolvenKit JSON dumps of animAnimGraph (read-only).

Usage: python -I graph_inspect.py <graph.json> [--leaves] [--names]
  --leaves : list every animation leaf whose clip name contains 'reload' with the chain of
             parent node types / names up to the root (to see under which event/state it sits).
  --names  : print sorted CName values that look like events/features (for diffing versions).
Prints JSON to stdout. No writes.
"""
import json
import sys
from collections import defaultdict


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def cname(v):
    if isinstance(v, dict) and v.get("$type") == "CName":
        return v.get("$value")
    return None


def walk_index(root):
    """Index every handle (HandleId -> Data) and record parent edges between RedClass dicts."""
    handles = {}
    objects = []  # (obj_id, dict)
    parents = defaultdict(list)  # child obj id -> [(parent obj id, property path)]

    def visit(node, owner, path):
        if isinstance(node, dict):
            if "HandleId" in node and "Data" in node:
                handles[node["HandleId"]] = node["Data"]
                visit(node["Data"], owner, path + "->")
                return
            if "HandleRefId" in node:
                parents[("h", node["HandleRefId"])].append((owner, path))
                return
            if "$type" in node and node["$type"] not in ("CName", "CString", "ResourcePath"):
                me = id(node)
                objects.append((me, node))
                if owner is not None:
                    parents[("o", me)].append((owner, path))
                for k, v in node.items():
                    if k.startswith("$"):
                        continue
                    visit(v, me, k)
                return
            for k, v in node.items():
                visit(v, owner, path + "." + k)
        elif isinstance(node, list):
            for i, v in enumerate(node):
                visit(v, owner, path + "[%d]" % i)

    visit(root, None, "root")
    return handles, objects, parents


def main():
    path = sys.argv[1]
    flags = set(sys.argv[2:])
    data = load(path)["Data"]["RootChunk"]
    handles, objects, parents = walk_index(data)
    byid = {me: o for me, o in objects}
    # map handle data dict id -> handle ids
    hid_of = defaultdict(list)
    for hid, d in handles.items():
        hid_of[id(d)].append(hid)

    def parent_list(obj):
        out = list(parents.get(("o", id(obj)), []))
        for hid in hid_of.get(id(obj), []):
            out += parents.get(("h", hid), [])
        return out

    def label(o):
        bits = [o.get("$type")]
        for key in ("name", "eventName", "animation", "group", "featureName", "tag"):
            v = cname(o.get(key))
            if v:
                bits.append("%s=%s" % (key, v))
        return " ".join(bits)

    result = {}
    if "--leaves" in flags:
        leaves = []
        for me, o in objects:
            clip = cname(o.get("animation"))
            if clip and "reload" in clip.lower():
                chains = []
                frontier = [(o, [])]
                seen = set()
                while frontier and len(chains) < 6:
                    cur, chain = frontier.pop()
                    ps = parent_list(cur)
                    if not ps or len(chain) > 40:
                        chains.append(chain)
                        continue
                    for pid, prop in ps:
                        if (pid, len(chain)) in seen:
                            continue
                        seen.add((pid, len(chain)))
                        p = byid[pid]
                        frontier.append((p, chain + ["%s <%s>" % (label(p), prop)]))
                leaves.append({"leaf": label(o), "chains": chains[:3]})
        result["leaves"] = leaves
    if "--names" in flags:
        names = set()
        for me, o in objects:
            for k, v in o.items():
                c = cname(v)
                if c and k in ("name", "eventName", "group", "featureName", "event", "animFeatureName",
                               "variableName", "startEvent", "endEvent", "tag", "className"):
                    names.add("%s:%s" % (k, c))
        result["names"] = sorted(names)
    json.dump(result, sys.stdout, indent=1)


if __name__ == "__main__":
    main()
