"""Print the top of an animgraph's output tree: node types along pose links, to a given depth.
usage: graphtop.py graph.json [maxdepth] [stop_type]"""
import json, sys

d = json.load(open(sys.argv[1], encoding="utf-8"))
maxdepth = int(sys.argv[2]) if len(sys.argv) > 2 else 8
rc = d["Data"]["RootChunk"]
print("RootChunk keys:", [k for k in rc.keys()])

handles = {}


def index(o):
    if isinstance(o, dict):
        if "HandleId" in o and isinstance(o.get("Data"), dict):
            handles[o["HandleId"]] = o["Data"]
        for v in o.values():
            index(v)
    elif isinstance(o, list):
        for v in o:
            index(v)


index(rc)


def resolve(o):
    if isinstance(o, dict):
        if "HandleRefId" in o:
            return o["HandleRefId"], handles.get(o["HandleRefId"])
        if "HandleId" in o:
            return o["HandleId"], o.get("Data")
    return None, None


SKIP_LINK = ("animFloatLink", "animVectorLink", "animIntLink", "animBoolLink", "animQuaternionLink", "animTransformLink")


def links(n, prefix=""):
    """yield (field, handle, node) for child anim nodes (pose links and nested node containers)."""
    for k, v in n.items():
        if k.startswith("vis") or k == "$type":
            continue
        items = v if isinstance(v, list) else [v]
        for i, it in enumerate(items):
            f = prefix + ("%s[%d]" % (k, i) if isinstance(v, list) else k)
            if not isinstance(it, dict):
                continue
            if it.get("$type") in SKIP_LINK:
                continue
            h, nd = resolve(it)
            if nd is not None:
                if str(nd.get("$type", "")).startswith("animAnimNode") or "State" in str(nd.get("$type", "")):
                    yield f, h, nd
                else:
                    yield from links(nd, f + "/")
            else:
                yield from links(it, f + "/")


seen = set()


def name(n):
    for k in ("debugName", "name", "graphName"):
        v = n.get(k)
        if isinstance(v, dict) and v.get("$value"):
            return v["$value"]
    return ""


def show(h, n, depth, field):
    tag = "%s%s <- %s  h=%s %s" % ("  " * depth, n.get("$type"), field, h, name(n))
    if h in seen:
        print(tag, "(seen)")
        return
    seen.add(h)
    print(tag)
    if depth >= maxdepth:
        return
    for f, ch, cn in links(n):
        show(ch, cn, depth + 1, f)


root = rc.get("rootNode")
h, n = resolve(root)
if n is None:
    print("no rootNode; nodesToInit[0]")
    h, n = resolve(rc["nodesToInit"][0])
show(h, n, 0, "root")
