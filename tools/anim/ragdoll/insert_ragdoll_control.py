"""Insert a pass-through animAnimNode_RagdollControl between animAnimNode_Output and its input in an animgraph JSON.
usage: patch_rdctl.py in.json out.json"""
import copy, json, sys

src, dst = sys.argv[1], sys.argv[2]
d = json.load(open(src, encoding="utf-8"))
rc = d["Data"]["RootChunk"]
root = rc["nodesToInit"][0]
assert root["Data"]["$type"] == "animAnimNode_Root", root["Data"]["$type"]
out = root["Data"]["nodes"][0]
assert out["Data"]["$type"] == "animAnimNode_Output"
child = out["Data"]["node"]["node"]
assert "HandleId" in child, "expected the inline definition of the output's input"
mx = 0


def maxh(o):
    global mx
    if isinstance(o, dict):
        for k in ("HandleId", "HandleRefId"):
            if k in o:
                mx = max(mx, int(o[k]))
        for v in o.values():
            maxh(v)
    elif isinstance(o, list):
        for v in o:
            maxh(v)


maxh(d)
newh = str(mx + 1)
vis = {k: copy.deepcopy(v) for k, v in out["Data"].items() if k.startswith("vis") or k in ("debugName", "poseInfoLogger")}
node = {"$type": "animAnimNode_RagdollControl", "blendInDuration": 0.200000003, "blendOutDuration": 0.5}
node.update({k: vis[k] for k in ("debugName",)})
node["id"] = 4294967295
node["inputPoseNode"] = {"$type": "animPoseLink", "node": child}
node["poseInfoLogger"] = None
for k in sorted(vis):
    if k.startswith("vis"):
        node[k] = vis[k]
out["Data"]["node"]["node"] = {"HandleId": newh, "Data": node}
rc["nodesToInit"].append({"HandleRefId": newh})
json.dump(d, open(dst, "w", encoding="utf-8"), indent=2)
print("inserted RagdollControl handle", newh, "input handle", child["HandleId"], "nodesToInit", len(rc["nodesToInit"]))
