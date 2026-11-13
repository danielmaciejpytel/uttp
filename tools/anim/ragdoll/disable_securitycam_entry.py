"""Guard the ANY -> tpp_securitycam_hacking global transition of player_locomotion.animgraph (WolvenKit JSON).
The transition fires on an edge of TPPRepresentation.IsActive while out of a vehicle and its state (looping idle_hacking:
crouched, right hand at the temple) only exits on IsActive == 0 or isInVehicle == 1. UTPP keeps IsActive = true on foot, so once
entered (e.g. right after the knockdown StatusEffect state) V stays in that pose.
The transition keeps its vanilla featureName (TPPRepresentation) and isEnabled = 1. One condition is added to its
CompositeSimultaneous (all conditions must hold): WrapperValue utpp_foot_tpp with checkIfWrapperIsSet = 0, i.e. the transition
is only allowed while the wrapper variable utpp_foot_tpp is NOT set. UTPP sets that variable to 1 while its on-foot third-person
body is wanted (scripts\\utpp_body.reds), so the vanilla security-camera takeover (wrapper 0) keeps entering the state and
the UTPP body never does. checkIfWrapperIsSet = 0 means "wrapper not set" (same use as the sandstormLocomotion / boxLocomotion /
guitarLocomotion / sickLocomotion guards on the Fall and ground transitions of this graph).
usage: disable_securitycam_entry.py in.json out.json   (file name kept for the pack script)"""
import copy, json, sys

WRAPPER = "utpp_foot_tpp"

src, dst = sys.argv[1], sys.argv[2]
d = json.load(open(src, encoding="utf-8"))
sm = d["Data"]["RootChunk"]["nodesToInit"][0]["Data"]["nodes"][0]["Data"]["node"]["node"]["Data"]
assert sm["$type"] == "animAnimNode_StateMachine", sm["$type"]
names = [s["Data"]["name"]["$value"] for s in sm["states"]]
target = names.index("tpp_securitycam_hacking")


def has_edge(c):
    if isinstance(c, dict):
        if c.get("$type") == "animAnimStateTransitionCondition_BoolEdgeFeature" \
                and c["featureName"]["$value"] == "TPPRepresentation" and c["featurePropertyName"]["$value"] == "IsActive":
            return True
        return any(has_edge(v) for v in c.values())
    if isinstance(c, list):
        return any(has_edge(v) for v in c)
    return False


def find_wrapper_template(o):
    """First vanilla WrapperValue transition condition with checkIfWrapperIsSet == 0 (the shape to clone)."""
    if isinstance(o, dict):
        if o.get("$type") == "animAnimStateTransitionCondition_WrapperValue" and o.get("checkIfWrapperIsSet") == 0:
            return o
        for v in o.values():
            r = find_wrapper_template(v)
            if r is not None:
                return r
    elif isinstance(o, list):
        for v in o:
            r = find_wrapper_template(v)
            if r is not None:
                return r
    return None


def max_handle(o):
    m = 0
    if isinstance(o, dict):
        for k, v in o.items():
            if k in ("HandleId", "HandleRefId"):
                m = max(m, int(v))
            else:
                m = max(m, max_handle(v))
    elif isinstance(o, list):
        for v in o:
            m = max(m, max_handle(v))
    return m


hits = [i for i, t in enumerate(sm["globalTransitions"])
        if t["Data"]["targetStateIndex"] == target and has_edge(t["Data"]["condition"])]
if len(hits) != 1:
    raise SystemExit(f"expected exactly one ANY -> tpp_securitycam_hacking edge transition, found {hits}")
t = sm["globalTransitions"][hits[0]]["Data"]
if t["isEnabled"] != 1:
    raise SystemExit(f"transition {hits[0]} isEnabled is {t['isEnabled']}, refusing (input must be the vanilla graph)")
comp = t["condition"]["Data"]
if comp["$type"] != "animAnimStateTransitionCondition_CompositeSimultaneous":
    raise SystemExit(f"transition {hits[0]} condition is {comp['$type']}, expected CompositeSimultaneous")
for c in comp["conditions"]:
    cd = c["Data"]
    if cd["$type"] == "animAnimStateTransitionCondition_BoolEdgeFeature" and cd["featureName"]["$value"] != "TPPRepresentation":
        raise SystemExit("edge feature already changed, refusing (input must be the vanilla graph)")
    if cd["$type"] == "animAnimStateTransitionCondition_WrapperValue":
        raise SystemExit("transition already has a WrapperValue condition, refusing")

template = find_wrapper_template(sm)
if template is None:
    raise SystemExit("no vanilla WrapperValue (checkIfWrapperIsSet 0) condition found to clone")
new_data = copy.deepcopy(template)
new_data["wrapperName"]["$value"] = WRAPPER
new_data["checkIfWrapperIsSet"] = 0
new_id = max_handle(d) + 1
before = len(comp["conditions"])
comp["conditions"].append({"HandleId": str(new_id), "Data": new_data})
assert len(comp["conditions"]) == before + 1
json.dump(d, open(dst, "w", encoding="utf-8"), indent=2)
print(f"globalTransitions[{hits[0]}] -> [{target}] tpp_securitycam_hacking: added WrapperValue {WRAPPER} "
      f"checkIfWrapperIsSet=0 (HandleId {new_id}); featureName and isEnabled unchanged")
