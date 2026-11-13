"""Build the ArchiveXL entity patches that add an engine ragdoll component to the player entities (UTPP ragdoll PoC 'R1', LOCAL TEST).

NPCs get their ragdoll from base\\characters\\base_entities\\npc_base\\npc_base.ent: an entRagdollComponent named "RagdollComponent"
with exactly the fields {id, isEnabled = 1, isReplicable = 0, name} (no control binding, no other data). The player entities
player_{wa,ma}_fpp(.ent|_ep1.ent) have no such component (checked in the 2.5 data: the only 'ragdoll' text in them is the PlayerPuppet
field allowVehicleCollisionRagdoll). This script authors utpp\\ragdoll\\utpp_ragdoll_wa.ent / utpp_ragdoll_ma.ent holding ONE such
component each (CRUID ASCII "UTPPRGW1" / "UTPPRGM1"), delivered by utpp_ragdoll_poc.archive.xl as `resource: patch:` with
props [components] (same delivery as the working bike animation-set patch, tools/anim/vehicle-entry/make_bike_ext_patch.py).

The component is NOT hand-copied: it is read from the 2.5 game's own npc_base.ent (basegame_4_appearance.archive, unbundled READ-ONLY
into the work dir and converted with WolvenKit), the field set is asserted, then only id is replaced. The package header values
(Flags, Type, WolvenKit/Game version) come from the same current-CLI conversion, so they match what the 2.5 game files carry.
Round trip: JSON -> `convert deserialize` -> .ent -> `convert serialize` -> JSON; the whole RootChunk must come back unchanged
except the default values WolvenKit writes for the empty placeholder entEntity (handle numbers are renumbered).
Usage: python -I make_ragdoll_ent_patch.py --stage <stage root> --work <empty work dir> [--game <Cyberpunk 2077 dir>]
       [--npc-base-json <already converted npc_base.ent.json>] [--wkit-exe <exe>]
"""
import argparse
import copy
import datetime
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # the tools folder
from utpp_paths import project_root, game_dir  # noqa: E402

ROOT = Path(project_root())
WKIT_DEFAULT = ROOT / r"Tools\WolvenKit.CLI\WolvenKit.CLI.exe"
GAME_DEFAULT = Path(game_dir())
NPC_ARCHIVE = Path("archive") / "pc" / "content" / "basegame_4_appearance.archive"
NPC_DEPOT = Path("base") / "characters" / "base_entities" / "npc_base" / "npc_base.ent"
DEPOT = Path("utpp") / "ragdoll"
FILES = {"w": "utpp_ragdoll_wa.ent", "m": "utpp_ragdoll_ma.ent"}
# CRUIDs: ASCII "UTPPRGW1" / "UTPPRGM1" (unique, never used by the game; ArchiveXL matches components by name + id)
CRUID = {"w": str(int.from_bytes(b"UTPPRGW1", "big")), "m": str(int.from_bytes(b"UTPPRGM1", "big"))}
COMPONENT_NAME = "RagdollComponent"
EXPECTED_FIELDS = {"$type", "id", "isEnabled", "isReplicable", "name"}
ROUNDTRIP_IGNORED_PACKAGE_KEYS = ()  # Sections comes back unchanged (6), so nothing is ignored
ENTITY_DEFAULT_KEYS = {"customCameraTarget", "renderSceneLayerMask"}  # written by WolvenKit for the empty placeholder entEntity


def run(args):
    print(">", " ".join(str(a) for a in args))
    r = subprocess.run([str(a) for a in args], capture_output=True, text=True)
    out = (r.stdout or "") + (r.stderr or "")
    if r.returncode != 0 or "[ 0: Error" in out or "Exception" in out:
        print(out.strip()[-3000:])
        raise SystemExit(f"command failed ({r.returncode})")
    return out


def load(p):
    return json.loads(Path(p).read_text(encoding="utf-8"))


def diffs(a, b, path="", limit=20):
    out = []

    def rec(x, y, p):
        if len(out) >= limit:
            return
        if isinstance(x, dict) and isinstance(y, dict):
            for k in sorted(set(x) | set(y)):
                if k not in x or k not in y:
                    out.append(f"{p}/{k}: only in {'b' if k not in x else 'a'}")
                else:
                    rec(x[k], y[k], f"{p}/{k}")
        elif isinstance(x, list) and isinstance(y, list):
            if len(x) != len(y):
                out.append(f"{p}: length {len(x)} != {len(y)}")
                return
            for i, (xi, yi) in enumerate(zip(x, y)):
                rec(xi, yi, f"{p}[{i}]")
        elif x != y:
            out.append(f"{p}: {x!r} != {y!r}")

    rec(a, b, path)
    return out


def find_ragdoll(items, where):
    found = [c for c in items if isinstance(c, dict) and c.get("$type") == "entRagdollComponent"]
    if len(found) != 1:
        raise SystemExit(f"npc_base {where}: expected exactly one entRagdollComponent, found {len(found)}")
    return found[0]


def npc_base_reference(a, work):
    """Returns (npc_base json doc, component) from the game's own npc_base.ent."""
    if a.npc_base_json:
        doc = load(a.npc_base_json)
    else:
        archive = Path(a.game) / NPC_ARCHIVE
        if not archive.is_file():
            raise SystemExit(f"game archive missing (use --npc-base-json): {archive}")
        ex = work / "npc_base-unbundle"
        ex.mkdir(parents=True)
        run([a.wkit_exe, "unbundle", archive, "-w", "*npc_base.ent", "-o", ex])  # read-only on the game folder
        ent = ex / NPC_DEPOT
        if not ent.is_file():
            raise SystemExit(f"npc_base.ent not found after unbundle: {ent}")
        js = work / "npc_base-json"
        js.mkdir()
        run([a.wkit_exe, "convert", "serialize", ent, "-o", js])
        doc = load(js / "npc_base.ent.json")
    root = doc["Data"]["RootChunk"]
    comp_c = find_ragdoll(root["compiledData"]["Data"]["Chunks"], "compiledData")
    comp_l = find_ragdoll(root["components"], "components")
    for tag, c in (("compiledData", comp_c), ("components", comp_l)):
        if set(c) != EXPECTED_FIELDS:
            raise SystemExit(f"npc_base {tag}: unexpected entRagdollComponent field set {sorted(c)}")
    if diffs(comp_c, comp_l):
        raise SystemExit("npc_base: compiledData and components entries of the ragdoll component differ")
    if comp_c["name"]["$value"] != COMPONENT_NAME or comp_c["isEnabled"] != 1 or comp_c["isReplicable"] != 0:
        raise SystemExit(f"npc_base: unexpected ragdoll component values: {comp_c}")
    return doc, comp_c


def make_component(ref, g):
    c = copy.deepcopy(ref)
    c["id"] = CRUID[g]
    return c


def entity_template(g, ref_doc, ref_comp):
    comp = make_component(ref_comp, g)
    rref = ref_doc["Data"]["RootChunk"]
    package = {
        "Version": 4,
        "Sections": 6,  # same value as the camera / bike patches that work in game (round trip returns 6; vanilla packages carry 7)
        "CruidIndex": 0,
        "CruidDict": {"0": "0", "1": CRUID[g]},
        "Chunks": [{"$type": "entEntity"}, copy.deepcopy(comp)],
    }
    root = {
        "$type": "entEntityTemplate",
        "appearances": [],
        "backendDataOverrides": [],
        "bindingOverrides": [],
        "compiledData": {"BufferId": "0", "Flags": rref["compiledData"]["Flags"], "Type": rref["compiledData"]["Type"], "Data": package},
        "compiledEntityLODFlags": 0,
        "componentResolveSettings": [],
        "components": [copy.deepcopy(comp)],
        "cookingPlatform": rref["cookingPlatform"],
        "defaultAppearance": {"$type": "CName", "$storage": "string", "$value": "None"},
        # no handle is defined inside the package (the component has no control binding), so the root entity is handle 0
        "entity": {"HandleId": "0", "Data": {"$type": "entEntity"}},
        "includeInstanceBuffer": None,
        "includes": [],
        "inplaceResources": [],
        "localData": None,
        "resolvedDependencies": [],
        "visualTagsSchema": None,
    }
    hdr = ref_doc["Header"]
    return {"Header": {"WolvenKitVersion": hdr["WolvenKitVersion"], "WKitJsonVersion": hdr["WKitJsonVersion"],
                       "GameVersion": hdr["GameVersion"],
                       "ExportedDateTime": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
                       "DataType": "CR2W", "ArchiveFileName": ""},
            "Data": {"Version": ref_doc["Data"]["Version"], "BuildVersion": ref_doc["Data"]["BuildVersion"],
                     "RootChunk": root, "EmbeddedFiles": []}}


def check_roundtrip(src_json, rt_json, g, ref_comp):
    src, rt = load(src_json)["Data"]["RootChunk"], load(rt_json)["Data"]["RootChunk"]
    for d in (src, rt):
        pk = d["compiledData"]["Data"]
        for k in ROUNDTRIP_IGNORED_PACKAGE_KEYS:
            pk.pop(k, None)
        d["entity"]["HandleId"] = "*"  # handle numbers are renumbered by the serializer, only the data matters
    # WolvenKit writes the default values of the (otherwise empty) placeholder entEntity; patches only merge `components`
    for holder, key in ((rt["entity"], "Data"), (rt["compiledData"]["Data"]["Chunks"], 0)):
        extra = set(holder[key]) - {"$type"}
        if not extra <= ENTITY_DEFAULT_KEYS:
            raise SystemExit(f"round trip: unexpected placeholder entity fields {sorted(extra)}")
        holder[key] = {"$type": holder[key]["$type"]}
    for holder, key in ((src["entity"], "Data"), (src["compiledData"]["Data"]["Chunks"], 0)):
        holder[key] = {"$type": holder[key]["$type"]}
    d = diffs(src, rt)
    if d:
        raise SystemExit(f"round trip mismatch for {rt_json}:\n  " + "\n  ".join(d))
    comps = rt["components"]
    chunks = rt["compiledData"]["Data"]["Chunks"]
    if len(comps) != 1 or len(chunks) != 2 or comps[0]["$type"] != "entRagdollComponent" or chunks[1]["$type"] != "entRagdollComponent":
        raise SystemExit("round trip: expected exactly one entRagdollComponent in components and in compiledData")
    expect = make_component(ref_comp, g)
    if diffs(comps[0], expect) or diffs(chunks[1], expect):
        raise SystemExit("round trip: component differs from the npc_base component (apart from id)")
    print(f"round trip ok: {Path(rt_json).name} -> {json.dumps(comps[0], separators=(',', ':'))}")
    return comps[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True, help="staging root; .ent files go to <stage>/utpp/ragdoll/")
    ap.add_argument("--work", required=True, help="work dir for JSON / intermediates (must not exist or be empty)")
    ap.add_argument("--game", default=str(GAME_DEFAULT), help="Cyberpunk 2077 directory (read only, for npc_base.ent)")
    ap.add_argument("--npc-base-json", help="use this already converted npc_base.ent.json instead of unbundling the game archive")
    ap.add_argument("--wkit-exe", default=str(WKIT_DEFAULT))
    a = ap.parse_args()
    wkit = Path(a.wkit_exe)
    if not wkit.is_file():
        raise SystemExit(f"WolvenKit CLI missing: {wkit}")
    work, stage = Path(a.work), Path(a.stage)
    if work.exists() and any(work.iterdir()):
        raise SystemExit(f"work dir is not empty, refusing: {work}")
    work.mkdir(parents=True, exist_ok=True)
    depot = stage / DEPOT
    for fname in FILES.values():
        if (depot / fname).exists():
            raise SystemExit(f"stage file exists, refusing to overwrite: {depot / fname}")
    ref_doc, ref_comp = npc_base_reference(a, work)
    print("npc_base reference component:", json.dumps(ref_comp, separators=(",", ":")))
    depot.mkdir(parents=True, exist_ok=True)
    jdir, tmp, rtdir = work / "json", work / "deser", work / "roundtrip"
    for d in (jdir, tmp, rtdir):
        d.mkdir(parents=True)
    result = {}
    for g, fname in FILES.items():
        j = jdir / (fname + ".json")
        j.write_text(json.dumps(entity_template(g, ref_doc, ref_comp), indent=2) + "\n", encoding="utf-8")
        run([wkit, "convert", "deserialize", j, "-o", tmp])
        built = tmp / fname
        if not built.is_file():
            raise SystemExit(f"deserialize produced no {fname}")
        run([wkit, "convert", "serialize", built, "-o", rtdir])
        result[fname] = check_roundtrip(j, rtdir / (fname + ".json"), g, ref_comp)
        (depot / fname).write_bytes(built.read_bytes())
    (work / "ent-report.json").write_text(json.dumps(
        {"npcBaseComponent": ref_comp, "components": result}, indent=2) + "\n", encoding="utf-8")
    print("ENT_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
