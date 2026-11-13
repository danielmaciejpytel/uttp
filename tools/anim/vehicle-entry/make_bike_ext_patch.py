"""Build the ArchiveXL entity patch that adds the UTPP third-person bike sets as an animation setup EXTENSION component.

Why (2026-10-28, diag c8 'mount seat pop'): the game registers its motorcycle sets on an entAnimationSetupExtensionComponent named
'MotorcycleAnimsets' (bound to the 'root' animated component) of player_{wa,ma}_fpp(_ep1).ent: FPP p?a_sportbikeN at priority 119
with [sportbikeN_vehicle_seat_front_left], TPP tpp_p?a_sportbikeN at 100 with [player_tpp_head, seat]. The UTPP sets registered by
utpp_vehicle_bike.archive.xl / utpp_vehicle_tpp.archive.xl go through ArchiveXL's 'animations' feature, which only pushes into the
root entAnimatedComponent's own animations.gameplay (ArchiveXL src/App/Extensions/Animation/Extension.cpp, InitializeAnimations
hook). In every recorded TPP ride the first-person enter clip still played (its last frame -> TPP drive_slow_bike f0 = the logged
9.1 cm / 28 deg seat step), so the root-merged entries are not taken for the workspot entry (H2), or the precedence is the other
way round (H1, tested by the root route at priority 130).
This patch adds a SEPARATE extension component 'UTPPMotorcycleAnimsets' (own name and CRUID, so ArchiveXL's MergeComponents appends
it and never replaces the vanilla one; same control binding as the vanilla component: bindName root, NoBinding excluded) holding
only UTPP entries:
  * the six UTPP enter sets (utpp\\vehicle\\utpp_tpp_p?a_sportbike[2|3]_enter_{l,r}.anims) at priority 94, gated by
    [utpp_vehicle_tpp, utpp_bike_route_ext, sportbikeN_vehicle_seat_front_left] (route bit 1);
  * the three game TPP riding sets (base\\animations\\vehicle\\sportbikeN\\character\\tpp_p?a_sportbikeN.anims, by path) at 95,
    gated by [utpp_vehicle_tpp, utpp_bike_route_ride, sportbikeN_vehicle_seat_front_left] (route bit 4), so the clip after the
    enter (drive_slow_bike) is the TPP one even before the game's player_tpp_head is on. These sets also hold the TPP exit_l /
    exit_r: with bit 4 the exit clips come from the TPP set too (switch it off alone to keep vanilla riding / exits).
Lower number = winning entry inside the extension pool is what the vanilla data shows (TPP 100 [player_tpp_head, seat] beats FPP
119 [seat] when the TPP head variable is on; player_default_vehicle 254 with no vars is the fallback). With the route variable at 0
(first person; or Game.UTPPSetVehicleBikeRoute without bit 1 / 4) none of these entries can match: vanilla FPP is unchanged.
Route default in utpp_vehicle_anim.reds is 0 since diag c8b (enter via the workspot override, make_bike_workspot.py).
Same delivery as the camera patch (plugin/UTPP/archive/make_camera_patch.py, working in game): `resource: patch:` with
props [components]; ArchiveXL 1.27.3+ ResourcePatch PatchPackageResults merges the components of the patch's compiledData package.

Outputs:
  tools/anim/vehicle-entry/patch-json/utpp/vehicle/utpp_bike_animsets_{wa,ma}.ent.json   (authored here, tracked)
  with --wkit <depot dir>: WolvenKit CLI `convert deserialize` into <depot dir>/utpp/vehicle/*.ent, then `convert serialize`
  round trip into <depot dir>/../patch-roundtrip and a check that the component came back unchanged.
  --print-xl: the `resource: patch:` section appended to utpp_vehicle_bike.archive.xl (one .xl next to the one archive that also
  holds the patch .ent files; the installer allow-list already covers utpp_vehicle_bike.archive(.xl)).
Usage: python -I make_bike_ext_patch.py                 (JSON only)
       python -I make_bike_ext_patch.py --wkit <stage>  (stage = the archive staging root, e.g. ...\\utpp_vehicle_bike)
       python -I make_bike_ext_patch.py --print-xl      (prints the .xl section)
"""
import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

HERE = Path(__file__).resolve().parent
JSON_DIR = HERE / "patch-json"
DEPOT = Path("utpp") / "vehicle"
WKIT_DEFAULT = Path(project_root(), r"Tools\WolvenKit.CLI\WolvenKit.CLI.exe")

COMPONENT_NAME = "UTPPMotorcycleAnimsets"
ROUTE_VAR = "utpp_bike_route_ext"
RIDE_VAR = "utpp_bike_route_ride"
WRAPPER = "utpp_vehicle_tpp"
PRIO_ENTER = 94
PRIO_RIDE = 95
# CRUIDs: ASCII "UTPPBKW1" / "UTPPBKM1" (unique, never used by the game; ArchiveXL matches components by name + id)
CRUID = {"w": str(0x55545050424B5731), "m": str(0x55545050424B4D31)}
FILES = {"w": "utpp_bike_animsets_wa.ent", "m": "utpp_bike_animsets_ma.ent"}
TARGETS = {
    "w": [r"base\characters\entities\player\player_wa_fpp.ent", r"ep1\characters\entities\player\player_wa_fpp_ep1.ent"],
    "m": [r"base\characters\entities\player\player_ma_fpp.ent", r"ep1\characters\entities\player\player_ma_fpp_ep1.ent"],
}


def cname(v):
    return {"$type": "CName", "$storage": "string", "$value": v}


def entry(path, prio, seat_no, route_var):
    return {
        "$type": "animAnimSetupEntry",
        "animSet": {"DepotPath": {"$type": "ResourcePath", "$storage": "string", "$value": path}, "Flags": "Default"},
        "priority": prio,
        "variableNames": [cname(WRAPPER), cname(route_var), cname(f"sportbike{seat_no}_vehicle_seat_front_left")],
    }


def entries(g):
    out = []
    for b in ("", "2", "3"):
        n = b or "1"
        for side in ("l", "r"):
            out.append(entry(rf"utpp\vehicle\utpp_tpp_p{g}a_sportbike{b}_enter_{side}.anims", PRIO_ENTER, n, ROUTE_VAR))
    for b in ("", "2", "3"):
        n = b or "1"
        d = "sportbike" + b
        out.append(entry(rf"base\animations\vehicle\{d}\character\tpp_p{g}a_sportbike{b}.anims", PRIO_RIDE, n, RIDE_VAR))
    return out


def binding(handle_id):
    # same binding as the vanilla MotorcycleAnimsets / CarAnimsets components (2.31 extraction of player_wa_fpp_ep1.ent)
    return {"HandleId": str(handle_id), "Data": {
        "$type": "entAnimationControlBinding",
        "bindName": cname("root"),
        "enabled": 1,
        "enableMask": {"$type": "entTagMask",
                       "excludedTags": {"$type": "redTagList", "tags": [cname("NoBinding")]},
                       "hardTags": {"$type": "redTagList", "tags": []},
                       "softTags": {"$type": "redTagList", "tags": []}}}}


def component(g, control_binding):
    return {
        "$type": "entAnimationSetupExtensionComponent",
        "animations": {"$type": "animAnimSetup", "cinematics": [], "gameplay": entries(g)},
        "controlBinding": control_binding,
        "id": CRUID[g],
        "isOverrideContainer": 0,
        "isReplicable": 0,
        "name": cname(COMPONENT_NAME),
    }


def entity_template(g):
    # WolvenKit JSON handles are numbered in document order: compiledData (first) defines handle 0 (the control binding),
    # the components list refers to it, the root entity is handle 1 (vanilla .ent: same scheme).
    compiled = component(g, binding(0))
    listed = component(g, {"HandleRefId": "0"})
    package = {
        "Version": 4,
        "Sections": 6,  # recomputed by WolvenKit on write
        "CruidIndex": 0,
        "CruidDict": {"0": "0", "1": CRUID[g]},
        "Chunks": [{"$type": "entEntity"}, compiled],
    }
    root = {
        "$type": "entEntityTemplate",
        "appearances": [],
        "backendDataOverrides": [],
        "bindingOverrides": [],
        "compiledData": {"BufferId": "0", "Flags": 4063232,
                         "Type": "WolvenKit.RED4.Archive.Buffer.RedPackage, WolvenKit.RED4, Version=9.0.1.0, "
                                 "Culture=neutral, PublicKeyToken=null",
                         "Data": package},
        "compiledEntityLODFlags": 0,
        "componentResolveSettings": [],
        "components": [listed],
        "cookingPlatform": "PLATFORM_PC",
        "defaultAppearance": cname("None"),
        "entity": {"HandleId": "1", "Data": {"$type": "entEntity"}},
        "includeInstanceBuffer": None,
        "includes": [],
        "inplaceResources": [],
        "localData": None,
        "resolvedDependencies": [],
        "visualTagsSchema": None,
    }
    return {"Header": {"WolvenKitVersion": "9.0.1", "WKitJsonVersion": "0.0.9", "GameVersion": 2310,
                       "ExportedDateTime": "2026-10-28T00:00:00Z", "DataType": "CR2W", "ArchiveFileName": ""},
            "Data": {"Version": 195, "BuildVersion": 0, "RootChunk": root, "EmbeddedFiles": []}}


def xl_section():
    lines = [
        "",
        "# Route 'ext' / 'ride' (seat pop fix 2026-10-28): ArchiveXL resource patch adding the extension component",
        "# " + COMPONENT_NAME + " (own name and id: appended next to the game's MotorcycleAnimsets, never replacing it) to the",
        "# player FPP entities: UTPP enter sets at priority " + str(PRIO_ENTER) + " [" + WRAPPER + ", " + ROUTE_VAR + ", seat] and the",
        "# game's TPP riding sets (incl. their exit clips) at " + str(PRIO_RIDE) + " [" + WRAPPER + ", " + RIDE_VAR + ", seat].",
        "# Lower number wins inside the extension pool (vanilla: TPP 100 [player_tpp_head, seat] over FPP 119 [seat]). The patch",
        "# resources are packed into utpp_vehicle_bike.archive (make_bike_ext_patch.py); only `components` are merged (props).",
        "resource:",
        "  patch:",
    ]
    for g in ("w", "m"):
        lines += ["    " + str(DEPOT / FILES[g]).replace("/", "\\") + ":", "      props:", "        - components", "      targets:"]
        lines += ["        - " + t for t in TARGETS[g]]
    return "\n".join(lines) + "\n"


def strip_binding(c):
    # WolvenKit adds animations.hash = '0' on serialize; it is not part of the authored data.
    out = {k: v for k, v in c.items() if k != "controlBinding"}
    if isinstance(out.get("animations"), dict):
        out["animations"] = {k: v for k, v in out["animations"].items() if not (k == "hash" and v in ("0", 0))}
    return out


def check_roundtrip(src_json, rt_json):
    src = json.loads(Path(src_json).read_text(encoding="utf-8"))["Data"]["RootChunk"]
    rt = json.loads(Path(rt_json).read_text(encoding="utf-8"))["Data"]["RootChunk"]
    a, b = src["compiledData"]["Data"], rt["compiledData"]["Data"]
    ca, cb = a["Chunks"][1], b["Chunks"][1]
    bind = (cb.get("controlBinding") or {}).get("Data") or {}
    if strip_binding(ca) != strip_binding(cb) or a["CruidDict"] != b["CruidDict"]:
        raise SystemExit(f"round trip mismatch (compiled component): {rt_json}")
    if bind.get("bindName", {}).get("$value") != "root":
        raise SystemExit(f"round trip: control binding lost: {rt_json}")
    if strip_binding(src["components"][0]) != strip_binding(rt["components"][0]):
        raise SystemExit(f"round trip mismatch (components list): {rt_json}")
    print(f"round trip ok: {Path(rt_json).name} ({cb['$type']} {cb['name']['$value']}, "
          f"{len(cb['animations']['gameplay'])} entries, bind {bind['bindName']['$value']})")


def run(args):
    print(">", " ".join(str(a) for a in args))
    r = subprocess.run([str(a) for a in args], capture_output=True, text=True)
    out = (r.stdout or "") + (r.stderr or "")
    print(out.strip()[-2000:])
    if r.returncode != 0:
        raise SystemExit(f"command failed ({r.returncode})")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wkit", metavar="STAGE", help="archive staging root: write STAGE/utpp/vehicle/*.ent and check a round trip")
    ap.add_argument("--wkit-exe", default=str(WKIT_DEFAULT))
    ap.add_argument("--print-xl", action="store_true", help="print the resource section for utpp_vehicle_bike.archive.xl")
    a = ap.parse_args()
    if a.print_xl:
        sys.stdout.write(xl_section())
        return 0
    out_json = JSON_DIR / DEPOT
    out_json.mkdir(parents=True, exist_ok=True)
    for g, fname in FILES.items():
        p = out_json / (fname + ".json")
        p.write_text(json.dumps(entity_template(g), indent=2) + "\n", encoding="utf-8")
        print("json:", p)
    if not a.wkit:
        return 0
    wkit = Path(a.wkit_exe)
    if not wkit.is_file():
        raise SystemExit(f"WolvenKit CLI missing: {wkit}")
    stage = Path(a.wkit)
    depot = stage / DEPOT
    depot.mkdir(parents=True, exist_ok=True)
    rt = stage.parent / "patch-roundtrip"
    tmp = stage.parent / "patch-deser"
    for d in (rt, tmp):
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True)
    for fname in FILES.values():
        src = out_json / (fname + ".json")
        shutil.copy2(src, tmp / src.name)
        run([wkit, "convert", "deserialize", tmp / src.name, "-o", depot])
        built = depot / fname
        if not built.is_file() and (tmp / fname).is_file():
            shutil.move(str(tmp / fname), str(built))
        if not built.is_file():
            raise SystemExit(f"deserialize produced no {fname}")
        run([wkit, "convert", "serialize", built, "-o", rt])
        check_roundtrip(src, rt / (fname + ".json"))
    shutil.rmtree(tmp)
    return 0


if __name__ == "__main__":
    sys.exit(main())
