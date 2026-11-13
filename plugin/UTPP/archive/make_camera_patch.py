"""Build the ArchiveXL entity patch that adds the UTPP camera component to the player entities.

The patch resource is authored here from the component schema only (type, name, id, fov).
ArchiveXL 1.27.3 (ResourcePatch/Extension.cpp, PatchPackageResults + MergeComponents) merges the
components found in the patch's *compiledData* package into the target entity when the entity is extracted, so the
package below holds a bare root entity plus one camera component. The `.xl` limits the patch to `components`
(props), so the root entity of the patch is never merged.

Outputs (nothing is written to the game folder):
  archive/json/utpp/camera/utpp_player_camera.ent.json            variant A1: gameCameraComponent   (default)
  archive/json/utpp/camera/utpp_player_camera_fpptype.ent.json    variant A2: gameFPPCameraComponent (fallback)
  Build/UTPP/archive_stage/raw/utpp_camera/utpp/camera/*.ent       WolvenKit CLI `convert deserialize`
  Build/UTPP/archive_stage/utpp_camera.archive                     WolvenKit CLI `pack`
  Build/UTPP/archive_stage/utpp_camera.archive.xl                  copy of the active variant's .xl
  Build/UTPP/archive_stage/roundtrip/*.json                        `convert serialize` of the built .ent (check)

Usage: python make_camera_patch.py [--json-only] [--variant a1|a2]
"""
import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p / "Research").is_dir() and (p / "Tools").is_dir())
WKIT = ROOT / "Tools" / "WolvenKit.CLI" / "WolvenKit.CLI.exe"
STAGE = ROOT / "Build" / "UTPP" / "archive_stage"
JSON_DIR = HERE / "json"
DEPOT_DIR = Path("utpp") / "camera"

COMPONENT_NAME = "utppCamera"
# CRUIDs: ASCII "UTPPCAM1" / "UTPPCAM2" (unique, never used by the game; ArchiveXL matches components by name + id)
CRUID_A1 = str(0x5554505043414D31)
CRUID_A2 = str(0x5554505043414D32)
CAMERA_FOV = 60.0  # start value only; the CET controller sets the FOV every frame (SetFOV)

CAMERA_ANIM_PARAMS = [
    "animParamdofFarBlur", "animParamdofFarFocus", "animParamdofIntensity", "animParamdofNearBlur",
    "animParamdofNearFocus", "animParamFovOverrideValue", "animParamFovOverrideWeight",
    "animParamWeaponBlurIntensity", "animParamWeaponEdgesSharpness", "animParamWeaponFarPlaneCM",
    "animParamWeaponNearPlaneCM", "animParamWeaponVignetteCircular", "animParamWeaponVignetteIntensity",
    "animParamWeaponVignetteRadius", "animParamZoomOverrideValue", "animParamZoomOverrideWeight",
    "animParamZoomWeaponOverrideValue", "animParamZoomWeaponOverrideWeight",
]


def cname(v):
    return {"$type": "CName", "$storage": "string", "$value": v}


def identity_transform():
    fp = {"$type": "FixedPoint", "Bits": 0}
    return {"$type": "WorldTransform",
            "Orientation": {"$type": "Quaternion", "i": 0, "j": 0, "k": 0, "r": 1},
            "Position": {"$type": "WorldPosition", "x": dict(fp), "y": dict(fp), "z": dict(fp)}}


def camera_component(variant):
    """Own camera component. No parentTransform: the component lives in the entity (root) space, so it does not
    inherit the FPP camera bone motion (design report 2.1, binding (a))."""
    c = {"$type": "gameCameraComponent" if variant == "a1" else "gameFPPCameraComponent"}
    # CameraComponent: animation parameters unset (no weapon zoom / DOF / FOV driven by the animation graph)
    for p in CAMERA_ANIM_PARAMS:
        c[p] = cname("None")
    c.update({
        "farPlaneOverride": 0,
        "fov": CAMERA_FOV,
        "fovOverrideValue": 0,
        "fovOverrideWeight": 0,
        "id": CRUID_A1 if variant == "a1" else CRUID_A2,
        "isReplicable": 0,
        "localTransform": identity_transform(),
        "motionBlurScale": 1,
        "name": cname(COMPONENT_NAME),
        "nearPlaneOverride": 0,
        "parentTransform": None,
        "weaponPlane": {"$type": "SWeaponPlaneParams", "blurIntensity": 0, "weaponNearPlaneCM": 0},
        "zoom": 1,
        "zoomOverrideValue": 0,
        "zoomOverrideWeight": 0,
        "zoomWeaponOverrideValue": 0,
        "zoomWeaponOverrideWeight": 0,
    })
    if variant == "a2":
        # FPP camera type (fallback): own input integration and limits neutralised; UTPP writes the orientation.
        c.update({"headingLocked": 0, "pitchMax": 0, "pitchMin": 0, "yawMaxLeft": 0, "yawMaxRight": 0,
                  "sensitivityMultX": 0, "sensitivityMultY": 0, "timeDilationCurveName": cname("None")})
    return dict(sorted(c.items(), key=lambda kv: (kv[0] != "$type", kv[0].lower())))


def entity_template(variant):
    comp = camera_component(variant)
    package = {
        "Version": 4,
        "Sections": 6,  # computed by WolvenKit on write (round trip: 6)
        "CruidIndex": 0,
        "CruidDict": {"0": "0", "1": comp["id"]},
        "Chunks": [{"$type": "entEntity"}, comp],
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
        "components": [comp],
        "cookingPlatform": "PLATFORM_PC",
        "defaultAppearance": cname("None"),
        "entity": {"HandleId": "0", "Data": {"$type": "entEntity"}},
        "includeInstanceBuffer": None,
        "includes": [],
        "inplaceResources": [],
        "localData": None,
        "resolvedDependencies": [],
        "visualTagsSchema": None,
    }
    return {"Header": {"WolvenKitVersion": "9.0.1", "WKitJsonVersion": "0.0.9", "GameVersion": 2310,
                       "ExportedDateTime": "2026-10-07T00:00:00Z", "DataType": "CR2W",
                       "ArchiveFileName": ""},
            "Data": {"Version": 195, "BuildVersion": 0, "RootChunk": root, "EmbeddedFiles": []}}


VARIANT_FILES = {"a1": "utpp_player_camera.ent", "a2": "utpp_player_camera_fpptype.ent"}


def sha256(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def check_roundtrip(src_json, rt_json):
    """The built .ent must serialize back to the authored component (WolvenKit may only add entity defaults)."""
    src = json.loads(Path(src_json).read_text(encoding="utf-8"))["Data"]["RootChunk"]
    rt = json.loads(Path(rt_json).read_text(encoding="utf-8"))["Data"]["RootChunk"]
    a, b = src["compiledData"]["Data"], rt["compiledData"]["Data"]
    if a["Chunks"][1] != b["Chunks"][1] or a["CruidDict"] != b["CruidDict"] or src["components"] != rt["components"]:
        raise SystemExit(f"round trip mismatch: {rt_json}")
    print(f"round trip ok: {Path(rt_json).name} ({b['Chunks'][1]['$type']} {b['Chunks'][1]['name']['$value']})")


def run(args):
    print(">", " ".join(str(a) for a in args))
    r = subprocess.run([str(a) for a in args], capture_output=True, text=True)
    out = (r.stdout or "") + (r.stderr or "")
    print(out.strip()[-3000:])
    if r.returncode != 0:
        raise SystemExit(f"command failed ({r.returncode})")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json-only", action="store_true")
    ap.add_argument("--variant", choices=["a1", "a2"], default="a1", help="variant whose .xl is staged")
    a = ap.parse_args()

    out_json = JSON_DIR / DEPOT_DIR
    out_json.mkdir(parents=True, exist_ok=True)
    for v, fname in VARIANT_FILES.items():
        p = out_json / (fname + ".json")
        p.write_text(json.dumps(entity_template(v), indent=2) + "\n", encoding="utf-8")
        print("json:", p)
    if a.json_only:
        return

    if not WKIT.is_file():
        raise SystemExit(f"WolvenKit CLI missing: {WKIT}")
    raw = STAGE / "raw" / "utpp_camera"
    if raw.exists():
        shutil.rmtree(raw)
    (raw / DEPOT_DIR).mkdir(parents=True)
    # deserialize each JSON into the raw depot tree
    for fname in VARIANT_FILES.values():
        src = out_json / (fname + ".json")
        tmp = STAGE / "deser"
        if tmp.exists():
            shutil.rmtree(tmp)
        tmp.mkdir(parents=True)
        shutil.copy2(src, tmp / src.name)
        run([WKIT, "convert", "deserialize", tmp / src.name, "-o", raw / DEPOT_DIR])
        built = raw / DEPOT_DIR / fname
        if not built.is_file():
            # older CLI builds write next to the json
            alt = tmp / fname
            if alt.is_file():
                shutil.move(str(alt), str(built))
        if not built.is_file():
            raise SystemExit(f"deserialize produced no {fname}")
        shutil.rmtree(tmp)
    # round trip check: serialize the binary back to json
    rt = STAGE / "roundtrip"
    if rt.exists():
        shutil.rmtree(rt)
    rt.mkdir(parents=True)
    for fname in VARIANT_FILES.values():
        run([WKIT, "convert", "serialize", raw / DEPOT_DIR / fname, "-o", rt])
        check_roundtrip(out_json / (fname + ".json"), rt / (fname + ".json"))
    # pack
    archive = STAGE / "utpp_camera.archive"
    if archive.exists():
        archive.unlink()
    run([WKIT, "pack", raw, "-o", STAGE])
    if not archive.is_file():
        raise SystemExit("pack produced no utpp_camera.archive")
    xl_src = HERE / ("utpp_camera.archive.xl" if a.variant == "a1" else "variant_fpptype/utpp_camera.archive.xl")
    shutil.copy2(xl_src, STAGE / "utpp_camera.archive.xl")
    # the other variant's .xl, for a manual switch (only one .xl next to the archive may be installed)
    other = "variant_fpptype" if a.variant == "a1" else "variant_default"
    (STAGE / other).mkdir(exist_ok=True)
    shutil.copy2(HERE / ("variant_fpptype/utpp_camera.archive.xl" if a.variant == "a1" else "utpp_camera.archive.xl"),
                 STAGE / other / "utpp_camera.archive.xl")
    for p in [archive, STAGE / "utpp_camera.archive.xl", STAGE / other / "utpp_camera.archive.xl"] + \
             [raw / DEPOT_DIR / f for f in VARIANT_FILES.values()]:
        print(f"{sha256(p)}  {p.relative_to(ROOT)}  ({p.stat().st_size} B)")


if __name__ == "__main__":
    sys.exit(main())
