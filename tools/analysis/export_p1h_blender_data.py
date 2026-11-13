#!/usr/bin/env python3
"""Export the sampled Overture P1H reload and existing corrected poses for Blender.

The output is source data, not a Blender scene or a claim that the capture is an
original authored animation clip. The overlay supplies the armature topology;
the matching extracted player rig supplies rest-local transforms because the
overlay's BONE section stores names and parent indices only.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import struct
import sys
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from compare_action_geometry import fk_position, unit, multiply, rotate  # noqa: E402

DEFAULT_NATIVE = ROOT / "Build/experiments/ReloadTransferOffline/pistol-overture-94-native.txt"
DEFAULT_NATIVE_MANIFEST = ROOT / "Build/experiments/ReloadTransferOffline/pistol-overture-94-native.json"
DEFAULT_OVERLAY = ROOT / "Data/motion/utpp_overlays_woman.utppov"
DEFAULT_RIG = ROOT / "Extracted/game/json/vanilla/player_woman_skeleton.rig.json"
DEFAULT_AIM = ROOT / "Build/experiments/ReloadTransferOffline/prodcap-p1h-aim-base.json"
DEFAULT_READY = ROOT / "Build/experiments/ReloadTransferOffline/prodcap-p1h-ready-base.json"
DEFAULT_OUT = ROOT / "Animations/Blender/Pistol1H/p1h_overture_reload_blender.json"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def overlay_skeleton(path: Path) -> tuple[list[str], list[int]]:
    data = path.read_bytes()
    if len(data) < 64 or data[:8] != b"UTPPOV\0\0":
        raise ValueError(f"Invalid overlay header: {path}")
    bone_count, section_count = struct.unpack_from("<II", data, 24)
    if not (0 < bone_count <= 128) or 64 + section_count * 16 > len(data):
        raise ValueError("Invalid overlay section table/count")
    if struct.unpack_from("<I", data, 36)[0] != zlib.crc32(data[64:]):
        raise ValueError("Overlay CRC mismatch")
    for i in range(section_count):
        tag, offset, size, count = struct.unpack_from("<4sIII", data, 64 + i * 16)
        if tag != b"BONE":
            continue
        if count != bone_count or size != bone_count * 36 or offset + size > len(data):
            raise ValueError("Invalid overlay BONE section")
        names, parents = [], []
        for j in range(bone_count):
            raw = data[offset + j * 36:offset + j * 36 + 32].split(b"\0", 1)[0]
            names.append(raw.decode("utf-8"))
            parents.append(struct.unpack_from("<i", data, offset + j * 36 + 32)[0])
        if len(set(names)) != bone_count or any(p < -1 or p >= j for j, p in enumerate(parents)):
            raise ValueError("Invalid overlay bone names/topology")
        return names, parents
    raise ValueError("Overlay has no BONE section")


def overlay_static_pose(path: Path, wanted: str, bone_count: int) -> list[list[float]]:
    """Read one authored local pose from the overlay's POSE/FRAM sections."""
    data = path.read_bytes()
    _, section_count = struct.unpack_from("<II", data, 24)
    sections = {}
    for i in range(section_count):
        tag, offset, size, count = struct.unpack_from("<4sIII", data, 64 + i * 16)
        if offset + size > len(data):
            raise ValueError("Overlay section bounds")
        sections[tag.decode("ascii")] = (offset, size, count)
    if "POSE" not in sections or "FRAM" not in sections or "CURV" not in sections:
        raise ValueError("Overlay missing POSE/FRAM/CURV")
    pose_offset, _, pose_count = sections["POSE"]
    selected = None
    for i in range(pose_count):
        pos = pose_offset + i * 104
        name = data[pos:pos + 64].split(b"\0", 1)[0].decode("utf-8")
        if name == wanted:
            selected = struct.unpack_from("<I", data, pos + 68)[0]
            break
    if selected is None:
        raise ValueError(f"Overlay pose not found: {wanted}")
    frame_offset, frame_size, _ = sections["FRAM"]
    curve_count = sections["CURV"][2]
    floats_per_frame = bone_count * 17 + curve_count
    needed = (selected + 1) * floats_per_frame * 4
    if selected < 0 or needed > frame_size:
        raise ValueError(f"Invalid FRAM index for {wanted}")
    pose = []
    for i in range(bone_count):
        vals = struct.unpack_from("<10f", data, frame_offset + (selected * floats_per_frame + i * 17) * 4)
        if not all(math.isfinite(x) for x in vals):
            raise ValueError(f"Non-finite overlay transform {wanted} bone {i}")
        pose.append(list(vals[:3]) + list(unit(vals[3:7])) + list(vals[7:10]))
    return pose


def qstransform(value: dict) -> dict:
    t = value["Translation"]
    q = value["Rotation"]
    s = value["Scale"]
    return {
        "translation": [float(t[k]) for k in ("X", "Y", "Z")],
        "rotation_xyzw": [float(q[k]) for k in ("i", "j", "k", "r")],
        "scale": [float(s[k]) for k in ("X", "Y", "Z")],
    }


def load_rest(path: Path, overlay_names: list[str], overlay_parents: list[int]) -> list[dict]:
    root = json.loads(path.read_text(encoding="utf-8"))["Data"]["RootChunk"]
    names = [x["$value"] for x in root["boneNames"]]
    parents = [int(x) for x in root["boneParentIndexes"]]
    if names[:len(overlay_names)] != overlay_names or parents[:len(overlay_names)] != overlay_parents:
        raise ValueError("Extracted rig does not exactly match overlay bone names/parents")
    return [qstransform(root["boneTransforms"][i]) for i in range(len(overlay_names))]


def parse_native(path: Path) -> tuple[list[str], list[dict]]:
    tokens = iter(path.read_text(encoding="utf-8").split())
    bone_count, frame_count = int(next(tokens)), int(next(tokens))
    names = [next(tokens) for _ in range(bone_count)]
    frames = []
    for ordinal in range(frame_count):
        tick = int(next(tokens))
        bones = {}
        for name in names:
            v = [float(next(tokens)) for _ in range(7)]
            bones[name] = {"translation": v[:3], "rotation_xyzw": unit(v[3:]), "scale": [1.0, 1.0, 1.0]}
        frames.append({"ordinal": ordinal, "ticks": tick, "bones": bones})
    if next(tokens, None) is not None:
        raise ValueError("Trailing tokens in native fixture")
    return names, frames


def load_corrected(path: Path, expected_names: list[str], expected_ticks: list[int]) -> dict[int, dict[str, dict]]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    names = raw.get("bone_names")
    if names != expected_names:
        raise ValueError(f"Corrected bone_names differ from overlay: {path.name}")
    result = {}
    for frame in raw.get("frames", []):
        if not frame.get("accepted", False):
            raise ValueError(f"Rejected frame in corrected input: {path.name} tick={frame.get('ticks')}")
        tick = int(frame["ticks"])
        values = frame["bones"]
        if len(values) != len(names):
            raise ValueError(f"Bone count mismatch at {tick} in {path.name}")
        result[tick] = {}
        for name, v in zip(names, values):
            if len(v) != 10 or not all(math.isfinite(float(x)) for x in v):
                raise ValueError(f"Malformed transform {name} at tick {tick}")
            q = unit([float(x) for x in v[3:7]])
            result[tick][name] = {"translation": [float(x) for x in v[:3]],
                                  "rotation_xyzw": q, "scale": [float(x) for x in v[7:10]]}
    if sorted(result) != expected_ticks:
        raise ValueError(f"Corrected frame ticks do not exactly match native reload samples: {path.name}")
    return result


def qcompose(a: list[float], b: list[float]) -> list[float]:
    return list(unit(multiply(a, b)))


def model_landmarks(frame_bones: dict[str, dict], parents_by_name: dict[str, str | None], names: list[str]) -> dict:
    local = {i: (frame_bones[name]["translation"], frame_bones[name]["rotation_xyzw"])
             for i, name in enumerate(names)}
    parent_indices = {i: (names.index(parents_by_name[name]) if parents_by_name[name] is not None else -1)
                      for i, name in enumerate(names)}
    landmarks = {}
    for name in ("LeftHand", "RightHand", "WeaponLeft", "WeaponRight"):
        i = names.index(name)
        p, q = fk_position(i, local, [parent_indices[j] for j in range(len(names))], {})
        landmarks[name] = {"position_m": list(p), "rotation_xyzw": list(unit(q)), "space": "overlay armature model space"}
    return landmarks


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--native", type=Path, default=DEFAULT_NATIVE)
    p.add_argument("--native-manifest", type=Path, default=DEFAULT_NATIVE_MANIFEST)
    p.add_argument("--overlay", type=Path, default=DEFAULT_OVERLAY)
    p.add_argument("--rig", type=Path, default=DEFAULT_RIG)
    p.add_argument("--aim", type=Path, default=DEFAULT_AIM)
    p.add_argument("--ready", type=Path, default=DEFAULT_READY)
    p.add_argument("--output", type=Path, default=DEFAULT_OUT)
    a = p.parse_args()

    names, parents = overlay_skeleton(a.overlay)
    rest = load_rest(a.rig, names, parents)
    native_names, native_frames = parse_native(a.native)
    manifest = json.loads(a.native_manifest.read_text(encoding="utf-8"))
    if len(native_frames) != 96 or manifest.get("totalFrames") != 96 or manifest.get("reloadFrames") != 94:
        raise ValueError("Expected exactly pre + 94 reload + post native samples")
    if manifest.get("sourceSHA256") != sha256(Path(manifest["source"])):
        raise ValueError("Native trace source hash does not match original extraction manifest")
    if len(native_names) != 71 or any(n not in names for n in native_names):
        raise ValueError("Unexpected native sample bone set")
    if native_names != names[2:]:
        raise ValueError("Native sampled bones must exactly cover overlay bones after Root/Trajectory")

    trace_path = Path(manifest["source"])
    header = trace_path.open(encoding="utf-8", errors="replace").readline()
    m = re.search(r"qpc_hz=(\d+)", header)
    if not m:
        raise ValueError("QPC frequency missing from source trace header")
    qpc_hz = int(m.group(1))
    ticks = [f["ticks"] for f in native_frames]
    base_tick = ticks[0]
    corrected_aim = load_corrected(a.aim, names, ticks[1:-1])
    corrected_ready = load_corrected(a.ready, names, ticks[1:-1])
    sourcepack_aim = overlay_static_pose(a.overlay, "Pose_Pistol1H_Stand_Aim_Idle", len(names))
    sourcepack_ready = overlay_static_pose(a.overlay, "Pose_Pistol1H_Stand_Ready_Idle", len(names))
    parents_by_name = {n: (names[parents[i]] if parents[i] >= 0 else None) for i, n in enumerate(names)}

    # Missing Root/Trajectory channels were not sampled by the native runtime fixture.
    # Keep them at extracted rest, and label their provenance rather than presenting them as captured.
    native_track = []
    for f in native_frames:
        bones = {n: dict(rest[i]) for i, n in enumerate(names[:2])}
        bones.update(f["bones"])
        phase = "pre_boundary" if f["ordinal"] == 0 else "post_boundary" if f["ordinal"] == 95 else "reload_sample"
        native_track.append({
            "ordinal": f["ordinal"], "tick": f["ticks"],
            "frame": (f["ticks"] - base_tick) * 60.0 / qpc_hz,
            "time_seconds": (f["ticks"] - base_tick) / qpc_hz,
            "action_name": "Pistol 1H Reload" if phase == "reload_sample" else "Reload boundary / neutral",
            "clip_name": None,
            "phase": phase, "action_code": 0 if f["ordinal"] in (0, 95) else 2,
            "local": [[*(bones[n]["translation"]), *(bones[n]["rotation_xyzw"]), *(bones[n]["scale"])] for n in names],
            "unobserved_channels": {"Root": "static_rest_unobserved", "Trajectory": "static_rest_unobserved"},
            "landmarks_model": model_landmarks(bones, parents_by_name, names),
        })

    def corrected_track(data: dict[int, dict[str, dict]], label: str) -> list[dict]:
        out = []
        for ordinal, tick in enumerate(ticks[1:-1], 1):
            bones = data[tick]
            out.append({
                "ordinal": ordinal, "tick": tick,
                "frame": (tick - base_tick) * 60.0 / qpc_hz,
                "time_seconds": (tick - base_tick) / qpc_hz,
                "phase": "reload_corrected_pose",
                "action_name": label, "clip_name": None,
                "local": [[*(bones[n]["translation"]), *(bones[n]["rotation_xyzw"]), *(bones[n]["scale"])] for n in names],
                "unobserved_channels": {},
                "landmarks_model": model_landmarks(bones, parents_by_name, names),
            })
        return out

    def static_overlay_track(pose: list[list[float]], name: str, action_name: str) -> list[dict]:
        return [{"ordinal": ordinal, "tick": tick,
                 "frame": (tick - base_tick) * 60.0 / qpc_hz,
                 "time_seconds": (tick - base_tick) / qpc_hz,
                 "action_name": action_name, "clip_name": name,
                 "phase": "static_overlay_reference_repeated_per_tick",
                 "local": pose,
                 "landmarks_model": model_landmarks(
                     {bone: {"translation": pose[i][:3], "rotation_xyzw": pose[i][3:7], "scale": pose[i][7:10]}
                      for i, bone in enumerate(names)}, parents_by_name, names)}
                for ordinal, tick in enumerate(ticks[1:-1], 1)]

    sources = [a.native, a.native_manifest, trace_path, a.overlay, a.rig, a.aim,
               Path(str(a.aim) + ".authored.txt"), a.ready, Path(str(a.ready) + ".authored.txt")]
    payload = {
        "schema": "utpp.blender.sampled_armature_animation.v1",
        "title": "Female V Pistol 1H Overture reload transfer",
        "provenance_class": "runtime_sampled_and_offline_corrected; not an original authored clip",
        "coordinate_space": {
            "local_transforms": "per-bone local transforms in Cyberpunk player rig basis",
            "translation_unit": "meter",
            "rotation": "unit quaternion XYZW",
            "scale": "XYZ scale; native sampled channels use 1,1,1",
            "timebase": {"source": "QPC ticks", "qpc_hz": qpc_hz, "fps": 60,
                         "frame_origin": "first native pre-boundary sample", "fractional_frames_preserved": True},
        },
        "bone_names": names,
        "parents": parents,
        "restLocal": [[*rest[i]["translation"], *rest[i]["rotation_xyzw"], *rest[i]["scale"]] for i in range(len(names))],
        "tracks": [
            {"name": "native_reload_sampled", "action_name": "Pistol 1H Reload", "clip_name": None,
             "description": "Runtime-sampled native pose; 94 reload action frames plus two boundary frames.",
             "frames": native_track},
            {"name": "corrected_tpp_aim", "action_name": "Pistol 1H Aim", "clip_name": None,
             "description": "Existing offline production-cap P1H Aim correction, paired by exact source tick.",
             "frames": corrected_track(corrected_aim, "Pistol 1H Aim")},
            {"name": "corrected_tpp_ready", "action_name": "Pistol 1H Ready", "clip_name": None,
             "description": "Existing offline production-cap P1H Ready correction, paired by exact source tick.",
             "frames": corrected_track(corrected_ready, "Pistol 1H Ready")},
            {"name": "gasp_authored_aim", "action_name": "Pistol 1H Aim", "clip_name": "Pose_Pistol1H_Stand_Aim_Idle",
             "description": "Static authored overlay pose repeated at each reload sample tick; reference only, not a per-tick retarget or animated clip.",
             "frames": static_overlay_track(sourcepack_aim, "Pose_Pistol1H_Stand_Aim_Idle", "Pistol 1H Aim")},
            {"name": "gasp_authored_ready", "action_name": "Pistol 1H Ready", "clip_name": "Pose_Pistol1H_Stand_Ready_Idle",
             "description": "Static authored overlay pose repeated at each reload sample tick; reference only, not a per-tick retarget or animated clip.",
             "frames": static_overlay_track(sourcepack_ready, "Pose_Pistol1H_Stand_Ready_Idle", "Pistol 1H Ready")},
        ],
        "landmark_semantics": "FK-derived model-space positions/orientations from exported local transforms; no mesh collision or visual-quality claim.",
        "sources": [{"path": str(x.relative_to(ROOT)) if x.is_relative_to(ROOT) else str(x), "sha256": sha256(x)} for x in sources],
        "limitations": [
            "The overlay stores hierarchy names/parent indices, not rest transforms; rest-local data comes from the matching extracted player rig.",
            "Root and Trajectory were absent from the 71-bone native runtime capture; their exported native action values are static rest placeholders marked unobserved.",
            "Native data is sampled at runtime and corrected Aim/Ready data is an offline retarget result; neither is the original authored game animation clip.",
            "No Blender import, mesh deformation, contact preservation, or visual acceptance was run by this exporter.",
        ],
    }
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(a.output), "bones": len(names), "nativeFrames": len(native_track),
                      "aimFrames": len(corrected_aim), "readyFrames": len(corrected_ready),
                      "frameRange60fps": [native_track[0]["frame"], native_track[-1]["frame"]],
                      "sha256": sha256(a.output)}, indent=2))


if __name__ == "__main__":
    main()
