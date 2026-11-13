"""Vehicle animset data check (read-only).

Extracts (WolvenKit.CLI unbundle, read-only on the game archives) the player
entities and the sport/sportbike character + vehicle animsets, serializes them
to JSON and writes a fact table: animset registrations on the player entities
(path, priority, variables), clip names, durations, root motion, events and
rig compatibility.

Usage (python -I):
  vehicle_anims_check.py --wkit <WolvenKit.CLI.exe> --game <Cyberpunk 2077 dir>
                         --out <Extracted\\game\\vehicle-anims-2.31> --table <json>
                         [--skip-extract]
Exit 0 = listing produced. Nothing is written to the game folder.
"""
import argparse
import hashlib
import json
import math
import os
import re
import struct
import subprocess
import sys

ARCHIVES = {
    "basegame_1_engine": r"archive\pc\content\basegame_1_engine.archive",
    "basegame_4_animation": r"archive\pc\content\basegame_4_animation.archive",
    "ep1_2_gamedata": r"archive\pc\ep1\ep1_2_gamedata.archive",
}

# (archive key, regex) - first pass: entities and animsets.
PASS1 = [
    ("basegame_1_engine",
     r"(entities\\player\\player_(wa|ma)_(fpp|tpp_reflexion)\.ent$)"
     r"|(vehicle\\(sport|sportbike[23]?)\\character\\(tpp_)?p(w|m)a_[^\\]*\.anims$)"
     r"|(vehicle\\generic\\pwa_generic_driver\.anims$)"
     r"|(((player_(wo)?man_skeleton)|(woman_base)|(man_base))\.rig$)"),
    ("basegame_4_animation",
     r"(vehicle\\(sport|sportbike[23]?)\\character\\(wa|ma)_[^\\]*\.anims$)"
     r"|(vehicle\\sport\\vehicle\\v_sport(1_quadra_turbo|2_porsche_911turbo)\.anims$)"
     r"|(vehicle\\sportbike[23]?\\vehicle\\[^\\]*\.anims$)"
     r"|(vehicle\\generic\\generic_bike\.anims$)"),
    ("ep1_2_gamedata",
     r"entities\\player\\player_[^\\]*_ep1\.ent$"),
]


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    return p.returncode, p.stdout + p.stderr


def list_archive(wkit, arc, regex):
    rc, out = run([wkit, "archive", arc, "-l", "-r", regex])
    files = [l.strip() for l in out.splitlines() if l.strip().lower().endswith((".ent", ".anims", ".rig"))]
    return rc, sorted(set(files))


def unbundle(wkit, arc, regex, outdir):
    os.makedirs(outdir, exist_ok=True)
    rc, out = run([wkit, "unbundle", arc, "-o", outdir, "-r", regex])
    return rc, out


def serialize(wkit, raw, jsondir):
    os.makedirs(jsondir, exist_ok=True)
    rc, out = run([wkit, "convert", "serialize", raw, "-o", jsondir])
    return rc, out


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest().upper()


def walk_files(root, exts):
    for d, _, fs in os.walk(root):
        for f in fs:
            if f.lower().endswith(exts):
                yield os.path.join(d, f)


# ---------------------------------------------------------------- JSON helpers
def jload(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def root_chunk(doc):
    return doc["Data"]["RootChunk"]


def val(x):
    """Unwrap WolvenKit JSON scalars ({'$type':'CName','$value':..} etc.)."""
    if isinstance(x, dict):
        if "$value" in x:
            return x["$value"]
        if "DepotPath" in x:
            return val(x["DepotPath"])
    return x


def iter_nodes(x):
    if isinstance(x, dict):
        yield x
        for v in x.values():
            yield from iter_nodes(v)
    elif isinstance(x, list):
        for v in x:
            yield from iter_nodes(v)


def do_extract(args, report):
    raw = os.path.join(args.out, "raw")
    jsd = os.path.join(args.out, "json")
    prov = {}
    for key, rx in PASS1:
        arc = os.path.join(args.game, ARCHIVES[key])
        rc, files = list_archive(args.wkit, arc, rx)
        report.setdefault("extract_log", []).append({"archive": key, "list_rc": rc, "count": len(files)})
        for f in files:
            prov.setdefault(f, []).append(key)
        rc, out = unbundle(args.wkit, arc, rx, raw)
        report["extract_log"][-1]["unbundle_rc"] = rc
        if rc != 0:
            report["extract_log"][-1]["unbundle_tail"] = out[-2000:]
    return prov


# ---------------------------------------------------------------- root motion
def decode_motion(me):
    """animSplineCompressedMotionExtraction: 16-byte keys.
    pos key: u32 (low 16 bits = normalised time) + 3 x f32 (x, y, z, metres).
    rot key: u16 time, u16 flags, 3 x f32 quaternion (i, j, k), r implied.
    Returns end displacement, path length, max height and end yaw (deg)."""
    if not me:
        return None
    d = me["Data"]
    out = {"type": d["$type"]}
    if d["$type"] != "animSplineCompressedMotionExtraction":
        return out
    dur = float(d.get("duration") or 0.0)
    pk = bytes(d.get("posKeysData") or [])
    rk = bytes(d.get("rotKeysData") or [])
    pos = [struct.unpack_from("<I3f", pk, i) for i in range(0, len(pk) - 15, 16)]
    rot = [struct.unpack_from("<HH3f", rk, i) for i in range(0, len(rk) - 15, 16)]
    if pos:
        end = pos[-1][1:]
        plen = 0.0
        for a, b in zip(pos, pos[1:]):
            plen += math.dist(a[1:], b[1:])
        out.update({
            "pos_keys": len(pos),
            "end_xyz_m": [round(v, 3) for v in end],
            "planar_m": round(math.hypot(end[0], end[1]), 3),
            "path_m": round(plen, 3),
            "z_min_m": round(min(p[3] for p in pos), 3),
            "z_max_m": round(max(p[3] for p in pos), 3),
            "pos_end_time_s": round((pos[-1][0] & 0xFFFF) / 65535.0 * dur, 3),
        })
    if rot:
        i, j, k = rot[-1][2:]
        r = math.sqrt(max(0.0, 1.0 - (i * i + j * j + k * k)))
        yaw = math.degrees(2.0 * math.atan2(k, r))
        out.update({"rot_keys": len(rot), "end_yaw_deg": round(yaw, 1),
                    "end_quat_ijk": [round(i, 4), round(j, 4), round(k, 4)]})
    return out


# ---------------------------------------------------------------- animsets
def summarize_events(entry):
    ev = entry.get("events")
    res = []
    if not ev:
        return res
    for x in ev["Data"]["events"]:
        d = x["Data"]
        item = {"type": d["$type"].replace("animAnimEvent_", ""), "name": val(d.get("eventName")),
                "frame": d.get("startFrame"), "dur_frames": d.get("durationInFrames")}
        for extra in ("actionName", "side", "customEvent"):
            if extra in d and val(d[extra]) not in (None, "None"):
                item[extra] = val(d[extra])
        res.append(item)
    res.sort(key=lambda e: (e["frame"] or 0, e["type"]))
    return res


def load_animset(path):
    rc = root_chunk(jload(path))
    clips = {}
    for e in rc["animations"]:
        ed = e["Data"]
        an = ed["animation"]["Data"]
        buf = an["animBuffer"]["Data"]
        name = val(an["name"])
        ftracks = [val(t["trackInfo"]["name"]) for t in (an.get("additionalTracks") or {}).get("entries", [])]
        clips[name] = {
            "duration_s": round(float(an["duration"]), 4),
            "frames": buf.get("numFrames"),
            "joints": buf.get("numJoints"),
            "extra_joints": buf.get("numExtraJoints"),
            "tracks": buf.get("numTracks"),
            "type": an.get("animationType"),
            "root_motion": decode_motion(an.get("motionExtraction")),
            "events": summarize_events(ed),
            "float_tracks": ftracks,
        }
    return {"rig": val(rc["rig"]), "clip_count": len(clips), "clips": clips}


# ---------------------------------------------------------------- rigs
def load_rig(path):
    rc = root_chunk(jload(path))
    names = [val(n) for n in rc["boneNames"]]
    parents = rc["boneParentIndexes"]
    tr = []
    for t in rc["boneTransforms"]:
        v = t["Translation"]
        tr.append((v["X"], v["Y"], v["Z"]))
    return {"names": names, "parents": parents, "local_t": tr,
            "tracks": [val(n) for n in rc.get("trackNames") or []],
            "retargets_raw": json.dumps(rc.get("retargets"))[:600]}


def compare_rigs(a, b):
    """a = source (clip rig), b = target (player rig)."""
    ia = {n: i for i, n in enumerate(a["names"])}
    ib = {n: i for i, n in enumerate(b["names"])}
    common = [n for n in a["names"] if n in ib]
    only_a = [n for n in a["names"] if n not in ib]
    only_b = [n for n in b["names"] if n not in ia]
    same_index = sum(1 for n in common if ia[n] == ib[n])

    def pname(rig, idx, i):
        p = rig["parents"][i]
        return rig["names"][p] if p >= 0 else None
    parent_mismatch = [n for n in common if pname(a, ia, ia[n]) != pname(b, ib, ib[n])]
    diffs = []
    for n in common:
        ta, tb = a["local_t"][ia[n]], b["local_t"][ib[n]]
        la, lb = math.hypot(*ta), math.hypot(*tb)
        diffs.append((n, round(la, 4), round(lb, 4), round(math.dist(ta, tb), 4)))
    diffs.sort(key=lambda x: -x[3])
    key = ["Hips", "Spine", "Spine1", "Spine2", "Spine3", "Neck", "Neck1", "Head",
           "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
           "RightShoulder", "RightArm", "RightForeArm", "RightHand",
           "LeftUpLeg", "LeftLeg", "LeftFoot", "RightUpLeg", "RightLeg", "RightFoot"]
    key_missing = [n for n in key if n not in ia or n not in ib]
    return {
        "bones_source": len(a["names"]), "bones_target": len(b["names"]),
        "common": len(common), "same_index": same_index,
        "source_prefix_equals_target_prefix": a["names"][:len(common)] == b["names"][:len(common)],
        "only_in_source": only_a, "only_in_target_count": len(only_b), "only_in_target_sample": only_b[:40],
        "parent_mismatch": parent_mismatch,
        "key_bones_missing": key_missing,
        "max_local_translation_delta_m": diffs[0] if diffs else None,
        "top_translation_deltas": diffs[:12],
        "mean_local_translation_delta_m": round(sum(d[3] for d in diffs) / len(diffs), 4) if diffs else None,
    }


# ---------------------------------------------------------------- entities
VEH_RX = re.compile(r"\\animations\\vehicle\\", re.I)


def ent_vehicle_entries(path):
    rc = root_chunk(jload(path))
    res = {"components": [], "includes": [val(i) for i in rc.get("includes") or []]}
    comp_lists = {"components": rc.get("components") or []}
    cd = rc.get("compiledData")
    if isinstance(cd, dict) and isinstance(cd.get("Data"), dict):
        comp_lists["compiledData"] = cd["Data"].get("Chunks") or []
    for where, comps in comp_lists.items():
        for c in comps:
            if not isinstance(c, dict) or "animations" not in c:
                continue
            anims = c["animations"] or {}
            entries = []
            for gi, g in enumerate(anims.get("gameplay") or []):
                p = val(g.get("animSet"))
                if p and VEH_RX.search("\\" + p):
                    entries.append({"idx": gi, "set": p, "priority": g.get("priority"),
                                    "vars": [val(v) for v in g.get("variableNames") or []]})
            info = {"where": where, "type": c.get("$type"), "name": val(c.get("name")),
                    "rig": val(c.get("rig")) if isinstance(c.get("rig"), dict) else None,
                    "gameplay_total": len(anims.get("gameplay") or []), "vehicle_entries": entries}
            for k in ("animatedComponentName", "targetComponentName"):
                if k in c:
                    info[k] = val(c[k])
            if entries or info["type"] == "entAnimatedComponent":
                res["components"].append(info)
    return res


def find(entries, pattern):
    rx = re.compile(pattern, re.I)
    return [e for e in entries if rx.search(e["set"])]


def clip_cmp(src, dst, names, rig_ok):
    rows = []
    for n in names:
        a = src["clips"].get(n)
        b = dst["clips"].get(n)
        row = {"clip": n, "npc": None, "player": None}
        if a:
            row["npc"] = {"dur": a["duration_s"], "joints": a["joints"], "rm": a["root_motion"],
                          "events": [f'{e["type"]}:{e["name"]}@{e["frame"]}' for e in a["events"]]}
        if b:
            row["player"] = {"dur": b["duration_s"], "joints": b["joints"], "rm": b["root_motion"],
                             "events": [f'{e["type"]}:{e["name"]}@{e["frame"]}' for e in b["events"]]}
        if a and b:
            row["dur_delta_s"] = round(a["duration_s"] - b["duration_s"], 3)
            ra, rb = a["root_motion"] or {}, b["root_motion"] or {}
            if "end_xyz_m" in ra and "end_xyz_m" in rb:
                row["rm_end_delta_m"] = round(math.dist(ra["end_xyz_m"], rb["end_xyz_m"]), 3)
            if "end_yaw_deg" in ra and "end_yaw_deg" in rb:
                row["rm_yaw_delta_deg"] = round(ra["end_yaw_deg"] - rb["end_yaw_deg"], 1)
            row["dur_ok_0.05"] = abs(row["dur_delta_s"]) <= 0.05
        rows.append(row)
    return rows


def analyse(args, report):
    J = os.path.join(args.out, "json")
    prov_path = os.path.join(args.out, "provenance.json")
    prov = jload(prov_path) if os.path.exists(prov_path) else {}
    raw = os.path.join(args.out, "raw")
    files = {}
    for p in walk_files(raw, (".ent", ".anims", ".rig")):
        rel = os.path.relpath(p, raw).replace("/", "\\")
        files[rel] = {"sha256": sha256(p), "bytes": os.path.getsize(p),
                      "archives": prov.get(rel, [])}
    js = {f[:-5]: os.path.join(J, f) for f in os.listdir(J) if f.endswith(".json")}

    rigs = {k: load_rig(v) for k, v in js.items() if k.endswith(".rig")}
    sets = {k[:-6]: load_animset(v) for k, v in js.items() if k.endswith(".anims")}
    ents = {k[:-4]: ent_vehicle_entries(v) for k, v in js.items() if k.endswith(".ent")}

    rig_cmp = {}
    pairs = [("woman_base.rig", "player_woman_skeleton.rig"), ("man_base.rig", "player_man_skeleton.rig")]
    for a, b in pairs:
        if a in rigs and b in rigs:
            rig_cmp[f"{a} -> {b}"] = compare_rigs(rigs[a], rigs[b])

    # MOTO-2: bike sets on each player entity
    moto2 = {}
    for en, e in ents.items():
        per = {}
        for c in e["components"]:
            for x in c["vehicle_entries"]:
                if "sportbike" in x["set"].lower():
                    per.setdefault(c["where"], []).append(
                        {"component": c["name"], "ctype": c["type"], **x})
        if per:
            moto2[en] = per
    # set content: which clip names exist only in the FPP set
    moto2_clips = {}
    for g in ("pwa", "pma"):
        for n in ("", "2", "3"):
            f, t = sets.get(f"{g}_sportbike{n}"), sets.get(f"tpp_{g}_sportbike{n}")
            if f and t:
                fn, tn = set(f["clips"]), set(t["clips"])
                moto2_clips[f"{g}_sportbike{n}"] = {
                    "fpp_clips": len(fn), "tpp_clips": len(tn), "shared": len(fn & tn),
                    "only_fpp": sorted(fn - tn), "only_tpp_sample": sorted(tn - fn)[:60],
                    "fpp_rig": f["rig"], "tpp_rig": t["rig"],
                    "shared_durations_differ": sorted(
                        c for c in fn & tn if abs(f["clips"][c]["duration_s"] - t["clips"][c]["duration_s"]) > 0.05),
                }

    # MOTO-7: wa_sportbike enter/exit vs pwa_sportbike
    moto7 = {}
    for n in ("", "2", "3"):
        for npc, pl in ((f"wa_sportbike{n}", f"pwa_sportbike{n}"), (f"ma_sportbike{n}", f"pma_sportbike{n}")):
            if npc in sets and pl in sets:
                names = sorted(c for c in set(sets[npc]["clips"]) | set(sets[pl]["clips"])
                               if re.search(r"(enter|exit|idle_to|to_idle|begin)", c))
                moto7[f"{npc} vs {pl}"] = {"npc_rig": sets[npc]["rig"], "player_rig": sets[pl]["rig"],
                                           "clips": clip_cmp(sets[npc], sets[pl], names, None)}
        tp = f"tpp_pwa_sportbike{n}"
        if tp in sets and f"pwa_sportbike{n}" in sets:
            names = sorted(c for c in sets[tp]["clips"] if re.search(r"(enter|exit|begin)", c))
            moto7[f"{tp} vs pwa_sportbike{n}"] = {"clips": clip_cmp(sets[tp], sets[f"pwa_sportbike{n}"], names, None)}

    # CAR-6: wa_sportN_driver vs pwa_sportN_driver + vehicle door clips
    car6 = {}
    for n, veh in (("1", "v_sport1_quadra_turbo"), ("2", "v_sport2_porsche_911turbo")):
        for npc, pl in ((f"wa_sport{n}_driver", f"pwa_sport{n}_driver"), (f"ma_sport{n}_driver", f"pma_sport{n}_driver")):
            if npc in sets and pl in sets:
                names = sorted(c for c in set(sets[npc]["clips"]) | set(sets[pl]["clips"])
                               if re.search(r"(enter|exit|steal|body_out)", c))
                car6[f"{npc} vs {pl}"] = {"npc_rig": sets[npc]["rig"], "player_rig": sets[pl]["rig"],
                                          "clips": clip_cmp(sets[npc], sets[pl], names, None)}
        if veh in sets:
            car6[veh] = {"rig": sets[veh]["rig"],
                         "clips": {k: {"dur": v["duration_s"], "frames": v["frames"],
                                       "events": [f'{e["type"]}:{e["name"]}@{e["frame"]}' for e in v["events"]]}
                                   for k, v in sets[veh]["clips"].items()}}

    # ------------------------------------------------ verdict inputs (computed)
    fpp_ents = [e for e in ents if re.match(r"player_(wa|ma)_fpp(_ep1)?$", e)]
    tpp_reg, fpp_reg = {}, {}
    for en in fpp_ents:
        for x in moto2.get(en, {}).get("components", []):
            nm = x["set"].split("\\")[-1]
            if nm.startswith("tpp_"):
                tpp_reg.setdefault(en, []).append((nm, x["priority"], x["vars"], x["component"]))
            elif re.match(r"p(w|m)a_sportbike\d?\.anims$", nm):
                fpp_reg.setdefault(en, []).append((nm, x["priority"], x["vars"], x["component"]))
    # vanilla precedence pattern on the player root component: priority vs specificity
    prio_pattern = {}
    root_ent = js.get("player_wa_fpp_ep1.ent")
    if root_ent:
        rc = root_chunk(jload(root_ent))
        for c in rc.get("components") or []:
            if val(c.get("name")) == "root" and "animations" in c:
                for g in c["animations"].get("gameplay") or []:
                    key = str(g.get("priority"))
                    d = prio_pattern.setdefault(key, {"entries": 0, "with_vars": 0, "sample": []})
                    d["entries"] += 1
                    d["with_vars"] += 1 if g.get("variableNames") else 0
                    if len(d["sample"]) < 3:
                        d["sample"].append([val(g.get("animSet")).split("\\")[-1], [val(v) for v in g.get("variableNames") or []]])
    m7 = {}
    for k, v in moto7.items():
        if k.startswith(("wa_", "ma_")):
            for r in v["clips"]:
                if r["clip"] in ("enter_l", "enter_r") and r.get("npc") and r.get("player"):
                    m7[f'{k}:{r["clip"]}'] = {"dur_delta_s": r["dur_delta_s"], "ok_0.05": r["dur_ok_0.05"],
                                              "rm_end_delta_m": r.get("rm_end_delta_m"), "rm_yaw_delta_deg": r.get("rm_yaw_delta_deg")}
    c6 = {}
    for k, v in car6.items():
        if " vs " in k:
            for r in v["clips"]:
                if r["clip"] in ("enter", "exit") and r.get("npc") and r.get("player"):
                    c6[f'{k}:{r["clip"]}'] = {"dur_delta_s": r["dur_delta_s"], "rm_end_delta_m": r.get("rm_end_delta_m"),
                                              "rm_yaw_delta_deg": r.get("rm_yaw_delta_deg"),
                                              "npc_events": len(r["npc"]["events"]), "player_events": len(r["player"]["events"]),
                                              "door_events_any": any("door" in e.lower() for e in r["npc"]["events"] + r["player"]["events"])}
    wr = rig_cmp.get("woman_base.rig -> player_woman_skeleton.rig", {})
    verdict_inputs = {
        "moto2": {
            "tpp_sets_on_fpp_entities": tpp_reg,
            "fpp_sets_on_fpp_entities": fpp_reg,
            "all_fpp_entities_have_tpp_sets": all(len(tpp_reg.get(e, [])) == 3 for e in fpp_ents),
            "tpp_vars_include_player_tpp_head": all("player_tpp_head" in t[2] for l in tpp_reg.values() for t in l),
            "tpp_sets_lack_enter_clips": all("enter_l" in v["only_fpp"] and "enter_r" in v["only_fpp"] for v in moto2_clips.values()),
            "root_priority_pattern_player_wa_fpp_ep1": prio_pattern,
        },
        "moto7_enter_compare": m7,
        "moto7_all_within_0.05s": bool(m7) and all(v["ok_0.05"] for v in m7.values()),
        "car6_enter_exit_compare": c6,
        "car6_door_events_in_character_clips": any(v["door_events_any"] for v in c6.values()),
        "rig_female": {k: wr.get(k) for k in ("bones_source", "bones_target", "common", "same_index",
                                              "parent_mismatch", "key_bones_missing", "top_translation_deltas")},
    }

    table = {
        "schema": "utpp-vehicle-anims-2.31/1",
        "verdict_inputs": verdict_inputs,
        "tool": "Source/tools/vehicle_anims/vehicle_anims_check.py",
        "wolvenkit_cli": args.wkit,
        "game": args.game,
        "extracted_to": args.out,
        "note": "Read-only extraction from the game archives; nothing written to the game folder. "
                "FACTS are data read from the files; priority direction and runtime selection are not provable offline.",
        "files": files,
        "entities": ents,
        "moto2_bike_registrations": moto2,
        "moto2_set_contents": moto2_clips,
        "moto7_enter_exit_compare": moto7,
        "car6_enter_exit_compare": car6,
        "rig_compare": rig_cmp,
        "animsets": {k: {"rig": v["rig"], "clip_count": v["clip_count"],
                         "clips": {c: {kk: vv for kk, vv in d.items() if kk in ("duration_s", "frames", "joints", "extra_joints")}
                                   for c, d in v["clips"].items()}} for k, v in sets.items()},
    }
    if args.table:
        os.makedirs(os.path.dirname(args.table), exist_ok=True)
        with open(args.table, "w", encoding="utf-8", newline="\n") as f:
            json.dump(table, f, indent=1, ensure_ascii=False)
    print(f"files={len(files)} entities={len(ents)} animsets={len(sets)} rigs={len(rigs)}")
    for k, v in rig_cmp.items():
        print(f"rig {k}: src={v['bones_source']} dst={v['bones_target']} common={v['common']} "
              f"same_index={v['same_index']} only_src={len(v['only_in_source'])} parent_mismatch={len(v['parent_mismatch'])} "
              f"key_missing={v['key_bones_missing']} mean_dT={v['mean_local_translation_delta_m']}")
    required = ["player_wa_fpp", "player_wa_fpp_ep1", "player_ma_fpp", "player_ma_fpp_ep1"]
    missing = [r for r in required if r not in ents]
    missing += [s for s in ("pwa_sportbike", "tpp_pwa_sportbike", "wa_sportbike", "pwa_sport1_driver",
                            "wa_sport1_driver", "pwa_sport2_driver", "wa_sport2_driver") if s not in sets]
    missing += [r for r in ("woman_base.rig", "player_woman_skeleton.rig") if r not in rigs]
    if missing:
        print("MISSING:", missing)
        return 1
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wkit", required=True)
    ap.add_argument("--game", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--table")
    ap.add_argument("--skip-extract", action="store_true")
    ap.add_argument("--extract-only", action="store_true")
    args = ap.parse_args()
    report = {}
    if not args.skip_extract:
        prov = do_extract(args, report)
        with open(os.path.join(args.out, "provenance.json"), "w", encoding="utf-8") as f:
            json.dump(prov, f, indent=1)
        rc, out = serialize(args.wkit, os.path.join(args.out, "raw"), os.path.join(args.out, "json"))
        report["serialize_rc"] = rc
        if rc != 0:
            report["serialize_tail"] = out[-2000:]
        print(json.dumps(report, indent=1))
    if args.extract_only:
        return 0
    return analyse(args, report)


if __name__ == "__main__":
    sys.exit(main())
