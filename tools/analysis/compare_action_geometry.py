"""Measure the geometric cost of replacing native weapon-action clavicles.

The pose trace stores local game transforms (s0) and the MM snapshot (mm_).
For each complete Shoot/Reload frame, this tool changes only the two native
shoulder *rotations* to MM rotations, then evaluates both FK chains. It does
not simulate skinning, IK, or animation timing. No input is modified.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

TARGETS = ("LeftHand", "RightHand", "WeaponRight")
SHOULDERS = ("LeftShoulder", "RightShoulder")
ACTIONS = {2: "Reload", 8: "Shoot"}


def finite(values):
    return all(math.isfinite(x) for x in values)


def unit(q):
    norm = math.sqrt(sum(x * x for x in q))
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError("invalid quaternion")
    return tuple(x / norm for x in q)


def multiply(a, b):
    x, y, z, w = a
    u, v, t, s = b
    return (w * u + x * s + y * t - z * v,
            w * v - x * t + y * s + z * u,
            w * t + x * v - y * u + z * s,
            w * s - x * u - y * v - z * t)


def rotate(q, v):
    # q * (v,0) * conjugate(q), using the equivalent cross-product form.
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = (2 * (y * vz - z * vy),
                  2 * (z * vx - x * vz),
                  2 * (x * vy - y * vx))
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def angle(a, b):
    return math.degrees(2 * math.acos(min(1.0, abs(sum(x * y for x, y in zip(unit(a), unit(b)))))))


def percentile(sorted_values, fraction):
    if not sorted_values:
        return None
    at = (len(sorted_values) - 1) * fraction
    lo = math.floor(at)
    hi = math.ceil(at)
    return sorted_values[lo] + (sorted_values[hi] - sorted_values[lo]) * (at - lo)


def stats(values):
    values = sorted(values)
    if not values:
        return {"count": 0}
    return {"count": len(values), "mean": sum(values) / len(values),
            "p50": percentile(values, .5), "p95": percentile(values, .95),
            "max": values[-1]}


def fk_position(index, local, parents, cache):
    if index in cache:
        return cache[index]
    p, q = local[index]
    parent = parents[index]
    if parent < 0 or parent not in local:
        result = (p, q)
    else:
        pp, pq = fk_position(parent, local, parents, cache)
        rp = rotate(pq, p)
        result = (tuple(pp[i] + rp[i] for i in range(3)), unit(multiply(pq, q)))
    cache[index] = result
    return result


def load_rig(path):
    root = json.loads(path.read_text(encoding="utf-8"))["Data"]["RootChunk"]
    names = [item["$value"] for item in root["boneNames"]]
    parents = root["boneParentIndexes"]
    if len(names) != len(parents) or len(names) != len(set(names)):
        raise ValueError("invalid or duplicate rig bones")
    return names, parents, {name: i for i, name in enumerate(names)}


def analyze_frame(rows, names, parents, indices, result):
    try:
        action = int(rows[0]["action"])
    except (ValueError, KeyError, TypeError):
        result["skipped_frames_by_reason"]["malformed_action"] += 1
        return
    if action not in ACTIONS:
        return
    result["frames_by_action"][ACTIONS[action]] += 1
    local = {}
    mm_shoulder = {}
    frame_map = {}
    try:
        for row in rows:
            if int(row["action"]) != action:
                raise ValueError("mixed actions in one frame")
            game_index = int(row["gameIndex"])
            if game_index < 0 or game_index >= len(names):
                continue
            bone = int(row["bone"])
            if bone in frame_map and frame_map[bone] != game_index:
                raise ValueError("inconsistent gameIndex mapping")
            frame_map[bone] = game_index
            p = tuple(float(row[f"s0_{i}"]) for i in range(3))
            q = tuple(float(row[f"s0_{i}"]) for i in range(4, 8))
            if not finite(p + q):
                continue
            local[game_index] = (p, unit(q))
            if game_index in (indices[SHOULDERS[0]], indices[SHOULDERS[1]]):
                mq = tuple(float(row[f"mm_{i}"]) for i in range(3, 7))
                mm_shoulder[game_index] = unit(mq)
        if any(indices[n] not in local or indices[n] not in mm_shoulder for n in SHOULDERS):
            raise ValueError("missing shoulder transform")
        if any(indices[n] not in local for n in TARGETS):
            raise ValueError("missing target transform")
        changed = local.copy()
        for name in SHOULDERS:
            i = indices[name]
            changed[i] = (local[i][0], mm_shoulder[i])
        native_cache, changed_cache = {}, {}
        action_name = ACTIONS[action]
        for name in SHOULDERS:
            i = indices[name]
            result["angles_deg"][action_name][name].append(angle(local[i][1], mm_shoulder[i]))
        for name in TARGETS:
            i = indices[name]
            a = fk_position(i, local, parents, native_cache)[0]
            b = fk_position(i, changed, parents, changed_cache)[0]
            result["displacements_m"][action_name][name].append(math.dist(a, b))
        result["valid_frames_by_action"][action_name] += 1
    except (ValueError, KeyError, TypeError, RecursionError, OverflowError):
        result["skipped_frames_by_reason"]["incomplete_or_malformed_transform"] += 1


def run(trace, rig):
    names, parents, indices = load_rig(rig)
    for name in SHOULDERS + TARGETS:
        if name not in indices:
            raise ValueError(f"rig missing {name}")
    result = {
        "trace": str(trace.resolve()), "rig": str(rig.resolve()),
        "method": "game s0 local pose FK; replace only Left/RightShoulder local rotations with mm_ snapshot; keep native local positions and all child transforms; report model-space displacement",
        "units": {"angles": "degrees", "displacement": "meters"},
        "frames_by_action": Counter(), "valid_frames_by_action": Counter(),
        "skipped_frames_by_reason": Counter(),
        "angles_deg": defaultdict(lambda: defaultdict(list)),
        "displacements_m": defaultdict(lambda: defaultdict(list)),
    }
    with trace.open("r", encoding="utf-8-sig", newline="") as handle:
        data = (line for line in handle if not line.startswith("#"))
        reader = csv.DictReader(data)
        needed = {"ticks", "desc", "thread", "rig", "seq", "action", "bone", "gameIndex"}
        needed.update(f"s0_{i}" for i in range(8))
        needed.update(f"mm_{i}" for i in range(7))
        if not needed.issubset(reader.fieldnames or []):
            raise ValueError("pose trace is missing required columns")
        key, frame = None, []
        for row in reader:
            if None in row or row.get("ticks") is None:
                result["skipped_frames_by_reason"]["malformed_csv_row"] += 1
                continue
            frame_key = tuple(row[n] for n in ("ticks", "desc", "thread", "rig", "seq"))
            if key is not None and frame_key != key:
                analyze_frame(frame, names, parents, indices, result)
                frame = []
            key = frame_key
            frame.append(row)
        if frame:
            analyze_frame(frame, names, parents, indices, result)
    output = {k: v for k, v in result.items() if k not in ("angles_deg", "displacements_m")}
    output["angles_deg"] = {a: {b: stats(v) for b, v in per.items()}
                            for a, per in result["angles_deg"].items()}
    output["displacements_m"] = {a: {b: stats(v) for b, v in per.items()}
                                  for a, per in result["displacements_m"].items()}
    output["limitations"] = [
        "The mm_ snapshot is not necessarily the final rendered pose or the intended TPP correction.",
        "The FK calculation uses traced local positions and quaternions; scale, skinning, attachment offsets and IK are omitted.",
        "A zero displacement can mean action handback already makes MM shoulders match native shoulders; it does not prove anatomical quality.",
    ]
    return output


def main():
    project = Path(__file__).resolve().parents[3]
    default_dir = project / "Research/logs/game/pose-trace-test-20261015-082006"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, default=next(default_dir.glob("utpp-pose-*.csv"), None))
    parser.add_argument("--rig", type=Path, default=project / "Extracted/game/json/vanilla/player_woman_skeleton.rig.json")
    parser.add_argument("--output", type=Path, default=default_dir / "action-geometry.json")
    args = parser.parse_args()
    if args.trace is None:
        parser.error("no pose trace found; specify --trace")
    output = run(args.trace, args.rig)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"frames_by_action": output["frames_by_action"],
                      "valid_frames_by_action": output["valid_frames_by_action"],
                      "displacements_m": output["displacements_m"]}, indent=2))


if __name__ == "__main__":
    main()
