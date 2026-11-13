"""Compare recorded start/stop replay CSVs; reconstructed support-toe drift only.

No game control. Contact curves mark eligibility, not actual sole collision or
FootLocker state. Recorded capsule/actor motion is retained in both runs.
"""
import argparse
import csv
import json
import math
from pathlib import Path


def read_feed(path):
    result = []
    for line in Path(path).read_text().splitlines():
        if line.startswith("#"):
            continue
        values = line.split()
        if len(values) >= 31:
            result.append(dict(t=float(values[0]), speed=math.hypot(float(values[2]), float(values[3])),
                               input=math.hypot(float(values[5]), float(values[6])),
                               capsule=list(map(float, values[19:22])), ground=int(values[27])))
    return result


def measure(path, feed, start, end):
    rows = list(csv.DictReader(Path(path).open()))
    selected = [(row, min(feed, key=lambda f: abs(f["t"] - float(row["t"]))))
                for row in rows if start <= float(row["t"]) <= end]
    result = {"samples": len(selected), "support": {}, "clips": []}
    last_clip = None
    for row, _ in selected:
        if row["clip"] != last_clip:
            result["clips"].append(dict(t=float(row["t"]), clip=int(row["clip"]), phase=float(row["clipT"])))
            last_clip = row["clip"]
    for side in ("Left", "Right"):
        contact = "contact" + side[0]
        intervals = []
        active = []
        for row, frame in selected:
            eligible = frame["ground"] == 1 and frame["speed"] < .05 and frame["input"] < .1 and float(row[contact]) >= .8
            if eligible:
                yaw = math.radians(float(row["actorYaw"]))
                x, y = (float(row["final_" + side + "ToeBase" + axis]) for axis in "xy")
                c, s = math.cos(yaw), math.sin(yaw)
                active.append((float(row["t"]), frame["capsule"][0] + c*x-s*y, frame["capsule"][1] + s*x+c*y))
            elif active:
                intervals.append(active)
                active = []
        if active:
            intervals.append(active)
        intervals = [v for v in intervals if v[-1][0] - v[0][0] >= .05]
        result["support"][side] = dict(
            intervals=len(intervals), eligibleSeconds=sum(v[-1][0]-v[0][0] for v in intervals),
            maxPlanarDriftCm=max((100*max(math.hypot(p[1]-v[0][1], p[2]-v[0][2]) for p in v) for v in intervals), default=None))
    return result


def main():
    parser = argparse.ArgumentParser()
    for argument in ("baseline", "candidate", "feed", "output"):
        parser.add_argument(argument)
    args = parser.parse_args()
    feed = read_feed(args.feed)
    # Exact raw-feed windows: first two post-stop intervals; turning and air are
    # controls, and explicitly do not count as planted support while airborne.
    windows = {"short1": (291.829662, 292.295799), "short2": (292.596519, 292.946137),
               "turn_control": (293.887, 294.8), "air_control": (297.83, 298.4)}
    output = {"windows": {}, "method": "Recorded actor Rot2 + model FK toe + capsule XY; contact>=.8, ground, speed<.05, input released, continuous>=.05s.",
              "limits": ["No sole/ground-plane collision evidence; toe-joint Z is not penetration.",
                         "Eligibility is not actual FootLocker support state; report coverage to avoid false improvement.",
                         "Replay retains recorded actor and C++ defaults; does not execute candidate Lua.",
                         "Feed omits native acceleration; do not fabricate it from target acceleration."],
              "screening": "Advisory support drift goal<=2cm, warning>5cm; A/B increase>2cm requires review. Not runtime pass/fail."}
    for name, (start, end) in windows.items():
        output["windows"][name] = {key: measure(path, feed, start, end) for key, path in
                                    (("baseline", args.baseline), ("candidate", args.candidate))}
        episode = output["windows"][name]
        episode["comparison"] = {}
        for side in ("Left", "Right"):
            before = episode["baseline"]["support"][side]
            after = episode["candidate"]["support"][side]
            a, b = before["maxPlanarDriftCm"], after["maxPlanarDriftCm"]
            episode["comparison"][side] = dict(
                driftDeltaCm=None if a is None or b is None else b-a,
                coverageDeltaSeconds=after["eligibleSeconds"]-before["eligibleSeconds"],
                reviewNeeded=(b is not None and (b > 5 or (a is not None and b-a > 2))),
                reducedCoverage=after["eligibleSeconds"] + .05 < before["eligibleSeconds"])
    Path(args.output).write_text(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
