"""Hand-written oracle of the slide chooser model (source pack CHT_PoseSearchDatabases_Relaxed rows, analysis 2026-10-19).

python make_slide_oracle.py --out Research/inventory/chooser_slide_oracle.json

Independent of slide_spec.apply_chooser: the expectations are written here from the 1.2 chooser rules
  * MovementMode == Sliding (any stance / state / gait) -> Slide table: MovementDirection Forward -> PSD_Slide_FeetOut,
    any other direction -> PSD_Slide_KneesOut (nothing else);
  * MovementMode_LastFrame == Sliding with MovementMode OnGround adds one exit database to the regular rows of the movement
    table that is reached: Stand Idles -> ExitToStandIdle, Crouch Idle -> ExitToCrouchIdle, Crouch Moving -> ExitToCrouchWalk,
    Stand Walks -> ExitToWalk, Stand Runs -> ExitToRun, Stand Sprint -> ExitToSprint;
  * everything else is the regular result of the bundle without slide data ("base").
Vector = the chooser inputs the test sets (E_* values; bools false; TimeToLand 1.5). Without a slide section the base table
must give the same sets for the vectors with mode != 2 (no database starts with PSD_Slide_).
"""
from __future__ import annotations

import argparse
import json
import sys

EXIT_DB = {
    (0, 0, None): "PSD_Slide_ExitToStandIdle",
    (0, 1, 0): "PSD_Slide_ExitToWalk", (0, 1, 1): "PSD_Slide_ExitToRun", (0, 1, 2): "PSD_Slide_ExitToSprint",
    (1, 0, None): "PSD_Slide_ExitToCrouchIdle",
    (1, 1, None): "PSD_Slide_ExitToCrouchWalk",
}
SPEED = {0: 150.0, 1: 375.0, 2: 585.0}


def exit_db(stance, state, gait):
    if state == 0:
        return EXIT_DB[(stance, 0, None)]
    if stance == 0:
        return EXIT_DB[(0, 1, gait)]
    return EXIT_DB[(1, 1, None)]


def vectors():
    out = []
    for lod in (0, 1):
        for mode in (0, 1, 2):
            for last in (0, 1, 2):
                for stance in (0, 1):
                    for state in (0, 1):
                        for gait in (0, 1, 2):
                            if state == 0 and gait:
                                continue
                            for direction in (0, 1):
                                if mode != 2 and direction:
                                    continue
                                v = dict(lod=lod, mode=mode, last=last, stance=stance, state=state, gait=gait,
                                         direction=direction, speed=0.0 if state == 0 else SPEED[gait])
                                if mode == 2:
                                    v["slide_only"] = ["PSD_Slide_FeetOut" if direction == 0 else "PSD_Slide_KneesOut"]
                                elif mode == 0 and last == 2:
                                    v["extra"] = exit_db(stance, state, gait)
                                out.append(v)
    return out


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    vs = vectors()
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        json.dump({"format": "utpp-chooser-slide-oracle", "version": 1, "vectors": vs}, f, indent=0)
        f.write("\n")
    print(len(vs), "vectors,", sum(1 for v in vs if "extra" in v), "exit,", sum(1 for v in vs if "slide_only" in v), "slide")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
