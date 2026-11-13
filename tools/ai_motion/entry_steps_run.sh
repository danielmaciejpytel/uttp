#!/bin/bash
# Real stepping for a car ENTRY clip: MotionBricks in-betweening of the gliding span of the shipped BUILDER clip -> steps npz -> build_entry_steps.py (splice) -> builder GLB with the steps.
# usage: entry_steps_run.sh <row, e.g. utpp_car_pwa_standard3_driver> <out dir relative to the project root> <c> <N> <targets> <seed> [extra args of exit_steps_gen.py assemble]
# span c..g, g = c + 4N - 7 (frames c-3..c = context, g..g+targets-1 = target, both from the clip). Imports exit_steps_gen.py / foot_pin.py / clip_inbetween.py / game_skeleton_bridge.py (not edited).
# Needs the Kimodo env python for the data steps and Tools/ai-motion (uv, MotionBricks, GPU) for the generator. No game.
cd "${UTPP_PROJECT_ROOT:-$(cd "$(dirname "$0")/../../.." && pwd)}" || exit 1   # the project root (the folder that contains Source)
ROW=$1; OUT=$2; C=$3; N=$4; TG=$5; SEED=$6; shift 6
SRC=Source/tools/ai_motion; PY=Tools/ai-motion/envs/kimodo/Scripts/python.exe; TP=$SRC/data/body_tpose_generator.npz
G=$((C + 4 * N - 7)); mkdir -p "$OUT"
RUN=$($PY -I $SRC/entry_steps_gen.py rundir "$ROW" | tr -d '\r')
$PY -I $SRC/game_skeleton_bridge.py dump-glb "$RUN/glb/$ROW.anims.glb" enter "$OUT/cur.dump.npz" | tail -1
$PY -I $SRC/entry_steps_gen.py prep "$OUT/cur.dump.npz" "$OUT/scene.dump.npz" --f1 $((G + TG))
$PY -I $SRC/game_skeleton_bridge.py fwd "$OUT/scene.dump.npz" $TP "$OUT/scene.soma.npz" | tail -1
P="$OUT/mb_${C}_${N}_${TG}_s$SEED"
( source Tools/ai-motion/env.sh; cd Tools/ai-motion/motionbricks
  uv run python ../../../$SRC/clip_inbetween.py "../../../$OUT/scene.soma.npz" $C $N "../../../$P" --targets $TG --jitter $SEED 2>&1 | grep -E "MB_INBETWEEN|Error|error" | cut -c1-160 )
$PY -I $SRC/foot_pin.py "$P.gen.npz" $TP "$P.pin.npz" --ground "$OUT/scene.soma.npz" --labels | cut -c1-200
$PY -I $SRC/exit_steps_gen.py assemble "$OUT/scene.dump.npz" $TP "$P.pin.npz" $C $G "$OUT/steps_c${C}_N${N}_s$SEED.steps.npz" --leg-ik --end-fix --knee-smooth 2.0 "$@"
$PY -I $SRC/exit_steps_gen.py clipdump "$OUT/steps_c${C}_N${N}_s$SEED.steps.npz" "$OUT/cur.dump.npz" "$OUT/after_c${C}_N${N}_s$SEED.dump.npz"
