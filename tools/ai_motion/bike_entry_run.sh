#!/bin/bash
# MotionBricks stepping for a MOTORCYCLE ENTRY clip (get-on, enter_l / enter_r of utpp_tpp_p?a_sportbike[N]): in-betweens the approach span c..g of the shipped clip (both feet
# glide / skate over the floor there) with real steps. Same stages as bike_steps_run.sh / entry_steps_run.sh, no support-foot lock (the clip starts standing still):
# prep (floor from frames 0..g+targets) -> fwd -> MotionBricks -> foot_pin -> assemble. Input = a game_skeleton_bridge.py dump-glb npz of the shipped clip (root motion included).
# usage: bike_entry_run.sh <dump.npz> <out dir (relative to project root)> <c> <N> <targets> <seed> [extra args of exit_steps_gen.py assemble]
#   g = c + 4N - 7 (first frame where the generated span meets the clip again; frames c-3..c = context, g..g+targets-1 = target, both from the clip). Output <out>/bike_c<c>_N<N>_t<t>_s<seed>.steps.npz
# Kimodo env python for the data steps, Tools/ai-motion (uv, MotionBricks, GPU) for the generator. No game, no install.
cd "${UTPP_PROJECT_ROOT:-$(cd "$(dirname "$0")/../../.." && pwd)}" || exit 1   # the project root (the folder that contains Source)
DUMP=$1; OUT=$2; C=$3; N=$4; TG=$5; SEED=$6; shift 6
SRC=Source/tools/ai_motion; PY=Tools/ai-motion/envs/kimodo/Scripts/python.exe; TP=$SRC/data/body_tpose_generator.npz
G=$((C + 4 * N - 7)); mkdir -p "$OUT"
TAG=c${C}_N${N}_t${TG}_s$SEED
SC=$OUT/sc_c${C}_N${N}_t${TG}
if mkdir "$SC.lock" 2>/dev/null; then
  $PY -I $SRC/entry_steps_gen.py prep "$DUMP" "$SC.scene.npz" --f1 $((G + TG)) > "$SC.prep.log" 2>&1
  $PY -I $SRC/game_skeleton_bridge.py fwd "$SC.scene.npz" $TP "$SC.soma.npz" > "$SC.fwd.log" 2>&1
  touch "$SC.ok"
else
  while [ ! -f "$SC.ok" ]; do sleep 1; done
fi
( source Tools/ai-motion/env.sh; cd Tools/ai-motion/motionbricks
  uv run python ../../../$SRC/clip_inbetween.py "../../../$SC.soma.npz" $C $N "../../../$OUT/mb_$TAG" --targets $TG --jitter $SEED > "../../../$OUT/mb_$TAG.log" 2>&1 )
$PY -I $SRC/foot_pin.py "$OUT/mb_$TAG.gen.npz" $TP "$OUT/mb_$TAG.pin.npz" --ground "$SC.soma.npz" --labels > "$OUT/pin_$TAG.log" 2>&1
$PY -I $SRC/exit_steps_gen.py assemble "$SC.scene.npz" $TP "$OUT/mb_$TAG.pin.npz" $C $G "$OUT/bike_$TAG.steps.npz" --leg-ik --end-fix --knee-smooth 2.0 "$@" > "$OUT/assemble_$TAG.log" 2>&1
echo "DONE $OUT $TAG $(tail -1 "$OUT/assemble_$TAG.log" | cut -c1-160)"
