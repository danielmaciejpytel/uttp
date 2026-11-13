#!/bin/bash
# MotionBricks stepping for a MOTORCYCLE exit clip (get-off): same stages as exit_steps_run.sh (prep -> pinpre -> fwd -> MotionBricks -> foot_pin -> assemble), but
# starts from a dump npz of game_skeleton_bridge.py dump-glb (bike_steps_gen.py dumps / measures) and takes the support foot side as an argument. No game, no install.
# usage: bike_steps_run.sh <dump.npz> <out dir (relative to project root)> <support Left|Right> <k0 first planted frame of the support foot> <c> <N> <targets> <seed> [tail frame]
#   g = c + 4N - 7 (the first frame where the generated span meets the clip again); the support foot is locked k0..c before the generation (context holds the foot), the feet planted at g
#   are held until the tail frame (last clip frame). Seeds = --jitter SEED (the model is deterministic). Outputs <out>/bike_<tag>.steps.npz (+ intermediate files).
# Kimodo env python for the data steps, Tools/ai-motion (uv, MotionBricks) for the generator.
cd "${UTPP_PROJECT_ROOT:-$(cd "$(dirname "$0")/../../.." && pwd)}" || exit 1   # the project root (the folder that contains Source)
DUMP=$1; OUT=$2; SIDE=$3; K0=$4; C=$5; N=$6; TG=$7; SEED=$8; TAIL=${9:-58}
SRC=Source/tools/ai_motion; PY=Tools/ai-motion/envs/kimodo/Scripts/python.exe; TP=$SRC/data/body_tpose_generator.npz
G=$((C + 4 * N - 7)); mkdir -p "$OUT"
TAG=c${C}_k${K0}_N${N}_t${TG}_s$SEED
# opt-in gentle correction of the generated stepping (exit_steps_gen.py shape_feet): BIKE_NARROW="K[,natural cm[,pin cm[,ease]]]" (same syntax as UTPP_EXIT_NARROW of exit_steps_run.sh); the tag gets the suffix _nw
NW=""
if [ -n "$BIKE_NARROW" ]; then
  IFS=, read -r NK NN NP NE <<< "$BIKE_NARROW"; NW="--narrow $NK --natural ${NN:-58} --pin-glide ${NP:-1.0} --shape-ease ${NE:--1}"; TAG=${TAG}_nw
fi
# opt-in glide pin of the planted runs over the whole extended clip (exit_steps_gen.py pin_by_height, assemble --pin-h-full): BIKE_PINH=<cm>; tag suffix _ph (run pick afterwards; refine does not parse it)
PH=""
if [ -n "$BIKE_PINH" ]; then PH="--pin-h-full $BIKE_PINH"; TAG=${TAG}_ph; fi
# scene files are shared by the seeds of one (clip, c, k0): stage 1 only once (lock dir); pinpre depends on c
SC=$OUT/sc_c${C}_k${K0}
if mkdir "$SC.lock" 2>/dev/null; then
  $PY -I $SRC/exit_steps_gen.py prep "$DUMP" "$SC.scene.npz" > "$SC.prep.log" 2>&1
  $PY -I $SRC/exit_steps_gen.py pinpre "$SC.scene.npz" $TP "$SC.pin.npz" $SIDE $K0 $C > "$SC.pinpre.log" 2>&1
  $PY -I $SRC/game_skeleton_bridge.py fwd "$SC.pin.npz" $TP "$SC.soma.npz" > "$SC.fwd.log" 2>&1
  touch "$SC.ok"
else
  while [ ! -f "$SC.ok" ]; do sleep 1; done
fi
( source Tools/ai-motion/env.sh; cd Tools/ai-motion/motionbricks
  uv run python ../../../$SRC/clip_inbetween.py "../../../$SC.soma.npz" $C $N "../../../$OUT/mb_$TAG" --targets $TG --jitter $SEED > "../../../$OUT/mb_$TAG.log" 2>&1 )
$PY -I $SRC/foot_pin.py "$OUT/mb_$TAG.gen.npz" $TP "$OUT/mb_$TAG.pin.npz" --ground "$SC.soma.npz" --labels > "$OUT/pin_$TAG.log" 2>&1
$PY -I $SRC/exit_steps_gen.py assemble "$SC.pin.npz" $TP "$OUT/mb_$TAG.pin.npz" $C $G "$OUT/bike_$TAG.steps.npz" --leg-ik --end-fix --lift 7 --floor --knee-smooth 2.0 \
  --head $((K0 - 3)):$K0 --tail $TAIL $NW $PH > "$OUT/assemble_$TAG.log" 2>&1
echo "DONE $OUT $TAG $(tail -1 "$OUT/assemble_$TAG.log" | cut -c1-160)"
