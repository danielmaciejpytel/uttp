#!/bin/bash
# Real stepping for a car EXIT clip: MotionBricks in-betweening of the exit tail -> steps npz for build_exit.ps1 -Steps (exit_steps_gen.py header explains the stages).
# usage: exit_steps_run.sh <exit .anims.glb of the head-duck run (roundtrip)> <out dir relative to the project root> [c N targets seed]   default 32 8 4 10 (chosen 2026-11-05, g = c + 4N - 7 = 57)
# Opt-in (default off = the original behaviour): UTPP_EXIT_PIN=<k0> locks the LEFT support foot from frame k0 to c before the generation (exit_steps_gen.py pinpre; the context of MotionBricks then holds the foot) and the steps npz
# also covers frames k0-3..c-1 (scene) -> no glide of the support foot before c; UTPP_EXIT_TAIL=<last frame> (60) additionally holds the feet that are planted at g until the last frame (npz span ..T, ramp_end 1). Both together: c, g of the npz = k0-3, T.
# Opt-in (2026-11-06): UTPP_EXIT_NARROW=K[,natural,pin,ease] (narrow the stride by K x the excess over a natural ankle distance, pin gliding planted runs), UTPP_EXIT_TAILSIDES=Left|Right|Left,Right (with UTPP_EXIT_TAIL: which feet are held).
# The model is deterministic: sample diversity = --jitter SEED (seeded noise on the context history c-3..c-1 only, frame c stays exact). Needs the Kimodo env python (scipy) for the data steps and
# Tools/ai-motion (uv, MotionBricks) for the generator; no game, no GPU other than MotionBricks.
cd "${UTPP_PROJECT_ROOT:-$(cd "$(dirname "$0")/../../.." && pwd)}" || exit 1   # the project root (the folder that contains Source)
case "$1" in *_pma_*) export UTPP_RIG=man;; esac   # male V clips (pma_* sets): game_skeleton_bridge uses player_man_skeleton (reference pose / bone lengths); female (default) unchanged; override with UTPP_RIG=woman|man
GLB=$1; OUT=$2; C=${3:-32}; N=${4:-8}; TG=${5:-4}; SEED=${6:-10}
SRC=Source/tools/ai_motion; PY=Tools/ai-motion/envs/kimodo/Scripts/python.exe; TP=$SRC/data/body_tpose_generator.npz
G=$((C + 4 * N - 7)); mkdir -p "$OUT"
$PY -I $SRC/game_skeleton_bridge.py dump-glb "$GLB" exit "$OUT/cur.dump.npz" | tail -1
$PY -I $SRC/exit_steps_gen.py prep "$OUT/cur.dump.npz" "$OUT/scene.dump.npz"
if [ -n "$UTPP_EXIT_PLAN" ]; then   # opt-in: footstep planner (exit_steps_gen.py plan) instead of MotionBricks; UTPP_EXIT_PLAN = its options, e.g. "--lag -14 --rswing 9 --rover 2 --ease 0.45"
  $PY -I $SRC/exit_steps_gen.py plan "$OUT/scene.dump.npz" $TP "$OUT/exit_steps2_plan.steps.npz" $UTPP_EXIT_PLAN | cut -c1-300
  $PY -I $SRC/exit_steps_gen.py rank "$OUT/cur.dump.npz" "$OUT/exit_steps2_plan.steps.npz" | cut -c1-300
  echo "next: powershell -File Source/tools/anim/vehicle-entry/build_exit.ps1 -Out <new run dir> -Steps <abs path of $OUT/exit_steps2_plan.steps.npz> -StepsBase <head-duck run dir>"
  exit 0
fi
HT=""
if [ -n "$UTPP_EXIT_PIN" ]; then
  $PY -I $SRC/exit_steps_gen.py pinpre "$OUT/scene.dump.npz" $TP "$OUT/scene_pin.dump.npz" Left $UTPP_EXIT_PIN $C | cut -c1-200
  cp "$OUT/scene.dump.npz" "$OUT/scene_unpinned.dump.npz"; cp "$OUT/scene_pin.dump.npz" "$OUT/scene.dump.npz"
  HT="--head $((UTPP_EXIT_PIN - 3)):$UTPP_EXIT_PIN"
fi
[ -n "$UTPP_EXIT_TAIL" ] && HT="$HT --tail $UTPP_EXIT_TAIL"
[ -n "$UTPP_EXIT_TAILSIDES" ] && HT="$HT --tail-sides $UTPP_EXIT_TAILSIDES"   # opt-in: only these feet are held until the last frame (Left, Right or Left,Right)
if [ -n "$UTPP_EXIT_NARROW" ]; then   # opt-in (2026-11-06): "K[,natural cm[,pin cm[,ease]]]" = gentle stride narrowing + glide pin of the planted runs (exit_steps_gen.py shape_feet), default 58 cm, 1.0, -1
  IFS=, read -r NK NN NP NE <<< "$UTPP_EXIT_NARROW"; HT="$HT --narrow $NK --natural ${NN:-58} --pin-glide ${NP:-1.0} --shape-ease ${NE:--1}"
fi
$PY -I $SRC/game_skeleton_bridge.py fwd "$OUT/scene.dump.npz" $TP "$OUT/scene.soma.npz" | tail -1
( source Tools/ai-motion/env.sh; cd Tools/ai-motion/motionbricks
  uv run python ../../../$SRC/clip_inbetween.py "../../../$OUT/scene.soma.npz" $C $N "../../../$OUT/mb_${C}_${N}_${TG}_s$SEED" --targets $TG --jitter $SEED 2>&1 | grep MB_INBETWEEN | cut -c1-90 )
$PY -I $SRC/foot_pin.py "$OUT/mb_${C}_${N}_${TG}_s$SEED.gen.npz" $TP "$OUT/mb_${C}_${N}_${TG}_s$SEED.pin.npz" --ground "$OUT/scene.soma.npz" --labels | cut -c1-200
$PY -I $SRC/exit_steps_gen.py assemble "$OUT/scene.dump.npz" $TP "$OUT/mb_${C}_${N}_${TG}_s$SEED.pin.npz" $C $G "$OUT/exit_steps_c${C}_N${N}_s$SEED.steps.npz" --leg-ik --end-fix --lift 7 --floor --knee-smooth 2.0 $HT
$PY -I $SRC/exit_steps_gen.py rank "$OUT/cur.dump.npz" "$OUT/exit_steps_c${C}_N${N}_s$SEED.steps.npz" | cut -c1-300
echo "next: powershell -File Source/tools/anim/vehicle-entry/build_exit.ps1 -Out <new run dir> -Steps <abs path of the steps npz> -StepsBase <head-duck run dir>"
