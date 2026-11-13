#!/bin/bash
# VX_Exit_Idle_N10_MB (car exit end -> MotionBricks gap N=10 -> source pack M_Neutral_Stand_Idle_Loop) for a NEW exit clip, same recipe as the 2026-11-05 14:01 / 14:05 pilot (checked: the old exit as input reproduces the installed
# clip to 4e-5 outside Trajectory / Hips) + vx_anchor.py (exit ankles exact in the first frames).
# usage: vx_run.sh <exit .anims.glb (round-tripped)> <out prefix relative to the project root> [scene opts, e.g. --inplace]   -> <prefix>.VXa.utppclip.json
# then: mmbake with --extra-assets (see Build/work/aimotion-db/extra-assets-aimotion3.json, bake_base3.sh: the slide spec / v2 config copies there carry the paths after the Animations / Extracted reorg)
# Opt-in environment (default = the pilot behaviour above, byte for byte):
#   VX_IDLE=<idle .utppclip.json of the matching gender>  compose the scene from the new exit + that idle (compose_transition.py --rig cp --align facing, idle frames 0..39, N=10) instead of reusing the pilot scene's idle half
#       (the pilot scene is the female sport2 one: another class ends elsewhere, a male V needs the male retargeted idle: Animations/retarget/out_sourcepack_set3_man/M_Neutral_Stand_Idle_Loop.utppclip.json)
#   VX_ENDA=<last exit frame>  (default 60 = 61-frame exits; utility exits have 81 frames -> 80): scene = exit | 32 gap frames | 40 idle frames, clip = frames ENDA-10 .. ENDA+73, blend span ENDA .. ENDA+33
cd "${UTPP_PROJECT_ROOT:-$(cd "$(dirname "$0")/../../.." && pwd)}" || exit 1   # the project root (the folder that contains Source)
case "$1" in *_pma_*) export UTPP_RIG=man;; esac   # male V clips (pma_* sets): game_skeleton_bridge uses player_man_skeleton (reference pose / bone lengths); female (default) unchanged; override with UTPP_RIG=woman|man
GLB=$1; O=$2; VX_SCENE_OPTS=${3:-}   # optional: --inplace (idle on the exit end feet, no walk in the gap)
SRC=Source/tools/ai_motion; PY=Tools/ai-motion/envs/kimodo/Scripts/python.exe; TP=$SRC/data/body_tpose_generator.npz
EA=${VX_ENDA:-60}; GG=$((EA + 33)); T0=$((EA - 10)); T1=$((EA + 73))
OLD=Research/logs/other/ai-motion/vehexit-20261105/ex_idle_N10.dump.npz   # pilot scene: its idle half (frames 93..132) is reused
$PY -I $SRC/game_skeleton_bridge.py dump-glb "$GLB" exit $O.A.dump.npz | tail -1
if [ -n "$VX_IDLE" ]; then
  $PY -I $SRC/game_skeleton_bridge.py dump-clip "$VX_IDLE" $O.idle.dump.npz | tail -1
  $PY -I $SRC/compose_transition.py $O.A.dump.npz $EA $O.idle.dump.npz 0 40 10 $O.compose.dump.npz --rig cp --align facing | tail -1
  OLD=$O.compose.dump.npz
fi
$PY -I $SRC/vx_transition.py $O.A.dump.npz $OLD $O.scene.dump.npz --endA $EA $VX_SCENE_OPTS
$PY -I $SRC/game_skeleton_bridge.py fwd $O.scene.dump.npz $TP $O.scene.soma.npz | tail -1
( source Tools/ai-motion/env.sh; cd Tools/ai-motion/motionbricks; uv run python ../../../$SRC/clip_inbetween.py ../../../$O.scene.soma.npz $EA 10 ../../../$O.gen 2>&1 | grep MB_INBETWEEN | cut -c1-80 )
$PY -I $SRC/foot_pin.py $O.gen.gen.npz $TP $O.pin.npz --ground $O.scene.soma.npz | cut -c1-300
$PY -I $SRC/splice_span.py $O.scene.soma.npz $O.pin.npz $O.full.soma.npz | tail -1
$PY -I $SRC/trim_scene.py $O.full.soma.npz $O.trim.soma.npz $T0 $T1
$PY -I $SRC/game_skeleton_bridge.py inv $O.trim.soma.npz $O.scene.dump.npz $TP $O.VX.utppclip.json --frames $T0:$T1 --leg-ik --blend-span $EA:$GG --name VX_Exit_Idle_N10_MB | tail -1
$PY -I $SRC/vx_anchor.py $O.VX.utppclip.json $O.A.dump.npz $O.scene.dump.npz $TP $O.VXa.utppclip.json --name VX_Exit_Idle_N10_MB --start $T0
