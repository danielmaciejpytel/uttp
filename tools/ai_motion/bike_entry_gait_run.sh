#!/bin/bash
# Gait pass (bike_entry_gait.py) + splice (build_entry_steps.py) + glide table for the 12 shipped bike entry clips. usage: bike_entry_gait_run.sh <work dir> <out subdir> [gait args]
# <work dir> holds dump/<name>.npz (game_skeleton_bridge.py dump-glb of the shipped clips) and enter-orig/utpp_tpp_<name>.anims.glb. Kimodo env python. No game.
cd "${UTPP_PROJECT_ROOT:-$(cd "$(dirname "$0")/../../.." && pwd)}" || exit 1   # the project root (the folder that contains Source)
W=$1; O=$2; shift 2
PY=Tools/ai-motion/envs/kimodo/Scripts/python.exe; S=Source/tools/ai_motion
mkdir -p $W/$O
for dmp in $W/dump/*.npz; do
  n=$(basename $dmp .npz); c=$(echo $n | grep -o "enter_[lr]")
  $PY -I $S/bike_entry_gait.py $dmp $W/$O/$n.steps.npz "$@" > $W/$O/$n.gait.log 2>&1 || { echo "$n GAIT FAIL: $(tail -1 $W/$O/$n.gait.log)"; continue; }
  $PY -I Source/tools/anim/vehicle-entry/build_entry_steps.py --glb $W/enter-orig/utpp_tpp_$n.anims.glb --clip $c --name $c --steps $W/$O/$n.steps.npz --out $W/$O/$n.anims.glb --report $W/$O/$n.splice.json > $W/$O/$n.splice.log 2>&1 || { echo "$n SPLICE FAIL"; continue; }
  $PY -I $S/bike_steps_gen.py measure $n=$W/$O/$n.anims.glb:$c --json $W/$O/$n.glide.json | cut -c1-200
done
