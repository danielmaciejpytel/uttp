#!/bin/bash
# Glide pin of a car ENTRY clip WITHOUT a generator (entry_steps_gen.py pinonly): builder GLB dump -> pinned steps npz -> after dump + glide numbers.
# usage: entry_pin_run.sh <row> <out dir relative to the project root> [pinonly options, e.g. --vthr 0.5 --margin 4]
cd "${UTPP_PROJECT_ROOT:-$(cd "$(dirname "$0")/../../.." && pwd)}" || exit 1   # the project root (the folder that contains Source)
ROW=$1; OUT=$2; shift 2
SRC=Source/tools/ai_motion; PY=Tools/ai-motion/envs/kimodo/Scripts/python.exe; TP=$SRC/data/body_tpose_generator.npz
mkdir -p "$OUT"
RUN=$($PY -I $SRC/entry_steps_gen.py rundir "$ROW" | tr -d '\r')
$PY -I $SRC/game_skeleton_bridge.py dump-glb "$RUN/glb/$ROW.anims.glb" enter "$OUT/cur.dump.npz" | tail -1
cp "Build/work/vehentry-mb-20261106/base/$ROW/plants.json" "$OUT/plants.json"
$PY -I $SRC/entry_steps_gen.py pinonly "$OUT/cur.dump.npz" "$OUT/plants.json" $TP "$OUT/steps_pin.steps.npz" "$@" | cut -c1-400
[ -f "$OUT/steps_pin.steps.npz" ] && $PY -I $SRC/exit_steps_gen.py clipdump "$OUT/steps_pin.steps.npz" "$OUT/cur.dump.npz" "$OUT/after_pin.dump.npz" && $PY -I $SRC/entry_steps_gen.py measure "$OUT/cur.dump.npz" "$OUT/after_pin.dump.npz"
