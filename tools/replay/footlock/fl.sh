#!/bin/bash
# usage: fl.sh <label> [replay options...]  -> Build/work/footlock/out/<label>_{woman,man}.txt (FOOTLOCK tables); both DBs, all feeds
R="${UTPP_PROJECT_ROOT:-$(cd "$(dirname "$0")/../../../.." && pwd)}"
EXE="$R/Build/UTPP/utpp_motion_replay.exe"
LABEL="$1"; shift
mkdir -p "$R/Build/work/footlock/out"
FEEDS=("$R"/Build/test-data/replay/2026*.feed "$R"/Build/test-data/replay/synth_footlock/*.feed)
declare -A DB=( [woman]="$R/Data/utpp_motion_woman_knee_twistfix_slide_aimotion4.utppmm" [man]="$R/Data/utpp_motion_man_knee_slide.utppmm" )
for g in woman man; do
  "$EXE" --db "${DB[$g]}" --footlock --brief --label "$LABEL-$g" "$@" "${FEEDS[@]}" > "$R/Build/work/footlock/out/${LABEL}_$g.txt" 2>&1
  echo "== $LABEL $g exit $?"
done
