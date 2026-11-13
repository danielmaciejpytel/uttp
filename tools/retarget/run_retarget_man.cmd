@echo off
rem Retarget the source pack export (Animations\sourcepack\als_export) onto player_man_skeleton (male V).
rem Usage: run_retarget_man.cmd pilot   (13 clips of Research\inventory\sourcepack-retarget-pilot.txt + Pose_Masculine_Stand_Idle,
rem                                              with previews) -> Animations\retarget\out_sourcepack_man_pilot
rem        run_retarget_man.cmd full    (every exported FBX, no previews) -> Animations\retarget\out_sourcepack_man
rem        run_retarget_man.cmd rtz     (RTZ variant, Source\tools\retarget\RTZ.txt: the 110 clips of
rem                                              Animations\retarget\out_sourcepack_woman_rtz\clips.txt = the RTZ list of
rem                                              Research\inventory\sourcepack-rtz-clips.json, --rtz, no previews)
rem                                              -> Animations\retarget\out_sourcepack_man_rtz; run after "full" (check_rtz.py
rem                                              compares against out_sourcepack_man)
rem Same pipeline as run_retarget_woman.cmd (solver retarget_batch.py --keep-all-curves, contacts meta, checks), with
rem the male target: armature Animations\retarget\cp_player_man_skeleton.blend (build_cp_armature.py), rig
rem Extracted\game\json\vanilla-man\player_man_skeleton.rig.json (WolvenKit CLI, read-only from basegame_1_engine.archive), bone map
rem bonemap_source_to_game_man.json (derived from the woman map; differences listed in its "differences_vs_woman").
rem Contacts: the meta dir Animations\retarget\meta_sourcepack is rig-independent and shared with the woman run; it is built here
rem only when missing (same command and thresholds as the woman wrapper), never rebuilt over an existing one.
rem No posture step: posture_offsets.json is
rem measured against the game female reference idle and is not applied to the male V; Animations\retarget\posture_offsets_man.json
rem (measured against the game male reference idle) is a measurement only and is not applied either (it fixes the idle shoulders but
rem over-corrects locomotion; the woman output out_sourcepack_woman is also unposed). The Masculine style is a runtime overlay.
rem Logs: Research\logs\retarget-sourcepack-man[-pilot].log.
setlocal
set "BLENDER=%ProgramFiles(x86)%\Steam\steamapps\common\Blender\blender.exe"
if defined UTPP_BLENDER set "BLENDER=%UTPP_BLENDER%"
for %%I in ("%~dp0..\..\..") do set "ROOT=%%~fI"
set "TOOLS=%ROOT%\Source\tools\retarget"
set "EXP=%ROOT%\Animations\sourcepack\als_export"
set "META=%ROOT%\Animations\retarget\meta_sourcepack"
set "PILOT=%ROOT%\Research\inventory\sourcepack-retarget-pilot.txt"
set "CPBLEND=%ROOT%\Animations\retarget\cp_player_man_skeleton.blend"
set "RIG=%ROOT%\Extracted\game\json\vanilla-man\player_man_skeleton.rig.json"
set "BONEMAP=%TOOLS%\bonemap_source_to_game_man.json"
set "EXTRA=Pose_Masculine_Stand_Idle"
set "RTZLIST=%ROOT%\Animations\retarget\out_sourcepack_woman_rtz\clips.txt"
set "RTZGROUPS=%ROOT%\Research\inventory\sourcepack-rtz-clips.json"
if not exist "%EXP%\fbx" (
  echo source pack export missing: %EXP%\fbx
  exit /b 3
)
if not exist "%CPBLEND%" (
  echo male armature missing: %CPBLEND% ^(build_cp_armature.py^)
  exit /b 3
)
if not exist "%RIG%" (
  echo male rig json missing: %RIG%
  exit /b 3
)
if /i "%~1"=="pilot" (
  set "OUT=%ROOT%\Animations\retarget\out_sourcepack_man_pilot"
  set "LOG=%ROOT%\Research\logs\retarget-sourcepack-man-pilot.log"
) else if /i "%~1"=="full" (
  set "OUT=%ROOT%\Animations\retarget\out_sourcepack_man"
  set "LOG=%ROOT%\Research\logs\retarget-sourcepack-man.log"
) else if /i "%~1"=="rtz" (
  set "OUT=%ROOT%\Animations\retarget\out_sourcepack_man_rtz"
  set "LOG=%ROOT%\Research\logs\retarget-sourcepack-man-rtz.log"
) else (
  echo usage: %~nx0 pilot^|full^|rtz
  exit /b 2
)
echo UTPP retarget source pack -^> player_man_skeleton %~1 > "%LOG%"
if exist "%META%\M_Neutral_Run_Loop_F.json" (
  echo contacts meta present: %META% ^(not rebuilt^) >> "%LOG%"
) else (
  python "%TOOLS%\export_foot_contacts.py" build --export "%EXP%" --threshold 60 --threshold-stops 80 --out "%META%" >> "%LOG%" 2>&1
  if errorlevel 1 (
    echo contacts build failed, see %LOG%
    exit /b 4
  )
)
if /i "%~1"=="pilot" (
  "%BLENDER%" --background --factory-startup --python "%TOOLS%\retarget_batch.py" -- --root "%ROOT%" --fbx-dir "%EXP%\fbx" --meta-dir "%META%" --out-dir "%OUT%" --list "%PILOT%" --clips "%EXTRA%" --cp-blend "%CPBLEND%" --rig-json "%RIG%" --bonemap "%BONEMAP%" --keep-all-curves >> "%LOG%" 2>&1
) else if /i "%~1"=="rtz" (
  "%BLENDER%" --background --factory-startup --python "%TOOLS%\retarget_batch.py" -- --root "%ROOT%" --fbx-dir "%EXP%\fbx" --meta-dir "%META%" --out-dir "%OUT%" --list "%RTZLIST%" --no-preview --rtz --cp-blend "%CPBLEND%" --rig-json "%RIG%" --bonemap "%BONEMAP%" --keep-all-curves >> "%LOG%" 2>&1
) else (
  "%BLENDER%" --background --factory-startup --python "%TOOLS%\retarget_batch.py" -- --root "%ROOT%" --fbx-dir "%EXP%\fbx" --meta-dir "%META%" --out-dir "%OUT%" --no-preview --cp-blend "%CPBLEND%" --rig-json "%RIG%" --bonemap "%BONEMAP%" --keep-all-curves >> "%LOG%" 2>&1
)
set RC=%ERRORLEVEL%
findstr /C:"UTPP_RETARGET_DONE" "%LOG%"
if /i "%~1"=="pilot" (
  python "%TOOLS%\check_retarget_out.py" --out-dir "%OUT%" --meta-dir "%META%" --rig "%RIG%" --list "%PILOT%" --clips "%EXTRA%" --pair-set pilot --pair "%EXTRA%:Pose_Neutral_Stand_Idle" --require-all-curves --json "%OUT%\check.json" > "%OUT%\check.txt" 2>&1
) else if /i "%~1"=="rtz" (
  python "%TOOLS%\check_retarget_out.py" --out-dir "%OUT%" --meta-dir "%META%" --rig "%RIG%" --list "%RTZLIST%" --require-all-curves --json "%OUT%\check.json" > "%OUT%\check.txt" 2>&1
) else (
  python "%TOOLS%\check_retarget_out.py" --out-dir "%OUT%" --meta-dir "%META%" --rig "%RIG%" --pair-set full --require-all-curves --json "%OUT%\check.json" > "%OUT%\check.txt" 2>&1
)
if errorlevel 1 echo check: curve gaps, see %OUT%\check.txt
if /i "%~1"=="rtz" (
  "%BLENDER%" --background --factory-startup --python "%TOOLS%\check_rtz.py" -- --rtz-dir "%OUT%" --ref-dir "%ROOT%\Animations\retarget\out_sourcepack_man" --fbx-dir "%EXP%\fbx" --meta-dir "%META%" --list "%RTZLIST%" --groups "%RTZGROUPS%" --rig-json "%RIG%" --json "%OUT%\check_rtz.json" --txt "%OUT%\check_rtz.txt" >> "%LOG%" 2>&1
  if errorlevel 1 echo check_rtz: failures, see %OUT%\check_rtz.txt
  echo check: %OUT%\check.txt
  exit /b %RC%
)
python "%TOOLS%\head_yaw_clip_list.py" --clips-dir "%OUT%" --out "%OUT%\headyaw_keep.json" >> "%LOG%" 2>&1
echo check: %OUT%\check.txt
exit /b %RC%
