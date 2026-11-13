@echo off
rem Retarget the source pack export (Animations\sourcepack\als_export) onto player_woman_skeleton.
rem Usage: run_retarget_woman.cmd pilot   (13 clips of Research\inventory\sourcepack-retarget-pilot.txt, with previews)
rem        run_retarget_woman.cmd full    (every exported FBX, no previews) -> Animations\retarget\out_sourcepack_woman
rem Steps: 1) contacts: export_foot_contacts.py build -> Animations\retarget\meta_sourcepack (export meta + contact_l/r; the
rem        export dir is not modified; thresholds 60 / Stops 80 cm/s from export_foot_contacts.py validate)
rem        2) retarget_batch.py --keep-all-curves (same bonemap, CP armature, rig and solver as the current databases;
rem           every meta curve is kept)
rem        3) check_retarget_out.py --require-all-curves, full-pose pairs from --pair-set pilot|full of
rem           Research\inventory\sourcepack-names.json -> <out>\check.json / check.txt; full run also writes the RTZ
rem           list Research\inventory\sourcepack-rtz-clips.json (jump/land, traversal, from_traversal with Hips dev > 0.25 m)
rem        4) head_yaw_clip_list.py -> <out>\headyaw_keep.json: the only clips head_yaw_audit.py may process
rem           (locomotion only, no Pose_*/AO_*/*_Transition/single-frame/additive assets), e.g.
rem           python head_yaw_audit.py measure --dirs "<out>" --keep "<out>\headyaw_keep.json" --out <log>
rem Contacts of derived clips: accepted (export_foot_contacts.py validate -> sourcepack-contacts-review.json).
rem Posture (posture_offsets.py / crouch_posture.py / head_yaw_audit.py apply) is a separate later step on the output dir.
rem Logs: Research\logs\retarget-sourcepack-woman[-pilot].log.
setlocal
set "BLENDER=%ProgramFiles(x86)%\Steam\steamapps\common\Blender\blender.exe"
if defined UTPP_BLENDER set "BLENDER=%UTPP_BLENDER%"
for %%I in ("%~dp0..\..\..") do set "ROOT=%%~fI"
set "TOOLS=%ROOT%\Source\tools\retarget"
set "EXP=%ROOT%\Animations\sourcepack\als_export"
set "META=%ROOT%\Animations\retarget\meta_sourcepack"
set "PILOT=%ROOT%\Research\inventory\sourcepack-retarget-pilot.txt"
if not exist "%EXP%\fbx" (
  echo source pack export missing: %EXP%\fbx
  exit /b 3
)
if /i "%~1"=="pilot" (
  set "OUT=%ROOT%\Animations\retarget\pilot_sourcepack_woman"
  set "LOG=%ROOT%\Research\logs\retarget-sourcepack-woman-pilot.log"
) else if /i "%~1"=="full" (
  set "OUT=%ROOT%\Animations\retarget\out_sourcepack_woman"
  set "LOG=%ROOT%\Research\logs\retarget-sourcepack-woman.log"
) else (
  echo usage: %~nx0 pilot^|full
  exit /b 2
)
python "%TOOLS%\export_foot_contacts.py" build --export "%EXP%" --threshold 60 --threshold-stops 80 --out "%META%" > "%LOG%" 2>&1
if errorlevel 1 (
  echo contacts build failed, see %LOG%
  exit /b 4
)
if /i "%~1"=="pilot" (
  "%BLENDER%" --background --factory-startup --python "%TOOLS%\retarget_batch.py" -- --root "%ROOT%" --fbx-dir "%EXP%\fbx" --meta-dir "%META%" --out-dir "%OUT%" --list "%PILOT%" --keep-all-curves >> "%LOG%" 2>&1
) else (
  "%BLENDER%" --background --factory-startup --python "%TOOLS%\retarget_batch.py" -- --root "%ROOT%" --fbx-dir "%EXP%\fbx" --meta-dir "%META%" --out-dir "%OUT%" --no-preview --keep-all-curves >> "%LOG%" 2>&1
)
set RC=%ERRORLEVEL%
findstr /C:"UTPP_RETARGET_DONE" "%LOG%"
if /i "%~1"=="pilot" (
  python "%TOOLS%\check_retarget_out.py" --out-dir "%OUT%" --meta-dir "%META%" --list "%PILOT%" --pair-set pilot --require-all-curves --json "%OUT%\check.json" > "%OUT%\check.txt" 2>&1
) else (
  python "%TOOLS%\check_retarget_out.py" --out-dir "%OUT%" --meta-dir "%META%" --pair-set full --require-all-curves --json "%OUT%\check.json" --rtz-out "%ROOT%\Research\inventory\sourcepack-rtz-clips.json" > "%OUT%\check.txt" 2>&1
)
if errorlevel 1 echo check: curve gaps, see %OUT%\check.txt
python "%TOOLS%\head_yaw_clip_list.py" --clips-dir "%OUT%" --out "%OUT%\headyaw_keep.json" >> "%LOG%" 2>&1
echo check: %OUT%\check.txt
exit /b %RC%
