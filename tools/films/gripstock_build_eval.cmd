@echo off
rem Rifle grip + stock 2026-11-10: compiles gripstock_eval.cpp against a COPY of Source\plugin\UTPP\src.
rem   gripstock_build_eval.cmd before  -> tree_before = current src with the files of the pre-fix backup (state before the fix)
rem   gripstock_build_eval.cmd after   -> tree_after  = current src (with the fix), compiled with /DGRIPSTOCK_NEW
rem -> Build\work\rifle-grip-stock-20261110\bin\gripstock_eval_<mode>.exe (log bin\build_<mode>.log). Never writes into Build\UTPP or Source.
setlocal
for %%I in ("%~dp0..\..\..") do set "R=%%~fI"
set "W=%R%\Build\work\rifle-grip-stock-20261110"
set "BK=%R%\Backups\2026-11-10\rifle-grip-stock\Source\plugin\UTPP\src"
set "M=%~1"
if "%M%"=="" set "M=after"
if not exist "%W%\bin\obj_%M%" mkdir "%W%\bin\obj_%M%"
call "%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
robocopy "%R%\Source\plugin\UTPP\src" "%W%\tree_%M%\plugin\UTPP\src" /MIR /NFL /NDL /NJH /NJS /NP >nul
if not exist "%W%\tree_%M%\tools\films" mkdir "%W%\tree_%M%\tools\films"
copy /Y "%R%\Source\tools\films\gripstock_eval.cpp" "%W%\tree_%M%\tools\films\gripstock_eval.cpp" >nul
set "DEF="
if /I "%M%"=="before" (
  copy /Y "%BK%\mm\MotionPose.hpp" "%W%\tree_%M%\plugin\UTPP\src\mm\MotionPose.hpp" >nul
  copy /Y "%BK%\mm\ForearmTwist.hpp" "%W%\tree_%M%\plugin\UTPP\src\mm\ForearmTwist.hpp" >nul
  copy /Y "%BK%\mm\GripIK.hpp" "%W%\tree_%M%\plugin\UTPP\src\mm\GripIK.hpp" >nul
  if exist "%W%\tree_%M%\plugin\UTPP\src\mm\SupportGrip.hpp" del "%W%\tree_%M%\plugin\UTPP\src\mm\SupportGrip.hpp"
) else (
  set "DEF=/DGRIPSTOCK_NEW"
)
cl /nologo /std:c++20 /EHsc /O2 /W3 /DNOMINMAX %DEF% "%W%\tree_%M%\tools\films\gripstock_eval.cpp" /Fo"%W%\bin\obj_%M%\\" /Fe"%W%\bin\gripstock_eval_%M%.exe" > "%W%\bin\build_%M%.log" 2>&1
set RC=%ERRORLEVEL%
type "%W%\bin\build_%M%.log"
certutil -hashfile "%W%\tree_%M%\plugin\UTPP\src\mm\MotionPose.hpp" SHA256 | findstr /v ":"
echo BUILD_EXIT %RC%
exit /b %RC%
