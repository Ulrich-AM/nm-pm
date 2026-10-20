@echo off
setlocal
cd /d "%~dp0"
if exist "build\Release\locomotion_phase2.exe" (
  start "" "build\Release\locomotion_phase2.exe"
  exit /b 0
)
if exist "build\locomotion_phase2.exe" (
  start "" "build\locomotion_phase2.exe"
  exit /b 0
)
echo LO-003 executable not found. Run build_windows.bat first.
pause
exit /b 1
