@echo off
setlocal
cd /d "%~dp0"
if exist "build\Release\locomotion_phase3.exe" (
  start "" "build\Release\locomotion_phase3.exe"
  exit /b 0
)
if exist "build\locomotion_phase3.exe" (
  start "" "build\locomotion_phase3.exe"
  exit /b 0
)
echo LO-004 executable not found. Run build_windows.bat first.
pause
exit /b 1
