@echo off
setlocal
cd /d "%~dp0"

if exist "build\Release\blood_phase6.exe" (
  start "" "build\Release\blood_phase6.exe"
  exit /b 0
)

if exist "build\blood_phase6.exe" (
  start "" "build\blood_phase6.exe"
  exit /b 0
)

echo blood_phase6.exe was not found.
echo Run build_windows.bat first.
exit /b 1
