@echo off
setlocal
cd /d "%~dp0"

where cmake >nul 2>nul
if errorlevel 1 (
  echo CMake was not found in PATH.
  exit /b 1
)

cmake -S . -B build
if errorlevel 1 exit /b 1

cmake --build build --config Release
if errorlevel 1 exit /b 1

ctest --test-dir build -C Release --output-on-failure
if errorlevel 1 exit /b 1

echo.
echo Build complete.
echo Run run_phase5.bat for the current drip-maturation and radial-pooling lab.
echo Older phase launchers remain available for regression testing.
