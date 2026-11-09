@echo off
call "%~dp0run_python.cmd" validate_scene.py %*
if errorlevel 1 (
  pause
  exit /b 1
)
