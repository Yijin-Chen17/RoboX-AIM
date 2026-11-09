@echo off
call "%~dp0run_python.cmd" view_scene.py --rover --route road_red %*
if errorlevel 1 (
  pause
  exit /b 1
)
