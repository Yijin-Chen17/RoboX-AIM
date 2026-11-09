@echo off
call "%~dp0run_python.cmd" view_scene.py --robox %*
if errorlevel 1 (
  pause
  exit /b 1
)
