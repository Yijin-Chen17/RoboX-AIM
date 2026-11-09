@echo off
setlocal
cd /d "%~dp0"
if exist "%~dp0.venv\Scripts\python.exe" (
  set "RMUC_PYTHON=%~dp0.venv\Scripts\python.exe"
) else if exist "%~dp0..\..\.venv\Scripts\python.exe" (
  set "RMUC_PYTHON=%~dp0..\..\.venv\Scripts\python.exe"
) else if exist "%~dp0..\..\work\venv\Scripts\python.exe" (
  set "RMUC_PYTHON=%~dp0..\..\work\venv\Scripts\python.exe"
) else (
  set "RMUC_PYTHON=python"
)
"%RMUC_PYTHON%" -c "import mujoco, numpy" >nul 2>&1
if errorlevel 1 (
  echo Python dependencies are missing. Run these commands in this folder:
  echo python -m venv .venv
  echo .venv\Scripts\python.exe -m pip install -r requirements.txt
  exit /b 1
)
"%RMUC_PYTHON%" -X utf8 %*
exit /b %ERRORLEVEL%
