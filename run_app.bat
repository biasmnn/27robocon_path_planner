@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
if exist "%~dp0.venv\Scripts\python.exe" (
  "%~dp0.venv\Scripts\python.exe" main.py
) else (
  where python >nul 2>nul
  if errorlevel 1 (py -3 main.py) else (python main.py)
)
if errorlevel 1 pause
endlocal
