@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
if not exist "%~dp0.venv\Scripts\python.exe" (
  where python >nul 2>nul
  if errorlevel 1 (py -3 -m venv .venv) else (python -m venv .venv)
  if errorlevel 1 goto failed
)
"%~dp0.venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 goto failed
echo 依赖安装完成。双击 run_app.bat 启动。
pause
exit /b 0
:failed
echo 安装失败，请确认已安装 Python 3.10 或更高版本并可访问包下载源。
pause
exit /b 1
