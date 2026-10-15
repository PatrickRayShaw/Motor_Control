@echo off
REM 在这里修改你要运行的文件名（注意不要有行尾空格）
set SCRIPT_NAME=motor.py

if "%PY_MODE%"=="py" (
    py -3.12 "%~dp0%SCRIPT_NAME%"
) else if "%PY_MODE%"=="python" (
    python "%~dp0%SCRIPT_NAME%"
) else (
    "%LOCALAPPDATA%\Programs\Python\Python312\python.exe" "%~dp0%SCRIPT_NAME%"
)
