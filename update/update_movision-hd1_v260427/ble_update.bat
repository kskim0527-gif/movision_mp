@echo off
setlocal
chcp 65001 > nul

echo.
echo ================================================================
echo    movision BLE OTA Update Tool
echo ================================================================
echo.

:: 1. Python Check
where python >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Python was not found. Please install Python 3.
    pause
    exit /b 1
)

:: 2. Library Check
python -c "import bleak" >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [INFO] Installing bleak library...
    python -m pip install bleak -q
)

:: 3. Run Script
set "SD=%~dp0"
if exist "%SD%ble_ota_tool.py" (
    python "%SD%ble_ota_tool.py"
) else (
    echo [ERROR] ble_ota_tool.py not found in %SD%
)

if %ERRORLEVEL% NEQ 0 pause
