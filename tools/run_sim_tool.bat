@echo off
setlocal
cd /d "%~dp0"

echo ==========================================
echo  MOVISION BLE SIMULATOR LAUNCHER
echo ==========================================
echo.
echo [1] Starting Local Server (Port: 8080)...
echo [2] Opening Simulator in Browser...
echo.
echo * Note: Keep this window open while using the simulator.
echo.

:: Open browser
start "" "http://localhost:8080/sim_tool.html"

:: Run Python server
python -m http.server 8080

if %ERRORLEVEL% neq 0 (
    echo.
    echo [ERROR] Python not found. 
    echo Please run this in ESP-IDF Command Prompt.
    pause
)

endlocal
