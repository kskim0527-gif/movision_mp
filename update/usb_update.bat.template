@echo off
chcp 65001 > nul
title movision USB OTA

setlocal EnableDelayedExpansion

set BUILD=%~dp0
set APP_BIN=%BUILD%update.bin
set STORAGE_BIN=%BUILD%storage.bin
set BOOTLOADER=%BUILD%bootloader.bin
set PARTITION=%BUILD%partition-table.bin
set OTADATA=%BUILD%ota_data_initial.bin

set ADDR_BOOTLOADER=0x0
set ADDR_PARTITION=0x8000
set ADDR_OTADATA=0xf000
set ADDR_APP=0x20000
set ADDR_STORAGE=0x3b2000

set FLASH_MODE=dio
set FLASH_FREQ=80m
set FLASH_SIZE=8MB
set BAUD=921600

set PYTHON=C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe
set ESPTOOL=C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\esptool.exe
if not exist "%PYTHON%" set PYTHON=python
if exist "%ESPTOOL%" (
    set ESPTOOL_CMD="%PYTHON%" "%ESPTOOL%"
) else (
    set ESPTOOL_CMD="%PYTHON%" -m esptool
)

cls
echo.
echo ================================================================
echo    movision USB OTA Update Tool
echo ================================================================
echo.

:: ── COM 포트 자동 감지 (ESP32-S3: VID=303A PID=1001) ─────────────
echo  [1] Searching for movision device... (VID=303A PID=1001)
echo.

set FOUND_PORT=
set FOUND_COUNT=0

:: PowerShell로 VID/PID 직접 하드코딩 검색 (변수 확장 문제 회피)
for /f "usebackq tokens=*" %%A in (`powershell -NoProfile -Command "$p = Get-PnpDevice -Class Ports -ErrorAction SilentlyContinue | Where-Object { $_.InstanceId -like '*VID_303A*PID_1001*' -and $_.Status -eq 'OK' }; if ($p) { $p | ForEach-Object { if ($_.FriendlyName -match 'COM(\d+)') { $matches[1] } } } else { '' }"`) do (
    if not "%%A"=="" (
        set FOUND_PORT=%%A
        set /a FOUND_COUNT+=1
        echo      Found: COM%%A
    )
)

if "%FOUND_COUNT%"=="0" (
    echo  [!] ESP32-S3 device not found automatically.
    echo.
    echo  -- All available COM ports --
    powershell -NoProfile -Command "Get-PnpDevice -Class Ports -ErrorAction SilentlyContinue | Where-Object {$_.Status -eq 'OK'} | ForEach-Object { '  ' + $_.FriendlyName }"
    echo.
    set /p FOUND_PORT="  Enter COM port number (e.g. 4 for COM4) : "
    if "!FOUND_PORT!"=="" goto :end
) else if "!FOUND_COUNT!" GTR "1" (
    echo.
    echo  Multiple devices found.
    set /p FOUND_PORT="  Enter COM port number to use : "
) else (
    echo      Auto-selected: COM%FOUND_PORT%
)

set COM=COM%FOUND_PORT%
echo.
echo  Port: %COM%   Baud: %BAUD%

:: ── 빌드 파일 확인 ────────────────────────────────────────────────
echo.
echo  [2] Build files:
set HAS_APP=0
set HAS_STORAGE=0

if exist "%APP_BIN%" (
    set HAS_APP=1
    for %%F in ("%APP_BIN%") do echo      [OK] Firmware : %%~nxF  (%%~zF bytes)
) else (
    echo      [--] Firmware : not found
)
if exist "%STORAGE_BIN%" (
    set HAS_STORAGE=1
    for %%F in ("%STORAGE_BIN%") do echo      [OK] Storage  : %%~nxF  (%%~zF bytes)
) else (
    echo      [--] Storage  : not found
)

echo.
echo  Select update type:
echo    1. Firmware + Storage
echo    2. Firmware only
echo    3. Storage only
echo    4. Full reset (bootloader + partition + otadata + app + storage)
echo    0. Cancel
echo.
set /p CHOICE="  Choice (0-4): "
if "%CHOICE%"=="0" goto :end

echo.
echo  NOTE: Close idf_monitor.py before flashing!
echo  Press any key to start flashing...
pause > nul
echo.

:: ── 플래시 ────────────────────────────────────────────────────────
echo  [3] Flashing...
echo  ----------------------------------------------------------------

if "%CHOICE%"=="1" (
    echo  Mode: Firmware + Storage
    %ESPTOOL_CMD% --chip esp32s3 --port %COM% --baud %BAUD% ^
        --before default_reset --after hard_reset write_flash ^
        --flash_mode %FLASH_MODE% --flash_freq %FLASH_FREQ% --flash_size %FLASH_SIZE% ^
        %ADDR_APP%     "%APP_BIN%" ^
        %ADDR_STORAGE% "%STORAGE_BIN%"
    goto :done
)
if "%CHOICE%"=="2" (
    echo  Mode: Firmware only
    %ESPTOOL_CMD% --chip esp32s3 --port %COM% --baud %BAUD% ^
        --before default_reset --after hard_reset write_flash ^
        --flash_mode %FLASH_MODE% --flash_freq %FLASH_FREQ% --flash_size %FLASH_SIZE% ^
        %ADDR_APP% "%APP_BIN%"
    goto :done
)
if "%CHOICE%"=="3" (
    echo  Mode: Storage only
    %ESPTOOL_CMD% --chip esp32s3 --port %COM% --baud %BAUD% ^
        --before default_reset --after hard_reset write_flash ^
        --flash_mode %FLASH_MODE% --flash_freq %FLASH_FREQ% --flash_size %FLASH_SIZE% ^
        %ADDR_STORAGE% "%STORAGE_BIN%"
    goto :done
)
if "%CHOICE%"=="4" (
    echo  Mode: Full reset
    %ESPTOOL_CMD% --chip esp32s3 --port %COM% --baud %BAUD% ^
        --before default_reset --after hard_reset write_flash ^
        --flash_mode %FLASH_MODE% --flash_freq %FLASH_FREQ% --flash_size %FLASH_SIZE% ^
        %ADDR_BOOTLOADER% "%BOOTLOADER%" ^
        %ADDR_PARTITION%  "%PARTITION%"  ^
        %ADDR_OTADATA%    "%OTADATA%"    ^
        %ADDR_APP%        "%APP_BIN%"    ^
        %ADDR_STORAGE%    "%STORAGE_BIN%"
    goto :done
)

:done
echo.
if errorlevel 1 (
    echo ================================================================
    echo  [FAILED] Flash error occurred!
    echo  - Close idf_monitor.py and retry
    echo  - Reconnect USB cable and retry
    echo  - Check COM port in Device Manager
    echo ================================================================
) else (
    echo ================================================================
    echo  [SUCCESS] Flash complete! Device is restarting...
    echo ================================================================
)

:end
echo.
pause
