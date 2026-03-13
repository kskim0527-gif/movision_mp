@echo off
chcp 65001 > nul
title movision BLE OTA 업데이트

:: ================================================================
::  movision BLE OTA 업데이트 도구
::  이 파일을 더블클릭하면 블루투스로 장치를 업데이트합니다.
:: ================================================================

echo.
echo ================================================================
echo    movision BLE OTA 업데이트 도구
echo ================================================================
echo.

:: ── Python 확인 ──────────────────────────────────────────────
where python >nul 2>&1
if errorlevel 1 (
    echo [오류] Python이 설치되어 있지 않습니다.
    echo        https://python.org 에서 Python 3.8 이상을 설치하세요.
    pause
    exit /b 1
)

for /f "tokens=*" %%i in ('python --version 2^>^&1') do set PYVER=%%i
echo  Python 버전: %PYVER%

:: ── bleak 라이브러리 확인 / 자동 설치 ────────────────────────
echo  bleak 라이브러리 확인 중...
python -m pip show bleak >nul 2>&1
if errorlevel 1 (
    echo  bleak 설치 중... (최초 1회)
    python -m pip install bleak -q
    if errorlevel 1 (
        echo [오류] bleak 설치 실패. 인터넷 연결을 확인하세요.
        pause
        exit /b 1
    )
    echo  bleak 설치 완료!
) else (
    echo  bleak OK
)

:: ── 빌드 파일 확인 ────────────────────────────────────────────
echo.
set BUILD_DIR=%~dp0..\build

if exist "%BUILD_DIR%\movision.bin" (
    echo  [OK] 펌웨어  : %BUILD_DIR%\movision.bin
) else (
    echo  [--] 펌웨어  : 없음 (storage만 업데이트됩니다)
)

if exist "%BUILD_DIR%\storage.bin" (
    echo  [OK] Storage : %BUILD_DIR%\storage.bin
) else (
    echo  [--] Storage : 없음 (firmware만 업데이트됩니다)
)

:: ── OTA 실행 ──────────────────────────────────────────────────
echo.
echo  PC의 블루투스가 켜져 있고,
echo  movision 장치의 전원이 ON 상태인지 확인하세요.
echo.
echo  계속하려면 아무 키나 누르세요...
pause > nul

echo.
python "%~dp0ble_ota.py"

echo.
if errorlevel 1 (
    echo [실패] 업데이트 중 오류가 발생했습니다.
) else (
    echo [성공] 업데이트가 완료되었습니다!
)

pause
