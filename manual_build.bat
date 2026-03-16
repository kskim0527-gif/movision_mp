@echo off
set "IDF_PATH=C:\esp\v5.5.3\esp-idf"
set "PATH=C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20251107\xtensa-esp-elf\bin;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\cmake\3.30.2\bin;%PATH%"
set "PYTHON=C:\Espressif\tools\python\v5.5.3\venv\Scripts\python.exe"
set "IDF_SKIP_CHECK_SUBMODULES=1"

echo ========================================
echo Cleaning build directory...
echo ========================================
if exist build rd /s /q build

echo ========================================
echo Starting build...
echo ========================================
cmake -G Ninja -DPYTHON_DEPS_CHECKED=1 -DESP_PLATFORM=1 -B build -S . -DSDKCONFIG=sdkconfig
if %errorlevel% neq 0 (
    echo CMake failed!
    exit /b %errorlevel%
)

ninja -C build
if %errorlevel% neq 0 (
    echo Ninja failed!
    exit /b %errorlevel%
)

echo ========================================
echo Packaging update files...
echo ========================================
%PYTHON% package_update.py
if %errorlevel% neq 0 (
    echo Packaging failed!
    exit /b %errorlevel%
)

echo ========================================
echo Build and Packaging Complete!
echo ========================================
pause
