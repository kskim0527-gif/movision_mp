@echo off
setlocal

:: 1. Environment paths
set "IDF_PATH=C:\esp"
set "PYTHON_EXE=C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe"

:: 2. Add build tools to PATH (Compiler, CMake, Ninja)
set "PATH=C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20251107\xtensa-esp-elf\bin;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\cmake\3.30.2\bin;%PATH%"

set "SCRIPT_APPLY_PIN=tools\apply_pin_config.py"
set "SCRIPT_PACKAGE=package_update.py"

echo ==========================================
echo [1/2] Building for movision_ws...
echo ==========================================
"%PYTHON_EXE%" "%SCRIPT_APPLY_PIN%" ws
if %errorlevel% neq 0 goto :error

:: Force clean build by removing build directory
if exist build rd /s /q build

:: Build
call "%PYTHON_EXE%" "%IDF_PATH%\tools\idf.py" build
if %errorlevel% neq 0 goto :error

"%PYTHON_EXE%" "%SCRIPT_PACKAGE%" ws
if %errorlevel% neq 0 goto :error

echo.
echo ==========================================
echo [2/2] Building for movision-0223...
echo ==========================================
"%PYTHON_EXE%" "%SCRIPT_APPLY_PIN%" 0223
if %errorlevel% neq 0 goto :error

:: Force clean build for the second board too
if exist build rd /s /q build

:: Build
call "%PYTHON_EXE%" "%IDF_PATH%\tools\idf.py" build
if %errorlevel% neq 0 goto :error

"%PYTHON_EXE%" "%SCRIPT_PACKAGE%" 0223
if %errorlevel% neq 0 goto :error

echo.
echo ==========================================
echo All builds completed successfully!
echo Results are in the "update" folder.
echo ==========================================
pause
exit /b 0

:error
echo.
echo [ERROR] Build failed.
pause
exit /b 1
