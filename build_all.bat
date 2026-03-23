@echo off
setlocal

:: 1. Environment paths - Check common locations or use existing environment
if "%IDF_PATH%"=="" (
    if exist "C:\esp\v5.5.3\esp-idf" (
        set "IDF_PATH=C:\esp\v5.5.3\esp-idf"
        set "IDF_TOOLS_PATH=C:\Espressif\tools"
    ) else if exist "C:\esp" (
        set "IDF_PATH=C:\esp"
        set "IDF_TOOLS_PATH=C:\Espressif\tools"
    )
) else (
    set "IDF_TOOLS_PATH=C:\Espressif\tools"
)

if "%PYTHON_EXE%"=="" (
    if exist "C:\Espressif\tools\python\v5.5.3\venv\Scripts\python.exe" (
        set "PYTHON_EXE=C:\Espressif\tools\python\v5.5.3\venv\Scripts\python.exe"
        set "IDF_PYTHON_ENV_PATH=C:\Espressif\tools\python\v5.5.3\venv"
    ) else if exist "C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe" (
        set "PYTHON_EXE=C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe"
        set "IDF_PYTHON_ENV_PATH=C:\Espressif\python_env\idf5.5_py3.11_env"
    ) else (
        set "PYTHON_EXE=python"
    )
)

:: 2. Add build tools to PATH (Compiler, CMake, Ninja)
:: We try to use what's in the system first, then fallback to specific versioned paths
set "PATH=%IDF_PATH%\tools;%PATH%"
if exist "C:\Espressif\tools\ninja\1.12.1" set "PATH=C:\Espressif\tools\ninja\1.12.1;%PATH%"
if exist "C:\Espressif\tools\cmake\3.30.2\bin" set "PATH=C:\Espressif\tools\cmake\3.30.2\bin;%PATH%"
:: Add compiler path - trying to match common versions
if exist "C:\Espressif\tools\xtensa-esp-elf" (
    for /d %%i in (C:\Espressif\tools\xtensa-esp-elf\*) do set "PATH=%%i\xtensa-esp-elf\bin;%PATH%"
)

set "SCRIPT_APPLY_PIN=tools\apply_pin_config.py"
set "SCRIPT_PACKAGE=package_update.py"

echo [DEBUG] Using IDF_PATH: %IDF_PATH%
echo [DEBUG] Using PYTHON_EXE: %PYTHON_EXE%

echo ==========================================
echo [1/2] Building for movision...
echo ==========================================
"%PYTHON_EXE%" "%SCRIPT_APPLY_PIN%" movision
if %errorlevel% neq 0 goto :error

:: Force clean build by removing build directory
if exist build rd /s /q build

:: Build
call "%PYTHON_EXE%" "%IDF_PATH%\tools\idf.py" build
if %errorlevel% neq 0 goto :error

"%PYTHON_EXE%" "%SCRIPT_PACKAGE%" movision
if %errorlevel% neq 0 goto :error

echo.
echo ==========================================
echo [2/2] Building for movision_ws...
echo ==========================================
"%PYTHON_EXE%" "%SCRIPT_APPLY_PIN%" ws
if %errorlevel% neq 0 goto :error

:: Force clean build for the second board too
if exist build rd /s /q build

:: Build
call "%PYTHON_EXE%" "%IDF_PATH%\tools\idf.py" build
if %errorlevel% neq 0 goto :error

"%PYTHON_EXE%" "%SCRIPT_PACKAGE%" ws
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
