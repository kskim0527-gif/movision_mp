@echo off
title Movition Factory PC Flash Tool
color 0f

echo ==============================================
echo       Movition Windows OS Factory Tool
echo ==============================================
echo Please enter the COM PORT NUMBER (Example: For COM4, just type 4)
echo Press Enter without typing anything to default to COM4.
echo.

set /p PORT_NUM="Enter Port Number: "
if "%PORT_NUM%"=="" set PORT_NUM=4

echo.
echo Starting to flash on COM%PORT_NUM%...
echo.

factory_flash.exe -p COM%PORT_NUM%

echo.
echo Finished. Press any key to exit.
pause >nul
