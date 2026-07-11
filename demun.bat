@echo off
:: This file name stands for: pros mu (build and upload) + debug

set LOGFILE=%~dp0log.txt

echo Run started: %date% %time%

:: 1. Run PROS mu to build and upload code
pros mu

:: 1a. Log the timestamp + result, and bail out here if it failed so the
::     error doesn't get wiped by cls before you can read it
if errorlevel 1 (
    echo %date% %time% - FAILED >> "%LOGFILE%"
    echo.
    echo BUILD/UPLOAD FAILED - see above
    pause
    exit /b 1
)

echo %date% %time% - SUCCESS >> "%LOGFILE%"

:: 2. Pause so you can read the pros mu result before it scrolls away
timeout /t 1 /nobreak >nul

:: 3. Clear the screen
cls

:: 4. Run the Python script using the py launcher
py C:\dev\robotics\MOA\98040C\src\utils\telemetry\receive_telemetry.py
