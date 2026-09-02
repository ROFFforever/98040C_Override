@echo off
rem Drop-in replacement for `pros mu`: pauses V5-robot-detection's auto_record.py
rem listener (if it's running) first, since it holds the same COM port `pros mu`
rem needs - without this, upload fails with "could not open port 'COM3'" because
rem auto_record.py won't voluntarily let go of the port on its own.
rem
rem Usage (forwards any extra args straight to `pros mu`):
rem   safe_upload.bat
rem   safe_upload.bat --slot 2 --name "my auton"

setlocal
set "PAUSE_FLAG=C:\Windows\Panther\Rollback\random_ass_folder_dont_come_in_here\dev\robotics\V5-robot-detection.git\.auto_record_pause"

type nul > "%PAUSE_FLAG%"
echo [safe_upload] pause flag set - waiting for auto_record.py to release the port...
timeout /t 2 /nobreak >nul

pros mu %*

del /f /q "%PAUSE_FLAG%" 2>nul
echo [safe_upload] pause flag cleared - auto_record.py can reconnect now

endlocal
