# Drop-in replacement for `pros mu`: pauses V5-robot-detection's auto_record.py
# listener (if it's running) first, since it holds the same COM port `pros mu`
# needs - without this, upload fails with "could not open port 'COM3'" because
# auto_record.py won't voluntarily let go of the port on its own.
#
# Usage (forwards any extra args straight to `pros mu`):
#   .\safe_upload.ps1
#   .\safe_upload.ps1 --slot 2 --name "my auton"

$PauseFlag = "C:\Windows\Panther\Rollback\random_ass_folder_dont_come_in_here\dev\robotics\V5-robot-detection.git\.auto_record_pause"

New-Item -ItemType File -Path $PauseFlag -Force | Out-Null
Write-Host "[safe_upload] pause flag set - waiting for auto_record.py to release the port..."
Start-Sleep -Seconds 2

try {
    pros mu @args
}
finally {
    Remove-Item -Path $PauseFlag -Force -ErrorAction SilentlyContinue
    Write-Host "[safe_upload] pause flag cleared - auto_record.py can reconnect now"
}
