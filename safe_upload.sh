#!/bin/zsh
# Mac version of safe_upload.bat: pauses auto_record.py so it releases the
# serial port, runs `pros mu`, then lets auto_record.py reconnect.
# Usage: ./safe_upload.sh [extra pros mu args]

PAUSE_FLAG="${0:A:h}/../V5-robot-detection/.auto_record_pause"

touch "$PAUSE_FLAG"
trap 'rm -f "$PAUSE_FLAG"; echo "[safe_upload] pause flag cleared - auto_record.py can reconnect now"' EXIT
echo "[safe_upload] pause flag set - waiting for auto_record.py to release the port..."
sleep 2

pros mu "$@"
