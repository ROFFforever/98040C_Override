import json
from pathlib import Path

#saved settings live next to the tool so they follow the repo between machines
SETTINGS_DIR = Path(__file__).resolve().parent.parent / "settings"


def load(name, default):
    path = SETTINGS_DIR / name
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return default


def save(name, data):
    SETTINGS_DIR.mkdir(exist_ok=True)
    (SETTINGS_DIR / name).write_text(json.dumps(data, indent=2) + "\n")
