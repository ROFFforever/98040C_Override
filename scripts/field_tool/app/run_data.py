import csv
import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from app import field_config as cfg
from app.kinematics import compute_motion

DEFAULT_FRAME_DT = 0.02  #track.csv time_s = frame x 0.02 (50fps)


@dataclass
class Trace:
    key: str             #"robot" or "camera"
    name: str
    color: str
    t: np.ndarray        #seconds on the run's clock (see Run.clock)
    x: np.ndarray        #inches
    y: np.ndarray
    heading: np.ndarray  #degrees CCW, 0 = facing +X
    frame: np.ndarray    #video frame of each point, NaN if unknown
    frame_exact: bool    #camera points ARE frames, robot points only map to the nearest one
    no_frame_reason: str = "outside video"
    v: np.ndarray = None
    a: np.ndarray = None
    omega: np.ndarray = None

    def __len__(self):
        return len(self.t)


@dataclass
class Run:
    name: str
    folder: Path
    robot: Trace | None
    camera: Trace | None
    clock: str           #"robot" = ndjson t (s since program start), "video" = camera-only run
    synced: bool
    notes: list = field(default_factory=list)

    def traces(self):
        return [tr for tr in (self.robot, self.camera) if tr is not None]


def is_run_folder(folder):
    folder = Path(folder)
    return (folder / "track.csv").exists() or any(folder.glob("*.ndjson"))


def list_runs():
    """Run folders in V5-robot-detection/runs, newest first (same order as its GUI)."""
    if not cfg.RUNS_DIR.exists():
        return []
    runs = [d for d in cfg.RUNS_DIR.iterdir() if d.is_dir() and is_run_folder(d)]
    return sorted(runs, key=lambda d: _read_run_info(d).get("generated") or "", reverse=True)


def load_run(folder):
    folder = Path(folder)
    info = _read_run_info(folder)
    notes = []

    offset = float(info.get("sync_offset_s") or 0.0)
    scale = float(info.get("sync_scale") or 1.0)
    #RUNS.md: offset exactly 0 and scale exactly 1 means the run was never synced
    synced = not (offset == 0.0 and scale == 1.0)

    cam = _load_track(folder / "track.csv")
    log_path = _find_robot_log(folder, info)
    robot_cols = _load_pose_log(log_path, notes) if log_path else None
    if robot_cols is not None and len(robot_cols[0]) == 0:
        notes.append(f"{log_path.name} has no pose lines")
        robot_cols = None

    clock = "robot" if robot_cols is not None else "video"
    frame_dt = _frame_dt(cam)

    camera = None
    if cam is not None:
        frames, video_t, x, y, h = cam
        #robot seconds = video seconds x sync_scale + sync_offset_s (from run_info.json)
        t = video_t * scale + offset if clock == "robot" else video_t
        camera = make_trace("camera", "Camera tag", cfg.CAMERA_COLOR, t, x, y, h, frames.astype(float), True)

    robot = None
    if robot_cols is not None:
        t, x, y, h = robot_cols
        if synced and cam is not None:
            frame = (t - offset) / scale / frame_dt
            frame[(frame < cam[0][0] - 0.5) | (frame > cam[0][-1] + 0.5)] = np.nan
            reason = "outside video"
        else:
            frame = np.full(len(t), np.nan)
            reason = "no video sync"
        robot = make_trace("robot", "Robot odom", cfg.ROBOT_COLOR, t, x, y, h, frame, False, reason)

    if robot is not None and camera is not None and not synced:
        notes.append("run was never synced, so camera and robot times don't line up")
    if robot is None:
        notes.append("no robot log")
    if camera is None:
        notes.append("no camera track")
    return Run(folder.name, folder, robot, camera, clock, synced, notes)


def make_trace(key, name, color, t, x, y, h, frame, frame_exact, reason="outside video"):
    tr = Trace(key, name, color, t, x, y, h, frame, frame_exact, reason)
    tr.v, tr.a, tr.omega = compute_motion(t, x, y, h, cfg.VELOCITY_WINDOW_S, cfg.ACCEL_WINDOW_S)
    return tr


def _read_run_info(folder):
    path = Path(folder) / "run_info.json"
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except (json.JSONDecodeError, OSError):
        return {}


def _find_robot_log(folder, info):
    #prefer the copy inside the folder - run_info's absolute path breaks if the runs folder moves
    logged = info.get("robot_log")
    if logged:
        local = folder / Path(logged).name
        if local.exists():
            return local
        if Path(logged).exists():
            return Path(logged)
    logs = sorted(folder.glob("*.ndjson"), key=lambda p: p.stat().st_size, reverse=True)
    return logs[0] if logs else None


def _load_track(path):
    """Camera track -> (frame, video_t, x, y, heading), only frames where the tag was found."""
    if not path.exists():
        return None
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                if r.get("found", "1") != "1":
                    continue
                rows.append((int(r["frame"]), float(r["time_s"]), float(r["x_in"]), float(r["y_in"]), float(r["heading_deg"])))
            except (KeyError, ValueError):
                continue  #blank/garbled row
    if not rows:
        return None
    a = np.array(rows)
    a = a[np.argsort(a[:, 0], kind="stable")]
    return a[:, 0].astype(int), a[:, 1], a[:, 2], a[:, 3], a[:, 4]


def _load_pose_log(path, notes):
    """Pose lines {t, x, y, heading, ...} from the robot's ndjson -> (t seconds, x, y, heading)."""
    raw = path.read_bytes()
    #SD card logs are utf-8, but `pros terminal > file` in PowerShell writes utf-16
    text = raw.decode("utf-16") if raw[:2] in (b"\xff\xfe", b"\xfe\xff") else raw.decode("utf-8-sig", errors="replace")
    rows, bad = [], 0
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            bad += 1  #e.g. a "nan" printed by std::format isn't valid JSON
            continue
        if "event" in d or "axis" in d:
            continue
        try:
            rows.append((float(d["t"]) / 1000.0, float(d["x"]), float(d["y"]), float(d["heading"])))
        except (KeyError, TypeError, ValueError):
            continue
    if bad:
        notes.append(f"skipped {bad} unreadable log lines")
    if not rows:
        return np.empty(0), np.empty(0), np.empty(0), np.empty(0)
    a = np.array(rows)
    a = a[np.argsort(a[:, 0], kind="stable")]
    return a[:, 0], a[:, 1], a[:, 2], a[:, 3]


def _frame_dt(cam):
    if cam is None or len(cam[0]) < 2:
        return DEFAULT_FRAME_DT
    df = np.diff(cam[0])
    ok = df > 0
    return float(np.median(np.diff(cam[1])[ok] / df[ok])) if ok.any() else DEFAULT_FRAME_DT
