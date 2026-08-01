import argparse
import csv
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# The robot side doesn't send this yet - see Lesson 7 in LESSONS.md for the
# one JSON line per tick you still need to add to the C++ telemetry code.
# Expected: one JSON object per line, e.g. {"t": 12345, "x": 10.2, "y": 24.6, "heading": 87.3}
# t = pros::millis() in ms, x/y = inches, heading = degrees. Lines that don't
# have all four keys (like the existing {"debug": "..."} lines) are skipped.
T_KEY = "t"
X_KEY = "x"
Y_KEY = "y"
HEADING_KEY = "heading"

MOTION_THRESHOLD_IN = 2.0  # how far the tag must move before we call it "start of motion"


def read_camera_csv(path):
    t, x, y, heading = [], [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            if row["found"] != "1":
                continue
            t.append(float(row["time_s"]))
            x.append(float(row["x_in"]))
            y.append(float(row["y_in"]))
            heading.append(float(row["heading_deg"]))
    return np.array(t), np.array(x), np.array(y), np.array(heading)


def read_robot_log(path):
    t, x, y, heading = [], [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not all(k in record for k in (T_KEY, X_KEY, Y_KEY, HEADING_KEY)):
                continue
            t.append(record[T_KEY] / 1000.0)
            x.append(record[X_KEY])
            y.append(record[Y_KEY])
            heading.append(record[HEADING_KEY])

    t = np.array(t)
    order = np.argsort(t)
    return t[order], np.array(x)[order], np.array(y)[order], np.array(heading)[order]


def time_of_first_motion(t, x, y, threshold_in):
    start_x, start_y = x[0], y[0]
    dist = np.hypot(x - start_x, y - start_y)
    moved = np.nonzero(dist > threshold_in)[0]
    if len(moved) == 0:
        return None
    return t[moved[0]]


def auto_sync_offset(t_cam, x_cam, y_cam, t_robot, x_robot, y_robot, threshold_in):
    cam_start = time_of_first_motion(t_cam, x_cam, y_cam, threshold_in)
    robot_start = time_of_first_motion(t_robot, x_robot, y_robot, threshold_in)

    if cam_start is None or robot_start is None:
        print("couldn't detect a clear start-of-motion in one of the logs - "
              "using offset 0.0 (probably wrong, pass --sync-offset yourself)")
        return 0.0

    offset = robot_start - cam_start
    print(f"auto-sync: camera starts moving at t={cam_start:.2f}s, "
          f"robot starts moving at t={robot_start:.2f}s -> offset={offset:+.2f}s")
    return offset


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def interpolate_heading(t_query, t_source, heading_source):
    # can't np.interp raw degrees across the -180/180 wrap (179 and -179 are
    # 2 degrees apart in reality but average to 0, which is wrong). Instead,
    # treat each heading as a point on the unit circle, interpolate that
    # point, then convert back - this sidesteps the wraparound completely.
    rad = np.radians(heading_source)
    cos_i = np.interp(t_query, t_source, np.cos(rad))
    sin_i = np.interp(t_query, t_source, np.sin(rad))
    return np.degrees(np.arctan2(sin_i, cos_i))


def main():
    parser = argparse.ArgumentParser(description="Compare camera ground truth vs robot odometry.")
    parser.add_argument("camera_csv", help="CSV written by 06_video_to_csv.py")
    parser.add_argument("robot_log", help="NDJSON telemetry log from the robot")
    parser.add_argument("--sync-offset", type=float, default=None,
                         help="seconds to add to camera time to line it up with the robot clock "
                              "(default: auto-detect from start-of-motion)")
    parser.add_argument("--motion-threshold", type=float, default=MOTION_THRESHOLD_IN,
                         help="inches of movement that counts as 'started moving' (default: %(default)s)")
    parser.add_argument("--out", default="comparison.png", help="where to save the plot")
    args = parser.parse_args()

    t_cam, x_cam, y_cam, heading_cam = read_camera_csv(args.camera_csv)
    t_robot, x_robot, y_robot, heading_robot = read_robot_log(args.robot_log)

    if len(t_cam) < 2:
        print(f"not enough detected-tag rows in {args.camera_csv} to compare")
        sys.exit(1)
    if len(t_robot) < 2:
        print(f"not enough valid pose lines in {args.robot_log} to compare")
        print(f"(expecting JSON lines with keys: {T_KEY}, {X_KEY}, {Y_KEY}, {HEADING_KEY})")
        sys.exit(1)

    if args.sync_offset is not None:
        offset = args.sync_offset
        print(f"using manual sync offset: {offset:+.2f}s")
    else:
        offset = auto_sync_offset(t_cam, x_cam, y_cam, t_robot, x_robot, y_robot,
                                   args.motion_threshold)

    t_cam_aligned = t_cam + offset

    in_range = (t_cam_aligned >= t_robot[0]) & (t_cam_aligned <= t_robot[-1])
    skipped = len(t_cam_aligned) - int(in_range.sum())
    if skipped:
        print(f"skipping {skipped} camera samples outside the robot log's time range")

    t_cam_aligned = t_cam_aligned[in_range]
    x_cam, y_cam, heading_cam = x_cam[in_range], y_cam[in_range], heading_cam[in_range]

    x_robot_i = np.interp(t_cam_aligned, t_robot, x_robot)
    y_robot_i = np.interp(t_cam_aligned, t_robot, y_robot)
    heading_robot_i = interpolate_heading(t_cam_aligned, t_robot, heading_robot)

    pos_error = np.hypot(x_cam - x_robot_i, y_cam - y_robot_i)
    heading_error = wrap_deg(heading_cam - heading_robot_i)

    worst = int(np.argmax(pos_error))
    print(f"\ncompared {len(pos_error)} samples")
    print(f"position error (in):  mean={pos_error.mean():.2f}  "
          f"median={np.median(pos_error):.2f}  max={pos_error.max():.2f} "
          f"(at t={t_cam_aligned[worst]:.2f}s)")
    print(f"heading error (deg):  mean={np.abs(heading_error).mean():.2f}  "
          f"max={np.abs(heading_error).max():.2f}")

    fig, (ax_path, ax_pos, ax_heading) = plt.subplots(3, 1, figsize=(8, 11))

    ax_path.plot(x_cam, y_cam, "g-", label="camera (ground truth)")
    ax_path.plot(x_robot_i, y_robot_i, "b--", label="robot odometry")
    ax_path.set_xlabel("x (in)")
    ax_path.set_ylabel("y (in)")
    ax_path.set_aspect("equal", adjustable="datalim")
    ax_path.legend()
    ax_path.set_title("path")

    ax_pos.plot(t_cam_aligned, pos_error, "r-")
    ax_pos.set_xlabel("time (s)")
    ax_pos.set_ylabel("position error (in)")
    ax_pos.set_title("position error over time")

    ax_heading.plot(t_cam_aligned, heading_error, "m-")
    ax_heading.set_xlabel("time (s)")
    ax_heading.set_ylabel("heading error (deg)")
    ax_heading.set_title("heading error over time")

    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"saved plot to {args.out}")
    plt.show()


if __name__ == "__main__":
    main()
