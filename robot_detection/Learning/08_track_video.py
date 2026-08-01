import csv
import sys
from pathlib import Path

import cv2
import numpy as np

TARGET_TAG_ID = 0

# Flip to -1 if headings come out backwards - see Lesson 6 in LESSONS.md for
# how to check.
HEADING_SIGN = 1

# Real-world field coordinates (inches) of the 4 points you'll click on the
# first frame, IN THE ORDER you click them. Defaults to one 24x24 in field
# tile - edit to match whatever 4 points you actually pick.
FIELD_POINTS_IN = [
    (0.0, 0.0),
    (24.0, 0.0),
    (24.0, 24.0),
    (0.0, 24.0),
]

WINDOW = "click the 4 points in order - r redo, c confirm, q/ESC quit"

# Phone video is often much taller/wider than a monitor (e.g. 1080x1920
# portrait). Shown at native size, the window gets clipped by the screen and
# you only ever see a corner of it. This caps the *display* size only - clicks
# are converted back to full-resolution pixel coordinates before being stored,
# so calibration accuracy is unaffected.
MAX_DISPLAY_DIM = 900

clicked_points = []
display_scale = 1.0


def on_click(event, x, y, flags, userdata):
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    if len(clicked_points) < 4:
        clicked_points.append((float(x) / display_scale, float(y) / display_scale))


def draw(display_frame):
    canvas = display_frame.copy()
    for i, (x, y) in enumerate(clicked_points):
        dx, dy = int(x * display_scale), int(y * display_scale)
        cv2.circle(canvas, (dx, dy), 6, (0, 255, 0), -1)
        cv2.putText(canvas, str(i), (dx + 10, dy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    if len(clicked_points) == 4:
        cv2.putText(canvas, "press c to confirm and start tracking, r to redo",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return canvas


def calibrate_on_first_frame(frame):
    global display_scale

    h, w = frame.shape[:2]
    display_scale = min(1.0, MAX_DISPLAY_DIM / max(h, w))
    disp_w, disp_h = int(w * display_scale), int(h * display_scale)
    display_frame = cv2.resize(frame, (disp_w, disp_h)) if display_scale != 1.0 else frame

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, disp_w, disp_h)
    cv2.setMouseCallback(WINDOW, on_click)

    print("click these 4 points on the first frame, in this exact order (field inches):")
    for i, p in enumerate(FIELD_POINTS_IN):
        print(f"   {i}: field ({p[0]:.1f} in, {p[1]:.1f} in)")
    print("press r to restart the 4 clicks, c to confirm once all 4 look right")

    while True:
        cv2.imshow(WINDOW, draw(display_frame))
        key = cv2.waitKey(20) & 0xFF

        if key == ord("r"):
            clicked_points.clear()
            print("cleared - click the 4 points again")

        elif key == ord("c") and len(clicked_points) == 4:
            cv2.destroyWindow(WINDOW)
            src = np.float32(clicked_points)
            dst = np.float32(FIELD_POINTS_IN)
            homography, _ = cv2.findHomography(src, dst)
            return homography

        elif key in (ord("q"), 27):
            cv2.destroyWindow(WINDOW)
            print("quit before confirming - nothing processed")
            sys.exit(0)


def tag_field_pose(corners, homography):
    pts = corners.reshape(4, 2).astype(np.float32).reshape(-1, 1, 2)
    transformed = cv2.perspectiveTransform(pts, homography).reshape(4, 2)

    center = transformed.mean(axis=0)
    top_edge = transformed[1] - transformed[0]
    heading = HEADING_SIGN * np.degrees(np.arctan2(top_edge[1], top_edge[0]))

    return center[0], center[1], heading


def track(cap, first_frame, fps, homography, out_path):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

    frame = first_frame
    frame_idx = 0
    found_count = 0

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["frame", "time_s", "found", "x_in", "y_in", "heading_deg"])

        while True:
            time_s = frame_idx / fps
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            row = [frame_idx, f"{time_s:.4f}", 0, "", "", ""]

            if ids is not None:
                for tag_corners, tag_id in zip(corners, ids.ravel()):
                    if tag_id != TARGET_TAG_ID:
                        continue
                    x_in, y_in, heading = tag_field_pose(tag_corners, homography)
                    row = [frame_idx, f"{time_s:.4f}", 1,
                           f"{x_in:.3f}", f"{y_in:.3f}", f"{heading:.2f}"]
                    found_count += 1
                    break

            writer.writerow(row)

            if frame_idx % 100 == 0:
                print(f"frame {frame_idx}  ({time_s:6.1f}s)  found so far: {found_count}")

            frame_idx += 1
            ok, frame = cap.read()
            if not ok:
                break

    return frame_idx, found_count


def main():
    if len(sys.argv) < 2:
        print("usage: 08_track_video.py path/to/video.mp4 [output.csv]")
        sys.exit(1)

    video_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else video_path.with_suffix(".csv")
    homography_path = video_path.with_name(video_path.stem + "_homography.npy")

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"could not open {video_path}")
        sys.exit(1)

    ok, first_frame = cap.read()
    if not ok:
        print(f"could not read a first frame from {video_path}")
        sys.exit(1)

    homography = calibrate_on_first_frame(first_frame)
    np.save(homography_path, homography)
    print(f"saved homography to {homography_path} (only valid for this camera setup - "
          f"see LESSONS.md Lesson 5/8 if you move the camera)")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps <= 0:
        fps = 30.0
        print(f"video didn't report a frame rate - assuming {fps} fps")

    total_frames, found_count = track(cap, first_frame, fps, homography, out_path)
    cap.release()

    pct = 100.0 * found_count / total_frames if total_frames else 0.0
    print(f"\ndone: {total_frames} frames, tag found in {found_count} ({pct:.1f}%)")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
