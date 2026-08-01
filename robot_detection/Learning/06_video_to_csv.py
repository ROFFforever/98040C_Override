import csv
import sys
from pathlib import Path

import cv2
import numpy as np

HOMOGRAPHY_PATH = Path(__file__).parent / "homography.npy"
TARGET_TAG_ID = 0

# Flip to -1 if headings come out backwards once you check them against
# reality (see Lesson 6 in LESSONS.md for how to tell).
HEADING_SIGN = 1


def load_homography():
    if not HOMOGRAPHY_PATH.exists():
        print(f"no {HOMOGRAPHY_PATH.name} found - run 05_homography.py first")
        sys.exit(1)
    return np.load(HOMOGRAPHY_PATH)


def tag_field_pose(corners, homography):
    pts = corners.reshape(4, 2).astype(np.float32).reshape(-1, 1, 2)
    transformed = cv2.perspectiveTransform(pts, homography).reshape(4, 2)

    center = transformed.mean(axis=0)
    top_edge = transformed[1] - transformed[0]
    heading = HEADING_SIGN * np.degrees(np.arctan2(top_edge[1], top_edge[0]))

    return center[0], center[1], heading


def main():
    if len(sys.argv) < 2:
        print("usage: 06_video_to_csv.py path/to/video.mp4 [output.csv]")
        sys.exit(1)

    video_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else video_path.with_suffix(".csv")

    homography = load_homography()

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"could not open {video_path}")
        sys.exit(1)

    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps <= 0:
        fps = 30.0
        print(f"video didn't report a frame rate - assuming {fps} fps")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

    frame_idx = 0
    found_count = 0

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["frame", "time_s", "found", "x_in", "y_in", "heading_deg"])

        while True:
            ok, frame = cap.read()
            if not ok:
                break

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

    cap.release()

    pct = 100.0 * found_count / frame_idx if frame_idx else 0.0
    print(f"\ndone: {frame_idx} frames, tag found in {found_count} ({pct:.1f}%)")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
