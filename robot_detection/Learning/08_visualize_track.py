import csv
import sys
from pathlib import Path

import cv2
import numpy as np

MAX_DISPLAY_DIM = 900
HEADING_TICK_IN = 6.0


def load_track(csv_path):
    frame_indices = []
    field_points = []
    headings = []
    not_found_frames = set()
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["found"] != "1":
                not_found_frames.add(int(row["frame"]))
                continue
            frame_indices.append(int(row["frame"]))
            field_points.append((float(row["x_in"]), float(row["y_in"])))
            headings.append(float(row["heading_deg"]))
    return frame_indices, field_points, headings, not_found_frames


def find_homography_path(video_path):
    per_video = video_path.with_name(video_path.stem + "_homography.npy")
    if per_video.exists():
        return per_video
    shared = video_path.with_name("homography.npy")
    if shared.exists():
        return shared
    return None


def project_path_and_headings(field_points, headings, inv_homography):
    centers = np.float32(field_points)
    angles = np.radians(headings)
    tips = centers + HEADING_TICK_IN * np.column_stack((np.cos(angles), np.sin(angles)))

    pts = np.vstack((centers, tips)).astype(np.float32).reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(pts, inv_homography).reshape(-1, 2)

    n = len(field_points)
    return projected[:n], projected[n:]


def main():
    if len(sys.argv) < 3:
        print("usage: 08_visualize_track.py path/to/video.mp4 path/to/track.csv [output.mp4]")
        sys.exit(1)

    video_path = Path(sys.argv[1])
    csv_path = Path(sys.argv[2])
    out_path = Path(sys.argv[3]) if len(sys.argv) > 3 else None

    homography_path = find_homography_path(video_path)
    if homography_path is None:
        print(f"no homography file found next to {video_path} - run 08_track_video.py on this video first")
        sys.exit(1)

    homography = np.load(homography_path)
    inv_homography = np.linalg.inv(homography)

    frame_indices, field_points, headings, not_found_frames = load_track(csv_path)
    if not frame_indices:
        print(f"no rows with found=1 in {csv_path} - nothing to draw")
        sys.exit(1)

    centers_px, tips_px = project_path_and_headings(field_points, headings, inv_homography)
    center_by_frame = {fi: (int(x), int(y)) for fi, (x, y) in zip(frame_indices, centers_px)}
    tip_by_frame = {fi: (int(x), int(y)) for fi, (x, y) in zip(frame_indices, tips_px)}

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"could not open {video_path}")
        sys.exit(1)

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    writer = None
    if out_path is not None:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(out_path), fourcc, fps, (frame_w, frame_h))

    display_scale = 1.0
    if writer is None:
        display_scale = min(1.0, MAX_DISPLAY_DIM / max(frame_w, frame_h))

    trail = []
    frame_idx = 0
    ok, frame = cap.read()
    while ok:
        if frame_idx in center_by_frame:
            trail.append(center_by_frame[frame_idx])

        if len(trail) > 1:
            cv2.polylines(frame, [np.array(trail, dtype=np.int32)], False, (0, 255, 255), 2)

        if frame_idx in center_by_frame:
            center = center_by_frame[frame_idx]
            cv2.circle(frame, center, 8, (0, 0, 255), -1)
            cv2.line(frame, center, tip_by_frame[frame_idx], (255, 0, 0), 2)
        elif frame_idx in not_found_frames:
            cv2.putText(frame, "TAG NOT DETECTED", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        if writer is not None:
            writer.write(frame)
        else:
            display_frame = frame
            if display_scale != 1.0:
                display_frame = cv2.resize(
                    frame, (int(frame_w * display_scale), int(frame_h * display_scale)))
            cv2.imshow("path overlay - q/ESC to quit", display_frame)
            key = cv2.waitKey(max(1, int(1000 / fps))) & 0xFF
            if key in (ord("q"), 27):
                break

        frame_idx += 1
        ok, frame = cap.read()

    cap.release()
    if writer is not None:
        writer.release()
        print(f"wrote {out_path}")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
