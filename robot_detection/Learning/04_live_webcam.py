import sys

import cv2
import numpy as np

CAMERA_INDEX = 0
CORNER_COLORS = ((0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255))


def open_camera(index):
    for backend in (cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY):
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            return cap
        cap.release()
    return None


def draw_tag(frame, pts, tag_id):
    center = pts.mean(axis=0)
    top_edge = pts[1] - pts[0]
    heading = np.degrees(np.arctan2(top_edge[1], top_edge[0]))
    size_px = np.linalg.norm(top_edge)

    cv2.polylines(frame, [pts.astype(np.int32)], True, (0, 255, 0), 2)

    for color, (x, y) in zip(CORNER_COLORS, pts):
        cv2.circle(frame, (int(x), int(y)), 5, color, -1)

    forward = top_edge / size_px * (size_px * 0.7)
    cv2.arrowedLine(frame, center.astype(int), (center + forward).astype(int),
                    (255, 0, 255), 2, tipLength=0.25)

    label = f"id {tag_id}  {heading:6.1f}deg  {size_px:.0f}px"
    cv2.putText(frame, label, (int(center[0]) - 90, int(center[1]) + 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
    cv2.putText(frame, label, (int(center[0]) - 90, int(center[1]) + 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)


def main():
    cap = open_camera(CAMERA_INDEX)
    if cap is None:
        print(f"could not open camera {CAMERA_INDEX}")
        print("try changing CAMERA_INDEX to 1 or 2 at the top of this file")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"camera open at {width}x{height}")
    print("hold a printed tag up to the camera.  q or ESC to quit.")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

    tick_freq = cv2.getTickFrequency()
    last_tick = cv2.getTickCount()
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("dropped frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            for tag_corners, tag_id in zip(corners, ids.ravel()):
                draw_tag(frame, tag_corners.reshape(4, 2), tag_id)

        now = cv2.getTickCount()
        dt = (now - last_tick) / tick_freq
        last_tick = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

        count = 0 if ids is None else len(ids)
        hud = f"{fps:5.1f} fps    tags: {count}"
        cv2.putText(frame, hud, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(frame, hud, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("live tag detection - q to quit", frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
