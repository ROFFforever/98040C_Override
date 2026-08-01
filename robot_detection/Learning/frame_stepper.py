import csv
import ctypes
import sys
import time
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np

TARGET_TAG_ID = 0
HEADING_SIGN = 1

FIELD_POINTS_IN = [
    (0.0, 0.0),
    (24.0, 0.0),
    (24.0, 24.0),
    (0.0, 24.0),
]

AXIS_LEN_IN = 24.0
TICK_INTERVAL_IN = 4.0
HEADING_TICK_IN = 6.0

MAX_DISPLAY_DIM = 900
CALIB_WINDOW = "click the 4 points in order - r redo, c confirm, q/ESC quit"
STEP_WINDOW = ("frame stepper  -  left/right +-1   ctrl+left/right +-5   "
               "up/down +-30   drag bar to seek   q/ESC quit")
TRACKBAR = "frame"

VK_LEFT, VK_UP, VK_RIGHT, VK_DOWN = 0x25, 0x26, 0x27, 0x28
VK_CONTROL = 0x11

REPEAT_INITIAL_DELAY_S = 0.35
REPEAT_INTERVAL_S = 0.06

FRAME_CACHE_SIZE = 90

RDW_INVALIDATE = 0x0001
RDW_UPDATENOW = 0x0100
RDW_ALLCHILDREN = 0x0080

try:
    _user32 = ctypes.windll.user32
except AttributeError:
    _user32 = None

_step_hwnd = None


def key_held(vk):
    if _user32 is None:
        return False
    return bool(_user32.GetAsyncKeyState(vk) & 0x8000)


def ctrl_held():
    return key_held(VK_CONTROL)


def force_repaint(window_name):
    global _step_hwnd
    if _user32 is None:
        return
    if _step_hwnd is None or not _user32.IsWindow(_step_hwnd):
        _step_hwnd = _user32.FindWindowW(None, window_name)
    if _step_hwnd:
        _user32.RedrawWindow(_step_hwnd, None, None, RDW_INVALIDATE | RDW_UPDATENOW | RDW_ALLCHILDREN)


def compute_display_scale(frame_w, frame_h):
    if _user32 is not None:
        max_w = _user32.GetSystemMetrics(0) * 0.92
        max_h = _user32.GetSystemMetrics(1) * 0.85
    else:
        max_w = max_h = MAX_DISPLAY_DIM
    return min(1.0, max_w / frame_w, max_h / frame_h)


clicked_points = []
display_scale = 1.0


def on_click(event, x, y, flags, userdata):
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    if len(clicked_points) < 4:
        clicked_points.append((float(x) / display_scale, float(y) / display_scale))


def draw_calib(display_frame):
    canvas = display_frame.copy()
    for i, (x, y) in enumerate(clicked_points):
        dx, dy = int(x * display_scale), int(y * display_scale)
        cv2.circle(canvas, (dx, dy), 6, (0, 255, 0), -1)
        cv2.putText(canvas, str(i), (dx + 10, dy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    if len(clicked_points) == 4:
        cv2.putText(canvas, "press c to confirm, r to redo",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return canvas


def calibrate_on_first_frame(frame):
    global display_scale

    h, w = frame.shape[:2]
    display_scale = compute_display_scale(w, h)
    disp_w, disp_h = int(w * display_scale), int(h * display_scale)
    display_frame = cv2.resize(frame, (disp_w, disp_h)) if display_scale != 1.0 else frame

    cv2.namedWindow(CALIB_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CALIB_WINDOW, disp_w, disp_h)
    cv2.setMouseCallback(CALIB_WINDOW, on_click)

    print("click these 4 points on the first frame, in this exact order (field inches):")
    for i, p in enumerate(FIELD_POINTS_IN):
        print(f"   {i}: field ({p[0]:.1f} in, {p[1]:.1f} in)")
    print("press r to restart the 4 clicks, c to confirm once all 4 look right")

    while True:
        cv2.imshow(CALIB_WINDOW, draw_calib(display_frame))
        key = cv2.waitKey(20) & 0xFF

        if key == ord("r"):
            clicked_points.clear()
            print("cleared - click the 4 points again")

        elif key == ord("c") and len(clicked_points) == 4:
            cv2.destroyWindow(CALIB_WINDOW)
            src = np.float32(clicked_points)
            dst = np.float32(FIELD_POINTS_IN)
            homography, _ = cv2.findHomography(src, dst)
            return homography

        elif key in (ord("q"), 27):
            cv2.destroyWindow(CALIB_WINDOW)
            print("quit before confirming - nothing processed")
            sys.exit(0)


def tag_field_pose(corners, homography):
    pts = corners.reshape(4, 2).astype(np.float32).reshape(-1, 1, 2)
    transformed = cv2.perspectiveTransform(pts, homography).reshape(4, 2)

    center = transformed.mean(axis=0)
    top_edge = transformed[1] - transformed[0]
    heading = HEADING_SIGN * np.degrees(np.arctan2(top_edge[1], top_edge[0]))

    return center[0], center[1], heading


def build_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    params.errorCorrectionRate = 1.0
    params.polygonalApproxAccuracyRate = 0.06
    return cv2.aruco.ArucoDetector(dictionary, params)


def precompute_track(video_path, homography):
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"could not reopen {video_path} for the precompute pass")

    detector = build_detector()
    track = {}
    frame_idx = 0
    found_count = 0

    ok, frame = cap.read()
    while ok:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            for tag_corners, tag_id in zip(corners, ids.ravel()):
                if tag_id != TARGET_TAG_ID:
                    continue
                x_in, y_in, heading = tag_field_pose(tag_corners, homography)
                track[frame_idx] = (x_in, y_in, heading)
                found_count += 1
                break

        if frame_idx % 100 == 0:
            print(f"precompute: frame {frame_idx}  found so far: {found_count}")

        frame_idx += 1
        ok, frame = cap.read()

    cap.release()
    print(f"precompute done: {frame_idx} frames, tag found in {found_count} "
          f"({100.0 * found_count / frame_idx:.1f}%)")
    return track


def save_track_csv(csv_path, track, fps):
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["frame", "time_s", "found", "x_in", "y_in", "heading_deg"])
        for frame_idx in sorted(track):
            x_in, y_in, heading = track[frame_idx]
            time_s = frame_idx / fps
            writer.writerow([frame_idx, f"{time_s:.4f}", 1,
                              f"{x_in:.3f}", f"{y_in:.3f}", f"{heading:.2f}"])


def load_track_csv(csv_path):
    track = {}
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["found"] != "1":
                continue
            track[int(row["frame"])] = (float(row["x_in"]), float(row["y_in"]), float(row["heading_deg"]))
    return track


def project_pose_to_px(inv_homography, x_in, y_in, heading_deg, scale):
    angle = np.radians(heading_deg)
    tip_x = x_in + HEADING_TICK_IN * np.cos(angle)
    tip_y = y_in + HEADING_TICK_IN * np.sin(angle)
    pts = np.float32([[x_in, y_in], [tip_x, tip_y]]).reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(pts, inv_homography).reshape(-1, 2) * scale
    return safe_point(projected[0]), safe_point(projected[1])


def safe_point(pt):
    x = float(np.clip(pt[0], -1e5, 1e5))
    y = float(np.clip(pt[1], -1e5, 1e5))
    return int(x), int(y)


def build_axis_overlay_px(inv_homography, scale):
    tick_values = np.arange(TICK_INTERVAL_IN, AXIS_LEN_IN + 1e-6, TICK_INTERVAL_IN)
    n = len(tick_values)

    origin = np.float32([[0.0, 0.0]])
    x_axis = np.float32([[v, 0.0] for v in tick_values])
    y_axis = np.float32([[0.0, v] for v in tick_values])
    pts = np.vstack([origin, x_axis, y_axis]).reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(pts, inv_homography).reshape(-1, 2) * scale

    origin_px = safe_point(projected[0])
    x_ticks_px = [safe_point(p) for p in projected[1:1 + n]]
    y_ticks_px = [safe_point(p) for p in projected[1 + n:1 + 2 * n]]
    return origin_px, x_ticks_px, y_ticks_px, tick_values


def draw_axis_overlay(canvas, inv_homography, scale):
    origin_px, x_ticks_px, y_ticks_px, tick_values = build_axis_overlay_px(inv_homography, scale)

    cv2.arrowedLine(canvas, origin_px, x_ticks_px[-1], (255, 0, 0), 2, tipLength=0.08)
    cv2.arrowedLine(canvas, origin_px, y_ticks_px[-1], (0, 128, 255), 2, tipLength=0.08)

    for val, p in zip(tick_values, x_ticks_px):
        cv2.circle(canvas, p, 3, (255, 0, 0), -1)
        cv2.putText(canvas, f"{val:.0f}", (p[0] + 5, p[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)
    for val, p in zip(tick_values, y_ticks_px):
        cv2.circle(canvas, p, 3, (0, 128, 255), -1)
        cv2.putText(canvas, f"{val:.0f}", (p[0] + 5, p[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 128, 255), 1)

    cv2.putText(canvas, "X+", x_ticks_px[-1], cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    cv2.putText(canvas, "Y+", y_ticks_px[-1], cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 255), 2)
    cv2.circle(canvas, origin_px, 5, (0, 255, 0), -1)


class Stepper:
    def __init__(self, cap, total_frames, fps, homography, track):
        self.cap = cap
        self.total_frames = total_frames
        self.fps = fps
        self.homography = homography
        self.inv_homography = np.linalg.inv(homography)
        self.track = track

        self.detector = build_detector()
        self.live_debug = False

        self._frame_cache = OrderedDict()
        self._next_read_idx = 0

        first_frame = self._read_raw(0)
        h, w = first_frame.shape[:2]
        self.display_scale = compute_display_scale(w, h)

        self.frame_idx = 0
        self._updating_trackbar = False

    def _read_raw(self, idx):
        cached = self._frame_cache.get(idx)
        if cached is not None:
            self._frame_cache.move_to_end(idx)
            return cached

        if idx != self._next_read_idx:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, idx)

        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError(f"could not read frame {idx}")
        self._next_read_idx = idx + 1

        self._frame_cache[idx] = frame
        if len(self._frame_cache) > FRAME_CACHE_SIZE:
            self._frame_cache.popitem(last=False)

        return frame

    def _clamp(self, idx):
        return max(0, min(idx, self.total_frames - 1))

    def render(self, idx):
        idx = self._clamp(idx)
        self.frame_idx = idx
        frame = self._read_raw(idx)

        s = self.display_scale
        if s != 1.0:
            dw, dh = int(frame.shape[1] * s), int(frame.shape[0] * s)
            canvas = cv2.resize(frame, (dw, dh), interpolation=cv2.INTER_AREA)
        else:
            canvas = frame.copy()

        draw_axis_overlay(canvas, self.inv_homography, s)

        rejected_count = None
        if self.live_debug:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.detector.detectMarkers(gray)
            rejected_count = len(rejected) if rejected is not None else 0

            if rejected is not None:
                for r in rejected:
                    pts = (r.reshape(-1, 2) * s).astype(np.int32)
                    cv2.polylines(canvas, [pts], True, (0, 255, 255), 1)

            if ids is not None:
                for tag_corners, tag_id in zip(corners, ids.ravel()):
                    if tag_id != TARGET_TAG_ID:
                        continue
                    pts4 = (tag_corners.reshape(4, 2) * s).astype(np.int32)
                    cv2.polylines(canvas, [pts4], True, (0, 255, 0), 1)
                    break

        pose = self.track.get(idx)
        found = pose is not None
        if found:
            x_in, y_in, heading = pose
            center_px, tip_px = project_pose_to_px(self.inv_homography, x_in, y_in, heading, s)
            cv2.line(canvas, center_px, tip_px, (255, 0, 255), 2)
            cv2.circle(canvas, center_px, 6, (0, 0, 255), -1)
            label = f"x={x_in:.2f} y={y_in:.2f} th={heading:.1f}deg"
            cv2.putText(canvas, label, (center_px[0] + 12, center_px[1] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        time_s = idx / self.fps
        status = "FOUND" if found else "NOT DETECTED"
        header = f"frame {idx}/{self.total_frames - 1}   t={time_s:6.2f}s   {status}   cached"
        if self.live_debug:
            header = (f"frame {idx}/{self.total_frames - 1}   t={time_s:6.2f}s   {status}   "
                      f"LIVE DEBUG - rejected_candidates={rejected_count}")
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(canvas, header, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow(STEP_WINDOW, canvas)
        force_repaint(STEP_WINDOW)

        if not self._updating_trackbar:
            self._updating_trackbar = True
            cv2.setTrackbarPos(TRACKBAR, STEP_WINDOW, idx)
            self._updating_trackbar = False

    def on_trackbar(self, pos):
        if self._updating_trackbar:
            return
        self.render(pos)

    def _step_horizontal(self, direction):
        step = 5 if ctrl_held() else 1
        self.render(self.frame_idx + direction * step)

    def _poll_repeat(self, vk, step_fn, held_state, now):
        st = held_state[vk]
        held = key_held(vk)

        if held and not st["down"]:
            step_fn()
            st["down"] = True
            st["next_time"] = now + REPEAT_INITIAL_DELAY_S
        elif held and st["down"] and now >= st["next_time"]:
            step_fn()
            st["next_time"] = now + REPEAT_INTERVAL_S
        elif not held:
            st["down"] = False

    def run(self):
        cv2.namedWindow(STEP_WINDOW, cv2.WINDOW_NORMAL)
        cv2.createTrackbar(TRACKBAR, STEP_WINDOW, 0, max(0, self.total_frames - 1), self.on_trackbar)

        print("controls: left/right = +-1 frame, ctrl+left/right = +-5, up/down = +-30, "
              "hold any of them to auto-repeat, drag the bar to seek, d = toggle live-detect "
              "debug overlay, q/ESC to quit")
        print("positions/headings are read from the cached track.csv by default (fast); "
              "toggle live debug with 'd' to see the actual detector output (incl. rejected "
              "candidates) for the frame you're on")
        if _user32 is None:
            print("warning: key-hold/ctrl detection needs Windows - only single presses will work here")

        self.render(0)

        held_state = {vk: {"down": False, "next_time": 0.0} for vk in (VK_LEFT, VK_RIGHT, VK_UP, VK_DOWN)}

        while True:
            key = cv2.waitKeyEx(15)
            if key in (27, ord("q"), ord("Q")):
                break

            if key in (ord("d"), ord("D")):
                self.live_debug = not self.live_debug
                print(f"live debug: {'ON' if self.live_debug else 'OFF'}")
                self.render(self.frame_idx)
                continue

            if _user32 is not None:
                now = time.monotonic()
                self._poll_repeat(VK_RIGHT, lambda: self._step_horizontal(1), held_state, now)
                self._poll_repeat(VK_LEFT, lambda: self._step_horizontal(-1), held_state, now)
                self._poll_repeat(VK_UP, lambda: self.render(self.frame_idx + 30), held_state, now)
                self._poll_repeat(VK_DOWN, lambda: self.render(self.frame_idx - 30), held_state, now)
            elif key != -1:
                extended = key & 0xFFFF0000
                if extended == VK_RIGHT << 16:
                    self._step_horizontal(1)
                elif extended == VK_LEFT << 16:
                    self._step_horizontal(-1)
                elif extended == VK_UP << 16:
                    self.render(self.frame_idx + 30)
                elif extended == VK_DOWN << 16:
                    self.render(self.frame_idx - 30)

        cv2.destroyAllWindows()


def main():
    if len(sys.argv) < 2:
        print("usage: frame_stepper.py path/to/video.mp4 [path/to/saved_homography.npy]")
        sys.exit(1)

    video_path = Path(sys.argv[1])
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"could not open {video_path}")
        sys.exit(1)

    if len(sys.argv) >= 3:
        homography_path = Path(sys.argv[2])
        if not homography_path.exists():
            print(f"homography file not found: {homography_path}")
            sys.exit(1)
        homography = np.load(homography_path)
        print(f"loaded homography from {homography_path} - skipping calibration")
    else:
        ok, first_frame = cap.read()
        if not ok:
            print(f"could not read a first frame from {video_path}")
            sys.exit(1)
        homography = calibrate_on_first_frame(first_frame)
        homography_path = video_path.with_name(video_path.stem + "_homography.npy")
        np.save(homography_path, homography)
        print(f"saved homography to {homography_path}")
        print("camera hasn't moved between recordings? pass this path as the 2nd "
              "argument on your next video to skip re-clicking:")
        print(f"    frame_stepper.py path/to/other_video.mp4 {homography_path}")

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if total_frames <= 0:
        print("video didn't report a frame count - can't build the seek bar")
        sys.exit(1)

    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps <= 0:
        fps = 30.0
        print(f"video didn't report a frame rate - assuming {fps} fps")

    track_csv_path = video_path.with_name(video_path.stem + "_track.csv")
    if track_csv_path.exists():
        track = load_track_csv(track_csv_path)
        print(f"loaded cached track from {track_csv_path} ({len(track)} frames with a detection)")
        print("delete that file and rerun to force a recompute (e.g. after changing detector settings)")
    else:
        print("precomputing tag detections for the whole video (one-time pass)...")
        track = precompute_track(video_path, homography)
        save_track_csv(track_csv_path, track, fps)
        print(f"saved {len(track)} detections to {track_csv_path} "
              f"(only-found rows - usable directly by 08_visualize_track.py for path drawing)")

    stepper = Stepper(cap, total_frames, fps, homography, track)
    stepper.run()
    cap.release()


if __name__ == "__main__":
    main()
