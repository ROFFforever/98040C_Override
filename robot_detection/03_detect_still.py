import sys
from pathlib import Path

import cv2
import numpy as np

TAGS_DIR = Path(__file__).parent / "tags"
CORNER_NAMES = ("TL", "TR", "BR", "BL")
CORNER_COLORS = ((0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255))


def tag_center(pts):
    return pts.mean(axis=0)


def tag_heading_degrees(pts):
    top_edge = pts[1] - pts[0]
    return np.degrees(np.arctan2(top_edge[1], top_edge[0]))


def tag_pixel_size(pts):
    return np.linalg.norm(pts[1] - pts[0])


def build_scene():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tag = cv2.aruco.generateImageMarker(dictionary, 3, 240)

    padded = np.full((320, 320), 255, dtype=np.uint8)
    padded[40:280, 40:280] = tag

    src = np.float32([[0, 0], [320, 0], [320, 320], [0, 320]])
    dst = np.float32([[120, 90], [520, 40], [560, 400], [90, 430]])
    warp = cv2.getPerspectiveTransform(src, dst)

    scene = cv2.warpPerspective(
        padded, warp, (700, 500),
        borderMode=cv2.BORDER_CONSTANT, borderValue=200,
    )
    return cv2.cvtColor(scene, cv2.COLOR_GRAY2BGR)


def load_image():
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
        image = cv2.imread(str(path))
        if image is None:
            print(f"could not read {path}")
            sys.exit(1)
        return image, str(path)

    default = TAGS_DIR / "tag36h11_id0_4in.png"
    if default.exists():
        return cv2.imread(str(default)), str(default)

    return build_scene(), "built-in synthetic scene (run 02_make_tag.py for real tags)"


def fit_to_screen(image, max_side=900):
    scale = max_side / max(image.shape[:2])
    if scale >= 1:
        return image
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)


def main():
    image, source = load_image()
    image = fit_to_screen(image)
    print(f"source: {source}")
    print(f"image shape: {image.shape}")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

    corners, ids, rejected = detector.detectMarkers(image)

    if ids is None:
        print("no tags found")
        print(f"({len(rejected)} shapes were considered and thrown out)")
        cv2.imshow("no detection", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    print(f"found {len(ids)} tag(s)\n")

    overlay = image.copy()

    for tag_corners, tag_id in zip(corners, ids.ravel()):
        pts = tag_corners.reshape(4, 2)

        center = tag_center(pts)
        heading = tag_heading_degrees(pts)
        size_px = tag_pixel_size(pts)

        print(f"id {tag_id}")
        for name, (x, y) in zip(CORNER_NAMES, pts):
            print(f"   {name}  x={x:7.1f}  y={y:7.1f}")
        print(f"   center  x={center[0]:7.1f}  y={center[1]:7.1f}")
        print(f"   heading {heading:6.1f} deg   top edge = {size_px:.1f} px long")
        print()

        cv2.polylines(overlay, [pts.astype(np.int32)], True, (0, 255, 0), 2)

        for name, color, (x, y) in zip(CORNER_NAMES, CORNER_COLORS, pts):
            cv2.circle(overlay, (int(x), int(y)), 6, color, -1)
            cv2.putText(overlay, name, (int(x) + 8, int(y) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.circle(overlay, center.astype(int), 4, (255, 255, 255), -1)

        forward = pts[1] - pts[0]
        forward = forward / np.linalg.norm(forward) * (size_px * 0.6)
        tip = center + forward
        cv2.arrowedLine(overlay, center.astype(int), tip.astype(int),
                        (255, 0, 255), 2, tipLength=0.25)

        cv2.putText(overlay, f"id {tag_id}  {heading:.1f}deg",
                    (int(center[0]) - 60, int(center[1]) + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv2.imshow("detection (press any key to close)", overlay)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
