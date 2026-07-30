import sys
from pathlib import Path

import cv2
import numpy as np

HOMOGRAPHY_PATH = Path(__file__).parent / "homography.npy"

# Real-world field coordinates (inches) of the 4 points you are about to click,
# IN THE SAME ORDER you click them. Defaults to the 4 corners of a single
# 24x24 in field tile - edit these to match whatever 4 points you actually
# pick in the photo (tile corners are the easiest choice since you already
# know they're exactly 24 in apart).
FIELD_POINTS_IN = [
    (0.0, 0.0),
    (24.0, 0.0),
    (24.0, 24.0),
    (0.0, 24.0),
]

WINDOW = "click the 4 points in order - r resets, q/ESC saves"

clicked_points = []
test_points = []
homography = None


def load_image():
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
        image = cv2.imread(str(path))
        if image is None:
            print(f"could not read {path}")
            sys.exit(1)
        return image

    print("no image given - grabbing one frame from the webcam instead")
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("usage: 05_homography.py path/to/overhead_photo.jpg")
        print("(or plug in the webcam so a frame can be grabbed automatically)")
        sys.exit(1)
    ok, frame = cap.read()
    cap.release()
    if not ok:
        print("could not grab a frame")
        sys.exit(1)
    return frame


def on_click(event, x, y, flags, userdata):
    global homography
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    if homography is None:
        if len(clicked_points) < 4:
            clicked_points.append((float(x), float(y)))
        return

    pixel = np.float32([[[x, y]]])
    field = cv2.perspectiveTransform(pixel, homography)[0, 0]
    print(f"pixel ({x}, {y})  ->  field ({field[0]:.2f} in, {field[1]:.2f} in)")
    test_points.append(((x, y), field))


def draw(image):
    canvas = image.copy()

    for i, (x, y) in enumerate(clicked_points):
        cv2.circle(canvas, (int(x), int(y)), 6, (0, 255, 0), -1)
        cv2.putText(canvas, str(i), (int(x) + 10, int(y) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    for (x, y), field in test_points:
        cv2.circle(canvas, (int(x), int(y)), 6, (0, 255, 255), -1)
        label = f"({field[0]:.1f}, {field[1]:.1f})in"
        cv2.putText(canvas, label, (int(x) + 10, int(y) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    if homography is not None:
        cv2.putText(canvas, "test mode - click anywhere to check its field coords",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return canvas


def main():
    global homography

    image = load_image()
    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, on_click)

    print("click these 4 points in the image, in this exact order:")
    for i, p in enumerate(FIELD_POINTS_IN):
        print(f"   {i}: field ({p[0]:.1f} in, {p[1]:.1f} in)")
    print("press r to restart the 4 clicks, q or ESC once you're done testing")

    while True:
        if len(clicked_points) == 4 and homography is None:
            src = np.float32(clicked_points)
            dst = np.float32(FIELD_POINTS_IN)
            homography, _ = cv2.findHomography(src, dst)
            np.save(HOMOGRAPHY_PATH, homography)
            print(f"\nsaved homography to {HOMOGRAPHY_PATH}")
            print("click a 5th known point (a different tile corner) to sanity-check it")

        cv2.imshow(WINDOW, draw(image))
        key = cv2.waitKey(20) & 0xFF

        if key == ord("r"):
            clicked_points.clear()
            test_points.clear()
            homography = None
            print("\ncleared - click the 4 points again")

        elif key in (ord("q"), 27):
            break

    cv2.destroyAllWindows()

    if homography is None:
        print("quit before 4 points were clicked - nothing saved")


if __name__ == "__main__":
    main()
