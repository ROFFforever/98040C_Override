import cv2
import numpy as np
from pathlib import Path

DPI = 300
TAG_INCHES = 4.0
QUIET_MODULES = 2
MODULES_PER_SIDE = 8
TAG_IDS = (0, 1, 2, 3, 4, 5)

OUT_DIR = Path(__file__).parent / "tags"


def make_printable_tag(dictionary, tag_id, dpi, tag_inches, quiet_modules):
    requested_px = int(round(tag_inches * dpi))
    module_px = max(1, requested_px // MODULES_PER_SIDE)
    tag_px = module_px * MODULES_PER_SIDE

    tag = cv2.aruco.generateImageMarker(dictionary, tag_id, tag_px)

    pad = module_px * quiet_modules
    page = np.full((tag_px + 2 * pad, tag_px + 2 * pad), 255, dtype=np.uint8)
    page[pad:pad + tag_px, pad:pad + tag_px] = tag

    return page, tag_px / dpi


def main():
    OUT_DIR.mkdir(exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

    for tag_id in TAG_IDS:
        page, actual_inches = make_printable_tag(
            dictionary, tag_id, DPI, TAG_INCHES, QUIET_MODULES
        )
        path = OUT_DIR / f"tag36h11_id{tag_id}_{TAG_INCHES:g}in.png"
        cv2.imwrite(str(path), page)
        print(f"wrote {path.name}  {page.shape[1]}x{page.shape[0]}px  black square = {actual_inches:.3f} in")

    print()
    print("PRINTING: use 'Actual size' / 100% scale, NOT 'fit to page'.")
    print(f"Then measure the black square with a ruler. It must be {TAG_INCHES:g} inches.")
    print("Print on MATTE paper. Glossy paper reflects light and breaks detection.")


if __name__ == "__main__":
    main()
