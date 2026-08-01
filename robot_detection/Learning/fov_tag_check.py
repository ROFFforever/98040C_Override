import math

FIELD_SIZE_IN = 144.0  # 12x12 ft VEX field

CAMERA_HEIGHT_IN = 87.0  # 7'3", camera above the floor
H_FOV_DEG = 85.0
V_FOV_DEG = 69.0
H_RES_PX = 1920
V_RES_PX = 1200

TAG_HEIGHT_IN = 9.0  # how far above the floor the tag sits (midpoint of 6-12in)
TAG_SIZE_IN = 4.0    # printed black-square size

RELIABLE_PX = 30.0
MARGINAL_PX = 20.0


def footprint(fov_deg, distance_in):
    return 2 * distance_in * math.tan(math.radians(fov_deg) / 2)


def main():
    h_floor = footprint(H_FOV_DEG, CAMERA_HEIGHT_IN)
    v_floor = footprint(V_FOV_DEG, CAMERA_HEIGHT_IN)

    print(f"floor-level field coverage @ {CAMERA_HEIGHT_IN:.1f} in camera height:")
    print(f"  H: {h_floor:.1f} in", "(covers field)" if h_floor >= FIELD_SIZE_IN
          else f"(SHORT by {FIELD_SIZE_IN - h_floor:.1f} in)")
    print(f"  V: {v_floor:.1f} in", "(covers field)" if v_floor >= FIELD_SIZE_IN
          else f"(SHORT by {FIELD_SIZE_IN - v_floor:.1f} in)")

    tag_distance = CAMERA_HEIGHT_IN - TAG_HEIGHT_IN
    h_at_tag = footprint(H_FOV_DEG, tag_distance)
    v_at_tag = footprint(V_FOV_DEG, tag_distance)
    h_px_per_in = H_RES_PX / h_at_tag
    v_px_per_in = V_RES_PX / v_at_tag

    tag_px_h = TAG_SIZE_IN * h_px_per_in
    tag_px_v = TAG_SIZE_IN * v_px_per_in
    worst = min(tag_px_h, tag_px_v)

    print(f"\ntag legibility @ {tag_distance:.1f} in camera-to-tag distance:")
    print(f"  {h_px_per_in:.2f} px/in (H), {v_px_per_in:.2f} px/in (V)")
    print(f"  {TAG_SIZE_IN:.1f} in tag -> {tag_px_h:.1f} px across (H), {tag_px_v:.1f} px across (V)")

    if worst >= RELIABLE_PX:
        verdict = "comfortable margin above the ~20-30px reliability floor"
    elif worst >= MARGINAL_PX:
        verdict = "inside the 20-30px floor - workable, thin margin"
    else:
        verdict = "BELOW the ~20px floor - expect unreliable detection"
    print(f"  verdict: {verdict}")


if __name__ == "__main__":
    main()
