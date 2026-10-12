from pathlib import Path

ASSETS_DIR = Path(__file__).resolve().parent.parent / "assets"
FIELD_IMAGE = ASSETS_DIR / "field.png"

#run folders made by V5-robot-detection (sits next to 98040C_Override.git)
RUNS_DIR = Path(__file__).resolve().parents[4] / "V5-robot-detection.git" / "runs"

#auton planner: plans (.json) live with the tool, exported code goes into the PROS project.
#Not src/Autons/ - that's a COLD_DIR in the Makefile, and the cold build only picks up its top-level
#files, so a subfolder there would never get compiled. GeneratedAutons is a normal hot dir.
REPO_DIR = Path(__file__).resolve().parents[3]
PLANS_DIR = Path(__file__).resolve().parent.parent / "autons"
EXPORT_SRC_DIR = REPO_DIR / "src" / "GeneratedAutons"
EXPORT_INC_DIR = REPO_DIR / "include" / "GeneratedAutons"

# Field coordinates match the robot code: inches, origin at field center, +X right, +Y up
# (red alliance on the left). Tile seams are at multiples of 24, the inner wall face is at +-70
# (same value WallLocalization and the camera tool use) - the outer tiles run under the walls.
WALL_IN = 70.0
TILE_IN = 24.0

# field.png calibration, measured off the tile-intersection markers on the center diamond
# (2000x2000 image, same on both axes). The image draws its walls at ~+-71.6, so calibrate
# off the tiles, not the walls. Re-measure these if the image ever gets swapped out.
IMAGE_CENTER_PX = 1000.0
IMAGE_PX_PER_TILE = 324.0

ROBOT_COLOR = "#ffb000"
CAMERA_COLOR = "#ff4fd8"
PLAN_COLOR = "#38d6ff"
PLAN_START_COLOR = "#7dff6b"

#velocity/accel come from a least-squares line fit over +-this many seconds around each point.
#Bigger = smoother but laggier. Camera is 50fps, so +-0.05s is ~5 frames.
VELOCITY_WINDOW_S = 0.05
ACCEL_WINDOW_S = 0.10

CLICK_TOLERANCE_PX = 10  #how close (screen pixels) a click has to be to grab a point
