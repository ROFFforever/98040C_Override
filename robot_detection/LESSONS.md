# Robot Detection — Lessons

Goal: get a camera to tell you where the robot *actually* is, so you can compare
that against where odometry *thinks* it is.

This folder teaches that in seven steps. Do them in order. Each one runs and shows
you something.

---

## Setup (already done)

A virtual environment lives in `.venv/` with OpenCV 5.0 and numpy installed.

A virtual environment is a private folder of libraries for just this project.
Java analogy: it is this project's own `lib/` folder of jars, instead of dumping
every jar into one shared system-wide folder where versions collide.

Run everything with the venv's python:

```
cd robot_detection
.venv/Scripts/python.exe 01_images_are_numbers.py
```

(If you'd rather not type that every time: `.venv\Scripts\activate` in PowerShell,
then plain `python` works until you close the terminal.)

---

## Lesson 1 — An image is just a grid of numbers

```
.venv/Scripts/python.exe 01_images_are_numbers.py
```

**This is the single most important idea in the whole project.** There is no
special "Image" object with magic inside. An image is a 3-dimensional array of
small integers, and every OpenCV function is just math on that array.

```python
img = np.zeros((300, 400, 3), dtype=np.uint8)
```

Java mental model:

```java
int[][][] img = new int[300][400][3];
```

- **300** = height, the number of rows
- **400** = width, the number of columns
- **3** = the three color channels for each pixel
- `uint8` = each number is 0–255 (an unsigned byte). Java's `byte` is signed
  (−128..127), which is why image code in Java is annoying and image code in
  numpy is not.

### Rows come first

`img[y, x]` — **row first, then column**. This trips up everyone, because when
you *draw*, OpenCV wants `(x, y)`:

```python
img[150, 200] = (255, 255, 255)         # row 150, column 200
cv2.circle(img, (300, 100), 45, ...)    # x=300, y=100
```

Indexing the array = `(row, col)` = `(y, x)`.
Passing a point to a drawing function = `(x, y)`.

Annoying, but consistent once you know it. Just remember: **square brackets are
`[y, x]`, parentheses are `(x, y)`.**

### The origin is top-left, and y grows DOWNWARD

Not like a math graph. Row 0 is the top of the image. This matters later for
angles.

### Colors are BGR, not RGB

Run the script and look at the printed values against the colored bars. `(255, 0, 0)`
is **blue**, not red. This is a historical quirk of OpenCV that you will never be
allowed to forget. Every time a color comes out wrong, this is why.

### Slicing

```python
img[20:80, 20:180] = (255, 0, 0)
```

That fills rows 20–79 and columns 20–179 in one line. In Java you'd write a
double `for` loop. numpy lets you address a whole rectangle at once, and it runs
at C speed rather than Python speed. This is why numpy exists.

### Grayscale

```python
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
```

Prints as shape `(300, 400)` — the 3 disappeared. One brightness number per pixel
instead of three colors. **Tag detection only ever looks at brightness**, so
grayscale is what actually gets processed. Working in grayscale is also ~3× less
data, which is why Lesson 4 converts before detecting.

---

## Lesson 2 — Make a tag you can print

```
.venv/Scripts/python.exe 02_make_tag.py
```

Writes six tags into `tags/`. Print **id 0** to start.

### What is an AprilTag?

A black-and-white square that encodes a small ID number, designed so a computer
can find it fast and never confuse one for another. Think of it as a chunky,
low-resolution QR code built for being seen from far away by a moving camera.

Why it beats tracking a colored blob:

| | Colored blob | AprilTag |
|---|---|---|
| Survives lighting changes | poorly | yes |
| Gives you **heading** | no | yes, free |
| Tells robots apart | no | yes, by ID |
| False positives | often | essentially never |

That "gives you heading free" row is the reason we use it. A blob gives you a
position. A tag gives you a full pose — x, y, **and** which way the robot faces.

### What "36h11" means

The dictionary name. **36** = 36 data bits (a 6×6 grid of black/white squares
inside the border). **h11** = any two valid tags in this family differ by at least
11 bits.

That second number is the important one. To mistake one tag for another, the
camera would have to misread 11 separate squares in exactly the wrong way. That's
why false detections basically don't happen. 36h11 is the standard choice and
what FRC uses on their real fields.

### The white border is not decoration

The script pads every tag with a white margin (a "quiet zone"). The detector finds
tags by hunting for a black square against a lighter background. No white margin,
no contrast at the outer edge, no detection. **Never crop it off.**

### Printing rules

1. **100% / "Actual size"** — never "fit to page". Scaling silently ruins your
   measurements later.
2. **Measure the black square with a ruler afterward.** It must be 4.000 in. This
   number becomes real-world ground truth later; if it's wrong, every measurement
   downstream is wrong by the same percentage.
3. **Matte paper.** Glossy reflects ceiling lights into the camera and blows out
   the pattern.
4. Glue it to cardboard or foam board. A curled tag is not a flat square, and all
   the math assumes flat.

---

## Lesson 3 — Detect a tag in a still image

```
.venv/Scripts/python.exe 03_detect_still.py
.venv/Scripts/python.exe 03_detect_still.py path/to/your/photo.jpg
```

With no argument it uses your generated tag. Take a photo of the printed tag with
your phone, drop it in this folder, and pass it as the argument — that is the real
exercise.

### The three lines that do everything

```python
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
detector   = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
corners, ids, rejected = detector.detectMarkers(image)
```

That's the entire detection step. Everything else in the file is drawing and
printing. You build the detector once, then call `detectMarkers` on every image.

Java analogy: `dictionary` is which alphabet to read, `detector` is a reusable
parser object you construct once, `detectMarkers` is `parser.parse(input)`.

### What you get back

- **`ids`** — the ID numbers found, or `None` if nothing was found. **Always check
  for `None` first.** Frames where the tag is blurry or out of view are normal, not
  a bug.
- **`corners`** — for each tag, the 4 corner points **in pixel coordinates**, as
  floats. Sub-pixel precision: you'll see values like `449.5`. That precision is
  exactly what makes ~1 cm accuracy possible later.
- **`rejected`** — square-ish shapes it looked at and threw out. Useful when
  debugging a tag that won't detect.

### Corner order is guaranteed

Always **top-left → top-right → bottom-right → bottom-left**, in the tag's own
frame. That means the order rotates with the tag — corner 0 stays the tag's own
top-left even when the tag is upside down. This is what makes heading possible.

### Center

```python
center = pts.mean(axis=0)
```

Average of the 4 corners. `axis=0` means "average down the rows", giving one
average x and one average y. In Java: sum all four x's over 4, sum all four y's
over 4.

### Heading

```python
top_edge = pts[1] - pts[0]
heading = np.degrees(np.arctan2(top_edge[1], top_edge[0]))
```

Subtracting corner 0 from corner 1 gives the vector *along the tag's top edge*.
Its angle is the tag's rotation.

`arctan2(dy, dx)` is regular `arctan(dy/dx)` with two fixes you want: it never
divides by zero when the edge is vertical, and it returns the full −180°..180°
range instead of collapsing opposite directions together. Plain `arctan` cannot
tell "pointing right" from "pointing left"; `arctan2` can, because it gets the
signs of dy and dx separately.

**Sign warning:** because image y grows *downward*, a positive angle here is
**clockwise** on screen — the opposite of a math class unit circle. You'll fix this
with one negation when you convert to field coordinates.

### Try to break it

That's the actual assignment for this lesson:

- Photograph the tag from a steep angle. Still detects? (The built-in synthetic
  scene is perspective-warped on purpose — it detects at −6.8°.)
- Photograph it in a dim room.
- Photograph it while waving it around. **This one should fail** — motion blur is
  the failure mode that will bite you on the real field.
- Cover one corner with your thumb.
- Print it at 1 inch and back away until it stops detecting. Note the distance.
  That ratio tells you how big your robot tag needs to be for your camera height.

---

## Lesson 4 — Live webcam detection

```
.venv/Scripts/python.exe 04_live_webcam.py
```

Hold the printed tag up to your laptop webcam. Press `q` or `ESC` to quit.
If the camera doesn't open, change `CAMERA_INDEX` at the top to 1 or 2.

Nothing here is new detection-wise. It is Lesson 3 in a `while` loop:

```
grab frame  ->  convert to gray  ->  detectMarkers  ->  draw  ->  show  ->  repeat
```

`cv2.waitKey(1)` is doing double duty — it waits 1 ms for a keypress *and* it is
what actually lets the window repaint. Remove it and you get a frozen gray box.

### What to watch for

This is your intuition-building step for the real thing. Watch the on-screen
numbers and notice:

- **Move the tag fast → detection drops out.** That's motion blur. This is the
  single biggest real-world risk for the field setup, and it's why locking a fast
  shutter speed matters more than lighting brightness.
- **Tilt the tag → heading changes smoothly.** Rotate it 180° and watch the value
  wrap from 180 to −180. You will have to handle that wrap in your comparison code.
- **Back away → the `px` number shrinks.** When it gets below roughly 20–30 px,
  detection becomes unreliable. This is the number that decides your tag size and
  camera height on the field: work out the pixels-per-inch at your planned mounting
  height and make sure the tag stays well above that floor.
- **Angle it steeply → still works.** Good. Your field camera won't be perfectly
  overhead, and it doesn't need to be.

---

## Lesson 5 — homography (pixels → field inches)

```
.venv/Scripts/python.exe 05_homography.py path/to/overhead_photo.jpg
```

No argument grabs one frame from your webcam instead, so you can test this at
your desk before you have a real overhead photo.

Everything so far has given you *pixels*. Pixels are useless for comparing to
odometry, which thinks in inches. Homography is the fix.

### What a homography actually is

A 3×3 matrix that maps "a point in this photo" to "a point on the field", flat
plane to flat plane. It knows nothing about cameras or lenses — it's pure
geometry: *if these 4 points in the photo correspond to these 4 known points in
reality, where does every other point land?*

Java mental model: it's a lookup function you build once —
`fieldPoint = someFunction(pixelPoint)` — except the "function" is one matrix
multiply, so it's also fast enough to call 30 times a second in Lesson 6.

**Why 4 points, always exactly 4?** A flat-to-flat perspective mapping (a
"projective transform") has 8 unknowns to solve for. Each point you click gives
you 2 equations (an x match and a y match), so 4 points × 2 equations = 8
equations for 8 unknowns. Fewer than 4 and the matrix is underdetermined —
`findHomography` can't solve it. More than 4 is allowed too (it'll
least-squares fit), but 4 is the minimum and simplest.

**Why this only works because the field is flat.** Homography assumes
everything you're mapping lies on one plane. The field floor qualifies. The
robot's tag, mounted a few inches up on top of the robot, technically does not
— it's on a *parallel* plane, not the *same* plane. For a camera mounted
straight overhead this error is negligible. For a camera at a shallow angle it
is not: a tag sitting higher off the ground gets pushed sideways in the image
the further it is from directly under the camera, and homography will
misread that sideways push as a position error. Keep the camera as close to
straight-down as your mounting allows, and mount it as high as you reasonably
can — height shrinks this effect because the sideways push is a small angle at
a large distance.

### Using the script

1. Pick 4 points you can identify in the photo *and* whose real-world inch
   coordinates you already know. Field tile corners are the easiest choice —
   VEX tiles are 24×24 in, so adjacent corners are always exactly 24 in apart.
2. Edit `FIELD_POINTS_IN` at the top of the file to match those 4 real-world
   coordinates, in the order you're about to click them. The default assumes
   one tile with its near corner at (0, 0).
3. Click those 4 points in the image window, in that exact order. The moment
   the 4th click lands, the script computes the homography and saves it to
   `homography.npy` — that file is what Lesson 6 loads.
4. It then switches to **test mode**: click a 5th point whose real coordinates
   you also know (a different tile corner), and it prints/draws what the
   homography *thinks* that point's field coordinates are. If it's off by more
   than about half an inch, your 4 original clicks weren't precise enough —
   press `r` and redo them, zooming into the photo first if you can.

`homography.npy` is just a saved numpy array — `np.load` in Lesson 6 reads the
matrix straight back, no re-clicking needed until you physically move the
camera.

### The origin and axes are *your* choice

Whatever you assign to point 0 becomes field-coordinate (0,0), and the field's
"positive x direction" and "positive y direction" are whatever direction your
point order implies. This matters in Lesson 6 when heading needs to agree with
however the robot already defines its own heading — see the note there.

---

## Lesson 6 — video to CSV

```
.venv/Scripts/python.exe 06_video_to_csv.py path/to/video.mp4 [output.csv]
```

Requires `homography.npy` from Lesson 5 to already exist. Runs Lesson 3's
detection loop over every frame of a recorded video instead of one still image,
converts each detection to field inches using the homography, and writes one
row per frame to a CSV.

### Recording the video

An iPhone propped up (or taped) directly above the field works fine as a
proof of concept — this is exactly what the project plan calls for before
buying a dedicated camera. Keep the phone still for the whole recording; if it
moves, the homography from Lesson 5 is now wrong for every frame after that.

### Why transform the corners, not reuse Lesson 3's angle trick

Lesson 3 computed heading straight from pixel coordinates:
`arctan2(top_edge[1], top_edge[0])`. That number is an angle **in the photo**.
A homography can skew, stretch, and rotate space unevenly (that's what
"perspective" means), so an angle measured in the photo does not generally
equal the same angle measured on the field — only straight-on, undistorted
views would agree, and a tilted camera is exactly the case you're using this
for.

The fix: transform the actual corner *points* through the homography first —
which is exact, because points are what homography is defined for — and only
then compute the angle, from the transformed points:

```python
transformed = cv2.perspectiveTransform(pts, homography)   # 4 corners, now in field inches
center = transformed.mean(axis=0)
top_edge = transformed[1] - transformed[0]
heading = np.degrees(np.arctan2(top_edge[1], top_edge[0]))
```

Nice side effect: this also quietly fixes Lesson 3's "clockwise on screen"
sign warning. That warning existed because image-pixel y grows downward. Once
you're working in *field* coordinates, the flip is already baked into however
you defined `FIELD_POINTS_IN` in Lesson 5 — there's nothing left to negate by
hand, unless your `HEADING_SIGN` check below says otherwise.

### Checking `HEADING_SIGN`

The one thing homography can't know is whether your robot's own heading
convention (whatever `drivetrain::getAngle()` returns) turns the same
direction as the field frame you happened to set up when you clicked points in
Lesson 5. Calibrate it once: place the robot, note the heading number this
script prints, rotate the robot exactly 90° **counterclockwise** as seen in the
photo, and check the reading again.

- Increased by ~90 → correct, leave `HEADING_SIGN = 1`.
- Decreased by ~90 (wrapped around) → flip to `HEADING_SIGN = -1`.

### Reading the CSV

```
frame, time_s, found, x_in, y_in, heading_deg
```

`found` is `1` or `0` — every frame gets a row even when nothing was detected,
with the pose columns left blank. **Do not skip writing "not found" rows.**
The whole point of Lesson 7 is lining this up in time against the robot's log,
and a CSV with gaps quietly removed is a CSV where every later timestamp is
wrong. `time_s` comes from `frame_idx / fps`, not the file's embedded
timestamps — some codecs report those inconsistently, and a constant frame
rate is a much safer assumption for a video you recorded yourself.

### Try to break it

- Check the detection rate the script prints at the end. Below ~90%, motion
  blur or tag size is going to hurt your Lesson 7 comparison — go back to
  Lesson 3/4's "try to break it" section and see which failure mode is
  happening.
- Open the CSV in a spreadsheet and skim the `x_in`/`y_in` columns for a
  frame or two of `found=0` in the middle of an otherwise clean run — a couple
  of dropped frames is normal, that's how tag detection behaves in the real
  world, not a sign that something is broken.

---

## Lesson 7 — compare against odometry

```
.venv/Scripts/python.exe 07_compare_odometry.py camera_track.csv robot_log.ndjson
```

This is the plot that's the actual point of this whole project: the camera's
path (ground truth) and the robot's odometry path, on the same axes, plus how
far apart they are over time.

### The robot side isn't sending this yet

The script expects `robot_log.ndjson` to be one JSON object per line, matching
the same style `Telemetry::debug()` already writes — for example:

```json
{"t": 12345, "x": 10.2, "y": 24.6, "heading": 87.3}
```

`t` is `pros::millis()` (milliseconds since the program started), `x`/`y` are
inches, `heading` is degrees. **This line doesn't exist in the C++ code yet.**
Something in `drivetrain::periodic()` (or wherever the pose gets updated each
tick) needs to call `TELEMETRY.send(...)` with that shape, once per tick, the
same way `debug()` already builds and sends its own JSON. Lines missing any of
the four keys — including the existing `{"debug": "..."}` lines — are ignored,
so debug prints and pose lines can safely share the same log file. Capture the
log with `pros terminal` redirected to a file while the robot runs the route
you also filmed.

If you'd rather name the keys differently on the robot side, change the
`T_KEY` / `X_KEY` / `Y_KEY` / `HEADING_KEY` constants at the top of the script
to match — nothing about the parsing logic cares what the keys are called.

### The two clocks don't start at the same moment

The camera's `time_s` starts at 0 when the *video* starts. The robot's `t`
starts at 0 when the *program* starts. There is no shared instant between the
two logs — you have to find one.

The script's answer: look for the moment each log shows the robot **starts
moving** (position changes by more than `--motion-threshold` inches from where
it began), and assume that's the same real-world instant in both logs. Whatever
gap exists between those two timestamps is the offset needed to line the rest
of the run up. This works well as long as the robot is sitting still at the
start of both recordings, which is the natural way to start a test run anyway.
If the auto-detected offset is obviously wrong (say, the recording started
mid-motion), pass `--sync-offset <seconds>` yourself instead — the script
prints its auto-detected offset either way so you can sanity-check it.

### Why heading gets interpolated as a point on a circle, not a number

`np.interp` on raw degrees breaks across the wrap: 179° and −179° are 2° apart
in reality, but naively averaged they come out to 0°, which is nowhere near
either one. The fix (`interpolate_heading` in the script): convert each
heading to a point on the unit circle — `(cos, sin)` — interpolate the x and y
of that point like any ordinary coordinate, then convert back with
`arctan2`. Averaging *points on a circle* instead of *angle numbers* sidesteps
the wraparound entirely, because points don't have a seam the way the numbers
−180/180 do.

### Reading the output

- **Printed summary** — mean/median/max position error in inches, mean/max
  heading error in degrees, and the timestamp where the worst position error
  happened. That last one is what answers the original question from
  `context.txt`: *where* is odometry least trustworthy, not just *how much* on
  average.
- **Top plot** — both paths overlaid in field inches. Divergence you can see
  directly.
- **Middle/bottom plots** — position error and heading error, each against
  time. A steady small error across the whole run points at a calibration
  problem (wrong wheel diameter, wrong tracking-center offset). An error that
  spikes at specific moments points at a specific *event* — a turn, a
  collision, a fast move that blurs the tag on the camera side instead of the
  robot actually being wrong.

### Try to break it

- Deliberately pass a `--sync-offset` that's off by half a second and watch
  the position-error plot get uniformly worse — this is what a *bad sync*
  looks like, as opposed to a *bad odometry* look (uniform vs. localized
  error, see above).
- Run a route with a sharp collision or wheel slip in the middle and see
  whether the error plot actually spikes where you expect.

---

## Lesson 8 — Lessons 5 and 6, combined for real recording sessions

```
.venv/Scripts/python.exe 08_track_video.py path/to/video.mp4 [output.csv]
```

Lessons 5 and 6 as two separate scripts exist to teach homography and tracking
as two separate ideas. In practice, though, you calibrate once per *recording
session* and then immediately track that same video — so this script does both
in one pass: it shows the video's own first frame, you click the 4 points on
it directly (no separate photo needed), and once you press `c` to confirm it
tracks the whole video and writes the CSV, same schema as Lesson 6.

**A homography is only valid for the exact camera position/height/angle/zoom
it was calibrated against** (see the conversation in this project about why —
straight-down cameras still have perspective, and moving the camera even
slightly gives every field point a new pixel location). Since a video doesn't
move mid-recording, one calibration is good for that entire video. But it is
**not** good for a different video shot from a different height or angle. That's
why this script saves the matrix as `<video_stem>_homography.npy` right next
to the video, instead of overwriting one shared `homography.npy` — each
recording session gets its own, so you can't accidentally reuse a stale one
from a different camera setup.

Everything else — `tag_field_pose`, the CSV columns, `HEADING_SIGN` — is
identical to Lesson 6; read that section for the reasoning.
