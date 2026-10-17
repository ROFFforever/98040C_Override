# RedHard / BlueHard notes

Route in one line: preload -> R' (red Goal), then out to Autonomous Line cross C for one red/yellow pin + the line cup -> stacked on neutral Goal N4, then flip the Bottom toggle to red and stop clear of it.
Files: `include/GeneratedAutons/RedHard.h`, `src/GeneratedAutons/RedHard.cpp`, selector `AUTON_SELECT` in `autonomous()` (`src/main.cpp`: 0 = old test code, 1 = RedHard, 2 = BlueHard).
Verified: `pros make bin/GeneratedAutons/RedHard.cpp.o` and `bin/main.cpp.o` compile. Timing/legality numbers below come from a footprint + trapezoid model replayed from the commands in `RedHard.cpp` itself (scratch scripts, not in the repo). Nothing was run on a robot.

## 0. Layout assumptions (read first)

Frame: inches, origin at center, +X right, +Y up, red on the left. PNG calibrated on the four Toggle bars (3.48 px/in, origin pixel (253, 254)). Goal centers were measured by colour segmentation, not eyeballed (error ~0.5 in).

| Item | Interpretation used | Evidence |
|---|---|---|
| Quadrants | Four triangles: Bottom (Toggle 0,-70), Left (-70,0), Top (0,70), Right (70,0). Each has 2 Goals. | Appendix B "Quadrant"; 4 Toggles |
| Autonomous Line | The DOUBLE diagonal (TL-BR, `y = -x`) plus the Midfield diamond (`|x|+|y| = 22.5`). The single diagonals (`y = x`) only divide the two Quadrants of one side. | Object count: 4 crosses x 5 (Cup + 4 Pins) + 4 vertex Cups + 4 red/blue Pins = 28 = SG7 note. Counting the single-line Cups too gives 36. |
| Red side | `x + y < 0` = Left + Bottom Quadrants (Red 1/2). Blue = Top + Right. | Red Alliance Goals both sit at lower-left |
| Red Goals (3.25 in) | R (-45, -22.4) Left quadrant, R' (-22.7, -44.5) Bottom quadrant | red squares in PNG, measured |
| Blue Goals | (44.4, 22.2) Right, (22.3, 44.5) Top | measured, = red rotated 180 deg within 0.6 in |
| Neutral Goals (5.8 in, start with a yellow/yellow Pin Placed) | N1 (-45, 22.5) Left, N4 (22.4, -44.8) Bottom, N2 (-22.5, 45) Top, N3 (44.8, -22.5) Right; tall center Goal (0,0) | dark squares with a yellow hex; labels 1-4 are the AprilTag ids |
| Crosses | NOT Goals. One Cup in the middle + 4 Pins lying along +-X/+-Y. Petals: left and bottom = red/yellow (red end outside), top and right = blue/yellow. Crosses at (-45,45), (-22.5,22.5), (22.5,-22.5), (45,-45) on the double line. | zoomed PNG; 8 red/yellow + 8 blue/yellow starting Pins = 4 crosses x 2 |
| Yellow Cups | Cup + yellow/yellow Pin: 4 on the single diagonals ((-22.5,-22.5), (-45,-45) are on the red side), 8 at wall trios. 5 Goal Pins + 8 + 4 = 17 yellow/yellow, matches the manual. | counts |
| Wall trios | 3 gray-side-up Cups at ~65.5 in behind each Quadrant Goal, 7.75 in apart (middle one holds a yellow Pin). 8 trios x 3 = 24 gray-up Cups, matches the manual. | counts |
| Loose Pins | None found. Every hexagon in the PNG sits in a Goal or a Cup. | PNG |
| Load Zone tape | corner box `x < -55.6`, `|y| > 43.4` (red), mirrored for blue | measured |
| Sizes used | footprint 16 x 15 in, tracking center in the middle (corner radius 10.97). Goal r = 3.0, Cup r = 1.7, lying Pin = 6.5 in segment, 0.8 in radius. Icons in the PNG are enlarged, true sizes unverified. | |
| Autonomous Line width | PNG shows the tape pair ~1.8 in wide overall. Route never needs to touch it, keep `x + y <= 1.5`. | |

**Disagreements with the supplied layout facts / CLAUDE.md:** (1) the crosses are not Goals, so the old eyeballed Goal list (-46,47), (-22,22)... does not apply; (2) the red-side Autonomous Line is only the double diagonal, so the BL cups are plain red-side objects (nobody can contest them); (3) Figure Q-1 is not in the text, the Quadrant/Goal map above is derived from the PNG and the counts; (4) toggle starting orientation and turn direction are in an Appendix A drawing that is not in the markdown: assumed yellow, one 120 deg step to red.

Other assumptions: Pins/Cups only picked/placed by the front mechanism; reach 8 in beyond the bumper (24 in total length, SG2); the Cup grabber is a high clamp that does not touch floor Pins; the Pin intake is selective (+-2.5 in about its axis) and disabled while a Pin is held.

## 1. Why this route and this order

Placing a Pin on a neutral Goal that already holds the yellow/yellow Pin needs a Cup (a second Pin half cannot nest in the Goal, SC2a-ii), so neutral Goals only take Cup + Pin stacks. R' is empty, so the Preload goes there first. That also frees the Pin slot (SG6) so the line Pin can be picked up.
Order: **preload first** (forced by SG6), **line Pin + line Cup second** (the only contested objects, the earlier we reach them the smaller the chance the ghost robot took one), **toggle last** (own-side, nobody can touch it, and the last interaction leaves the robot parked 6.5 in from the wall, clear of the Toggle for SC4b; flipping early would only add a second pass near the wall).
N4's own yellow/yellow Pin is already Placed, so with the Toggle red it scores 10 whether or not our stack hides it (see section 6), that is the "yellow half in that Quadrant's Goal" for requirement 4.
Chosen cluster is the Bottom quadrant: its two Goals, its Toggle, and Cross C are all within 60 in, and a legal start exists on the bottom wall (the left wall has no 15 in gap between the wall Cups and the Load Zone).

## 2. Timeline (nominal, mechanism pieces beside a chassis move shown with `+`)

Speed: drives use `Speed::NORMAL` {60, 0, 120} in/s, in/s, in/s^2 (ROBOT_TEST). Turns/swings use the angular controller (turn model 0.15 s + angle/4.5 rad/s + 0.08 s settle, well under the 10.6 rad/s ceiling).

| # | t_start | t_end | action | pose after (x, y, heading deg) | preset | why |
|---|---|---|---|---|---|---|
| 0 | 0.00 | 0.00 | `setPose(-41.5, -62, 90)` (or wall-sensor seed) | start: back edge on bottom wall | - | SG1 legal start |
| 1 | 0.00 | 0.64 | swing CW to 44.1, right wheels locked | (-39.7, -57.7, 44.1) | angular | leave the wall without an in-place spin (spin would swing a corner 3 in into the wall) |
| 2 | 0.64 | 1.29 | profile to R' place pose | (-33.2, -51.4, 44.1) | NORMAL | front edge 1.5 in outside the Goal rim |
| 3 | 1.29 | 1.57 | aim at R' | (-33.2, -51.4, 31.0) | angular | fine aim |
| 4 | 1.57 | 1.82 | MECH place_pin hold (preload -> R') | same | - | stationary |
| 5 | 1.82 | 2.19 | back off 2.1 in `+` place_pin settle 0.15 s | (-35.0, -52.5, 31.0) | NORMAL | clear the rim before spinning (corner radius 10.97 vs 14.6 to Goal) |
| 6 | 2.19 | 2.64 | turn to 90 | (-35.0, -52.5, 90.0) | angular | head north |
| 7 | 2.64 | 3.55 | profile north to (-35.0, -33.1) | (-35.0, -33.1, 90.0) | NORMAL | 17.8 in gap between cup Y_a and R' (body 15 in) |
| 8 | 3.55 | 4.13 | turn to 0 | (-35.0, -33.1, 0.0) | angular | head east |
| 9 | 4.13 | 5.47 | profile east to (9.5, -33.1) | (9.5, -33.1, 0.0) | NORMAL | 17.8 in gap between R' and Midfield-vertex cup / cup Y_b |
| 10 | 5.47 | 5.97 | face K | (9.5, -33.1, 69.0) | angular | |
| 11 | 5.97 | 6.37 | profile to K | (10.5, -30.5, 69.0) | NORMAL | cross C pick pose |
| 12 | 6.37 | 6.66 | turn to 55 | (10.5, -30.5, 55.0) | angular | left-petal Pin on the intake axis (1.3 in off, 10.7 in ahead) |
| 13 | 6.66 | 6.96 | MECH pick_pin hold (C left petal) | same | - | |
| 14 | - | - | Branch: Pin held? continue : ghost fallback (section 5) | | | |
| 15 | 6.96 | 7.26 | turn to 35 `+` pick_pin index 0.20 s | (10.5, -30.5, 35.0) | angular | Cup (14.4 in ahead, 0.3 in off axis) on the grabber axis |
| 16 | 7.26 | 7.61 | MECH pick_cup clamp | same | - | |
| 17 | 7.61 | 8.17 | face N4 `+` pick_cup stow 0.25 s | (10.5, -30.5, -50.4) | angular | |
| 18 | 8.17 | 8.73 | profile to N4 place pose | (14.5, -35.4, -50.4) | NORMAL | front edge 1.5 in outside N4's rim, approach from NW |
| 19 | 8.73 | 9.18 | MECH place_stack release (Cup + Pin on N4) | same | - | stationary |
| 20 | 9.18 | 9.75 | back off 6.5 in `+` place_stack retract 0.25 s | (10.4, -30.4, -50.4) | NORMAL | 6.5 (not 3.5) so the rear-left corner clears cross C's bottom petal by 0.9 in when turning |
| 21 | 9.75 | 10.22 | face T | (10.4, -30.4, -112.4) | angular | |
| 22 | 10.22 | 11.27 | profile to T | (0.0, -55.5, -112.4) | NORMAL | |
| 23 | 11.27 | 11.59 | turn to 270 | (0.0, -55.5, 270.0) | angular | face the wall |
| 24 | 11.59 | 12.39 | MECH flip_toggle(red) | same | - | front edge 4.1 in from the Toggle face, wall 6.5 in away |

**Total planned: 12.39 s of 15.0 s** (slack 2.61 s). With the ghost fallback: 14.07 s (slack 0.93 s). With zero mechanism overlap (everything serial): 13.24 s / 14.92 s.
Chassis planned motion time 10.24 s. Chassis timeouts (1.25x each, rounded up to 0.05 s, `budget()` in the code) sum to 13.30 s = 1.299x. Fallback path: +1.21 s planned, +1.55 s timeouts, total ratio 1.297x. Both <= 1.3x.

### Mechanism overlap and CancelRunning
Four settle pieces (0.15 + 0.20 + 0.25 + 0.25 = 0.85 s) run beside a chassis move through `alongside(mech, chassisStep)` = `Parallel`. Everything lives in the one `Sequence` that owns the chassis, so the scheduler never receives a second command that wants the drivetrain: `scheduleNow()` conflict handling (CancelRunning) can only fire for commands scheduled separately, and none are. `Parallel::isFinished()` waits for both children, so a chassis timeout can not cut a mechanism piece short, and a slow mechanism piece just delays the next step instead of being cancelled by it. `MechStep` has no requirements, so it never claims a Subsystem. Confidence on the physical side of this split (can the stow/retract really overlap a move) is medium, see section 7.

## 3. Inventory after every step (SG6: max 1 Pin + 1 Cup)

| After step | Pins held | Cups held | Note |
|---|---|---|---|
| 0 start | 1 (preload red/yellow) | 0 | |
| 4 place_pin | 0 | 0 | preload Placed in R' |
| 13 pick_pin | 1 (C left petal red/yellow) | 0 | only one Pin inside the +-2.5 in window; nearest other Pin is 8.0 in off axis |
| 14 fallback (only if the grab failed) | 0 then 1 (C bottom petal) | 0 | never 2: the first attempt held nothing |
| 16 pick_cup | 1 | 1 (C line Cup, clear side up) | at the limit, intake disabled while a Pin is held |
| 19 place_stack | 0 | 0 | Cup + Pin on N4 |
| 24 flip_toggle | 0 | 0 | |

## 4. Rules touched and how each is met

| Step | Rule | How |
|---|---|---|
| 0 | SG1 | back edge on bottom wall (y=-70), side on tiles; gaps: 7.3 in to cup Y_a, 2.05 in to the first wall Cup, 6.6 in to the Load Zone tape, none touch a Goal/Loader/Toggle; stationary; only the preload touches the robot. Partner must start in the Left quadrant (SG1d). |
| 0 | SG5 | preload = red/yellow Pin |
| all | SG2/SG3 | 16 in body + 8 in reach = 24 in max, height unchanged |
| all | SG7 | red side is `x + y < 0`. Over every pose of the route, including the swept corners during turns and the swing, the largest footprint `x + y` is -4.5 (at K, heading -92), i.e. 3.2 in from the line centre on our side, the far tape edge is ~0.9 in on the other side. The line objects are reached with the 8 in reach, the body never enters the line. Blue run: 4.5 on the other sign, same margin. |
| all | SG8 | route never enters the Midfield: closest footprint point is `|x|+|y| = 25.3` vs 22.5 at the inner diamond edge. Only engagement is touching Autonomous Line objects (cross C Pin and Cup). |
| 18-19 | SG10 | nothing is removed from N4: the stack is lowered onto its Pin from 1.5 in outside the rim; the Cup's lower half nests over the yellow half, it does not pull it out. A failed placement is the SG10 exception (Q&A 3250). |
| all | SG9 | nearest blue Goal is >60 in away. R' is our own colour. |
| 2-4, 18-19 | SC2 | R': 1 Pin half nested in the Goal. N4: original yellow Pin half in one half of the Cup, line Pin half in the other half, one Pin half per Cup half. |
| 24 | SC4 | Toggle flipped as the last action, mechanism retracted in the 0.8 s, robot parked 4.1 in from the face. |
| 24 | SC5a | N4 and R' are Bottom-quadrant Goals, so their yellow halves are Owned when the Bottom Toggle is red. |
| end | SC8 clause 3 | final footprint y >= -63.5, wall at -70: 6.5 in. |
| all | SG4 | nothing leaves the field, nothing is dropped outside a Goal on purpose. |
| all | SG11/SG12/SG13 | not applicable in autonomous / not used. |

## 5. Stress cases

### 5.1 Start-pose error (+-0.5 in x/y, +-1.5 deg) and 5.2 odometry drift (1 % distance + 0.8 deg per 90 deg turned)
Model: true error at an action = start offset (0.71 in) + sum over segments of `d * (0.01 + sin(eps))`, `eps = 1.5 deg + 0.8 deg * turned/90` (all terms aligned, worst case). Turned angle and path length are accumulated from the actual command list (path 122.6 in, 481 deg of turning).

| Scoring action | Path so far (in) / turned (deg) | Worst-case pos error, odometry only | Placement tolerance | Verdict odometry-only |
|---|---|---|---|---|
| place_pin R' | 13.8 / 59 | 1.3 in | +-2.5 in lateral | pass |
| pick_pin (C left) | 82.6 / 291 | 5.7 in | +-2.5 in (intake axis) | FAIL in worst case |
| pick_cup | 82.6 / 311 | 5.7 in | +-2.5 in | FAIL in worst case |
| place_stack N4 | 88.9 / 396 | 6.3 in | +-2.5 in | FAIL in worst case |
| flip_toggle | 122.6 / 481 | 9.9 in | +-4 in | FAIL in worst case |

So the route does **not** survive the stated worst case on odometry alone; 3.0 of the 5.7 in at K comes from the 44.5 in east leg flown with ~3.3 deg of assumed heading error. What the route leans on:
- Start: `main.cpp` seeds the pose with `wallLocalization.set_initial_pose(90, NegXNegY)` (back sensor -> y, left sensor -> x, both look at clear walls at this start). That removes the +-0.5 in term when the sensors answer; heading stays at the alignment error (+-1.5 deg), and falls back to the nominal pose if a sensor misses.
- East leg (heading 0, axis aligned): the background `WallLocalization` (gate 1.8 in + 5 % of travel, max 6 in) can pull x (front sensor, east wall 60 in away, beam clear of cross C) and y (right sensor, south wall). It can only help if it accepts, the gate is wide enough, and the beam is clear. The y beam hits the Toggle face (2.4 in short) for |x| < 13, so y should not be trusted there. Not verified against data, check with the camera runs before relying on it.
- If WallLocalization pulls the error to <= 1.0 in at the end of the east leg, the remaining path K -> N4 is 6 in, adding ~0.5 in, N4 ends ~1.5 in (pass). The Toggle step (31 in more, ~4.5 in with drift) stays marginal against +-4 in, and the front sensor reads the Toggle, not the wall, so there is no y reset there.
- Where it cannot help: the Y_a/R' gap (x from -43.3 to -25.5) is driven open loop (clearance 0.8 in to the Y_a cup, 2.0 in to R'); the cross C poses are diagonal (heading 55/35) so wall beams are oblique; K and N4 place poses are 38-43 in from the nearest wall.
- Pinch points (clearance with Goal r 3.0, Cup r 1.7): gap A 0.8 in to Y_a (a loose Cup, harmless), 2.0 in to R'; east leg 1.4 in to R' and 1.4 in to the Midfield vertex Cup / Y_b. Touching a loose Cup is legal; touching R' could knock the preload off.
- No path comes within 1 in of a wall except the start: back edge starts ON the wall (SG1f requires contact) and the first ~10 deg of the swing keeps that corner within 0.1 in of it (the model says 0.1 in past, in reality the wall stops it); the drive to R' then keeps the lowest corner 1.4 in off the wall; the Toggle pose is 6.5 in off the wall.

### 5.3 Ghost opponent (one Autonomous Line Pin removed at a random time)
Line Pin used: cross C left petal (22.5-8.1 .. 22.5-1.6, -22.5), red/yellow.
- Trigger: after the 0.30 s grab, `pinHeld()` (placeholder for the intake's Pin sensor, true by default; `redhard_force_pin_miss = true` rehearses it) is false. The Branch only exists at the K pose, so the position condition is implicit. It is evaluated when the Branch starts, not when the auton is built.
- Fallback: C bottom petal (also red/yellow, legal, on our side of the line): turn to 25, drive 3.4 in to (12.0, -33.5), full 0.5 s grab, drive back to K, continue. Cost +1.68 s, total 14.07 s, still scores N4 the same. The bottom petal pose has the Pin 1.1 in off axis, 12.1 in ahead.
- If the ghost takes the *bottom* petal instead, the primary grab is unchanged. If it takes a blue petal, no effect. Removing the Cup is not in the scenario; the route has no fallback for a missing C Cup (the stack would have to be skipped).
- The ghost's Pin removal can not contest anything else: R', N4, the Toggle and the BL cups are red-side only (SG7).

### 5.4 Mirror
BlueHard is produced by the same function with `Frame{blue = true}`: `(x, y, heading) -> (-x, -y, heading + 180)`, i.e. a rotation by 180 deg about the center, NOT a reflection. The measured layout is 180-degree symmetric (red Goal R' (-22.7,-44.5) -> (22.7, 44.5) vs measured blue Goal (22.3, 44.5), 0.4 in error; R -> (45.0, 22.4) vs (44.4, 22.2), 0.6 in) and not mirror symmetric (`x -> -x` would send R' onto the neutral Goal N4). Consequences: no angle sign flip and no swing side swap (a rotation keeps handedness, CW stays CW, `DriveSide::RIGHT` stays RIGHT); heading is `+180` and wrapped. Colour swaps: R' becomes the blue alliance Goal, N4 becomes N2 (neutral, (-22.5, 45)), the Bottom Toggle becomes the Top Toggle and `flip_toggle` targets blue, the preload is blue/yellow, line petal C-left (red outer) becomes cross B's right petal (blue outer, checked in the zoomed PNG). Start becomes (41.5, 62, heading -90) on the top wall. Checks run: every blue pose rotated back lands on the red pose; blue footprints keep `x + y >= +4.5` against the allowed `>= -1.5`; the closest Midfield approach is identical. Seeding for blue uses `set_initial_pose(270, PosXPosY)`.

### 5.5 Possession trap
See section 3: the intake is only commanded when `pinHeld()` is false (in nominal flow after the preload is placed), the fallback only runs after a failed grab, and the Cup grabber only touches Cups. Cross C leaves four Pins on the floor, so the window rule matters: at the K pin pose the other three C Pins are 8.0 in (bottom), 17+ in (top, beyond reach), and far right off axis; at the cup pose the bottom petal is 4.3 in off the grabber axis, harmless because the grabber is a clamp and a Pin is already held.

### 5.6 Yellow-half trap: final state of every Pin placed
Bottom Toggle red at the end (flipped last, robot clear), all in Bottom-quadrant Goals.

| Pin | Final state | Points |
|---|---|---|
| Preload red/yellow in R' | one half nested in the Goal, the other half visible. Placed red-up: red half | 5. Placed yellow-up: yellow half Owned | 10 |
| N4 original yellow/yellow Pin | Placed from the start, stays Placed (nothing removed). Upper half sits in the lower half of the Cup | see N4 total |
| C line Pin red/yellow in the Cup on N4 | lower half in the Cup's other half, upper half visible | see N4 total |
| N4 total | A stack of one Cup between two Pins always shows exactly 2 halves: Cup transparent half down -> original yellow (10) + new Pin's upper half; Cup opaque half down -> new Pin's two halves (red 5 + yellow 10). Either way >= 15 (20 if the new Pin goes in yellow-up with the transparent half down). | 15 min |
| Cup | Placed (nested with a Placed Pin), no points | 0 |

Guaranteed on paper: R' 5 + N4 15 = **20 points, 3 Pin halves** (up to 30 with favourable Pin orientation). Yellow halves count only because the Toggle is red and not touched (SC4); if the Toggle ends yellow, those Pins score 5 only (R' red-up) and N4's red half 5.

## 6. Win Point and bars
Pin halves scored for red in red-side Quadrants: R' 1, N4 2 = **3**. Goals with >= 2 halves: **1** (N4).
- Standard bar (6 halves, 2 Goals with >= 2): **not met.**
- Worlds-qualifier bar (7 halves, 3 Goals with >= 2): **not met.**
- Rough lower bound for why: every extra half needs another pick + stack (pick 0.65-1.1 s of mechanism, place 0.45 + 0.25, plus ~2 s of driving in a 22.5 in object lattice with 1.4 in clearances). Standard needs 3 more halves: +9 s or more, i.e. >= 21 s total. Worlds needs 4 more halves plus a third Goal in the other Quadrant, which means the Left Toggle (80 in away) and N1: >= 28 s. With the given mechanism times neither bar fits in 15 s from this start; the route above is the best the geometry allowed that still satisfies all five required steps with margin for the ghost fallback.
- Expected score: 20 points (range 20-30), plus the 12-point Autonomous Bonus if the opponent scores less and neither alliance commits a violation (not in our control).

## 7. Mechanism placeholders and the pose each needs

All are `MechStep` timed no-ops in `RedHard.cpp` (log `mech_step` to telemetry). "Hold" = robot stationary, "settle" = may overlap the next chassis move (assumed split, to be confirmed).

| Step (function) | Time | Pose / heading / approach it needs | Confidence |
|---|---|---|---|
| place_pin: `placePinHold` + `placePinSettle` (preload -> R') | 0.25 + 0.15 | (-33.2, -51.4) heading 31, Goal centre 12.5 in ahead (front edge 1.5 in outside the rim, mechanism reaches over the centre with 4.5 of 8 in reach), red half up | 0.7 |
| pick_pin: `pickPinHold` + `pickPinIndex` (C left petal) | 0.30 + 0.20 | (10.5, -30.5) heading 55, Pin midpoint 10.7 in ahead, 1.3 in off axis, approach from SW (body clear of the bottom petal by 0.9 in) | 0.5 (selectivity, Pin lying flat) |
| pick_pin fallback: `pickPinFull` (C bottom petal) | 0.50 | (12.0, -33.5) heading 25, Pin 12.8 in ahead, 1.1 in off axis | 0.4 |
| pick_cup: `pickCupClamp` + `pickCupStow` (line Cup) | 0.35 + 0.25 | (10.5, -30.5) heading 35, Cup centre 14.4 in ahead (6.4 of 8 in reach), 0.3 in off axis, high clamp over floor Pins | 0.5 |
| place_stack: `placeStackRelease` + `placeStackRetract` (Cup + Pin on N4) | 0.45 + 0.25 | (14.5, -35.4) heading -50.4, Goal centre 12.5 in ahead, from NW; Cup lowered over N4's Pin (either Cup orientation scores >= 15 as shown above) | 0.5 |
| flip_toggle: `flipToggle(blue)` | 0.80 | (0, -55.5) heading 270, face 4.1 in ahead, centred within +-4 in; one 120 deg step to red assumed; must be retracted and clear before the end | 0.4 (direction/steps unknown) |

If the Toggle needs two steps or the stow/retract pieces cannot overlap a move, the nominal total goes to 13.2 s and the ghost fallback to 14.9-15.7 s, the Toggle (last step) is what would be lost.

## 8. Confidence per chassis step (model-based, nothing on a robot yet)

| Steps | Confidence | Main risk |
|---|---|---|
| 1-5 swing, drive, place at R' | 0.75 | swing arc and wall (justified above), Goal r unverified |
| 6-8 gap A and turn east | 0.55 | 0.8 in / 2.0 in clearance, open loop |
| 9 east leg | 0.6 | 1.4 in both sides; relies on heading hold and WallLocalization |
| 10-12 K approach | 0.55 | lattice, 0.9 in to the bottom petal |
| 17-18 N4 approach | 0.7 | 1.5 in to the rim |
| 20-23 back off, toggle approach | 0.65 | 0.9 in to the bottom petal; Toggle face as a sensor target |
| Whole route within 15 s | 0.85 nominal, 0.65 with fallback | stacked timeouts if a turn is slow |

## 9. Top failure modes
1. Open-loop precision in the 22.5 in object lattice: 1.4 in clearances at the east leg and 0.8 in at gap A against a worst-case odometry error of 5.7-6.3 in at the scoring poses; success depends on WallLocalization accepting on the east leg.
2. Mechanism assumptions that nothing on this robot has tested: selective Pin pick from a cross, high-clamp Cup grab, 24 in reach, the placement orientation and the 0.85 s of overlap, Toggle steps; plus the layout reading (Figure Q-1 and the Toggle drawing are not available).
3. Time: the fallback leaves 0.93 s, and the Toggle is deliberately last, so a slow rotate chain loses the 10 point Toggle ownership first. A ghost or partner robot physically blocking the 17.8 in gaps has no avoidance (SG8).
