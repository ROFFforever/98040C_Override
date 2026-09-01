# Per-Robot Retune / Reconfigure Checklist

Everything below is hardware- or robot-specific and must be re-checked (and
usually re-measured/re-tuned) whenever this codebase is deployed on a
**different physical robot** — new chassis, new motor cartridges, new sensor
placement, etc. Grouped roughly in the order you'd actually do them.

All of the values called out below live in **`src/main.cpp`** unless
otherwise noted — that file is the one place that wires code to a specific
robot.

---

## 1. Motor & sensor ports

**File:** `src/main.cpp` (top, lines ~17-27)

```cpp
pros::MotorGroup leftMotors({18, 20});
pros::MotorGroup rightMotors({-11, -12});
pros::Motor intake_motor_1(1);
pros::Imu imu(10);
pros::Rotation vertRotation(-16);
pros::Rotation horizRotation(15);
```

- **What to do:** Plug the new robot's motors/sensors into the V5 brain,
  check the port number shown on the brain's screen (or in PROS terminal)
  for each device, and update the numbers here.
- **Negative port number = reversed.** If a motor spins the wrong way, you
  don't need to physically re-wire it — just flip the sign of its port
  number (e.g. `18` → `-18`).
- Any additional motors (more intake stages, lifts, etc.) need their own
  `pros::Motor` declared here and added to whatever subsystem uses them
  (see `intake_motors({...})` below).
- **Piston/pneumatics note:** the `piston` subsystem class already exists
  (`include/Subsystems/piston.h`) but is **not currently instantiated in
  `main.cpp`**. If the new robot has pneumatics, you'll need to add a
  `pros::adi::DigitalOut` for each solenoid port (A-H on the brain/expander)
  and construct a `piston` object plus a `PistonTeleopCommand` binding it to
  a controller button.

---

## 2. Drivetrain physical dimensions

**File:** `src/main.cpp`, line ~42

```cpp
drivetrain chassis(&leftMotors, &rightMotors, &imu, Units::WHEEL_325, 360, ...);
```

- `Units::WHEEL_325` (in `include/Units.h`) is the **drive wheel diameter in
  inches**. If the new robot uses a different wheel size, either add a new
  constant to `Units.h` (following the `WHEEL_325` naming pattern) or pass
  the raw diameter directly.
- The `360` argument is `wheelRPM` — the wheel's **actual output RPM after
  external gearing** (e.g. a 600rpm blue cartridge geared 5:6 externally =
  360 wheel RPM used here). **Note:** the comment on that line currently
  says `// 450 = wheel's actual output rpm after gearing`, but the value
  passed is `360` — double check which is correct for whichever robot you're
  tuning and fix the stale comment/value if they disagree.
- **How to find your wheel RPM:** `(motor cartridge internal RPM) × (external
  gear ratio, driven teeth / driving teeth or vice versa depending on
  reduction)`.
- `Units::CARTRIDGE_RPM = 600.0` (in `Units.h`) assumes **blue (600rpm)
  cartridges** on every drive motor. If the new robot uses green (200rpm) or
  red (100rpm) cartridges, update this constant — it's used in
  `drivetrain::getLeftDistance()/getRightDistance()` to convert motor
  rotations into wheel distance, so getting it wrong silently corrupts all
  distance/odometry math even if you never touch odom wheels.

---

## 3. Odometry (dead wheel) configuration

**File:** `src/main.cpp`, lines ~26-27

```cpp
odom_wheel vert(&vertRotation, 1.25, 2.125);
odom_wheel horiz(&horizRotation, 0.75, 2.125);
```

Constructor is `odom_wheel(rotation_sensor, offset, wheel_diameter)`.

- **`wheel_diameter`** (2.125 here) — the odom pod's tracking wheel
  diameter in inches. Measure the actual wheel you're using (common sizes:
  2", 2.75", 3.25").
- **`offset`** — signed distance in inches from the **tracking center**
  (the point on the robot whose motion you're modeling) to the wheel,
  measured perpendicular to that wheel's rolling direction.
  - For the **vert** (forward-rolling) wheel: positive = mounted **left**
    of center.
  - For the **horiz** (sideways-rolling) wheel: positive = mounted
    **behind** center.
  - **How to measure:** measure from the middle of the robot (or wherever
    you define as your tracking center) straight out to the wheel, in a
    straight line perpendicular to how that wheel rolls. Use a ruler/calipers,
    not CAD guesses, if possible — this directly scales your position
    tracking accuracy.
- If the new robot has **no odom wheel** in one or both directions, pass
  `nullptr` for that `pros::Rotation*` — the code already handles a missing
  dead wheel (see `odom_wheel::get_dist()`/`get_dist_delta()` returning
  `Units::ERROR`, and `drivetrain::update_pos()` bailing out if more than
  one sensor is missing).
- Port numbers/reversal for `vertRotation`/`horizRotation` follow the same
  rule as motors above (negative = reversed).

---

## 4. IMU

**File:** `src/main.cpp`, line ~20

```cpp
pros::Imu imu(10);
```

- Just the port number — update to match wherever the new IMU is plugged
  in.
- No tuning needed beyond port number; `chassis.calibrateIMU()` (called in
  `initialize()`) handles calibration automatically at startup. Just make
  sure the robot is **sitting still** during that ~2 second window (don't
  pick it up right after powering on).

---

## 5. Feedforward constants (kV, kA, kS) — lateral & angular

**File:** `src/main.cpp`, lines ~33-39

```cpp
velocity_feed_forward ff_lateral(0.1495..., 0.0088..., 0.5832...);  // kV, kA, kS
velocity_feed_forward ff_angular(0, 0, 0);  // NOT TUNED — all zero
```

These describe how many volts it takes to drive the robot at a given
velocity/acceleration. They are **specific to one robot's weight, wheel
grip, gearing, and motor count** — always re-tune on a new robot, never
copy values across robots.

- **kV** — volts per (inch/sec) of steady-state velocity.
- **kA** — volts per (inch/sec²) of acceleration.
- **kS** — volts needed just to overcome static friction (the minimum
  "kick" to get the robot moving at all).
- **How to (re-)tune, lateral:**
  1. In `opcontrol()` in `main.cpp`, comment out the normal teleop loop
     content and instead schedule `DriveCharacterize` in `LATERAL_TUNING`
     mode (see the commented-out `DriveCharacterize kav(&chassis,2);` block
     already in `main.cpp` — mode `2` = `Units::LATERAL_TUNING`).
  2. Run it on the actual field/practice surface (friction matters).
     `DriveCharacterize` drives the robot through a fixed power sequence
     (defined in `src/Commands/DriveCharacterize.cpp`, `LATERAL_STAGES`),
     collects velocity/acceleration/voltage samples, fits `V = kS·sign(v) +
     kV·v + kA·a` via least squares, and prints `kS`, `kV`, `kA` plus an
     `r2` (fit quality, closer to 1.0 = better) and `rmse` over the PROS
     terminal/telemetry link.
  3. Copy the printed `kS`, `kV`, `kA` into `ff_lateral(...)` in `main.cpp`
     (note the constructor order is `kV, kA, kS`, but the printed JSON order
     is `kS, kV, kA` — don't transpose them by mistake).
  4. Optionally verify with `FeedForwardTest` (already wired up in
     `opcontrol()`) — it commands a few known velocities open-loop using
     your new constants and reports how close the achieved velocity was to
     target (`steady_state_rmse`). Small error = good fit.
- **How to (re-)tune, angular:** same idea, using `AngularCharacterize`
  (`src/Commands/AngularCharacterize.cpp`) instead — schedule it the same
  way `DriveCharacterize` is scheduled in `MODE = Units::ANGULAR_TUNING`
  (mode `1`), or the standalone `AngularCharacterize` command directly. It
  spins the robot in place through a power sequence and fits the same
  `V = kS·sign(w) + kV·w + kA·α` model for turning. Copy the results into
  `ff_angular(...)`. **This is currently all zeros and must be tuned before
  any autonomous turning (`Rotate` command) will behave well**, since
  `Rotate` feeds its trapezoid profile output straight through
  `ff_angular->update(v, a)`.

---

## 6. PID constants

**File:** `src/main.cpp`, lines ~30-31

```cpp
PID residual_lateral_PID(0,0,0,0,0);
PID angular_pid(0,0,0,0,0);
```

Constructor is `PID(kP, kI, kD, windup_range, max_integral)`. **Both are
currently all zeros — i.e. untuned/disabled.** These act as the "residual"
correction PIDs that clean up whatever error is left over after the
feedforward + trapezoid profile gets the robot close to target (see
`Rotate::execute()` for how `residual_angular_pid` is layered on top of
`ff_angular`).

- **What each does:**
  - `kP` — proportional: output scales directly with current error.
  - `kI` — integral: output accumulates over time to kill steady-state
    error the P term alone can't close. `windup_range` limits how far off
    target the robot can be and still accumulate integral (prevents
    "integral windup" — a huge buildup from a large, sustained early error
    causing overshoot later). `max_integral` caps the accumulated value
    outright.
  - `kD` — derivative: reacts to the rate of change of error, damps
    oscillation/overshoot.
- **How to tune (classic manual process — no auto-tuner in this codebase):**
  1. Start with all gains at 0 (already the case).
  2. Raise `kP` until the robot reliably reaches/oscillates lightly around
     target without being sluggish. Too high = visible oscillation/shaking.
  3. Add `kD` to damp out oscillation from step 2 — should smooth things out
     without the robot moving noticeably slower to settle.
  4. Only add `kI` if there's a persistent small steady-state error that
     `kP`/`kD` alone won't close (e.g. robot always stops ~1° short). Keep
     `windup_range` tight (only accumulate when close to target) and
     `max_integral` small — integral is the easiest term to make things
     worse with.
  5. Re-run and watch actual behavior each time — there's no shortcut, this
     is trial and error on the real robot/field surface.
- Test via the `Rotate` command (angular) for `angular_pid`, or via a
  lateral drive command (e.g. `tank_motion_profile`) for
  `residual_lateral_PID`.

---

## 7. `angular_kS` / `lateral_kS` residual "kick" terms

**File:** declared in `include/Subsystems/drivetrain.h` (`int angular_kS;
int lateral_kS;`), used in `src/Commands/Rotate.cpp` (`angular_kS`).

- **These are currently never assigned a value anywhere in `main.cpp` or
  the `drivetrain` constructors** — they're uninitialized `int` members
  (garbage value, not guaranteed to be 0). `Rotate::execute()` actively
  uses `drive->angular_kS` to add a static-friction "kick" on top of the
  residual PID output once the trapezoid profile finishes and the robot is
  just settling onto the final angle.
- **Action needed on every robot:** explicitly set `chassis.angular_kS =
  <value>;` (and `lateral_kS` if/when something starts using it — currently
  nothing does) somewhere after `chassis` is constructed in `main.cpp`,
  e.g. in `initialize()`. A reasonable starting value is close to the `kS`
  you already found from `DriveCharacterize`/`AngularCharacterize` in
  millivolts (those report in **volts**, so multiply by 1000).
- This is a real gap in the current code, not just a "tune per robot" item
  — flag it if you're setting up a fresh robot and don't want undefined
  behavior here.

---

## 8. Motion params (`angular_slow/normal/fast`, `lateral_slow/normal/fast`)

**File:** private members of `drivetrain` (`include/Subsystems/drivetrain.h`,
line ~45), read via `drivetrain::get_angular_params(Speed)`
(`src/Subsystems/drivetrain.cpp`).

- **These are also never assigned anywhere** — neither `drivetrain`
  constructor sets them, so they currently hold garbage `MotionParams`
  (uninitialized `cruise_vel`, `final_vel`, `accel`; `init_vel` at least
  defaults to `Units::CURRENT_VEL` since that has a default member
  initializer in the `MotionParams` struct itself).
- **Action needed:** decide on actual cruise-velocity/accel numbers for
  SLOW/NORMAL/FAST angular (and presumably lateral, though there's no
  `get_lateral_params` yet — only angular is wired up) motions, and set
  them, e.g. via a new setter or directly after construction in `main.cpp`.
  Reasonable starting point: cruise velocity somewhat below the top speed
  you saw in `DriveCharacterize`'s velocity data, accel similarly below
  what you saw the robot achieve during the ramp-up parts of that test.
- Until this is filled in, any code path that calls
  `chassis.get_angular_params(Speed::...)` (not currently called anywhere
  in the shipped commands, but is present as connective tissue for future
  `Rotate`-style calls that take a `Speed` instead of raw `MotionParams`)
  will hand back meaningless numbers.

---

## 9. Rotate command settle range / timing

**File:** `src/Commands/Rotate.cpp`, lines ~4, ~10

```cpp
const double settle_range_config = degToRad(4.5);
...
this->max_time = max_time == Units::AUTO_TIME ? 1000 : max_time; // default 1 second
```

- `settle_range_config` (4.5°) is how close to the target heading the robot
  must stay for 8 consecutive ticks (`exit_consecutive_counter >= 8`, at
  100Hz = 80ms) before `Rotate` calls itself finished. Tighter = more
  accurate but can time out more if the robot has slop/backlash; looser =
  faster but less precise. Re-check this feels right once feedforward/PID
  above are actually tuned for the new robot — a well-tuned robot can
  usually tolerate a tighter settle range.
- `max_time` default of 1000ms is a safety timeout so autonomous doesn't
  hang forever if the robot physically can't reach target (stuck, etc). Only
  needs revisiting if turns on the new robot are unusually slow (heavier
  robot, lower gearing) and legitimately need more than ~1s to complete —
  the file comment already notes ~790ms is a realistic "safe" full turn
  time on the current robot.

---

## 10. Controller bindings

**File:** `src/main.cpp` (`IntakeTeleopCommand` construction, line ~45) and
`src/Commands/ArcadeDriveCommand.cpp`.

```cpp
IntakeTeleopCommand intakeTeleop(&intake_motors, &controller,
    pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_L1);
```

- Arcade drive is hardcoded to left stick Y (throttle) + right stick X
  (turn) in `ArcadeDriveCommand::execute()`. Change there if a different
  robot/driver wants a different stick layout (e.g. single-stick, tank
  drive).
- Intake in/out buttons (`L2`/`L1` above) and any piston toggle bindings
  (`PistonTeleopCommand`, not currently instantiated in `main.cpp` — see
  §1) are just constructor arguments — swap the
  `pros::E_CONTROLLER_DIGITAL_*` constants to match whatever button layout
  the new drive team wants.

---

## Quick reference — everything in one table

| # | What | Where | Must change every robot? |
|---|------|-------|---------------------------|
| 1 | Motor/sensor port numbers | `main.cpp` top | Yes, always |
| 2 | Drive wheel diameter (`Units::WHEEL_325`) | `Units.h` / `main.cpp` | Yes, if wheel size differs |
| 2 | `wheelRPM` arg to `drivetrain` ctor | `main.cpp` | Yes, gearing-dependent |
| 2 | `Units::CARTRIDGE_RPM` | `Units.h` | Yes, if motor cartridge color differs |
| 3 | Odom wheel diameter + offsets | `main.cpp` | Yes, measure physically |
| 4 | IMU port | `main.cpp` | Yes |
| 5 | `ff_lateral` kV/kA/kS | `main.cpp` | Yes — re-run `DriveCharacterize` |
| 5 | `ff_angular` kV/kA/kS | `main.cpp` | Yes — re-run `AngularCharacterize` (currently all 0) |
| 6 | `residual_lateral_PID`, `angular_pid` | `main.cpp` | Yes — manual tune (currently all 0) |
| 7 | `angular_kS` / `lateral_kS` | not set anywhere | **Currently missing entirely — must add** |
| 8 | `*_slow/normal/fast` motion params | not set anywhere | **Currently missing entirely — must add** |
| 9 | `Rotate` settle range / max_time | `Rotate.cpp` constants | Sanity-check after retuning FF/PID |
| 10 | Controller bindings | `main.cpp` / `ArcadeDriveCommand.cpp` | Only if driver preference differs |

**Suggested tuning order** for a fresh robot: 1 → 2 → 3 → 4 (get sensors
reporting correctly first) → 5 (feedforward, lateral then angular) → 6
(PID) → 7 → 8 → 9 (settle behavior) → 10 (driver preference, do anytime).
