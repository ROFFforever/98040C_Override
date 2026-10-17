# Field Tool

Field viewer that will grow into the path planner / run comparison tool (plan vs odom vs camera).

Run from the repo root: `python scripts/field_tool/run.py` (needs PySide6 + numpy, which the system Python already has).

## Layout
- `run.py`: entry point
- `app/field_config.py`: field constants, image calibration, colors, smoothing windows
- `app/field_view.py`: the field canvas, plus `field_to_scene` / `scene_to_field`, the only places that convert between field inches and screen coordinates
- `app/run_data.py`: loads a V5-robot-detection run folder (track.csv + robot ndjson + sync from run_info.json)
- `app/kinematics.py`: velocity / acceleration / angular velocity from positions
- `app/run_overlay.py`: draws the runs and robot outlines, point selection, data callout, playback
- `app/keybinds.py`: every keyboard shortcut + the keybinds menu. Add new shortcuts to `BINDS`
- `app/robot_profiles.py`: robot size profiles + their editor
- `app/settings.py`: reads/writes `settings/*.json`
- `app/main_window.py`: window, toolbars, status bar
- `app/planner/`: auton planner
  - `steps.py`: every placeable step (params in C++ argument order, the C++ call, a motion model). Add new movements here
  - `motion.py`: idealized motion models (trapezoid timing, turns, swing arcs)
  - `plan.py`: the plan data + JSON load/save
  - `simulate.py`: pose after each step, timing, warnings
  - `codegen.py`: plan → `.h` / `.cpp`
  - `parse.py`: `.cpp` → plan (reads back the generated shape plus hand edits; unknown lines become Custom code steps)
  - `store.py`: plans live in `src/GeneratedAutons/<name>.cpp` (+ `include/GeneratedAutons/<name>.h`), old json plans migrate on first open
  - `planner.py`: state, undo/redo, placing, autosave
  - `playback.py`: plan → a playable trace, auto start-time alignment
  - `plan_overlay.py` / `plan_panel.py`: field drawing / side panel
- `assets/field.png`: field image (copied from Downloads)
- `autons/`: old-format plans (`<name>.json`). No longer used once a plan has its `.cpp`; safe to delete after that
- `settings/`: created on first save. `keybinds.json` only stores keys you changed; `robot_profiles.json` stores the profiles

## Coordinates
Inches, origin at field center, +X right, +Y up, red alliance on the left. This matches `Information/override_field.png` and the robot's `Pose`. Tile seams are at multiples of 24. The real wall face is at +-70 (the cyan outline). The image draws its walls about 1.6 in further out.

## Runs
Pick a run from the dropdown (lists `V5-robot-detection.git/runs`), or Open folder… for any folder with a `track.csv` and/or `.ndjson` log.
- Orange = robot odom (SD card log, ~100Hz), pink = camera AprilTag (track.csv, 50fps).
- `t` is the robot clock (seconds since program start, same as ndjson `t` / 1000). Camera times are converted with `sync_scale` / `sync_offset_s`. Camera-only runs use video time.
- Frame #: camera points show their exact frame. Robot points show the nearest frame (≈), or "—" if outside the video or the run was never synced.
- Motion data: `v` = velocity along the robot's heading (+ forward, - reversing), `a` = acceleration along the heading, `ω` = angular velocity (+ CCW). These come from a least-squares line fit over a short window around each point (see `field_config.py`), not raw differences, since camera noise would swamp them.

## Playhead, outlines, playback
There's one playhead time. Clicking or stepping a point moves it to that point's time, and playing moves it in real time. Each source draws a robot outline (active profile, white edge = front) at its point closest to the playhead, so odom and camera are always shown at the same instant. A source's outline hides when the playhead is outside its data (e.g. before the robot program started). The data box follows its source during playback. The slider at the bottom scrubs.

## Robot profiles
Robot: dropdown picks the active profile; Edit… adds/edits them. Length/width are the robot's footprint. "Center forward/left of tracking ctr" is where the middle of the robot is relative to the pose point that odom and the camera report, so the outline lines up when the tracking center isn't the middle of the robot.

The profile also holds the planner's robot numbers: track width (for swing arcs) and the `Speed::SLOW/NORMAL/FAST` cruise/accel from `set_speeds_lateral()` in `main.cpp` (defaults are ROBOT_MAIN's). These only drive the drawn paths and time estimates.

## Auton planner (P)
Build an auton out of your existing factories, see it on the field, export C++.
- **Add a step**: press 1-8 (or the buttons). Steps with a field target (move to point, rotate, swing, move forward…) then wait for a click: the point to drive to, the direction to face, or how far along the heading. Shift snaps (1 in / 15°), Alt places it backwards. Move/drive-to-point stay in placing mode so you can click a chain of waypoints; Esc stops. New steps go after the selected one.
- **Edit**: drag the waypoint dots, the start square / its heading diamond, or the selected step's distance/angle handle. Or type exact values in the Step box. Ctrl+Z / Ctrl+Y undo/redo. W turns the selected run point (from a loaded run) into a move-to-point.
- **Styles** (Function → Style):
  - *Lines*: one statement per step in a `void name(drivetrain* chassis)`. Each step is `await(...)` (blocks) or, with Async (Shift+A), `...->schedule()` (keeps going). Waits become `pros::delay()`. Ctrl+G wraps a step in a `new Sequence({...})`; Ctrl+↑/↓ moves steps into/out of it. The sequence itself can be awaited or async.
  - *Sequence*: the whole auton is `Sequence* name(drivetrain* chassis)` returning one `new Sequence({...})`, like `Autons/SoloAWP.cpp`. Async is ignored.
- **Custom code** (8): pasted as-is. In Lines style it's a statement (add your own subsystem calls between steps); inside a Sequence it has to be a `Command*` expression. Add whatever it uses to *Extra params* (e.g. `Lift* lift`) and *Extra includes* (e.g. `Subsystems/Lift.h, Commands/LiftMoveToCommand.h`).
- **Chaining (Min speed)**: Rotate, Face, Swing, Move to point, Drive to point and Move forward have a *Min speed (chain)* box (last argument, 0 = off). Above 0 the motion never slows below it, hands off the moment it crosses its target (or the early exit range), and leaves the motors running into the next motion, like LemLib's `minSpeed`. Turns take it out of 127, moves in in/s (the profile plans to arrive at that speed instead of 0). The planner drops the settle time for these and the path timing uses the faster arrival. Only use it when another motion follows: the last step of an auton should stay at 0. *Turn gate* on Move to point is the `turn_gate` argument (auto = 30°).
- **Warnings**: robot outline past the wall, an async chassis command getting cancelled by the next chassis command (the scheduler's CancelRunning), over the 15 s / 60 s limit. Times are rough estimates from the profile's speeds, not a real sim.
- **Export** (Ctrl+E): writes `include/GeneratedAutons/<name>.h` and `src/GeneratedAutons/<name>.cpp`. Not `src/Autons/`: that's a COLD_DIR, and the cold build only takes its top-level files, so a subfolder there would never compile. Include the header in `main.cpp` and call it from `autonomous()` (Lines: `name(&chassis);`, Sequence: `await(name(&chassis));`). Each file's first line holds a hash. If you edit the file by hand, export asks before overwriting it.
- The plan IS the exported `.cpp`: it autosaves there, and edits made to the file (VS Code, anything) show up in
  the tool within a second (Ctrl+Z undoes a reload). If the file is mid-edit and unreadable, the tool keeps the
  last good plan and shows a warning. The name is the C++ function name. Only files the tool wrote (first line
  `// field_tool plan`) are listed or overwritten - RedHard.cpp etc. are never touched.
- **Play the plan**: while the planner is open, the plan is a third trace ("Plan", blue) on the same playhead as odom/camera. Space, the slider, speed and ←/→ all work on it. Clicking the planned path, or Shift+↑, opens its data box (predicted x/y/θ or v/a/ω). The bottom bar's *Plan starts at* sets where plan t = 0 falls on the run's clock. **Auto** lines the plan's first motion up with the odom's first motion (camera's if there's no odom), or uses 0 with no run loaded. Ctrl+Home starts the plan at the playhead. The playback is the same rough motion model as the time estimate (trapezoid moves, constant-rate turns), not a physics sim. When a later chassis command interrupts an async one, the async motion is cut off where the new one starts.

## Controls
Keybinds… (Ctrl+K) lists and rebinds every key. Defaults:

| Key | Action |
| --- | --- |
| click | select nearest point |
| ← / → | previous / next point (shift = 10 points) |
| ↑ / ↓ | camera / odom point at the same time |
| Esc | deselect + clear playhead |
| Space | play / pause |
| Home / End | go to start / end |
| [ / ] | slower / faster (0.1x – 4x) |
| M | position data / motion data |
| T | time / frame number |
| R / C / B | show robot odom / camera tag / robot outlines |
| F / G / A | fit field / tile grid / axes |
| Ctrl+O / Ctrl+P / Ctrl+K | open run folder / robot profiles / keybinds |
| scroll | zoom |
| middle or right drag, Ctrl + drag | pan |
| P | auton planner |
| 1-8 | add move to point / drive to point / move forward / rotate / face point / swing / wait / custom code |
| W | selected run point → move to point |
| PgUp / PgDown | select previous / next step (or click a step's end dot) |
| Ctrl+↑ / Ctrl+↓ | move step (into / out of sequences at the edges) |
| Del / Ctrl+D | delete / duplicate step |
| Shift+A / Ctrl+G | async / wrap in Sequence (again = unwrap) |
| Ctrl+Z / Ctrl+Y | undo / redo |
| Ctrl+N / Ctrl+E / Ctrl+Shift+C | new plan / export / copy .cpp |
| Esc | stop placing → deselect step → clear run playhead |
| Shift+↑ | planned point at the same time |
| Ctrl+Home | plan starts at the playhead |
