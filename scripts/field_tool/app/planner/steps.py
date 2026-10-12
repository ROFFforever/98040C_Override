"""Every step the auton planner can place. Each one is a factory in the robot code (drivetrain.h etc.):
its params are listed in the SAME ORDER as the C++ arguments, so code generation is just filling them in.

To add a movement: add a StepType at the bottom with its params, the C++ call, the headers it needs,
and a simulate function (where the robot ends up + roughly how long it takes). Add a keybind for it in
keybinds.py ("add_<id>") if you want one."""
import math
from dataclasses import dataclass, field
from typing import Callable

from app.planner.motion import SETTLE_S, Motion, wrap

#drivetrain.cpp: moveToPoint skips its turn when already within this, and its turn exits early at this
MOVE_TO_POINT_SKIP_TURN_DEG = 15.0
MOVE_TO_POINT_TURN_EARLY_EXIT_DEG = 5.0
TURN_SETTLE_DEG = 5.0  #Rotate/Swing default settle range


@dataclass(frozen=True)
class Param:
    name: str
    label: str
    kind: str              #float | int | bool | choice | auto (float, None = the C++ AUTO constant) | text
    default: object = None
    unit: str = ""
    lo: float = -1e6
    hi: float = 1e6
    step: float = 1.0
    decimals: int = 2
    choices: tuple = ()
    cpp_enum: str = ""     #choice -> "<cpp_enum>::<value>"
    cpp_auto: str = ""     #auto -> C++ text used for None
    optional: bool = True  #has a default argument in C++ (can be left off the end of the call)


@dataclass(frozen=True)
class StepType:
    id: str
    label: str
    call: str                   #C++ before the "(", e.g. "chassis->moveToPoint". Empty = special-cased in codegen
    params: tuple
    simulate: Callable          #(start pose (x, y, heading), params dict, robot profile) -> Motion
    describe: Callable          #params dict -> short text for the step list
    includes: tuple = ()        #headers the generated .cpp needs for this step
    handle: str = ""            #how it's placed / dragged on the field: point | distance | angle | "" (form only)
    drivetrain: bool = True     #needs the chassis (for async overlap warnings)
    repeat_place: bool = False  #stay in placing mode after a click, so a chain of points is quick to lay down


def num(v):
    """Number for C++/display: no trailing zeros, max 3 decimals."""
    v = round(float(v), 3)
    if v == int(v):
        return str(int(v))
    return f"{v:.3f}".rstrip("0").rstrip(".")


# ---- shared params

def _early_exit(unit):
    return Param("early_exit", "Early exit range", "float", 0.0, unit, 0, 48, 0.5,
                 decimals=1 if unit == "°" else 2)


MAX_TIME = Param("max_time", "Max time", "auto", None, "s", 0, 30, 0.1, cpp_auto="Units::AUTO_TIME")
SETTLE_IN = Param("settle", "Settle range", "auto", None, "in", 0, 24, 0.25, cpp_auto="Units::AUTO")
SETTLE_DEG = Param("settle", "Settle range", "auto", None, "°", 0, 45, 0.5, decimals=1, cpp_auto="Units::AUTO")
MAX_SPEED = Param("max_speed", "Max speed", "int", 127, "/127", 1, 127, 1, decimals=0)
SPEED = Param("speed", "Speed", "choice", "NORMAL", choices=("SLOW", "NORMAL", "FAST"), cpp_enum="Speed")
BACKWARDS = Param("backwards", "Backwards", "bool", False)
X = Param("x", "x", "float", 0.0, "in", -72, 72, 0.5, optional=False)
Y = Param("y", "y", "float", 0.0, "in", -72, 72, 0.5, optional=False)
TARGET_DEG = Param("target", "Target heading", "float", 0.0, "°", -360, 360, 1, decimals=1, optional=False)


# ---- helpers for the simulate functions

def _turn_rate(profile, p, factor=1.0):
    return profile.turn_rate_dps * factor * min(1.0, p.get("max_speed", 127) / 127.0)


def _check_time(m, p, needed, auto_cap=None):
    if p.get("max_time") is not None and needed > p["max_time"]:
        m.notes.append(f"max time {num(p['max_time'])} s may cut it off (needs ≈{needed:.1f} s)")
    elif p.get("max_time") is None and auto_cap is not None and needed > auto_cap:
        m.notes.append(f"auto timeout ({auto_cap:.2f} s) may cut it off (needs ≈{needed:.1f} s)")


def _turn(m, delta, p, profile, factor=1.0, pivot=None):
    """Rotate/Swing: turn by delta degrees (sign already chosen), honoring early exit + settle + timeouts."""
    early = p.get("early_exit", 0.0)
    eff = max(0.0, abs(delta) - early) * (1 if delta >= 0 else -1)
    needed = abs(eff) / _turn_rate(profile, p, factor) + (0 if early > 0 else SETTLE_S)
    #Rotate.cpp / Swing.cpp: auto max_time = 0.5 + 0.4 s per radian
    _check_time(m, p, needed, 0.5 + 0.4 * math.radians(abs(delta)))
    m.turn(eff, needed - (0 if early > 0 else SETTLE_S), pivot)
    if early <= 0:
        m.hold(SETTLE_S)
    else:
        m.notes.append(f"hands off {num(early)}° early, still turning")


def _drive_to(m, x, y, backwards, vel, accel, p):
    """tank_motion_profile to (x, y): straight line, heading pulled onto the line, early exit stops short."""
    x0, y0, _ = m.pose
    dist = math.hypot(x - x0, y - y0)
    if dist < 0.01:
        m.notes.append("already at the target")
        return
    face = math.degrees(math.atan2(y - y0, x - x0)) + (180 if backwards else 0)
    early = min(p.get("early_exit", 0.0), dist)
    if early > 0:  #exits once the remaining distance is under early_exit, motors still going
        f = (dist - early) / dist
        x, y = x0 + (x - x0) * f, y0 + (y - y0) * f
        m.notes.append(f"hands off {num(early)} in early, still moving")
    t = m.line(x, y, face, vel, accel)
    if early <= 0:
        m.hold(SETTLE_S)
    _check_time(m, p, t + (0 if early > 0 else SETTLE_S))


def _face_angle(pose, x, y, backwards):
    return math.degrees(math.atan2(y - pose[1], x - pose[0])) + (180 if backwards else 0)


# ---- step simulations

def _sim_move_to_point(pose, p, profile):
    m = Motion(pose)
    #drivetrain::moveToPoint = rotate_to_point (skipped under 15°, exits 5° early) then the profiled drive
    turn = wrap(_face_angle(pose, p["x"], p["y"], p["backwards"]) - pose[2])
    if abs(turn) > MOVE_TO_POINT_SKIP_TURN_DEG and math.hypot(p["x"] - pose[0], p["y"] - pose[1]) > 0.01:
        eff = turn - math.copysign(MOVE_TO_POINT_TURN_EARLY_EXIT_DEG, turn)
        m.turn(eff, abs(eff) / profile.turn_rate_dps)
    vel, accel = profile.lateral(p["speed"])
    _drive_to(m, p["x"], p["y"], p["backwards"], vel, accel, p)
    return m


def _sim_drive_to_point(pose, p, profile):
    m = Motion(pose)
    turn = wrap(_face_angle(pose, p["x"], p["y"], p["backwards"]) - pose[2])
    if abs(turn) > 30:
        m.notes.append(f"starts {abs(turn):.0f}° off the line with no turn first - the real path will curve")
    vel, accel = profile.lateral(p["speed"])
    _drive_to(m, p["x"], p["y"], p["backwards"], vel, accel, p)
    return m


def _sim_move_forward(pose, p, profile):
    m = Motion(pose)
    #moveForward keeps its heading: drives to a point `distance` straight ahead (or behind)
    sign = -1 if p["backwards"] else 1
    h = math.radians(pose[2])
    x = pose[0] + sign * p["distance"] * math.cos(h)
    y = pose[1] + sign * p["distance"] * math.sin(h)
    _drive_to(m, x, y, p["backwards"], p["max_speed_in"], profile.normal_accel, p)
    return m


def _sim_rotate(pose, p, profile):
    m = Motion(pose)
    _turn(m, wrap(p["target"] - pose[2]), p, profile)  #Rotate takes the short way around
    return m


def _sim_rotate_to_point(pose, p, profile):
    m = Motion(pose)
    if math.hypot(p["x"] - pose[0], p["y"] - pose[1]) < 0.01:
        m.notes.append("target is on top of the robot")
        return m
    _turn(m, wrap(_face_angle(pose, p["x"], p["y"], p["backwards"]) - pose[2]), p, profile)
    return m


def _sim_swing(pose, p, profile):
    m = Motion(pose)
    #Swing.cpp: locked LEFT turns CCW (right side drives forward), RIGHT turns CW, reverse flips both.
    #It goes the long way around if that's the only way to turn in the forced direction.
    settle = p["settle"] if p["settle"] is not None else TURN_SETTLE_DEG
    err = wrap(p["target"] - pose[2])
    required = (1 if p["side"] == "LEFT" else -1) * (-1 if p["reverse"] else 1)
    if abs(err) > settle:
        if required > 0 and err < 0:
            err += 360
        elif required < 0 and err > 0:
            err -= 360
    if abs(err) > 200:
        m.notes.append(f"swings the long way ({abs(err):.0f}°) - that's the only way in this direction")
    #pivots around the locked side's wheels
    left = math.radians(pose[2] + 90)
    half = profile.track_width_in / 2 * (1 if p["side"] == "LEFT" else -1)
    pivot = (pose[0] + half * math.cos(left), pose[1] + half * math.sin(left))
    _turn(m, err, p, profile, factor=0.5, pivot=pivot)
    return m


def _sim_wait(pose, p, profile):
    m = Motion(pose)
    m.hold(p["ms"] / 1000.0)
    return m


def _sim_code(pose, p, profile):
    m = Motion(pose)
    m.notes.append("custom code - not simulated")
    return m


def _flags(p):
    out = []
    if p.get("backwards"):
        out.append("back")
    if p.get("speed", "NORMAL") != "NORMAL":
        out.append(p["speed"].lower())
    if p.get("early_exit"):
        out.append(f"exit {num(p['early_exit'])}")
    return ("  " + ", ".join(out)) if out else ""


STEP_TYPES = [
    StepType("move_to_point", "Move to point", "chassis->moveToPoint",
             (X, Y, _early_exit("in"), BACKWARDS, SPEED, MAX_TIME, SETTLE_IN),
             _sim_move_to_point,
             lambda p: f"moveToPoint ({num(p['x'])}, {num(p['y'])})" + _flags(p),
             handle="point", repeat_place=True),
    StepType("drive_to_point", "Drive to point (no turn)", "chassis->Tank_motion_profile",
             (X, Y, _early_exit("in"), SPEED, MAX_TIME, SETTLE_IN, BACKWARDS),
             _sim_drive_to_point,
             lambda p: f"driveTo ({num(p['x'])}, {num(p['y'])})" + _flags(p),
             includes=("Commands/tank_motion_profile.hpp",), handle="point", repeat_place=True),
    StepType("move_forward", "Move forward", "chassis->moveForward",
             (Param("distance", "Distance", "float", 24.0, "in", 0, 200, 0.5, optional=False),
              Param("backwards", "Backwards", "bool", False, optional=False),
              Param("max_speed_in", "Max speed", "float", 60.0, "in/s", 1, 200, 1, decimals=1, optional=False),
              _early_exit("in"), MAX_TIME, SETTLE_IN),
             _sim_move_forward,
             lambda p: f"moveForward {num(p['distance'])} in @ {num(p['max_speed_in'])}" + _flags(p),
             includes=("Commands/tank_motion_profile.hpp",), handle="distance"),
    StepType("rotate", "Rotate to heading", "chassis->rotate",
             (TARGET_DEG, _early_exit("°"), MAX_TIME, SETTLE_DEG, MAX_SPEED),
             _sim_rotate,
             lambda p: f"rotate {num(p['target'])}°" + _flags(p),
             includes=("Commands/Rotate.h",), handle="angle"),
    StepType("rotate_to_point", "Rotate to face point", "chassis->rotate_to_point",
             (X, Y, _early_exit("°"), BACKWARDS, MAX_TIME, SETTLE_DEG, MAX_SPEED),
             _sim_rotate_to_point,
             lambda p: f"face ({num(p['x'])}, {num(p['y'])})" + _flags(p),
             includes=("Commands/Rotate.h",), handle="point"),
    StepType("swing", "Swing", "chassis->swing",
             (TARGET_DEG,
              Param("side", "Locked side", "choice", "LEFT", choices=("LEFT", "RIGHT"), cpp_enum="DriveSide",
                    optional=False),
              Param("reverse", "Reverse", "bool", False),
              _early_exit("°"), MAX_TIME, SETTLE_DEG, MAX_SPEED),
             _sim_swing,
             lambda p: f"swing {num(p['target'])}° lock {p['side'].lower()}" + (", rev" if p["reverse"] else "")
                       + _flags(p),
             includes=("Commands/Swing.h",), handle="angle"),
    StepType("wait", "Wait", "",
             (Param("ms", "Time", "int", 250, "ms", 0, 15000, 50, decimals=0, optional=False),),
             _sim_wait,
             lambda p: f"wait {p['ms']} ms",
             drivetrain=False),
    StepType("code", "Custom code", "",
             (Param("text", "C++", "text", "", optional=False),),
             _sim_code,
             lambda p: "code: " + (p["text"].strip().splitlines() or ["(empty)"])[0],
             drivetrain=False),
]
BY_ID = {s.id: s for s in STEP_TYPES}


def default_params(type_id):
    return {prm.name: prm.default for prm in BY_ID[type_id].params}
