"""Runs a plan through the step models: pose after every step, rough timing, and warnings
(robot past the wall, async steps getting interrupted, over the time limit)."""
import math
from dataclasses import dataclass, field

from app import field_config as cfg
from app.planner.plan import Group


@dataclass
class StepSim:
    path: tuple
    number: int          #1-based, in run order
    step: object
    start: tuple         #(x, y, heading) when the step starts
    end: tuple
    t0: float            #estimated start time in the auton (s)
    duration: float
    samples: list        #(t, x, y, heading), t absolute
    notes: list = field(default_factory=list)
    is_async: bool = False


@dataclass
class PlanSim:
    steps: list
    by_path: dict
    total: float
    warnings: list       #(path or None, text)


def can_async(step):
    """Only commands can be scheduled; waits are pros::delay and custom code is pasted as-is."""
    return bool(step.kind.call)


def _wall_overshoot(samples, profile):
    """Furthest any corner of the robot outline gets past the +-WALL_IN wall face."""
    corners = [(profile.center_forward_in + fx * profile.length_in / 2, profile.center_left_in + fy * profile.width_in / 2)
               for fx in (-1, 1) for fy in (-1, 1)]
    worst = 0.0
    for _, x, y, h in samples:
        c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
        for f, l in corners:
            px, py = x + f * c - l * s, y + f * s + l * c
            worst = max(worst, abs(px) - cfg.WALL_IN, abs(py) - cfg.WALL_IN)
    return worst


def simulate_plan(plan, profile):
    lines = plan.style == "lines"
    pose = (plan.start_x, plan.start_y, plan.start_heading % 360)
    steps, warnings = [], []
    cursor = 0.0       #when the next awaited line starts
    latest = 0.0       #when the last thing (awaited or async) finishes
    busy = None        #(until, label) - an async chassis command that's still running

    def run(path, step, t0, is_async):
        nonlocal pose
        m = step.kind.simulate(pose, step.params, profile)
        sim = StepSim(path, len(steps) + 1, step, pose, (m.pose[0], m.pose[1], m.pose[2] % 360), t0, m.t,
                      [(t0 + t, x, y, h) for t, x, y, h in m.samples], list(m.notes), is_async)
        over = _wall_overshoot(sim.samples, profile)
        if over > 0.05:
            sim.notes.append(f"robot goes {over:.1f} in past the wall")
        steps.append(sim)
        pose = sim.end
        return sim

    def check_interrupt(path, t, uses_chassis):
        nonlocal busy
        if busy is None or not uses_chassis:
            return
        if t < busy[0] - 1e-6:
            warnings.append((path, f"step {len(steps) + 1} interrupts {busy[1]} (≈{busy[0] - t:.1f} s left) - "
                                   f"a new chassis command cancels the running one"))
        busy = None

    for i, item in enumerate(plan.items):
        if isinstance(item, Group):
            is_async = lines and item.is_async
            uses_chassis = any(s.kind.drivetrain for s in item.steps)
            check_interrupt((i,), cursor, uses_chassis)  #a Sequence claims the chassis as soon as it starts
            t = cursor
            first = len(steps) + 1
            for j, s in enumerate(item.steps):
                t += run((i, j), s, t, is_async).duration
            if is_async and uses_chassis:
                busy = (t, f"async sequence (steps {first}-{len(steps)})")
        else:
            is_async = lines and item.is_async and can_async(item)
            check_interrupt((i,), cursor, item.kind.drivetrain)
            t = cursor + run((i,), item, cursor, is_async).duration
            if is_async and item.kind.drivetrain:
                busy = (t, f"async step {len(steps)}")
        latest = max(latest, t)
        if not is_async:
            cursor = t

    if busy is not None and busy[0] > cursor + 1e-6:
        warnings.append((None, f"{busy[1]} is still running when the auton function returns "
                               f"(it keeps going in the scheduler)"))
    if not lines and any(it.is_async for it in plan.items):
        warnings.append((None, "async is ignored in Sequence style - everything runs in order"))
    if latest > plan.time_limit:
        warnings.append((None, f"≈{latest:.1f} s is over the {plan.time_limit:g} s limit"))
    for sim in steps:
        warnings += [(sim.path, f"step {sim.number}: {n}") for n in sim.notes if "past the wall" in n]
    return PlanSim(steps, {s.path: s for s in steps}, latest, warnings)
