"""Turns a simulated plan into a Trace, so it plays / scrubs / steps on the same playhead as odom and camera."""
import numpy as np

from app import field_config as cfg
from app.run_data import make_trace

PLAN_KEY = "plan"
DT = 0.02  #resample the plan at 50 Hz, same as the camera


def plan_trace(sim, plan, offset):
    """Plan poses on a uniform time grid, shifted by offset (s) onto the run's clock."""
    ts, xs, ys, hs = [], [], [], []
    for s in sim.steps:
        if not s.step.kind.drivetrain or not s.samples:
            continue  #waits / code don't move the robot (and an async motion keeps going through them)
        t0 = s.samples[0][0]
        while ts and ts[-1] > t0 + 1e-9:  #this chassis command cancels an async one that's still running
            for arr in (ts, xs, ys, hs):
                arr.pop()
        for t, x, y, h in s.samples:
            ts.append(t)
            xs.append(x)
            ys.append(y)
            hs.append(h)
    start = (0.0, plan.start_x, plan.start_y, plan.start_heading)
    if not ts or ts[0] > 0:
        for arr, v in zip((ts, xs, ys, hs), start):
            arr.insert(0, v)
    total = max(sim.total, ts[-1], DT)
    grid = np.arange(0.0, total + DT / 2, DT)
    #headings are wrapped differently step to step, unwrap before interpolating so 359 -> 1 doesn't spin around
    h = np.degrees(np.unwrap(np.radians(np.asarray(hs, float))))
    x = np.interp(grid, ts, xs)
    y = np.interp(grid, ts, ys)
    heading = np.interp(grid, ts, h) % 360
    return make_trace(PLAN_KEY, "Plan", cfg.PLAN_COLOR, grid + offset, x, y, heading,
                      np.full(len(grid), np.nan), False, "planned")


def first_motion_time(tr, v_min, w_min, hold=5):
    """Time the trace starts moving for real: |v| or |ω| over the threshold for `hold` samples in a row."""
    if tr is None or len(tr) < hold:
        return None
    moving = (np.abs(np.nan_to_num(tr.v)) > v_min) | (np.abs(np.nan_to_num(tr.omega)) > w_min)
    run = np.convolve(moving.astype(int), np.ones(hold, int), "valid") == hold
    idx = np.flatnonzero(run)
    return float(tr.t[idx[0]]) if len(idx) else None


def auto_offset(run, plan_tr_at_zero):
    """Line the plan's first motion up with the run's first motion (odom if there is one - camera is noisier)."""
    if run is None:
        return 0.0
    if run.robot is not None:
        run_t = first_motion_time(run.robot, 3.0, 20.0)
        fallback = float(run.robot.t[0])
    else:
        run_t = first_motion_time(run.camera, 6.0, 45.0)
        fallback = float(run.camera.t[0])
    plan_t = first_motion_time(plan_tr_at_zero, 3.0, 20.0)
    if run_t is None or plan_t is None:
        return fallback
    return run_t - plan_t
