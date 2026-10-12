import numpy as np


def local_slope(t, values, half_window):
    """Slope of a least-squares line through every sample within +-half_window seconds of each point.
    Averages out sensor noise, and works with uneven sample spacing and gaps (dropped camera frames).
    NaN where there aren't at least 3 usable samples in the window."""
    t = np.asarray(t, dtype=float)
    values = np.asarray(values, dtype=float)
    ok = np.isfinite(values)
    v = np.where(ok, values, 0.0)
    w = ok.astype(float)
    t0 = t - t[0]  #keeps the sums small so float precision stays good

    lo = np.searchsorted(t, t - half_window, side="left")
    hi = np.searchsorted(t, t + half_window, side="right")

    #running sums make every window sum O(1), so this stays fast with thousands of points
    def window_sum(a):
        c = np.concatenate(([0.0], np.cumsum(a)))
        return c[hi] - c[lo]

    n = window_sum(w)
    st, sv = window_sum(t0 * w), window_sum(v)
    stt, stv = window_sum(t0 * t0 * w), window_sum(t0 * v)
    denom = n * stt - st * st
    with np.errstate(invalid="ignore", divide="ignore"):
        slope = (n * stv - st * sv) / denom
    slope[(n < 3) | (denom <= 1e-12)] = np.nan
    return slope


def compute_motion(t, x, y, heading_deg, vel_window, accel_window):
    """Returns (v, a, omega):
    v     = velocity along the robot's heading, in/s (+ driving forward, - reversing)
    a     = acceleration along the heading, in/s^2 (+ speeding up forward / braking a reverse)
    omega = angular velocity, deg/s (+ = CCW)"""
    vx = local_slope(t, x, vel_window)
    vy = local_slope(t, y, vel_window)
    h = np.radians(heading_deg)
    v = vx * np.cos(h) + vy * np.sin(h)
    a = local_slope(t, v, accel_window)
    omega = local_slope(t, np.degrees(np.unwrap(h)), vel_window)  #unwrap so 359 -> 1 isn't a -358 deg jump
    return v, a, omega
