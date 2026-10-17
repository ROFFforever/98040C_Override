"""Idealized motion models the planner uses to predict where each step leaves the robot and roughly how long it takes.
Mirrors the robot code's behavior (turn directions, skip/early-exit ranges), not its control loops."""
import math

DT = 0.02         #sample spacing (s) for drawn paths
SETTLE_S = 0.15   #rough time a motion without early exit spends settling (8 ticks + PID tail)


def wrap(deg):
    """-> (-180, 180]"""
    deg = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if deg == -180.0 else deg


def _trap_shape(dist, vel, accel, vf):
    """-> (peak speed, accel time, cruise time, decel time) of a 0 -> cruise -> vf trapezoid covering dist.
    vf is the speed it arrives at (min_speed when chained); too short to reach cruise gives a triangle."""
    vf = min(vf, vel)
    d_up = vel * vel / (2 * accel)
    d_down = (vel * vel - vf * vf) / (2 * accel)
    if d_up + d_down >= dist:
        peak = max(math.sqrt(accel * dist + vf * vf / 2), vf)
        return peak, peak / accel, 0.0, (peak - vf) / accel
    return vel, vel / accel, (dist - d_up - d_down) / vel, (vel - vf) / accel


def trap_time(dist, vel, accel, vf=0.0):
    """Time a 0 -> cruise -> vf trapezoid profile takes to cover dist (vf = 0 stops at the end)."""
    if dist <= 0:
        return 0.0
    if accel <= 0:
        return dist / vel
    return sum(_trap_shape(dist, vel, accel, vf)[1:])


def trap_pos(t, dist, vel, accel, vf=0.0):
    total = trap_time(dist, vel, accel, vf)
    if t >= total:
        return dist
    if t <= 0:
        return 0.0
    if accel <= 0:
        return vel * t
    peak, t_up, t_cruise, _ = _trap_shape(dist, vel, accel, vf)
    if t <= t_up:
        return 0.5 * accel * t * t
    d_up = 0.5 * accel * t_up * t_up
    if t <= t_up + t_cruise:
        return d_up + peak * (t - t_up)
    left = total - t  #time left on the way down to vf
    return dist - (vf * left + 0.5 * accel * left * left)


class Motion:
    """Pose samples (t, x, y, heading) built up one phase at a time, starting at t = 0."""

    def __init__(self, pose):
        self.samples = [(0.0, *pose)]
        self.notes = []

    @property
    def t(self):
        return self.samples[-1][0]

    @property
    def pose(self):
        return self.samples[-1][1:]

    def hold(self, duration):
        if duration > 0:
            self.samples.append((self.t + duration, *self.pose))

    def turn(self, delta_deg, duration, pivot=None):
        """Change heading by delta_deg; with a pivot (x, y) the robot swings around that point."""
        t0, x0, y0, h0 = self.samples[-1]
        n = max(1, math.ceil(duration / DT))
        for k in range(1, n + 1):
            f = k / n
            d = delta_deg * f
            x, y = x0, y0
            if pivot is not None:
                c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
                rx, ry = x0 - pivot[0], y0 - pivot[1]
                x, y = pivot[0] + rx * c - ry * s, pivot[1] + rx * s + ry * c
            self.samples.append((t0 + duration * f, x, y, h0 + d))

    def line(self, x1, y1, heading_end, vel, accel, vf=0.0):
        """Drive straight to (x1, y1) on a trapezoid profile that arrives at vf in/s (0 = stops). Heading eases from
        the current one to heading_end over the first 30% of the move (the heading PID pulling it onto the line)."""
        t0, x0, y0, h0 = self.samples[-1]
        dist = math.hypot(x1 - x0, y1 - y0)
        total = trap_time(dist, vel, accel, vf)
        dh = wrap(heading_end - h0)
        n = max(1, math.ceil(total / DT))
        for k in range(1, n + 1):
            t = total * k / n
            f = trap_pos(t, dist, vel, accel, vf) / dist if dist > 0 else 1.0
            self.samples.append((t0 + t, x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, h0 + dh * min(1.0, f / 0.3)))
        return total
