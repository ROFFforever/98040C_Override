"""
Analysis tool for Commands/Tuning/LateralMotionDiagnostic telemetry.

Workflow:
  1. On the robot, schedule LateralMotionDiagnostic(&chassis, distanceIn, usePID, ...)
     in opcontrol() (see main.cpp for a commented-out example near the other
     Tuning commands).
  2. Capture the run: `pros terminal > run_pid_on.txt` (Ctrl+C once the robot
     stops), or use receive_telemetry.py.
  3. Run this script against the saved file:
       python analyze_motion_diagnostic.py --input run_pid_on.txt
     Or compare a PID-on run against a PID-off ("pure KAV") run side by side:
       python analyze_motion_diagnostic.py --input run_pid_on.txt --compare run_pid_off.txt

What it looks for:
  - Peak position overshoot beyond the commanded distance.
  - Settling time (first moment error stays within settle_range for good).
  - Whether actual velocity ever runs meaningfully ahead of the profile's
    scheduled velocity during the active "profile" phase - a positive
    velSurplus there means the robot is carrying more speed than the
    trapezoid profile accounted for, which is the leading suspect for
    overshoot on a heavier/taller chassis (no kA margin + underdamped
    residual PID).
"""

import argparse
import json


FIELDS = ("t", "pidOn", "phase", "targetPosIn", "actualPosIn", "errorIn",
          "targetVel", "actualVel", "velSurplus", "targetAccel",
          "ffMv", "pidMv", "appliedPidMv", "totalMv")


def load_run(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                continue
            if "targetPosIn" not in data:
                continue  # not a LateralMotionDiagnostic line (e.g. periodic() pose spam)
            rows.append(data)
    return rows


def summarize(rows, label):
    if not rows:
        print(f"[{label}] no usable samples")
        return

    distance = rows[-1]["targetPosIn"]
    peak_pos = max(r["actualPosIn"] for r in rows)
    overshoot = peak_pos - distance

    settle_range_guess = None
    # find the last time error crossed back inside a small band and stayed there
    final_error = rows[-1]["errorIn"]

    profile_rows = [r for r in rows if r["phase"] == "profile"]
    max_surplus = max((r["velSurplus"] for r in profile_rows), default=0.0)
    max_surplus_t = next((r["t"] for r in profile_rows if r["velSurplus"] == max_surplus), None)

    settle_rows = [r for r in rows if r["phase"] == "settle"]
    settle_start_t = settle_rows[0]["t"] if settle_rows else None

    print(f"--- {label} ---")
    print(f"  pidOn:                 {rows[0]['pidOn']}")
    print(f"  commanded distance:    {distance:.2f} in")
    print(f"  peak actual position:  {peak_pos:.2f} in  (overshoot: {overshoot:+.2f} in)")
    print(f"  final error:           {final_error:+.2f} in")
    print(f"  profile phase ends at: {settle_start_t} ms" if settle_start_t is not None else "  profile phase never ended (timed out?)")
    print(f"  max velocity surplus during profile phase (actual - scheduled): {max_surplus:+.2f} in/s"
          + (f" at t={max_surplus_t}ms" if max_surplus_t is not None else ""))
    if max_surplus > 3.0:
        print("  -> robot ran notably faster than the profile scheduled during the active phase.")
        print("     That surplus has to be shed somewhere - likely cause of any overshoot below.")
    print()


def plot(runs):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed - skipping plots (pip install matplotlib). Summary above still applies.")
        return

    fig, (ax_pos, ax_vel, ax_volt) = plt.subplots(3, 1, sharex=True, figsize=(10, 9))

    colors = ["tab:blue", "tab:orange"]
    for (rows, label), color in zip(runs, colors):
        if not rows:
            continue
        t = [r["t"] / 1000.0 for r in rows]
        distance = rows[-1]["targetPosIn"]

        ax_pos.plot(t, [r["targetPosIn"] for r in rows], color=color, linestyle="--", linewidth=1,
                    label=f"{label} target")
        ax_pos.plot(t, [r["actualPosIn"] for r in rows], color=color, linewidth=1.5,
                    label=f"{label} actual")

        ax_vel.plot(t, [r["targetVel"] for r in rows], color=color, linestyle="--", linewidth=1,
                    label=f"{label} target")
        ax_vel.plot(t, [r["actualVel"] for r in rows], color=color, linewidth=1.5,
                    label=f"{label} actual")

        ax_volt.plot(t, [r["ffMv"] for r in rows], color=color, linestyle=":", linewidth=1,
                     label=f"{label} ff")
        ax_volt.plot(t, [r["appliedPidMv"] for r in rows], color=color, linestyle="-.", linewidth=1,
                     label=f"{label} pid")
        ax_volt.plot(t, [r["totalMv"] for r in rows], color=color, linewidth=1.5,
                     label=f"{label} total")

        # mark where the profile handed off to settle
        settle_rows = [r for r in rows if r["phase"] == "settle"]
        if settle_rows:
            handoff_t = settle_rows[0]["t"] / 1000.0
            for ax in (ax_pos, ax_vel, ax_volt):
                ax.axvline(handoff_t, color=color, alpha=0.3, linewidth=1)

        ax_pos.axhline(distance, color=color, alpha=0.3, linewidth=1)

    ax_pos.set_ylabel("position (in)")
    ax_pos.set_title("Target vs actual position (dashed = target, solid = actual, faint vline = profile->settle handoff)")
    ax_pos.legend(fontsize=8)
    ax_pos.grid(True)

    ax_vel.set_ylabel("velocity (in/s)")
    ax_vel.set_title("Target vs actual velocity - actual running above target during 'profile' phase is the overshoot signature")
    ax_vel.legend(fontsize=8)
    ax_vel.grid(True)

    ax_volt.set_ylabel("voltage (mV)")
    ax_volt.set_xlabel("time (s)")
    ax_volt.set_title("Feedforward vs applied PID vs total commanded voltage")
    ax_volt.legend(fontsize=8)
    ax_volt.grid(True)

    fig.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Analyze LateralMotionDiagnostic telemetry")
    parser.add_argument("--input", type=str, required=True, help="path to a saved telemetry capture")
    parser.add_argument("--compare", type=str, default=None,
                         help="optional second capture to overlay (e.g. a usePID=false run against a usePID=true run)")
    parser.add_argument("--no-plot", action="store_true", help="print summary only, skip matplotlib")
    args = parser.parse_args()

    runs = [(load_run(args.input), args.input)]
    if args.compare:
        runs.append((load_run(args.compare), args.compare))

    for rows, label in runs:
        summarize(rows, label)

    if not args.no_plot:
        plot(runs)


if __name__ == "__main__":
    main()
