import argparse
import csv
import json
import re
import subprocess

import numpy as np


def find_v5_port():
    result = subprocess.run(["pros", "lsusb"], capture_output=True, text=True)

    ports = re.findall(r"(COM\d+) - VEX V5 (?:Communications|Controller) Port", result.stdout)
    ports = list(dict.fromkeys(ports))

    if not ports:
        print("No VEX V5 brain found in `pros lsusb`. Is it plugged in via USB (wired, or via the controller)?")
        print(result.stdout)
        return

    if len(ports) > 1:
        print(f"Multiple V5 brains found on {ports} - `pros terminal` will pick its own default.")
    else:
        print(f"Found V5 on {ports[0]}")


def collect_samples(proc):
    samples = []

    for line in proc.stdout:
        line = line.strip()
        if not line:
            continue

        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            print(f"[unparsed] {line!r}")
            continue

        if data.get("characterize_done"):
            break

        if data.get("characterize"):
            samples.append((data["t"], data["volts"], data["vel"]))
            if len(samples) % 20 == 0:
                print(f"...{len(samples)} samples collected")
        else:
            print(data)

    return samples


# Quasistatic ramp test: voltage climbs slowly enough that acceleration stays
# ~0 throughout, so along the ramp volts ~= kS + kV * vel (the kA term drops
# out because accel ~= 0). That means this fit can ONLY give you kV and kS -
# it deliberately can't tell you kA, since there's no meaningful acceleration
# signal in this data to regress against. kA needs a separate step/dynamic
# test (snap to a fixed high voltage from rest and log the transient).
def fit_kv_ks(samples, trim_seconds):
    t = np.array([s[0] for s in samples])
    volts = np.array([s[1] for s in samples])
    vel = np.array([s[2] for s in samples])

    mask = t >= trim_seconds
    volts = volts[mask]
    vel = vel[mask]

    if len(vel) < 2:
        raise ValueError(f"Not enough samples after trimming the first {trim_seconds}s (got {len(vel)}).")

    kV, kS = np.polyfit(vel, volts, 1)
    return kV, kS


def write_csv(path, samples):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "volts", "vel"])
        writer.writerows(samples)


def main():
    parser = argparse.ArgumentParser(description="Fit kV/kS from a DriveCharacterizationCommand ramp test")
    parser.add_argument("--trim-seconds", type=float, default=1.0,
                         help="drop samples before this many seconds to cut out the stiction breakaway transient")
    parser.add_argument("--csv", type=str, default=None, help="path to dump the raw (t, volts, vel) samples")
    args = parser.parse_args()

    find_v5_port()

    proc = subprocess.Popen(
        ["pros", "terminal"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    print("Listening via `pros terminal`... run DriveCharacterizationCommand on the robot now.")

    try:
        samples = collect_samples(proc)
    finally:
        proc.terminate()

    if not samples:
        print("No characterization samples were collected.")
        return

    print(f"Collected {len(samples)} samples.")

    if args.csv:
        write_csv(args.csv, samples)
        print(f"Raw samples written to {args.csv}")

    kV, kS = fit_kv_ks(samples, args.trim_seconds)

    print(f"kV = {kV:.4f} mV per (in/s)")
    print(f"kS = {kS:.4f} mV")
    print("kA was NOT fit here - a quasistatic ramp keeps acceleration near zero on purpose, "
          "so there's no real signal to pull kA out of. Run a separate step/dynamic test for that.")


if __name__ == "__main__":
    main()
