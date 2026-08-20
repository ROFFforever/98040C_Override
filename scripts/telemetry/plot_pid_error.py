import argparse
import json

import matplotlib.pyplot as plt


def load_samples(path, field):
    times = []
    errors = []

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                print(f"[unparsed] {line!r}")
                continue

            if "t" not in data or field not in data:
                continue

            times.append(data["t"] / 1000.0)  # ms -> seconds
            errors.append(data[field])

    return times, errors


def main():
    parser = argparse.ArgumentParser(description="Plot error vs time from AngularPIDTune/LateralPIDTune telemetry")
    parser.add_argument("--input", type=str, default="input.txt", help="path to the saved telemetry lines")
    parser.add_argument("--field", type=str, default="errorDeg",
                         help="error field to plot - errorDeg for AngularPIDTune, errorIn for LateralPIDTune")
    args = parser.parse_args()

    times, errors = load_samples(args.input, args.field)

    if not times:
        print(f"No usable samples found in {args.input} (looking for field {args.field!r})")
        return

    print(f"Loaded {len(times)} samples from {args.input}")

    unit = "deg" if args.field == "errorDeg" else "in" if args.field == "errorIn" else args.field

    plt.axhline(0, color="gray", linewidth=1, linestyle="--")
    plt.plot(times, errors, marker="o", markersize=3)
    plt.xlabel("time (s)")
    plt.ylabel(f"error ({unit})")
    plt.title("PID error vs time")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
