import argparse
import json

import matplotlib.pyplot as plt


ERROR_FIELDS = ("errorDeg", "errorIn")


def load_samples(path, field=None):
    times = []
    errors = []
    used_field = field

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

            if "t" not in data:
                continue

            if used_field is None:
                used_field = next((f for f in ERROR_FIELDS if f in data), None)
                if used_field is None:
                    continue

            if used_field not in data:
                continue

            times.append(data["t"] / 1000.0)  # ms -> seconds
            errors.append(data[used_field])

    return times, errors, used_field


def main():
    parser = argparse.ArgumentParser(description="Plot error vs time from AngularPIDTune/LateralPIDTune telemetry")
    parser.add_argument("--input", type=str, default="input.txt", help="path to the saved telemetry lines")
    parser.add_argument("--field", type=str, default=None,
                         help="error field to plot - auto-detects errorDeg (AngularPIDTune) or "
                              "errorIn (LateralPIDTune) from the log if not given")
    args = parser.parse_args()

    times, errors, field = load_samples(args.input, args.field)

    if not times:
        print(f"No usable samples found in {args.input} (looking for field {args.field or ERROR_FIELDS})")
        return

    print(f"Loaded {len(times)} samples from {args.input} (field {field!r})")

    unit = "deg" if field == "errorDeg" else "in" if field == "errorIn" else field

    plt.axhline(0, color="gray", linewidth=1, linestyle="--")
    plt.plot(times, errors, marker="o", markersize=3)
    plt.xlabel("time (s)")
    plt.ylabel(f"error ({unit})")
    plt.title("PID error vs time")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
