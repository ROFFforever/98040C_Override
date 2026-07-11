import json
import re
import subprocess


def find_v5_port():
    """Ask the PROS CLI what's plugged in, purely to print which port we
    expect `pros terminal` to land on. We deliberately do NOT pass this port
    into `pros terminal` - explicitly naming a port breaks the connection
    (confirmed: `pros terminal COM3` dies with "Connection to COM3 broken",
    while bare `pros terminal` streams fine). Likely cause: explicit-port
    connections route through the "share" backend, which `pros terminal
    --help` admits is "not yet implemented" in this CLI version. So this is
    just an FYI print - the real auto-detection is left to `pros terminal`
    itself, since that's the path that actually works.
    """
    result = subprocess.run(["pros", "lsusb"], capture_output=True, text=True)

    ports = re.findall(r"(COM\d+) - VEX V5 (?:Communications|Controller) Port", result.stdout)
    ports = list(dict.fromkeys(ports))  # dedupe while keeping first-seen order -
                                         # lsusb lists the same COM port again
                                         # under "User Ports" when it's a controller tether

    if not ports:
        print("No VEX V5 brain found in `pros lsusb`. Is it plugged in via USB (wired, or via the controller)?")
        print(result.stdout)
        return

    if len(ports) > 1:
        print(f"Multiple V5 brains found on {ports} - `pros terminal` will pick its own default.")
    else:
        print(f"Found V5 on {ports[0]}")


def main():
    # `pros terminal` already knows how to decode the V5's serial protocol and
    # print plain text - rather than reimplement that decoding ourselves, just
    # run it as a subprocess and read the plain text it prints. Deliberately
    # NOT passed a port - see find_v5_port() for why.
    find_v5_port()

    proc = subprocess.Popen(
        ["pros", "terminal"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    print("Listening via `pros terminal`... (Ctrl+C to stop)")

    for line in proc.stdout:
        line = line.strip()
        if not line:
            continue

        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            # Don't just swallow it - if this fires a lot, it means something
            # (a prefix, a color code, extra text) is stopping us from ever
            # recognizing valid JSON, so surface it instead of going silent.
            print(f"[unparsed] {line!r}")
            continue

        print(data)


if __name__ == "__main__":
    main()
