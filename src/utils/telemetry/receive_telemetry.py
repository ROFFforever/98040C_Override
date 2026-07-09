import json
import subprocess


def main():
    # `pros terminal` already knows how to decode the V5's serial protocol and
    # print plain text - rather than reimplement that decoding ourselves, just
    # run it as a subprocess and read the plain text it prints.
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
            continue

        print(data)


if __name__ == "__main__":
    main()
