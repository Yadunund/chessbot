# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Find the arm: ping Feetech servo ids on serial ports, read nothing else.

Answers the first question of any hardware bring-up - is the arm on this port, at this
baud rate, and powered? A servo that replies is wired and awake; silence means the wrong
port, the wrong baud rate, or no bus power.

    pixi run python tools/dev/ping_servos.py /dev/ttyACM0 /dev/ttyACM1

Sends only the protocol's PING instruction, which reads no register and writes nothing.
The port is configured with stty, since the environment has no pyserial and the bus runs
at 1 Mbaud, which termios cannot set portably.
"""

from __future__ import annotations

import argparse
import os
import select
import subprocess
import time

BAUD_RATES = (1_000_000, 500_000, 115_200)
PING = 0x01


def configure(port: str, baud: int):
    subprocess.run(["stty", "-F", port, str(baud), "raw", "-echo", "-echoe", "-echok", "-crtscts"],
                   check=True, capture_output=True)


def ping(port: str, baud: int, ids: range, timeout_s: float = 0.1) -> list[tuple[int, float]]:
    """Ids that replied, each with how long the reply took in milliseconds.

    The latency matters: the driver gives a servo 5 ms to answer, so a bus that is slower
    than that on this machine fails to initialise even though every servo is alive.
    """
    configure(port, baud)
    found = []
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    try:
        for servo_id in ids:
            body = bytes([servo_id, 0x02, PING])
            packet = b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF])
            # Drop anything already buffered, so a reply cannot be mistaken for an earlier one.
            while select.select([fd], [], [], 0)[0]:
                os.read(fd, 256)
            started = time.monotonic()
            os.write(fd, packet)
            deadline = started + timeout_s
            reply = b""
            while time.monotonic() < deadline and len(reply) < 6:
                if select.select([fd], [], [], max(0.0, deadline - time.monotonic()))[0]:
                    reply += os.read(fd, 6 - len(reply))
            if reply.startswith(b"\xff\xff") and len(reply) >= 4 and reply[2] == servo_id:
                found.append((servo_id, 1000.0 * (time.monotonic() - started)))
    finally:
        os.close(fd)
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("ports", nargs="+")
    parser.add_argument("--ids", type=int, nargs=2, default=[1, 7], metavar=("FIRST", "LAST"))
    parser.add_argument("--baud", type=int, nargs="*", default=list(BAUD_RATES))
    args = parser.parse_args()
    ids = range(args.ids[0], args.ids[1] + 1)
    for port in args.ports:
        for baud in args.baud:
            try:
                found = ping(port, baud, ids)
            except (OSError, subprocess.CalledProcessError) as exc:
                print(f"{port} at {baud}: {exc}")
                continue
            if not found:
                print(f"{port} at {baud}: no reply")
                continue
            latencies = ", ".join(f"{sid}: {ms:.1f} ms" for sid, ms in found)
            print(f"{port} at {baud}: {latencies}")


if __name__ == "__main__":
    main()
