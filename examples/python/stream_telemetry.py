#!/usr/bin/env python3
"""Log telemetry frames to a CSV file, or print them as they arrive.

    python stream_telemetry.py --seconds 10 --csv run.csv
    python stream_telemetry.py --serial /dev/cu.usbmodem1101 --frames 200
"""

from __future__ import annotations

import argparse
import csv
import sys

from crawlingbot import Robot


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.4.1")
    parser.add_argument("--port", type=int, default=4242)
    parser.add_argument("--serial", metavar="PORT")
    parser.add_argument("--hz", type=int, default=50, help="stream rate, max 50")
    parser.add_argument("--frames", type=int, help="stop after N frames")
    parser.add_argument("--seconds", type=float, help="stop after N seconds")
    parser.add_argument("--csv", metavar="FILE", help="write frames here instead of printing")
    args = parser.parse_args()

    robot = Robot.serial(args.serial) if args.serial else Robot.tcp(args.host, args.port)

    writer = None
    handle = None
    if args.csv:
        handle = open(args.csv, "w", newline="", encoding="utf-8")
        writer = csv.writer(handle)
        writer.writerow(["t_ms", "roll", "pitch", "yaw", "ax", "ay", "az", "vx", "vy", "vz"])

    count = 0
    try:
        with robot:
            for frame in robot.frames(hz=args.hz, count=args.frames, duration=args.seconds):
                count += 1
                if writer is not None:
                    writer.writerow(
                        [
                            frame.timestamp_ms,
                            f"{frame.roll:.2f}",
                            f"{frame.pitch:.2f}",
                            f"{frame.yaw:.2f}",
                            f"{frame.accel[0]:.3f}",
                            f"{frame.accel[1]:.3f}",
                            f"{frame.accel[2]:.3f}",
                            f"{frame.velocity[0]:.3f}",
                            f"{frame.velocity[1]:.3f}",
                            f"{frame.velocity[2]:.3f}",
                        ]
                    )
                else:
                    print(
                        f"t={frame.timestamp_ms:>8d} "
                        f"roll={frame.roll:+7.2f} pitch={frame.pitch:+7.2f} "
                        f"yaw={frame.yaw:+8.2f} still={int(frame.still)}"
                    )
    except KeyboardInterrupt:
        pass
    finally:
        if handle is not None:
            handle.close()

    print(f"{count} frames", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
