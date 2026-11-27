#!/usr/bin/env python3
"""Run the guided calibration and stream each stage to the console.

The robot will ask you (on its OLED and in this log) to keep still, then to
place it in six orientations, then to rotate it slowly for 20 seconds.

    python calibrate.py                 # all sensors
    python calibrate.py --target mag
"""

from __future__ import annotations

import argparse

from crawlingbot import Robot


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.4.1")
    parser.add_argument("--port", type=int, default=4242)
    parser.add_argument("--serial", metavar="PORT")
    parser.add_argument(
        "--target", choices=["all", "gyro", "accel", "mag"], default="all"
    )
    args = parser.parse_args()

    robot = Robot.serial(args.serial) if args.serial else Robot.tcp(args.host, args.port)

    with robot:
        print(f"calibrating {args.target}; follow the robot's OLED prompts")
        ok = robot.calibrate(args.target, on_progress=lambda line: print(f"  {line}"))
        snap = robot.snapshot()
        print("calibration succeeded" if ok else "calibration FAILED")
        print(f"robot now reports cal={int(snap.calibrated)}")
        return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
