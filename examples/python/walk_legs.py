#!/usr/bin/env python3
"""Drive the legs through a short routine, then park.

This is the "act" half of the interface: it shows how a policy from any
language would send actions. Moves are ramped by the firmware, so a large step
is safe, but this script still waits for each move to settle.

    python walk_legs.py --host 192.168.4.1
"""

from __future__ import annotations

import argparse
import time

from crawlingbot import Robot


def wait_for_legs(robot: Robot, timeout: float = 5.0) -> None:
    """Poll GET until both axes stop moving."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not robot.snapshot().servo_moving:
            return
        time.sleep(0.05)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.4.1")
    parser.add_argument("--port", type=int, default=4242)
    parser.add_argument("--serial", metavar="PORT")
    args = parser.parse_args()

    robot = Robot.serial(args.serial) if args.serial else Robot.tcp(args.host, args.port)

    with robot:
        if not robot.snapshot().calibrated:
            print("robot reports cal=0; run `python calibrate.py` first")
            return 1

        print("lifting")
        robot.act(180, 60)
        wait_for_legs(robot)

        print("pushing")
        robot.act(60, 60)
        wait_for_legs(robot)

        print("resetting")
        robot.act(180, 180)
        wait_for_legs(robot)

        print(f"final pose: {robot.snapshot().down}/{robot.snapshot().up}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
