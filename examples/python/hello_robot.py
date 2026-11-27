#!/usr/bin/env python3
"""Smallest possible client: connect, check identity, read one snapshot.

    python hello_robot.py                      # TCP, default 192.168.4.1:4242
    python hello_robot.py --serial /dev/cu.usbmodem1101
"""

from __future__ import annotations

import argparse

from crawlingbot import Robot, list_serial_ports


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.4.1", help="robot IP")
    parser.add_argument("--port", type=int, default=4242, help="robot TCP port")
    parser.add_argument("--serial", metavar="PORT", help="use serial instead of TCP")
    parser.add_argument("--list-ports", action="store_true", help="print serial ports and exit")
    args = parser.parse_args()

    if args.list_ports:
        for port in list_serial_ports():
            print(port)
        return 0

    if args.serial:
        robot = Robot.serial(args.serial)
    else:
        robot = Robot.tcp(args.host, args.port)

    with robot:
        print(f"connected to {robot.description}")
        print(f"uptime      : {robot.ping()} ms")
        info = robot.id_info()
        print(f"robot id    : {info['id']} ({info['ssid']})")
        snap = robot.snapshot()
        print(f"attitude    : roll {snap.roll:+.2f}  pitch {snap.pitch:+.2f}  yaw {snap.yaw:+.2f} deg")
        print(f"accel       : {snap.accel[0]:+.3f} {snap.accel[1]:+.3f} {snap.accel[2]:+.3f} m/s^2")
        print(f"temperature : {snap.temperature_c:.1f} C")
        print(f"calibrated  : {snap.calibrated}   still: {snap.still}")
        print(f"legs        : down {snap.down} (target {snap.down_target}), up {snap.up} (target {snap.up_target})")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
