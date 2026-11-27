#!/usr/bin/env python3
"""Crawling Bot V2 - robot visualizer and command console.

Plots everything the robot streams, shows its orientation and heading, runs a
pass/fail health checklist, and gives you a button for every protocol command.

    python tools/robot_visualizer.py --demo                 # no hardware
    python tools/robot_visualizer.py --tcp                  # 192.168.4.1:4242
    python tools/robot_visualizer.py --serial auto          # find the ESP32
    python tools/robot_visualizer.py --serial /dev/cu.usbmodem1101
    python tools/robot_visualizer.py --replay run.csv       # a recorded run

Keys:  space pause | [ ] time window | m Y-range preset | z zero position
Overrides: --fps, --window, --preset, --log, --no-blit, --host, --tcp-port,
           --stream-hz (the rate it asks for on connect, default 50)

The window is split in two:

  left   six consolidated charts (acceleration, gyro, attitude, state, velocity,
         bias/calibration). They replace the twelve charts of the old IMU
         visualizer; the summary panel on the right still shows every number.
  right  3D orientation, heading dial, health checklist, live state, and the
         command console with one button per protocol command.

Health has two sources: the robot's own boot log (``HEALTH,``/``CAL,``/
``PROTOCOL,`` lines) and checks the viewer makes on the live stream (rate,
estimator seed, sensor magnitude). A row turns red only when it can say what is
wrong. See tools/README.md.
"""
from __future__ import annotations

import argparse
import queue
import sys
from pathlib import Path

# Allow "python tools/robot_visualizer.py" and "python -m tools.robot_visualizer".
sys.path.insert(0, str(Path(__file__).resolve().parent))

from robot_viewer.app import App                  # noqa: E402
from robot_viewer.config import Config            # noqa: E402
from robot_viewer.logging_sink import Logger      # noqa: E402
from robot_viewer.sources import (                # noqa: E402
    DemoSource,
    ReplaySource,
    SerialSource,
    TcpSource,
    find_serial_port,
    list_serial_ports,
)
from robot_viewer.telemetry import LineParser     # noqa: E402


def build_link(config, replay=None, demo=False):
    """Create (source, mode) for the requested transport."""
    out = queue.SimpleQueue()
    parser = LineParser(config.line_prefix)

    if demo:
        return DemoSource(out, parser, config), out, "DEMO"
    if replay:
        return ReplaySource(out, parser, config, str(replay)), out, "REPLAY"
    if config.transport == "tcp":
        return TcpSource(out, parser, config), out, "LIVE"
    return SerialSource(out, parser, config), out, "LIVE"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Crawling Bot V2 robot visualizer")
    link = parser.add_argument_group("connection")
    link.add_argument("--serial", nargs="?", const="auto", metavar="PORT",
                      help="serial port, or --serial with no value to auto-detect")
    link.add_argument("--tcp", action="store_true", help="use TCP instead of serial")
    link.add_argument("--host", default="192.168.4.1", help="robot IP for --tcp")
    link.add_argument("--tcp-port", type=int, default=4242, help="robot TCP port")
    link.add_argument("--baud", type=int, default=115200)
    link.add_argument("--demo", action="store_true", help="synthetic data, no robot")
    link.add_argument("--replay", metavar="CSV", help="replay a recorded CSV")
    link.add_argument("--speed", type=float, default=1.0, help="replay speed")
    link.add_argument("--list-ports", action="store_true", help="print serial ports and exit")

    view = parser.add_argument_group("view")
    view.add_argument("--fps", type=int, default=30)
    view.add_argument("--window", type=float, help="initial time window, seconds")
    view.add_argument("--preset", choices=["overview", "detail", "auto"])
    view.add_argument("--log", action="store_true", help="record to logs/*.csv")
    view.add_argument("--log-dir", default="logs")
    view.add_argument("--no-blit", action="store_true", help="full redraws (slower, safest)")
    view.add_argument("--stream-hz", type=int, default=None,
                      help="rate to request automatically (default 50, 0 = never ask)")
    args = parser.parse_args(argv)

    if args.list_ports:
        ports = list_serial_ports()
        guess = find_serial_port()
        if not ports:
            print("no serial ports found (install pyserial?)", file=sys.stderr)
        for port in ports:
            print(f"{port}   <- likely the robot" if port == guess else port)
        print(f"auto would choose: {guess or 'nothing'}")
        return 0

    config = Config()
    if args.serial is not None:
        config.transport, config.port = "serial", args.serial
    if args.tcp:
        config.transport = "tcp"
    config.host, config.tcp_port, config.baud = args.host, args.tcp_port, args.baud
    config.fps = args.fps
    if args.stream_hz is not None:
        config.auto_stream_hz = max(0, min(50, args.stream_hz))
        config.auto_stream = config.auto_stream_hz > 0
    config.use_blit = not args.no_blit
    config.log, config.log_dir = args.log, args.log_dir
    if args.preset:
        config.preset = args.preset
    if args.window:
        config.window_steps = tuple(sorted(set(config.window_steps) | {args.window}))
        config.window_index = config.window_steps.index(args.window)

    if args.replay:
        config.transport = "replay"

    source, out, mode = build_link(config, replay=args.replay, demo=args.demo)

    logger = None
    if config.log and mode != "REPLAY":
        logger = Logger(config.log_dir, config.line_prefix)

    app = App(config, source, out, logger, mode)
    source.start()
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
