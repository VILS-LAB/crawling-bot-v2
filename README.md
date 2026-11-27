# Crawling Bot V2 — robot firmware

Stable firmware for the IUST AI-course crawling robot. The robot is a
**self-contained service**: it owns the sensors, the servos, the display and the
network, and it exposes everything a controller needs over one ASCII protocol on
**USB serial and TCP**. Your learning algorithm runs wherever you like — on a
laptop, in a notebook, in Python, C++, MATLAB, JavaScript — and talks to the
robot over WiFi.

> 🐍 **Writing a Python controller?** `pip install "crawlingbot[serial]"` and skip
> to [docs/languages/python.md](docs/languages/python.md). Releasing the package
> is documented in [docs/PYPI.md](docs/PYPI.md).

<!-- Photograph placeholder. Drop the assembled-robot photo at
     docs/hardware/img/robot-overview.jpg, then uncomment the image tag that
     used to be here (an alt-text + path markdown image). -->

> 📌 **This repository is firmware only.** It does not contain a reference
> solution, an RL library or a course assignment. Those live with the course;
> here you get a robot that boots reliably, stays calibrated, protects its
> mechanics and answers commands deterministically.

## What the robot gives you

| Capability | Detail |
|---|---|
| State observation | 9-DOF attitude (roll/pitch/yaw), body acceleration, gyro, magnetic field, still/motion flag, velocity/position estimate |
| Actions | two leg servos, each 0-180°, ramped and soft-limited |
| Transport | USB serial (115200) and TCP `<robot-ip>:4242`, same protocol on both |
| Discovery | the robot is a WiFi AP `ESP32-AP-<n>`, password `12345678` |
| Feedback | 128×64 OLED, structured serial log, and a telemetry stream |
| Recovery | hardware watchdog, non-blocking boot, over-the-air updates |

The whole interface is one text line protocol — no SDK, no generated stubs:

```text
PING                      -> OK PONG 123456
ID                        -> OK id=1 ssid=ESP32-AP-1 ip=192.168.4.1
GET                       -> OK SNAP,id=1,t=51402,roll=1.24,pitch=-0.31,...
STREAM 50                 -> OK stream=50        (live telemetry)
ACT 120 60                -> OK ACT d=120 u=60   (move both legs)
STOP | PARK               -> OK STOP | OK PARK
CAL [all|gyro|accel|mag]  -> OK CAL sensors=all  (guided calibration)
SETID 3                   -> OK id=3
```

Full grammar, field meanings and error codes: **[docs/PROTOCOL.md](docs/PROTOCOL.md)**.

## Quick start

```bash
# 1. flash the robot (first time over USB, later over the air)
pio run --target upload

# 2. join the robot's WiFi: ESP32-AP-<n> / 12345678

# 3. talk to it from Python (reference client, no dependencies)
cd examples/python
python hello_robot.py --host 192.168.4.1
```

```text
connected to tcp://192.168.4.1:4242
uptime      : 51402 ms
robot id    : 1 (ESP32-AP-1)
attitude    : roll +1.24  pitch -0.31  yaw +97.05 deg
legs        : down 180 (target 180), up 180 (target 180)
```

From there: `walk_legs.py` sends actions, `stream_telemetry.py` logs frames,
`calibrate.py` runs the guided calibration. See
[examples/python/README.md](examples/python/README.md).

## Writing a client in your language

There is nothing to build or link. Open a socket to port 4242 (or the serial
port at 115200 baud), write one command per line, read lines until you get the
`OK`/`ERR` reply.

```python
import socket

s = socket.create_connection(("192.168.4.1", 4242), timeout=5)
stream = s.makefile("r")            # ONE buffered reader for the session

def command(text):
    s.sendall(text.encode() + b"\n")
    while True:
        line = stream.readline().strip()
        if line.startswith(("OK", "ERR")):
            return line                 # this is the reply
        # otherwise it is an IMU,/CAL, event line: keep reading

print(command("GET"))
print(command("ACT 120 60"))
```

Three rules cover almost every mistake:

1. **One reply per command.** `IMU,...` and `CAL,...` lines are events, not
   replies; skip them while waiting.
2. **Your socket may stall.** Never block the robot: enable streaming with
   `STREAM <hz>` and drain it, or leave streaming off and poll `GET`.
3. **Angles are clamped, not rejected.** `ACT` answers with the targets the
   robot actually accepted.

Language guides with runnable snippets live in [docs/languages/](docs/languages/README.md).

## Repository layout

```text
src/main.cpp            boot sequence and the main loop
lib/
  IMU/                  MPU9250 driver + PX4-ported attitude/calibration stack
  ServoControl/         ramped, soft-limited servo driver
  Display/              OLED driver
  CommandServer/        the serial + TCP protocol
  Network/              WiFi AP, OTA, robot identity
  HealthCheck/          boot self-test for the servos
  I2CBus/               shared-bus mutex (display + IMU)
  SystemWatchdog/       hardware task watchdog
include/config.h        pins, rates, limits, tuning
test/host/              math + protocol tests that run on a PC
tools/                  robot visualizer, log analysis, tests
examples/python/        reference client and runnable scripts
docs/                   see below
```

## Documentation

| Document | Read it when you want to… |
|---|---|
| **[docs/PROTOCOL.md](docs/PROTOCOL.md)** | implement a client: full command grammar, fields, errors |
| [docs/QUICKSTART.md](docs/QUICKSTART.md) | flash the robot and send your first command |
| [docs/BUILD.md](docs/BUILD.md) | build, flash, OTA, wiring, troubleshooting |
| [docs/CALIBRATION.md](docs/CALIBRATION.md) | calibrate the IMU and fix failed calibrations |
| [docs/HARDWARE.md](docs/HARDWARE.md) | wiring, pin map, power, 3D parts, PCB, bill of materials |
| [docs/PROJECT.md](docs/PROJECT.md) | what this robot is for, project phases, who does what |
| [docs/SERVO_SAFETY.md](docs/SERVO_SAFETY.md) | how the legs are protected |
| [docs/SERVO_HEALTH.md](docs/SERVO_HEALTH.md) | interpret the boot servo self-test |
| [docs/imu/](docs/imu/README.md) | the IMU/AHRS design in depth |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | firmware internals: tasks, buses, logging, failure modes |
| [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) | repo conventions, tests, release process |
| [docs/PYPI.md](docs/PYPI.md) | publish the `crawlingbot` Python package to PyPI |
| [docs/MIGRATION.md](docs/MIGRATION.md) | history: Arduino → PlatformIO, student-code → client |
| [tools/README.md](tools/README.md) | watch the robot live: dashboard, health panel, command buttons |
| [docs/languages/](docs/languages/README.md) | client snippets per language |

## Tests

```bash
test/host/run.sh                        # C++ math + protocol, no hardware
cd examples/python && python -m unittest discover -s tests   # client
```

## Hardware at a glance

ESP32-S3-DevKitM-1 · MPU9250 9-DOF IMU · SSD1306 128×64 OLED · 2× hobby servo.
Pins, power and mechanical parts: [docs/HARDWARE.md](docs/HARDWARE.md).

## Status

Firmware complete and stable; see [CHANGELOG.md](CHANGELOG.md).
Mechanical files (3D print bodies, PCB, component spreadsheet) are tracked in
[docs/hardware/](docs/hardware/README.md) and are being added incrementally.

---

MIT License · Built with PlatformIO
