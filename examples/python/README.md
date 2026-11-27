# Python client for the Crawling Bot

A small, dependency-light client for the robot's serial/TCP protocol. Use it as
the reference implementation when you write your own client in another language.

* **Standard library only** for TCP; `pyserial` is needed only for `--serial`.
* Typed results (`Snapshot`, `ImuFrame`) instead of raw strings.
* Turns "one reply per command" into ordinary Python calls, and skips the
  telemetry/calibration event lines for you.
* Every call has a deadline, so a robot that stops answering raises
  `RobotTimeout` instead of hanging.

## Install

Nothing to install for TCP use — copy the folder or add it to `PYTHONPATH`:

```bash
cd examples/python
python hello_robot.py --host 192.168.4.1
```

For serial:

```bash
pip install pyserial
python hello_robot.py --serial /dev/cu.usbmodem1101 --list-ports
```

## Five-minute tour

```python
from crawlingbot import Robot

with Robot.tcp("192.168.4.1") as bot:
    print(bot.ping())            # uptime in ms
    print(bot.id_info())         # {'id': '1', 'ssid': 'ESP32-AP-1', ...}

    snap = bot.snapshot()        # one GET
    print(snap.roll, snap.pitch, snap.yaw)
    print(snap.calibrated, snap.down, snap.down_target)

    bot.act(120, 60)             # clamped targets come back
    bot.stop()
    bot.park()
```

Streaming, with automatic cleanup:

```python
with Robot.tcp() as bot:
    for frame in bot.frames(hz=50, count=500):
        print(frame.roll, frame.pitch, frame.yaw)
```

Calibration:

```python
with Robot.tcp() as bot:
    ok = bot.calibrate("all", on_progress=print)
```

## Scripts in this folder

| Script | What it does |
|---|---|
| `hello_robot.py` | connect and print identity + one snapshot |
| `walk_legs.py` | move the legs through a routine, waiting for each move |
| `stream_telemetry.py` | log or print telemetry frames (`--csv` for a file) |
| `calibrate.py` | run the guided calibration with live stage output |

## Tests

The tests run against a fake robot that emits the firmware's exact strings, so
no hardware is needed:

```bash
cd examples/python
python -m unittest discover -s tests -v
```

`tests/fake_robot.py` is also useful on its own: it lets you develop a client in
any language without a robot. Start it from Python and point your client at the
printed port.

## API summary

| Method | Protocol command | Notes |
|---|---|---|
| `ping()` | `PING` | returns uptime in ms |
| `id_info()` | `ID` | raw key/value dict |
| `robot_id` | `ID` | property, int |
| `set_id(n)` | `SETID n` | validated 1–8 |
| `snapshot()` | `GET` | returns `Snapshot` |
| `stream(hz)` | `STREAM hz` | 0 turns it off, max 50 |
| `frames(...)` | `STREAM` | iterator over `ImuFrame` |
| `act(down, up)` | `ACT d u` | returns the clamped targets |
| `stop()` / `park()` | `STOP` / `PARK` | |
| `calibrate(target, on_progress=)` | `CAL [all\|gyro\|accel\|mag]` | blocks; returns bool |

Anything the client does not wrap is still reachable:

```python
bot.command("GET")       # raw key/value dict, raises RobotError on ERR
bot.command("CAL gyro")
```

See [../../docs/PROTOCOL.md](../../docs/PROTOCOL.md) for the wire format.
