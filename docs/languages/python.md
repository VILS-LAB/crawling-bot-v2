# Python

The reference client is in [`examples/python`](../../examples/python/README.md)
and is covered by tests. Prefer it over writing a client from scratch.

```bash
cd examples/python
python hello_robot.py --host 192.168.4.1
python walk_legs.py  --host 192.168.4.1
python stream_telemetry.py --csv run.csv --seconds 30
```

Using the library directly:

```python
from crawlingbot import Robot

with Robot.tcp("192.168.4.1") as bot:
    print(bot.ping())
    snap = bot.snapshot()
    print(snap.roll, snap.pitch, snap.yaw, snap.calibrated)
    bot.act(120, 60)

# serial instead of TCP (needs pyserial)
# with Robot.serial("/dev/cu.usbmodem1101") as bot: ...
```

Streaming:

```python
with Robot.tcp() as bot:
    for frame in bot.frames(hz=50, count=500):
        print(frame.roll, frame.pitch, frame.yaw, frame.still)
```

## A client with no dependencies at all

If you would rather not use the package:

```python
import socket

s = socket.create_connection(("192.168.4.1", 4242), timeout=5)
reader = s.makefile("r")

def command(text):
    s.sendall(text.encode() + b"\n")
    while True:
        line = reader.readline().strip()
        if line.startswith(("OK", "ERR")):
            return line
        # IMU,/CAL, event line: keep reading

print(command("GET"))
print(command("ACT 120 60"))
```

## Notes

* `Robot` serializes commands; use one `Robot` per thread.
* Telemetry frames keep their order; a `frames()` iterator disables streaming
  when it finishes, even on an exception.
* Calibration blocks for minutes; pass `on_progress=` to show stages.
