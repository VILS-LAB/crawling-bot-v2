# Examples

Runnable code for working with the robot. **Client** examples are what a student
project uses; **firmware** examples are for maintainers extending the robot.

## Client examples (students)

Start here. The robot is driven entirely through the protocol
([PROTOCOL.md](PROTOCOL.md)), so these examples work from any machine on the
robot's WiFi.

| Example | Language | Shows |
|---|---|---|
| [`examples/python/hello_robot.py`](../examples/python/hello_robot.py) | Python | connect, read identity and one snapshot |
| [`examples/python/walk_legs.py`](../examples/python/walk_legs.py) | Python | send actions, wait for the legs to settle |
| [`examples/python/stream_telemetry.py`](../examples/python/stream_telemetry.py) | Python | stream frames to screen or CSV |
| [`examples/python/calibrate.py`](../examples/python/calibrate.py) | Python | guided calibration with live stages |
| [languages/cpp.md](languages/cpp.md) | C++ | raw socket client, no dependencies |
| [languages/javascript.md](languages/javascript.md) | Node | `net`-based client with promise replies |
| [languages/matlab.md](languages/matlab.md) | MATLAB | `tcpclient` session for quick experiments |

### Observe → decide → act

The shape of every controller, regardless of language or algorithm:

```python
from crawlingbot import Robot

with Robot.tcp("192.168.4.1") as bot:
    while True:                       # your loop, with your own stop condition
        snap = bot.snapshot()         # 1. observe
        action = policy(snap)         # 2. decide (your code)
        bot.act(*action)              # 3. act
```

What a reward function might look at, from the same snapshot:

| Field | Meaning | Typical use |
|---|---|---|
| `roll`, `pitch` | body tilt, degrees | penalise falling over |
| `accel` | body acceleration, m/s² | detect a step or an impact |
| `still` | motion detector | reward progress, reject noise |
| `velocity` | estimated speed, m/s | reward forward motion |
| `down`, `up` | current leg angles | detect a completed step |

### Logging a run for offline learning

```python
import csv
from crawlingbot import Robot

with Robot.tcp() as bot, open("run.csv", "w", newline="") as handle:
    writer = csv.writer(handle)
    writer.writerow(["t_ms", "roll", "pitch", "yaw", "vx", "vy", "down", "up", "action"])
    for frame in bot.frames(hz=50, count=3000):
        snap = bot.snapshot()
        action = policy(frame)                 # your decision
        bot.act(*action)
        writer.writerow([frame.timestamp_ms, frame.roll, frame.pitch, frame.yaw,
                         *frame.velocity, snap.down, snap.up, action])
```

### Driving the visualizers over TCP

`tools/imu_visualizer*.py` read the same `IMU,` CSV the robot streams, so a
network bridge is enough if a tool only speaks serial:

```bash
# stream to a file and replay it
python examples/python/stream_telemetry.py --seconds 30 --csv run.csv
python tools/imu_visualizer_v2.py --replay run.csv
```

## Firmware examples (maintainers)

> Only needed if you are changing the robot itself. Students should not edit
> these.

### Screen pages

```cpp
void showTelemetry()
{
    const ImuTelemetry &imu = ahrs.getTelemetry();
    display.clear();
    display.print(ahrs.isMoving() ? "MOVING" : "STILL", 0, 0);
    display.setCursor(0, 16);
    display.print("R:");
    display.printFloat(imu.roll, 1);
    display.print(" P:");
    display.printFloat(imu.pitch, 1);
    display.refresh();
}
```

### Adding a boot step

```cpp
// src/main.cpp
showBootMessage("Checking battery...");
if (!batteryOk())
{
    Serial.println("WARN,BATTERY_LOW");
    // Do not halt: the robot must stay reachable.
}
```

### Reading sensor state from firmware

```cpp
ahrs.update();                       // copy the newest snapshot
const ImuTelemetry &imu = ahrs.getTelemetry();
Serial.printf("roll=%.2f pitch=%.2f yaw=%.2f trust=%.3f\n",
              imu.roll, imu.pitch, imu.yaw, imu.trust);
```

### Adding a protocol command

See [DEVELOPMENT.md](DEVELOPMENT.md#adding-a-protocol-command) — it lists every
file that must change, including the tests and the Python client.

## See also

* [PROTOCOL.md](PROTOCOL.md) — the interface these examples use
* [languages/](languages/README.md) — per-language guides
* [PROJECT.md](PROJECT.md) — how the examples fit into the course phases
