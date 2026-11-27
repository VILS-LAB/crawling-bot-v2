# Development tools

Diagnostics for the robot while you work on the firmware. These are **not**
needed to use the robot — for that, use the client library in
[`examples/python`](../examples/python/README.md).

All of them read the same `IMU,` CSV that the firmware streams, and they accept
`--demo` (synthetic data) and `--replay <csv>` (a recorded run), so you can
develop without hardware.

| Tool | What it does |
|---|---|
| **[`robot_visualizer.py`](robot_visualizer.py)** | **start here.** Robot dashboard + command console: every telemetry signal, 3D orientation, heading dial, pass/fail health panel and a button for every protocol command. Serial, TCP, replay or demo |
| [`imu_visualizer_v2.py`](imu_visualizer_v2.py) | *legacy* IMU-only dashboard, superseded by the robot visualizer. Kept because it plots raw and filtered gyro side by side and has an interactive slider panel; nothing in this repo depends on it |
| [`imu_visualizer.py`](imu_visualizer.py) | *legacy* compact dashboard, superseded |
| [`imu_event_based_plot.py`](imu_event_based_plot.py) | event/still-segment analysis, posts a table and writes CSVs |
| [`qlearn_visualizer.py`](qlearn_visualizer.py) | Q-table viewer for a *separate* RL firmware that emits `Q,` lines; kept for reference, does not work against this firmware |
| [`test_robot_viewer.py`](test_robot_viewer.py) | unit tests for the visualizer (columns, ring buffer, panels, health, console, geometry, rendering) |
| [`test_viewer_geometry.py`](test_viewer_geometry.py) | unit tests for the shared geometry helpers |

## Robot visualizer

```bash
python tools/robot_visualizer.py --demo            # synthetic robot, no hardware
python tools/robot_visualizer.py --tcp             # 192.168.4.1:4242 (robot's AP)
python tools/robot_visualizer.py --serial auto     # find the ESP32 over USB
python tools/robot_visualizer.py --serial /dev/cu.usbmodem1101
python tools/robot_visualizer.py --replay run.csv  # a recorded run
```

It **asks the robot to stream on its own** (`STREAM 50`), because the robot will
not volunteer data: a new TCP client starts with the stream off, and the serial
port runs whatever rate the last session left behind. If frames are already
arriving, the viewer adopts that rate instead of overriding it. Use
`--stream-hz <n>` to change the request, or `--stream-hz 0` to never ask.

Needs `numpy`, `matplotlib` and, only for `--serial`, `pyserial`:

```bash
pip install numpy matplotlib pyserial
```

### What it shows

The window is a **left column of six consolidated charts** plus a **right column
instrument cluster**. The old visualizer spent twelve charts on this data and ran
out of room for the things you actually watch while testing, so related signals
now share one axis.

| Chart | Consolidates | Reading it |
|---|---|---|
| Linear acceleration | 3 charts → 1 | the three axes share a unit and a scale |
| Gyroscope | raw + filtered | filtered solid, raw faint behind it |
| Attitude + drift | attitude and its gyro-only shadow | the *drift* line (attitude − shadow) is the signal, not the pair |
| State | trust / still / moving + the 6-row failure bar | shows *why* the robot is not still, next to the verdict |
| Velocity + speed | displacement and speed | |
| Bias + health | estimator bias and calibration/sampling health | split in two, because deg/s and g/uT/Hz cannot share an axis |

The right column holds the **3D body**, a **heading dial** (0° = north,
clockwise), the **health checklist**, live **state text**, and the **command
console**.

### Health panel

Nine rows, each `✓` / `!` / `✗` / `·` (unknown):

`Robot answering`, `Identity / network`, `Telemetry stream`, `IMU sample rate`,
`Attitude estimator`, `Calibration`, `Servo self-test`, `Sensor sanity`,
`Protocol handshake`.

Two sources feed it: the robot's own boot log (`HEALTH,`, `CAL,`, `PROTOCOL,`
lines) and checks the viewer makes on the live stream. A row only turns red when
the viewer can say *what* is wrong, and the headline summarises the worst row —
so a green panel means the robot is genuinely healthy, not merely connected.

### Command console

One button per protocol command: `PING`, `ID`, `GET`, `SETID` (with a number
box), `STREAM 50`, `STREAM 0`, `ACT` (with two angle sliders), `STOP`, `PARK`
and `CAL all`. Each button sends the command and shows the robot's reply beside
it, so the console doubles as a protocol exerciser.

Only one command is in flight at a time; the button greys out until the reply
arrives or the request times out. `CAL all` is marked blocking and walks through
the three stages (gyro → accel → mag) from the `CAL,` progress lines.

If a command answers with **`TIMEOUT (no reply)`** while the robot clearly obeyed
it, the reply is being lost between the robot and the console — see
*troubleshooting* below.

### Reading the numbers

Two rates matter and they are not the same:

* **`imu`** — how fast the IMU task is sampling (should be ~200 Hz). This is the
  robot's own health.
* **`rx`** — how fast frames actually arrive at the viewer. This is what the
  *link* allows. Measured on a USB-CDC S3: 50 Hz requested arrives at ~50 Hz,
  so a low `rx` means something else is competing for the port (see below).

`STREAM <hz>` only *requests* a rate; a 440-byte frame at 50 Hz needs ~220 kB/s.
When `rx` sits well below the requested rate the panel says so
(`12 Hz of 50 requested`) and points at the transport. Fixes, in order of effect:

1. lower the rate — `STREAM 20` is usually plenty for watching a robot;
2. use TCP instead of USB serial (WiFi has more headroom than a busy USB bus);
   USB-CDC ignores the baud rate, so `--baud` is not a lever there;
3. close other programs holding the port (`screen`, Arduino IDE monitor, a
   previous `pio device monitor`) — two readers split the bytes and both see
   corrupted frames.

### Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| every command says `TIMEOUT (no reply)` but the servos move | the viewer lost the reply | fixed in viewer 1.0.1: replies and diagnostics are no longer parsed as telemetry |
| moving a slider never changes where the leg goes (always 90/90) | the buttons captured the slider values once at startup | fixed in viewer 1.0.2: `ACT` reads the sliders when you click it |
| a legacy `imu_visualizer*.py` shows nothing | the robot only streams when asked, and it remembers the last `STREAM` per transport, so an earlier `STREAM 0` leaves the port silent. The legacy tools never sent it | they now send `STREAM` on connect (`--hz`, default 50); the new viewer always has |
| `rx` far below the requested rate | the link, not the robot | see *Reading the numbers* |
| an empty window over TCP while `nc 192.168.4.1 4242` works | a fresh TCP client is accepted with streaming **off** | fixed in viewer 1.0.3: the viewer sends `STREAM` on connect |
| an empty window over serial | a previous session left the port at `STREAM 0` | same fix; or press `STREAM 50` |
| frames stop when the window is busy | the robot drops a stalled client on purpose | keep the window on screen; lower `--fps` if the machine struggles |
| `no serial port found` | nothing plugged in, or the port is busy | `--list-ports` marks the likely robot |

### Keys

| Key | Action |
|---|---|
| `space` | pause the display (data keeps recording) |
| `[` `]` | shorter / longer time window |
| `m` | cycle the Y-range preset: overview → detail → auto |
| `z` | zero the top-down position |

`--log` records the stream to `logs/*.csv` in the firmware's own column format,
so a session can be replayed or opened by the other tools.

## Seeing the raw stream

```text
STREAM 50
OK stream=50
IMU,t_ms,dt,ax,ay,az,...
IMU,...
```

Column order is documented in [docs/PROTOCOL.md](../docs/PROTOCOL.md#stream) and
is append-only: new columns appear at the end so older tools keep working.
