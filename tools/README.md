# Development tools

Diagnostics for the robot while you work on the firmware. These are **not**
needed to use the robot — for that, use the client library in
[`examples/python`](../examples/python/README.md).

All of them read the same `IMU,` CSV that the firmware streams, and they accept
`--demo` (synthetic data) and `--replay <csv>` (a recorded run), so you can
develop without hardware.

| Tool | What it does |
|---|---|
| [`imu_visualizer.py`](imu_visualizer.py) | compact dashboard: accel, gyro, velocity, attitude vs. gyro-only shadow, 3D body, top-down path |
| [`imu_visualizer_v2.py`](imu_visualizer_v2.py) | full dashboard: per-panel ranges, still-failure breakdown, estimator bias, calibration health, logging and replay |
| [`imu_event_based_plot.py`](imu_event_based_plot.py) | event/still-segment analysis, posts a table and writes CSVs |
| [`qlearn_visualizer.py`](qlearn_visualizer.py) | Q-table viewer for a *separate* RL firmware that emits `Q,` lines; kept for reference, does not work against this firmware |
| [`test_viewer_geometry.py`](test_viewer_geometry.py) | unit tests for the viewer's geometry helpers |

## Typical use

```bash
# live from a serial port
python tools/imu_visualizer_v2.py --port auto

# record over the network, then replay offline
python examples/python/stream_telemetry.py --seconds 30 --csv run.csv
python tools/imu_visualizer_v2.py --replay run.csv

# no robot at all
python tools/imu_visualizer_v2.py --demo
```

## Seeing the raw stream

```text
STREAM 50
OK stream=50
IMU,t_ms,dt,ax,ay,az,...
IMU,...
```

Column order is documented in [docs/PROTOCOL.md](../docs/PROTOCOL.md#stream) and
is append-only: new columns appear at the end so older tools keep working.
