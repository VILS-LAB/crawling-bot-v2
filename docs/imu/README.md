# IMU documentation

Start here.

| Document | Read it when you want to… |
|---|---|
| [ARCHITECTURE.md](./ARCHITECTURE.md) | understand the whole signal chain, the frame conventions, the design decisions and the known limitations |
| [CALIBRATION.md](./CALIBRATION.md) | actually calibrate the robot, or debug a failed calibration |
| [ROTATION_MATRIX.md](./ROTATION_MATRIX.md) | know exactly what `rotateBodyToWorld()` does and why it is frozen |
| [PX4_PORT.md](./PX4_PORT.md) | audit what came from PX4, what differs, and what was left out |
| [IMPLEMENTATION_HISTORY.md](./IMPLEMENTATION_HISTORY.md) | compare the current stack against the previous one and against the original `main` branch, and see why each rewrite happened |

## TL;DR

* The IMU stack is a **port of PX4's IMU path**: PX4 filters, PX4
  `attitude_estimator_q`, PX4 calibration math.
* The project's **body→world rotation is unchanged** — it is the validated
  contract, and the estimator reports Euler angles in the same convention.
* Sampling runs in a **dedicated FreeRTOS task on core 1**, so OTA and the OLED
  cannot disturb the estimator's `dt`.
* Calibration is **guided** (gyro still, six accel positions, 20 s mag
  rotation) and stored per robot number in NVS.
* Telemetry is **append-only CSV** with a header line, so both visualizers keep
  working across firmware revisions.
* This is the **third** generation of the IMU stack. See
  [IMPLEMENTATION_HISTORY.md](./IMPLEMENTATION_HISTORY.md) for what the previous
  two got wrong and what changed.

## Quick commands

```bash
# firmware
pio run -e esp32-s3-devkitm-1

# host math tests (no hardware, ~1 s)
test/host/run.sh

# visualizers
python3 tools/imu_visualizer.py    /dev/cu.usbmodem1101
python3 tools/imu_visualizer_v2.py --port auto
```

## Conventions cheat sheet

```text
body  : X forward, Y left,  Z up
world : X forward, Y left,  Z up
Euler : R = Rz(yaw) * Ry(pitch) * Rx(roll)
quat  : body -> world
units : accel m/s^2, gyro deg/s, mag uT, angles deg, position m
```
