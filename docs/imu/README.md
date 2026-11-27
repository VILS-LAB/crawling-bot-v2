# IMU documentation

Start here.

| Document | Read it when you want to… |
|---|---|
| [ARCHITECTURE.md](./ARCHITECTURE.md) | understand the whole signal chain, the frame conventions, the design decisions and the known limitations |
| [CALIBRATION.md](./CALIBRATION.md) | understand the calibration math and the fits (the *procedure* is in [../CALIBRATION.md](../CALIBRATION.md)) |
| [ROTATION_MATRIX.md](./ROTATION_MATRIX.md) | know exactly what `rotateBodyToWorld()` does and why it is frozen |
| [PX4_PORT.md](./PX4_PORT.md) | audit what came from PX4, what differs, and what was left out |
| [IMPLEMENTATION_HISTORY.md](./IMPLEMENTATION_HISTORY.md) | compare the current stack against the previous one and against the original `main` branch, and see why each rewrite happened |

> **Where this fits.** These pages document the sensing half of the robot. The
> student-facing interface is the protocol ([../PROTOCOL.md](../PROTOCOL.md)); the
> firmware around these modules is in [../ARCHITECTURE.md](../ARCHITECTURE.md).

## TL;DR

* The firmware uses the MPU9250's accelerometer, gyroscope and AK8963
  magnetometer. Its calibration fits, filters and quaternion attitude estimator
  are adapted from PX4; the complete PX4 runtime is not present.
* The estimator's firmware frame and the sensor's physical mounting must not be
  conflated. Firmware coordinates are body `+X forward, +Y left, +Z up`, while
  this board is mounted with sensor/body `+X` toward chassis right and `+Y`
  toward chassis forward. Consequently yaw is not the chassis compass heading;
  see [ARCHITECTURE.md §2](./ARCHITECTURE.md#2-frame-conventions-read-this-before-touching-any-math).
* Sampling and estimation run in a dedicated FreeRTOS task pinned to core 1.
  The I2C transaction is shared with the OLED under a mutex; estimation itself
  runs after the bus lock is released.
* Boot only checks a complete stored calibration; it does not start guided
  calibration. `CAL` runs gyro, six-position accelerometer and magnetometer
  stages in order. Calibration is stored per robot number in NVS.
* CSV telemetry is emitted by the command server (USB serial defaults to 10 Hz;
  TCP clients start quiet), and contains filtered diagnostic fields alongside
  the unfiltered values used by the estimator. See [PROTOCOL.md](../PROTOCOL.md).
* Host tests cover math, filters, calibration fits, estimator behavior, servo
  motion and protocol parsing. They do not validate the physical sensor,
  mounting or live calibration procedure.

This is the third generation of the stack. [IMPLEMENTATION_HISTORY.md](./IMPLEMENTATION_HISTORY.md)
keeps earlier design context; [ARCHITECTURE.md](./ARCHITECTURE.md) describes
current behavior and is the primary implementation guide.

## Source of truth

For current behavior, use these files in this order rather than historical
counts or prose summaries:

| Question | Implementation |
|---|---|
| Sensor setup, task order, calibration orchestration and runtime processing | [`AHRS.cpp`](../../lib/IMU/AHRS.cpp), [`AHRS.h`](../../lib/IMU/AHRS.h) |
| Estimator equations and defaults | [`ImuAttitudeEstimator.cpp`](../../lib/IMU/ImuAttitudeEstimator.cpp), [`ImuAttitudeEstimator.h`](../../lib/IMU/ImuAttitudeEstimator.h) |
| Calibration model, fits and NVS record validation | [`ImuCalibration.cpp`](../../lib/IMU/ImuCalibration.cpp), [`ImuCalibration.h`](../../lib/IMU/ImuCalibration.h) |
| Rotation contract | [`ImuRotation.cpp`](../../lib/IMU/ImuRotation.cpp) |
| Filter implementations | [`ImuFilters.cpp`](../../lib/IMU/ImuFilters.cpp), [`AHRS.h`](../../lib/IMU/AHRS.h) (cutoffs) |
| Sample ranges, bus, task placement and stream settings | [`config.h`](../../include/config.h) and `AHRS.h` |
| CSV columns and format | [`ImuTelemetry.cpp`](../../lib/IMU/ImuTelemetry.cpp) |
| Protocol and stream routing | [`CommandServer.cpp`](../../lib/CommandServer/CommandServer.cpp), [PROTOCOL.md](../PROTOCOL.md) |
| Host math/logic coverage | [`test/host/README.md`](../../test/host/README.md), [`test_imu_math.cpp`](../../test/host/test_imu_math.cpp) |

## Quick commands

```bash
# firmware
pio run -e esp32-s3-devkitm-1

# host math tests (no hardware, ~1 s)
test/host/run.sh

# visualizers
python3 tools/imu_visualizer.py    /dev/cu.usbmodem1101
python3 tools/imu_visualizer_v2.py --port auto

# robot console (serial or TCP :4242, same commands)
PING, ID, GET, ACT <down> <up>, STOP, PARK, STREAM <hz>, CAL
```

## Conventions cheat sheet

```text
firmware body : X forward, Y left, Z up (FLU)
firmware world: X north,   Y west, Z up
rotation      : Rz(yaw) * Ry(pitch) * Rx(roll)
quaternion    : body -> world
units         : accel m/s^2, gyro deg/s, mag uT, angles deg, position m
```

The physical sensor mounting is rotated relative to the chassis axes; it is
explained in [ARCHITECTURE.md §2](./ARCHITECTURE.md#2-frame-conventions-read-this-before-touching-any-math).
