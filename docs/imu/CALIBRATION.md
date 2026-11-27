# IMU calibration: the math and the fits

How the calibration model works and what each stage computes. For the
**procedure** (when to calibrate, how to run it, what a failure means) see
[../CALIBRATION.md](../CALIBRATION.md). For the *why* behind each step in the
pipeline, see [ARCHITECTURE.md §4](./ARCHITECTURE.md#4-calibration).

## What the model is

```
accel_corrected = accelMatrix * (accel_raw - accelBias)   full 3x3, units of g
gyro_corrected  = gyro_raw - gyroBias                     deg/s
mag_corrected   = magMatrix * (mag_raw - magOffset)       uT
```

PX4 throws away the off-diagonal accelerometer terms; this project keeps them
because the GY-91 breakout is hand-soldered and its axes are not perfectly
orthogonal.

## Gyro: mean over a still window

The stage settles for 1.5 s, then collects up to 2500 samples while the robot is
still. Success prints `CAL,gyro,bias=x,y,z`.

The acceptance check is PX4's: per-axis standard deviation must be ≤ 0.20 °/s **and** the
median must be within 0.6 °/s of the mean. The median check catches a slow,
steady rotation that a variance test alone would miss.

## Accelerometer: six-position solve

Six orientations, one per face. For each, the firmware waits until the
orientation is correct before collecting.

| Step | Face up | Reference vector |
|---|---|---|
| 1 | X+ (nose up) | ( g, 0, 0) |
| 2 | X- (nose down) | (-g, 0, 0) |
| 3 | Y+ (left down) | ( 0, g, 0) |
| 4 | Y- (right down) | ( 0,-g, 0) |
| 5 | Z+ (upside down) | ( 0, 0, g) |
| 6 | Z- (upright) | ( 0, 0,-g) |

Acceptance per position:

* hold it **still** — `|gyro|` must stay under 3 °/s,
* the dominant axis must match and have the right sign,
* `|accel|` must be within 0.85–1.15 g,
* 180 samples are averaged.

The six averaged vectors are then solved for `accelBias` and an `accelMatrix`.
Success prints both plus the residual:

```
CAL,accelerometer,bias=...
CAL,accelerometer,matrix=...
CAL,accelerometer,rms_g=0.00412
```

`rms_g` below ~0.02 g is good. Higher means the robot moved during a step or
the mounting is loose.

## Magnetometer: sphere then ellipsoid

A 20 s rotating sample cloud is fitted with Levenberg-Marquardt — a sphere
first, then an ellipsoid seeded from the sphere result. Samples are prompted
apart by at least 3 µT so one orientation cannot dominate the fit.

Success prints:

```
CAL,magnetometer,samples=...
CAL,magnetometer,sphere_radius=47.2,rms=0.83
CAL,magnetometer,ellipsoid_rms=0.41
CAL,magnetometer,offset=...
CAL,magnetometer,fit=ellipsoid
```

* `sphere_radius` should be 20–70 µT (Earth's field is ~25–65 µT). Outside that
  range the fit is rejected.
* `rms` below ~1 µT is good. Above ~2 µT, redo it somewhere with less
  interference.
* `fit=ellipsoid` means the ellipsoid refinement improved on the sphere fit,
  which is what you want on a hand-soldered board.

## Verifying

Use `GET` (or the Python client's `snapshot()`):

```text
GET
OK SNAP,id=1,...,cal=1,d=180,dt=180,u=180,ut=180,act_moving=0
```

The telemetry stream carries the residuals:

| Field | Good value | Meaning |
|---|---|---|
| `acc_norm_g` | 0.99–1.01 | accelerometer magnitude after correction |
| `gyro_norm_dps` | < 0.1 | residual rate, bias removed |
| `mag_norm` | 25–65 µT | local field magnitude |
| `rate_hz` | ~200 | measured IMU sampling rate |
| `accel_rms_g` | < 0.02 | six-position fit residual |
| `mag_rms_ut` | < 2 | sphere/ellipsoid fit residual |
| `mag_fit` | 1 or 2 | 1 = sphere, 2 = ellipsoid |

Behavioural check, with live telemetry:

* level and still → `roll ≈ pitch ≈ 0`,
* roll the robot 30° → `roll ≈ 30` within ~2 s,
* yaw it by hand → `yaw` changes smoothly.

## Storage

Calibration is stored in NVS (`Preferences`, namespace `imu-cal`) as one blob
per robot number (`r1` … `r8`), so eight robots can share the firmware. The
record carries a magic and a version; a version mismatch is ignored, and the
robot starts uncalibrated rather than using stale data.

There is no separate "erase" command: rerunning `CAL` overwrites the record, and
changing the robot number with `SETID` makes the robot look up a different key.

## Troubleshooting

See the table in [../CALIBRATION.md](../CALIBRATION.md#troubleshooting), which
covers every `CAL,` failure message.
