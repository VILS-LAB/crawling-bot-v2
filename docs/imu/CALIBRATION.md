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

The firmware stores a full 3×3 accelerometer correction matrix, including
off-diagonal terms. This differs from the diagonal correction retained by PX4's
sensor-calibration path and can represent cross-axis coupling in the assembled
breakout; it does not compensate for arbitrary board-to-chassis rotation.

## Gyro: mean and quality checks over a still window

Each attempt settles for 1.5 s, then collects up to 2500 accepted samples while
the robot is still (at least 2000 are required). Samples with acceleration
outside 0.85–1.15 g or gyro magnitude above 8 °/s are skipped. Success prints
`CAL,gyro,bias=x,y,z`.

The firmware rejects an attempt if any per-axis standard deviation exceeds
0.20 °/s or if the rolling median of the **last three accepted samples** differs
from that axis's run mean by more than 0.6 °/s. The rolling median is not a
median over the full run and is not a reliable detector of every slow or
constant rotation, so keep the robot still for the complete stage.

## Accelerometer: six-position solve

Six orientations, one per face. For each, the firmware waits until the
orientation is correct before collecting.

| Step | Prompt | Reference vector (g) |
|---|---|---|
| 1 | `X+` (sensor/body +X points up) | `( 1, 0, 0)` |
| 2 | `X-` (sensor/body -X points up) | `(-1, 0, 0)` |
| 3 | `Y+` (sensor/body +Y points up) | `( 0, 1, 0)` |
| 4 | `Y-` (sensor/body -Y points up) | `( 0,-1, 0)` |
| 5 | `Z+` (sensor/body +Z points up) | `( 0, 0, 1)` |
| 6 | `Z-` (sensor/body -Z points up) | `( 0, 0,-1)` |

The board is mounted rotated relative to the chassis. Follow the sensor-axis
prompt, not chassis-face names; the firmware does not apply a separate
sensor-to-chassis rotation.

Acceptance per position:

* the named sensor/body axis must be dominant and have the requested sign,
* hold it **still** — the calibrated gyro magnitude must stay under 3 °/s,
* raw `|accel|` must be within 0.85–1.15 g and the requested axis magnitude at
  least 0.75 g,
* 180 samples are averaged, at least 10 ms apart; each face has a 120 s timeout.

The six averaged vectors are then solved for `accelBias` and an `accelMatrix`.
Success prints both plus the residual:

```
CAL,accelerometer,bias=...
CAL,accelerometer,matrix=...
CAL,accelerometer,rms_g=0.00412
```

`rms_g` is a fit diagnostic, not a firmware acceptance threshold. Lower is
better; a large residual can indicate movement during a step, incorrect face
placement, or non-ideal sensor behavior. The calibration routine does not
reject a fit based on this RMS value.

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

*   The fitted sphere radius must be 20–70 µT or the sphere fit is rejected. The
  Earth's field is commonly around 25–65 µT, but nearby interference changes the
  local field; the fit window is only a plausibility check.* The RMS is a fit diagnostic, not a firmware acceptance gate. Lower is
  generally better; a high residual suggests poor orientation coverage or
  magnetic interference and is a reason to repeat the fit in a cleaner area.
* `fit=ellipsoid` means the ellipsoid refinement improved on the sphere fit,
  which is what you want on a hand-soldered board.

## Verifying

Use `GET` (or the Python client's `snapshot()`):

```text
GET
OK SNAP,id=1,...,cal=1,d=180,dt=180,u=180,ut=180,act_moving=0
```

The `IMU,` stream carries calibration residuals and live diagnostic fields;
`GET` is a separate, smaller snapshot. Refer to [../PROTOCOL.md](../PROTOCOL.md)
for the stream setup and [ARCHITECTURE.md §9](./ARCHITECTURE.md#9-telemetry-contract)
for column meanings. Residual values describe the fit and are not firmware
acceptance gates. Check the calibrated field magnitude against the local
expectation; magnetic interference can still affect heading even after a fit.

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
