# Calibrating the robot

The MPU9250 needs a per-robot calibration: every board has its own accelerometer
offset, gyro bias and magnetic distortion. The firmware stores it in NVS, keyed
by robot number, and **verifies** it at boot.

> This page is the practical procedure. For the math and the PX4-derived fitting
> routines, see [imu/CALIBRATION.md](imu/CALIBRATION.md).

## When to calibrate

* A new board, or after replacing/remounting the IMU.
* When the robot boots with `Uncalibrated - run CAL` on the OLED.
* When `GET` reports `cal=0`.
* After the robot takes a hard hit, or when yaw drifts quickly while still.
* When a robot is moved to a very different magnetic environment (a new lab).

## What boot does (and does not do)

Boot never blocks waiting for a human and never starts a full calibration:

```text
load calibration -> verify (2 s warm-up + 200 samples) -> valid?
                                                          ├─ yes -> Ready
                                                          └─ no  -> servos parked,
                                                                    "Uncalibrated - run CAL"
```

A robot with `cal=0` still boots, answers `PING`/`ID`/`GET` and accepts OTA.
Boot parks the servos and skips the servo health motion test, but the current
`ACT` command handler does not enforce calibration state; an explicit movement
command can still move them. Leave the robot parked until calibration has
passed, and follow the calibration instructions before normal operation.

## Running the calibration

Over TCP (recommended) or serial — same commands:

```bash
cd examples/python
python calibrate.py                    # all three sensors
python calibrate.py --target gyro      # one stage
```

Or by hand:

```text
CAL              -> OK CAL sensors=all
CAL gyro         -> OK CAL sensors=gyro
CAL accel        -> OK CAL sensors=accel
CAL mag          -> OK CAL sensors=mag
```

Progress arrives as `CAL,...` lines; the run ends with `CAL,DONE` and
`CALRESULT,ok=1`. Send `x` on serial to cancel a stage.

### Stage 1 — gyro (about 15 s)

Keep the robot **completely still** on a solid surface. A 1.5 s settling
period is followed by up to 2,500 accepted samples (at least 2,000 required);
samples outside 0.85–1.15 g or above 8 °/s are ignored. Each axis must meet
both a standard-deviation limit of 0.20 °/s and a median-to-mean difference of
0.6 °/s or less. Failed quality checks are retried up to five times.

### Stage 2 — accelerometer (six positions, ~2 min)

The OLED names the sensor/body axis and sign that must point up; the firmware
waits until that orientation is detected before collecting. Since the board is
mounted rotated relative to the chassis, use the axis prompt rather than
assuming a chassis-face label. Each position requires 180 samples collected at
10 ms intervals while the gyro is below 3 °/s and acceleration is within
0.85–1.15 g; a position times out after 120 s.

| Step | OLED says | Meaning |
|---|---|---|
| 1 | `Put X+ up` | sensor/body +X points up |
| 2 | `Put X- up` | sensor/body -X points up |
| 3 | `Put Y+ up` | sensor/body +Y points up |
| 4 | `Put Y- up` | sensor/body -Y points up |
| 5 | `Put Z+ up` | sensor/body +Z points up |
| 6 | `Put Z- up` | sensor/body -Z points up |

### Stage 3 — magnetometer (20 s)

Rotate the robot **slowly through all orientations for 20 seconds** in a spot
away from metal, magnets, speakers and high-current wiring. Up to 1,200 samples
are collected, with at least 3 µT separation from the previous accepted sample;
at least 200 are required. A dense cluster in one orientation produces a poor
fit. The firmware fits a sphere, then accepts an ellipsoid refinement only when
its residual is no worse; the fitted field radius must be 20–70 µT.

## Checking the result

```text
GET
OK SNAP,...,cal=1,...
```

`cal=1` requires a complete stored record containing all six accelerometer
positions and a magnetometer fit, then a successful boot self-check. A lone
`CAL gyro` on a blank robot leaves `cal=0` because the record still lacks the
required accel and mag calibration. A newly completed guided run is used
immediately; after reboot, the stored record must pass its self-check again.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `CAL,SELF_CHECK,failed,run_CAL` at boot | temperature drift or the mounting changed | rerun `CAL` |
| `CAL,SELF_CHECK,incomplete,run_CAL_all` | only some stages are stored | run `CAL` (all) |
| `CAL,gyro,motion_detected_retrying` | fan, footsteps, cable tug | isolate the robot, rerun |
| `CAL,FAIL,accelerometer,solve_failed` | one position was wrong | redo all six |
| `CAL,FAIL,magnetometer,not_enough_samples` | rotated too little | rotate more, slower |
| `CAL,FAIL,magnetometer,sphere_fit_failed` | magnetic interference | move away from metal, redo |
| `mag_rms_ut` above ~2 µT after a good run | metal nearby or poor coverage | relocate and recalibrate |
| Yaw/heading is unreliable | magnetic disturbance, mounting offset, or an older fit | check the heading convention in [imu/ARCHITECTURE.md](imu/ARCHITECTURE.md#2-frame-conventions-read-this-before-touching-any-math), relocate, then rerun `CAL mag` |

## Storage

The record is keyed by robot number (`r1` … `r8`) in the NVS namespace
`imu-cal`. Consequences worth knowing:

* Two robots on the same bench do not share a calibration.
* Changing a robot's number with `SETID` means it looks for a different key — it
  will appear uncalibrated until you run `CAL` again.
* To force a recalibration, change the robot's magnetic/thermal environment or
  rerun `CAL`; there is no separate "reset" command.

## Related reading

* [imu/CALIBRATION.md](imu/CALIBRATION.md) — the math behind each stage
* [imu/ARCHITECTURE.md](imu/ARCHITECTURE.md) — where calibration sits in the pipeline
* [HARDWARE.md](HARDWARE.md) — assembly order and the still-pose requirement
