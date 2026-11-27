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

A robot with `cal=0` still boots, still answers `PING`/`ID`/`GET` and still
accepts OTA. It just will not move its legs, because a move is only meaningful
once the robot knows its own orientation.

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

Keep the robot **completely still** on a solid surface. The firmware rejects the
run if any axis moves more than 0.20 °/s, and retries up to five times.

### Stage 2 — accelerometer (six positions, ~2 min)

The OLED names the face that must point up; the firmware waits until it sees
that orientation before collecting. Follow it exactly — a wrong position is
detected and refused, not silently averaged in.

| Step | OLED says | Meaning |
|---|---|---|
| 1 | `Put X+ up` | nose up |
| 2 | `Put X- up` | nose down |
| 3 | `Put Y+ up` | left side down |
| 4 | `Put Y- up` | right side down |
| 5 | `Put Z+ up` | upside down |
| 6 | `Put Z- up` | upright |

### Stage 3 — magnetometer (20 s)

Rotate the robot **slowly through all orientations** in a spot away from metal,
magnets, speakers and laptop power supplies. A dense cluster in one orientation
produces a bad sphere/ellipsoid fit.

## Checking the result

```text
GET
OK SNAP,...,cal=1,...
```

`cal=1` requires **both** the accelerometer and magnetometer stages to be
present. A lone `CAL gyro` on a blank robot leaves `cal=0` on purpose, so a later
run cannot silently use identity accel/mag matrices.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `CAL,SELF_CHECK,failed,run_CAL` at boot | temperature drift or the mounting changed | rerun `CAL` |
| `CAL,SELF_CHECK,incomplete,run_CAL_all` | only some stages are stored | run `CAL` (all) |
| `CAL,gyro,motion_detected_retrying` | fan, footsteps, cable tug | isolate the robot, rerun |
| `CAL,FAIL,accelerometer,solve_failed` | one position was wrong | redo all six |
| `CAL,FAIL,magnetometer,not_enough_samples` | rotated too little | rotate more, slower |
| `CAL,FAIL,magnetometer,sphere_fit_failed` | magnetic interference | move away from metal, redo |
| `mag_rms_ut` above ~2 µT after a good run | metal nearby | relocate and recalibrate |
| Yaw still drifts | gyro bias, or mag fit is old | rerun `CAL mag` |

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
