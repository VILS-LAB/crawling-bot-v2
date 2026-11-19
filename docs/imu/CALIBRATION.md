# IMU calibration guide

Practical guide for calibrating the MPU9250 on the GY-91. For the *why* behind
each step, see [ARCHITECTURE.md §4](./ARCHITECTURE.md#4-calibration).

## When to calibrate

* First boot of a new board.
* After moving the IMU, changing its mounting, or re-soldering.
* When the boot self-check fails (`CAL,SELF_CHECK,recalibration_required`).
* When `ebx/eby/ebz` in telemetry sit persistently away from zero.
* When `mag_rms_ut` is above ~2 µT.

## Opening the menu

Send `q` over the serial console (115200 baud). Telemetry pauses and the menu
prints:

```
IMU MENU: 1=gyro 2=accel 3=mag 4=all 5=status 6=clear 0=close
```

| Key | Action |
|---|---|
| `1` | gyro only |
| `2` | accelerometer only |
| `3` | magnetometer only |
| `4` | all three (recommended) |
| `5` | print status |
| `6` | erase the stored calibration |
| `0` | close the menu, resume telemetry |

During any calibration, `x` cancels and `?` prints help.

## Gyro

1. Put the robot on a solid surface.
2. Press `1` (or `4`).
3. **Do not touch it** for the 1.5 s settle plus the ~12 s collection.
4. Success prints `CAL,gyro,bias=x,y,z`.

If it prints `CAL,gyro,motion_detected_retrying`, something moved. It retries up
to 5 times, then fails.

The check is PX4's: per-axis standard deviation must be ≤ 0.20 °/s **and** the
median must be within 0.6 °/s of the mean. The median check catches a slow,
steady rotation that a variance test alone would miss.

## Accelerometer

Six guided positions. For each one the firmware waits until the orientation is
correct before it starts collecting, and the OLED tells you what it wants.

| Step | OLED says | Meaning |
|---|---|---|
| 1 | `Put X+ up` | robot nose pointing up |
| 2 | `Put X- up` | robot nose pointing down |
| 3 | `Put Y+ up` | robot left side down |
| 4 | `Put Y- up` | robot right side down |
| 5 | `Put Z+ up` | robot upside down |
| 6 | `Put Z- up` | robot right side up (normal) |

For each position:

* hold it **still** — `|gyro|` must stay under 3 °/s,
* the dominant axis must match and have the right sign,
* `|accel|` must be within 0.85–1.15 g,
* 180 samples are averaged.

If the OLED shows `Wrong position`, it also shows what it currently sees
(`Current: Z+`) and what it needs (`Need: X+`).

Success prints the bias, the full 3×3 matrix, and the residual:

```
CAL,accelerometer,bias=...
CAL,accelerometer,matrix=...
CAL,accelerometer,rms_g=0.00412
```

`rms_g` below ~0.02 g is good. Higher means the robot moved during a step or
the mounting is loose.

## Magnetometer

1. Press `3` (or `4`).
2. **Rotate the robot slowly through all orientations** for 20 s — figure
   eights, all three axes, no fast spins.
3. Keep it away from metal, magnets, speakers and powered wires.

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

Press `5` for status:

```
IMU,STATUS,robot=1,saved=1,acc_norm_g=1.0002,gyro_norm_dps=0.031,
mag_norm_ut=47.2,still=1,stdA=0.00120,trust=1.000,rate_hz=199.8,
accel_rms_g=0.00412,mag_rms_ut=0.83,mag_fit=2
```

| Field | Good value |
|---|---|
| `acc_norm_g` | 0.99–1.01 |
| `gyro_norm_dps` | < 0.1 |
| `mag_norm_ut` | 25–65 |
| `rate_hz` | ~200 |
| `accel_rms_g` | < 0.02 |
| `mag_rms_ut` | < 2 |

Then close the menu (`0`) and check the live telemetry:

* level and still → `roll ≈ pitch ≈ 0`,
* roll the robot 30° → `roll ≈ 30` within ~2 s,
* yaw left by hand → `yaw` decreases.

## Storage

Calibration is stored in NVS (`Preferences`, namespace `imu-cal`) as one blob
per robot number (`r1` … `r8`), so eight robots can share the firmware. The
record has a magic and a version; a version mismatch is ignored and the robot
recalibrates instead of using stale data.

`6` erases the record for the current robot number.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `CAL,gyro,motion_detected_retrying` | robot moved, or a fan/vibration source is running |
| `CAL,FAIL,accelerometer,solve_failed` | a face pair was too close together — one position was wrong |
| `CAL,FAIL,magnetometer,sphere_fit_failed` | not enough rotation, or too much magnetic interference |
| `CAL,FAIL,magnetometer,not_enough_samples` | rotated too slowly / too little |
| `CAL,SELF_CHECK,recalibration_required` | temperature drift or a real change in the mounting |
| `mag_rms_ut` high after a good run | metal nearby; move away and redo |
| `ebx/eby/ebz` drifting | gyro calibration left a bias; redo `1` |
