# Servo health check

Two servos (leg down / leg up) are driven through `lib/ServoControl`. The boot
health check in `lib/HealthCheck` returns a **numeric code per servo** over the
serial console, so a host tool or a student script can parse the result without
reading the OLED.

> This page covers the boot self-test. For how normal moves are made safe
> (ramping, soft limits, the non-blocking API), see
> [SERVO_SAFETY.md](SERVO_SAFETY.md).

## Order of operations at boot

```
attach servos  ->  verify stored calibration  ->  servo health check  ->  Ready
                   (guided CAL only when you ask for it)
```

Boot never blocks on a human: it only *verifies* the stored calibration. If the
check fails (or nothing is stored), the robot skips the health check and leaves
the servos parked; the OLED shows `Uncalibrated - run CAL`. The `ACT` command
handler is not interlocked with calibration, so an explicit command can still
move a servo. Calibrate before normal use and do not command movement while
uncalibrated. The full health check runs on the next boot with a valid
calibration.

The health check runs **after** calibration, on purpose:

| Step | Why it must be here |
|---|---|
| `servos.begin()` | attach the PWM channels and park the legs at the initial angle *before* calibration, so the robot is already in its final, still pose when the gyro/accel calibration starts |
| `ahrs.calibrate()` | verifies the stored calibration and needs the robot completely still while doing so; the guided `CAL` (gyro bias, six-position accel fit) additionally requires a stationary frame |
| `healthCheck.run()` | **moves the servos on purpose**. Doing this before calibration would inject rotation into the gyro bias, add noise to the accelerometer fit, and could make the calibration fail or silently learn a bad bias |

So: **attach before calibration, test after calibration.** Never test the servos
between "keep still" and the end of calibration.

## Codes

`ServoHealthCode` (in [lib/ServoControl/ServoHealth.h](../lib/ServoControl/ServoHealth.h)).
The numbers are a serial contract — new codes are appended, existing ones are
never renumbered.

| Code | Name | Meaning | What to check |
|---|---|---|---|
| 0 | `OK` | attached, PWM read-back matches, IMU saw the move | nothing |
| 1 | `ATTACH_FAILED` | `attach()` returned 0: pin not usable for PWM or no free LEDC channel | GPIO choice, library pin whitelist, channel count |
| 2 | `NOT_ATTACHED` | attached earlier, channel lost now | detach/reset path |
| 3 | `PULSE_MISMATCH` | written angle and read-back pulse width differ by more than `SERVO_HEALTH_PULSE_TOLERANCE_US` | PWM path, timer sharing between the two channels |
| 4 | `NO_MOTION` | sweep produced no gyro response above the noise floor | servo power, signal wire, mechanical binding |
| 5 | `DISABLED` | `ENABLE_SERVOS == 0`, nothing was tested | not a failure |

A `DISABLED` servo counts as pass in the summary: the check was never asked to
run.

## Serial lines

```
HEALTH,SERVO,START
HEALTH,SERVO,down,pin=32,code=0,status=OK
HEALTH,SERVO,up,pin=33,code=4,status=NO_MOTION
HEALTH,SUMMARY,ok=0,down=0,up=4
```

If something fails at boot the firmware also prints
`ERROR,SERVO_ATTACH_FAILED` (in `setup()`) or `ERROR,HEALTH_CHECK_FAILED` (after
the check). Both use the same `ERROR,<TAG>[,detail]` style as the IMU errors, and
neither line starts with `IMU,`, so the telemetry visualizers ignore them.

## How each check works

1. **Attach** — `ServoControl::begin()` stores the `attach()` result per servo.
   `attach()` is the only place the library reports a bad pin.
2. **Electrical** — write the current angle, wait
   `SERVO_HEALTH_SETTLE_MS`, then compare `readMicroseconds()` against the
   expected pulse for that angle. This proves the firmware → PWM path is alive.
   It cannot prove the horn moved, because `readMicroseconds()` returns the value
   the library wrote, not a measurement.
3. **Motion** — measure the gyro norm for `SERVO_HEALTH_BASELINE_MS` (the noise
   floor while nothing moves), then sweep `SERVO_HEALTH_SWEEP_DEG` degrees away
   and back. If the peak gyro norm never exceeds `baseline +
   SERVO_HEALTH_MOTION_DPS`, the IMU felt nothing and the code is `NO_MOTION`.
   Set `ENABLE_SERVO_MOTION_CHECK` to `0` to run the electrical check only
   (useful on a bench with the legs removed).

The motion check needs the IMU task running, which is why the health check is
placed after `ahrs.startTask()`.

## Pin note for the ESP32-S3

`ESP32Servo` 0.13.0 compiles with `ENFORCE_PINS` and, on the S3, only accepts
PWM on **GPIO 1–21, 35–45, 47, 48**. GPIO 32/33 — named by older course
material — are valid on a classic ESP32 but **not** on the S3 with this library
version, so `attach()` returns 0 and you get code 1 (`ATTACH_FAILED`) for both
servos.

The firmware uses **GPIO 6 and 7**, which are inside the allowed range, so the
default build is unaffected. If you rewire the servos:

* stay inside the allowed range and update `SERVO_PIN_DOWN` / `SERVO_PIN_UP`
  in [include/config.h](../include/config.h), or
* upgrade `madhephaestus/ESP32Servo` to `^3.2.1`, which uses the newer
  Arduino-ESP32 LEDC API that does not keep that whitelist.

Either way the health check reports the truth instead of failing silently.
