# Host-side tests

Compiles the **real** firmware math sources with the system C++ compiler and
runs them on the desktop. No hardware, no Arduino, no PlatformIO — about one
second per run.

```bash
test/host/run.sh
```

## What is covered

| Section | Checks |
|---|---|
| quaternion basics | rotate / rotateInverse round trip, matrix ↔ quaternion, Euler extraction |
| rotation contract | `rotateBodyToWorld` for level / roll 90 / yaw 90, matrix form agrees with the sequential form, gravity at rest |
| estimator initialization | recovers a known attitude exactly for 8 roll/pitch/yaw cases, gravity-only fallback levels |
| filters | low-pass DC gain and attenuation, pass-through when disabled, median spike rejection, notch attenuation and DC, alpha convergence |
| attitude estimator | leveling, 30° roll convergence, gyro integration, bias rejection, bias learning, NaN robustness |
| accelerometer calibration | six-position solve recovers a known bias + matrix, degenerate face pair rejected |
| magnetometer calibration | LM sphere fit recovers a known offset + radius, ellipsoid refinement is not worse |
| still detection | constant 1.025 g acceleration, zero gyro and low noise become still using the 40-sample mean |
| end-to-end tracking | yaw tracked against a synthetic ground truth |
| servo motion profile | lands exactly on target, respects speed/accel limits, no overshoot, continuous retarget, soft limits, degenerate dt/config |

## Why this exists

The IMU math is the part of the firmware that is hardest to debug on the bench:
a sign error or a transposed matrix produces plausible-looking numbers that are
simply wrong. These tests pin the math down so a refactor cannot silently change
what the robot believes about its own orientation.

The sources under test are the same files the firmware compiles:

```
lib/IMU/ImuMath.h
lib/IMU/ImuFilters.cpp
lib/IMU/ImuAttitudeEstimator.cpp
lib/IMU/ImuCalibration.cpp
lib/IMU/ImuRotation.cpp
lib/ServoControl/ServoMotion.cpp
```

`ImuCalibration.cpp` guards its `Preferences` persistence behind
`#if defined(ARDUINO)`, which is what lets it build on the host.

## Adding a test

Add a `testXxx()` function and call it from `main()`. Use `check()` for
booleans and `checkNear()` for floats; both increment the global counters and
print on failure. The process exits non-zero if anything failed, so it drops
straight into CI.
