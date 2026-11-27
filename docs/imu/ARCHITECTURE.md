# IMU / AHRS architecture

This is the implementation guide for the current IMU/AHRS stack: how MPU9250
and AK8963 readings become attitude, gravity-compensated acceleration and a
short-horizon dead-reckoning estimate. The sensor calibration, selected filters
and quaternion estimator are adapted from PX4; this is not a complete PX4 IMU
runtime.

The project's body-to-world rotation is a separate, frozen transform contract
used for gravity removal and visualization. The estimator also maintains a
quaternion, but its output and this transform must use the same frame convention.

This is the **third** generation of the IMU stack. [IMPLEMENTATION_HISTORY.md](./IMPLEMENTATION_HISTORY.md)
keeps the prior-generation context; it is historical, so use current source and
this architecture document for today's behavior.

---

## 1. Hardware and constraints

| Item | Value |
|---|---|
| IMU | InvenSense MPU9250 on a GY-91 breakout |
| Magnetometer | AK8963, inside the MPU9250 package |
| MCU | ESP32-S3 (dual core, Xtensa LX7) |
| Bus | I2C, address `0x68`, AK8963 at `0x0C` |
| Accel range | ±4 g (`IMU_ACCEL_RANGE`) |
| Gyro range | ±500 °/s (`IMU_GYRO_RANGE`) |
| Sample rate | 200 Hz (`SMPLRT_DIV` = 200 Hz, gyro DLPF 41 Hz, accel DLPF 45 Hz) |
| Mag output rate | 100 Hz continuous (`MAG_MODE` = 0x06) |

Two hardware facts drive most of the design decisions:

1. **The AK8963 is behind the MPU9250's I2C master.** It is only readable
   through the MPU9250 bypass, and it updates at 100 Hz while the accel/gyro
   update at 200 Hz. So the magnetometer is *not* available on every sample.
2. **The GY-91 is hand-soldered.** Its three axes are not perfectly orthogonal
   and its gain is not perfectly matched. That is why the accelerometer
   calibration keeps a full 3×3 matrix instead of PX4's diagonal-only shortcut.

---

## 2. Frame conventions (read this before touching any math)

This is the single most common source of "the robot rotates the wrong way"
bugs. The **estimator's firmware convention** is:

```
body frame  : X forward, Y left,  Z up      (FLU, right-handed)
world frame : X north,   Y west,  Z up      (right-handed)
Euler       : R = Rz(yaw) * Ry(pitch) * Rx(roll)
quaternion  : body -> world
units       : accel m/s^2, gyro deg/s at the AHRS interface (rad/s inside the
              estimator), mag uT, angles deg, position m, velocity m/s
```

These are firmware coordinate labels, not a statement that the GY-91's PCB
axes physically point along chassis forward/left/up. The current mounting is
shown below; calibration corrects sensor biases and scale/cross-axis errors,
not an arbitrary mounting rotation.

### How the IMU is physically mounted (important)

The estimator labels its body frame FLU. On this robot, the GY-91 is mounted
with its sensor/body axes mapped to the chassis as follows:

```
chassis forward = sensor/body +Y
chassis right   = sensor/body +X
chassis up      = sensor/body +Z
```

This is a sensor mounting convention, not a permutation performed by
`readAndCalibrate()`: the firmware treats the measured axes as body X/Y/Z and
calibration only estimates biases and scale/cross-axis correction. Consumers
that want chassis compass heading must apply the mounting offset. The 90°
physical relationship appears as a 270° constant because yaw is
counter-clockwise from north while compass heading is clockwise from north:

```
heading = (270 - yaw) mod 360
```

Derivation: the nose direction in the world frame is
`R · (0,1,0) = (-sin yaw, cos yaw, 0)` in `(north, west, up)`. Converting to
`(east, north)` gives `east = -cos yaw`, `north = -sin yaw`, so
`heading = atan2(east, north) = atan2(-cos yaw, -sin yaw) = (270 - yaw) mod 360`.

Both visualizers implement this. They also draw everything in a **compass
display frame** `(east, north, up)` — north up, east right — instead of the
firmware's `(north, west, up)`, so the 3D view, the heading dial and the
top-down map all agree. The conversion is a fixed matrix:

```
display = [[0, -1, 0],      # east  = -west
           [1,  0, 0],      # north =  north
           [0,  0, 1]]      # up    =  up
```

If you remount the IMU, update and bench-validate the heading and display-frame
conversion in both visualizers. The firmware currently has no configurable
sensor-to-chassis mounting transform.

### Why this matters

PX4 uses **FRD body / NED world** (X forward, Y right, Z down). The estimator
math is frame-agnostic *as long as the gravity direction and the cross-product
order are consistent*, but two places need explicit care:

* **Initialization.** PX4 builds the rotation matrix rows as
  `north`, `east = down × north`, `down`. With Y pointing **left** instead of
  right, the correct second row is `west = north × down`. Using PX4's order
  verbatim produces a 180° roll error at startup. See
  [`ImuAttitudeEstimator.cpp`](../../lib/IMU/ImuAttitudeEstimator.cpp)
  `initialize()`.
* **Gravity sign.** In NED the accelerometer reads −g along earth Z; in FLU it
  reads +g along earth Z. The estimator's `k = -accel.normalized()` expression
  happens to be correct in both, which is why the update loop is unchanged.

### The rotation contract

`rotateBodyToWorld()` in [`ImuRotation.cpp`](../../lib/IMU/ImuRotation.cpp) is
**frozen**. It is the sequential rotation

```
body -> Rx(roll) -> Ry(pitch) -> Rz(yaw) -> world
```

which is algebraically `Rz(yaw) · Ry(pitch) · Rx(roll)` — the same convention
the estimator reports Euler angles in. It is kept as an explicit sequential
implementation (not a matrix multiply) because that is the exact code path the
hardware and both visualizers were validated against. `bodyToWorldMatrix()`
exposes the equivalent matrix for tests and tooling; the host test asserts the
two agree to 1e-5.

See also [ROTATION_MATRIX.md](./ROTATION_MATRIX.md).

---

## 3. Signal chain

```
                    ┌──────────────────────────────────────────────┐
   MPU9250 @200 Hz  │  IMU task, pinned to core 1                  │
        │           │                                              │
        ▼           │                                              │
   readAndCalibrate │  accel: M·(raw − bias)      [g → m/s²]       │
        │           │  gyro : raw − bias          [deg/s]          │
        │           │  mag  : M·(raw − offset)    [mG → uT]        │
        ├───────────┼── unfiltered calibrated accel/gyro/mag ──────┤
        │           │  estimator input                             │
        ▼           │                                              │
    runFilters      │  diagnostic fields: gyro median→notch→LPF2p  │
        │           │  accel LPF2p; mag LPF2p                      │
        └───────────┴── filtered values are not estimator inputs ──┘
        ▼
   runEstimator     │  PX4 attitude_estimator_q adaptation         │
        │           │  quaternion + integral gyro bias             │
        ▼           │                                              │
 updateStillDetection│ 40-sample accel window, 3 independent checks │
        │           │                                              │
        ▼           │                                              │
 updateDeadReckoning│  rotateBodyToWorld → remove gravity → ZUPT   │
        │           │  → deadband → integrate v, p                │
        ▼           │                                              │
 publishSnapshot    │  double-buffered ImuTelemetry under a spinlock│
        └───────────┴──────────────────────────────────────────────┘
                              │
                              ▼
                    Arduino loop (core 0)
                    ahrs.update() copies the snapshot
```

### 3.1 Why a separate task

The Arduino `loop()` drives the OLED (I2C, ~10 ms per refresh), servos and
command handling. Running estimation inside `loop()` would make its timing
sensitive to that work. A dedicated task isolates it from the loop, though it
still shares core 1 with the OTA handler and can contend with other tasks there.

So the sampling loop lives in its own FreeRTOS task:

```c
#define IMU_TASK_CORE     1   // IMU and OTA tasks currently use core 1
#define IMU_TASK_PRIORITY 5
#define IMU_TASK_STACK    4096
```

* **Core 1** is selected by `IMU_TASK_CORE`; the OTA handler is also pinned to
  core 1 at lower task priority. The IMU task uses priority 5, while the Arduino
  loop runs separately; pinning does not guarantee zero scheduling jitter.
* Changing `IMU_TASK_CORE` changes the contention profile. Re-test measured
  `rate_hz` and estimator behavior on hardware if task placement or workloads
  change; do not assume OTA can never affect sampling.
* The task is **paused** (`taskPaused`) during calibration, because calibration
  needs exclusive access to the sensor and drives the OLED.
* `ahrs.update()` in `loop()` is a cheap copy of the latest snapshot under a
  spinlock. It never blocks on I2C.

#### Shared I2C bus

The OLED (core 0) and the MPU9250 (core 1) are on the same SDA/SCL pins.
Interleaving their transfers makes the sensor return shifted or garbled bytes,
so **every** bus access goes through a recursive mutex in
[`lib/I2CBus`](../../lib/I2CBus/I2CBus.h):

* `i2c::begin()` starts the peripheral once at `I2C_CLOCK_HZ` (400 kHz) and owns
  the clock for both drivers (`Display` and `AHRS` pass `periphBegin = false` /
  create no device of their own).
* `i2c::Guard` is an RAII lock with a timeout (`I2C_LOCK_TIMEOUT_MS`). A timeout
  means the other core is wedged; the caller skips its transfer instead of
  blocking the loop.
* `AHRS::updateSensor()` holds the lock for exactly one `mpu->update()` burst.
  The getters below it read values cached in the driver, so the hold time stays
  at one I2C transaction and the estimator work runs unlocked.
* The Adafruit SSD1306 library sets the bus to `clkDuring` for a transfer and
  restores `clkAfter`; both are constructed as 400 kHz so a display refresh does
  not drop the bus back to 100 kHz under the 200 Hz IMU reads.

#### Hardware watchdog

`lib/SystemWatchdog` wraps `esp_task_wdt`:

| Task | Subscription |
|---|---|
| Arduino loop (core 0) | `watchdog::begin()` in `setup()` |
| IMU task (core 1) | `watchdog::addCurrentTask()` in `taskEntry()` |

The timeout is `WATCHDOG_TIMEOUT_S` (3 s) with panic + reset, so a hang reboots
the robot instead of leaving powered, frozen legs. Legitimate blocking work is
handled explicitly:

* `CAL` (`runGuidedCalibration`) uses `watchdog::Suspend`, which unsubscribes
  the loop task for the duration of the guided procedure and resubscribes on
  exit — a multi-minute calibration is not a hang.
* The IMU task feeds the watchdog even while paused, so a calibration pause is
  not mistaken for a stall.
* The health check, `ServoControl::waitUntilIdle()` and the boot self-check feed
  explicitly around/inside their blocking waits.

### 3.2 Snapshot handoff

`ImuTelemetry` is written by the IMU task into `pendingTelemetry` and copied by
`update()` into `telemetry` inside `portENTER_CRITICAL`. All public getters read
`telemetry`, never the task-owned members, so there is no torn read and no race
on `velocity` / `position` / `linearAccel`.

---

## 4. Calibration

The model is PX4's (`src/lib/sensor_calibration/*.hpp`):

```
accel_corrected = accelMatrix · (accel_raw − accelBias)     [g]
gyro_corrected  = gyro_raw − gyroBias                       [deg/s]
mag_corrected   = magMatrix  · (mag_raw  − magOffset)       [uT]
```

### 4.1 Gyroscope — `calibrateGyroscope()`

Port of PX4's `gyro_calibration_worker` + the quality gate from
`do_gyro_calibration`:

1. Settle 1.5 s.
2. Collect up to 2500 samples, rejecting any sample where `|accel|` is outside
   0.85–1.15 g or `|gyro|` > 8 °/s (i.e. the robot moved).
3. Require ≥ 2000 accepted samples.
4. Per axis, compute the mean and standard deviation across the accepted
   samples. A rolling `MedianFilter3` is also updated for each sample; its final
   median is the median of the last three accepted values, not of the full run.
5. Reject if `sigma > 0.20 °/s` or if that final three-sample median differs
   from the run mean by more than 0.6 °/s. Retry up to 5 times.

The median/mean comparison is an additional end-of-window consistency check,
not a robust median of the full calibration run. It should not be relied on to
detect every slow or constant rotation; keep the robot still throughout and
interpret the acceptance limits as checks, not proof of a perfect calibration.

### 4.2 Accelerometer — `calibrateAccelerometer()`

Port of PX4's `do_accel_calibration`, six guided positions:

| Side | Reference vector (g) |
|---|---|
| Prompt/side | Reference vector (g) |
|---|---|
| `X+` | `( 1, 0, 0)` |
| `X-` | `(-1, 0, 0)` |
| `Y+` | `( 0, 1, 0)` |
| `Y-` | `( 0,-1, 0)` |
| `Z+` | `( 0, 0, 1)` |
| `Z-` | `( 0, 0,-1)` |

The prompt means the named sensor/body axis points up. The firmware does not
apply a separate sensor-to-chassis rotation, so do not infer a chassis face name
from `X+`/`Y+` without checking the actual board mounting.

For each side the firmware waits for the *correct* orientation (dominant axis
matches, sign matches, `|a|` within 0.85–1.15 g, `|gyro|` < 3 °/s), then
averages 180 samples.

The solve is PX4's:

```
offset[i] = (raw[2i][i] + raw[2i+1][i]) / 2
A.row(0)  = raw[X+] − offset
A.row(1)  = raw[Y+] − offset
A.row(2)  = raw[Z+] − offset
M         = (A⁻¹)ᵀ
```

**One deliberate difference from PX4.** PX4 computes `accel_T = A⁻¹ · 1g` and
then only keeps `diag(accel_T)`, so the transpose is invisible there. This
project keeps the full matrix, and for the model `corrected = M · (raw − bias)`
the correct matrix is `(A⁻¹)ᵀ`. The host test
(`test/host/test_imu_math.cpp`) synthesizes data from a known bias + matrix and
asserts the solver recovers both.

Keeping the full matrix is what makes the GY-91's non-orthogonal axes
correctable. The residual (`accelFitRms`) is reported in telemetry and in the
`CAL,accelerometer,rms_g=` line.

### 4.3 Magnetometer — `calibrateMagnetometer()`

Port of PX4's `mag_calibration` + `lm_fit.cpp`:

1. Collect a rotating sample cloud for 20 s (max 1200 samples), enforcing a
   minimum 3 µT distance between consecutive samples so the fit is not dominated
   by one dense cluster.
2. **Sphere fit** — Levenberg–Marquardt on `(radius, offset)`, seeded from the
   sample centroid and the mean centroid distance.
3. **Ellipsoid fit** — LM on `(offset, diag, offdiag)`, seeded from the sphere
   result. Accepted only if it improves the residual.
4. Reject if the radius falls outside 20–70 µT (PX4's 0.2–0.7 G, converted).

The result is stored as `magOffset` + `magMatrix`, and `magFieldNorm` keeps the
fitted radius for the boot self-check. The radius window is a plausibility
check, not proof that the local magnetic field is undisturbed.

> **Unit note.** PX4 works in Gauss. This project works in µT (1 G = 100 µT).
> The LM thresholds in `ImuCalibration.cpp` are scaled accordingly — if you port
> more PX4 calibration code, check the units first.

### 4.4 Boot self-check — `verifyCalibration()`

`AHRS::calibrate()` runs at boot and **never blocks on a human**. After a short
bounded warm-up (2 s, watchdog-fed), if a stored calibration exists, 200 samples
are checked:

| Check | Limit |
|---|---|
| `| |accel| − 1 g |` | ≤ 0.05 g |
| `|gyro|` (bias removed) | ≤ 1.0 °/s |
| corrected magnetometer norm vs stored field norm | ≤ 20 % (skipped if no positive stored norm) |
| temperature drift since calibration | ≤ 15 °C (skipped if stored temperature ≤ 0 °C) |

On success the robot goes straight to normal operation (`CAL,SELF_CHECK,ok`).
On failure — or when nothing is stored — it boots **uncalibrated**: the servos
stay parked, the OLED shows `Uncalibrated - run CAL`, and `GET` reports `cal=0`.
A full guided recalibration only runs when the operator sends `CAL`.

### 4.5 Storage

`Preferences` namespace `imu-cal`, one blob per robot number (`r1` … `r8`),
magic `0x494D5533`, version `5`. The version was bumped because the record
layout changed (full accel matrix, mag matrix, fit diagnostics). An old record
is simply ignored and the robot recalibrates.

### 4.6 Runtime gyro bias tracking

While the robot is genuinely still, the calibrated gyro bias is nudged toward
the raw reading with `alpha = 0.001` per sample, **clamped to ±0.5 °/s around
the calibrated value**. The clamp is the safety property: a false "still"
detection can never walk the bias away.

This is separate from the estimator's own integral bias state (see §5.3).

---

## 5. Attitude estimator

Port of PX4's `attitude_estimator_q`
(`src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp`), in
[`ImuAttitudeEstimator.cpp`](../../lib/IMU/ImuAttitudeEstimator.cpp).

### 5.1 Structure

```
correction = 0

# magnetometer heading
if mag usable:
    mag_earth = q · mag
    err       = wrap(atan2(mag_earth.y, mag_earth.x) − declination)
    gain      = magWeight · (spinRate > 50°/s ? min(spinRate/50°/s, 10) : 1)
    correction += q⁻¹ · (0, 0, −err) · gain

# accelerometer tilt
k = q⁻¹ · (0, 0, −1)                 # world down, in body
if 0.9 g < |accel| < 1.1 g:
    correction += k × accel_normalized · accelWeight

# integral gyro bias
if spinRate < 10°/s:
    gyroBias += correction · gyroBiasWeight · dt
    clamp(gyroBias, ±gyroBiasLimit)

rates = gyro + gyroBias
correction += rates
q += q.derivative1(correction) · dt
q.normalize()
```

### 5.2 Initialization

`initialize(accel, mag)` builds the initial quaternion from the gravity and
magnetic-field vectors (PX4's `init_attitude_q`), with the handedness fix
described in §2. If the magnetometer is unusable it falls back to a
gravity-only leveling with zero heading.

The estimator is seeded lazily: `runEstimator()` waits until `|accel|` is
between 0.5 g and 1.5 g before calling `initialize()`, so the initial attitude
is never built from noise.

### 5.3 Tuning, and the one parameter that differs from PX4

| Parameter | PX4 default | Here | Why |
|---|---|---|---|
| `accelWeight` | 0.2 | 0.2 | same |
| `magWeight` | 0.1 | 0.1 | same |
| `gyroBiasWeight` | 0.1 | **0.02** | see below |
| `gyroBiasLimit` | 0.05 rad/s | 0.15 rad/s | GY-91 residual bias after a short calibration can exceed 0.05 |
| accel fusion window | 0.9–1.1 g | 0.9–1.1 g | same |
| bias freeze rate | 0.175 rad/s | 0.175 rad/s | same |
| mag gain boost | > 0.873 rad/s, ×10 | same | same |

**Why `gyroBiasWeight` is lower.** PX4's 0.1 is tuned for a flying vehicle,
where the accelerometer correction is transient and the bias integrator mostly
sees real bias. On a ground robot that sits still for minutes, a *persistent*
attitude error is continuously fed into the bias integrator, and the
(attitude, bias) loop becomes lightly damped: the estimate overshoots and takes
about a minute to settle. With 0.02 both poles of the linearized loop are real
and negative, so it settles in a few seconds with no overshoot. The host test
`testEstimator()` covers both the "converges to 30° roll" and the "converges
with a gyro bias present" cases.

### 5.4 What is *not* ported

PX4's estimator also fuses GPS velocity, vision odometry, motion-capture
odometry and GPS-derived magnetic declination. None of those inputs exist on
this robot, so those code paths are omitted rather than stubbed. Magnetic
declination is a plain constant in `AttitudeEstimatorConfig` (default 0, i.e.
yaw is relative to magnetic north).

Note that `yaw` is the rotation of the firmware's body +X axis, which on this
robot points **right**, not forward. See §2 for the `heading = (270 - yaw) mod 360`
conversion.

---

## 6. Filters

Direct ports of PX4's `src/lib/mathlib/math/filter/*.hpp`, in
[`ImuFilters.cpp`](../../lib/IMU/ImuFilters.cpp). All of them **disable
themselves (pass-through)** when their parameters are invalid, which is PX4's
behaviour and keeps the pipeline safe against a nonsense cutoff.

| Filter | PX4 source | Used for | Cutoff |
|---|---|---|---|
| `LowPassFilter2p` | `LowPassFilter2p.hpp` | diagnostic accel, gyro and mag; gravity-free accel telemetry | 30 / 40 / 5 / 7.5 Hz |
| `NotchFilter` | `NotchFilter.hpp` | diagnostic gyro path (optional) | disabled by default |
| `MedianFilter3` | `MedianFilter.hpp` | diagnostic gyro path | window 3 |
| `AlphaFilter` | `AlphaFilter.hpp` | available utility; not used in the active AHRS pipeline | — |

### 6.1 Diagnostic filter paths

The AHRS computes filtered gyro, accel and mag values for telemetry and
inspection:

```
gyro : median(3) → notch (optional) → LPF2p(40 Hz)
accel: LPF2p(30 Hz) (computed, not included in CSV)
mag  : LPF2p(5 Hz)
```

The estimator and dead-reckoning stages use the **unfiltered calibrated**
accel, gyro and mag. The `filteredGyro` and `filteredMag` values are serialized;
the filtered accel intermediate is computed but is not exposed in the CSV. The
`flax..flaz` acceleration telemetry is separately low-passed at 7.5 Hz after
gravity removal. Filtering exists for diagnostics, not as an estimator input.

* `median(3)` is on the diagnostic gyro path before the notch and low-pass.
* The notch is **disabled by default** (`FILTER_GYRO_NOTCH_HZ = 0`). If you add
  a vibration source with a known frequency, configure
  `FILTER_GYRO_NOTCH_HZ` and `FILTER_GYRO_NOTCH_BW_HZ`.
* The 30 Hz accel and 40 Hz gyro cutoffs correspond to PX4 defaults; the mag
  low-pass is 5 Hz and the post-gravity-removal acceleration low-pass is 7.5 Hz.

### 6.2 Linear acceleration chain

```
rotateBodyToWorld → remove gravity → LPF2p(7.5 Hz)
```

The 7.5 Hz cutoff is the value the previous implementation used and the
visualizers were tuned against. The **integration** for velocity/position uses
the unfiltered value, so the dead-reckoning phase response is unchanged.

### 6.3 What was removed

The previous implementation used a per-axis
`Median(5) → Butterworth → adaptive scalar Kalman` chain. It was replaced
because:

* the adaptive Kalman's innovation-scaled measurement noise made the filter
  *slower* exactly when the signal was changing fastest, which is backwards for
  a motion-tracking application;
* three cascaded filters added ~30 ms of group delay to the gyro, which the
  estimator then integrated as real rotation;
* The selected PX4 filter implementations have documented coefficient
  behavior that can be compared with upstream source; this project still needs
  its own validation on the robot.

The old sources are kept in `archive/imu-pre-px4/` for reference.

---

## 7. Still detection and ZUPT

A 40-sample sliding window over calibrated acceleration (in g) is used for
three checks. For the magnitude check, the firmware averages each sample's
acceleration norm (not the norm of the window's mean vector):

| Bit | Check | Limit |
|---|---|---|
| 0 | mean acceleration norm `mean(|accel|) − 1 g` | enter still below 0.03 g; leave still above 0.05 g |
| 1 | `|gyro|` | < 0.5 °/s |
| 2 | accel vector standard deviation | < 0.004 g |

The magnitude threshold is hysteretic: enter still below 0.03 g error and
remain still until it exceeds 0.05 g. `still = (mask == 0)` only after the full
40-sample standard-deviation window is available. After 200 ms of continuous
passing checks the robot is declared static and velocity is zeroed (ZUPT). The
mask is published so a client can inspect which check is failing.

**Trust diagnostic.** A smoothed 0..1 value is published for clients and
visualizers: its target is `1.0` while still, `0.0` while moving, and `0.05`
after 5 s without a still interval. This value is not currently passed into the
estimator; accelerometer fusion is controlled by the estimator's 0.9–1.1 g
magnitude gate (§5.1). Do not interpret `trust` as the actual fusion weight.

---

## 8. Dead reckoning

```
worldAccel = rotateBodyToWorld(roll, pitch, 0, accel)
linearAccel.xy = worldAccel.xy
linearAccel.z  = worldAccel.z − gravityReference
```

The current implementation deliberately supplies yaw zero: this levels the
acceleration using roll/pitch and removes gravity from Z, but does **not** rotate
horizontal acceleration by the estimator's yaw. Therefore `lax..laz` and the
integrated position should not be treated as a fully yaw-aligned world-frame
navigation solution. `gravityReference` is a slow (alpha 0.02) estimate of the
vertical gravity component, updated only while still; it absorbs a small
constant Z offset that would otherwise integrate into drift.

Then, only while moving:

```
if deadbandMode == 1:  a = |a| < DEADBAND ? 0 : a
if deadbandMode == 2:  a = sign(a) · max(|a| − DEADBAND, 0)
v += a · dt
p += v · dt
```

`PLANAR_MODE` forces `vz = pz = 0`, which is correct for a ground crawler.

**This is dead reckoning, not a position solution.** Accelerometer bias and
attitude error accumulate when integrating acceleration twice; the estimate is
not bounded by an external position reference and is unsuitable for navigation.
It is provided only as a short-term motion cue/visualization. `PLANAR_MODE` is
currently enabled at compile time, so vertical velocity and position are forced
to zero; reported planar coordinates are still not a position fix.

---

## 9. Telemetry contract

The command server formats one 51-field CSV frame for each stream interval,
including the `IMU` tag.
USB serial defaults to 10 Hz (`TELEMETRY_INTERVAL_MS = 100 ms`); each TCP client
starts with streaming off and sets its own rate with `STREAM`. Serial headers
are emitted at boot and periodically, while a TCP header is sent when that
client enables streaming. The requested stream rate is capped at 50 Hz and the
achieved delivery rate may be lower if the device or client cannot keep up:

```
IMU,t_ms,dt,ax,ay,az,lax,lay,laz,flax,flay,flaz,gx,gy,gz,gbx,gby,gbz,
fgx,fgy,fgz,magx,magy,magz,roll,pitch,yaw,vx,vy,vz,px,py,pz,temp_c,
moving,still,still_failure_mask,stdA,trust,rollG,pitchG,
ebx,eby,ebz,acc_norm_g,mag_norm,rate_hz,accel_rms_g,mag_rms_ut,mag_fit,est_init
```

There are 51 CSV fields including the leading `IMU` record tag. The layout is
append-only by convention, and the included visualizers tolerate missing or
extra trailing values. The telemetry formatter uses one `snprintf`; serial
writes only a complete frame when the transmit buffer has room, while TCP
streaming drops a stalled client rather than blocking the main loop.

| Column group | Meaning |
|---|---|
| `ax..az` | calibrated accel, m/s², body |
| `lax..laz` | gravity-compensated acceleration, m/s², rotated with roll/pitch and yaw set to zero (unfiltered; not fully world/yaw aligned) |
| `flax..flaz` | same, low-passed at 7.5 Hz |
| `gx..gz` | bias-corrected gyro, °/s, body |
| `gbx..gbz` | calibration gyro bias, °/s |
| `fgx..fgz` | filtered gyro, °/s |
| `magx..magz` | calibrated mag, µT, body |
| `roll,pitch,yaw` | estimator Euler, °, project convention |
| `vx..pz` | dead-reckoned velocity (m/s) and position (m) |
| `temp_c` | die temperature, °C |
| `moving,still` | 0/1 |
| `still_failure_mask` | bit *i* = still check *i* failing |
| `stdA` | accel vector std-dev over the still window, g |
| `trust` | 0..1 smoothed diagnostic target; not currently consumed by the estimator |
| `rollG,pitchG` | gyro-only shadow attitude, ° (drift diagnostic) |
| `ebx..ebz` | **estimator** integral gyro bias, °/s |
| `acc_norm_g` | `|accel|` in g |
| `mag_norm` | `|mag|` in µT |
| `rate_hz` | measured IMU task sampling rate |
| `accel_rms_g` | accel calibration residual, g |
| `mag_rms_ut` | mag calibration residual, µT |
| `mag_fit` | 0 none / 1 sphere / 2 ellipsoid |
| `est_init` | estimator initialized flag |

### Command protocol

Serial (115200) and TCP (port 4242) share one line protocol; the full reference
is [../PROTOCOL.md](../PROTOCOL.md).

```
CAL                 -> OK CAL sensors=all      (gyro + accel + mag)
CAL gyro            -> gyro only
CAL accel           -> accelerometer only
CAL mag             -> magnetometer only
GET                 -> OK SNAP,...             (attitude + servos, incl. cal=1/0)
STREAM 50           -> OK stream=50            (telemetry at 50 Hz)
```

During a calibration, `x` on serial cancels and `?` prints help. The former
single-key menu (`q`, then `1`…`6`) has been removed.

---

## 10. Visualizers

Both tools live in `tools/` and read the same CSV.

| Tool | Purpose |
|---|---|
| [`imu_visualizer.py`](../../tools/imu_visualizer.py) | compact 3×4 matplotlib dashboard: accel, gyro, velocity, attitude vs shadow, 3D body, top-down path |
| [`imu_visualizer_v2.py`](../../tools/imu_visualizer_v2.py) | full dashboard with per-panel Y-range presets, still-failure breakdown, estimator bias, calibration health, logging and replay |

Both were updated for the new columns:

* `imu_visualizer.py` — `FIELDS` extended to all 50 telemetry values, plus the
  leading `IMU` record tag; a
  `REQUIRED_FIELDS` set so it tolerates older/newer firmware, and the status
  line now shows the measured sampling rate.
* `imu_visualizer_v2.py` — `DEFAULT_FIELDS` extended, the parser accepts both
  shorter and longer lines, and two new panels were added:
  * **Estimator gyro bias (deg/s)** — `ebx/eby/ebz`. Flat near zero means the
    estimator is not fighting a real bias; a slow ramp means the calibration
    left some behind.
  * **Calibration / sampling health** — `rate_hz`, `accel_rms_g`, `mag_rms_ut`.

Run them with:

```bash
python3 -m pip install pyserial matplotlib numpy
python3 tools/imu_visualizer.py    /dev/cu.usbmodem1101
python3 tools/imu_visualizer_v2.py --port auto
```

---

## 11. Testing

### Host tests (no hardware)

```bash
test/host/run.sh
```

Compiles the **real** firmware math sources with the host C++ compiler and
runs assertions covering:

* quaternion ↔ matrix ↔ Euler round trips,
* the frozen `rotateBodyToWorld` contract (including that the matrix form
  agrees with the sequential form),
* filter DC gain, attenuation and pass-through-when-disabled,
* estimator leveling, 30° roll convergence, gyro integration, bias rejection
  and bias learning, NaN robustness,
* the six-position accel solve recovering a known bias + matrix,
* the LM sphere and ellipsoid fits recovering a known offset and radius,
* end-to-end yaw tracking against a synthetic ground truth.

The same script also runs servo-motion and command-parser host tests. Host tests
exercise portable math and logic only; they do not exercise Arduino hardware,
I2C contention, FreeRTOS scheduling, the live guided-calibration UI or sensor
mounting. See [`test/host/README.md`](../../test/host/README.md) for exact scope.

### Firmware build

```bash
pio run -e esp32-s3-devkitm-1
```

### Bench checks

1. **Level and still** → `roll ≈ pitch ≈ 0`, `acc_norm_g ≈ 1.000`,
   `rate_hz ≈ 200`, `still = 1`.
2. **Roll the robot 30°** → `roll ≈ 30` within ~2 s, `pitch` stays near 0.
3. **Yaw the robot left by hand** → `yaw` decreases (project convention).
4. **Lift the robot and hold it** → `still = 0`, the published `trust`
   diagnostic falls toward 0, and `rollG/pitchG` can drift away from
   `roll/pitch`. The estimator itself does not currently use `trust`.
5. Interpret `ebx/eby/ebz` as the estimator's integral bias state, not a direct
   calibration pass/fail indicator.

---

## 12. Design decisions, in one place

| Decision | Alternative | Why this one |
|---|---|---|
| Port PX4's estimator instead of writing a new one | Madgwick / Mahony / EKF | PX4's is battle-tested, has an explicit bias state, and its tuning is documented. The library's built-in Madgwick is disabled because it hides its state and cannot be tuned. |
| Keep `rotateBodyToWorld` verbatim | Use the estimator's quaternion directly | The sequential rotation is the validated contract; changing it would silently change what every plot means. The quaternion is still available for anything that needs it. |
| Full 3×3 accel matrix | PX4's diagonal-only | The GY-91 is hand-soldered; cross-axis terms are real. Costs 6 floats and one 3×3 multiply. |
| LM sphere + ellipsoid mag fit | Library `calibrateMag()` | The library's routine is a min/max box fit with no residual reporting and no ellipsoid refinement. PX4's LM fit gives a real residual we can gate on. |
| Dedicated IMU task on core 1 | Run in `loop()` | Stable `dt`; OTA and OLED cannot starve the estimator. |
| `gyroBiasWeight = 0.02` | PX4's 0.1 | 0.1 is under-damped for a robot that sits still for minutes (see §5.3). |
| Notch disabled by default | Enable at a fixed frequency | A crawler has no dominant vibration line; a mis-tuned notch would just add phase lag. |
| Append-only CSV columns | Reorder / rename | Keeps both visualizers working across firmware revisions. |
| `Preferences` blob per robot | One global blob | Eight robots share the firmware; each needs its own calibration. |
| Shared-bus mutex (`lib/I2CBus`) | Separate I2C buses / no locking | The board has one bus; collisions corrupt sensor reads and are intermittent and hard to debug. |
| `esp_task_wdt` with panic | Rely on the Arduino core default | The core starts the loop task unsubscribed, so a hang otherwise leaves the servos powered. |
| Boot never waits for input | Prompt for the robot number on serial | An erased EEPROM used to hang the boot; now it defaults to 1 and `SETID` fixes it. |

---

## 13. Known limitations

* **Yaw depends on a usable magnetic field.** The AK8963 is sensitive to
  nearby ferrous material and current-carrying wiring. A good fit residual does
  not rule out a locally distorted field; compare `mag_norm`, rotate the robot
  away from interference and repeat calibration if heading is unreliable.
* **Dead reckoning drifts.** By construction. See §8.
* **No vibration/FFT notch.** If you add a motor with a strong line, set
  `FILTER_GYRO_NOTCH_HZ` manually; there is no dynamic notch bank.
* **No thermal calibration.** PX4 has a full thermal offset model. Here the
  boot self-check only *detects* a large temperature change and forces a
  recalibration.
* **Linear acceleration ignores estimator yaw.** The current dead-reckoning
  transform uses yaw zero, so horizontal components and positions are not a
  fully world-aligned navigation estimate.
* **`PLANAR_MODE` is compile-time.** A flying or climbing variant would need it
  as a runtime flag and would need a more complete gravity/position solution.

---

## 14. File map

| File | Role |
|---|---|
| [`lib/IMU/ImuMath.h`](../../lib/IMU/ImuMath.h) | Vec3 / Mat3 / Quat, frame-agnostic, host-testable |
| [`lib/IMU/ImuFilters.h`](../../lib/IMU/ImuFilters.h) / [`.cpp`](../../lib/IMU/ImuFilters.cpp) | PX4 filter ports + pipelines |
| [`lib/IMU/ImuAttitudeEstimator.h`](../../lib/IMU/ImuAttitudeEstimator.h) / [`.cpp`](../../lib/IMU/ImuAttitudeEstimator.cpp) | PX4 `attitude_estimator_q` port |
| [`lib/IMU/ImuCalibration.h`](../../lib/IMU/ImuCalibration.h) / [`.cpp`](../../lib/IMU/ImuCalibration.cpp) | calibration model, six-position solve, LM mag fit, persistence |
| [`lib/IMU/ImuRotation.h`](../../lib/IMU/ImuRotation.h) / [`.cpp`](../../lib/IMU/ImuRotation.cpp) | the frozen body→world rotation contract |
| [`lib/IMU/AHRS.h`](../../lib/IMU/AHRS.h) / [`.cpp`](../../lib/IMU/AHRS.cpp) | orchestration: task, pipeline stages, calibration UI, ZUPT |
| [`lib/IMU/ImuTelemetry.h`](../../lib/IMU/ImuTelemetry.h) / [`.cpp`](../../lib/IMU/ImuTelemetry.cpp) | CSV formatting (shared by serial and TCP streaming) |
| [`lib/I2CBus/`](../../lib/I2CBus/I2CBus.h) | shared I2C mutex between OLED and IMU |
| [`lib/SystemWatchdog/`](../../lib/SystemWatchdog/SystemWatchdog.h) | `esp_task_wdt` wrapper |
| [`lib/CommandServer/`](../../lib/CommandServer/CommandServer.h) | serial + TCP command protocol |
| [`include/config.h`](../../include/config.h) | pins, ranges, task core/priority/stack, deadband, I2C and watchdog settings |
| [`test/host/`](../../test/host) | host test harness |
| [`archive/imu-pre-px4/`](../../archive/imu-pre-px4) | the previous implementation, for reference |
