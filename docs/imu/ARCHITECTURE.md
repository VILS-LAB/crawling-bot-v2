# IMU / AHRS architecture

Everything about how this robot turns MPU9250 register reads into a usable
attitude, a gravity-free acceleration and a short-term position estimate.

The short version: **the signal chain is a port of PX4's IMU path**, with the
project's own body→world rotation kept exactly as it was, because that rotation
is the one that has been validated against the hardware and both visualizers.

This is the **third** generation of the IMU stack. If you want to know what the
previous two did and why they were replaced, read
[IMPLEMENTATION_HISTORY.md](./IMPLEMENTATION_HISTORY.md). This document only
describes what runs today.

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
bugs. The **firmware** uses one convention everywhere:

```
body  frame : X forward, Y left,  Z up      (FLU, right-handed)
world frame : X north,   Y west,  Z up      (N-W-U, right-handed)
Euler       : R = Rz(yaw) * Ry(pitch) * Rx(roll)     (ZYX intrinsic)
quaternion  : body -> world,  v_world = q * v_body * q^-1
units       : accel m/s^2, gyro deg/s (rad/s inside the estimator),
              mag uT, angles deg, position m, velocity m/s
```

### How the IMU is physically mounted (important)

The firmware body frame is FLU, but on this robot the GY-91 is bolted down so
that **its axes line up with the chassis**:

```
robot forward  = body +Y
robot right    = body +X
robot up       = body +Z
```

So the firmware's "forward" axis (body +X) actually points at the robot's
**right-hand side**, and the robot's nose is body **+Y**. Nothing in the
firmware is wrong — the estimator is self-consistent — but any consumer that
wants a compass heading must account for the mounting offset. The physical
mounting is a 90° rotation, which shows up as a **270°** constant because yaw is
counter-clockwise from north while a compass heading is clockwise from north:

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

If you ever remount the IMU so body +X points forward, set
`Config.heading_offset_deg = 0` in `tools/imu_visualizer_v2.py` and
`HEADING_OFFSET_DEG = 0` in `tools/imu_visualizer.py`.

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
        ▼           │                                              │
    runFilters      │  gyro : median(3) → notch → LPF2p(40 Hz)    │
        │           │  accel: LPF2p(30 Hz)                        │
        │           │  mag  : LPF2p(5 Hz)                         │
        ▼           │                                              │
   runEstimator     │  PX4 attitude_estimator_q port               │
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

The Arduino `loop()` also drives the OLED (I2C, ~10 ms per refresh), the servos
and the OTA handler. If the estimator ran inside `loop()`, its `dt` would jitter
by tens of milliseconds every time the display refreshed, and the quaternion
integration would accumulate error proportional to that jitter.

So the sampling loop lives in its own FreeRTOS task:

```c
#define IMU_TASK_CORE     1   // core 0 hosts loopTask + WiFi/OTA
#define IMU_TASK_PRIORITY 5
#define IMU_TASK_STACK    4096
```

* **Core 1** is chosen because core 0 is where the Arduino `loopTask` and the
  WiFi/OTA stack live. Keeping the IMU off core 0 means an OTA transfer cannot
  starve the estimator.
* If you ever need core 1 for something else, set `IMU_TASK_CORE` to `0`. The
  task then shares core 0 with the loop and WiFi; it still works, but `dt`
  becomes noisier. There is no need to disable OTA — the two coexist either way.
* The task is **paused** (`taskPaused`) during calibration, because calibration
  needs exclusive access to the sensor and drives the OLED.
* `ahrs.update()` in `loop()` is a cheap copy of the latest snapshot under a
  spinlock. It never blocks on I2C.

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
4. Per axis, compute the mean and the standard deviation, **and** run a
   `MedianFilter3` over the same samples.
5. Reject the run if `sigma > 0.20 °/s` **or** `|median − mean| > 0.6 °/s`
   (PX4's `maxoff = radians(0.6)`). Retry up to 5 times.

The median/mean check is the important one: a slow, steady rotation has a small
standard deviation but a large median/mean gap, so it catches motion that a
variance test alone would miss.

### 4.2 Accelerometer — `calibrateAccelerometer()`

Port of PX4's `do_accel_calibration`, six guided positions:

| Side | Reference vector (g) |
|---|---|
| `X+` (tail down) | `( 1, 0, 0)` |
| `X-` (nose down) | `(-1, 0, 0)` |
| `Y+` (left) | `( 0, 1, 0)` |
| `Y-` (right) | `( 0,-1, 0)` |
| `Z+` (upside down) | `( 0, 0, 1)` |
| `Z-` (right side up) | `( 0, 0,-1)` |

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
fitted radius for the boot self-check.

> **Unit note.** PX4 works in Gauss. This project works in µT (1 G = 100 µT).
> The LM thresholds in `ImuCalibration.cpp` are scaled accordingly — if you port
> more PX4 calibration code, check the units first.

### 4.4 Boot self-check — `checkCalibrationAtBoot()`

After a 5 s warm-up, if a stored calibration exists, 200 samples are checked:

| Check | Limit |
|---|---|
| `| |accel| − 1 g |` | ≤ 0.05 g |
| `|gyro|` (bias removed) | ≤ 1.0 °/s |
| `|mag|` vs stored field norm | ≤ 20 % |
| temperature drift since calibration | ≤ 15 °C |

Any failure triggers a full guided recalibration.

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
| `LowPassFilter2p` | `LowPassFilter2p.hpp` | accel, gyro, mag, linear accel | 30 / 40 / 5 / 7.5 Hz |
| `NotchFilter` | `NotchFilter.hpp` | gyro (optional) | disabled by default |
| `MedianFilter3` | `MedianFilter.hpp` | gyro spike rejection | — |
| `AlphaFilter` | `AlphaFilter.hpp` | available for time-constant smoothing | — |

### 6.1 Gyro chain

```
median(3) → notch (optional) → LPF2p(40 Hz)
```

* `median(3)` removes single-sample I2C glitches before they reach the notch,
  which would otherwise ring.
* The notch is **disabled by default** (`FILTER_GYRO_NOTCH_HZ = 0`). On a
  crawler there is no dominant motor/prop frequency to notch out. If you add a
  vibration source, set `FILTER_GYRO_NOTCH_HZ` and `FILTER_GYRO_NOTCH_BW_HZ`
  (PX4's `IMU_GYRO_NF1_FRQ` / `IMU_GYRO_NF1_BW`).
* 40 Hz is PX4's `IMU_GYRO_CUTOFF` default.

### 6.2 Accel chain

```
LPF2p(30 Hz)
```

PX4's `IMU_ACCEL_CUTOFF` default. Note that PX4 applies this only to the signal
sent to the controllers, not to the estimator. Here it is applied to the
telemetry/`filteredAccel` path; the estimator is fed the **unfiltered**
calibrated accel, matching PX4, so the estimator's own 0.9–1.1 g gate and the
`k × accel` correction see the real signal.

### 6.3 Linear acceleration chain

```
rotateBodyToWorld → remove gravity → LPF2p(7.5 Hz)
```

The 7.5 Hz cutoff is the value the previous implementation used and the
visualizers were tuned against. The **integration** for velocity/position uses
the unfiltered value, so the dead-reckoning phase response is unchanged.

### 6.4 What was removed

The previous implementation used a per-axis
`Median(5) → Butterworth → adaptive scalar Kalman` chain. It was replaced
because:

* the adaptive Kalman's innovation-scaled measurement noise made the filter
  *slower* exactly when the signal was changing fastest, which is backwards for
  a motion-tracking application;
* three cascaded filters added ~30 ms of group delay to the gyro, which the
  estimator then integrated as real rotation;
* PX4's chain is the one that has been validated on millions of flight hours,
  and matching it makes the behaviour explainable against PX4 documentation.

The old sources are kept in `archive/imu-pre-px4/` for reference.

---

## 7. Still detection and ZUPT

Unchanged in semantics from the previous implementation, because both
visualizers and the `still_failure_mask` telemetry depend on it.

A 40-sample sliding window over the calibrated accel (in g) produces three
independent checks:

| Bit | Check | Limit |
|---|---|---|
| 0 | `| |accel| − 1 g |` | < 0.02 g |
| 1 | `|gyro|` | < 0.5 °/s |
| 2 | accel vector standard deviation | < 0.004 g |

`still = (mask == 0)`. After 200 ms of continuous stillness the robot is
declared static and velocity is zeroed (ZUPT). The mask is published so the
visualizer can show *which* check is failing.

**Trust.** A smoothed 0..1 value that gates how much the accelerometer is
allowed to correct the attitude:

* `1.0` while still,
* `0.0` while moving,
* `0.05` if it has been moving for more than 5 s (a slow fallback so a
  long-duration motion cannot leave the attitude uncorrected forever).

---

## 8. Dead reckoning

```
worldAccel = rotateBodyToWorld(roll, pitch, 0, accel)   # yaw is irrelevant for gravity
linearAccel.xy = worldAccel.xy
linearAccel.z  = worldAccel.z − gravityReference
```

`gravityReference` is a slow (alpha 0.02) estimate of the world-Z gravity
magnitude, updated only while still. It absorbs the small constant Z offset that
would otherwise integrate into unbounded vertical drift.

Then, only while moving:

```
if deadbandMode == 1:  a = |a| < DEADBAND ? 0 : a
if deadbandMode == 2:  a = sign(a) · max(|a| − DEADBAND, 0)
v += a · dt
p += v · dt
```

`PLANAR_MODE` forces `vz = pz = 0`, which is correct for a ground crawler.

**This is dead reckoning, not a position solution.** Double-integrating an IMU
drifts cubically with time; expect centimetres over a few seconds and metres
over a minute. It exists for short-term motion cues and for the visualizer's
top-down plot, not for navigation.

---

## 9. Telemetry contract

One CSV line per `TELEMETRY_INTERVAL_MS` (100 ms), plus a header line at boot
and every 5 s:

```
IMU,t_ms,dt,ax,ay,az,lax,lay,laz,flax,flay,flaz,gx,gy,gz,gbx,gby,gbz,
fgx,fgy,fgz,magx,magy,magz,roll,pitch,yaw,vx,vy,vz,px,py,pz,temp_c,
moving,still,still_failure_mask,stdA,trust,rollG,pitchG,
ebx,eby,ebz,acc_norm_g,mag_norm,rate_hz,accel_rms_g,mag_rms_ut,mag_fit,est_init
```

50 columns. **New columns are only ever appended**, so:

* an older visualizer keeps working (it ignores trailing columns),
* a newer visualizer can read older firmware (missing columns show as `NaN`).

The line is built with a single `snprintf` into a 512-byte buffer and sent with
one `Serial.write`, so the visualizer never sees a half line.

| Column group | Meaning |
|---|---|
| `ax..az` | calibrated accel, m/s², body |
| `lax..laz` | gravity-free accel, m/s², world (unfiltered) |
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
| `trust` | 0..1 accel fusion weight |
| `rollG,pitchG` | gyro-only shadow attitude, ° (drift diagnostic) |
| `ebx..ebz` | **estimator** integral gyro bias, °/s |
| `acc_norm_g` | `|accel|` in g |
| `mag_norm` | `|mag|` in µT |
| `rate_hz` | measured IMU task sampling rate |
| `accel_rms_g` | accel calibration residual, g |
| `mag_rms_ut` | mag calibration residual, µT |
| `mag_fit` | 0 none / 1 sphere / 2 ellipsoid |
| `est_init` | estimator initialized flag |

### Serial console

Send `q` to open the calibration menu (telemetry pauses):

```
1 = gyro only      2 = accel only     3 = mag only     4 = all
5 = status         6 = clear stored   0 = close menu
```

During a calibration, `x` cancels and `?` prints help.

---

## 10. Visualizers

Both tools live in `tools/` and read the same CSV.

| Tool | Purpose |
|---|---|
| [`imu_visualizer.py`](../../tools/imu_visualizer.py) | compact 3×4 matplotlib dashboard: accel, gyro, velocity, attitude vs shadow, 3D body, top-down path |
| [`imu_visualizer_v2.py`](../../tools/imu_visualizer_v2.py) | full dashboard with per-panel Y-range presets, still-failure breakdown, estimator bias, calibration health, logging and replay |

Both were updated for the new columns:

* `imu_visualizer.py` — `FIELDS` extended to all 50 columns, a
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

Compiles the **real** firmware math sources with the system C++ compiler and
runs 79 assertions covering:

* quaternion ↔ matrix ↔ Euler round trips,
* the frozen `rotateBodyToWorld` contract (including that the matrix form
  agrees with the sequential form),
* filter DC gain, attenuation and pass-through-when-disabled,
* estimator leveling, 30° roll convergence, gyro integration, bias rejection
  and bias learning, NaN robustness,
* the six-position accel solve recovering a known bias + matrix,
* the LM sphere and ellipsoid fits recovering a known offset and radius,
* end-to-end yaw tracking against a synthetic ground truth.

This is the fastest way to catch a regression in the math — it runs in about a
second and needs no robot.

### Firmware build

```bash
pio run -e esp32-s3-devkitm-1
```

### Bench checks

1. **Level and still** → `roll ≈ pitch ≈ 0`, `acc_norm_g ≈ 1.000`,
   `rate_hz ≈ 200`, `still = 1`.
2. **Roll the robot 30°** → `roll ≈ 30` within ~2 s, `pitch` stays near 0.
3. **Yaw the robot left by hand** → `yaw` decreases (project convention).
4. **Lift the robot and hold it** → `still = 0`, `trust` falls to 0,
   `rollG/pitchG` start to drift away from `roll/pitch` (that gap is the
   estimator doing its job).
5. **`ebx/eby/ebz`** should sit near zero. A persistent non-zero value means
   the gyro calibration left a bias behind — rerun `1` from the menu.

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

---

## 13. Known limitations

* **Yaw drifts without a good mag calibration.** The AK8963 is noisy and sits
  next to the motor wiring. If `mag_rms_ut` is above ~2 µT, redo the mag
  calibration away from metal.
* **Dead reckoning drifts.** By construction. See §8.
* **No vibration/FFT notch.** If you add a motor with a strong line, set
  `FILTER_GYRO_NOTCH_HZ` manually; there is no dynamic notch bank.
* **No thermal calibration.** PX4 has a full thermal offset model. Here the
  boot self-check only *detects* a large temperature change and forces a
  recalibration.
* **`PLANAR_MODE` is compile-time.** A flying or climbing variant would need it
  as a runtime flag.

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
| [`lib/IMU/ImuTelemetry.h`](../../lib/IMU/ImuTelemetry.h) / [`.cpp`](../../lib/IMU/ImuTelemetry.cpp) | CSV output |
| [`include/config.h`](../../include/config.h) | pins, ranges, task core/priority/stack, deadband |
| [`test/host/`](../../test/host) | host test harness |
| [`archive/imu-pre-px4/`](../../archive/imu-pre-px4) | the previous implementation, for reference |
