# The body → world rotation contract

This document describes the rotation the project uses to move a body-frame
vector into the world frame. It is **frozen**: the hardware and both
visualizers were validated against it, so it is not replaced by the attitude
estimator's internal math.

Implementation: [`ImuRotation.cpp`](../../lib/IMU/ImuRotation.cpp).
Full architecture: [ARCHITECTURE.md](./ARCHITECTURE.md).

## Frames

```text
firmware body  : X forward, Y left,  Z up     (FLU, right-handed)
firmware world : X north,   Y west,  Z up     (N-W-U, right-handed)
```

The sensor is mounted with sensor/body +X toward chassis right and +Y toward
chassis forward. The code does not rotate sensor samples into a separate chassis
frame, so the firmware's body-frame "forward" label does not match physical
chassis forward on this mounting. See
[ARCHITECTURE.md §2](./ARCHITECTURE.md#2-frame-conventions-read-this-before-touching-any-math)
for the `heading = (270 - yaw) mod 360` conversion the visualizers apply.

## The rotation

The rotation is applied sequentially, X then Y then Z:

```text
body acceleration -> rotation X (roll) -> rotation Y (pitch) -> rotation Z (yaw) -> world
```

For angles `x = roll`, `y = pitch`, `z = yaw` (degrees):

```text
Ax1 = AccX
Ay1 = AccY*cos(x) - AccZ*sin(x)
Az1 = AccY*sin(x) + AccZ*cos(x)

Ax2 = Ax1*cos(y) + Az1*sin(y)
Az2 = -Ax1*sin(y) + Az1*cos(y)

RealAccX = Ax2*cos(z) - Ay1*sin(z)
RealAccY = Ax2*sin(z) + Ay1*cos(z)
RealAccZ = Az2
```

Algebraically this is

```text
R = Rz(yaw) · Ry(pitch) · Rx(roll)
```

which is the same convention the attitude estimator reports Euler angles in
(`Quat::toEulerDegrees`). `bodyToWorldMatrix()` exposes the equivalent matrix;
the host test asserts the two agree to `1e-5`.

## Gravity removal

After the rotation, the Z component is the sum of the rotated acceleration's
vertical component and gravity. The AHRS currently calls this transform with
yaw zero, so its horizontal components are not yaw-aligned to the estimator's
world frame. A slowly-adapting `gravityReference` (updated only while the robot
is still) is subtracted from Z:

```text
linearAccel.z = worldAccel.z - gravityReference
```

This absorbs the small constant Z offset that would otherwise integrate into
unbounded vertical drift. It is not a substitute for a full gravity model.

## Why it is kept as a sequential rotation

The estimator produces a quaternion, and `q.rotate(v)` would give the same
result. The sequential form is kept because:

1. it is the exact code path the hardware was validated with;
2. it makes the contract readable and auditable without quaternion algebra;
3. changing it would silently change the meaning of every plot in both
   visualizers.

The quaternion is available through the `AHRS` C++ API (`getQuatW/X/Y/Z`). It
is not part of the current `IMU,` CSV fields or the `GET` snapshot. A client
that needs the quaternion must use an API that exposes those getters or extend
the protocol deliberately.

## Verification

`test/host/test_imu_math.cpp` → `testRotationContract()` checks:

* level: world == body,
* roll 90°: body +Y → world +Z, body +Z → world −Y,
* yaw 90°: body +X → world +Y,
* the matrix form agrees with the sequential form to `1e-5`,
* at rest and level, the world Z component is exactly `+1 g`.
