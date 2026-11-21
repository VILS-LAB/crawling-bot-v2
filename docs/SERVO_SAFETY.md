# Servo motion safety

The two leg servos are the only part of the robot a student's policy can
physically break. A policy that jumps straight from 20° to 160° in one write
draws a current spike, strips a gear, or slams the leg into the ground.

`lib/ServoControl` makes that hard to do by accident. **Every** public move goes
through a trapezoidal motion profile, and the driver remembers where each leg
is, so a new target always starts from the real position instead of snapping.

## What the driver guarantees

| Guarantee | How |
|---|---|
| No instant jumps | every move is ramped: accelerate → cruise → decelerate |
| No overshoot | the braking point is derived from the remaining distance |
| No jump when retargeting | a new target mid-move just brakes and heads for it |
| No slamming into the ground | soft limits clamp every target *and* every profile step |
| No runaway after a loop stall | `update()` clamps `dt` to `SERVO_UPDATE_MAX_DT_MS` |
| Position is always known | `getCurrentDownAngle()` / `getCurrentUpAngle()` |

The one unavoidable instant write is the power-on park in `begin()`: the
firmware cannot know where the horn physically is until it drives it. **Place
the legs at the park pose before switching the robot on.**

## Two ways to move

### Non-blocking (preferred)

```cpp
void loop() {
    servos.update();        // must run every loop

    servos.moveDownTo(120); // sets a target, returns immediately
    servos.moveUpTo(60);
}
```

`update()` is cheap and never blocks. It advances both profiles by the elapsed
time and only touches the PWM peripheral when the rounded angle changed.

### Blocking

```cpp
servos.moveDownSmooth(120);   // returns when the leg arrives
servos.moveBothSmooth(120, 60);
```

These still ramp; they just spin on `update()` internally. Fine in `setup()` or
a simple sketch, but they block the display, OTA and telemetry while running.

## Testing the servos from the serial monitor

`src/main.cpp` ships a small test console. Open the serial monitor at 115200
baud, newline terminated, and type:

| Command | Effect |
|---|---|
| `d <angle>` | move the down leg to `<angle>` (0-180) |
| `u <angle>` | move the up leg to `<angle>` |
| `b <down> <up>` | move both legs |
| `s` | stop both legs where they are |
| `p` | go to the park (initial) pose |
| `t` | go to the test pose (90/90) |
| `?` | print the current angles and targets |

Every command goes through the ramped profile, so even a typo like `d 180`
cannot slam the leg into the ground. The IMU calibration menu (`q`) still works;
the servo console stays out of its way.

## Tuning

All in [include/config.h](../include/config.h):

| Macro | Default | Meaning |
|---|---|---|
| `SERVO_MAX_SPEED_DPS` | `180.0f` | cruise speed, degrees/second |
| `SERVO_ACCEL_DPS2` | `600.0f` | acceleration, degrees/second² |
| `SERVO_DOWN_MIN_ANGLE` / `SERVO_DOWN_MAX_ANGLE` | `0` / `180` | soft limits for the down leg |
| `SERVO_UP_MIN_ANGLE` / `SERVO_UP_MAX_ANGLE` | `0` / `180` | soft limits for the up leg |
| `SERVO_UPDATE_MAX_DT_MS` | `50` | largest time step one `update()` will integrate |

At the defaults a full 180° sweep takes about **1.3 s**. Lower the speed if the
legs still hit the ground; raise it if the robot feels sluggish.

Narrow the soft limits to the real safe range of the assembled robot — that is
the single most effective protection, because it makes an out-of-range action
from a policy a no-op instead of a crash.

Runtime tuning:

```cpp
servos.setSpeed(90.0f);    // degrees per second
servos.setAccel(300.0f);   // degrees per second squared
servos.setDownLimits(20, 160);
servos.setUpLimits(20, 160);
servos.stop();             // freeze both legs where they are
```

## API summary

| Method | Blocks? | Notes |
|---|---|---|
| `moveDownTo(angle)` / `moveUpTo(angle)` | no | sets a target, clamped to the soft limits |
| `moveBothTo(down, up)` | no | both targets at once |
| `update()` | no | call every `loop()` |
| `isMoving()` | no | true while either leg is travelling |
| `stop()` | no | target := current position |
| `moveDownSmooth(angle)` / `moveUpSmooth(angle)` | yes | ramped, returns when done |
| `moveBothSmooth(down, up)` | yes | ramped, returns when both are done |
| `waitUntilIdle(timeoutMs = 0)` | yes | `0` estimates from the profile |
| `moveDown(angle)` / `moveUp(angle)` | yes | legacy names, now ramped |
| `setInitialPosition()` / `setTestPosition()` | yes | ramped presets |
| `setSpeed()` / `setAccel()` | no | runtime tuning |
| `setDownLimits()` / `setUpLimits()` | no | runtime soft limits |
| `getCurrentDownAngle()` / `getCurrentUpAngle()` | no | commanded position |
| `getDownTarget()` / `getUpTarget()` | no | current target |

### Legacy `stepDelay` argument

The old `moveDownSmooth(angle, stepDelay)` moved 2° per `stepDelay` ms. That
argument is still accepted and converted to a cruise speed of
`2000 / stepDelay` °/s, so existing sketches keep their pace — but they now get
the ramp and the soft limits too. Pass `0` (the default) to use
`SERVO_MAX_SPEED_DPS`.

## How the profile works

`lib/ServoControl/ServoMotion.h` is a dependency-free trapezoid generator, which
is why it can be unit-tested on the host:

```
speed
  ^
  |      ______________
  |     /              \
  |    /                \
  |   /                  \
  +--+--------------------+----> time
     accel   cruise   decel
```

Each `update()`:

1. clamps `dt` to `SERVO_UPDATE_MAX_DT_MS`,
2. computes the fastest speed that still allows a full stop within the
   remaining distance (the discrete-time braking limit),
3. ramps the velocity toward that speed, never faster than `accel`,
4. integrates the angle, snapping exactly onto the target when it lands,
5. clamps to the soft limits.

Because the braking limit is recomputed from the *current* distance every step,
changing the target mid-move is safe: the leg simply brakes and heads for the
new target.

## Tests

The profile is covered by host tests that need no hardware:

```bash
test/host/run.sh
```

They check that the axis lands exactly on the target, never exceeds the speed or
acceleration limits, never overshoots, stays continuous when retargeted
mid-move, respects the soft limits, and survives degenerate `dt`/config values.

## What this cannot protect against

- **Wrong wiring or a dead servo.** Use the boot health check
  ([SERVO_HEALTH.md](SERVO_HEALTH.md)) to catch that.
- **A leg that is mechanically blocked.** The profile will keep pushing at the
  configured speed; there is no current sensing to detect a stall.
- **Power-on position.** The park write in `begin()` is instant by necessity.
- **A student bypassing the driver** and calling `servo.write()` directly.
