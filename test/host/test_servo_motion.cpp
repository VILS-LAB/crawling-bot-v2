// Host-side tests for the servo motion profile.
//
// Compiles the real lib/ServoControl/ServoMotion.cpp with the system compiler,
// so the ramp that protects the gears is verified without a robot.
//
// Build & run:
//   test/host/run.sh

#include <math.h>
#include <stdio.h>

#include "../../lib/ServoControl/ServoMotion.h"

namespace
{

int gFailures = 0;
int gChecks = 0;

void check(bool condition, const char *what)
{
    ++gChecks;
    if (!condition)
    {
        ++gFailures;
        printf("  FAIL: %s\n", what);
    }
}

void checkNear(float actual, float expected, float tolerance, const char *what)
{
    ++gChecks;
    if (!(fabsf(actual - expected) <= tolerance))
    {
        ++gFailures;
        printf("  FAIL: %s (got %.6f, expected %.6f +- %.6f)\n", what, actual, expected,
               tolerance);
    }
}

void section(const char *name)
{
    printf("\n[%s]\n", name);
}

ServoMotionConfig defaultConfig()
{
    ServoMotionConfig config;
    config.maxSpeed = 180.0f;
    config.accel = 600.0f;
    return config;
}

ServoMotionState makeState(float angle, float target)
{
    ServoMotionState state;
    state.angle = angle;
    state.target = target;
    state.velocity = 0.0f;
    state.minAngle = 0.0f;
    state.maxAngle = 180.0f;
    return state;
}

// Runs the profile to completion (or a hard cap) and returns the number of
// simulated milliseconds.
float runToIdle(ServoMotionState &state, const ServoMotionConfig &config, float dt,
                int maxSteps = 100000)
{
    int steps = 0;
    while (!servoMotionIdle(state) && steps < maxSteps)
    {
        servoMotionStep(state, config, dt);
        ++steps;
    }
    return steps * dt * 1000.0f;
}

// ---------------------------------------------------------------------------
void testReachesTargetExactly()
{
    section("reaches target exactly");

    const ServoMotionConfig config = defaultConfig();
    ServoMotionState state = makeState(0.0f, 180.0f);

    runToIdle(state, config, 0.01f);

    checkNear(state.angle, 180.0f, 0.001f, "lands on the target");
    checkNear(state.velocity, 0.0f, 0.001f, "stops at the target");
    check(servoMotionIdle(state), "reports idle after arrival");
}

void testNeverExceedsSpeedOrAccel()
{
    section("respects speed and acceleration limits");

    const ServoMotionConfig config = defaultConfig();
    ServoMotionState state = makeState(0.0f, 180.0f);

    const float dt = 0.005f;
    float previousVelocity = 0.0f;
    float maxSpeedSeen = 0.0f;
    float maxAccelSeen = 0.0f;

    for (int i = 0; i < 2000 && !servoMotionIdle(state); ++i)
    {
        servoMotionStep(state, config, dt);
        const float speed = fabsf(state.velocity);
        const float accel = fabsf(state.velocity - previousVelocity) / dt;
        if (speed > maxSpeedSeen)
            maxSpeedSeen = speed;
        if (accel > maxAccelSeen)
            maxAccelSeen = accel;
        previousVelocity = state.velocity;
    }

    check(maxSpeedSeen <= config.maxSpeed + 0.5f, "never exceeds max speed");
    check(maxAccelSeen <= config.accel + 1.0f, "never exceeds max acceleration");
}

void testNoOvershoot()
{
    section("does not overshoot");

    const ServoMotionConfig config = defaultConfig();
    ServoMotionState state = makeState(0.0f, 90.0f);

    const float dt = 0.005f;
    float highest = state.angle;
    for (int i = 0; i < 5000 && !servoMotionIdle(state); ++i)
    {
        servoMotionStep(state, config, dt);
        if (state.angle > highest)
            highest = state.angle;
    }

    check(highest <= 90.0f + 0.01f, "never passes the target");
    checkNear(state.angle, 90.0f, 0.001f, "settles on the target");
}

void testRetargetMidMoveIsContinuous()
{
    section("retarget mid-move stays continuous");

    const ServoMotionConfig config = defaultConfig();
    ServoMotionState state = makeState(0.0f, 180.0f);

    const float dt = 0.005f;
    for (int i = 0; i < 40; ++i)
        servoMotionStep(state, config, dt);

    const float before = state.angle;
    state.target = 0.0f; // reverse direction mid-flight
    servoMotionStep(state, config, dt);

    check(fabsf(state.angle - before) < 2.0f, "no jump when the target changes");
    check(state.angle >= 0.0f && state.angle <= 180.0f, "stays inside the limits");

    runToIdle(state, config, dt);
    checkNear(state.angle, 0.0f, 0.001f, "reaches the new target");
}

void testSoftLimits()
{
    section("soft limits");

    const ServoMotionConfig config = defaultConfig();
    ServoMotionState state = makeState(90.0f, 90.0f);
    state.minAngle = 30.0f;
    state.maxAngle = 150.0f;

    checkNear(servoMotionClamp(state, -50.0f), 30.0f, 0.001f, "clamps below min");
    checkNear(servoMotionClamp(state, 500.0f), 150.0f, 0.001f, "clamps above max");
    checkNear(servoMotionClamp(state, 100.0f), 100.0f, 0.001f, "passes through in range");

    state.target = 180.0f;
    runToIdle(state, config, 0.01f);
    check(state.angle <= 150.0f + 0.001f, "profile never drives past max limit");

    state.target = 0.0f;
    runToIdle(state, config, 0.01f);
    check(state.angle >= 30.0f - 0.001f, "profile never drives past min limit");
}

void testZeroAndNegativeDt()
{
    section("degenerate dt");

    const ServoMotionConfig config = defaultConfig();
    ServoMotionState state = makeState(10.0f, 100.0f);

    checkNear(servoMotionStep(state, config, 0.0f), 10.0f, 0.0001f, "dt=0 is a no-op");
    checkNear(servoMotionStep(state, config, -1.0f), 10.0f, 0.0001f, "dt<0 is a no-op");
    checkNear(state.velocity, 0.0f, 0.0001f, "velocity untouched by degenerate dt");
}

void testDegenerateConfig()
{
    section("degenerate config");

    ServoMotionConfig config;
    config.maxSpeed = 0.0f;
    config.accel = 0.0f;

    ServoMotionState state = makeState(0.0f, 45.0f);
    runToIdle(state, config, 0.01f);

    checkNear(state.angle, 45.0f, 0.001f, "still reaches the target with a zero config");
}

void testEstimateIsReasonable()
{
    section("time estimate");

    const ServoMotionConfig config = defaultConfig();

    checkNear(servoMotionEstimateMs(config, 0.0f), 0.0f, 0.001f, "zero distance is zero time");

    // 180 deg at 180 dps with 600 dps^2: 0.3 s ramp up + 0.7 s cruise + 0.3 s
    // ramp down = 1.3 s.
    checkNear(servoMotionEstimateMs(config, 180.0f), 1300.0f, 5.0f, "full sweep estimate");

    // The estimate must not be shorter than the real move.
    ServoMotionState state = makeState(0.0f, 180.0f);
    const float actualMs = runToIdle(state, config, 0.001f);
    check(servoMotionEstimateMs(config, 180.0f) >= actualMs - 1.0f,
          "estimate is not optimistic");
}

void testSmallMoveIsTriangular()
{
    section("small move");

    const ServoMotionConfig config = defaultConfig();
    ServoMotionState state = makeState(90.0f, 95.0f);

    float peakSpeed = 0.0f;
    const float dt = 0.001f;
    for (int i = 0; i < 5000 && !servoMotionIdle(state); ++i)
    {
        servoMotionStep(state, config, dt);
        if (fabsf(state.velocity) > peakSpeed)
            peakSpeed = fabsf(state.velocity);
    }

    check(peakSpeed < config.maxSpeed, "short move never reaches cruise speed");
    checkNear(state.angle, 95.0f, 0.001f, "short move lands on target");
}

} // namespace

int main()
{
    printf("servo motion profile tests\n");

    testReachesTargetExactly();
    testNeverExceedsSpeedOrAccel();
    testNoOvershoot();
    testRetargetMidMoveIsContinuous();
    testSoftLimits();
    testZeroAndNegativeDt();
    testDegenerateConfig();
    testEstimateIsReasonable();
    testSmallMoveIsTriangular();

    printf("\n%d checks, %d failures\n", gChecks, gFailures);
    return gFailures == 0 ? 0 : 1;
}
