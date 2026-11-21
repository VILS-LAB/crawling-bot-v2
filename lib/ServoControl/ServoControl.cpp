#include "ServoControl.h"
#include "../../include/config.h"

#include <math.h>

namespace
{
// Pulse range handed to attach(). Keep in sync with expectedPulseUs().
constexpr int kMinPulseUs = MIN_PULSE_WIDTH;
constexpr int kMaxPulseUs = MAX_PULSE_WIDTH;

int expectedPulseUs(int angle)
{
    return map(angle, 0, 180, kMinPulseUs, kMaxPulseUs);
}

int roundAngle(float angle)
{
    return static_cast<int>(angle + (angle >= 0.0f ? 0.5f : -0.5f));
}
} // namespace

ServoControl::ServoControl(uint8_t pinDown, uint8_t pinUp)
    : pinDown(pinDown), pinUp(pinUp),
      downAttached(false),
      upAttached(false),
      lastUpdateMs(0),
      lastWrittenDown(INITIAL_DOWN_ANGLE),
      lastWrittenUp(INITIAL_UP_ANGLE)
{
    motionConfig.maxSpeed = SERVO_MAX_SPEED_DPS;
    motionConfig.accel = SERVO_ACCEL_DPS2;

    downState.angle = INITIAL_DOWN_ANGLE;
    downState.target = INITIAL_DOWN_ANGLE;
    downState.minAngle = SERVO_DOWN_MIN_ANGLE;
    downState.maxAngle = SERVO_DOWN_MAX_ANGLE;

    upState.angle = INITIAL_UP_ANGLE;
    upState.target = INITIAL_UP_ANGLE;
    upState.minAngle = SERVO_UP_MIN_ANGLE;
    upState.maxAngle = SERVO_UP_MAX_ANGLE;
}

bool ServoControl::begin()
{
    downAttached = servoDown.attach(pinDown, kMinPulseUs, kMaxPulseUs) != 0;
    upAttached = servoUp.attach(pinUp, kMinPulseUs, kMaxPulseUs) != 0;

    // Power-on park. This is the only instant write in the driver: the horn
    // position is unknown until the servo is driven, so there is nothing to
    // ramp from. Students should place the legs at the park pose before power.
    if (downAttached)
        servoDown.write(INITIAL_DOWN_ANGLE);
    if (upAttached)
        servoUp.write(INITIAL_UP_ANGLE);

    downState.angle = downState.target = INITIAL_DOWN_ANGLE;
    downState.velocity = 0.0f;
    upState.angle = upState.target = INITIAL_UP_ANGLE;
    upState.velocity = 0.0f;

    lastWrittenDown = INITIAL_DOWN_ANGLE;
    lastWrittenUp = INITIAL_UP_ANGLE;
    lastUpdateMs = millis();

    return downAttached && upAttached;
}

bool ServoControl::isDownAttached()
{
    return servoDown.attached();
}

bool ServoControl::isUpAttached()
{
    return servoUp.attached();
}

ServoHealthCode ServoControl::checkDown()
{
    if (!downAttached)
        return SERVO_HEALTH_ATTACH_FAILED;
    return checkServo(servoDown, roundAngle(downState.angle));
}

ServoHealthCode ServoControl::checkUp()
{
    if (!upAttached)
        return SERVO_HEALTH_ATTACH_FAILED;
    return checkServo(servoUp, roundAngle(upState.angle));
}

ServoHealthCode ServoControl::checkServo(Servo &servo, int angle)
{
    if (!servo.attached())
        return SERVO_HEALTH_NOT_ATTACHED;

    servo.write(angle);
    delay(SERVO_HEALTH_SETTLE_MS);

    const int expected = expectedPulseUs(angle);
    const int actual = servo.readMicroseconds();
    if (abs(actual - expected) > SERVO_HEALTH_PULSE_TOLERANCE_US)
        return SERVO_HEALTH_PULSE_MISMATCH;

    return SERVO_HEALTH_OK;
}

// ---------------------------------------------------------------------------
// Non-blocking API
// ---------------------------------------------------------------------------

void ServoControl::moveDownTo(int angle)
{
    downState.target = servoMotionClamp(downState, static_cast<float>(angle));
}

void ServoControl::moveUpTo(int angle)
{
    upState.target = servoMotionClamp(upState, static_cast<float>(angle));
}

void ServoControl::moveBothTo(int downAngle, int upAngle)
{
    moveDownTo(downAngle);
    moveUpTo(upAngle);
}

void ServoControl::update()
{
    const uint32_t now = millis();
    uint32_t elapsedMs = now - lastUpdateMs;
    lastUpdateMs = now;

    // A long stall (OTA, display refresh, a student's delay()) must not turn
    // into a huge single step. Clamp dt and let the profile catch up over the
    // next few updates instead.
    if (elapsedMs > SERVO_UPDATE_MAX_DT_MS)
        elapsedMs = SERVO_UPDATE_MAX_DT_MS;

    const float dt = static_cast<float>(elapsedMs) / 1000.0f;

    const float downAngle = servoMotionStep(downState, motionConfig, dt);
    const float upAngle = servoMotionStep(upState, motionConfig, dt);

    const int downRounded = roundAngle(downAngle);
    const int upRounded = roundAngle(upAngle);

    // Only touch the PWM peripheral when the commanded angle actually changed.
    if (downRounded != lastWrittenDown)
        writeDown(downRounded);
    if (upRounded != lastWrittenUp)
        writeUp(upRounded);
}

bool ServoControl::isMoving() const
{
    return !servoMotionIdle(downState) || !servoMotionIdle(upState);
}

void ServoControl::stop()
{
    downState.target = downState.angle;
    downState.velocity = 0.0f;
    upState.target = upState.angle;
    upState.velocity = 0.0f;
}

// ---------------------------------------------------------------------------
// Blocking helpers
// ---------------------------------------------------------------------------

bool ServoControl::moveDownSmooth(int targetAngle, int stepDelayMs)
{
    const float savedSpeed = motionConfig.maxSpeed;
    applyLegacyStepDelay(stepDelayMs);
    moveDownTo(targetAngle);
    const bool done = waitUntilIdle(0);
    motionConfig.maxSpeed = savedSpeed;
    return done;
}

bool ServoControl::moveUpSmooth(int targetAngle, int stepDelayMs)
{
    const float savedSpeed = motionConfig.maxSpeed;
    applyLegacyStepDelay(stepDelayMs);
    moveUpTo(targetAngle);
    const bool done = waitUntilIdle(0);
    motionConfig.maxSpeed = savedSpeed;
    return done;
}

bool ServoControl::moveBothSmooth(int downAngle, int upAngle, int stepDelayMs)
{
    const float savedSpeed = motionConfig.maxSpeed;
    applyLegacyStepDelay(stepDelayMs);
    moveBothTo(downAngle, upAngle);
    const bool done = waitUntilIdle(0);
    motionConfig.maxSpeed = savedSpeed;
    return done;
}

// The old API moved 2 degrees per stepDelayMs. Translate that into a cruise
// speed so a sketch written against the old signature keeps its pace, while
// still getting the ramp and the soft limits. The caller restores the speed
// afterwards, so this never leaks into later moves.
void ServoControl::applyLegacyStepDelay(int stepDelayMs)
{
    if (stepDelayMs <= 0)
        return;
    setSpeed(2000.0f / static_cast<float>(stepDelayMs));
}

bool ServoControl::waitUntilIdle(uint32_t timeoutMs)
{
    if (timeoutMs == 0)
        timeoutMs = estimateTimeoutMs();

    const uint32_t start = millis();
    while (isMoving())
    {
        update();
        if (millis() - start >= timeoutMs)
            return false;
        delay(1);
    }
    return true;
}

uint32_t ServoControl::estimateTimeoutMs() const
{
    const float downDistance = fabsf(downState.target - downState.angle);
    const float upDistance = fabsf(upState.target - upState.angle);
    const float distance = fmaxf(downDistance, upDistance);

    // 25% margin plus a fixed allowance for the last few update ticks.
    return static_cast<uint32_t>(servoMotionEstimateMs(motionConfig, distance) * 1.25f) + 100;
}

// ---------------------------------------------------------------------------
// Presets and tuning
// ---------------------------------------------------------------------------

void ServoControl::setInitialPosition()
{
    moveBothSmooth(INITIAL_DOWN_ANGLE, INITIAL_UP_ANGLE, 0);
}

void ServoControl::setTestPosition()
{
    moveBothSmooth(90, 90, 0);
}

void ServoControl::setDownLimits(int minAngle, int maxAngle)
{
    if (minAngle > maxAngle)
    {
        const int swap = minAngle;
        minAngle = maxAngle;
        maxAngle = swap;
    }
    downState.minAngle = static_cast<float>(constrain(minAngle, 0, 180));
    downState.maxAngle = static_cast<float>(constrain(maxAngle, 0, 180));
    downState.target = servoMotionClamp(downState, downState.target);
    downState.angle = servoMotionClamp(downState, downState.angle);
}

void ServoControl::setUpLimits(int minAngle, int maxAngle)
{
    if (minAngle > maxAngle)
    {
        const int swap = minAngle;
        minAngle = maxAngle;
        maxAngle = swap;
    }
    upState.minAngle = static_cast<float>(constrain(minAngle, 0, 180));
    upState.maxAngle = static_cast<float>(constrain(maxAngle, 0, 180));
    upState.target = servoMotionClamp(upState, upState.target);
    upState.angle = servoMotionClamp(upState, upState.angle);
}

void ServoControl::setSpeed(float degreesPerSecond)
{
    if (degreesPerSecond > 0.0f)
        motionConfig.maxSpeed = degreesPerSecond;
}

void ServoControl::setAccel(float degreesPerSecondSquared)
{
    if (degreesPerSecondSquared > 0.0f)
        motionConfig.accel = degreesPerSecondSquared;
}

int ServoControl::getCurrentDownAngle() const
{
    return roundAngle(downState.angle);
}

int ServoControl::getCurrentUpAngle() const
{
    return roundAngle(upState.angle);
}

int ServoControl::getDownTarget() const
{
    return roundAngle(downState.target);
}

int ServoControl::getUpTarget() const
{
    return roundAngle(upState.target);
}

// ---------------------------------------------------------------------------
// Internals
// ---------------------------------------------------------------------------

void ServoControl::writeDown(int angle)
{
    if (downAttached)
        servoDown.write(angle);
    lastWrittenDown = angle;
}

void ServoControl::writeUp(int angle)
{
    if (upAttached)
        servoUp.write(angle);
    lastWrittenUp = angle;
}
