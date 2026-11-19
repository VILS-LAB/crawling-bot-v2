#include "ServoControl.h"
#include "../../include/config.h"

namespace
{
// Pulse range handed to attach(). Keep in sync with expectedPulseUs().
constexpr int kMinPulseUs = MIN_PULSE_WIDTH;
constexpr int kMaxPulseUs = MAX_PULSE_WIDTH;

int expectedPulseUs(int angle)
{
    return map(angle, 0, 180, kMinPulseUs, kMaxPulseUs);
}
} // namespace

ServoControl::ServoControl(uint8_t pinDown, uint8_t pinUp) 
    : pinDown(pinDown), pinUp(pinUp), 
      currentDownAngle(INITIAL_DOWN_ANGLE), 
      currentUpAngle(INITIAL_UP_ANGLE),
      downAttached(false),
      upAttached(false) {
}

bool ServoControl::begin() {
    downAttached = servoDown.attach(pinDown, kMinPulseUs, kMaxPulseUs) != 0;
    upAttached = servoUp.attach(pinUp, kMinPulseUs, kMaxPulseUs) != 0;

    if (downAttached)
        servoDown.write(INITIAL_DOWN_ANGLE);
    if (upAttached)
        servoUp.write(INITIAL_UP_ANGLE);

    currentDownAngle = INITIAL_DOWN_ANGLE;
    currentUpAngle = INITIAL_UP_ANGLE;

    return downAttached && upAttached;
}

bool ServoControl::isDownAttached() {
    return servoDown.attached();
}

bool ServoControl::isUpAttached() {
    return servoUp.attached();
}

ServoHealthCode ServoControl::checkDown() {
    if (!downAttached)
        return SERVO_HEALTH_ATTACH_FAILED;
    return checkServo(servoDown, currentDownAngle);
}

ServoHealthCode ServoControl::checkUp() {
    if (!upAttached)
        return SERVO_HEALTH_ATTACH_FAILED;
    return checkServo(servoUp, currentUpAngle);
}

ServoHealthCode ServoControl::checkServo(Servo &servo, int angle) {
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

void ServoControl::moveDown(int angle) {
    angle = constrain(angle, 0, 180);
    servoDown.write(angle);
    currentDownAngle = angle;
}

void ServoControl::moveUp(int angle) {
    angle = constrain(angle, 0, 180);
    servoUp.write(angle);
    currentUpAngle = angle;
}

void ServoControl::moveDownSmooth(int targetAngle, int stepDelay) {
    targetAngle = constrain(targetAngle, 0, 180);
    moveServoSmooth(servoDown, currentDownAngle, targetAngle, stepDelay);
}

void ServoControl::moveUpSmooth(int targetAngle, int stepDelay) {
    targetAngle = constrain(targetAngle, 0, 180);
    moveServoSmooth(servoUp, currentUpAngle, targetAngle, stepDelay);
}

void ServoControl::setInitialPosition() {
    moveDown(INITIAL_DOWN_ANGLE);
    moveUp(INITIAL_UP_ANGLE);
}

void ServoControl::setTestPosition() {
    moveDown(90);
    moveUp(90);
}

int ServoControl::getCurrentDownAngle() {
    return currentDownAngle;
}

int ServoControl::getCurrentUpAngle() {
    return currentUpAngle;
}

void ServoControl::moveServoSmooth(Servo &servo, int &currentAngle, int targetAngle, int stepDelay) {
    if (currentAngle < targetAngle) {
        for (int angle = currentAngle; angle <= targetAngle; angle += 2) {
            servo.write(angle);
            delay(stepDelay);
        }
    } else {
        for (int angle = currentAngle; angle >= targetAngle; angle -= 2) {
            servo.write(angle);
            delay(stepDelay);
        }
    }
    // Ensure we reach the exact target angle
    servo.write(targetAngle);
    currentAngle = targetAngle;
}
